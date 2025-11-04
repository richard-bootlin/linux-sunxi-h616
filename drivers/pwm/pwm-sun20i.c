// SPDX-License-Identifier: GPL-2.0
/*
 * PWM Controller Driver for sunxi platforms (D1, T113-S3 and R329)
 *
 * Limitations:
 * - When the parameters change, the current running period is not completed
 *   and new settings are applied immediately.
 * - The PWM output goes to a HIGH-Z state when the channel is disabled.
 * - Changing the clock configuration (PWM_CLK_CFG)
 *   may cause a brief output glitch.
 *
 * Copyright (c) 2023 Aleksandr Shubin <privatesub2@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define PWM_CLK_CFG(pair)		(0x20 + ((pair) * 0x4))
#define PWM_CLK_CFG_SRC			GENMASK(8, 7)
#define PWM_CLK_CFG_DIV_M		GENMASK(3, 0)
#define PWM_CLK_DIV_M_MAX		8

#define PWM_CLK_GATE			0x40
#define PWM_CLK_GATE_BYPASS(chan)	BIT((chan) + 16)
#define PWM_CLK_GATE_GATING(chan)	BIT(chan)

#define PWM_ENABLE(c)			(c->data->enable_reg)
#define PWM_SUN20I_ENABLE		0x80

#define PWM_ENABLE_EN(chan)		BIT(chan)

#define PWM_SUN20I_CTL			0x100
#define PWM_CTL(c, chan)		(c->data->ctl_reg + (chan) * 0x20)
#define PWM_CTL_ACT_STA			BIT(8)
#define PWM_CTL_PRESCAL_K		GENMASK(7, 0)
#define PWM_CTL_PRESCAL_K_MAX		field_max(PWM_CTL_PRESCAL_K)

#define PWM_PERIOD(c, chan)		(c->data->ctl_reg + 4 + (chan) * 0x20)
#define PWM_PERIOD_ENTIRE_CYCLE		GENMASK(31, 16)
#define PWM_PERIOD_ACT_CYCLE		GENMASK(15, 0)

#define PWM_PCNTR_SIZE			BIT(16)

/*
 * PWM_MAGIC is used to quickly compute the values of the clock dividers
 * div_m (PWM_CLK_CFG_DIV_M) & prescale_k (PWM_CTL_PRESCAL_K)
 * without using a loop. These dividers limit the # of cycles in a period
 * to PWM_PCNTR_SIZE (65536) by applying a scaling factor of
 * 1/(div_m * (prescale_k + 1)) to the clock source.
 *
 * PWM_MAGIC is derived by solving for div_m and prescale_k
 * such that for a given requested period,
 *
 * i) div_m is minimized for any prescale_k ≤ PWM_CTL_PRESCAL_K_MAX,
 * ii) prescale_k is minimized.
 *
 * The derivation proceeds as follows, with val = # of cycles for requested
 * period:
 *
 * for a given value of div_m we want the smallest prescale_k such that
 *
 * (val >> div_m) // (prescale_k + 1) ≤ 65536 (= PWM_PCNTR_SIZE)
 *
 * This is equivalent to:
 *
 * (val >> div_m) ≤ 65536 * (prescale_k + 1) + prescale_k
 * ⟺ (val >> div_m) ≤ 65537 * prescale_k + 65536
 * ⟺ (val >> div_m) - 65536 ≤ 65537 * prescale_k
 * ⟺ ((val >> div_m) - 65536) / 65537 ≤ prescale_k
 *
 * As prescale_k is integer, this becomes
 *
 * ((val >> div_m) - 65536) // 65537 ≤ prescale_k
 *
 * And is minimized at
 *
 * ((val >> div_m) - 65536) // 65537
 *
 * Now we pick the smallest div_m that satifies prescale_k ≤ 255
 * (i.e PWM_CTL_PRESCAL_K_MAX),
 *
 * ((val >> div_m) - 65536) // 65537 ≤ 255
 * ⟺ (val >> div_m) - 65536 ≤ 255 * 65537 + 65536
 * ⟺ val >> div_m ≤ 255 * 65537 + 2 * 65536
 * ⟺ val >> div_m < (255 * 65537 + 2 * 65536 + 1)
 * ⟺ div_m = fls((val) / (255 * 65537 + 2 * 65536 + 1))
 *
 * Suggested by Uwe Kleine-König
 */
#define PWM_MAGIC			(255 * 65537 + 2 * 65536 + 1)
#define PWM_DIV_CONST			65537

struct sun20i_pwm_data {
	unsigned int enable_reg;
	unsigned int ctl_reg;
};

struct sun20i_pwm_chip {
	struct clk *clk_hosc, *clk_apb;
	void __iomem *base;
	const struct sun20i_pwm_data *data;
};

static inline struct sun20i_pwm_chip *to_sun20i_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 sun20i_pwm_readl(struct sun20i_pwm_chip *chip,
				   unsigned long offset)
{
	return readl(chip->base + offset);
}

static inline void sun20i_pwm_writel(struct sun20i_pwm_chip *chip,
				     u32 val, unsigned long offset)
{
	writel(val, chip->base + offset);
}

static int sun20i_pwm_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u16 ent_cycle, act_cycle, prescale_k;
	u64 clk_rate, tmp;
	u8 div_m;
	u32 val;

	val = sun20i_pwm_readl(sun20i_chip, PWM_CLK_CFG(pwm->hwpwm / 2));
	div_m = FIELD_GET(PWM_CLK_CFG_DIV_M, val);
	if (div_m > PWM_CLK_DIV_M_MAX)
		div_m = PWM_CLK_DIV_M_MAX;

	/*
	 * If CLK_CFG_SRC is 0, use the hosc clock;
	 * otherwise (any nonzero value) use the APB clock.
	 */
	if (FIELD_GET(PWM_CLK_CFG_SRC, val) == 0)
		clk_rate = clk_get_rate(sun20i_chip->clk_hosc);
	else
		clk_rate = clk_get_rate(sun20i_chip->clk_apb);

	val = sun20i_pwm_readl(sun20i_chip, PWM_CTL(sun20i_chip, pwm->hwpwm));
	state->polarity = (PWM_CTL_ACT_STA & val) ?
			   PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;

	prescale_k = FIELD_GET(PWM_CTL_PRESCAL_K, val) + 1;

	val = sun20i_pwm_readl(sun20i_chip, PWM_ENABLE(sun20i_chip));
	state->enabled = (PWM_ENABLE_EN(pwm->hwpwm) & val) ? true : false;

	val = sun20i_pwm_readl(sun20i_chip, PWM_PERIOD(sun20i_chip, pwm->hwpwm));
	act_cycle = FIELD_GET(PWM_PERIOD_ACT_CYCLE, val);

	ent_cycle = FIELD_GET(PWM_PERIOD_ENTIRE_CYCLE, val);

	/*
	 * The duration of the active phase should not be longer
	 * than the duration of the period
	 */
	if (act_cycle > ent_cycle)
		act_cycle = ent_cycle;

	/*
	 * We have act_cycle <= ent_cycle <= 0xffff, prescale_k <= 0x100,
	 * div_m <= 8. So the multiplication fits into an u64 without
	 * overflow.
	 */
	tmp = ((u64)(act_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_UP_ULL(tmp, clk_rate);
	tmp = ((u64)(ent_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->period = DIV_ROUND_UP_ULL(tmp, clk_rate);

	return 0;
}

static int sun20i_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u64 bus_rate, hosc_rate, val, ent_cycle, act_cycle;
	u32 clk_gate, clk_cfg, pwm_en, ctl, reg_period;
	u32 prescale_k, div_m;
	bool use_bus_clk;

	pwm_en = sun20i_pwm_readl(sun20i_chip, PWM_ENABLE(sun20i_chip));
	clk_gate = sun20i_pwm_readl(sun20i_chip, PWM_CLK_GATE);

	if (!state->enabled) {
		if (state->enabled != pwm->state.enabled) {
			clk_gate &= ~PWM_CLK_GATE_GATING(pwm->hwpwm);
			pwm_en &= ~PWM_ENABLE_EN(pwm->hwpwm);
			sun20i_pwm_writel(sun20i_chip, pwm_en, PWM_ENABLE(sun20i_chip));
			sun20i_pwm_writel(sun20i_chip, clk_gate, PWM_CLK_GATE);
		}
		return 0;
	}

	ctl = sun20i_pwm_readl(sun20i_chip, PWM_CTL(sun20i_chip, pwm->hwpwm));
	clk_cfg = sun20i_pwm_readl(sun20i_chip, PWM_CLK_CFG(pwm->hwpwm / 2));
	hosc_rate = clk_get_rate(sun20i_chip->clk_hosc);
	bus_rate = clk_get_rate(sun20i_chip->clk_apb);
	if (pwm_en & PWM_ENABLE_EN(pwm->hwpwm ^ 1)) {
		/* If the neighbor channel is enabled, use the current clock settings */
		use_bus_clk = FIELD_GET(PWM_CLK_CFG_SRC, clk_cfg) != 0;
		val = mul_u64_u64_div_u64(state->period,
					  (use_bus_clk ? bus_rate : hosc_rate),
					  NSEC_PER_SEC);

		div_m = FIELD_GET(PWM_CLK_CFG_DIV_M, clk_cfg);
	} else {
		/*
		 * Select the clock source based on the period.
		 * Since bus_rate > hosc_rate, which means bus_rate
		 * can provide a higher frequency than hosc_rate.
		 */
		use_bus_clk = false;
		val = mul_u64_u64_div_u64(state->period, hosc_rate, NSEC_PER_SEC);
		/*
		 * If the calculated value is ≤ 1, the period is too short
		 * for proper PWM operation
		 */
		if (val <= 1) {
			use_bus_clk = true;
			val = mul_u64_u64_div_u64(state->period, bus_rate, NSEC_PER_SEC);
			if (val <= 1)
				return -EINVAL;
		}
		div_m = fls(DIV_ROUND_DOWN_ULL(val, PWM_MAGIC));
		if (div_m > PWM_CLK_DIV_M_MAX)
			return -EINVAL;

		/* Set up the CLK_DIV_M and clock CLK_SRC */
		clk_cfg = FIELD_PREP(PWM_CLK_CFG_DIV_M, div_m);
		clk_cfg |= FIELD_PREP(PWM_CLK_CFG_SRC, use_bus_clk);

		sun20i_pwm_writel(sun20i_chip, clk_cfg, PWM_CLK_CFG(pwm->hwpwm / 2));
	}

	/* Calculate prescale_k and determine the number of cycles for a full PWM period */
	ent_cycle = val >> div_m;
	prescale_k = DIV_ROUND_DOWN_ULL(ent_cycle, PWM_DIV_CONST);
	if (prescale_k > PWM_CTL_PRESCAL_K_MAX)
		prescale_k = PWM_CTL_PRESCAL_K_MAX;

	do_div(ent_cycle, prescale_k + 1);

	/* ent_cycle must not be zero */
	if (ent_cycle == 0)
		return -EINVAL;

	/* For N cycles, PPRx.PWM_ENTIRE_CYCLE = (N-1) */
	reg_period = FIELD_PREP(PWM_PERIOD_ENTIRE_CYCLE, ent_cycle - 1);

	/* Calculate the active cycles (duty cycle) */
	val = mul_u64_u64_div_u64(state->duty_cycle,
				  (use_bus_clk ? bus_rate : hosc_rate),
				  NSEC_PER_SEC);
	act_cycle = val >> div_m;
	do_div(act_cycle, prescale_k + 1);

	/*
	 * The formula of the output period and the duty-cycle for PWM are as follows.
	 * T period = PWM0_PRESCALE_K / PWM01_CLK * (PPR0.PWM_ENTIRE_CYCLE + 1)
	 * T high-level = PWM0_PRESCALE_K / PWM01_CLK * PPR0.PWM_ACT_CYCLE
	 * Duty-cycle = T high-level / T period
	 */
	reg_period |= FIELD_PREP(PWM_PERIOD_ACT_CYCLE, act_cycle);
	sun20i_pwm_writel(sun20i_chip, reg_period,
			  PWM_PERIOD(sun20i_chip, pwm->hwpwm));

	ctl = FIELD_PREP(PWM_CTL_PRESCAL_K, prescale_k);
	if (state->polarity == PWM_POLARITY_NORMAL)
		ctl |= PWM_CTL_ACT_STA;

	sun20i_pwm_writel(sun20i_chip, ctl, PWM_CTL(sun20i_chip, pwm->hwpwm));

	if (state->enabled != pwm->state.enabled) {
		clk_gate &= ~PWM_CLK_GATE_BYPASS(pwm->hwpwm);
		clk_gate |= PWM_CLK_GATE_GATING(pwm->hwpwm);
		pwm_en |= PWM_ENABLE_EN(pwm->hwpwm);
		sun20i_pwm_writel(sun20i_chip, pwm_en, PWM_ENABLE(sun20i_chip));
		sun20i_pwm_writel(sun20i_chip, clk_gate, PWM_CLK_GATE);
	}

	return 0;
}

static const struct pwm_ops sun20i_pwm_ops = {
	.apply = sun20i_pwm_apply,
	.get_state = sun20i_pwm_get_state,
};

static const struct sun20i_pwm_data sun20i_d1_pwm_data = {
	.enable_reg = PWM_SUN20I_ENABLE,
	.ctl_reg = PWM_SUN20I_CTL,
};

static const struct of_device_id sun20i_pwm_dt_ids[] = {
	{
		.compatible = "allwinner,sun20i-d1-pwm",
		.data = &sun20i_d1_pwm_data,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, sun20i_pwm_dt_ids);

static int sun20i_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct sun20i_pwm_chip *sun20i_chip;
	const struct sun20i_pwm_data *data;
	struct reset_control *rst;
	u32 npwm;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "allwinner,npwms", &npwm);
	if (ret < 0)
		npwm = 8; /* Default value */

	if (npwm > 16) {
		dev_info(&pdev->dev, "PWM lines number out of range (%u > 16)", npwm);
		return -EINVAL;
	}

	chip = devm_pwmchip_alloc(&pdev->dev, npwm, sizeof(*sun20i_chip));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	sun20i_chip = to_sun20i_pwm_chip(chip);

	data = of_device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	sun20i_chip->data = data;

	sun20i_chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sun20i_chip->base))
		return PTR_ERR(sun20i_chip->base);

	rst = devm_reset_control_get_exclusive_deasserted(&pdev->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(rst),
				     "Failed to get reset control\n");

	chip->ops = &sun20i_pwm_ops;

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to add PWM chip\n");

	return 0;
}

static struct platform_driver sun20i_pwm_driver = {
	.driver = {
		.name = "sun20i-pwm",
		.of_match_table = sun20i_pwm_dt_ids,
	},
	.probe = sun20i_pwm_probe,
};
module_platform_driver(sun20i_pwm_driver);

MODULE_AUTHOR("Aleksandr Shubin <privatesub2@gmail.com>");
MODULE_DESCRIPTION("Allwinner sun20i PWM driver");
MODULE_LICENSE("GPL");
