// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Allwinner sun50i Pulse Width Modulation Controller
 *
 * (C) Copyright 2025 Richard Genoud, Bootlin <richard.genoud@bootlin.com>
 *
 * Based on drivers/pwm/pwm-sun4i.c with Copyright:
 *
 * Copyright (C) 2014 Alexandre Belloni <alexandre.belloni@bootlin.com>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>
#include <linux/io.h>

/* PWM IRQ Enable Register */
#define PWM_IER				0x0

/* PWM IRQ Status Register */
#define PWM_ISR				0x4

/* PWM Capture IRQ Enable Register */
#define PWM_CIER			0x10

/* PWM Capture IRQ Status Register */
#define PWM_CISR			0x14

/* PWMCC Pairs Clock Configuration Registers */
#define PWM_XY_CLK_CR(pair)		(0x20 + ((pair) * 0x4))
#define PWM_XY_CLK_CR_SRC_SHIFT		7
#define PWM_XY_CLK_CR_SRC_MASK		1
#define PWM_XY_CLK_CR_GATE_BIT		4
#define PWM_XY_CLK_CR_BYPASS_BIT(chan)	((chan) % 2 + 5)
#define PWM_XY_CLK_CR_DIV_M_SHIFT	0

/* PWMCC Pairs Dead Zone Control Registers */
#define PWM_XY_DZ(pair)			(0x30 + ((pair) * 0x4))

/* PWM Enable Register */
#define PWM_ENR				0x40
#define PWM_ENABLE(x)			BIT(x)

/* PWM Capture Enable Register */
#define PWM_CER				0x44

/* PWM Control Register */
#define PWM_CTRL_REG(chan)		(0x60 + (chan) * 0x20)
#define PWM_CTRL_PRESCAL_K_SHIFT	0
#define PWM_CTRL_PRESCAL_K_WIDTH	8
#define PMW_CTRL_ACTIVE_STATE		BIT(8)

/* PWM Period Register */
#define PWM_PRD_REG(ch)			(0x64 + (ch) * 0x20)
#define PWM_PRD(prd)			(((prd) - 1) << 16)
#define PWM_PRD_MASK			GENMASK(31, 16)
#define PWM_DTY_MASK			GENMASK(15, 0)
#define PWM_REG_PRD(reg)		(FIELD_GET(PWM_PRD_MASK, reg) + 1)
#define PWM_REG_DTY(reg)		FIELD_GET(PWM_DTY_MASK, reg)

/* PWM Count Register */
#define PWM_CNT_REG(x)			(0x68 + (x) * 0x20)

/* PWM Capture Control Register */
#define PWM_CCR(x)			(0x6c + (x) * 0x20)

/* PWM Capture Rise Lock Register */
#define PWM_CRLR(x)			(0x70 + (x) * 0x20)

/* PWM Capture Fall Lock Register */
#define PWM_CFLR(x)			(0x74 + (x) * 0x20)

/* PWM Period bit field */
#define PWM_ENTIRE_CYCLE(x)		(((x) - 1) << 16)
#define PWM_ACTIVE_CYCLE(x)		((x) - 1)

/*
 * Block diagram of the PWM clock controller:
 *
 *                   ________       _____
 *                  |        |     |     |
 * PWM_clock_xy --->| /div_k |---->| Mux |---> PWM_clock_x
 *                  |________|  +->|_____|
 *                              |
 *                      Bypass  |
 *                    +---------+
 *                    |
 *             _____  |              ______      ________
 * OSC24M --->|     | | PWM_clk_src |      |    |        |
 * APB1 ----->| Mux |-+------------>| Gate |--->| /div_m |--> PWM_clock_xy
 *            |_____| |             |______|    |________|
 *                    |
 *                    | Bypass
 *                    +---------+
 *                              |
 *                   ________   |   _____
 *                  |        |  +->|     |
 * PWM_clock_xy --->| /div_k |---->| Mux |---> PWM_clock_y
 *                  |________|     |_____|
 *
 *
 */


/*
 * Table used to generate PMW_clock_XY from PMW_clk_XY_src.
 * It's actually CLK_DIVIDER_POWER_OF_TWO, but limited to /256
 */
static const struct clk_div_table clk_table_xy_div[] = {
	{ .val = 0, .div = 1, },
	{ .val = 1, .div = 2, },
	{ .val = 2, .div = 4, },
	{ .val = 3, .div = 8, },
	{ .val = 4, .div = 16, },
	{ .val = 5, .div = 32, },
	{ .val = 6, .div = 64, },
	{ .val = 7, .div = 128, },
	{ .val = 8, .div = 256, },
	{ .val = 0, .div = 0, }, /* last entry */
};

#define PWM_XY_GATE(_pair, _reg)			\
struct clk_gate gate_xy_##_pair = {			\
	.reg = (void *)_reg,				\
	.bit_idx = PWM_XY_CLK_CR_GATE_BIT,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_gate_ops,			\
	}						\
};

#define PWM_XY_SRC_MUX(_pair, _reg)			\
struct clk_mux mux_xy_##_pair = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CR_SRC_SHIFT,		\
	.mask = PWM_XY_CLK_CR_SRC_MASK,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_mux_ops,			\
	}						\
};

#define PWM_XY_DIV(_pair, _reg)				\
struct clk_divider rate_xy_##_pair = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CR_DIV_M_SHIFT,		\
	.table = clk_table_xy_div,			\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_divider_ops,		\
	}						\
};

#define PWM_X_MUX(_idx, _reg, _chan)			\
struct clk_mux mux_x_##_idx = {				\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CR_BYPASS_BIT(_chan),	\
	.mask = 1,					\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_mux_ops,			\
	}						\
};

#define PWM_X_DIV(_idx, _reg)				\
struct clk_divider rate_x_##_idx = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_CTRL_PRESCAL_K_SHIFT,		\
	.width = PWM_CTRL_PRESCAL_K_WIDTH,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_divider_ops,		\
	}						\
};

#define PWM_XY_CLK_SRC(_pair, _reg)			\
	static PWM_XY_SRC_MUX(_pair, _reg)

#define PWM_XY_CLK(_pair, _reg)				\
	static PWM_XY_GATE(_pair, _reg);		\
	static PWM_XY_DIV(_pair, _reg)

#define PWM_X_CLK(_idx)							\
	static PWM_X_MUX(_idx, PWM_XY_CLK_CR((_idx) >> 1), _idx);	\
	static PWM_X_DIV(_idx, PWM_CTRL_REG(_idx))

#define REF_CLK_XY_SRC(_pair)						\
	{								\
		.name = "pwm-xy-clk-src" #_pair,			\
		.parent_names = (const char *[]){ "osc24M", "apb1" },	\
		.num_parents = 2,					\
		.mux_hw = &mux_xy_##_pair.hw,				\
	}

#define REF_CLK_XY(_pair)						\
	{								\
		.name = "pwm-xy-clk" #_pair,				\
		.parent_names = (const char *[]){ "pwm-xy-clk-src" #_pair }, \
		.num_parents = 1,					\
		.gate_hw = &gate_xy_##_pair.hw,				\
		.rate_hw = &rate_xy_##_pair.hw,				\
		.flags = CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,	\
	}

#define REF_CLK_X(_idx, _pair)						\
	{								\
		.name = "pwm-clk" #_idx,				\
		.parent_names = (const char *[]){			\
			"pwm-xy-clk" #_pair,				\
			"pwm-xy-clk-src" #_pair				\
		},							\
		.num_parents = 2,					\
		.mux_hw = &mux_x_##_idx.hw,				\
		.rate_hw = &rate_x_##_idx.hw,				\
		.flags = CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,	\
	}

/*
 * Clocks obtained after the 1st mux
 *             _____
 * OSC24M --->|     |
 * APB1 ----->| Mux |---> PWM_clk_src
 *            |_____|
 */
PWM_XY_CLK_SRC(01, PWM_XY_CLK_CR(0));
PWM_XY_CLK_SRC(23, PWM_XY_CLK_CR(1));
PWM_XY_CLK_SRC(45, PWM_XY_CLK_CR(2));

/*
 * Clocks obtained after the 1st div
 *              ______      ________
 * PWM_clk_src |      |    |        |
 * ----------->| Gate |--->| /div_m |--> PWM_clock_xy
 *             |______|    |________|
 *
 */
PWM_XY_CLK(01, PWM_XY_CLK_CR(0));
PWM_XY_CLK(23, PWM_XY_CLK_CR(1));
PWM_XY_CLK(45, PWM_XY_CLK_CR(2));

/*
 * Clocks obtained after the 2nd mux
 *                      Bypass
 *                     ---------+
 *                   ________   |   _____
 *                  |        |  +->|     |
 * PWM_clock_xy --->| /div_k |---->| Mux |---> PWM_clock_y
 *                  |________|     |_____|
 */
PWM_X_CLK(0);
PWM_X_CLK(1);
PWM_X_CLK(2);
PWM_X_CLK(3);
PWM_X_CLK(4);
PWM_X_CLK(5);

struct clk_pwm_data {
	const char *name;
	const char * const *parent_names;
	int num_parents;
	struct clk_hw *mux_hw;
	struct clk_hw *rate_hw;
	struct clk_hw *gate_hw;
	unsigned long flags;
};

struct clk_pwm_pdata {
	struct clk_hw_onecell_data *hw_data;
	spinlock_t lock;
	void __iomem *reg;
};

static struct clk_pwm_data pwmcc_data[] = {
	REF_CLK_X(0, 01),
	REF_CLK_X(1, 01),
	REF_CLK_X(2, 23),
	REF_CLK_X(3, 23),
	REF_CLK_X(4, 45),
	REF_CLK_X(5, 45),
	REF_CLK_XY(01),
	REF_CLK_XY(23),
	REF_CLK_XY(45),
	REF_CLK_XY_SRC(01),
	REF_CLK_XY_SRC(23),
	REF_CLK_XY_SRC(45),
	{ /* sentinel */ },
};

struct h616_pwm_data {
	unsigned int npwm;
};

struct h616_pwm_chip {
	struct clk_pwm_pdata *clk_pdata;
	struct clk **pwm_clocks;
	struct clk *bus_clk;
	struct reset_control *rst;
	void __iomem *base;
	const struct h616_pwm_data *data;
	/* Mutex to protect pwm apply state */
	struct mutex mutex;
};

static int h616_add_composite_clk(const struct clk_pwm_data *data,
				  void __iomem *reg, spinlock_t *lock,
				  struct device *dev, struct clk_hw **hw)
{
	const struct clk_ops *mux_ops = NULL, *gate_ops = NULL, *rate_ops = NULL;
	struct clk_hw *mux_hw = NULL, *gate_hw = NULL, *rate_hw = NULL;


	if (data->mux_hw) {
		struct clk_mux *mux;

		mux_hw = data->mux_hw;
		mux = to_clk_mux(mux_hw);
		mux->lock = lock;
		mux_ops = mux_hw->init->ops;
		mux->reg = (u64)mux->reg + reg ;
	}

	if (data->gate_hw) {
		struct clk_gate *gate;

		gate_hw = data->gate_hw;
		gate = to_clk_gate(gate_hw);
		gate->lock = lock;
		gate_ops = gate_hw->init->ops;
		gate->reg = (u64)gate->reg + reg;
	}

	if (data->rate_hw) {
		struct clk_divider *rate;

		rate_hw = data->rate_hw;
		rate = to_clk_divider(rate_hw);
		rate_ops = rate_hw->init->ops;
		rate->lock = lock;
		rate->reg = (u64)rate->reg + reg;

		if (rate->table) {
			const struct clk_div_table *clkt;
			int table_size = 0;

			for (clkt = rate->table; clkt->div; clkt++)
				table_size++;
			rate->width = order_base_2(table_size);
		}
	}
	*hw = clk_hw_register_composite(dev, data->name, data->parent_names,
					data->num_parents, mux_hw,
					mux_ops, rate_hw, rate_ops,
					gate_hw, gate_ops, data->flags);

	return PTR_ERR_OR_ZERO(*hw);
}

static int h616_pwmcc_register(struct platform_device *pdev,
			       struct h616_pwm_chip *pwm)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_pwm_pdata *pdata;
	struct device *dev = &pdev->dev;
	int num_clocks = 0;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	while (pwmcc_data[num_clocks].name)
		num_clocks++;

	pdata->hw_data = devm_kzalloc(dev, struct_size(pdata->hw_data, hws, num_clocks),
				      GFP_KERNEL);
	if (!pdata->hw_data)
		return -ENOMEM;

	pdata->hw_data->num = num_clocks;

	pdata->reg = pwm->base;

	spin_lock_init(&pdata->lock);

	for (int i = 0; i < num_clocks; i++) {
		struct clk_hw **hw = &pdata->hw_data->hws[i];
		if (h616_add_composite_clk(&pwmcc_data[i],
						  pdata->reg,
						  &pdata->lock, dev, hw))
			dev_err(dev, "Can't register pwm clock %s\n",
				pwmcc_data[i].name);
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get,
				     pdata->hw_data);
	if (ret) {
		dev_err(dev, "Error adding clock provider\n");
		for (int i = 0; i < num_clocks; i++)
			clk_hw_unregister(pdata->hw_data->hws[i]);
		return ret;
	}

	pwm->clk_pdata = pdata;

	return num_clocks;
}

static inline struct h616_pwm_chip *to_h616_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 h616_pwm_readl(struct h616_pwm_chip *h616chip,
				 unsigned long offset)
{
	return readl(h616chip->base + offset);
}

static inline void h616_pwm_writel(struct h616_pwm_chip *h616chip,
				   u32 val, unsigned long offset)
{
	writel(val, h616chip->base + offset);
}


static struct clk *h616_pwm_get_clk(struct device *dev, unsigned int hwpwm)
{
	struct clk *pwm_clk = NULL;
	char *clk_name;

	clk_name = kasprintf(GFP_KERNEL, "pwm-clk%d", hwpwm);
	if (!clk_name)
		return ERR_PTR(-ENOMEM);

	pwm_clk = devm_clk_get_prepared(dev, clk_name);

	kfree(clk_name);

	return pwm_clk;
}

static int h616_pwm_get_state(struct pwm_chip *chip,
			      struct pwm_device *pwm,
			      struct pwm_state *state)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct clk **pwm_clocks = h616chip->pwm_clocks;
	u64 clk_rate, tmp;
	u32 val;

	if (IS_ERR_OR_NULL(pwm_clocks[pwm->hwpwm])) {
		pwm_clocks[pwm->hwpwm] = h616_pwm_get_clk(&chip->dev, pwm->hwpwm);
		if (IS_ERR(pwm_clocks[pwm->hwpwm])) {
			printk("ERROR CLK GET %d\n", PTR_ERR(pwm_clocks[pwm->hwpwm]));

			return PTR_ERR(pwm_clocks[pwm->hwpwm]);
		}
	}
	clk_rate = clk_get_rate(h616chip->pwm_clocks[pwm->hwpwm]);
	if (!clk_rate)
		return -EINVAL;

	val = h616_pwm_readl(h616chip, PWM_ENR);
	state->enabled = !!(PWM_ENABLE(pwm->hwpwm) & val);

	val = h616_pwm_readl(h616chip, PWM_XY_CLK_CR(pwm->hwpwm >> 2));
	if (val & PWM_XY_CLK_CR_BYPASS_BIT(pwm->hwpwm)) {
		/*
		 * When bypass is enabled, the PWM logic is inactive.
		 * The pwm_xy_clk_src is directly routed to pwm-clk
		 */
		state->period = DIV_ROUND_UP_ULL(NSEC_PER_SEC, clk_rate);
		state->duty_cycle = DIV_ROUND_UP_ULL(state->period, 2);
		state->polarity = PWM_POLARITY_NORMAL;
		return 0;
	}

	state->enabled &= !!(BIT(PWM_XY_CLK_CR_GATE_BIT) & val);

	val = h616_pwm_readl(h616chip, PWM_CTRL_REG(pwm->hwpwm));
	if (val & PMW_CTRL_ACTIVE_STATE)
		state->polarity = PWM_POLARITY_NORMAL;
	else
		state->polarity = PWM_POLARITY_INVERSED;

	val = h616_pwm_readl(h616chip, PWM_PRD_REG(pwm->hwpwm));

	tmp = NSEC_PER_SEC * PWM_REG_DTY(val);
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

	tmp = NSEC_PER_SEC * PWM_REG_PRD(val);
	state->period = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);
printk("%s duty=%llu period=%llu rate=%llu\n", __func__, state->duty_cycle,
       state->period, clk_rate);
	return 0;
}

static int h616_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			  const struct pwm_state *state)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct clk **pwm_clocks = h616chip->pwm_clocks;
	u32 ctrl, duty = 0, period = 0, val;
	struct pwm_state cstate;
	unsigned int delay_us;
	u64 rate;
	bool bypass;
	int ret;

	if (IS_ERR_OR_NULL(pwm_clocks[pwm->hwpwm])) {
		pwm_clocks[pwm->hwpwm] = h616_pwm_get_clk(&chip->dev, pwm->hwpwm);
		if (IS_ERR(pwm_clocks[pwm->hwpwm])) {
			printk("ERROR CLK GET %d\n", PTR_ERR(pwm_clocks[pwm->hwpwm]));

			return PTR_ERR(pwm_clocks[pwm->hwpwm]);
		}
	}
	pwm_get_state(pwm, &cstate);

	rate = DIV_ROUND_UP_ULL(NSEC_PER_SEC, state->period);

	ret = clk_set_rate(h616chip->pwm_clocks[pwm->hwpwm], rate);
	if (ret) {
		dev_err(pwmchip_parent(chip), "failed to set PWM clock rate\n");
		return ret;
	}

	val = h616_pwm_readl(h616chip, PWM_XY_CLK_CR(pwm->hwpwm >> 2));
	bypass = !! (val & PWM_XY_CLK_CR_BYPASS_BIT(pwm->hwpwm));

	/*
	 * If bypass is set, the PWM logic (polarity, duty) can't be applied
	 */

	if (bypass && (state->polarity == PWM_POLARITY_INVERSED)) {
		dev_warn(pwmchip_parent(chip),
			 "Can't set inversed polarity with bypass enabled\n");
	} else {
		val = h616_pwm_readl(h616chip, PWM_CTRL_REG(pwm->hwpwm));
		val &= ~PMW_CTRL_ACTIVE_STATE;
		if (state->polarity == PWM_POLARITY_NORMAL)
			val |= PMW_CTRL_ACTIVE_STATE;
		h616_pwm_writel(h616chip, val, PWM_CTRL_REG(pwm->hwpwm));
	}

	if (bypass && (state->duty_cycle * 2 != state->period)) {
		dev_warn(pwmchip_parent(chip),
			 "Can't set a duty cycle with bypass enabled\n");
	} else {
		val = h616_pwm_readl(h616chip, PWM_PRD_REG(pwm->hwpwm));
printk("reg dty= 0x%llx\n", val);
#if 0
		val = (duty & PWM_DTY_MASK) | PWM_PRD(period);
	h616_pwm_writel(h616chip, val, PWM_CH_PRD(pwm->hwpwm));

	duty = state->duty * rate
duty: state->duty * fq clock / Nsec_per_sec
period: fq clock * state->period / Nsec_per_sec
	tmp = NSEC_PER_SEC * PWM_REG_DTY(val);
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

	tmp = NSEC_PER_SEC * PWM_REG_PRD(val);
	state->period = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

#endif
	}

printk("%s duty=%llu period=%llu rate=%llu\n", __func__, state->duty_cycle,
       state->period, rate);
printk("get rate=%llu\n", clk_get_rate(h616chip->pwm_clocks[pwm->hwpwm]));


	if (state->enabled && !cstate.enabled) {
		clk_prepare_enable(h616chip->pwm_clocks[pwm->hwpwm]);
printk("get rate=%llu\n", clk_get_rate(h616chip->pwm_clocks[pwm->hwpwm]));
		if (ret) {
			dev_err(pwmchip_parent(chip), "failed to enable PWM clock\n");
			return ret;
		}
	}

	if (!state->enabled && cstate.enabled) {
		clk_disable_unprepare(h616chip->pwm_clocks[pwm->hwpwm]);
		return 0;
	}

	return 0;
}

static const struct pwm_ops h616_pwm_ops = {
	.apply = h616_pwm_apply,
	.get_state = h616_pwm_get_state,
};

static int h616_pwm_probe(struct platform_device *pdev)
{
	const struct h616_pwm_data *data;
	struct device *dev = &pdev->dev;
	struct h616_pwm_chip *pwm;
	struct pwm_chip *chip;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	chip = devm_pwmchip_alloc(dev, data->npwm, sizeof(*pwm));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	pwm = to_h616_pwm_chip(chip);
	pwm->data = data;
	pwm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pwm->base))
		return PTR_ERR(pwm->base);

	pwm->bus_clk = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(pwm->bus_clk))
		return dev_err_probe(dev, PTR_ERR(pwm->bus_clk),
				     "Failed to get bus clock\n");

	ret = h616_pwmcc_register(pdev, pwm);
	printk(" h616_pwmcc_register return %d\n", ret);


	pwm->pwm_clocks = devm_kmalloc_array(dev, data->npwm,
					     sizeof(*(pwm->pwm_clocks)),
					     GFP_KERNEL);

#if 0
	for (int i = 1; i < data->npwm; i++) {
		char *clk_name;

		clk_name = devm_kasprintf(dev, GFP_KERNEL, "pwm-clk%d", i);
		if (!clk_name)
			return -ENOMEM;

		pwm->pwm_clocks[i] = devm_clk_get(dev, clk_name);
		if (IS_ERR(pwm->pwm_clocks[i])) {
			return dev_err_probe(dev, PTR_ERR(pwm->pwm_clocks[i]),
					     "Failed to get %s clock\n", clk_name);
		}
		devm_kfree(dev, clk_name);
	}
#endif
	pwm->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(pwm->rst))
		return dev_err_probe(dev, PTR_ERR(pwm->rst),
				     "get reset failed\n");

	/* Deassert reset */
	ret = reset_control_deassert(pwm->rst);
	if (ret) {
		dev_err(dev, "cannot deassert reset control: %pe\n",
			ERR_PTR(ret));
		return ret;
	}

	chip->ops = &h616_pwm_ops;

	ret = pwmchip_add(chip);
	if (ret < 0) {
		dev_err(dev, "failed to add PWM chip: %d\n", ret);
		goto err_pwm_add;
	}

	platform_set_drvdata(pdev, chip);

	return 0;

err_pwm_add:
	clk_disable_unprepare(pwm->bus_clk);
	reset_control_assert(pwm->rst);

	return ret;
}

static const struct h616_pwm_data sun50i_h616_pwm_data = {
	.npwm = 6,
};

static const struct of_device_id h616_pwm_dt_ids[] = {
	{
		.compatible = "allwinner,sun50i-h616-pwm",
		.data = &sun50i_h616_pwm_data,
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, h616_pwm_dt_ids);


static struct platform_driver h616_pwm_driver = {
	.driver = {
		.name = "h616-pwm",
		.of_match_table = h616_pwm_dt_ids,
	},
	.probe = h616_pwm_probe,
};
module_platform_driver(h616_pwm_driver);

MODULE_ALIAS("platform:h616-pwm");
MODULE_AUTHOR("Richard Genoud <richard.genoud@bootlin.com>");
MODULE_DESCRIPTION("Allwinner H616 PWM driver");
MODULE_LICENSE("GPL");
