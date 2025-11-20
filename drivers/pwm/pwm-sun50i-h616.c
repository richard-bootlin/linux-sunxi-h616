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

#ifndef UINT32_MAX
#define UINT32_MAX 0xffffffffU
#endif

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
 *             _____      ______      ________
 * OSC24M --->|     |    |      |    |        |
 * APB1 ----->| Mux |--->| Gate |--->| /div_m |-----> PWM_clock_src_xy
 *            |_____|    |______|    |________|
 *                       ________                      _____
 *                      |        |   PWM_clock_x_div  |     |
 * PWM_clock_src_xy -+->| /div_k |------------------->| Mux |---> PWM_clock_x
 *                   |  |________|  +---------------->|_____|
 *                   |              |
 *                   |    Bypass    |
 *                   +--------------+
 *
 *                       ________                      _____
 *                      |        |   PWM_clock_y_div  |     |
 * PWM_clock_src_xy -+->| /div_k |------------------->| Mux |---> PWM_clock_y
 *                   |  |________|  +---------------->|_____|
 *                   |              |
 *                   |    Bypass    |
 *                   +--------------+
 *
 */


/*
 * Table used for /div_m (diviser before obtaining PMW_clk_XY_src)
 * It's actually CLK_DIVIDER_POWER_OF_TWO, but limited to /256
 */
static const struct clk_div_table clk_table_div_m[] = {
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

#define PWM_XY_SRC_GATE(_pair, _reg)			\
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
	.mask = PWM_XY_CLK_CR_SRC_MASK,			\
	.flags = CLK_MUX_ROUND_CLOSEST,			\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_mux_ops,			\
	}						\
};

#define PWM_XY_SRC_DIV(_pair, _reg)			\
struct clk_divider rate_xy_##_pair = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CR_DIV_M_SHIFT,		\
	.table = clk_table_div_m,			\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_divider_ops,		\
	}						\
};

#define PWM_X_MUX(_idx, _reg, _chan)			\
struct clk_mux mux_x_##_idx = {				\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CR_BYPASS_BIT(_chan),	\
	.mask = 1,					\
	.flags = CLK_MUX_ROUND_CLOSEST,			\
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
	static PWM_XY_SRC_MUX(_pair, _reg)		\
	static PWM_XY_SRC_GATE(_pair, _reg);		\
	static PWM_XY_SRC_DIV(_pair, _reg)

#define PWM_X_CLK_DIV(_idx)				\
	static PWM_X_DIV(_idx, PWM_CTRL_REG(_idx))

#define PWM_X_CLK(_idx)							\
	static PWM_X_MUX(_idx, PWM_XY_CLK_CR((_idx) >> 1), _idx);

#define REF_CLK_XY_SRC(_pair)						\
	{								\
		.name = "pwm-xy-clk-src" #_pair,			\
		.parent_names = (const char *[]){ "osc24M", "apb1" },	\
		.num_parents = 2,					\
		.mux_hw = &mux_xy_##_pair.hw,				\
		.gate_hw = &gate_xy_##_pair.hw,				\
		.rate_hw = &rate_xy_##_pair.hw,				\
	}

#define REF_CLK_X_DIV(_idx, _pair)					\
	{								\
		.name = "pwm-xy-clk-div" #_idx,			\
		.parent_names = (const char *[]){ "pwm-xy-clk-src" #_pair }, \
		.num_parents = 1,					\
		.rate_hw = &rate_x_##_idx.hw,				\
		.flags = CLK_SET_RATE_PARENT,	\
	}

#define REF_CLK_X(_idx, _pair)						\
	{								\
		.name = "pwm-clk" #_idx,				\
		.parent_names = (const char *[]){			\
			"pwm-xy-clk-div" #_idx,				\
			"pwm-xy-clk-src" #_pair				\
		},							\
		.num_parents = 2,					\
		.mux_hw = &mux_x_##_idx.hw,				\
		.flags = CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,	\
	}

/*
 * PWM_clock_src_xy generation:
 *             _____      ______      ________
 * OSC24M --->|     |    |      |    |        |
 * APB1 ----->| Mux |--->| Gate |--->| /div_m |-----> PWM_clock_src_xy
 *            |_____|    |______|    |________|
 */
PWM_XY_CLK_SRC(01, PWM_XY_CLK_CR(0));
PWM_XY_CLK_SRC(23, PWM_XY_CLK_CR(1));
PWM_XY_CLK_SRC(45, PWM_XY_CLK_CR(2));

/*
 * PWM_clock_x_div generation:
 *                       ________
 *                      |        | PWM_clock_x/y_div
 * PWM_clock_src_xy --->| /div_k |------------------->
 *                      |________|
 */
PWM_X_CLK_DIV(0);
PWM_X_CLK_DIV(1);
PWM_X_CLK_DIV(2);
PWM_X_CLK_DIV(3);
PWM_X_CLK_DIV(4);
PWM_X_CLK_DIV(5);

/*
 * PWM_clock_x/y generation:
 *                    Bypass
 * PWM_clock_src_xy ---------+
 *                           |   _____
 *                           +->|     |
 * PWM_clock_x/y_div ---------->| Mux |---> PWM_clock_x/y
 *                              |_____|
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

#define CLK_X_DIV_IDX(h616chip, ch) ((h616chip)->data->npwm + (ch >> 1))
#define CLK_XY_SRC_IDX(h616chip, ch) ((h616chip)->data->npwm * 2 + (ch >> 1))
static struct clk_pwm_data pwmcc_data[] = {
	REF_CLK_X(0, 01),
	REF_CLK_X(1, 01),
	REF_CLK_X(2, 23),
	REF_CLK_X(3, 23),
	REF_CLK_X(4, 45),
	REF_CLK_X(5, 45),
	REF_CLK_X_DIV(0, 01),
	REF_CLK_X_DIV(1, 01),
	REF_CLK_X_DIV(2, 23),
	REF_CLK_X_DIV(3, 23),
	REF_CLK_X_DIV(4, 45),
	REF_CLK_X_DIV(5, 45),
	REF_CLK_XY_SRC(01),
	REF_CLK_XY_SRC(23),
	REF_CLK_XY_SRC(45),
	{ /* sentinel */ },
};

struct h616_pwm_data {
	unsigned int npwm;
};

struct h616_pwm_channel {
	struct clk *pwm_clk;
	unsigned long rate;
	unsigned int entire_cycles;
	unsigned int active_cycles;
	bool bypass;
};

struct h616_pwm_chip {
	struct clk_pwm_pdata *clk_pdata;
	struct h616_pwm_channel *channels;
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

static int h616_pwm_init_clocks(struct platform_device *pdev,
				struct h616_pwm_chip *pwm)
{
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

		ret = h616_add_composite_clk(&pwmcc_data[i], pdata->reg,
					     &pdata->lock, dev, hw);
		if (ret) {
			dev_err_probe(dev, ret,
				      "Failed to register hw clock %s\n",
				      pwmcc_data[i].name);
			for (i--; i >= 0; i--)
				clk_hw_unregister(pdata->hw_data->hws[i]);
			return ret;
		}
	}

	pwm->clk_pdata = pdata;

	return 0;
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

static int h616_pwm_set_bypass(struct pwm_chip *chip, unsigned int idx,
			       bool en_bypass)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[idx];
	struct clk_hw_onecell_data *hw_data = h616chip->clk_pdata->hw_data;
	struct clk *current_parent_clk;
	struct clk *bypass_clk, *div_clk;

	bypass_clk = hw_data->hws[CLK_XY_SRC_IDX(h616chip, idx)]->clk;
	div_clk = hw_data->hws[CLK_X_DIV_IDX(h616chip, idx)]->clk;

	current_parent_clk = clk_get_parent(chan->pwm_clk);

	if (en_bypass && (bypass_clk != current_parent_clk))
		return clk_set_parent(chan->pwm_clk, bypass_clk);

	if (!en_bypass && (bypass_clk == current_parent_clk))
		return clk_set_parent(chan->pwm_clk, div_clk);

	return 0;
}

static int h616_pwm_force_bypass(struct pwm_chip *chip, unsigned int idx)
{
	return h616_pwm_set_bypass(chip, idx, true);
}

static int h616_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[pwm->hwpwm];
	struct device *dev = pwmchip_parent(chip);
	int err;

	err = clk_prepare_enable(chan->pwm_clk);
	if (err < 0) {
		dev_err(dev, "failed to enable clock %s: %d\n",
			__clk_get_name(chan->pwm_clk), err);
	}
	printk("prepare enable %s\n", __clk_get_name(chan->pwm_clk));

	return err;
}

static void h616_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[pwm->hwpwm];

	clk_disable_unprepare(chan->pwm_clk);
	printk("unprepare disable %s\n", __clk_get_name(chan->pwm_clk));
}

static int h616_pwm_get_state(struct pwm_chip *chip,
			      struct pwm_device *pwm,
			      struct pwm_state *state)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[pwm->hwpwm];
	u64 clk_rate, tmp;
	u32 val;

	clk_rate = clk_get_rate(chan->pwm_clk);
	if (!clk_rate)
		return -EINVAL;

	val = h616_pwm_readl(h616chip, PWM_ENR);
	state->enabled = !!(PWM_ENABLE(pwm->hwpwm) & val);

	val = h616_pwm_readl(h616chip, PWM_XY_CLK_CR(pwm->hwpwm >> 2));
	if (val & BIT(PWM_XY_CLK_CR_BYPASS_BIT(pwm->hwpwm))) {
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

static int h616_pwm_calc(struct pwm_chip *chip, unsigned int idx,
			 const struct pwm_state *state)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[idx];
	unsigned int cnt, duty_cnt;
	long fin_freq;
	u64 duty, period, freq, fin_period;

	unsigned long max_rate;
	duty = state->duty_cycle;
	period = state->period;

	max_rate = clk_round_rate(chan->pwm_clk, UINT32_MAX);

	if ((period * max_rate >= NSEC_PER_SEC) &&
	    (period * max_rate < 2* NSEC_PER_SEC) &&
	    (duty * max_rate * 2 >= NSEC_PER_SEC)) {
		/*
		 * If the requested period is to small to be generated by the
		 * PWM, we can just select the highest clock and bypass the
		 * PWM logic
		 */
		dev_dbg(pwmchip_parent(chip), "Setting bypass (period=%lld)\n",
			period);
		freq = div64_u64(NSEC_PER_SEC, period);
		chan->bypass = true;
		duty = period / 2;
	} else if (period == 41) {
		/* TODO: find something */
		freq = 24000000;
		chan->bypass = true;
		duty = period / 2;

	} else {
		chan->bypass = false;
		freq = div64_u64(NSEC_PER_SEC * 0xffffULL, period);
		if (freq > UINT32_MAX)
			freq = UINT32_MAX;
	}

	fin_freq = clk_round_rate(chan->pwm_clk, freq);
	printk("clk_round_rate from %llu = %ld\n", freq, fin_freq);
	if (fin_freq <= 0) {
		dev_err(pwmchip_parent(chip),
			"invalid source clock frequency %llu\n", freq);
		return fin_freq ? fin_freq : -EINVAL;
	}

	fin_period = div64_u64(NSEC_PER_SEC, fin_freq);

	dev_dbg(pwmchip_parent(chip), "fin_freq: %ld Hz\n", fin_freq);

	cnt = mul_u64_u64_div_u64(fin_freq, period, NSEC_PER_SEC);
	if (cnt > 0xffff) {
		dev_err(pwmchip_parent(chip), "unable to get period cnt\n");
		return -EINVAL;
	}

	dev_dbg(pwmchip_parent(chip), "period=%llu cnt=%u duty=%llu\n",
		period, cnt, duty);

	duty_cnt = mul_u64_u64_div_u64(fin_freq, duty, NSEC_PER_SEC);

	dev_dbg(pwmchip_parent(chip), "duty=%llu duty_cnt=%u\n", duty, duty_cnt);

	if (duty_cnt >= cnt)
		duty_cnt = cnt - 1;

	chan->active_cycles = duty_cnt;
	chan->entire_cycles = cnt;

	chan->rate = fin_freq;

	return 0;
}

static int h616_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			  const struct pwm_state *state)
{
	struct h616_pwm_chip *h616chip = to_h616_pwm_chip(chip);
	struct h616_pwm_channel *chan = &h616chip->channels[pwm->hwpwm];
	u32 duty = 0, period = 0, val;
	struct pwm_state cstate;
	unsigned int delay_us;
	u64 rate;
	bool bypass;
	int ret;

	ret = h616_pwm_calc(chip, pwm->hwpwm, state);
	if (ret) {
		dev_err(pwmchip_parent(chip), "period exceeds the maximum value\n");
		return ret;
	}

	pwm_get_state(pwm, &cstate);

	printk("trying to set rate=%llu(period=%llu)\n", chan->rate, state->period);
	ret = clk_set_rate(chan->pwm_clk, chan->rate);
	if (ret) {
		dev_err(pwmchip_parent(chip), "failed to set PWM %d clock rate to %lu\n",
			pwm->hwpwm, chan->rate);
		return ret;
	}

	if (chan->bypass)
		h616_pwm_set_bypass(chip, pwm->hwpwm, chan->bypass);

	/*
	 * If bypass is set, the PWM logic (polarity, duty) can't be applied
	 */

	if (chan->bypass && (state->polarity == PWM_POLARITY_INVERSED)) {
		dev_warn(pwmchip_parent(chip),
			 "Can't set inversed polarity with bypass enabled\n");
	} else {
		val = h616_pwm_readl(h616chip, PWM_CTRL_REG(pwm->hwpwm));
		val &= ~PMW_CTRL_ACTIVE_STATE;
		if (state->polarity == PWM_POLARITY_NORMAL)
			val |= PMW_CTRL_ACTIVE_STATE;
		h616_pwm_writel(h616chip, val, PWM_CTRL_REG(pwm->hwpwm));
	}

	if (chan->bypass && (state->duty_cycle * 2 != state->period)) {
		dev_warn(pwmchip_parent(chip),
			 "Can't set a duty cycle with bypass enabled\n");
	}
	if (!chan->bypass) {
		if (chan->entire_cycles == 0) {
			dev_warn(pwmchip_parent(chip), "entire_cycles==0 !!!\n");
			chan->entire_cycles++;
		}
		val = FIELD_PREP(PWM_DTY_MASK, chan->active_cycles);
		val |= FIELD_PREP(PWM_PRD_MASK, chan->entire_cycles - 1);
		printk("write=0x%x ch=%d\n", val, pwm->hwpwm);
		h616_pwm_writel(h616chip, val, PWM_PRD_REG(pwm->hwpwm));
	}

printk("%s duty=%llu period=%llu rate=%llu\n", __func__, state->duty_cycle,
       state->period, rate);
printk("get rate=%lu\n", clk_get_rate(chan->pwm_clk));

#if 0

	if (state->enabled && !cstate.enabled) {
		clk_prepare_enable(h616chip->pwm_clocks[pwm->hwpwm]);
printk("get rate=%lu\n", clk_get_rate(h616chip->pwm_clocks[pwm->hwpwm]));
		if (ret) {
			dev_err(pwmchip_parent(chip), "failed to enable PWM clock\n");
			return ret;
		}

	}

	if (!state->enabled && cstate.enabled) {
		clk_disable_unprepare(h616chip->pwm_clocks[pwm->hwpwm]);
	}
#endif
	if (state->enabled != cstate.enabled) {
		val = h616_pwm_readl(h616chip, PWM_ENR);
		if (state->enabled)
			val |= PWM_ENABLE(pwm->hwpwm);
		else
			val &= ~PWM_ENABLE(pwm->hwpwm);
		h616_pwm_writel(h616chip, val, PWM_ENR);
	}
	return 0;
}

static const struct pwm_ops h616_pwm_ops = {
	.apply = h616_pwm_apply,
	.get_state = h616_pwm_get_state,
	.request = h616_pwm_request,
	.free = h616_pwm_free,
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

	ret = h616_pwm_init_clocks(pdev, pwm);
	if (ret)
		return ret;

	pwm->channels = devm_kmalloc_array(dev, data->npwm,
					     sizeof(*(pwm->channels)),
					     GFP_KERNEL);
	for (int i = 0; i < data->npwm; i++) {
		struct clk_hw **hw = &pwm->clk_pdata->hw_data->hws[i];
		pwm->channels[i].pwm_clk = devm_clk_hw_get_clk(dev, *hw, NULL);
		if (IS_ERR(pwm->channels[i].pwm_clk))
			return dev_err_probe(dev, PTR_ERR(pwm->channels[i].pwm_clk),
					     "failed to register PWM clock %d\n", i);
	}

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
