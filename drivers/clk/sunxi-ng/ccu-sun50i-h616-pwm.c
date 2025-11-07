// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinnner H616 PWM clock controller
 *
 * Copyright (c) 2025 Bootlin, Richard GENOUD <richard.genoud@bootlin.com>
 *
 * Based on the Marvell Armada 37xx SoC Peripheral clocks which is:
 *   Copyright (c) 2016 Marvell
 *   Gregory CLEMENT <gregory.clement@bootlin.com>
 *
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

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "ccu-sun50i-h616-pwm.h"

#define PWM_XY_CLK_CFG(pair)		(0x20 + ((pair) * 0x4))
#define PWM_XY_CLK_CFG_SRC_SHIFT	7
#define PWM_XY_CLK_CFG_SRC_MASK		1
#define PWM_XY_CLK_CFG_GATE_BIT		4
#define PWM_XY_CLK_CFG_BYPASS_BIT(chan) ((chan) % 2 + 5)
#define PWM_XY_CLK_CFG_DIV_M_SHIFT	0

#define PWM_CTL(chan)			(0x60 + (chan) * 0x20)
#define PWM_CTL_PRESCAL_K_SHIFT		0
#define PWM_CTL_PRESCAL_K_WIDTH		8

struct clk_pwm_divider {
	struct clk_divider div;
	struct regmap *base;
	unsigned int reg;
};

struct clk_pwm_mux {
	struct clk_mux mux;
	struct regmap *base;
	unsigned int reg;
};

struct clk_pwm_gate {
	struct clk_gate gate;
	struct regmap *base;
	unsigned int reg;
};

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

static const struct clk_ops clk_pwm_gate_ops;
static const struct clk_ops clk_pwm_div_ops;
static const struct clk_ops clk_pwm_mux_ops;

#define PWM_XY_GATE(_pair, _reg)			\
struct clk_pwm_gate gate_xy_##_pair = {			\
	.reg = _reg,					\
	.gate.bit_idx = PWM_XY_CLK_CFG_GATE_BIT,	\
	.gate.hw.init = &(struct clk_init_data){	\
		.ops =  &clk_pwm_gate_ops,		\
	}						\
};

#define PWM_XY_SRC_MUX(_pair, _reg)			\
struct clk_pwm_mux mux_xy_##_pair = {			\
	.reg = _reg,					\
	.mux.shift = PWM_XY_CLK_CFG_SRC_SHIFT,		\
	.mux.mask = PWM_XY_CLK_CFG_SRC_MASK,		\
	.mux.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_pwm_mux_ops,		\
	}						\
};

#define PWM_XY_DIV(_pair, _reg)				\
struct clk_pwm_divider rate_xy_##_pair = {		\
	.reg = _reg,					\
	.div.shift = PWM_XY_CLK_CFG_DIV_M_SHIFT,	\
	.div.table = clk_table_xy_div,			\
	.div.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_pwm_div_ops,		\
	}						\
};

#define PWM_X_MUX(_idx, _reg, _chan)			\
struct clk_pwm_mux mux_x_##_idx = {			\
	.reg = _reg,					\
	.mux.shift = PWM_XY_CLK_CFG_BYPASS_BIT(_chan),	\
	.mux.mask = 1,					\
	.mux.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_pwm_mux_ops,		\
	}						\
};

#define PWM_X_DIV(_idx, _reg)				\
struct clk_pwm_divider rate_x_##_idx = {		\
	.reg = _reg,					\
	.div.shift = PWM_CTL_PRESCAL_K_SHIFT,		\
	.div.width = PWM_CTL_PRESCAL_K_WIDTH,		\
	.div.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_pwm_divider_ops,		\
	}						\
};

#define PWM_XY_CLK_SRC(_pair, _reg)			\
	static PWM_XY_SRC_MUX(_pair, _reg)

#define PWM_XY_CLK(_pair, _reg)				\
	static PWM_XY_GATE(_pair, _reg);		\
	static PWM_XY_DIV(_pair, _reg)

#define PWM_X_CLK(_idx)							\
	static PWM_X_MUX(_idx, PWM_XY_CLK_CFG((_idx) >> 1), _idx);	\
	static PWM_X_DIV(_idx, PWM_CTL(_idx))

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
PWM_XY_CLK_SRC(01, PWM_XY_CLK_CFG(0));
PWM_XY_CLK_SRC(23, PWM_XY_CLK_CFG(1));
PWM_XY_CLK_SRC(45, PWM_XY_CLK_CFG(2));

/*
 * Clocks obtained after the 1st div
 *              ______      ________
 * PWM_clk_src |      |    |        |
 * ----------->| Gate |--->| /div_m |--> PWM_clock_xy
 *             |______|    |________|
 *
 */
PWM_XY_CLK(01, PWM_XY_CLK_CFG(0));
PWM_XY_CLK(23, PWM_XY_CLK_CFG(1));
PWM_XY_CLK(45, PWM_XY_CLK_CFG(2));

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

struct clk_pwm_driver_data {
	struct clk_hw_onecell_data *hw_data;
	spinlock_t lock;
	struct regmap *map;
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

#define to_clk_pwm_div(_hw) container_of(_hw, struct clk_pwm_div, hw)
#define to_clk_pwm_mux(_hw) container_of(_hw, struct clk_pwm_mux, hw)
#define to_clk_pwm_gate(_hw) container_of(_hw, struct clk_pwm_gate, hw)

/* clk-divider ops */
static inline u32 clk_div_readl(struct clk_divider *divider)
{
	struct clk_pwm_divider *pwm_div_clk = to_clk_pwm_div(divider);
	unsigned int val = 0;

	regmap_read(pwm_div_clk->base, pwm_div_clk->reg, &val);

	return val;
}

static inline void clk_div_writel(struct clk_divider *divider, unsigned int val)
{
	struct clk_pwm_divider *pwm_div_clk = to_clk_pwm_div(divider);

	regmap_write(pwm_div_clk->base, pwm_div_clk->reg, val);
}

static unsigned long clk_divider_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int val;

	val = clk_div_readl(divider) >> divider->shift;
	val &= clk_div_mask(divider->width);

	return divider_recalc_rate(hw, parent_rate, val, divider->table,
				   divider->flags, divider->width);
}

static int clk_divider_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	struct clk_divider *divider = to_clk_divider(hw);

	/* if read only, just return current value */
	if (divider->flags & CLK_DIVIDER_READ_ONLY) {
		u32 val;

		val = clk_div_readl(divider) >> divider->shift;
		val &= clk_div_mask(divider->width);

		return divider_ro_determine_rate(hw, req, divider->table,
						 divider->width,
						 divider->flags, val);
	}

	return divider_determine_rate(hw, req, divider->table, divider->width,
				      divider->flags);
}

static int clk_divider_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	int value;
	unsigned long flags = 0;
	u32 val;

	value = divider_get_val(rate, parent_rate, divider->table,
				divider->width, divider->flags);
	if (value < 0)
		return value;

	if (divider->lock)
		spin_lock_irqsave(divider->lock, flags);
	else
		__acquire(divider->lock);

	if (divider->flags & CLK_DIVIDER_HIWORD_MASK) {
		val = clk_div_mask(divider->width) << (divider->shift + 16);
	} else {
		val = clk_div_readl(divider);
		val &= ~(clk_div_mask(divider->width) << divider->shift);
	}
	val |= (u32)value << divider->shift;
	clk_div_writel(divider, val);

	if (divider->lock)
		spin_unlock_irqrestore(divider->lock, flags);
	else
		__release(divider->lock);

	return 0;
}

static const struct clk_ops clk_pwm_divider_ops = {
	.recalc_rate = clk_divider_recalc_rate,
	.determine_rate = clk_divider_determine_rate,
	.set_rate = clk_divider_set_rate,
};

static inline u32 clk_mux_readl(struct clk_mux *mux)
{
	struct clk_pwm_mux *pwm_mux_clk = to_clk_pwm_mux(mux);
	unsigned int val = 0;

	regmap_read(pwm_mux_clk->base, pwm_mux_clk->reg, &val);

	return val;
}

static inline void clk_mux_writel(struct clk_mux *mux, unsigned int val)
{
	struct clk_pwm_mux *pwm_mux_clk = to_clk_pwm_mux(mux);

	regmap_write(pwm_mux_clk->base, pwm_mux_clk->reg, val);
}

static u8 clk_mux_get_parent(struct clk_hw *hw)
{
	struct clk_mux *mux = to_clk_mux(hw);
	u32 val;

	val = clk_mux_readl(mux) >> mux->shift;
	val &= mux->mask;

	return clk_mux_val_to_index(hw, mux->table, mux->flags, val);
}

static int clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_mux *mux = to_clk_mux(hw);
	u32 val = clk_mux_index_to_val(mux->table, mux->flags, index);
	unsigned long flags = 0;
	u32 reg;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);
	else
		__acquire(mux->lock);

	if (mux->flags & CLK_MUX_HIWORD_MASK) {
		reg = mux->mask << (mux->shift + 16);
	} else {
		reg = clk_mux_readl(mux);
		reg &= ~(mux->mask << mux->shift);
	}
	val = val << mux->shift;
	reg |= val;
	clk_mux_writel(mux, reg);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);
	else
		__release(mux->lock);

	return 0;
}


static int clk_mux_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct clk_mux *mux = to_clk_mux(hw);

	return clk_mux_determine_rate_flags(hw, req, mux->flags);
}

static const struct clk_ops clk_pwm_mux_ops = {
	.get_parent = clk_mux_get_parent,
	.set_parent = clk_mux_set_parent,
	.determine_rate = clk_mux_determine_rate,
};

static inline u32 clk_gate_readl(struct clk_gate *gate)
{
	struct clk_pwm_gate *pwm_gate_clk = to_clk_pwm_gate(gate);
	unsigned int val = 0;

	regmap_read(pwm_gate_clk->base, pwm_gate_clk->reg, &val);

	return val;
}

static inline void clk_gate_writel(struct clk_gate *gate, u32 val)
{
	struct clk_pwm_gate *pwm_gate_clk = to_clk_pwm_gate(gate);

	regmap_write(pwm_gate_clk->base, pwm_gate_clk->reg, val);
}

static void clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct clk_gate *gate = to_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long flags;
	u32 reg;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);
	else
		__acquire(gate->lock);

	if (gate->flags & CLK_GATE_HIWORD_MASK) {
		reg = BIT(gate->bit_idx + 16);
		if (set)
			reg |= BIT(gate->bit_idx);
	} else {
		reg = clk_gate_readl(gate);

		if (set)
			reg |= BIT(gate->bit_idx);
		else
			reg &= ~BIT(gate->bit_idx);
	}

	clk_gate_writel(gate, reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
	else
		__release(gate->lock);
}

static int clk_gate_enable(struct clk_hw *hw)
{
	clk_gate_endisable(hw, 1);

	return 0;
}

static void clk_gate_disable(struct clk_hw *hw)
{
	clk_gate_endisable(hw, 0);
}

static int clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct clk_gate *gate = to_clk_gate(hw);

	reg = clk_gate_readl(gate);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);

	return reg ? 1 : 0;
}

static const struct clk_ops clk_pwm_gate_ops = {
	.enable = clk_gate_enable,
	.disable = clk_gate_disable,
	.is_enabled = clk_gate_is_enabled,
};

static int sun50i_h616_add_composite_clk(const struct clk_pwm_data *data,
					 struct regmap *map, spinlock_t *lock,
					 struct device *dev, struct clk_hw **hw)
{
	const struct clk_ops *mux_ops = NULL, *gate_ops = NULL, *rate_ops = NULL;
	struct clk_hw *mux_hw = NULL, *gate_hw = NULL, *rate_hw = NULL;


	if (data->mux_hw) {
		struct clk_pwm_mux *mux_pwm;
		struct clk_mux *mux;

		mux_hw = data->mux_hw;
		mux = to_clk_mux(mux_hw);
		mux->lock = lock;
		mux_ops = mux_hw->init->ops;
		mux_pwm = to_clk_pwm_mux(mux);
		mux_pwm->base = map;
	}

	if (data->gate_hw) {
		struct clk_gate *gate;

		gate_hw = data->gate_hw;
		gate = to_clk_gate(gate_hw);
		gate->lock = lock;
		gate_ops = gate_hw->init->ops;
		gate_pwm = to_clk_pwm_gate(gate);
		gate_pwm->base = map;
	}

	if (data->rate_hw) {
		struct clk_pwm_divider *rate_pwm;
		struct clk_divider *rate;

		rate_hw = data->rate_hw;
		rate = to_clk_divider(rate_hw);
		rate_ops = rate_hw->init->ops;
		rate->lock = lock;
		rate_pwm = to_clk_pwm_div(rate);
		rate_pwm->base = map;

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

static int sun50i_h616_pwmcc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_pwm_driver_data *driver_data;
	struct device *dev = &pdev->dev;
	struct clk *clk_bus;
	int num_clocks = 0;
	int ret;

	driver_data = devm_kzalloc(dev, sizeof(*driver_data), GFP_KERNEL);
	if (!driver_data)
		return -ENOMEM;

	while (pwmcc_data[num_clocks].name)
		num_clocks++;

	driver_data->hw_data = devm_kzalloc(dev,
					    struct_size(driver_data->hw_data,
							hws, num_clocks),
					    GFP_KERNEL);
	if (!driver_data->hw_data)
		return -ENOMEM;

	driver_data->hw_data->num = num_clocks;

	driver_data->map = syscon_regmap_lookup_by_compatible("allwinner,sun50i-h616-pwm");

	spin_lock_init(&driver_data->lock);

	clk_bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(clk_bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk_bus),
				     "Failed to get bus clock\n");

	for (int i = 0; i < num_clocks; i++) {
		struct clk_hw **hw = &driver_data->hw_data->hws[i];
		if (sun50i_h616_add_composite_clk(&pwmcc_data[i],
						  driver_data->map,
						  &driver_data->lock, dev, hw))
			dev_err(dev, "Can't register pwm clock %s\n",
				pwmcc_data[i].name);
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get,
				     driver_data->hw_data);
	if (ret) {
		for (int i = 0; i < num_clocks; i++)
			clk_hw_unregister(driver_data->hw_data->hws[i]);
		return ret;
	}

	platform_set_drvdata(pdev, driver_data);

	return 0;
}

static const struct of_device_id sun50i_h616_pwmcc_ids[] = {
	{ .compatible = "allwinner,sun50i-h616-pwmcc" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sun50i_h616_pwmcc_ids);

static struct platform_driver sun50i_h616_pwmcc_driver = {
	.probe	= sun50i_h616_pwmcc_probe,
	.driver	= {
		.name			= "sun50i-h616-pwmcc",
		.of_match_table		= sun50i_h616_pwmcc_ids,
	},
};

builtin_platform_driver(sun50i_h616_pwmcc_driver);

MODULE_DESCRIPTION("Support for the Allwinner H616 PWM clock controller");
MODULE_LICENSE("GPL");
