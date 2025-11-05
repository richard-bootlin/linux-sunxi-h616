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
#include <linux/module.h>
#include <linux/platform_device.h>

#include "ccu-sun50i-h616-pwm.h"

#define PWM_XY_CLK_CFG(pair)		(0x20 + ((pair) * 0x4))
#define PWM_XY_CLK_CFG_SRC_SHIFT	7
#define PWM_XY_CLK_CFG_SRC_MASK		3
#define PWM_XY_CLK_CFG_GATE_BIT		4
#define PWM_XY_CLK_CFG_BYPASS_BIT(chan) ((chan) % 2 + 5)
#define PWM_XY_CLK_CFG_DIV_M_SHIFT	0

#define PWM_CTL(chan)			(0x60 + (chan) * 0x20)
#define PWM_CTL_PRESCAL_K_SHIFT		0
#define PWM_CTL_PRESCAL_K_WIDTH		8

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

#define PWM_XY_GATE(_idx, _reg)				\
struct clk_gate gate_xy_##_idx = {			\
	.reg = (void *)_reg,				\
	.bit_idx = PWM_XY_CLK_CFG_GATE_BIT,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_gate_ops,			\
	}						\
};

#define PWM_XY_SRC_MUX(_idx, _reg)			\
struct clk_mux mux_xy_##_idx = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CFG_SRC_SHIFT,		\
	.mask = PWM_XY_CLK_CFG_SRC_MASK,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_mux_ops,			\
	}						\
};

#define PWM_XY_DIV(_idx, _reg)				\
struct clk_divider rate_xy_##_idx = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CFG_DIV_M_SHIFT,		\
	.table = clk_table_xy_div,			\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_divider_ops,		\
	}						\
};

#define PWM_X_MUX(_idx, _reg, _chan)			\
struct clk_mux mux_x_##_idx = {				\
	.reg = (void *)_reg,				\
	.shift = PWM_XY_CLK_CFG_BYPASS_BIT(_chan),	\
	.mask = 1,					\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_mux_ops,			\
	}						\
};

#define PWM_X_DIV(_idx, _reg)				\
struct clk_divider rate_x_##_idx = {			\
	.reg = (void *)_reg,				\
	.shift = PWM_CTL_PRESCAL_K_SHIFT,		\
	.width = PWM_CTL_PRESCAL_K_WIDTH,		\
	.hw.init = &(struct clk_init_data){		\
		.ops =  &clk_divider_ops,		\
	}						\
};

#define PWM_XY_CLK_SRC(_idx, _reg)			\
	static PWM_XY_SRC_MUX(_idx, _reg)

#define PWM_XY_CLK(_idx, _reg)				\
	static PWM_XY_GATE(_idx, _reg);			\
	static PWM_XY_DIV(_idx, _reg)

#define PWM_X_CLK(_idx)						\
	static PWM_X_MUX(_idx, PWM_XY_CLK_CFG(_idx), _idx);	\
	static PWM_X_DIV(_idx, PWM_CTL(_idx))

#define REF_CLK_XY_SRC(_name, _idx)					\
	{								\
		.name = #_name,						\
		.parent_names = (const char *[]){ "osc24M", "apb1" },	\
		.num_parents = 2,					\
		.mux_hw = &mux_xy_##_idx.hw,				\
	}

#define REF_CLK_XY(_name, _idx, _parent)				\
	{								\
		.name = #_name,						\
		.parent_names = (const char *[]){ #_parent },		\
		.num_parents = 1,					\
		.gate_hw = &gate_xy_##_idx.hw,				\
		.rate_hw = &rate_xy_##_idx.hw,				\
	}

#define REF_CLK_X(_name, _idx, _parent1, _parent2)			\
	{								\
		.name = #_name,						\
		.parent_names = (const char *[]){ #_parent1, #_parent2 }, \
		.num_parents = 2,					\
		.mux_hw = &mux_x_##_idx.hw,				\
		.rate_hw = &rate_x_##_idx.hw,				\
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
};

struct clk_pwm_driver_data {
	struct clk_hw_onecell_data *hw_data;
	spinlock_t lock;
	void __iomem *reg;
};

static struct clk_pwm_data pwmcc_data[] = {
	REF_CLK_XY_SRC(pwm-xy-clk-src-01, 01),
	REF_CLK_XY_SRC(pwm-xy-clk-src-23, 23),
	REF_CLK_XY_SRC(pwm-xy-clk-src-45, 45),
	REF_CLK_XY(pwm-xy-clk-01, 01, pwm-xy-clk-src_01),
	REF_CLK_XY(pwm-xy-clk-23, 23, pwm-xy-clk-src_23),
	REF_CLK_XY(pwm-xy-clk-45, 45, pwm-xy-clk-src_45),
	REF_CLK_X(pwm-0, 0, pwm-xy-clk-src_01, pwm-xy-clk-01),
	REF_CLK_X(pwm-1, 1, pwm-xy-clk-src_01, pwm-xy-clk-01),
	REF_CLK_X(pwm-2, 2, pwm-xy-clk-src_23, pwm-xy-clk-23),
	REF_CLK_X(pwm-3, 3, pwm-xy-clk-src_23, pwm-xy-clk-23),
	REF_CLK_X(pwm-4, 4, pwm-xy-clk-src_45, pwm-xy-clk-45),
	REF_CLK_X(pwm-5, 5, pwm-xy-clk-src_45, pwm-xy-clk-45),
	{ /* sentinel */ },
};

struct clk_pwm_xy {
	struct clk_hw hw;
	void __iomem *reg;
	u8 shift_mux;
	u32 mask_mux;
	u8 shift_div;
};

static int sun50i_h616_add_composite_clk(const struct clk_pwm_data *data,
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
					gate_hw, gate_ops, 0);

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

	driver_data->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(driver_data->reg))
		return PTR_ERR(driver_data->reg);

	spin_lock_init(&driver_data->lock);

	clk_bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(clk_bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk_bus),
				     "Failed to get bus clock\n");

	for (int i = 0; i < num_clocks; i++) {
		struct clk_hw **hw = &driver_data->hw_data->hws[i];
		if (sun50i_h616_add_composite_clk(&pwmcc_data[i],
						  driver_data->reg,
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
