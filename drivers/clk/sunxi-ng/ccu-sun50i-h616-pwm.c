// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Arm Ltd.
 * Based on the H6 CCU driver, which is:
 *   Copyright (c) 2017 Icenowy Zheng <icenowy@aosc.io>
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "ccu_common.h"

#include "ccu_div.h"

#include "ccu-sun50i-h616-pwm.h"

#define SUNXI_PWMCC(_struct, _name, _parents, _reg, _shift, _width,	\
		    _muxshift, _muxwidth, _gate, _flags)		\
	struct ccu_div _struct = {					\
		.enable		= _gate,				\
		.div		= _SUNXI_CCU_DIV_FLAGS(_shift, _width,	\
						       CLK_DIVIDER_POWER_OF_TWO), \
		.mux	= _SUNXI_CCU_MUX(_muxshift, _muxwidth),		\
		.common	= {						\
			.reg		= _reg,				\
			.hw.init	= CLK_HW_INIT_PARENTS(_name,	\
							      _parents, \
							      &ccu_div_ops, \
							      _flags),	\
		}							\
	}

static const char * const pwm_parents[] = { "osc24M", "apb1" };

static SUNXI_PWMCC(pwm01_clk, "pwm01", pwm_parents, 0x20,
		   0, 4, /* div_M */
		   7, 2, /* mux */
		   4,    /* gate */
		   0);   /* flags */


static SUNXI_PWMCC(pwm23_clk, "pwm23", pwm_parents, 0x24,
		   0, 4, /* div_M */
		   7, 2, /* mux */
		   4,    /* gate */
		   0);   /* flags */


static SUNXI_PWMCC(pwm45_clk, "pwm45", pwm_parents, 0x28,
		    0, 4, /* div_M */
		    7, 2, /* mux */
		    4,    /* gate */
		    0);   /* flags */


static struct ccu_common *sun50i_h616_pwmcc_clks[] = {
	&pwm01_clk.common,
	&pwm23_clk.common,
	&pwm45_clk.common,
};

static struct clk_hw_onecell_data sun50i_h616_pwmcc_hw_clks = {
	.hws	= {
		[CLK_PWM01] = &pwm01_clk.common.hw,
		[CLK_PWM23] = &pwm23_clk.common.hw,
		[CLK_PWM45] = &pwm45_clk.common.hw,
	},
	.num = CLK_NUMBER,
};

static const struct sunxi_ccu_desc sun50i_h616_pwmcc_desc = {
	.ccu_clks	= sun50i_h616_pwmcc_clks,
	.num_ccu_clks	= ARRAY_SIZE(sun50i_h616_pwmcc_clks),

	.hw_clks	= &sun50i_h616_pwmcc_hw_clks,
};

static int sun50i_h616_pwmcc_probe(struct platform_device *pdev)
{
	struct clk *clk_bus;
	void __iomem *reg;
	int ret;

	/* TODO: syscon */
	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	clk_bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(clk_bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk_bus),
				     "Failed to get bus clock\n");

	ret = devm_sunxi_ccu_probe(&pdev->dev, reg, &sun50i_h616_pwmcc_desc);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id sun50i_h616_pwmcc_ids[] = {
	{ .compatible = "allwinner,sun50i-h616-pwmcc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun50i_h616_pwmcc_ids);

static struct platform_driver sun50i_h616_pwmcc_driver = {
	.probe	= sun50i_h616_pwmcc_probe,
	.driver	= {
		.name			= "sun50i-h616-pwmcc",
		.suppress_bind_attrs	= true,
		.of_match_table		= sun50i_h616_pwmcc_ids,
	},
};
module_platform_driver(sun50i_h616_pwmcc_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Support for the Allwinner H616 PWM clock controller");
MODULE_LICENSE("GPL");
