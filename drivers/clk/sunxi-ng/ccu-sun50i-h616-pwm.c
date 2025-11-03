// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Arm Ltd.
 * Based on the H6 CCU driver, which is:
 *   Copyright (c) 2017 Icenowy Zheng <icenowy@aosc.io>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "ccu_common.h"

#include "ccu_div.h"
#include "ccu_gate.h"
#include "ccu_mp.h"
#include "ccu_mult.h"
#include "ccu_nk.h"
#include "ccu_nkm.h"
#include "ccu_nkmp.h"
#include "ccu_nm.h"

#include "ccu-sun50i-h616-pwm.h"

static const char * const pwm_parents[] = { "osc24M", "apb1" };

static struct clk_div_table pwm_div_table[] = {
	{ .val = 0, .div = 1 },
	{ .val = 1, .div = 2 },
	{ .val = 2, .div = 4 },
	{ .val = 3, .div = 8 },
	{ .val = 4, .div = 16 },
	{ .val = 5, .div = 32 },
	{ .val = 6, .div = 64 },
	{ .val = 7, .div = 128 },
	{ .val = 8, .div = 256 },
	{ /* Sentinel */ },
};

static SUNXI_CCU_DIV_TABLE_WITH_MUX_GATE(pwm01_clk, "pwm01", pwm_parents, 0x20,
					 pwm_div_table, 0, 4, /* div_M */
					 7, 2, /* mux */
					 4, /* gate */
					 0); /* flags */


static SUNXI_CCU_DIV_TABLE_WITH_MUX_GATE(pwm23_clk, "pwm23", pwm_parents, 0x24,
					 pwm_div_table, 0, 4, /* div_M */
					 7, 2, /* mux */
					 4, /* gate */
					 0); /* flags */


static SUNXI_CCU_DIV_TABLE_WITH_MUX_GATE(pwm45_clk, "pwm45", pwm_parents, 0x28,
					 pwm_div_table, 0, 4, /* div_M */
					 7, 2, /* mux */
					 4, /* gate */
					 0); /* flags */


static struct ccu_common *sun50i_h616_pwmcc_clks[] = {
	&pwm01_clk.common,
	&pwm23_clk.common,
	&pwm45_clk.common,
};

static struct clk_hw_onecell_data sun50i_h616_pwmcc_hw_clks = {
	.hws	= {
		[CLK_PWM01]		= &pwm01_clk.common.hw,
		[CLK_PWM23]		= &pwm23_clk.common.hw,
		[CLK_PWM45]		= &pwm45_clk.common.hw,
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
	void __iomem *reg;
	int ret;

	/* TODO: syscon */
	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

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
