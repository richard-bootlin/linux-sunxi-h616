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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>


/* PWM IRQ Enable Register */
#define SUNXI_PWM_IER			0x0

/* PWM IRQ Status Register */
#define SUNXI_PWM_ISR			0x4

/* PWM Capture IRQ Enable Register */
#define SUNXI_PWM_CIER			0x10

/* PWM Capture IRQ Status Register */
#define SUNXI_PWM_CISR			0x14

/* PWM01 Clock Configuration Register */
#define SUNXI_PWM_CCR01R		0x20

/* PWM23 Clock Configuration Register */
#define SUNXI_PWM_CCR23R		0x24

/* PWM45 Clock Configuration Register */
#define SUNXI_PWM_CCR45R		0x28

/* PWM01 Dead Zone Control Register */
#define SUNXI_PWM_DZCR01R		0x30

/* PWM23 Dead Zone Control Register */
#define SUNXI_PWM_DZCR23R		0x34

/* PWM45 Dead Zone Control Register */
#define SUNXI_PWM_DZCR45R		0x38

/* PWM Enable Register */
#define SUNXI_PWM_ENR			0x40

/* PWM Capture Enable Register */
#define SUNXI_PWM_CER			0x44

/* PWM Control Register */
#define SUNXI_PWM_CTRL_REG(x)		(0x60 + (x) * 0x20)

/* PWM Period Register */
#define SUNXI_PWM_PERIOD_REG(x)		(0x64 + (x) * 0x20)

/* PWM Count Register */
#define SUNXI_PWM_COUNT_REG(x)		(0x68 + (x) * 0x20)

/* PWM Capture Control Register */
#define SUNXI_PWM_CCR(x)		(0x6c + (x) * 0x20)

/* PWM Capture Rise Lock Register */
#define SUNXI_PWM_CRLR(x)		(0x70 + (x) * 0x20)

/* PWM Capture Fall Lock Register */
#define SUNXI_PWM_CFLR(x)		(0x74 + (x) * 0x20)


/* PWM Period bit field */
#define PWM_ENTIRE_CYCLE(x)		(((x) - 1) << 16)
#define PWM_ACTIVE_CYCLE(x)		((x) - 1)

/* PWM Clock Configuration bit field */
#define PMW_CLK_DIV_M(x)		(x)
#define PWM_CLK_GATING			BIT(4)
#define PWM23_CLK_SRC_BYPASS_TO_PWM2	BIT(5)
#define PWM45_CLK_SRC_BYPASS_TO_PWM4	BIT(5)
#define PWM01_CLK_SRC_BYPASS_TO_PWM1	BIT(6)
#define PWM23_CLK_SRC_BYPASS_TO_PWM3	BIT(6)
#define PWM45_CLK_SRC_BYPASS_TO_PWM5	BIT(6)
#define PWM_CLK_SRC_APB1		BIT(7)

/* PWM Enable bit field */
#define PWM_ENABLE(x)			BIT(x)


struct h616_pwm_data {
	unsigned int npwm;
};

struct h616_pwm_chip {
	struct clk_bulk_data *clocks;
	struct clk *bus_clk;
	struct clk *clk;
	struct reset_control *rst;
	void __iomem *base;
	const struct h616_pwm_data *data;
	/* Mutex to protect pwm apply state */
	struct mutex mutex;
};

static inline struct h616_pwm_chip *to_h616_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int h616_pwm_get_state(struct pwm_chip *chip,
			      struct pwm_device *pwm,
			      struct pwm_state *state)
{
	return 0;
}

static int h616_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			  const struct pwm_state *state)
{
	return 0;
}

static const struct pwm_ops h616_pwm_ops = {
	.apply = h616_pwm_apply,
	.get_state = h616_pwm_get_state,
};


static int h616_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	const struct h616_pwm_data *data;
	struct h616_pwm_chip *h616chip;
	int ret;

	data = of_device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	chip = devm_pwmchip_alloc(&pdev->dev, data->npwm, sizeof(*h616chip));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	h616chip = to_h616_pwm_chip(chip);
	h616chip->data = data;
	h616chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(h616chip->base))
		return PTR_ERR(h616chip->base);

	/* 
	   ret = devm_clk_bulk_get_all(&pdev->dev, &h616_chip->clocks);
	   if (ret < 0)
	   return dev_err_probe(&pdev->dev, ret, "failed to get clocks\n");
	   else if (!ret)
	   return dev_err_probe(&pdev->dev, -EINVAL, "no clocks in DT\n");
	   */
	h616chip->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(h616chip->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(h616chip->rst),
				     "get reset failed\n");

	/* Deassert reset */
	ret = reset_control_deassert(h616chip->rst);
	if (ret) {
		dev_err(&pdev->dev, "cannot deassert reset control: %pe\n",
			ERR_PTR(ret));
		return ret;
	}

	chip->ops = &h616_pwm_ops;

	ret = pwmchip_add(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		goto err_pwm_add;
	}

	platform_set_drvdata(pdev, chip);

	return 0;

err_pwm_add:
	clk_disable_unprepare(h616chip->bus_clk);
	reset_control_assert(h616chip->rst);

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
