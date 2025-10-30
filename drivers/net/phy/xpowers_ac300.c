// SPDX-License-Identifier: GPL-2.0-only
/*
 * X-Powers AC300 PHY control driver
 *
 * Copyright (C) 2025 Bootlin, Richard Genoud <richard.genoud@bootlin.com>
 *
 * Description:
 * The purpose of this driver is enabling and configuring the embedded PHY
 * of the AC300 chip.
 * The EPHY configuration is achieved through MDIO.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/phy.h>
#include <linux/types.h>

#define AC300_CTRL_PHY_ID	0xc0000000
#define AC300_CTRL_PHY_ID_MASK	0xfffffff0
#define AC300_CLOCK_FQ		24000000UL

#define AC300_SYS_CTRL_REG	0x0

/* System Control Register bits */
#define AC300_CHIP_VER_MASK	GENMASK(15, 12)
#define AC300_PKG_STATUS_MASK	GENMASK(11, 8)
#define AC300_CLK_SEL_SHIFT	6
#define AC300_CLK_25MHZ		(0 << AC300_CLK_SEL_SHIFT)
#define AC300_CLK_27MHZ		(1 << AC300_CLK_SEL_SHIFT)
#define AC300_CLK_24MHZ		(2 << AC300_CLK_SEL_SHIFT)
#define AC300_EPHUSE_CLK_GATING BIT(5)
#define AC300_EPHY_CLK_GATING	BIT(4)
#define AC300_MDIO_ERROR	BIT(3)  /* W1C */
#define AC300_CLKIN_GATING	BIT(2)
#define AC300_EPHY_NRESET	BIT(1)
#define AC300_CHIP_NRESET	BIT(0)

#define AC300_SYS_IO_REG	0x5

/* System IO register bits */
#define AC300_MDIO_DRV(lvl)	((lvl) << 14)
#define AC300_LED_DRV(lvl)	((lvl) << 12)
#define AC300_MII_DRV(lvl)	((lvl) << 10)
#define AC300_EPHY_IRQ_STATUS	BIT(9)
#define AC300_EPHY_IRQ_EN	BIT(8)
#define AC300_CLKIN_PAD_EN	BIT(4)
#define AC300_E_DPX_LED_IO_EN	BIT(3)
#define AC300_E_SPD_LED_IO_EN	BIT(2)
#define AC300_E_LNK_LED_IO_EN	BIT(1)
#define AC300_EPHY_MII_IO_EN	BIT(0)

#define AC300_EPHY_CONF_REG	0x6

/* EPHY configuration register bits */
#define AC300_BGS_EFFUSE(val)	((val) << 12)
#define AC300_RMII_SEL		BIT(11)
#define AC300_EPHY_MODE_SHIFT	9
#define AC300_EPHY_MODE_NORMAL	(0 << AC300_EPHY_MODE_SHIFT)
#define AC300_EPHY_MODE_SIM	(1 << AC300_EPHY_MODE_SHIFT)
#define AC300_EPHY_MODE_AFE	(2 << AC300_EPHY_MODE_SHIFT)
#define AC300_BIST_CLK_EN	BIT(3)
#define AC300_LED_POLARITY	BIT(1)
#define AC300_SHUTDOWN		BIT(0)

struct ac300_write_seq {
	u32 reg;
	u16 val;
	unsigned long delay_us;
};

const struct ac300_write_seq ac300_init_seq[] = {
	{
		/* reset */
		AC300_SYS_CTRL_REG, 0, 0
	},
	{
		/* de-reset */
		AC300_SYS_CTRL_REG, AC300_EPHY_NRESET | AC300_CHIP_NRESET, 0
	},
	{
		/* open clk gate */
		AC300_SYS_CTRL_REG, AC300_CLK_24MHZ |
			AC300_EPHUSE_CLK_GATING | AC300_EPHY_CLK_GATING |
			AC300_CLKIN_GATING | AC300_EPHY_NRESET |
			AC300_CHIP_NRESET, 0
	},
	{
		/* enable IO */
		AC300_SYS_IO_REG, AC300_MDIO_DRV(2) | AC300_LED_DRV(2) |
			AC300_MII_DRV(2) | AC300_CLKIN_PAD_EN |
			AC300_E_DPX_LED_IO_EN | AC300_E_SPD_LED_IO_EN |
			AC300_E_LNK_LED_IO_EN | AC300_EPHY_MII_IO_EN, 10000
	},
	{
		/* shutdown EPHY */
		AC300_EPHY_CONF_REG, AC300_RMII_SEL | AC300_SHUTDOWN, 10000
	},
	{
		/* power up EPHY */
		AC300_EPHY_CONF_REG, AC300_RMII_SEL, 0
	},
};

static int ac300_write_seq(struct phy_device *phydev,
			   const struct ac300_write_seq *seq, size_t num_regs)
{
	int err = 0;

	for (; num_regs-- > 0; seq++) {
		err = phy_write(phydev, seq->reg, seq->val);
		if (err)
			break;
		fsleep(seq->delay_us);
	}

	return err;
}

static int ac300_config_init(struct phy_device *phydev)
{
	return ac300_write_seq(phydev, ac300_init_seq,
			       ARRAY_SIZE(ac300_init_seq));
}

static int ac300_ephy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct clk *clk;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to request clock\n");

	clk_set_rate_exclusive(clk, AC300_CLOCK_FQ);

	return 0;
}

static struct phy_driver ac300_control_driver[] = {
	{
		.phy_id		= AC300_CTRL_PHY_ID,
		.phy_id_mask	= AC300_CTRL_PHY_ID_MASK,
		.name		= "X-Powers AC300 PHY control",
		.config_init	= ac300_config_init,
		.probe		= ac300_ephy_probe,
	}
};
module_phy_driver(ac300_control_driver);

MODULE_AUTHOR("Richard Genoud <richard.genoud@bootlin.com>");
MODULE_DESCRIPTION("X-Powers AC300 PHY control driver");
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused ac300_control_phy_tbl[] = {
	{ AC300_CTRL_PHY_ID, AC300_CTRL_PHY_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, ac300_control_phy_tbl);
