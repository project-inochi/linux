
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Inochi Amaoto <inochiama@outlook.com>
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/spinlock.h>
#include <linux/bug.h>
#include <linux/platform_device.h>

#include "clk-eic7700.h"

struct eic7700_pll_internal {
	u16				ctrl_offset;
	u16				status_offset;
	u8				status_lock_shift;
};

struct eic7700_pll {
	struct eic7700_clk_common	common;
	struct eic7700_pll_internal	pll;
};

#define PLL_CFG0			0x00
#define	PLL_EN				BIT(0)
#define	PLL_FRAC_MODE			BIT(9)
#define	PLL_REFDIV_MASK			GENMASK(17, 12)
#define	PLL_FBDEV_MASK			GENMASK(31, 20)

#define PLL_CFG1			0x04
#define PLL_FRAC_MASK			GENMASK(27, 4)

#define PLL_CFG2			0x08
#define PLL_POSTDIV0_MASK		GENMASK(3, 1)
#define PLL_POSTDIV1_MASK		GENMASK(18, 16)

#define PLL_DSKEWCAL			0x0c
#define PLL_SSC				0x10

#define PLL_FRAC_SHIFT			24

struct eic7700_div_internal {
	u32	offset;
	u8	shift;
	u8	width;
	u16	flags;
};

struct eic7700_gate_internal {
	u32	offset;
	u8	shift;
};

struct eic7700_div {
	struct eic7700_clk_common	common;
	struct eic7700_div_internal	div;
	struct eic7700_gate_internal	gate;
};

struct eic7700_fixed_div {
	struct eic7700_clk_common	common;
};

static inline struct eic7700_pll *hw_to_eic7700_pll(struct clk_hw *hw)
{
	struct eic7700_clk_common *common = hw_to_eic7700_clk_common(hw);

	return container_of(common, struct eic7700_pll, common);
}

static int eic7700_pll_enable(struct clk_hw *hw)
{
	struct eic7700_pll *pll = hw_to_eic7700_pll(hw);
	void __iomem *addr = pll->common.base + pll->pll.ctrl_offset;
	u32 val;

	guard(spinlock_irqsave)(pll->common.lock);

	val = readl(addr);
	val |= PLL_EN;
	writel(val, addr);

	return 0;
}

static void eic7700_pll_disable(struct clk_hw *hw)
{
	struct eic7700_pll *pll = hw_to_eic7700_pll(hw);
	void __iomem *addr = pll->common.base + pll->pll.ctrl_offset;
	u32 val;

	val = readl(addr);
	val &= ~PLL_EN;
	writel(val, addr);
}

static int eic7700_pll_is_enable(struct clk_hw *hw)
{
	struct eic7700_pll *pll = hw_to_eic7700_pll(hw);

	return readl(pll->common.base + pll->pll.ctrl_offset) & PLL_EN;
}
/*
 * parent_rate: 31 bit
 * fbdiv: 12 bit
 * refdiv: 6 bit
 * frac: 24bit
 * postdiv0: 3 bit
 * postdiv1: 3 bit
 */
static unsigned long eic7700_ipll_recalc_rate(unsigned long parent_rate,
					      unsigned long fbdiv,
					      unsigned long refdiv,
					      unsigned long postdiv0,
					      unsigned long postdiv1)
{
	u64 dividend = parent_rate * fbdiv;
	u64 divisor = 4 * refdiv * (postdiv0 + 1) * (postdiv1 + 1);

	return div64_u64(dividend, divisor);
}

static unsigned long eic7700_fpll_recalc_rate(unsigned long parent_rate,
					      unsigned long fbdiv,
					      unsigned long refdiv,
					      unsigned long frac,
					      unsigned long postdiv0,
					      unsigned long postdiv1)
{
	u64 dividend = (parent_rate << PLL_FRAC_SHIFT) * fbdiv + frac;
	u64 divisor = 4 * refdiv * (postdiv0 + 1) * (postdiv1 + 1);

	divisor <<= PLL_FRAC_SHIFT;

	return div64_u64(dividend, divisor);
}

static unsigned long eic7700_pll_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct eic7700_pll *pll = hw_to_eic7700_pll(hw);
	void __iomem *addr = pll->common.base + pll->pll.ctrl_offset;
	unsigned int cfg0, cfg1, cfg2;
	unsigned int fbdiv, refdiv, postdiv0, postdiv1, frac;

	cfg0 = readl(addr + PLL_CFG0);
	cfg1 = readl(addr + PLL_CFG1);
	cfg2 = readl(addr + PLL_CFG2);

	fbdiv = FIELD_GET(PLL_FBDEV_MASK, cfg0);
	refdiv = FIELD_GET(PLL_REFDIV_MASK, cfg0);
	frac = FIELD_GET(PLL_FRAC_MASK, cfg1);
	postdiv0 = FIELD_GET(PLL_POSTDIV0_MASK, cfg2);
	postdiv1 = FIELD_GET(PLL_POSTDIV1_MASK, cfg2);

	if (cfg0 & PLL_FRAC_MODE)
		return eic7700_fpll_recalc_rate(parent_rate, fbdiv, refdiv,
						frac, postdiv0, postdiv1);

	return eic7700_ipll_recalc_rate(parent_rate, fbdiv, refdiv,
					postdiv0, postdiv1);
}

static int eic7700_pll_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	return 0;
}

static int eic7700_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	return 0;
}

const struct clk_ops eic7700_gateable_pll_ops = {
	.disable = eic7700_pll_disable,
	.enable = eic7700_pll_enable,
	.is_enabled = eic7700_pll_is_enable,

	.recalc_rate = eic7700_pll_recalc_rate,
	.determine_rate = eic7700_pll_determine_rate,
	.set_rate = eic7700_pll_set_rate,
};

static inline struct eic7700_div *hw_to_eic7700_div(struct clk_hw *hw)
{
	struct eic7700_clk_common *common = hw_to_eic7700_clk_common(hw);

	return container_of(common, struct eic7700_div, common);
}

static u32 eic7700_div_get_divsor(struct eic7700_clk_common *common,
				  struct eic7700_div_internal *div)
{
	u32 reg = readl(common->base + div->offset);
	u32 value = (reg >> div->shift) & clk_div_mask(div->width);

	if (value < 2 && !(div->flags & CLK_DIVIDER_ALLOW_ZERO))
		value = 2;

	return value;
}

static unsigned long eic7700_div_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct eic7700_div *div = hw_to_eic7700_div(hw);
	u32 value = eic7700_div_get_divsor(&div->common, &div->div);

	return divider_recalc_rate(hw, parent_rate, value, NULL,
				   div->div.flags, div->div.width);
}

static int eic7700_div_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	struct eic7700_div *div = hw_to_eic7700_div(hw);
	u32 value;
	int ret;

	ret = divider_determine_rate(hw, req, NULL, div->div.width, div->div.flags);
	if (ret)
		return ret;

	value = divider_get_val(req->rate, req->best_parent_rate, NULL,
				div->div.width, div->div.flags);

	if (value < 2 && !(div->div.flags & CLK_DIVIDER_ALLOW_ZERO))
		req->rate = DIV_ROUND_UP_ULL((u64)req->best_parent_rate, 2);

	return 0;
}

static void eic7700_div_set_reg_div(struct eic7700_clk_common *common,
				    struct eic7700_div_internal *div,
				    struct eic7700_gate_internal *gate,
				    u32 value)
{
	void __iomem *div_addr = common->base + div->offset;
	void __iomem *gate_addr = common->base + gate->offset;
	u32 reg;

	/* gate */
	reg = readl(gate_addr);
	reg &= ~BIT(gate->shift);
	writel(reg, gate_addr);

	/* set value */
	reg = readl(div_addr);
	reg &= ~(clk_div_mask(div->width) << div->shift);
	reg |= (value << div->shift);
	writel(reg, div_addr);

	__delay(32);

	/* release gate */
	reg = readl(gate_addr);
	reg |= BIT(gate->shift);
	writel(reg, gate_addr);
}

static int eic7700_div_set_rate(struct clk_hw *hw,
				unsigned long rate, unsigned long parent_rate)
{
	struct eic7700_div *div = hw_to_eic7700_div(hw);
	u32 value;

	value = divider_get_val(rate, parent_rate, NULL,
				div->div.width, div->div.flags);

	guard(spinlock_irqsave)(div->common.lock);

	eic7700_div_set_reg_div(&div->common, &div->div, &div->gate, value);

	return 0;
}

const struct clk_ops eic7700_div_ops = {
	.recalc_rate = eic7700_div_recalc_rate,
	.determine_rate = eic7700_div_determine_rate,
	.set_rate = eic7700_div_set_rate,
};

static int eic7700_clk_probe(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id eic7700_clk_ids[] = {
	{ .compatible = "eswin,eic7700-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, eic7700_clk_ids);

static struct platform_driver eic7700_clk_driver = {
	.probe	= eic7700_clk_probe,
	.driver	= {
		.name			= "eic7700-clk",
		.suppress_bind_attrs	= true,
		.of_match_table		= eic7700_clk_ids,
	},
};
module_platform_driver(eic7700_clk_driver);
MODULE_DESCRIPTION("ESWIN eic7700 series SoCs clock controller");
MODULE_LICENSE("GPL");
