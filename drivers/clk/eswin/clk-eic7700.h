/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Inochi Amaoto <inochiama@outlook.com>
 */

#ifndef _CLK_ESWIN_EIC7700_H_
#define _CLK_ESWIN_EIC7700_H_

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/spinlock.h>

struct eic7700_clk_common {
	struct clk_hw	hw;
	void __iomem	*base;
	spinlock_t	*lock;
	unsigned int	id;
};

static inline struct eic7700_clk_common *
hw_to_eic7700_clk_common(struct clk_hw *hw)
{
	return container_of(hw, struct eic7700_clk_common, hw);
}

#endif /* _CLK_ESWIN_EIC7700_H_ */
