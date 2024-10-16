// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Sophgo SoCs.
 *
 * Copyright (C) 2023 Sophgo Tech Co., Ltd.
 *		http://www.sophgo.com
 *
 * Author: Lionel Li <fengchun.li@sophgo.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include "../../pci.h"
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_gpio.h>

#include "pcie-dw-sophgo.h"

static int sophgo_dw_pcie_probe(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sophgo_dw_pcie_of_match[] = {
	{ .compatible = "sophgo,sg2044-dw-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, sophgo_dw_pcie_of_match);

static struct platform_driver sophgo_dw_pcie_driver = {
	.driver = {
		.name	= "sophgo-dw-pcie",
		.of_match_table = sophgo_dw_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = sophgo_dw_pcie_probe,
};
builtin_platform_driver(sophgo_dw_pcie_driver);
