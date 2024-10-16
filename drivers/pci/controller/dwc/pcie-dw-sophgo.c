// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Sophgo SoCs.
 *
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_gpio.h>

#include "pcie-designware.h"

struct sophgo_plat_pcie_of_data {
	enum dw_pcie_device_mode	mode;
};

struct sophgo_dw_pcie {
	struct dw_pcie pci;
	struct clk_bulk_data *clks;
	unsigned int nr_clks;
	struct reset_control *rst;
	struct sophgo_plat_pcie_of_data *data;
};

static const struct dw_pcie_host_ops sophgo_dw_pcie_rc_ops = {
};

static int sophgo_dw_pcie_resource_get(struct platform_device *pdev,
				       struct sophgo_dw_pcie *pcie)
{
	int ret;

	ret = devm_clk_bulk_get_all(dev, &pcie->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	pcie->nr_clks = ret;

	pcie->rst = devm_reset_control_array_get_exclusive(&pdev->dev);
	if (IS_ERR(pcie->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(pcie->rst),
				     "failed to get reset lines\n");

	return 0;
}

static int sophgo_dw_pcie_configure_rc(struct sophgo_dw_pcie *pcie)
{
	struct dw_pcie_rp *pp;

	pp = &pcie->pci.pp;
	pp->ops = &sophgo_dw_pcie_rc_ops;

	return dw_pcie_host_init(pp);
}

static int sophgo_dw_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sophgo_plat_pcie_of_data *data;
	struct sophgo_dw_pcie *pcie;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	platform_set_drvdata(pdev, pcie);

	pcie->pci.dev = dev;
	pcie->pci.ops = &dw_pcie_ops;
	pcie->data = data;

	ret = sophgo_dw_pcie_resource_get(pdev, pcie);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(pcie->nr_clks, pcie->clks);
	if (ret)
		return ret;

	switch (data->mode) {
	case DW_PCIE_RC_TYPE:
		ret = sophgo_dw_pcie_configure_rc(pcie);
		break;
	default:
		ret = -ENOTSUPP;
		break;
	}

	return ret;
}

static const struct sophgo_plat_pcie_of_data sophgo_sg2044_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct of_device_id sophgo_dw_pcie_of_match[] = {
	{ .compatible = "sophgo,sg2044-dw-pcie",
	  .data = &sophgo_sg2044_pcie_rc_of_data },
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
