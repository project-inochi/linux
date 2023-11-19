// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sophgo SoC eFuse driver
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>

#define SG2044_EFUSE_CONTENT_BASE		0x800
#define SG2044_EFUSE_CONTENT_SIZE		0x200

#define SG2044_EFUSE_MD				0x00
#define SG2044_EFUSE_BOOT_DONE			BIT(7)
#define SG2044_BOOT_TIMEOUT			10000

#define SG2044_EFUSE_ALIGN			4

struct sophgo_efuses {
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
};

static int sg2044_efuses_read(void *context, unsigned int offset, void *val,
			      size_t bytes)
{
	struct sophgo_efuses *efuse = context;
	u32 value;
	unsigned int start, start_offset, end, nstrips;
	u8 *buf;
	int ret, i = 0;

	start = rounddown(offset, SG2044_EFUSE_ALIGN);
	start_offset = offset - start;
	end = roundup(offset + bytes, SG2044_EFUSE_ALIGN);
	nstrips = (end - start) / SG2044_EFUSE_ALIGN;

	ret = readl_poll_timeout(efuse->base + SG2044_EFUSE_MD, value,
				 (value & SG2044_EFUSE_BOOT_DONE),
				 1, 10000);
	if (ret < 0)
		return ret;

	buf = kzalloc(end - start, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (nstrips--) {
		unsigned int now = start + i;

		value = readl(efuse->base + SG2044_EFUSE_CONTENT_BASE + now);
		memcpy(&buf[i], &value, SG2044_EFUSE_ALIGN);

		i += SG2044_EFUSE_ALIGN;
	}

	memcpy(val, buf + start_offset, bytes);

	kfree(buf);

	return 0;
}

static int sophgo_efuses_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sophgo_efuses *efuse;
	struct resource *res;
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.add_legacy_fixed_of_cells = true,
		.read_only = true,
		.reg_read = sg2044_efuses_read,
		.stride = 1,
		.word_size = 1,
		.name = "sophgo-efuse",
		.id = NVMEM_DEVID_AUTO,
		.root_only = true,
	};

	efuse = devm_kzalloc(dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	efuse->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);

	efuse->num_clks = devm_clk_bulk_get_all_enabled(&pdev->dev, &efuse->clks);
	if (efuse->num_clks < 0)
		return dev_err_probe(dev, efuse->num_clks, "failed to get clocks\n");

	config.priv = efuse;
	config.size = SG2044_EFUSE_CONTENT_SIZE;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(config.dev, &config));
}

static const struct of_device_id sophgo_efuses_of_match[] = {
	{ .compatible = "sophgo,sg2044-efuse", },
	{}
};

MODULE_DEVICE_TABLE(of, sophgo_efuses_of_match);

static struct platform_driver sophgo_efuses_driver = {
	.driver = {
		.name = "sophgo_efuse",
		.of_match_table = sophgo_efuses_of_match,
	},
	.probe = sophgo_efuses_probe,
};

module_platform_driver(sophgo_efuses_driver);

MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_LICENSE("GPL");
