#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/platform_device.h>

#define	INTC_REGISTER_SIZE	4
#define	INTC_REGISTER_BITS	(BITS_PER_BYTE * INTC_REGISTER_SIZE)

#define HWINQ_TO_REG_OFF(_hwirq) \
	(INTC_REGISTER_SIZE * ((_hwirq) / INTC_REGISTER_BITS))

struct pcie_msi_bridge_ctrl {
	const struct irq_chip *chip;

	struct irq_domain *domain;

	void __iomem *regs;
	phys_addr_t device_report_phys;

	u32 nr_irqs;
	struct irq_fwspec fwspec;
	unsigned long *irq_bitmap;

	spinlock_t lock;
};

static void sophgo_sg2042_pcie_msi_ack_irq(struct irq_data *d)
{
	struct pcie_msi_bridge_ctrl *ctrl = irq_data_get_irq_chip_data(d);
	void __iomem *addr = ctrl->regs + HWINQ_TO_REG_OFF(d->hwirq);
	u32 value = BIT(d->hwirq % INTC_REGISTER_BITS);

	writel(value, addr);

	irq_chip_ack_parent(d);
}

static void sophgo_sg2042_pcie_msi_setup_msi_msg(struct irq_data *d,
						 struct msi_msg *msg)
{
	struct pcie_msi_bridge_ctrl *ctrl = irq_data_get_irq_chip_data(d);
	phys_addr_t addr = ctrl->device_report_phys + HWINQ_TO_REG_OFF(d->hwirq);

	msg->address_lo = lower_32_bits(addr);
	msg->address_hi = upper_32_bits(addr);
	msg->data = BIT(d->hwirq % INTC_REGISTER_BITS) ;
}

static const struct irq_chip sophgo_sg2042_pcie_msi_irq_chip = {
	.name = "Sophgo SG2042 PCIe MSI Bridge",
	.irq_ack = sophgo_sg2042_pcie_msi_ack_irq,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_compose_msi_msg = sophgo_sg2042_pcie_msi_setup_msi_msg,
};

static void sophgo_sg2044_pcie_msi_ack_irq(struct irq_data *d)
{
	struct pcie_msi_bridge_ctrl *ctrl = irq_data_get_irq_chip_data(d);

	writel(0, ctrl->regs + d->hwirq * INTC_REGISTER_SIZE);

	irq_chip_ack_parent(d);
}

static void sophgo_sg2044_pcie_msi_setup_msi_msg(struct irq_data *d,
						 struct msi_msg *msg)
{
	struct pcie_msi_bridge_ctrl *ctrl = irq_data_get_irq_chip_data(d);
	phys_addr_t addr = ctrl->device_report_phys + HWINQ_TO_REG_OFF(d->hwirq);

	msg->address_lo = lower_32_bits(addr);
	msg->address_hi = upper_32_bits(addr);
	msg->data = d->hwirq % INTC_REGISTER_BITS;
}

static const struct irq_chip sophgo_sg2044_pcie_msi_irq_chip = {
	.name = "Sophgo SG2044 PCIe MSI Bridge",
	.irq_ack = sophgo_sg2044_pcie_msi_ack_irq,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_compose_msi_msg = sophgo_sg2044_pcie_msi_setup_msi_msg,
};

// static int sophgo_pcie_msi_domain_translate(struct irq_domain *domain,
// 					    struct irq_fwspec *fwspec,
// 					    unsigned long *hwirq,
// 					    unsigned int *type)
// {
// 	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;

// 	if (fwspec->param_count != 2)
// 		return -EINVAL;
// 	if (fwspec->param[1] >= ctrl->nr_irqs)
// 		return -EINVAL;

// 	*hwirq = fwspec->param[0];
// 	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;

// 	return 0;
// }

static int sophgo_pcie_msi_domain_alloc(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs,
					void *args)
{
	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;
	irq_hw_number_t hwirq;
	unsigned long flags;
	int ret = -1;
	int i;

	spin_lock_irqsave(&ctrl->lock, flags);
	ret = bitmap_find_free_region(ctrl->irq_bitmap, ctrl->nr_irqs,
				      order_base_2(nr_irqs));
	spin_unlock_irqrestore(&ctrl->lock, flags);

	if (ret < 0) {
		pr_err("%s failed to alloc irq %d, total %d\n", __func__, virq, nr_irqs);
		return -ENOSPC;
	}

	hwirq = ret;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, hwirq + i,
				    ctrl->chip, ctrl, handle_edge_irq,
				    NULL, NULL);
	}

	return 0;
}

static void sophgo_pcie_msi_domain_free(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs)
{
	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	unsigned long flags;

	spin_lock_irqsave(&ctrl->lock, flags);
	bitmap_release_region(ctrl->irq_bitmap, data->hwirq, order_base_2(nr_irqs));
	spin_unlock_irqrestore(&ctrl->lock, flags);
}

static const struct irq_domain_ops sophgo_pcie_msi_domain_ops = {
	// .translate = sophgo_pcie_msi_domain_translate,
	.alloc = sophgo_pcie_msi_domain_alloc,
	.free = sophgo_pcie_msi_domain_free,
};

static int sophgo_pcie_msi_init(struct device *dev,
				struct pcie_msi_bridge_ctrl *ctrl)
{
	struct of_phandle_args args = {};
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct irq_domain *parent;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node, "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(dev->of_node, "msi-ranges",
					 args.args_count + 1, &ctrl->nr_irqs);
	if (ret)
		return ret;

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &ctrl->fwspec);

	dev_info(dev, "find %d args\n", args.args_count);
	dev_info(dev, "find %u irqs\n", ctrl->nr_irqs);

	ctrl->irq_bitmap = devm_bitmap_zalloc(dev, ctrl->nr_irqs, GFP_KERNEL);
	if (!ctrl->irq_bitmap)
		return -ENOMEM;

	parent = irq_find_matching_fwspec(&ctrl->fwspec, DOMAIN_BUS_ANY);
	if (!parent) {
		dev_err(dev, "failed to find parent domain\n");
		return -ENXIO;
	}

	ctrl->domain = irq_domain_create_hierarchy(parent, 0, ctrl->nr_irqs,
						   fwnode, &sophgo_pcie_msi_domain_ops,
						   ctrl);
	if (!ctrl->domain)
		return -ENOMEM;

	irq_domain_update_bus_token(ctrl->domain, DOMAIN_BUS_NEXUS);

	return 0;
}

static int sophgo_pcie_msi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pcie_msi_bridge_ctrl *ctrl;
	u64 set_addr;
	int ret;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->chip = device_get_match_data(dev);
	if (!ctrl->chip)
		return -EINVAL;

	platform_set_drvdata(pdev, ctrl);
	spin_lock_init(&ctrl->lock);

	ctrl->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->regs))
		return PTR_ERR(ctrl->regs);

	ret = of_property_read_u64(dev->of_node,
				   "sophgo,msi-device-reg", &set_addr);
	if (ret)
		return ret;

	ctrl->device_report_phys = set_addr;

	return sophgo_pcie_msi_init(dev, ctrl);
}

static const struct of_device_id sophgo_pcie_msi_of_match[] = {
	{ .compatible = "sophgo,sg2042-pcie-msi",
	  .data = &sophgo_sg2042_pcie_msi_irq_chip },
	{ .compatible = "sophgo,sg2044-pcie-msi",
	  .data = &sophgo_sg2044_pcie_msi_irq_chip },
	{ },
};
MODULE_DEVICE_TABLE(of, sophgo_pcie_msi_of_match);

static struct platform_driver sophgo_pcie_msi_driver = {
	.driver = {
		.name = "sophgo,pcie-intc",
		.suppress_bind_attrs = true,
		.of_match_table = sophgo_pcie_msi_of_match,
	},
	.probe = sophgo_pcie_msi_probe,
};
builtin_platform_driver(sophgo_pcie_msi_driver);
