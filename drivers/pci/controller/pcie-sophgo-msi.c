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

#define MAX_IRQ_NUMBER 512
/*
 * here we assume all plic hwirq and pic(PCIe Interrupt
 * Controller) hwirq should be contiguous.
 * sophgo_pcie_msi hwirq is index of bitmap (both software and
 * hardware), and starts from 0.
 * so we use pic hwirq as index to get plic hwirq and its
 * irq data.
 * when used as a msi parent, pic hwirq is written to Top
 * reg for triggering irq by a PCIe device.
 *
 * now we pre-requested plic interrupt, but may try request
 * plic interrupt when needed, like gicp_irq_domain_alloc.
 */

struct sophgo_pcie_msi_data {
	struct platform_device *pdev;
	int irq_num;

	struct irq_domain *domain;
	struct irq_chip *chip;

	DECLARE_BITMAP(irq_bitmap, MAX_IRQ_NUMBER);
	spinlock_t lock;

	void __iomem *reg_set;
	void __iomem *reg_clr;
	phys_addr_t reg_set_phys;
	phys_addr_t reg_clr_phys;

	int nr_irqs;
	struct msi_irq_data *data;

	irq_hw_number_t		plic_hwirqs[MAX_IRQ_NUMBER];
	int			plic_irqs[MAX_IRQ_NUMBER];
	struct irq_data		*plic_irq_datas[MAX_IRQ_NUMBER];
	int			pic_to_plic[MAX_IRQ_NUMBER]; // mapping from tic hwirq to plic hwirq
};

struct msi_irq_data {
	irq_hw_number_t		hwirq;
	int			irq;
	struct irq_data		*data;

	u32			address_high;
	u32			address_low;

	void __iomem		*reg_set;
	void __iomem		*reg_clr;
};

struct pcie_msi_bridge_ctrl {
	struct irq_domain *domain;
	struct irq_chip *chip;

	unsigned long *irq_bitmap;
	spinlock_t lock;

	void __iomem *reg_set;
	void __iomem *reg_clr;
	phys_addr_t reg_set_phys;
	phys_addr_t reg_clr_phys;

	int nr_irqs;
	struct msi_irq_data *data;
};

static void sophgo_pcie_msi_ack_irq(struct irq_data *d)
{
	struct sophgo_pcie_msi_data *data  = irq_data_get_irq_chip_data(d);
	int reg_off = 0;

	reg_off = d->hwirq;
	writel(0, (unsigned int *)data->reg_clr + reg_off);

	irq_chip_ack_parent(d);
}

static void sophgo_pcie_msi_mask_irq(struct irq_data *d)
{
	struct msi_irq_data *data = irq_data_get_irq_chip_data(d);
	int reg_off = 0;

	reg_off = d->hwirq;
	writel(1, (unsigned int *)data->reg_set + reg_off);

	irq_chip_mask_parent(d);
}

static void sophgo_pcie_msi_unmask_irq(struct irq_data *d)
{
	struct msi_irq_data *data = irq_data_get_irq_chip_data(d);
	int reg_off = 0;

	reg_off = d->hwirq;
	writel(0, (unsigned int *)data->reg_clr + reg_off);

	irq_chip_unmask_parent(data->data);
}

static void sophgo_pcie_msi_setup_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct msi_irq_data *data = irq_data_get_irq_chip_data(d);

	msg->address_lo = data->address_low;
	msg->address_hi = data->address_high;
	msg->data = d->hwirq % 32;
}

struct irq_chip sophgo_pcie_msi_irq_chip = {
	.name = "Sophgo PCIe MSI Bridge",
	.irq_ack = sophgo_pcie_msi_ack_irq,
	.irq_mask = sophgo_pcie_msi_mask_irq,
	.irq_unmask = sophgo_pcie_msi_unmask_irq,
	.irq_set_affinity = irq_chip_set_affinity_parent,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_compose_msi_msg = sophgo_pcie_msi_setup_msi_msg,
};

static int sophgo_pcie_msi_domain_translate(struct irq_domain *domain,
					    struct irq_fwspec *fwspec,
					    unsigned long *hwirq,
					    unsigned int *type)
{
	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;

	if (fwspec->param_count != 2)
		return -EINVAL;
	if (fwspec->param[1] >= ctrl->nr_irqs)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static int sophgo_pcie_msi_domain_alloc(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs,
					void *args)
{
	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;
	unsigned long flags;
	irq_hw_number_t hwirq;
	int i, ret = -1;

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
				    &sophgo_pcie_msi_irq_chip,
				    &ctrl->data[hwirq + i], handle_edge_irq,
				    NULL, NULL);
		ctrl->data[hwirq + i].hwirq = hwirq + i;
	}

	return 0;
}

static void sophgo_pcie_msi_domain_free(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct pcie_msi_bridge_ctrl *ctrl = domain->host_data;
	unsigned long flags;

	spin_lock_irqsave(&ctrl->lock, flags);
	bitmap_release_region(ctrl->irq_bitmap, data->hwirq, order_base_2(nr_irqs));
	spin_unlock_irqrestore(&ctrl->lock, flags);
}

static const struct irq_domain_ops sophgo_pcie_msi_domain_ops = {
	.translate = sophgo_pcie_msi_domain_translate,
	.alloc = sophgo_pcie_msi_domain_alloc,
	.free = sophgo_pcie_msi_domain_free,
};

static int sophgo_pcie_msi_init_irq(struct platform_device *pdev,
				    struct pcie_msi_bridge_ctrl *ctrl)
{
	int i;

	ctrl->nr_irqs = platform_irq_count(pdev);
	if (ctrl->nr_irqs < 0)
		return ctrl->nr_irqs;

	ctrl->data = devm_kcalloc(&pdev->dev, ctrl->nr_irqs,
				  sizeof(struct msi_irq_data), GFP_KERNEL);
	if (!ctrl->data)
		return -ENOMEM;

	for (i = 0; i < ctrl->nr_irqs; i++) {
		int irq = platform_get_irq(pdev, i);
		struct msi_irq_data *data = &ctrl->data[i];

		if (irq < 0)
			return irq;

		data->irq = irq;
		data->data = irq_get_irq_data(irq);
		data->hwirq = i;
		data->address_high = upper_32_bits(ctrl->reg_set_phys);
		data->address_low = lower_32_bits(ctrl->reg_set_phys) + 4 * (i / 32);
	}

	return 0;
}

static struct irq_domain *sophgo_pcie_msi_get_parent(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *parent;
	struct irq_domain *domain;

	if (!of_property_present(np, "interrupt-parent"))
		return ERR_PTR(-ENXIO);

	parent = of_irq_find_parent(np);
	if (!parent)
		return ERR_PTR(-ENXIO);

	domain = irq_find_host(parent);
	of_node_put(parent);
	if (!domain)
		/* domain not registered yet */
		return ERR_PTR(-EPROBE_DEFER);

	return domain;
}

static int sophgo_pcie_msi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct pcie_msi_bridge_ctrl *ctrl;
	struct resource *res;
	struct irq_domain *parent;
	int ret;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctrl);
	spin_lock_init(&ctrl->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "set");
	ctrl->reg_set = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctrl->reg_set)) {
		dev_err(&pdev->dev, "failed map set register\n");
		return PTR_ERR(ctrl->reg_clr);
	}
	ctrl->reg_set_phys = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "clr");
	ctrl->reg_clr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctrl->reg_clr)) {
		dev_err(&pdev->dev, "failed map clear register\n");
		return PTR_ERR(ctrl->reg_clr);
	}

	ret = sophgo_pcie_msi_init_irq(pdev, ctrl);
	if (ret < 0)
		return ret;

	ctrl->irq_bitmap = devm_bitmap_zalloc(dev, ctrl->nr_irqs, GFP_KERNEL);
	if (!ctrl->irq_bitmap)
		return -ENOMEM;

	parent = sophgo_pcie_msi_get_parent(pdev);
	if (IS_ERR(parent))
		return PTR_ERR(parent);

	ctrl->domain = irq_domain_create_hierarchy(parent, 0, ctrl->nr_irqs,
						   fwnode, &sophgo_pcie_msi_domain_ops,
						   ctrl);
	if (!ctrl->domain)
		return -ENOMEM;

	irq_domain_update_bus_token(ctrl->domain, DOMAIN_BUS_NEXUS);

	return 0;
}

// 	/*
// 	 * workaround to deal with IRQ conflict with TPU driver,
// 	 * skip the firt IRQ and mark it as used.
// 	 */
// 	//bitmap_allocate_region(data->irq_bitmap, 0, order_base_2(1));
// 	for (i = 0; i < data->irq_num; i++)
// 		irq_set_chained_handler_and_data(data->plic_irqs[i],
// 							sophgo_pcie_msi_irq_handler, data);

// 	irq_domain_update_bus_token(data->domain, DOMAIN_BUS_NEXUS);

static const struct of_device_id sophgo_pcie_msi_of_match[] = {
	{ .compatible = "sophgo,sg2044-pcie-msi" },
	{},
};
MODULE_DEVICE_TABLE(of, sophgo_pcie_msi_of_match);

static struct platform_driver sophgo_pcie_msi_driver = {
	.driver = {
		.name = "sophgo,pcie-intc",
		.suppress_bind_attrs = true,
		.of_match_table = of_match_ptr(sophgo_pcie_msi_of_match),
	},
	.probe = sophgo_pcie_msi_probe,
};
builtin_platform_driver(sophgo_pcie_msi_driver);
