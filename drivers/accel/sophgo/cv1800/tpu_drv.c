
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <drm/drm_accel.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>

#include <uapi/drm/cv1800_tpu_accel.h>

#define CV1800_TPU_NAME		"cv1800-tpu"
#define CV1800_TPU_DESC		"Sophgo CV1800 TPU Accelerators"

struct cv1800_tpu_handle {
	;
};

struct cv1800_tpu_drm_device {
	struct drm_device	drm;
};

static int cv1800_tpu_submit_ioctl(struct drm_device *dev,
				   void *data, struct drm_file *file)
{
	return 0;
}

static int cv1800_tpu_submit_wait(struct drm_device *dev,
				  void *data, struct drm_file *file)
{
	return 0;
}

static int cv1800_tpu_flush_ioctl(struct drm_device *dev,
				  void *data, struct drm_file *file)
{
	return 0;
}

static int cv1800_tpu_invalid_ioctl(struct drm_device *dev,
				    void *data, struct drm_file *file)
{
	return 0;
}

DEFINE_DRM_ACCEL_FOPS(cv1800_tpu_accel_fops);

static const struct drm_ioctl_desc cv1800_tpu_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(CV1800_TPU_SUBMIT, cv1800_tpu_submit_ioctl, 0),
	DRM_IOCTL_DEF_DRV(CV1800_TPU_WAIT, cv1800_tpu_submit_wait, 0),
	DRM_IOCTL_DEF_DRV(CV1800_TPU_FLUSH, cv1800_tpu_flush_ioctl, 0),
	DRM_IOCTL_DEF_DRV(CV1800_TPU_INVLD, cv1800_tpu_invalid_ioctl, 0),
};

static int cv1800_tpu_open(struct drm_device *dev, struct drm_file *file)
{
	return 0;
}

static void cv1800_tpu_postclose(struct drm_device *dev, struct drm_file *file)
{
}

static const struct drm_driver cv1800_tpu_accel_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_COMPUTE_ACCEL,

	.open			= cv1800_tpu_open,
	.postclose		= cv1800_tpu_postclose,

	.name			= CV1800_TPU_NAME,
	.desc			= CV1800_TPU_DESC,
	.date			= "20240613",

	.fops			= &cv1800_tpu_accel_fops,
	.ioctls			= cv1800_tpu_drm_ioctls,
	.num_ioctls		= ARRAY_SIZE(cv1800_tpu_drm_ioctls),
};

static int cv1800_tpu_probe(struct platform_device *pdev)
{
	struct cv1800_tpu_drm_device *tpu_drm;
	struct drm_device *drm_dev;
	int ret;

	tpu_drm = devm_drm_dev_alloc(&pdev->dev, &cv1800_tpu_accel_driver,
				     struct cv1800_tpu_drm_device, drm);
	if (IS_ERR(tpu_drm))
		return PTR_ERR(tpu_drm);

	/* TODO: check dev related struct */
	drm_dev = &tpu_drm->drm;
	drm_dev->dev_private = NULL;

	ret = drm_dev_register(&tpu_drm->drm, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register DRM device: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, tpu_drm);

	return 0;
}

static void cv1800_tpu_remove(struct platform_device *pdev)
{
	struct cv1800_tpu_drm_device *tpu_drm = platform_get_drvdata(pdev);
	struct drm_device *drm_dev = &tpu_drm->drm;

	drm_dev_unplug(drm_dev);
	/* TODO: other cleanup */
}

static const struct of_device_id cv1800_tpu_match[] = {
	{ .compatible = "sophgo,cv1800-tpu" },
	{},
};
MODULE_DEVICE_TABLE(of, cv1800_tpu_match);

static struct platform_driver cv1800_tpu_driver = {
	.probe = cv1800_tpu_probe,
	.remove_new = cv1800_tpu_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cvi-tpu",
		// .pm = &cv1800_tpu_pm_ops,
		.of_match_table = cv1800_tpu_match,
	},
};
module_platform_driver(cv1800_tpu_driver);

MODULE_AUTHOR("Inochi Amaoto <inochiama@outlook.com>");
MODULE_DESCRIPTION("Sophgo CV18XX/SG200X SoC TPU driver");
MODULE_LICENSE("GPL");
