/*
 * Hello World driver for tiny4412
 *
 * Copyright (c) 2017
 * Author: SY <1530454315@qq.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. 
 *
 */


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>

static int hello_world_probe(struct platform_device *pdev)
{
	void *platdata = NULL;
	int count = 0;

	pr_notice("[TINY4412] %s\n", __func__);

	platdata = dev_get_platdata(&pdev->dev);
	pr_notice("platdata = %p\n", platdata);
	pr_notice("device_tree = %p\n", pdev->dev.of_node);
	count = of_get_child_count(pdev->dev.of_node);
	pr_notice("child_count = %d\n", count);

	return 0;
}

static int hello_world_remove(struct platform_device *pdev)
{

	pr_notice("[TINY4412] %s\n", __func__);
	return 0;
}

static void hello_world_shutdown(struct platform_device *pdev)
{

	pr_notice("[TINY4412] %s\n", __func__);
	return ;
}

static int hello_world_suspend(struct platform_device *pdev, pm_message_t state)
{

	pr_notice("[TINY4412] %s\n", __func__);
	return 0;
}

static int hello_world_resume(struct platform_device *pdev)
{

	pr_notice("[TINY4412] %s\n", __func__);
	return 0;
}

static const struct of_device_id tiny4412_hello_world_dt_match[] = {
	{ .compatible = "tiny4412, hello_world" },
	{},
};

static struct platform_driver tiny4412_hello_world_driver = {
	.probe		= hello_world_probe,
	.remove		= hello_world_remove,
	.shutdown   = hello_world_shutdown,
	.suspend    = hello_world_suspend,
	.resume     = hello_world_resume, 
	.driver		= {
		.name	= "hello world",
		.of_match_table = tiny4412_hello_world_dt_match,
	},
};

module_platform_driver(tiny4412_hello_world_driver);

MODULE_AUTHOR("SY <1530454315@qq.com>");
MODULE_DESCRIPTION("TINY4412 Hello World driver");
MODULE_LICENSE("GPL v2");

