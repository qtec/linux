/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2016 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>

static int qtec_clk_probe(struct platform_device *pdev){
	struct device_node *node = pdev->dev.of_node;
	struct clk *clk;
	const char *clk_name = node->name;
	u32 rate;
	u32 accuracy = 0;

	if (of_property_read_u32(node, "clock-frequency", &rate))
		return -EINVAL;

	of_property_read_u32(node, "clock-accuracy", &accuracy);

	of_property_read_string(node, "clock-output-names", &clk_name);

	clk = clk_register_fixed_rate_with_accuracy(NULL, clk_name, NULL,
						    0, rate, accuracy);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	of_clk_add_provider(node, of_clk_src_simple_get, clk);
	platform_set_drvdata(pdev, clk);
	return 0;
}

static int __exit qtec_clk_remove(struct platform_device *pdev){
	struct clk *clk = platform_get_drvdata(pdev);

	if (clk)
		clk_unregister_fixed_rate(clk);

	return 0;
}

static struct of_device_id qtec_clk_of_match[] = {
	{ .compatible = "qtec,fixed-clock"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_clk_of_match);

static struct platform_driver qtec_clk_plat_driver = {
	.probe		= qtec_clk_probe,
	.remove		= qtec_clk_remove,
	.driver		={
			.name = "qtec_clk",
			.owner = THIS_MODULE,
			.of_match_table	= qtec_clk_of_match,
	},
};

module_platform_driver(qtec_clk_plat_driver);

MODULE_DESCRIPTION("Qtec fixed clock dummy driver");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
