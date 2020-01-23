// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/char/hw_random/aspeed-rng.c
 *
 * Copyright (C) 2020 Oscar A. Perez <linux@neuralgames.com>
 *
 * Derived from drivers/char/hw_random/timeriomem-rng.c
 *   Copyright (C) 2009 Alexander Clouter <alex@digriz.org.uk>
 *
 *   Derived from drivers/char/hw_random/omap-rng.c
 *     Copyright 2005 (c) MontaVista Software, Inc.
 *     Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * Overview:
 *   This driver is meant for SOCs like the AST2400/AST2500/AST2600 from
 *   AspeedTech that integrate non-deterministic Random Number Generators
 *   based on Ring Oscillators (a.k.a. RO-based HRNG).
 *   According to AspeedTech, the random number generators on these SOCs
 *   contain four free-run ROs that can be configured in eight different
 *   modes. The modes are just logic combinations of these four ROs that
 *   together generate a stream of random bits. These random bits are read
 *   from a 32bit data register and, new random data becomes available on
 *   this 32bit data register every one microsecond.
 *
 *   All the platform has to do is to provide to the driver the 'quality'
 *   entropy value, the 'mode' in which the combining ROs will generate
 *   the stream of random bits and, the 'period' value that is used as a
 *   wait-time between reads from the 32bit data register.
 *
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>

#define HRNG_CONTROL_REG_OFFSET		0x00
#define HRNG_OUTPUT_DATA_OFFSET		0x04
#define HRNG_CONTROL_ENABLE		BIT(0)
#define HRNG_CONTROL_MODE_MASK		GENMASK(3, 1)
#define HRNG_CONTROL_MODE_SHIFT		1

struct aspeed_hrng_data {
	void __iomem            *address;

	/* measures in usecs */
	unsigned int            period;

	/* bits of entropy per 1024 bits read */
	unsigned int            quality;
};

struct aspeed_hrng_private {
	struct device		*dev;
	void __iomem		*io_base;
	ktime_t			period;
	unsigned int		present:1;
	unsigned int		ro_mode:7;

	struct hrtimer		timer;
	struct completion	completion;

	struct hwrng		rng_ops;
};

static int aspeed_hrng_read(struct hwrng *hwrng, void *data,
			    size_t max, bool wait)
{
	struct aspeed_hrng_private
		*priv =	container_of(hwrng,
				     struct aspeed_hrng_private,
				     rng_ops);
	int retval = 0;
	int period_us = ktime_to_us(priv->period);

	/*
	 * There may not have been enough time for new data to be generated
	 * since the last request.  If the caller doesn't want to wait, let them
	 * bail out.  Otherwise, wait for the completion.  If the new data has
	 * already been generated, the completion should already be available.
	 */
	if (!wait && !priv->present)
		return 0;

	wait_for_completion(&priv->completion);

	do {
		/*
		 * After the first read, all additional reads will need to wait
		 * for the RNG to generate new data.  Since the period can have
		 * a wide range of values (1us to 1s have been observed), allow
		 * for 1% tolerance in the sleep time rather than a fixed value.
		 */
		if (retval > 0)
			usleep_range(period_us,
				     period_us + min(1, period_us / 100));

		*(u32 *)data = readl(priv->io_base + HRNG_OUTPUT_DATA_OFFSET);

		retval += sizeof(u32);
		data += sizeof(u32);
		max -= sizeof(u32);
	} while (wait && max >= sizeof(u32));

	/*
	 * Block any new callers until the RNG has had time to generate new
	 * data.
	 */
	priv->present = 0;
	reinit_completion(&priv->completion);
	hrtimer_forward_now(&priv->timer, priv->period);
	hrtimer_restart(&priv->timer);

	return retval;
}

static enum hrtimer_restart aspeed_hrng_trigger(struct hrtimer *timer)
{
	struct aspeed_hrng_private *priv =
			container_of(timer, struct aspeed_hrng_private, timer);

	priv->present = 1;
	complete(&priv->completion);

	return HRTIMER_NORESTART;
}

static void aspeed_hrng_enable(struct aspeed_hrng_private *priv)
{
	u32 regval;

	regval = readl(priv->io_base + HRNG_CONTROL_REG_OFFSET);
	regval &= ~HRNG_CONTROL_ENABLE;
	writel(regval, priv->io_base + HRNG_CONTROL_REG_OFFSET);
}

static void aspeed_hrng_disable(struct aspeed_hrng_private *priv)
{
	u32 regval;

	regval = readl(priv->io_base + HRNG_CONTROL_REG_OFFSET);
	regval |= HRNG_CONTROL_ENABLE;
	writel(regval, priv->io_base + HRNG_CONTROL_REG_OFFSET);
}

static void aspeed_hrng_set_mode(struct aspeed_hrng_private *priv)
{
	u32 regval;

	aspeed_hrng_disable(priv);
	regval = readl(priv->io_base + HRNG_CONTROL_REG_OFFSET);
	regval &= ~HRNG_CONTROL_MODE_MASK;
	regval |= (priv->ro_mode << HRNG_CONTROL_MODE_SHIFT) &
		  HRNG_CONTROL_MODE_MASK;
	writel(regval, priv->io_base + HRNG_CONTROL_REG_OFFSET);
	aspeed_hrng_enable(priv);
}

static int aspeed_hrng_probe(struct platform_device *pdev)
{
	struct aspeed_hrng_data *pdata = pdev->dev.platform_data;
	struct aspeed_hrng_private *priv;
	struct resource *res;
	int err = 0;
	int period;

	if (!pdev->dev.of_node && !pdata) {
		dev_err(&pdev->dev, "aspeed_hrng_data is missing\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (res->start % 4 != 0 || resource_size(res) != 8) {
		dev_err(&pdev->dev,
			"address space must be eight bytes wide and 32-bit aligned\n");
		return -EINVAL;
	}

	/* Allocate memory for the device structure (and zero it) */
	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct aspeed_hrng_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->dev = &pdev->dev;

	if (pdev->dev.of_node) {
		int i;

		if (!of_property_read_u32(pdev->dev.of_node, "period", &i)) {
			period = i;
		} else {
			dev_err(&pdev->dev, "missing period property\n");
			return -EINVAL;
		}

		if (!of_property_read_u32(pdev->dev.of_node, "quality", &i)) {
			priv->rng_ops.quality = i;
		} else {
			dev_info(&pdev->dev,
				 "missing quality property. Using default value of 0\n");
			priv->rng_ops.quality = 0;
		}

		if (!of_property_read_u32(pdev->dev.of_node, "mode", &i)) {
			priv->ro_mode = (i & 0x7);
		} else {
			dev_info(&pdev->dev,
				 "missing mode property. Using default mode 0x7\n");
			priv->ro_mode = 0x7;
		}
	} else {
		period = pdata->period;
		priv->rng_ops.quality = pdata->quality;
		priv->ro_mode = 0x7; /* default mode for the Ring Oscillators */
	}

	priv->period = ns_to_ktime(period * NSEC_PER_USEC);
	init_completion(&priv->completion);
	hrtimer_init(&priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	priv->timer.function = aspeed_hrng_trigger;

	priv->rng_ops.name = dev_name(&pdev->dev);
	priv->rng_ops.read = aspeed_hrng_read;

	priv->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->io_base))
		return PTR_ERR(priv->io_base);

	/* By default, RO RNG is set to mode 7 and RNG is enabled */
	dev_dbg(&pdev->dev, "setting RO RNG to mode %u\n", priv->ro_mode);
	aspeed_hrng_set_mode(priv);

	/* Assume random data is already available. */
	priv->present = 1;
	complete(&priv->completion);

	err = hwrng_register(&priv->rng_ops);
	if (err) {
		dev_err(&pdev->dev, "problem registering\n");
		return err;
	}

	dev_info(&pdev->dev, "RO-based RNG registered: mode %u @ %dus\n",
		 priv->ro_mode, period);

	return 0;
}

static int aspeed_hrng_remove(struct platform_device *pdev)
{
	struct aspeed_hrng_private *priv = platform_get_drvdata(pdev);

	aspeed_hrng_disable(priv);
	hwrng_unregister(&priv->rng_ops);
	hrtimer_cancel(&priv->timer);

	return 0;
}

static const struct of_device_id aspeed_hrng_match[] = {
	{ .compatible = "aspeed,ast2400-rng" },
	{ .compatible = "aspeed,ast2500-rng" },
	{ .compatible = "aspeed,ast2600-rng" },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_hrng_match);

static struct platform_driver aspeed_hrng_driver = {
	.driver = {
		.name		= "aspeed-rng",
		.of_match_table	= aspeed_hrng_match,
	},
	.probe		= aspeed_hrng_probe,
	.remove		= aspeed_hrng_remove,
};

module_platform_driver(aspeed_hrng_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Oscar A Perez <oscar.perez@ztsystems.com>");
MODULE_DESCRIPTION("Aspeed Ring Oscillator Random Number Generator driver");
