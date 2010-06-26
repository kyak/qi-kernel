/*
 * linux/arch/mips/jz4740/cpufreq.c
 *
 * cpufreq driver for JZ4740
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Copyright (c) 2010       Ulrich Hecht <ulrich.hecht@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>

#include <linux/cpufreq.h>

#include <linux/clk.h>
#include <asm/mach-jz4740/base.h>

#include "clock.h"

#define DEBUG_CPUFREQ

#ifdef DEBUG_CPUFREQ
#define dprintk(X...) printk(KERN_INFO X)
#else
#define dprintk(X...) do { } while(0)
#endif

struct jz4740_freq_percpu_info {
	struct cpufreq_frequency_table table[12];
};

static struct clk *pll;
static struct clk *cclk;

static struct jz4740_freq_percpu_info jz4740_freq_table;

static struct cpufreq_driver cpufreq_jz4740_driver;

static unsigned int jz4740_freq_get(unsigned int cpu)
{
	return clk_get_rate(cclk) / 1000;
}

static int jz4740_freq_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	struct cpufreq_frequency_table *table =	&jz4740_freq_table.table[0];
	struct cpufreq_freqs freqs;
	unsigned int new_index = 0;
	int ret;

	if (cpufreq_frequency_table_target(policy,
					   &jz4740_freq_table.table[0],
					   target_freq, relation, &new_index))
		return -EINVAL;

	dprintk(KERN_INFO "%s: target_freq %d new_index %d\n", __FUNCTION__,
	        target_freq, new_index);

	freqs = (struct cpufreq_freqs) {
		.old = jz4740_freq_get(policy->cpu),
		.new = table[new_index].frequency,
		.cpu = policy->cpu,
		.flags = cpufreq_jz4740_driver.flags,
	};

	/* Is there anything to do anyway? */
	if (freqs.new == freqs.old)
		return 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	dprintk(KERN_INFO "%s: cclk %p, setting from %d to %d\n",
		__FUNCTION__, cclk, freqs.old, freqs.new);
	ret = clk_set_rate(pll, freqs.new * 1000);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int jz4740_freq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,
					      &jz4740_freq_table.table[0]);
}

static int __init jz4740_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table =	&jz4740_freq_table.table[0];
	int ret;
	int i;

	dprintk(KERN_INFO "Jz4740 cpufreq driver\n");

	if (policy->cpu != 0)
		return -EINVAL;

	pll = clk_get(NULL, "pll");
	if (IS_ERR(pll)) {
		ret = PTR_ERR(pll);
		goto err_exit;
	}

	cclk = clk_get(NULL, "cclk");
	if (IS_ERR(cclk)) {
		ret = PTR_ERR(cclk);
		goto err_clk_put_pll;
	}

	/* FIXME: These hardcoded numbers should be board-specific, I guess. */
	policy->cur = 336000; /* in kHz */
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

	policy->cpuinfo.min_freq = 192000;
	policy->cpuinfo.transition_latency = 100000; /* in nanoseconds */

	/* 11 steps for a maximum of 432 MHz */
	for (i = 0; i < 11; i++) {
		table[i].index = i;
		table[i].frequency = policy->cpuinfo.min_freq + i * 24000;
	}
	policy->cpuinfo.max_freq = table[i-1].frequency;
	table[i].index = i;
	table[i].frequency = CPUFREQ_TABLE_END;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	/* for showing /sys/devices/system/cpu/cpuX/cpufreq/stats/ */
	cpufreq_frequency_table_get_attr(table, policy->cpu);
#endif

	return cpufreq_frequency_table_cpuinfo(policy, table);

err_clk_put_pll:
	clk_put(pll);
err_exit:
	return ret;
}

static struct cpufreq_driver cpufreq_jz4740_driver = {
	.init	= jz4740_cpufreq_driver_init,
	.verify	= jz4740_freq_verify,
	.target	= jz4740_freq_target,
	.get	= jz4740_freq_get,
	.name	= "jz4740",
};

static int __init jz4740_cpufreq_init(void)
{
	return cpufreq_register_driver(&cpufreq_jz4740_driver);
}

static void __exit jz4740_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&cpufreq_jz4740_driver);
}

module_init(jz4740_cpufreq_init);
module_exit(jz4740_cpufreq_exit);

MODULE_AUTHOR("Ulrich Hecht <ulrich.hecht@gmail.com>");
MODULE_DESCRIPTION("cpufreq driver for Jz4740");
MODULE_LICENSE("GPL");
