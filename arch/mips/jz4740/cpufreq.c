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

#include <linux/cpufreq.h>

#include <linux/clk.h>
#include <asm/mach-jz4740/jz4740.h>


struct jz4740_freq_percpu_info {
	struct cpufreq_frequency_table table[12];
};

static struct clk *cclk = NULL;

static struct jz4740_freq_percpu_info jz4740_freq_table;

#if 1
#define SDRAM_TREF              15625   /* Refresh period: 4096 refresh cycles/64ms */

inline int sdram_convert(unsigned int pllin,unsigned int *sdram_freq)
{
        register unsigned int ns, tmp;

        ns = 1000000000 / pllin;
        /* Set refresh registers */
        tmp = SDRAM_TREF/ns;
        tmp = tmp/64 + 1;
        if (tmp > 0xff) tmp = 0xff;
        *sdram_freq = tmp;

        return 0;

}

#define CPM_CPCCR_CLKOEN    (1 << 30)
#define CPM_CPCCR_LDIV_BIT    16
#define CPM_CPCCR_MDIV_BIT    12
#define CPM_CPCCR_PDIV_BIT    8
#define CPM_CPCCR_HDIV_BIT    4
#define CPM_CPCCR_CDIV_BIT    0
#define CPM_CPCCR_PCS        (1 << 21)
#define CFG_EXTAL 12000000    /* EXT clock: 12 Mhz */
#define CPM_CPPCR_PLLM_BIT    23
#define CPM_CPPCR_PLLN_BIT    18
#define CPM_CPPCR_PLLOD_BIT    16
#define CPM_CPPCR_PLLEN        (1 << 8)
#define CPM_CPPCR_PLLST_BIT    0
#define PLL_WAIT_500NS (500*(__cpm_get_cclk()/1000000000))

void pll_init(unsigned int clock)
{

        register unsigned int cfcr, plcr1;
        unsigned int sdramclock = 0;
        unsigned int tmp = 0, wait = PLL_WAIT_500NS;

        int n2FR[33] = {
                0, 0, 1, 2, 3, 0, 4, 0, 5, 0, 0, 0, 6, 0, 0, 0,
                7, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0,
                9
        };
        //int div[5] = {1, 4, 4, 4, 4}; /* divisors of I:S:P:L:M */
        int div[5] = {1, 3, 3, 3, 3}; /* divisors of I:S:P:L:M */
        int nf, pllout2;

        cfcr = CPM_CPCCR_CLKOEN |
                (n2FR[div[0]] << CPM_CPCCR_CDIV_BIT) |
                (n2FR[div[1]] << CPM_CPCCR_HDIV_BIT) |
                (n2FR[div[2]] << CPM_CPCCR_PDIV_BIT) |
                (n2FR[div[3]] << CPM_CPCCR_MDIV_BIT) |
                (n2FR[div[4]] << CPM_CPCCR_LDIV_BIT);

        pllout2 = (cfcr & CPM_CPCCR_PCS) ? clock : (clock / 2);

        /* Init UHC clock */
        REG_CPM_UHCCDR = pllout2 / 48000000 - 1;

        nf = clock * 2 / CFG_EXTAL;
        plcr1 = ((nf - 2) << CPM_CPPCR_PLLM_BIT) | /* FD */
                (0 << CPM_CPPCR_PLLN_BIT) |     /* RD=0, NR=2 */
                (0 << CPM_CPPCR_PLLOD_BIT) |    /* OD=0, NO=1 */
                (0x20 << CPM_CPPCR_PLLST_BIT) | /* PLL stable time */
                CPM_CPPCR_PLLEN;                /* enable PLL */

        sdram_convert(clock,&sdramclock);
        if(sdramclock > 0)
        {
        REG_EMC_RTCOR = sdramclock;
        REG_EMC_RTCNT = sdramclock;

        }else
        {
        printk("sdram init fail!\n");
        while(1);
        }
        /* init PLL */
        //REG_CPM_CPCCR = cfcr;
        __asm__ __volatile__(
                ".set noreorder\n\t"
                ".align 5\n"
                "sw %1,0(%0)\n\t"
                "li %3,0\n\t"
                "1:\n\t"
                "bne %3,%2,1b\n\t"
                "addi %3, 1\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                ".set reorder\n\t"
                :
                : "r" (CPM_CPCCR), "r" (cfcr), "r" (wait), "r" (tmp));

        REG_CPM_LPCDR = clock / 12000000 / 2 - 1; /* pixclock */
        //REG_CPM_CPPCR = plcr1;
        __asm__ __volatile__(
                ".set noreorder\n\t"
                ".align 5\n"
                "sw %1,0(%0)\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                ".set reorder\n\t"
                :
                : "r" (CPM_CPPCR), "r" (plcr1));


}
#endif

static unsigned int jz4740_freq_get(unsigned int cpu)
{
        unsigned long ret;
	cclk = clk_get(NULL, "cclk");
        ret = clk_get_rate(cclk);
        clk_put(cclk);
        return ret / 1000;
}

static int jz4740_freq_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	struct cpufreq_frequency_table *table =	&jz4740_freq_table.table[0];
	unsigned int new_index = 0;

	if (cpufreq_frequency_table_target(policy,
					   &jz4740_freq_table.table[0],
					   target_freq, relation, &new_index))
	{
		return -EINVAL;
	}

	printk(KERN_INFO "%s: target_freq %d new_index %d\n", __FUNCTION__, target_freq, new_index);

	if (table[new_index].frequency == jz4740_freq_get(policy->cpu))
	        return 0;

	cclk = clk_get(NULL, "cclk");
	printk(KERN_INFO "%s: cclk %p, setting to %d\n", __FUNCTION__, cclk, table[new_index].frequency);
	if (cclk) {
	        //clk_set_rate(cclk, table[new_index].frequency);
	        pll_init(table[new_index].frequency * 1000);
	        clk_put(cclk);
	}
	//jz4740_set_cpu_divider_index(policy->cpu, new_index);

	return 0;
}

static int jz4740_freq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,
					      &jz4740_freq_table.table[0]);
}

static int __init jz4740_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table =	&jz4740_freq_table.table[0];
	unsigned int MAX_FREQ;
	int i;

	printk(KERN_INFO "Jz4740 cpufreq driver\n");

	if (policy->cpu != 0)
		return -EINVAL;

	policy->cur = 336000; /* in kHz. Current and max frequency is determined by u-boot */
	MAX_FREQ = 432000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

	policy->cpuinfo.min_freq = 192000;
	policy->cpuinfo.max_freq = MAX_FREQ;
	policy->cpuinfo.transition_latency = 100000; /* in 10^(-9) s = nanoseconds */

	for (i = 0; i < 11; i++) {
	        table[i].index = i;
	        table[i].frequency = policy->cpuinfo.min_freq + i * 24000;
	}
	table[i].index = i;
	table[i].frequency = CPUFREQ_TABLE_END;
	
#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(table, policy->cpu); /* for showing /sys/devices/system/cpu/cpuX/cpufreq/stats/ */
#endif

	return  cpufreq_frequency_table_cpuinfo(policy, table);
}

static struct cpufreq_driver cpufreq_jz4740_driver = {
//      .flags          = CPUFREQ_STICKY,
        .init           = jz4740_cpufreq_driver_init,
        .verify         = jz4740_freq_verify,
        .target         = jz4740_freq_target,
        .get            = jz4740_freq_get,
        .name           = "jz4740",
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
