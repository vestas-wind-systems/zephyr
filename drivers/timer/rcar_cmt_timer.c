/*
 * Copyright (c) 2020 IoT.bzh <julien.massot@iot.bzh>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <drivers/timer/system_timer.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_clock_control.h>

#define DT_DRV_COMPAT rcar_cmt

#define TIMER_IRQ		DT_INST_IRQN(0)
#define TIMER_BASE_ADDR		DT_INST_REG_ADDR(0)
#define TIMER_CLOCK_FREQUENCY	DT_INST_PROP(0, clock_frequency)

#define CLOCK_CONTROLLER        DT_INST_CLOCKS_LABEL(0)
#define CLOCK_SUBSYS            DT_INST_CLOCKS_CELL(0, module)

#define CYCLES_PER_SEC		TIMER_CLOCK_FREQUENCY
#define CYCLES_PER_MS           CYCLES_PER_SEC / 1000
#define CYCLES_PER_TICK		(CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static struct rcar_cpg_clk mod_clk = {
	.module = DT_INST_CLOCKS_CELL(0, module),
	.domain = DT_INST_CLOCKS_CELL(0, domain),
};

BUILD_ASSERT(CYCLES_PER_TICK > 1,
	     "CYCLES_PER_TICK must be greater than 1");

#define CMCOR0_OFFSET	0x018	/* Compare match timer constant register 0 */
#define CMCNT0_OFFSET	0x014	/* Compare match timer counter 0 */
#define CMCSR0_OFFSET	0x010	/* Compare match timer control/status register 0 */

#define CMCLKE		0xB00	/* CLK enable register */
#define CLKEN		BIT(5)

#define CMSTR0_OFFSET	0x000	/* Compare match timer start register 0 */
#define START_BIT	BIT(0)

#define	CSR_CLK_DIV_1			0x00000007
#define	CSR_ENABLE_COUNTER_IN_DEBUG	BIT(3)
#define	CSR_ENABLE_INTERRUPT		BIT(5)
#define	CSR_FREE_RUN			BIT(8)
#define	CSR_WRITE_FLAG			BIT(13)
#define	CSR_OVERFLOW_FLAG		BIT(14)
#define	CSR_MATCH_FLAG			BIT(15)

static uint32_t cycle_count;

static void cmt_isr(void *arg)
{
	ARG_UNUSED(arg);
	uint32_t reg_val;

	/* Read the interrupt status, then write it back to clear the interrupt */
	reg_val = sys_read32(TIMER_BASE_ADDR + CMCSR0_OFFSET);
	reg_val &= ~CSR_MATCH_FLAG;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMCSR0_OFFSET);

	cycle_count += CYCLES_PER_TICK;

	/* Announce to the kernel*/
	z_clock_announce(1);
}

int z_clock_driver_init(const struct device *device)
{
	const struct device *clk;
	uint32_t reg_val;
	int i, ret;

	clk = device_get_binding(CLOCK_CONTROLLER);
	if (!clk) {
		return -ENODEV;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t *) &mod_clk);
	if (ret < 0) {
		return ret;
	}

	/* Clock Enable */
	reg_val = sys_read32(TIMER_BASE_ADDR + CMCLKE);
	reg_val |= CLKEN;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMCLKE);

	/* Stop the timer */
	reg_val = sys_read32(TIMER_BASE_ADDR + CMSTR0_OFFSET);
	reg_val &= ~START_BIT;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMSTR0_OFFSET);

	/* Set the timer as 32-bit, with RCLK/1 clock */
	reg_val = CSR_FREE_RUN;
	reg_val |= CSR_ENABLE_INTERRUPT;
	reg_val |= CSR_ENABLE_COUNTER_IN_DEBUG;
	reg_val |= CSR_CLK_DIV_1;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMCSR0_OFFSET);

	sys_write32(CYCLES_PER_TICK, TIMER_BASE_ADDR + CMCOR0_OFFSET);

	/* Reset the counter, check WRFLG first */
	while (sys_read32(TIMER_BASE_ADDR + CMCSR0_OFFSET) & CSR_WRITE_FLAG)
		;
	sys_write32(0, TIMER_BASE_ADDR + CMCNT0_OFFSET);

	for (i = 0; i < 100; i++) {
		if (!sys_read32(TIMER_BASE_ADDR + CMCNT0_OFFSET))
			break;
	}

	__ASSERT(sys_read32(TIMER_BASE_ADDR + CMCNT0_OFFSET) == 0,
			"Fail to clear CMCNT0 register");

	/* Connect timer interrupt */
	IRQ_CONNECT(TIMER_IRQ, 0, cmt_isr, 0, 0);
	irq_enable(TIMER_IRQ);

	/* Start the timer */
	reg_val = sys_read32(TIMER_BASE_ADDR + CMSTR0_OFFSET);
	reg_val |= START_BIT;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMSTR0_OFFSET);

	return 0;
}

uint32_t z_clock_elapsed(void)
{
	/* Always return 0 for tickful operation */
	return 0;
}

uint32_t z_timer_cycle_get_32(void)
{
	return cycle_count;
}
