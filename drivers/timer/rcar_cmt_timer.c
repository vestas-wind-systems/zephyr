/*
 * Copyright (c) 2020 IoT.bzh <julien.massot@iot.bzh>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <drivers/timer/system_timer.h>

#define DT_DRV_COMPAT rcar_cmt

#define TIMER_IRQ		DT_INST_IRQN(0)
#define TIMER_BASE_ADDR		DT_INST_REG_ADDR(0)
#define TIMER_CLOCK_FREQUENCY	DT_INST_PROP(0, clock_frequency)

#define CYCLES_PER_SEC		TIMER_CLOCK_FREQUENCY
#define CYCLES_PER_MS           CYCLES_PER_SEC / 1000
#define CYCLES_PER_TICK		(CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#ifdef CONFIG_TICKLESS_KERNEL
/* FIXME: we need to protect against concurent access of these variable
 * They can be set in thread context as well as read in interrupt context. 
 */
static uint32_t last_cycles;
static uint32_t match_value;
#define TICK_THRESHOLD 2

/* Maximum number of ticks. */
#define MAX_TICKS (UINT32_MAX / CYCLES_PER_TICK - 2)

BUILD_ASSERT(CYCLES_PER_TICK > TICK_THRESHOLD,
	     "CYCLES_PER_TICK must be greater than TICK_THRESHOLD for "
	     "tickless mode");

#else /* !CONFIG_TICKLESS_KERNEL */

BUILD_ASSERT(CYCLES_PER_TICK > 1,
	     "CYCLES_PER_TICK must be greater than 1 for ticking mode");

#endif

/* CMT0 Module Stop */
#define RMSTPCR3		0xE615011CU
#define MSTP303		BIT(3)

#define CMCOR0_OFFSET	0x018	/* Compare match timer constant register 0 */
#define CMCNT0_OFFSET	0x014	/* Compare match timer counter 0 */
#define CMCSR0_OFFSET	0x010	/* Compare match timer control/status register 0 */

#define CMCLKE		0xB00	/* CLK enable register */
#define CLKEN		BIT(5)

#define CMSTR0_OFFSET	0x000	/* Compare match timer start register 0 */
#define  START_BIT	BIT(0)

// CMSCSR, CMCSR0
#define	CSR_CLK_DIV_1			0x00000007
#define	CSR_ENABLE_COUNTER_IN_DEBUG	BIT(3)
#define	CSR_ENABLE_INTERRUPT		BIT(5)
#define	CSR_FREE_RUN			BIT(8)
#define	CSR_WRITE_FLAG			BIT(13)
#define	CSR_OVERFLOW_FLAG		BIT(14)
#define	CSR_MATCH_FLAG			BIT(15)
#define	MASK_MATCH_OVERFLOW_FLAG	(CSR_MATCH_FLAG | CSR_OVERFLOW_FLAG)

static uint32_t read_count()
{
	/* Read current counter value */
	return sys_read32(TIMER_BASE_ADDR + CMCNT0_OFFSET);
}

#ifdef CONFIG_TICKLESS_KERNEL
static uint32_t counter_overflowed()
{
	uint32_t val;

	val = sys_read32(TIMER_BASE_ADDR + CMCSR0_OFFSET);

	return val & CSR_OVERFLOW_FLAG;
}
#endif

static void update_match(uint32_t cycles, uint32_t match)
{
#ifdef CONFIG_TICKLESS_KERNEL
	uint32_t delta = match - cycles;
	/* Ensure that the match value meets the minimum timing requirements */
	if (delta < TICK_THRESHOLD) {
		match += CYCLES_PER_TICK;
	}

	last_cycles = cycles;
	delta = match - cycles;
	match_value = delta;
#endif
	/* Write counter match value for interrupt generation */
	sys_write32(match, TIMER_BASE_ADDR + CMCOR0_OFFSET);
}

static void cmt_isr(void *arg)
{
	uint32_t cycles;
	uint32_t ticks;

	ARG_UNUSED(arg);
	uint32_t reg_val;
	/* Read the interrupt status, then write it back to clear the interrupt */
	reg_val = sys_read32(TIMER_BASE_ADDR + CMCSR0_OFFSET);
	reg_val &= ~MASK_MATCH_OVERFLOW_FLAG;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMCSR0_OFFSET);

	/* Read counter value */
	cycles = read_count();

#ifdef CONFIG_TICKLESS_KERNEL
	/* The counter has overflowed */
	ticks = ( match_value + cycles ) / CYCLES_PER_TICK;
#else
	/* Update counter match value for the next interrupt */
	update_match(cycles, cycles + CYCLES_PER_TICK);

	/* Advance tick count by 1 */
	ticks = 1;
#endif
	/* Announce to the kernel*/
	z_clock_announce(ticks);
}

int z_clock_driver_init(const struct device *device)
{
	uint32_t reg_val;
	int i;
	/* FIXME: Enable only CMT0 module clock */
	reg_val = sys_read32(RMSTPCR3);
	reg_val &= ~MSTP303;
	sys_write32(reg_val, RMSTPCR3);

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
	sys_write32(reg_val, (TIMER_BASE_ADDR + CMCSR0_OFFSET));

	reg_val = IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
			0xFFFFFFFF : CYCLES_PER_TICK;
	sys_write32(reg_val, TIMER_BASE_ADDR + CMCOR0_OFFSET);

	/* Reset the counter, check WRFLG first */
	while (sys_read32(TIMER_BASE_ADDR + CMCSR0_OFFSET) & CSR_WRITE_FLAG)
		;
	sys_write32(0, (TIMER_BASE_ADDR + CMCNT0_OFFSET));

	for (i = 0; i < 100; i++) {
		if (!sys_read32(TIMER_BASE_ADDR + CMCNT0_OFFSET))
			break;
	}
#ifdef CONFIG_TICKLESS_KERNEL
	last_cycles = 0;
#endif
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

void z_clock_set_timeout(int32_t ticks, bool idle)
{
#ifdef CONFIG_TICKLESS_KERNEL
	uint32_t cycles;
	uint32_t next_cycles;

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = MAX(MIN(ticks - 1, (int32_t)MAX_TICKS), 0);

	/* Read counter value */
	cycles = read_count();

	next_cycles = cycles + ((uint32_t)ticks * CYCLES_PER_TICK);

	/* Set match value for the next interrupt */
	update_match(cycles, next_cycles);
#endif
}

uint32_t z_clock_elapsed(void)
{
#ifdef CONFIG_TICKLESS_KERNEL
	uint32_t cycles;

	/* Read counter value */
	cycles = read_count();
	if (counter_overflowed())
		cycles += match_value;
	else
		cycles -= last_cycles;
	return cycles / CYCLES_PER_TICK;
#else
	/* Always return 0 for tickful operation */
	return 0;
#endif
}

uint32_t z_timer_cycle_get_32(void)
{	
	uint32_t cycles;

	cycles = read_count();
#ifdef CONFIG_TICKLESS_KERNEL
	if (counter_overflowed())
		cycles += match_value;
	else
		cycles -= last_cycles;
#endif
	return cycles;
}
