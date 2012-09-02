/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform timer support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_MACH_JZ4740_TIMER
#define __ASM_MACH_JZ4740_TIMER

#define JZ_TIMER_CTRL_PWM_ABBRUPT_SHUTDOWN	BIT(9)
#define JZ_TIMER_CTRL_PWM_ACTIVE_LOW		BIT(8)
#define JZ_TIMER_CTRL_PWM_ENABLE		BIT(7)
#define JZ_TIMER_CTRL_PRESCALE_MASK		0x1c
#define JZ_TIMER_CTRL_PRESCALE_OFFSET		0x3
#define JZ_TIMER_CTRL_PRESCALE_1		(0 << 3)
#define JZ_TIMER_CTRL_PRESCALE_4		(1 << 3)
#define JZ_TIMER_CTRL_PRESCALE_16		(2 << 3)
#define JZ_TIMER_CTRL_PRESCALE_64		(3 << 3)
#define JZ_TIMER_CTRL_PRESCALE_256		(4 << 3)
#define JZ_TIMER_CTRL_PRESCALE_1024		(5 << 3)

#define JZ_TIMER_CTRL_PRESCALER(x) ((x) << JZ_TIMER_CTRL_PRESCALE_OFFSET)

#define JZ_TIMER_CTRL_SRC_EXT		BIT(2)
#define JZ_TIMER_CTRL_SRC_RTC		BIT(1)
#define JZ_TIMER_CTRL_SRC_PCLK		BIT(0)

void __init jz4740_timer_init(void);

void jz4740_timer_stop(unsigned int timer);
void jz4740_timer_start(unsigned int timer);
bool jz4740_timer_is_enabled(unsigned int timer);
void jz4740_timer_enable(unsigned int timer);
void jz4740_timer_disable(unsigned int timer);
void jz4740_timer_set_period(unsigned int timer, uint16_t period);
void jz4740_timer_set_duty(unsigned int timer, uint16_t duty);
void jz4740_timer_set_count(unsigned int timer, uint16_t count);
uint16_t jz4740_timer_get_count(unsigned int timer);
void jz4740_timer_ack_full(unsigned int timer);
void jz4740_timer_irq_full_enable(unsigned int timer);
void jz4740_timer_irq_full_disable(unsigned int timer);
uint16_t jz4740_timer_get_ctrl(unsigned int timer);
void jz4740_timer_set_ctrl(unsigned int timer, uint16_t ctrl);

void jz4740_timer_enable_watchdog(void);
void jz4740_timer_disable_watchdog(void);

#endif
