/*
 *  linux/include/asm-mips/mach-jz4740/board-a320.h
 *
 *  JZ4740-based A320 board definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *  Copyright (C) 2009        Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 *  Author:   <lhhuang@ingenic.cn>
 *  Modified: <iggarpe@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4740_A320_H__
#define __ASM_JZ4740_A320_H__

/*======================================================================
 * Installed memory
 */
#define MEMSIZE			0x02000000

/*======================================================================
 * GPIO
 */
#define GPIO_SD_CD		JZ_GPIO_PORTB(29)
#define GPIO_CHARG_STAT_N	JZ_GPIO_PORTB(30)
#define GPIO_USB_DETE		JZ_GPIO_PORTD(28)
#define GPIO_SND_MUTE_N		JZ_GPIO_PORTC(27)
#define GPIO_SND_UNKNOWN	JZ_GPIO_PORTD(7)
#define GPIO_LCD_BACKLIGHT	JZ_GPIO_PORTD(31)
#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

#endif /* __ASM_JZ4740_A320_H__ */
