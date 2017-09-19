/****************************************************************************
 * configs/lpc4357-dev/src/lpc43_autoleds.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc4357-dev.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* LED definitions **********************************************************/
/* The LPC4357-DEV has one user-controllable LED labelled D2~6 controlled by
 * the signal LED_3V3:
 *
 *  ---- ------- -------------
 *  LED  SIGNAL  MCU
 *  ---- ------- -------------
 *   D2  LED_3V3 PD_10 GPIO6[24]
 *   D3  LED_3V3 PD_11 GPIO6[25]
 *   D4  LED_3V3 PD_12 GPIO6[26]
 *   D5  LED_3V3 PD_13 GPIO6[27]
 *   D6  LED_3V3 PD_14 GPIO6[28]
 *  ---- ------- -------------
 *
 * LED is grounded and a high output illuminates the LED.
 *
 * If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 *
 *   -------------------------- --------
 *                              LED
 *   -------------------------- --------
 *   LED_STARTED                LED0
 *   LED_HEAPALLOCATE           LED1
 *   LED_IRQSENABLED            LED0 + LED1
 *   LED_STACKCREATED           LED2
 *   LED_INIRQ                  LED0 + LED2
 *   LED_SIGNAL                 LED1 + LED3
 *   LED_ASSERTION              LED0 + LED1 + LED2
 *   LED_PANIC                  N/C  + N/C  + N/C + LED3
 *   -------------------------- --------
 */
/* The following definitions map the encoded LED setting to GPIO settings */

#define LED_STARTED_BITS             (BOARD_LED0_BIT)
#define LED_HEAPALLOCATE_BITS        (BOARD_LED1_BIT)
#define LED_IRQSENABLED_BITS         (BOARD_LED0_BIT | BOARD_LED1_BIT)
#define LED_STACKCREATED_BITS        (BOARD_LED2_BIT)
#define LED_INIRQ_BITS               (BOARD_LED0_BIT | BOARD_LED2_BIT)
#define LED_SIGNAL_BITS              (BOARD_LED1_BIT | BOARD_LED3_BIT)
#define LED_ASSERTION_BITS           (BOARD_LED0_BIT | BOARD_LED1_BIT | BOARD_LED2_BIT)
#define LED_PANIC_BITS               (BOARD_LED3_BIT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const unsigned int g_ledbits[8] =
{
  LED_STARTED_BITS,
  LED_HEAPALLOCATE_BITS,
  LED_IRQSENABLED_BITS,
  LED_STACKCREATED_BITS,
  LED_INIRQ_BITS,
  LED_SIGNAL_BITS,
  LED_ASSERTION_BITS,
  LED_PANIC_BITS
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(FAR const char *msg)
{
  lpc43_pin_dump(PINCONFIG_LED, msg);
  lpc43_gpio_dump(GPIO_LED, msg);
}
#else
#  define led_dumppins(m)
#endif

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED0_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED0, false);
    }

  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED3, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED0_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED0, true);
    }

  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      lpc43_gpio_write(GPIO_LED3, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED pin as a GPIO outputs */

  led_dumppins("board_autoled_initialize() Entry)");

  /* Configure LED0~4 pin as a GPIO, then configure GPIO as an outputs */

  lpc43_pin_config(PINCONFIG_LED0);
  lpc43_gpio_config(GPIO_LED0);
  lpc43_pin_config(PINCONFIG_LED1);
  lpc43_gpio_config(GPIO_LED1);
  lpc43_pin_config(PINCONFIG_LED2);
  lpc43_gpio_config(GPIO_LED2);
  lpc43_pin_config(PINCONFIG_LED3);
  lpc43_gpio_config(GPIO_LED3);
  lpc43_pin_config(PINCONFIG_LED4);
  lpc43_gpio_config(GPIO_LED4);

  led_dumppins("board_autoled_initialize() Exit");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_clrbits(BOARD_LED0_BIT | BOARD_LED1_BIT | BOARD_LED2_BIT | BOARD_LED3_BIT);
  led_setbits(g_ledbits[led]);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_clrbits(g_ledbits[led]);
}

#endif /* CONFIG_ARCH_LEDS */
