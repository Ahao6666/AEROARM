/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gpioint.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GPIOINT_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GPIOINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit encoded gpioconf value
 *
 * 3322 2222 2222 1111 1111 1100 0000 0000
 * 1098 7654 3210 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ---- ---- ---- ----
 * .... .... .... ..B. .... .... .... .... Both detect mode
 * .... .... .... ...T .... .... .... .... Toggle detect mode
 * .... .... .... .... .... .... .... N... Noise Filter
 * .... .... .... .... .... .... .... .YYY Polarity
 */

/* GPIO Interrupt Polarity Definitions */

/* #define GPIOINT_INSTANT_HIGH      (0) */ /* Not supported */

/* #define GPIOINT_INSTANT_LOW       (1) */ /* Not supported */

#define GPIOINT_LEVEL_HIGH          (2) /* High Level */
#define GPIOINT_LEVEL_LOW           (3) /* Low Level */
#define GPIOINT_EDGE_RISE           (4) /* Rising Edge */
#define GPIOINT_EDGE_FALL           (5) /* Falling Edge */
#define GPIOINT_EDGE_BOTH           (7) /* Both Edge */
#define GPIOINT_PSEUDO_EDGE_RISE    (GPIOINT_LEVEL_HIGH | \
                                     GPIOINT_TOGGLE_MODE_MASK)
                                        /* Rising Edge without clear */

#define GPIOINT_PSEUDO_EDGE_FALL    (GPIOINT_LEVEL_LOW | \
                                     GPIOINT_TOGGLE_MODE_MASK)
                                        /* Falling Edge without clear */

#define GPIOINT_PSEUDO_EDGE_BOTH    (GPIOINT_TOGGLE_MODE_MASK | \
                                     GPIOINT_TOGGLE_BOTH_MASK)
                                        /* Both Edge without clear */

/* GPIO Interrupt Noise Filter Definitions */

#define GPIOINT_NOISE_FILTER_ENABLE     (1u << 3)
#define GPIOINT_NOISE_FILTER_DISABLE    (0u << 3)

/* Use Pseudo Edge Interrupt */

#define GPIOINT_TOGGLE_MODE_MASK        (1u << 16)
#define GPIOINT_TOGGLE_BOTH_MASK        (1u << 17)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cxd56_gpioint_status_s
{
  int       irq;
  uint32_t  polarity;
  bool      filter;
  bool      enable;
};

typedef struct cxd56_gpioint_status_s cxd56_gpioint_status_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_gpioint_config
 *
 * Description:
 *   Configure a GPIO pin as an GPIO pin interrupt source
 *
 * Input Parameters:
 *   pin - Pin number defined in cxd56_pinconfig.h
 *   gpiocfg - GPIO Interrupt Polarity and Noise Filter Configuration Value
 *   isr - Interrupt handler
 *   arg - Argument for the interrupt handler
 *
 * Returned Value:
 *   IRQ number on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The interrupt are disabled so that read-modify-write operations
 *   are safe.
 *
 ****************************************************************************/

int cxd56_gpioint_config(uint32_t pin, uint32_t gpiocfg, xcpt_t isr,
                         FAR void *arg);

/****************************************************************************
 * Name: cxd56_gpioint_irq
 *
 * Description:
 *   Get a GPIO interrupt number for specified pin number
 *
 * Returned Value:
 *   IRQ number on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_irq(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpioint_pin
 *
 * Description:
 *   Get a pin number for specified IRQ number
 *
 * Returned Value:
 *   Pin number on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_pin(int irq);

/****************************************************************************
 * Name: cxd56_gpioint_enable
 *
 * Description:
 *   Enable a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_enable(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpioint_disable
 *
 * Description:
 *   Disable a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_disable(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpioint_invert
 *
 * Description:
 *   Invert polarity of a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_invert(uint32_t pin);

/****************************************************************************
 * Name: cxd56_gpioint_status
 *
 * Description:
 *   Get a gpio interrupt status
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_status(uint32_t pin, cxd56_gpioint_status_t *stat);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GPIOINT_H */
