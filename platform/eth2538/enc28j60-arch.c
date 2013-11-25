/*
 * Copyright (c) 2012-2013, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "spi-arch.h"
#include "dev/spi.h"
#include "enc28j60.h"

#include <stdint.h>

/*
  RF1.16_SCK LV_SPI_SCK RF1.16 PA2
  RF1.18_MOSI LV_SPI_MOSI RF1.18 PA4
  RF1.20_MISO LV_SPI_MISO RF1.20 PA5
  RF2.10 ¯¯¯¯¯¯¯¯¯¯¯ LV_ACC_CS RF2.10 PD5
*/

/* CS = Chip Select */
#define SPI_SEL_PORT_BASE               GPIO_PORT_TO_BASE(SPI_SEL_PORT)
#define SPI_SEL_PIN_MASK                GPIO_PIN_MASK(SPI_SEL_PIN)
#define ENC28J60_SPI_CS_PORT            GPIO_D_NUM
#define ENC28J60_SPI_CS_PIN             5
#define ENC28J60_SPI_CS_PORT_BASE       GPIO_PORT_TO_BASE(ENC28J60_SPI_CS_PORT)
#define ENC28J60_SPI_CS_PIN_MASK        GPIO_PIN_MASK(ENC28J60_SPI_CS_PIN)
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_init(void)
{
  spi_init();

  /* Disable the LCD /CS. */
  GPIO_SOFTWARE_CONTROL(SPI_SEL_PORT_BASE, SPI_SEL_PIN_MASK);
  GPIO_SET_OUTPUT(SPI_SEL_PORT_BASE, SPI_SEL_PIN_MASK);
  GPIO_SET_PIN(SPI_SEL_PORT_BASE, SPI_SEL_PIN_MASK);
  ioc_set_over(SPI_SEL_PORT, SPI_SEL_PIN, IOC_OVERRIDE_DIS);

  GPIO_SOFTWARE_CONTROL(ENC28J60_SPI_CS_PORT_BASE, ENC28J60_SPI_CS_PIN_MASK);
  GPIO_SET_OUTPUT(ENC28J60_SPI_CS_PORT_BASE, ENC28J60_SPI_CS_PIN_MASK);
  GPIO_SET_PIN(ENC28J60_SPI_CS_PORT_BASE, ENC28J60_SPI_CS_PIN_MASK);
  ioc_set_over(ENC28J60_SPI_CS_PORT, ENC28J60_SPI_CS_PIN, IOC_OVERRIDE_DIS);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_select(void)
{
  GPIO_CLR_PIN(ENC28J60_SPI_CS_PORT_BASE, ENC28J60_SPI_CS_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_deselect(void)
{
  SPI_WAITFOREOTx();
  GPIO_SET_PIN(ENC28J60_SPI_CS_PORT_BASE, ENC28J60_SPI_CS_PIN_MASK);
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_write(uint8_t output)
{
  SPI_WAITFORTx_BEFORE();
  SPI_TXBUF = output;
  SPI_WAITFOREORx();
  return SPI_RXBUF;
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_read(void)
{
  return enc28j60_arch_spi_write(0);
}
/*---------------------------------------------------------------------------*/
