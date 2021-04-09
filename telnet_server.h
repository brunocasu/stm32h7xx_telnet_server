/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 */

/* Modified by Bruno Casu (2021 - SPRACE, São Paulo BR) */
/* This software provides a Serial over LAN implementation for the STM32H7 family */
/* 1 TAB = 2 Spaces */

#ifndef __TELNET_H__
#define __TELNET_H__

// STM32 HAL include
#include "stm32h7xx_hal.h"
// FreeRTOS includes
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
// LwIP includes
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"

// WARNING: User must set the interruption mode for the UART peripheral used
// WARNING: User must enable the creation of custom callbacks for the UART peripheral
//			this is done at the file stm32h7xx_hal_conf.h altering the value of USE_HAL_UART_REGISTER_CALLBACKS to 1

// number of simultaneous telnet connections used - up to 256
#define MAX_NUM_TELNET_INST   2 // user defined


typedef struct
{
  struct tcp_pcb* telnet_pcb;
  struct tcp_pcb* host_pcb;
  uint16_t tcp_port;
  StreamBufferHandle_t serial_input_stream;
  StreamBufferHandle_t tcp_input_stream;

} telnet_t;


// this function should be called once for each telnet connection
void telnet_create( telnet_t* instance, uint16_t port );

void telnet_transmit_char(telnet_t* instance, char character);

// Extra application: UDP echo server
void udp_echo_create(uint16_t port);

// When using LwIP Middleware:
// To enable Hostname go to /LwIP/src/include/lwip/opt.h and change the value of LWIP_NETIF_HOSTNAME to 1
// To edit Hostname go to /LWIP/Target/ethernetif.c and search for LWIP_NETIF_HOSTNAME

#endif /* __TELNET_H__ */
