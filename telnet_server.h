/*
 * MIT License
 * 
 * Copyright (c) 2021 Bruno Augusto Casu
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * This file is part of the lwIP based telnet server.
 * 
 * Contributor: André Muller Cascadan (2021 - SPRACE, São Paulo BR)
 *
 */

/* 1 TAB = 2 Spaces */

#ifndef __TELNET_H__
#define __TELNET_H__


#include "FreeRTOS.h"
#include "stream_buffer.h"

// LwIP includes
#include "lwip/tcp.h"



typedef struct
{
  struct tcp_pcb* telnet_pcb;   // Stores TCP connection instances
  struct tcp_pcb* host_pcb;
  uint16_t tcp_port;            // Port do be listened in this telnet connection
  StreamBufferHandle_t serial_input_stream;
  void (*receiver_callback)( uint8_t* buff, uint16_t len );
  void (*command_callback) ( uint8_t* cmd,  uint16_t len );
  uint8_t cmd_buff[10];
  uint16_t cmd_len;

} telnet_t;


/**
 * @brief Create a new instance of telnet server in a defined TCP port
 * 
 * @param instance           Pointer to the telnet connection handler
 * @param port               Number of the TCP connection Port
 * @param receiver_callback  Pointer to the receiver callback function.
 * @param command_callback   Exclusive callback to receive the telnet commands from client.
 *                           If defined as NULL commands will be sent to "receiver_callback".
 *                           In this case, user is responsible to filter commands form
 *                           characters.
 */
void telnet_create( telnet_t* instance,
                     uint16_t port,
                         void (*receiver_callback)( uint8_t* buff, uint16_t len ),
                         void (*command_callback) ( uint8_t* cmd,  uint16_t len )  );


/*
 * @brief Sends data  to the client.
 * 
 * returns: the amount of bytes actually received by the driver.
 *
 * This functions can be user to send characters or telnet commands.
 */
uint16_t telnet_transmit(telnet_t* instance, uint8_t* buff, uint16_t len);


// When using LwIP Middleware:
// To enable Hostname go to /LwIP/src/include/lwip/opt.h and change the value of LWIP_NETIF_HOSTNAME to 1
// To edit Hostname go to /LWIP/Target/ethernetif.c and search for LWIP_NETIF_HOSTNAME

#endif /* __TELNET_H__ */
