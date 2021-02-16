 /**
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
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable echo application.
 *
 **/

/* This file was modified by ST */

/* Modified by Bruno Casu (2021 - SPRACE, São Paulo BR) */
/* This software provides a Serial over LAN implementation for the STM32H7 family */
/* 1 TAB = 2 Spaces */

#include "telnet_server.h"

#if LWIP_TCP

// instance control defines
#define INSTANCE_NOT_FOUND    0
#define INSTANCE_FOUND        1
#define INSTANCE_NOT_LISTED   2

enum tcp_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

// structure to maintain connection information, used as additional argument in LwIP callbacks
struct tcp_mng_struct
{
  uint8_t state;                        /* current connection state */
  uint8_t telnet_instance;              /* telnet instance identifier */
  UART_HandleTypeDef* serial_handler;   /* handler of the UART peripheral */
  struct pbuf *p;                       /* pointer on the received/to be transmitted pbuf */
  struct tcp_pcb *pcb;                  /* pointer on the current tcp_pcb */
};

// functions created for the telnet implementation
void telnet_init(uint8_t telnet_inst);
void telnet_error_handler (uint8_t instance);
static void tcp_recv_reset(struct tcp_pcb *tpcb, struct tcp_mng_struct *es);
static void tcp_pbuf_to_serial (struct pbuf* p, UART_HandleTypeDef* serial_handler, uint8_t telnet_instance);

// functions based in the TCP echosever example
static err_t tcp_com_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_com_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_com_error(void *arg, err_t err);
static void tcp_com_connection_close(struct tcp_pcb *tpcb, struct tcp_mng_struct *es);

// global arrays to store TCP data for the different instances
static struct tcp_pcb* telnet_pcb[MAX_NUM_TELNET_INST];
static struct tcp_pcb* host_pcb[MAX_NUM_TELNET_INST];
static uint16_t tcp_port[MAX_NUM_TELNET_INST] = {0};
// global array to store serial handlers for the instances
static UART_HandleTypeDef* tcp_serial_handler[MAX_NUM_TELNET_INST];
// stream buffer array for serial to tcp transmission
StreamBufferHandle_t serial_input_stream[MAX_NUM_TELNET_INST];
// stream buffer array for the tcp to serial transmission
StreamBufferHandle_t tcp_input_stream[MAX_NUM_TELNET_INST];

// serial to TCP variables
unsigned char single_character; // store received char from UART

// serial to TCP task handler and attributes
osThreadId_t serial_to_tcp_TaskHandle;
const osThreadAttr_t serial_to_tcp_TaskAttributes = {
  .name = "serial to tcp",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 8
};
// serial to TCP task function
void serial_to_tcp_Task (void *argument);
// custom callback for UART recv
void telnet_serial_RxCpltCallback(UART_HandleTypeDef *UartHandle);


/**
 * @brief create a new instance of telnet connection in a defined TCP port connected to a defined serial port (UART)
 * @param port number of the TCP connection Port
 * @param serial_handler STM32_HAL handler for the UART peripheral
 * @retval None
 *
 * @note This function can be called multiple times on the program.
 * 		   For each call a new telnet instance will be created and its parameters
 * 		   will be stored in the global arrays accordingly
 */
void telnet_create (uint16_t port, UART_HandleTypeDef* serial_handler)
{
  static uint8_t telnet_instance = 0;
  uint8_t *inst_for_task;
  
  // register the Custom UART callback for the recv mode
  HAL_UART_RegisterCallback(serial_handler, HAL_UART_RX_COMPLETE_CB_ID, telnet_serial_RxCpltCallback);

  // check if new instance reaches the maximum
  if (telnet_instance < MAX_NUM_TELNET_INST)
  {
	inst_for_task = pvPortMalloc(sizeof(char));
	*inst_for_task = telnet_instance;

	// create the serial recv task for this instance - pass the telnet instance to each new task created
	serial_to_tcp_TaskHandle = osThreadNew(serial_to_tcp_Task, (void *)inst_for_task, &serial_to_tcp_TaskAttributes);

	// add the port of the TCP connection to the global array
    tcp_port[telnet_instance] = port;

    // add the handler of the serial peripheral to the global array
    tcp_serial_handler[telnet_instance] = serial_handler;

    // initialize new TCP connection for the given instance
    telnet_init(telnet_instance);

    // set counter for next instance
    telnet_instance++;
  }
}


/**
  * @brief Initializes the TCP server
  * @param telnet_inst number of the telnet instance
  * @retval None
  */
void telnet_init(uint8_t telnet_inst)
{
  err_t err;
  
  // create new TCP protocol control block for the given instance - store the pcb in the global array
  telnet_pcb[telnet_inst] = tcp_new();
  
  if (telnet_pcb[telnet_inst] != NULL)
  {
    // bind the TCP connection to defined port
    err = tcp_bind(telnet_pcb[telnet_inst], IP_ADDR_ANY, tcp_port[telnet_inst]);
    
    if (err == ERR_OK)
    {
      // start TCP listening
      telnet_pcb[telnet_inst] = tcp_listen(telnet_pcb[telnet_inst]);
      
      // set tcp_accept callback function
      tcp_accept(telnet_pcb[telnet_inst], tcp_com_accept);
    }
    else 
    {
      // free the pcb if binding failed
      memp_free(MEMP_TCP_PCB, telnet_pcb[telnet_inst]);
    }
  }
  else
  {
    // fail to create new protocol control block
    telnet_error_handler (telnet_inst);
  }
}


/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg not used
  * @param  newpcb pointer on tcp_pcb struct for the newly created TCP connection
  * @param  err not used
  * @retval err_t error status
  * 
  * @note This callback is called upon a request from the remote host. It will set receiver mode for
  *       the TCP server and also for the UART peripheral of the defined instance.
  */
static err_t tcp_com_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_mng_struct *es;
  int inst_located = 0;
  uint8_t inst = 0;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  // find the correct instance for this connection
  while (inst_located == INSTANCE_NOT_FOUND)
  {
    // match the received connection port with the telnet instance port
    if((newpcb->local_port == tcp_port[inst]) && (inst < MAX_NUM_TELNET_INST))
    {
      inst_located = INSTANCE_FOUND;
    }
    // increment instance value
    inst++;
    
    // if received port does not match with any instance return error
    if(inst >= MAX_NUM_TELNET_INST)
      return ERR_VAL;
  }
  inst--; // fix addition from the loop

  // set priority for the newly accepted TCP connection newpcb
  tcp_setprio(newpcb, TCP_PRIO_MIN);
  
  // save pcb data for transmission
  host_pcb[inst] = (struct tcp_pcb *)newpcb;

  // allocate the TCP control structure to maintain connection informations
  es = (struct tcp_mng_struct *)mem_malloc(sizeof(struct tcp_mng_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED; // update TCP state
    es->pcb = newpcb; // save connection pcb
    es->telnet_instance = inst; // save connection instance
    es->serial_handler = tcp_serial_handler[inst]; // save serial handler of this instance
    es->p = NULL;
    
    // pass newly allocated es structure as argument to newpcb
    tcp_arg(newpcb, es);
    
    // initialize lwip tcp_recv callback function for newpcb
    tcp_recv(newpcb, tcp_com_recv);
    
    // initialize lwip tcp_err callback function for newpcb
    tcp_err(newpcb, tcp_com_error);
    
    // initialize serial peripheral in recv mode
    HAL_UART_Receive_IT(tcp_serial_handler[inst], &single_character, 1);

    ret_err = ERR_OK;
  }
  else
  {
    // close TCP connection
    tcp_com_connection_close(newpcb, es);

    // return memory error
    ret_err = ERR_MEM;
  }
  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg pointer on a argument for the tcp_pcb connection - used for the TCP control struct
  * @param  tpcb pointer on the tcp_pcb connection
  * @param  pbuf pointer on the received pbuf
  * @param  err error information regarding the received pbuf
  * @retval err_t error code
  * 
  * @note As the TCP connection is accepted, this callback will be called upon the receiving of a TCP pkt.
  *       It will identify the instance of the telnet connection based on the TCP port that received the pkt.
  *       After confirming the instance it will transmit the data obtained from the TCP pkt using the defined serial port.
  */
static err_t tcp_com_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_mng_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_mng_struct *)arg;
  
  // if we receive an empty tcp frame from client => close connection
  if (p == NULL)
  {
    // remote host closed connection
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       // we're done sending, close connection
       tcp_com_connection_close(tpcb, es);
    }
    else // if there are still data to be sent
    {
      tcp_pbuf_to_serial(p, es->serial_handler, es->telnet_instance);
      
      // clear for new reception
      tcp_recv_reset(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  // else: a non empty frame was received from client but for some reason err != ERR_OK
  else if(err != ERR_OK)
  {
    // free received pbuf
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    // first data chunk in p->payload
    es->state = ES_RECEIVED;
    
    // store reference to incoming pbuf (chain)
    es->p = p;

    // forward pkt data to serial interface
    tcp_pbuf_to_serial(p, es->serial_handler, es->telnet_instance);

    // clear for new reception
    tcp_recv_reset(tpcb, es);

    ret_err = ERR_OK;
  }
  else if(es->state == ES_RECEIVED)
  {
    // more data received from client and previous data has been already sent
    if(es->p == NULL)
    {
      es->p = p;
  
      // forward pkt data to serial interface
      tcp_pbuf_to_serial(p, es->serial_handler, es->telnet_instance);

      // clear for new reception
      tcp_recv_reset(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;

      // chain pbufs to the end of what we received previously
      ptr = es->p;
      pbuf_chain(ptr,p);

    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    // odd case, remote side closing twice, trash data
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    // unknown es->state, trash data
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}


/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg pointer on argument parameter
  * @param  err not used
  * @retval None
  */
static void tcp_com_error(void *arg, err_t err)
{
  struct tcp_mng_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_mng_struct *)arg;
  if (es != NULL)
  {
    //  free es structure
    mem_free(es);
  }
}


/**
 * @brief Clear tcp struct to enable new reception
 * @param tpcb pointer on the tcp_pcb connection
 * @param es TCP control struct
 * @retval none
 *
 */
static void tcp_recv_reset(struct tcp_pcb *tpcb, struct tcp_mng_struct *es)
{
  struct pbuf *ptr;
  
  // get pointer on pbuf from es structure
  ptr = es->p;
  
  uint16_t plen;
  uint8_t freed;
  
  plen = ptr->len;
  
  // continue with next pbuf in chain (if any)
  es->p = ptr->next;
  
  if(es->p != NULL)
  {
    // increment reference count for es->p
    pbuf_ref(es->p);
  }
  
  // chop first pbuf from chain
  do
  {
    // try hard to free pbuf
    freed = pbuf_free(ptr);
  }
  while(freed == 0);
  // we can read more data now
  tcp_recved(tpcb, plen);
}


/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb pointer on the tcp connection
  * @param  es pointer on echo_state structure
  * @retval None
  */
static void tcp_com_connection_close(struct tcp_pcb *tpcb, struct tcp_mng_struct *es)
{
  // remove all callbacks
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  // delete es structure
  if (es != NULL)
  {
    mem_free(es);
  }  

  // close tcp connection
  tcp_close(tpcb);
}


/**
 * @brief Send the received packet data (TCP) via serial port (UART)
 * @param p TCP payload information struct
 * @param serial_handler STM32_HAL handler for the UART peripheral
 * @param telnet_instance instance value for this connection
 * @retval None
 * 
 * @note this function is called at the callback set in the tcp_recv() function
 */
static void tcp_pbuf_to_serial (struct pbuf* p, UART_HandleTypeDef* serial_handler, uint8_t telnet_instance)
{
  unsigned char* tcp_data;
  
  if (p->len > 0) // p->len is the received payload size
  {
    tcp_data = (unsigned char*)p->payload;

    HAL_UART_Transmit(tcp_serial_handler[telnet_instance], tcp_data, p->len, 100); // send character via serial - blocking mode
    // The data from the received TCP packet must be treated (in this case, sent via serial port) before this callback returns.
    // The tcp_recv_reset() function will clear the pbuf and enable the reading of a new TCP frame.
    // LwIP will manage the receiving of multiple frames using its MPU allocated region, and will call the recv callback
    // function as the data is processed and the pbuf is released.
  }
}


/**
 * @brief Telnet task to transmit data from the serial port to the TCP host connected
 * @param argument used to identify the telnet instance for the multiple tasks created
 * @retval None
 *
 */
void serial_to_tcp_Task (void *argument)
{
  unsigned char c;
  const TickType_t xBlockTime = pdMS_TO_TICKS( 20 ); // timeout to send the tcp message
  size_t xReceivedBytes; // control to identify end of msg
  static char serial_to_tcp_buff[512] = {0};
  static uint16_t msg_size = 0;
  int recv_ctr = 0;
  uint8_t *ti;
  
  ti = (uint8_t *)argument;
  uint8_t const inst = *ti;

  // create the stream buffer to receive single character from the ISR (UART custom callback)
  serial_input_stream[inst] = xStreamBufferCreate(256, 1);

  for(;;)
  {
    if (recv_ctr == 0) // start receiver without a timeout value: waiting for a new char from the serial
    {
      xReceivedBytes = xStreamBufferReceive(serial_input_stream[inst], &c, 1, portMAX_DELAY);
      recv_ctr = 1;
    }
    else if (recv_ctr == 1) // after receiving the first character, the buffer reading has a timeout
    {
      xReceivedBytes = xStreamBufferReceive(serial_input_stream[inst], &c, 1, xBlockTime); // return zero if timeout occurs: no data in the buffer
    }                                                                                                                       
    
    // add received char to buff (if any), increment msg size counter
    if (xReceivedBytes > 0)
    {
      serial_to_tcp_buff[msg_size] = c;
      msg_size++;
    }
    
    // check for the end of the msg: no chars received in the timeout period or buff max size reached
    if ( (xReceivedBytes == 0) || (msg_size >= (sizeof(serial_to_tcp_buff) -1)) )
    {
      // enqueue data for transmission - max size of TCP data sent is 512 Bytes
      tcp_write(host_pcb[inst], serial_to_tcp_buff, msg_size, TCP_WRITE_FLAG_COPY);
      // output the TCP data
      tcp_output(host_pcb[inst]);
      // reset msg size and return to receiver without timeout
      msg_size = 0;
      recv_ctr = 0;
    }
  }
}

/**
 * @brief Custom callback function for the interrupt handler of all UART peripherals set for the telnet instances
 * @param UartHandle STM32_HAL: handler for the UART peripheral
 * @retval None
 * 
 * @note This callback is called from the ISR triggered by a Character Receiving Complete (STM32_HAL defined)
 * 
 */
void telnet_serial_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int inst_located = INSTANCE_NOT_FOUND;
  uint8_t inst = 0;
  
  // reset UART recv
  HAL_UART_Receive_IT(UartHandle, &single_character, 1);
  
  // find the correct instance for this connection
  while (inst_located == INSTANCE_NOT_FOUND)
  {
    // match the UART instance that triggered the interrupt with the UART used in the telnet connection
    if((UartHandle->Instance == tcp_serial_handler[inst]->Instance) && (inst < MAX_NUM_TELNET_INST))
    {
      inst_located = INSTANCE_FOUND;
    }
    // increment instance value
    inst++;
    
    // if received UART does not match with any instance exit loop
    if(inst >= MAX_NUM_TELNET_INST)
    	inst_located = INSTANCE_NOT_LISTED;
  }
  inst--; // fix addition from the loop
  
  // send received char to stream buff of the located instance
  if(inst_located == INSTANCE_FOUND)
  {
    xStreamBufferSendFromISR(serial_input_stream[inst], &single_character, 1, &xHigherPriorityTaskWoken);
  }
  else if(inst_located == INSTANCE_NOT_LISTED)
  {
    telnet_error_handler (inst);
  }
}


/**
 * @brief Error handler function
 * @param instance telnet instance
 * @retval None
 * 
 */
void telnet_error_handler (uint8_t instance)
{
  while(1);
}


/**
 * @brief Callback function for Receiver Mode in the UDP Echo application
 *
 */
static void udpecho_raw_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, uint16_t port)
{
	if (p != NULL)
	{
		// Echo msg
		udp_sendto(pcb, p, addr, port);
		// Free Data Pointer
		pbuf_free(p);
	}
}

/**
 * @brief Extra application: create UDP Binding on a defined Port. Echoes all the messages sent in that port.
 * @param port Port number for the connection
 * @retval none
 *
 */
void udp_echo_create(uint16_t port)
{
  struct udp_pcb * pcb;

  // create new protocol control block for the UDP server
  pcb = udp_new();

  if (pcb != NULL)
  {
	// bing to the defined port
  	udp_bind(pcb, IP_ADDR_ANY, port);
  }

  // Set UDP receiver with a callback function
  udp_recv(pcb, udpecho_raw_recv, pcb);
}

#endif /* LWIP_TCP */
