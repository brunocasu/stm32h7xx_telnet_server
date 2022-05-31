/*
 * MIT License
 * 
 * Copyright (c) 2022 André Cascadan and Bruno Augusto Casu
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
 */

/* 1 TAB = 4 Spaces */


// FreeRTOS includes
#include "cmsis_os.h"  // TODO: Decide which API to use: FreeRTOS or CMSIS. Here they are mixed.
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

// LwIP includes
#include "lwip/api.h"


#include "telnet_server.h"


// Current implementation allows only single telnet connection (one instance).
static telnet_t telnet_instance;
static telnet_t* instance = &telnet_instance;

// Process input bytes
static void process_incoming_bytes (uint8_t *data, int data_len, telnet_t* inst_ptr);

// Callback for netconn interface
static void netconn_cb (struct netconn *conn, enum netconn_evt evt, u16_t len);



// serial to TCP task handler and attributes

const osThreadAttr_t serial_to_tcp_TaskAttributes = {
  .name = "serial to tcp",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 8
};

// Task functions
static void wrt_task (void *arg);
static void rcv_task (void *arg);



void telnet_create( uint16_t port,
                    void (*receiver_callback)( uint8_t* buff, uint16_t len ),
                    void (*command_callback) ( uint8_t* cmd,  uint16_t len )  )
{

  // Stores the callback pointers
  instance->receiver_callback = receiver_callback;
  instance->command_callback  = command_callback;

  err_t err;

  // Initializes command buffer size
  instance->cmd_len = 0;

  // Stores the port of the TCP connection to the global array
  instance->tcp_port = port;

  // Starts local listening
  instance->conn = netconn_new_with_callback(NETCONN_TCP, netconn_cb);
  if( instance->conn == NULL )
    return;

  err = netconn_bind(instance->conn, NULL, port);
  if ( err != ERR_OK )
	return;

  netconn_listen(instance->conn);

  // No connection still established
  instance->status = TELNET_CONN_STATUS_NONE;
  
  // Create the accept sentd task
  instance->wrt_task_handle = osThreadNew(wrt_task, NULL, &serial_to_tcp_TaskAttributes);
  instance->rcv_task_handle = osThreadNew(rcv_task, NULL, &serial_to_tcp_TaskAttributes);
}


/*
 * Netconn callback
 *
 * This project uses netconn with callback to manage the open/close events
 */
void netconn_cb (struct netconn *conn, enum netconn_evt evt, u16_t len)
{
	if( evt == NETCONN_EVT_RCVPLUS )
	{
		if( len == 0 ) // len = 0 means the connections is being opened or closed by the client
		{
			// Switch the connection status according to the current one
			if( instance->status == TELNET_CONN_STATUS_ACCEPTING)
				instance->status = TELNET_CONN_STATUS_CONNECTED;
			else
				instance->status = TELNET_CONN_STATUS_CLOSING;
		}
	}
}



/*
 * Process input bytes
 *
 * Filters out the telnet commands from the regular characters and calls the
 * proper user callback.
 */
static void process_incoming_bytes (uint8_t *data, int data_len, telnet_t* inst_ptr)
{
    const uint8_t IAC  = 255; // See RFC 854 for details
    const uint8_t WILL = 251;
    const uint8_t DONT = 254;
    //const uint8_t SB   = 254; TODO: Parse Subnegotiation commands.

    uint8_t* pbuf_payload = data;
    uint16_t pbuf_len     = data_len;

    /*
     * If command callback IS NOT defined, all bytes are sent to the receiver callback.
     *
     * In this case user is responsible to separate them from characters.
     */
	if(inst_ptr->command_callback == NULL)
	{
		inst_ptr->receiver_callback( pbuf_payload, pbuf_len );
	}

	//If command callback IS defined, all bytes are filtered out from the characters
	else
	{
		uint16_t char_offset   = 0;
		uint16_t char_ctr      = 0;

		for( int i = 0; i < pbuf_len; i++ )
		{
			// Counting characters: Command buffer is empty and IAC not found
			if( (inst_ptr->cmd_len == 0) && (pbuf_payload[i] != IAC) )
				++char_ctr;

			// Counting command bytes
			else
			{
				// Found IAC with command buffer empty:
				if( (inst_ptr->cmd_len == 0) && (pbuf_payload[i] == IAC) )
				{
					// Process the characters found up to this point
					if( char_ctr != 0 )
						inst_ptr->receiver_callback( &pbuf_payload[char_offset], char_ctr );

					inst_ptr->cmd_buff[inst_ptr->cmd_len] = pbuf_payload[i];
					inst_ptr->cmd_len++;
				}
				else if( (inst_ptr->cmd_len == 1) && (pbuf_payload[i] >= WILL) && (pbuf_payload[i] <= DONT) )
				{
					inst_ptr->cmd_buff[inst_ptr->cmd_len] = pbuf_payload[i];
					inst_ptr->cmd_len++;
				}
				else if( inst_ptr->cmd_len == 2 && (pbuf_payload[1] >= WILL) && (pbuf_payload[1] <= DONT) )
				{
					inst_ptr->cmd_buff[inst_ptr->cmd_len] = pbuf_payload[i];
					inst_ptr->cmd_len++;

					// Process the command
					inst_ptr->command_callback( inst_ptr->cmd_buff,  inst_ptr->cmd_len);

					// Restart counting characters. Erase command buffer.
					inst_ptr->cmd_len = 0;
					char_offset = i+1;
					char_ctr = 0;
				}
			}
		}

		// Process the characters found at the end of buffer scanning
		if( char_ctr != 0 )
			inst_ptr->receiver_callback( &pbuf_payload[char_offset], char_ctr );
	}
}




/**
 * @brief Telnet task to transmit data from the stream port to the TCP host connected
 * @param arg (not used used)
 * @retval None
 *
 */
static void wrt_task (void *arg)
{

  const TickType_t xBlockTime = pdMS_TO_TICKS( 20 ); // timeout to send the tcp message

  //TX
  static char tx_buff[512];
  const int tx_buff_size = 512;
  int n_from_stream;
  
  //RX
  struct netbuf *rx_netbuf;
  void *rx_data;
  u16_t rx_data_len;
  err_t recv_err;

  err_t accept_err;



  // create the stream buffer to receive characters from the stream
  instance->serial_input_stream = xStreamBufferCreate(256, 1);



  /*
   * Accept loop
   * Stays waiting for a connection. If connection breaks, it waits for a new one.
   */
  for(;;)
  {
	  instance->status = TELNET_CONN_STATUS_ACCEPTING;
	  accept_err = netconn_accept(instance->conn, &instance->newconn);
	  //netconn_set_recvtimeout ( newconn, 100 );
	  if( accept_err == ERR_OK )
	  {
		  // Transfer loop
		  for(;;)
		  {
			  n_from_stream = xStreamBufferReceive(instance->serial_input_stream, tx_buff, tx_buff_size, xBlockTime);

			  // Check the connections status before send bytes, if any
			  if( n_from_stream != 0 && instance->status == TELNET_CONN_STATUS_CONNECTED )
				  netconn_write(instance->newconn, tx_buff, n_from_stream, NETCONN_COPY);

			  // Iteratively reads all the available data
			  //recv_err = netconn_recv(newconn, &rx_netbuf);
			  //if ( recv_err == ERR_OK)
			  //{
			//	  netbuf_data(rx_netbuf, &rx_data, &rx_data_len);
			//	  process_incoming_bytes (rx_data, rx_data_len, instance);
              //
              //
			  //    netbuf_delete(rx_netbuf);
			  //}

			  // Force a connections close if a terminating was detected from client
			  if( instance->status == TELNET_CONN_STATUS_CLOSING )
				  break;

		  }

		  netconn_close (instance->newconn);
		  netconn_delete(instance->newconn);
	  }
  }
}

static void rcv_task (void *arg)
{
	struct netbuf *rx_netbuf;
	void *rx_data;
	uint16_t rx_data_len;
	err_t recv_err;

	for(;;)
	{

		if( instance->status != TELNET_CONN_STATUS_CONNECTED )
		{
			vTaskDelay(100); // *do nothing* delay if there is no connection.
		}
		else
		{
			// Iteratively reads all the available data
			recv_err = netconn_recv(instance->newconn, &rx_netbuf);
			if ( recv_err == ERR_OK)
			{
				// Navigate trough netbuffs until dump all data
				do
				{
					netbuf_data(rx_netbuf, &rx_data, &rx_data_len);
					process_incoming_bytes (rx_data, rx_data_len, instance);

				}while(  netbuf_next(rx_netbuf) >= 0 );

				netbuf_delete( rx_netbuf );
			}
		}
	}
}



uint16_t telnet_transmit(uint8_t* buff, uint16_t len)
{
  //TODO: flexibility for ISR context
  int ret;

  if( instance->serial_input_stream == NULL )
	  return 0;
  
  if( instance->status != TELNET_CONN_STATUS_CONNECTED )
	  return 0;

  // Send char to stream buffer of the associated instance
  //taskENTER_CRITICAL();
  ret = xStreamBufferSend(instance->serial_input_stream, buff, len, 100);
  //taskEXIT_CRITICAL();

  return ret;
}




