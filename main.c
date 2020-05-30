/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cybsp_wifi.h"
#include "cy_retarget_io.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <lwip/apps/sntp.h>

/* lwIP header files */
#include <lwip/tcpip.h>
#include <lwip/udp.h>
#include <lwip/api.h>
#include <lwipinit.h>
#include <mbedtlsinit.h>
#include "ip4_addr.h"

#include "network_credentials.h"
#include "timer_config.h"
#include "SPIMaster.h"
#include "SpiDma.h"
#include "types.h"
#include "main_include.h"

/******************************************************************************
* Global Variables
******************************************************************************/
/* Handle of the Queue to send TCP data packets */
volatile QueueHandle_t tcp_client_queue;
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t dataReceivedTaskHandle = NULL;
/* Packet instance */
tcp_data_packet_t tcp_pkt_buf;
/* The primary WIFI driver  */
whd_interface_t iface ;
/* Connection */
struct netconn *conn;
/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;
volatile bool send_tcp_data = false;
volatile bool DRDY_received = false;
uint8_t spi_transmit_data[ADC_BYTES_PER_FRAME];
uint8_t spi_receive_data[ADC_BYTES_PER_FRAME];
uint8_t received_data[ADC_BYTES_PER_FRAME];
volatile bool tcp_task_started = false;
uint32_t second_temp = 0;
uint32_t fraction_temp = 0;
uint8_t stream_data = 0;
uint8_t sync_done = 0;
volatile uint8_t do_tpsn_sync = 0;

/******************************************************************************
* Main
******************************************************************************/
int main()
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

	if(USE_ADC)
	{
		// Magic that makes the DMA work (both RX and TX)!
		Cy_TrigMux_Connect(TRIG0_IN_TR_GROUP13_OUTPUT13, TRIG0_OUT_CPUSS_DW0_TR_IN3, false, TRIGGER_TYPE_LEVEL);
		Cy_TrigMux_Connect(TRIG0_IN_TR_GROUP13_OUTPUT5, TRIG0_OUT_CPUSS_DW0_TR_IN2, false, TRIGGER_TYPE_LEVEL);
		Cy_TrigMux_Connect(TRIG13_IN_SCB1_TR_RX_REQ, TRIG13_OUT_TR_GROUP0_INPUT40, false, TRIGGER_TYPE_LEVEL);
		Cy_TrigMux_Connect(TRIG13_IN_SCB1_TR_TX_REQ, TRIG13_OUT_TR_GROUP0_INPUT32, false, TRIGGER_TYPE_LEVEL);

		// Init SPI Master
		initMaster();

		// Configure Rx and Tx DMA channels
		ConfigureTxDma(spi_transmit_data);
		ConfigureRxDma();

		// Configure the interrupt pin for ADC DRDY signal
		setup_drdy_interrupt();
	}
	else
	{
	    result = cyhal_gpio_init(DIG_IN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
	}

	timer_init();

    cyhal_gpio_init(ADC_DEBUG, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");

    // Initialize the transmission and reception buffers.
    for (int j = 0; j < ADC_BYTES_PER_FRAME; j++)
    {
    	spi_transmit_data[j] = 0;
    	spi_receive_data[j] = 0;
    }

    /* Queue to Receive TCP packets */
    tcp_client_queue = xQueueCreate(TCP_CLIENT_TASK_QUEUE_LEN, sizeof(tcp_data_packet_t));

    BaseType_t xReturned;
    xReturned = xTaskCreate(tcp_client_task, "Network task", TCP_CLIENT_TASK_STACK_SIZE, NULL,
                1, &networkTaskHandle);
    if( xReturned != pdPASS )
    {
        CY_ASSERT(0);
    }

    xReturned = xTaskCreate(data_received_task, "Data received task", TCP_CLIENT_TASK_STACK_SIZE, NULL,
                1, &dataReceivedTaskHandle);
    if( xReturned != pdPASS )
    {
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/******************************************************************************
* Functions
******************************************************************************/
void compute_sntp_timestamps()
{
	if (sync_received)
	{
		sync_done = 1;
		second_temp = second;
		fraction_temp = fraction;
		sync_received = 0;
		printf("tcp_client_task: second = %lu, fraction = %lu\n", second_temp, fraction_temp);
	}
	else
	{
		second_temp = second_temp + (fraction_temp + TIME_UPDATE_PERIOD) / 1000000;
		fraction_temp = (fraction_temp + TIME_UPDATE_PERIOD) % 1000000;
	}
}

/* Interrupt handler callback function */
void drdy_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
	/* Wait for at least t_CSSC and set CS HIGH */
	cyhal_gpio_write(ADC_CS, false);
	sendPacket();
}

/* Updates the local time using the specified offsets. */
void update_time(int32_t sec_offset, int32_t microsec_offset)
{
	__disable_irq();
    node_time.seconds = node_time.seconds + sec_offset;

    if (node_time.microseconds + microsec_offset < 0)
    	node_time.seconds--;
    else if (node_time.microseconds + microsec_offset >= 1000000)
    	node_time.seconds++;

    node_time.microseconds = (1000000 + node_time.microseconds + microsec_offset) % 1000000;
    __enable_irq();
}

/* Processes the synchronization message and updates the local time.*/
void process_sync_message(uint32_t *message, uint16_t message_len)
{
	int32_t sec_offset = 0;
	int32_t microsec_offset = 0;

	// Sent back by the TCP server
	volatile uint64_t t1 = (uint64_t)message[1] * 1000000 + message[2];
	// t2 and t3 are server timestamps
	volatile uint64_t t2 = (uint64_t)message[3] * 1000000 + message[4];
	volatile uint64_t t3 = (uint64_t)message[5] * 1000000 + message[6];
	// The time at which the sync packet was received
	__disable_irq();
	volatile uint64_t t4 = (uint64_t)node_time.seconds * 1000000 + node_time.microseconds;
	__enable_irq();

	volatile uint64_t delta = 0;
	delta = (uint64_t)(((t2 - t1) - (t4 - t3)) / 2);

	sec_offset = (int32_t)(delta / 1000000);
	microsec_offset = (int32_t)(delta % 1000000);

	// If seconds offset is bigger than a threshold, then the value is corrupted and
	// should be discarded.
	if (sec_offset < 2000000)
	{
		update_time(sec_offset, microsec_offset);
		sync_done = 1;
	}
}

/* Task used to receive data from other devices.*/
void data_received_task(void *arg)
{
    struct netbuf* netbuf = NULL;
    uint8_t *ptr;
    uint16_t plen = 0;
    uint32_t *message;
    uint16_t message_len = 0;
    volatile err_t recv_err;

	for(;;)
	{
		if(tcp_task_started)
		{
			// netconn_recv() blocks the task until data is available on connection conn.
	        while (( recv_err = netconn_recv(conn, &netbuf)) == ERR_OK)
	        {
	            do
	            {
	                netbuf_data(netbuf, (void *)&ptr, &plen);
	                message = (uint32_t*)ptr;
	                message_len = plen / 4;

					if (message[1] == START_STREAM_CMD)
					{
						stream_data = 1;
						printf("Streaming started! \n");
					}
					else if (message[0] == TPSN_SYNC_WORD)
					{
						process_sync_message(message, message_len);
					}
	            }
	            while (netbuf_next(netbuf) >= 0);

	            netbuf_delete(netbuf);
	        }
		}
	}
}

/* Task used to establish a connection to a remote TCP server and send data.*/
void tcp_client_task(void *arg)
{
	init_tcp_client();

	__disable_irq();
	adc_startup_procedure();
	__enable_irq();

	tcp_task_started = true;

	for(;;)
	{
		xQueueReceive(tcp_client_queue, &tcp_pkt_buf, portMAX_DELAY);

		tcp_pkt_buf.alignment_word = ALIGNMENT_WORD;

		__disable_irq();
		if (SYNC_TYPE == SNTP)
		{
			tcp_pkt_buf.sync_s = second_temp;
			tcp_pkt_buf.sync_f = fraction_temp;
		}
		else
		{
			tcp_pkt_buf.sync_s = node_time.seconds;
			tcp_pkt_buf.sync_f = node_time.microseconds;
		}
		__enable_irq();

		tcp_pkt_buf.device_id = DEVICE_ID;

		if (stream_data)
		{
			// Overwrite the ADC header with data from channel 1.
			tcp_pkt_buf.data[0] = tcp_pkt_buf.data[5];
			tcp_pkt_buf.data[1] = tcp_pkt_buf.data[4];
			tcp_pkt_buf.data[2] = tcp_pkt_buf.data[3];
		}
		else
		{
			tcp_pkt_buf.data[0] = 0;
			tcp_pkt_buf.data[1] = 0;
			tcp_pkt_buf.data[2] = 0;
		}

		if (sync_done)
		{
			if (do_tpsn_sync)
			{
				do_tpsn_sync = 0;
				tcp_pkt_buf.alignment_word = TPSN_SYNC_WORD;
			}

			netconn_write(conn, &tcp_pkt_buf, 16, NETCONN_NOFLAG);
		}
		else if (do_tpsn_sync)
		{
			do_tpsn_sync = 0;
			tcp_pkt_buf.alignment_word = TPSN_SYNC_WORD;
			netconn_write(conn, &tcp_pkt_buf, 16, NETCONN_NOFLAG);
		}
	}
}

void initialize_sntp(void)
{
	sntp_setoperatingmode(SNTP_OPMODE_POLL);

	struct ip_addr remote = {
		//.u_addr.ip4.addr = MAKE_IPV4_ADDRESS(89, 149, 54, 18),
		.u_addr.ip4.addr = MAKE_IPV4_ADDRESS(92, 86, 106, 228),
		.type = IPADDR_TYPE_V4
	};

	sntp_setserver(0, &remote);
	sntp_init();
}

/*Initializes the TCP client.*/
void init_tcp_client()
{
    cy_rslt_t result ;
    err_t err;
    whd_ssid_t ssid_data ;
    const char *ssid = WIFI_SSID;
    const char *key = WIFI_PASSWORD;
    struct netif *net;
    struct ip_addr remote = {
            .u_addr.ip4.addr = TCP_SERVER_IP_ADDRESS,
            .type = IPADDR_TYPE_V4
    };

    /* Variable to track the number of connection retries to the Wi-Fi AP specified
     * by WIFI_SSID macro */
    int conn_retries = 0;

    /* Initialize and start the tcpip_thread */
    tcpip_init(NULL, NULL) ;
    printf("lwIP TCP/IP stack initialized\n");

    /* Initialize the Wi-Fi Driver */
    result = cybsp_wifi_init_primary(&iface);

    if(result == CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi driver initialized \n");
    }
    else
    {
        printf("Wi-Fi Driver initialization failed!\n");
        CY_ASSERT(0);
    }

    /* Join the Wi-Fi AP */
    result = WHD_PENDING;
    ssid_data.length = strlen(ssid);
    memcpy(ssid_data.value, ssid, ssid_data.length);

    while(result != CY_RSLT_SUCCESS && conn_retries < MAX_CONNECTION_RETRIES)
    {
        result = whd_wifi_join(iface, &ssid_data, WHD_SECURITY_WPA2_AES_PSK,
                              (const uint8_t *)key, strlen(key));
        conn_retries++;
    }

    if(result == CY_RSLT_SUCCESS)
    {
        printf("Sucessfully joined the Wi-Fi network '%s'\n", ssid);
    }
    else
    {
        printf("Failed to join Wi-Fi network '%s'\n", ssid);
        CY_ASSERT(0);
    }

    /* Add the Wi-Fi interface to the lwIP stack */
    result = add_interface_to_lwip(iface, NULL);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi interface added to TCP/IP stack\n");
    }
    else
    {
        printf("Failed to add Wi-Fi interfce to lwIP stack!\n");
        CY_ASSERT(0);
    }

    /* Fetch the IP address assigned based on the added Wi-Fi interface */
    net = get_lwip_interface();

    /* Wait till IP address is assigned */
    while(net->ip_addr.u_addr.ip4.addr == 0)
    {
        vTaskDelay(RTOS_TASK_TICKS_TO_WAIT);
    }
    printf("IP Address %s assigned\n", ip4addr_ntoa(&net->ip_addr.u_addr.ip4));

    if (SYNC_TYPE == SNTP)
    {
    	/*Initialize the SNTP connection.*/
    	initialize_sntp();
    }

	/* Create a new TCP connection */
	conn = netconn_new(NETCONN_TCP);

	/* Connect to a specific remote IP address and port */
	err = netconn_connect(conn, &remote, TCP_SERVER_PORT);

	if(err != ERR_OK)
	{
		printf("netconn_connect returned error. Error code: %d\n", err);
		CY_ASSERT(0);
	}
}

/* Close the TCP connection and free its resources */
void close_tcp_connection()
{
	/* Close the TCP connection and free its resources */
	err_t err = netconn_delete(conn);

	if(err == ERR_OK)
	{
		printf("Info: Connection closed.\n");
	}
	else
	{
		printf("netconn_delete returned error. Error code: %d\n", err);
		CY_ASSERT(0);
	}
}

/*Sets up the Data Ready interrupt pin.*/
void setup_drdy_interrupt()
{
    cy_rslt_t rslt;
    /* Initialize pin ADC_DRDY as an input pin */
    rslt = cyhal_gpio_init(ADC_DRDY, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    /* Register callback function - gpio_interrupt_handler and pass the value global_count */
    cyhal_gpio_register_callback(ADC_DRDY, drdy_interrupt_handler, NULL);
    /* Enable falling edge interrupt event with interrupt priority set to 3 */
    cyhal_gpio_enable_event(ADC_DRDY, CYHAL_GPIO_IRQ_FALL, 7, true);
}

/*Initializes the ADC and sets it into RDATAC mode.*/
void adc_startup_procedure()
{
    // Initialize GPIO as an output with strong drive mode and initial value = false (low).
    cyhal_gpio_init(ADC_PDWN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(ADC_RST, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(ADC_START, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // Set ADC pins default status.
    cyhal_gpio_write(ADC_PDWN, true);
    cyhal_gpio_write(ADC_RST, true);
    cyhal_gpio_write(ADC_START, false);

    // Delay for Power-On Reset and Oscillator Start-Up.
    Cy_SysLib_Delay(500);

    // Issue Reset Pulse, wait for 18 t_clk.
    cyhal_gpio_write(ADC_RST, false);
    Cy_SysLib_Delay(20);
    cyhal_gpio_write(ADC_RST, true);
    Cy_SysLib_Delay(500);

    // Declare the command and receive SPI buffers.
    uint8_t command[3] = {0x00 , 0x00, 0x00};

    // Send 16 dummy bytes to have 11 + 16 = 27
    command[0] = 0;
    for (int k = 0; k < ADC_BYTES_PER_FRAME - 11; k++)
    {
    	send_command(command, 1, 1, 0);
    	Cy_SysLib_DelayUs(5);
    }

    // Send SDATAC command.
    command[0] = SDATAC;
    send_command(command, 1, 1, 0);
    Cy_SysLib_Delay(10);

    // Set internal reference.
    command[0] = WREG | CONFIG3_START;
    command[2] = CONFIG3;
	send_command(command, 3, 1, 5);
    Cy_SysLib_Delay(400);

    command[0] = WREG | CONFIG1_START;
    command[2] = CONFIG1;
    send_command(command, 3, 1, 5);
    Cy_SysLib_Delay(10);

    command[0] = WREG | CONFIG2_START;
    command[2] = CONFIG2;
    send_command(command, 3, 1, 5);
    Cy_SysLib_Delay(10);

    // Set Start = 1
    cyhal_gpio_write(ADC_START, true);

    // Send RDATAC
    command[0] = RDATAC;
    send_command(command, 1, 1, 0);
    Cy_SysLib_Delay(10);
}

