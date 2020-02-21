/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cybsp_wifi.h"
#include "cy_retarget_io.h"
#include "string.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* lwIP header files */
#include <lwip/tcpip.h>
#include <lwip/api.h>
#include <lwipinit.h>
#include <mbedtlsinit.h>
#include "ip4_addr.h"

/* mbedTLS header files */
#include "mbedtls_net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#include "mbedtls/config.h"

#include "network_credentials.h"
#include "timer_config.h"
#include "SpiDma.h"
#include "SPIMaster.h"
#include "SPISlave.h"
#include "Interface.h"

/******************************************************************************
* Macros
******************************************************************************/
#define MAX_CONNECTION_RETRIES            (10u)

#define MAKE_IPV4_ADDRESS(a, b, c, d)     ((((uint32_t) d) << 24) | \
                                          (((uint32_t) c) << 16) | \
                                          (((uint32_t) b) << 8) |\
                                          ((uint32_t) a))

/* Change the server IP address to match the TCP echo server address (IP address
 * of the PC)
 */
#define TCP_SERVER_IP_ADDRESS             MAKE_IPV4_ADDRESS(192, 168, 137, 1)

#define TCP_SERVER_PORT      50007
#define TCP_SERVER_HOSTNAME  "mytcpsecureserver"

/* 32-bit task notification value for the led_task */
#define LED_ON                            (0x00lu)
#define LED_OFF                           (0x01lu)

#define USER_BTN1_INTR_PRIORITY           (5)

#define LED_TASK_STACK_SIZE               (128)
#define TCP_CLIENT_TASK_STACK_SIZE        (1024*5)
#define LED_TASK_PRIORITY                 (1)
#define TCP_CLIENT_TASK_PRIORITY          (1)
#define TCP_CLIENT_TASK_QUEUE_LEN         (10)
#define CLIENT_TASK_Q_TICKS_TO_TIMEOUT    (100)
#define RTOS_TASK_TICKS_TO_WAIT           (100)

/* Size is in bytes. */
#define NUMBER_OF_SAMPLES				  (480)
#define PROTOCOL_OVERHEAD                 (4+4+4)

/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS	(5UL)
#define SIZE_OF_ELEMENT		(4UL)
#define SIZE_OF_PACKET		(NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)

/* Delay between successive SPI Master command transmissions */
#define CMD_DELAY			(1000)	//in milliseconds

/* Handle of the Queue to send TCP data packets */
QueueHandle_t tcp_client_queue;

/* Data structure to TCP data and data length */
typedef struct
{
	/* Buffer to hold SPI data to be sent via Wi-Fi */
	uint8_t txBuffer[NUMBER_OF_SAMPLES + PROTOCOL_OVERHEAD];
    uint16_t len;
}tcp_data_packet_t;

/******************************************************************************
* Function Prototypes
******************************************************************************/
void tcp_client_task(void *arg);
void handle_error(void);

/******************************************************************************
* Global Variables
******************************************************************************/
/* The primary WIFI driver  */
whd_interface_t iface ;

/* 32-bit task notification value containing the LED state */
uint32_t led_state = LED_OFF;

/* LED task handle */
TaskHandle_t led_task_handle;

/* Handle of the Queue to send TCP data packets */
QueueHandle_t tcpClientQ;

/* Connection */
struct netconn *conn;

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority ;
const size_t tcp_server_cert_len = sizeof( tcp_server_cert );
tcp_data_packet_t tcp_pkt_buf;

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function sets up user tasks and then starts
 *  the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main()
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Set up internal routing, pins, and clock-to-peripheral connections */
	init_cycfg_all();

	uint32 status = 0;

    /* Initialize the SPI Slave */
	status = initSlave();
	if(status == INIT_FAILURE)
	{
		/* NOTE: This function will block the CPU forever */
		handle_error();
	}

	/* Initialize the SPI Master */
    status = initMaster();
    if(status == INIT_FAILURE)
	{
		/* NOTE: This function will block the CPU forever */
		handle_error();
	}

//    status = ConfigureTxDma((uint32_t*)tcp_pkt_buf.txBuffer);
//    if(status == INIT_FAILURE)
//	{
//		/* NOTE: This function will block the CPU forever */
//		handle_error();
//	}

    /* Initialize the board support package */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("============================================================\n");
    printf("CE229112 - ModusToolbox Connectivity Example: TCP Client\n");
    printf("============================================================\n\n");

    /* Initialize timer to trigger TCP data transmission. */
    timer_init();


	/* Buffer to hold command packet to be sent to the slave by the master */
	uint8  txBuffer[5];

	/* Buffer to save the received data by the slave */
	uint32 rxBuffer[5];

	int nr = 0;

    for(;;)
    {
    	/* Form the command packet */
    	txBuffer[0] = 64;
    	txBuffer[1] = 65;
    	txBuffer[2] = 66;
    	txBuffer[3] = 67;
    	txBuffer[4] = 68;

    	/* Pass the command packet to the master along with the number of bytes to be
    	 * sent to the slave.*/
		sendPacket(txBuffer);

		printf("%d. Send Packet done!\n", nr);
		nr++;

		//status = checkTranferStatus();
		status = TRANSFER_COMPLETE;

		/* Check whether the master succeeded in transferring data */
		if(status == TRANSFER_COMPLETE)
		{
			printf("TRANSFER to slave COMPLETE!\n");
			/* The below code is for slave function. It is implemented in this same code
			 * example so that the master function can be tested without the need of one
			 * more kit. */

			/* Get the bytes received by the slave */
			status = readPacket(rxBuffer, SIZE_OF_PACKET);

			/* Check whether the slave succeeded in receiving the required number of bytes
			 * and in the right format */
			if(status == TRANSFER_COMPLETE)
			{
				/* Communication succeeded. */
				printf(" %lu \n", rxBuffer[PACKET_CMD_POS]);
			}
			else
			{
				/* Communication failed */
				handle_error();
			}
		}
		else
		{
			/* Communication failed */
			handle_error();
		}

		/* Give delay before initiating the next command */
		Cy_SysLib_Delay(CMD_DELAY/10);
    }




    /* Queue to Receive TCP packets */
    tcp_client_queue = xQueueCreate(TCP_CLIENT_TASK_QUEUE_LEN, sizeof(tcp_data_packet_t));

    xTaskCreate(tcp_client_task, "Network task", TCP_CLIENT_TASK_STACK_SIZE, NULL,
                TCP_CLIENT_TASK_PRIORITY, NULL);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
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

/*******************************************************************************
 * Function Name: tcp_client_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote TCP server and send
 *  LED ON/OFF state to the TCP server
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tcp_client_task(void *arg)
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

	/* Create a new TCP connection */
	conn = netconn_new(NETCONN_TCP);

	/* Connect to a specific remote IP address and port */
	err = netconn_connect(conn, &remote, TCP_SERVER_PORT);

	if(err != ERR_OK)
	{
		printf("netconn_connect returned error. Error code: %d\n", err);
		CY_ASSERT(0);
	}

    for(;;)
    {
    	if (timer_interrupt_flag)
    	{
    		timer_interrupt_flag = false;

    		/* Send data over the TCP connection */
    		err = netconn_write(conn, tcp_pkt_buf.txBuffer , tcp_pkt_buf.len,
    							NETCONN_NOFLAG);
    	}
    }
 }

/******************************************************************************
* Function Name: handle_error
*******************************************************************************
*
* Summary: 		This is a blocking function. It disables the interrupt and waits
* 				in an infinite loop. This function is called when an error is
* 				encountered during initialization of the blocks or during
* 				SPI communication.
*
* Parameters: 	None
*
* Return:		None
*
******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}
}
