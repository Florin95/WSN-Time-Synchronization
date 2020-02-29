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

/* Delay between successive SPI Master command transmissions */
#define CMD_DELAY			(1000)	//in milliseconds

/* LED States. LEDs in the supported kits are in active low connection. */
#define LED_ON 				(0)
#define LED_OFF				(1)

/* Delay between successive SPI Master command transmissions */
#define CMD_DELAY			(1000)	//in milliseconds

/* Number of elements in the transmit and receive buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS	(3UL)
#define SIZE_OF_ELEMENT		(4UL)
#define SIZE_OF_PACKET		(NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)


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

#define sSPI_MOSI				(P10_0)
#define sSPI_MISO				(P10_1)
#define sSPI_SCLK				(P10_2)
#define sSPI_SS					(P10_3)

#define mSPI_MOSI				(P6_0)
#define mSPI_MISO				(P6_1)
#define mSPI_SCLK				(P6_2)
#define mSPI_SS					(P6_3)

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
//    cy_rslt_t result;
//
//    /* This enables RTOS aware debugging in OpenOCD */
//    uxTopUsedPriority = configMAX_PRIORITIES - 1;
//
//    /* Set up internal routing, pins, and clock-to-peripheral connections */
//	init_cycfg_all();
//
//	uint32 status = 0;
//
//    /* Initialize the SPI Slave */
//	status = initSlave();
//	if(status == INIT_FAILURE)
//	{
//		/* NOTE: This function will block the CPU forever */
//		handle_error();
//	}
//
//	/* Initialize the SPI Master */
//    status = initMaster();
//    if(status == INIT_FAILURE)
//	{
//		/* NOTE: This function will block the CPU forever */
//		handle_error();
//	}
//
////    status = ConfigureTxDma((uint32_t*)tcp_pkt_buf.txBuffer);
////    if(status == INIT_FAILURE)
////	{
////		/* NOTE: This function will block the CPU forever */
////		handle_error();
////	}
//
//    /* Initialize the board support package */
//    result = cybsp_init() ;
//    CY_ASSERT(result == CY_RSLT_SUCCESS);
//
//    /* Enable global interrupts */
//    __enable_irq();
//
//    /* Initialize retarget-io to use the debug UART port */
//    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
//                        CY_RETARGET_IO_BAUDRATE);
//
//    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
//    printf("\x1b[2J\x1b[;H");
//    printf("============================================================\n");
//    printf("CE229112 - ModusToolbox Connectivity Example: TCP Client\n");
//    printf("============================================================\n\n");
//
//    /* Initialize timer to trigger TCP data transmission. */
//    timer_init();
//
//	/* Buffer to hold command packet to be sent to the slave by the master */
//	uint32  txBuffer[NUMBER_OF_ELEMENTS];
//
//	/* Buffer to save the received data by the slave */
//	uint32  rxBuffer[NUMBER_OF_ELEMENTS];
//
//
//	/* Local command variable */
//	uint32 cmd = LED_OFF;

    cy_rslt_t result;
    /* Set up the device based on configurator selections */
    result = cybsp_init();
	if(result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if(result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**************************\r\n");
    printf("PSoC 6 MCU SPI Master\r\n");
    printf("**************************\r\n\n");

    cyhal_spi_t mSPI;
    cyhal_spi_t sSPI;

    /* Configure user LED */
    printf(">> Configure user LED \r\n");
    result = cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	if(result != CY_RSLT_SUCCESS)
	{
		handle_error();
	}

    /* Configure SPI Master */
    printf(">> Configure SPI master \r\n");
    result = cyhal_spi_init(&mSPI, mSPI_MOSI, mSPI_MISO, mSPI_SCLK, mSPI_SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }
    result = cyhal_spi_set_frequency(&mSPI, 1000000);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }

    /* Configure SPI Slave */
    printf(">> Configure SPI slave \r\n\n");
    result = cyhal_spi_init(&sSPI, sSPI_MOSI, sSPI_MISO, sSPI_SCLK, sSPI_SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, true);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }
    result = cyhal_spi_set_frequency(&sSPI, 1000000);
    if(result != CY_SCB_SPI_SUCCESS)
    {
    	handle_error();
    }

    uint32_t cmd_send = true;
    uint32_t cmd_recv = false;

    /* Enable interrupts */
    __enable_irq();

    printf("User LED should start blinking \r\n");

    for(;;)
    {
    	/* Toggle the current LED state */
    	cmd_send = (cmd_send == true) ? false : true;

    	/* Send packet with command to the slave. */
        if (CY_RSLT_SUCCESS == cyhal_spi_send(&mSPI, cmd_send))
		{
        	/* The below code is for slave function. It is implemented in
        	   this code example so that the master function can be tested
        	   without the need of one more kit. */

			/* Read command packet at slave */
			if (CY_SCB_I2C_SUCCESS == cyhal_spi_recv(&sSPI, &cmd_recv))
			{
				/* Execute command */
				cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED, cmd_recv);
			}
			else
			{
				handle_error();
			}
		}
        else
        {
        	handle_error();
        }
		/* Give delay between commands. */
		Cy_SysLib_Delay(CMD_DELAY);
    }




//    /* Queue to Receive TCP packets */
//    tcp_client_queue = xQueueCreate(TCP_CLIENT_TASK_QUEUE_LEN, sizeof(tcp_data_packet_t));
//
//    xTaskCreate(tcp_client_task, "Network task", TCP_CLIENT_TASK_STACK_SIZE, NULL,
//                TCP_CLIENT_TASK_PRIORITY, NULL);
//
//    /* Start the FreeRTOS scheduler */
//    vTaskStartScheduler();
//
//    /* Should never get here */
//    CY_ASSERT(0);
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
