/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cybsp_wifi.h"
#include "cy_retarget_io.h"

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

#include "network_credentials.h"
#include "timer_config.h"
#include "SPIMaster.h"
#include "SpiDma.h"

/******************************************************************************
* Macros
******************************************************************************/
#define MAX_CONNECTION_RETRIES         (10u)

#define MAKE_IPV4_ADDRESS(a, b, c, d)  ((((uint32_t) d) << 24) | \
                                       (((uint32_t) c) << 16) | \
                                       (((uint32_t) b) << 8) |\
                                       ((uint32_t) a))

#define TCP_SERVER_IP_ADDRESS          MAKE_IPV4_ADDRESS(192, 168, 137, 1)
#define TCP_SERVER_PORT      		   50007
#define TCP_SERVER_HOSTNAME  		   "mytcpsecureserver"

#define TCP_CLIENT_TASK_STACK_SIZE     (1024*5)
#define TCP_CLIENT_TASK_PRIORITY       (1)
#define TCP_CLIENT_TASK_QUEUE_LEN      (10)
#define CLIENT_TASK_Q_TICKS_TO_TIMEOUT (100)
#define RTOS_TASK_TICKS_TO_WAIT        (100)

/*ADS1298 defines*/
#define mSPI_MOSI				(P6_0)
#define mSPI_MISO				(P6_1)
#define mSPI_SCLK				(P6_2)
#define mSPI_SS					(P6_3)

#define CONFIG1_START           (0X01)
#define CONFIG2_START           (0x02)
#define CONFIG3_START           (0x03)

#define WREG                    (0x40)
#define SDATAC                  (0x11)
#define RDATAC                  (0x10)
#define CONFIG1                 (0x85)
#define CONFIG2                 (0x00)
#define CONFIG3                 (0xC0)

#define ADS1298_PDWN            (P9_0)
#define ADS1298_RST             (P9_1)
#define ADS1298_START           (P9_2)
#define ADS1298_DRDY            (P0_2)

#define ADS1298_BYTES_PER_FRAME (27)
/******************************************************************************
* Function Prototypes
******************************************************************************/
void tcp_client_task(void *arg);
void SPI_send(uint8_t* transfer_buf, uint8_t* receive_buf, uint32_t size);
void ADS1298_StartUp_Procedure();
void setup_DRDY_interrupt();
void DRDY_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event);
void init_tcp_client();

/******************************************************************************
* Global Variables
******************************************************************************/
/* Data structure to TCP data and data length */
typedef struct
{
    char text[150];
    uint8_t len;
}tcp_data_packet_t;

tcp_data_packet_t tcp_pkt_buf;
/* Handle of the Queue to send TCP data packets */
QueueHandle_t tcp_client_queue;
/* The primary WIFI driver  */
whd_interface_t iface ;
/* Connection */
struct netconn *conn;
/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;
const size_t tcp_server_cert_len = sizeof( tcp_server_cert );
volatile bool DRDY_received = false;
uint8_t transmit_data[ADS1298_BYTES_PER_FRAME];
uint8_t receive_data[ADS1298_BYTES_PER_FRAME];

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

    // Magic that makes the DMA work (both RX and TX)!
	Cy_TrigMux_Connect(TRIG0_IN_TR_GROUP13_OUTPUT13, TRIG0_OUT_CPUSS_DW0_TR_IN3, false, TRIGGER_TYPE_LEVEL);
	Cy_TrigMux_Connect(TRIG0_IN_TR_GROUP13_OUTPUT5, TRIG0_OUT_CPUSS_DW0_TR_IN2, false, TRIGGER_TYPE_LEVEL);
	Cy_TrigMux_Connect(TRIG13_IN_SCB1_TR_RX_REQ, TRIG13_OUT_TR_GROUP0_INPUT40, false, TRIGGER_TYPE_LEVEL);
	Cy_TrigMux_Connect(TRIG13_IN_SCB1_TR_TX_REQ, TRIG13_OUT_TR_GROUP0_INPUT32, false, TRIGGER_TYPE_LEVEL);

	// Init SPI Master
    initMaster();

    // Configure Rx and Tx DMA channels
    ConfigureTxDma(transmit_data);
    ConfigureRxDma(receive_data);

    // Configure the interrupt pin for ADS1298 DRDY signal
    setup_DRDY_interrupt();

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");

    // Must be called after the SPI interface is initialized.
    ADS1298_StartUp_Procedure();

    // Initialize the transmission and reception buffers.
    for (int j = 0; j < ADS1298_BYTES_PER_FRAME; j++)
    {
    	transmit_data[j] = 0;
    	receive_data[j] = 0;
    }

    /* Initialize timer */
    timer_init();

    /* Queue to Receive TCP packets */
    tcp_client_queue = xQueueCreate(TCP_CLIENT_TASK_QUEUE_LEN, sizeof(tcp_data_packet_t));

    xTaskCreate(tcp_client_task, "Network task", TCP_CLIENT_TASK_STACK_SIZE, NULL,
                TCP_CLIENT_TASK_PRIORITY, NULL);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/******************************************************************************
* Functions
******************************************************************************/
/*Initializes the ADS1298 and sets it into RDATAC mode.*/
void ADS1298_StartUp_Procedure()
{
    // Initialize GPIO as an output with strong drive mode and initial value = false (low).
    cyhal_gpio_init(ADS1298_PDWN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(ADS1298_RST, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(ADS1298_START, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    // Set ADS pins default status.
    cyhal_gpio_write(ADS1298_PDWN, true);
    cyhal_gpio_write(ADS1298_RST, true);
    cyhal_gpio_write(ADS1298_START, false);

    // Delay for Power-On Reset and Oscillator Start-Up.
    Cy_SysLib_Delay(500);

    // Issue Reset Pulse, wait for 18 t_clk.
    cyhal_gpio_write(ADS1298_RST, false);
    Cy_SysLib_Delay(20);
    cyhal_gpio_write(ADS1298_RST, true);
    Cy_SysLib_Delay(500);

    // Declare the command and receive SPI buffers.
    uint8_t command[3] = {0x00 , 0x00, 0x00};
    uint8_t recv[1]= {0};

    // Send SDATAC command.
    command[0] = SDATAC;
    SPI_send(command, recv, 1);
    Cy_SysLib_Delay(10);

    // Set internal reference.
    command[0] = WREG | CONFIG3_START;
    command[2] = CONFIG3;
    SPI_send(command, recv, 3);
    Cy_SysLib_Delay(400);

    command[0] = WREG | CONFIG1_START;
    command[2] = CONFIG1;
    SPI_send(command, recv, 3);
    Cy_SysLib_Delay(10);

    command[0] = WREG | CONFIG2_START;
    command[2] = CONFIG2;
    SPI_send(command, recv, 3);
    Cy_SysLib_Delay(10);

    // Set Start = 1
    cyhal_gpio_write(ADS1298_START, true);

    // Send RDATAC
    command[0] = RDATAC;
    SPI_send(command, recv, 1);
    Cy_SysLib_Delay(10);
}


/* Interrupt handler callback function */
void DRDY_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
	DRDY_received = true;
}

/*Sets up the Data Ready interrupt pin.*/
void setup_DRDY_interrupt()
{
    cy_rslt_t rslt;
    /* Initialize pin ADS1298_DRDY as an input pin */
    rslt = cyhal_gpio_init(ADS1298_DRDY, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    /* Register callback function - gpio_interrupt_handler and pass the value global_count */
    cyhal_gpio_register_callback(ADS1298_DRDY, DRDY_interrupt_handler, NULL);
    /* Enable falling edge interrupt event with interrupt priority set to 3 */
    cyhal_gpio_enable_event(ADS1298_DRDY, CYHAL_GPIO_IRQ_FALL, 3, true);
}

/*Send SPI data using the CPU.*/
void SPI_send(uint8_t* transfer_buf, uint8_t* receive_buf, uint32_t size)
{
	Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, transfer_buf, size);
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

/*******************************************************************************
 * Task used to establish a connection to a remote TCP server and send data.
 *******************************************************************************/
void tcp_client_task(void *arg)
{
	init_tcp_client();

    for(;;)
    {
    	if (DRDY_received)
    	{
    		DRDY_received = false;
    		sendPacket();

    		/* Send data over the TCP connection */
    		err_t err = netconn_write(conn, tcp_pkt_buf.text , tcp_pkt_buf.len,
    							NETCONN_NOFLAG);

    		if (err != ERR_OK)
    		{
    			// ERROR
    		}
    	}
    }
 }
