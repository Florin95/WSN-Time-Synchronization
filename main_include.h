#ifndef MAIN_INCLUDE_H_
#define MAIN_INCLUDE_H_

/******************************************************************************
* Macros
******************************************************************************/
/* 0 - SNTP; 1 - TPSN */
#define SNTP                           (0)
#define TPSN                           (1)
#define SYNC_TYPE                      (TPSN)
#define USE_ADC                        (0)

#define DEVICE_ID                      (USE_ADC)
#define TPSN_SYNC_WORD                 (0xABABABAB)
#define ALIGNMENT_WORD                 (0xA5A5A5A5)
#define START_STREAM_CMD               (0xA0A0A0A0)

#define SYNC_INTERVAL                  (15) // seconds

#define DIG_IN                         (P5_0)

#define MAX_CONNECTION_RETRIES         (10u)
#define MAKE_IPV4_ADDRESS(a, b, c, d)  ((((uint32_t) d) << 24) | \
                                       (((uint32_t) c) << 16) | \
                                       (((uint32_t) b) << 8) |\
                                       ((uint32_t) a))

#define TCP_SERVER_IP_ADDRESS          MAKE_IPV4_ADDRESS(192, 168, 1, 3)
#define TCP_SERVER_PORT      		   50007
#define TCP_CLIENT_TASK_STACK_SIZE     (1024*5)
#define TCP_CLIENT_TASK_PRIORITY       (1)
#define TCP_CLIENT_TASK_QUEUE_LEN      (400)
#define CLIENT_TASK_Q_TICKS_TO_TIMEOUT (100)
#define RTOS_TASK_TICKS_TO_WAIT        (100)

/*ADS1298 defines*/
#define ADC_SAMPLING_PERIOD_US  (500)
#define CONFIG1_START           (0X01)
#define CONFIG2_START           (0x02)
#define CONFIG3_START           (0x03)

#define WREG                    (0x40)
#define SDATAC                  (0x11)
#define RDATAC                  (0x10)
#define CONFIG1                 (0x84) // 2KHz
#define SAMPLING_PERIOD         (500)  // us
#define CONFIG2                 (0x00)
#define CONFIG3                 (0xC0)

#define ADS1298_PDWN            (P9_0)
#define ADS1298_RST             (P9_1)
#define ADS1298_START           (P9_2)
#define ADS1298_DRDY            (P0_2)

#define ADS1298_DEBUG           (P9_5)

/******************************************************************************
* Function Prototypes
******************************************************************************/
void tcp_client_task(void *arg);
void data_received_task(void *arg);
void ads1298_startup_procedure();
void setup_drdy_interrupt();
void drdy_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event);
void init_tcp_client();
void initialize_sntp(void);
void compute_sntp_timestamps();

/******************************************************************************
* Variables
******************************************************************************/
extern volatile bool tcp_task_started;
extern volatile QueueHandle_t tcp_client_queue;

#endif /* MAIN_INCLUDE_H_ */
