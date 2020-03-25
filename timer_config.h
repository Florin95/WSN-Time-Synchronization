#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_


/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz  */
#define TCP_TIMER_CLOCK_HZ          (1000000)

/* LED blink timer period value */
#define TCP_TIMER_PERIOD            (500)


/*******************************************************************************
* Global Variables
*******************************************************************************/
extern bool timer_interrupt_flag;
/* Timer object used for blinking the LED */
extern cyhal_timer_t tcp_send_timer;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
void start_timer();
void stop_timer();

#endif /* TIMER_CONFIG_H_ */
