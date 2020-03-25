#include "cyhal.h"
#include "timer_config.h"
/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


static void isr_timer(void *callback_arg, cyhal_timer_event_t event);


bool timer_interrupt_flag = false;
/* Timer object used for blinking the LED */
cyhal_timer_t tcp_send_timer;

/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'led_blink_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
 void timer_init(void)
 {
	cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = \
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = TCP_TIMER_PERIOD,         /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,             /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&tcp_send_timer, (cyhal_gpio_t) NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&tcp_send_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&tcp_send_timer, TCP_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&tcp_send_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&tcp_send_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT \
                               , 5, true);

    /* Start the timer with the configured settings */
    start_timer();
 }

 void start_timer()
 {
	    /* Start the timer with the configured settings */
	    cyhal_timer_start(&tcp_send_timer);
 }

 void stop_timer()
 {
	    /* STOP the timer with the configured settings */
	    cyhal_timer_stop(&tcp_send_timer);
 }

 extern TaskHandle_t networkTaskHandle;
 extern volatile bool tcp_client_started;
 uint32_t dummy = 0;
/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
* 	callback_arg	Arguments passed to the interrupt callback
*	event			Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

	/* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;

//    if (tcp_client_started)
//    {
//		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
//
//		/* Notify the led_task about the change in LED state */
//		xTaskNotifyFromISR(networkTaskHandle, dummy, eSetValueWithoutOverwrite,
//						   &xHigherPriorityTaskWoken);
//
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//    }
}

/* [] END OF FILE */


