#include "cyhal.h"
#include "timer_config.h"
/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "types.h"
#include "main_include.h"

static void isr_timer(void *callback_arg, cyhal_timer_event_t event);

bool timer_interrupt_flag = false;
/* Timer object used for blinking the LED */
cyhal_timer_t tcp_send_timer;
volatile node_time_t node_time;
volatile uint32_t sampling_period_cnt = 0;
volatile uint32_t do_sync_timer = 0;

void timer_init(void)
{
	cy_rslt_t result;

	node_time.seconds = 0;
	node_time.microseconds = 0;

    const cyhal_timer_cfg_t led_blink_timer_cfg = \
    {
        .compare_value = 0,                  /* Timer compare value, not used */
        .period = TCP_TIMER_PERIOD,          /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,     /* Timer counts up */
        .is_compare = false,                 /* Don't use compare mode */
        .is_continuous = true,               /* Run timer indefinitely */
        .value = 0                           /* Initial value of counter */
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
    sampling_period_cnt = sampling_period_cnt + TCP_TIMER_PERIOD;
    do_sync_timer = do_sync_timer + TCP_TIMER_PERIOD;

    if ((do_sync_timer >= SYNC_INTERVAL_US) && (SYNC_TYPE == TPSN))
    {
    	do_tpsn_sync = 1;
    	do_sync_timer = 0;
    }

    node_time.seconds = node_time.seconds + (node_time.microseconds + TCP_TIMER_PERIOD) / 1000000;
    node_time.microseconds = (node_time.microseconds + TCP_TIMER_PERIOD) % 1000000;

    if ((sampling_period_cnt >= SAMPLING_PERIOD_US) && (USE_ADC == 0))
    {
    	sampling_period_cnt = 0;

    	if (SYNC_TYPE == SNTP)
    	{
    		compute_sntp_timestamps();
    	}

        tcp_data_packet_t tcp_packet;

    	BaseType_t xHigherPriorityTaskWoken;
		/* We have not woken a task at the start of the ISR. */
		xHigherPriorityTaskWoken = pdFALSE;

		// Associate a voltage of 100 mV to a digital value of 1.
		// 100 000 / 0.044
		uint32_t sample_100mV = 2272727;

		/* Read the logic level on the input pin */
		bool read_val = cyhal_gpio_read(DIG_IN);

		volatile uint8_t level = (uint8_t)read_val;
		tcp_packet.data[5] = level * (sample_100mV & 0x000000FF);
		tcp_packet.data[4] = level * ((sample_100mV & 0x0000FF00) >> 8);
		tcp_packet.data[3] = level * ((sample_100mV & 0x00FF0000) >> 16);

		if(tcp_task_started)
		{
			/* Send TCP data packet to the tcp_client_task */
			xQueueSendToBackFromISR(tcp_client_queue, &tcp_packet, &xHigherPriorityTaskWoken);
		}

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* [] END OF FILE */


