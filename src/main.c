/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "settings.h"
#include "utils.h"
#include "led.h"
#include "flow.h"
#include "dcmi.h"
#include "mt9v034.h"
#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "sonar.h"
#include "communication.h"
#include "debug.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "main.h"

/* coprocessor control register (fpu) */
#ifndef SCB_CPACR
#define SCB_CPACR (*((uint32_t*) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif


/* prototypes */
void delay(unsigned msec);
void buffer_reset(void);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
extern USBD_Class_cb_TypeDef custom_composite_cb;

/* fast image buffers for calculations */
uint8_t image_buffer_8bit_1[FULL_IMAGE_SIZE] __attribute__((section(".ccm")));
uint8_t image_buffer_8bit_2[FULL_IMAGE_SIZE] __attribute__((section(".ccm")));
uint8_t buffer_reset_needed;

/* boot time in milliseconds ticks */
volatile uint32_t boot_time_ms = 0;
/* boot time in 10 microseconds ticks */
volatile uint32_t boot_time10_us = 0;

/* timer constants */
#define NTIMERS         	9
#define TIMER_CIN       	0
#define TIMER_LED       	1
#define TIMER_DELAY     	2
#define TIMER_SONAR			3
#define TIMER_SYSTEM_STATE	4
#define TIMER_RECEIVE		5
#define TIMER_PARAMS		6
#define TIMER_IMAGE			7
#define TIMER_LPOS		8
#define MS_TIMER_COUNT		100 /* steps in 10 microseconds ticks */
#define LED_TIMER_COUNT		500 /* steps in milliseconds ticks */
#define SONAR_TIMER_COUNT 	100	/* steps in milliseconds ticks */
#define SYSTEM_STATE_COUNT	1000/* steps in milliseconds ticks */
#define PARAMS_COUNT		100	/* steps in milliseconds ticks */
#define LPOS_TIMER_COUNT 	100	/* steps in milliseconds ticks */
#define TIMER_IMAGE_COUNT   500

static volatile unsigned timer[NTIMERS];
static volatile unsigned timer_ms = MS_TIMER_COUNT;

/* timer/system booleans */
bool send_system_state_now = true;
bool receive_now = true;
bool send_params_now = true;
bool send_image_now = true;
bool send_lpos_now = true;
volatile bool usb_image_transfer_active = false;

/* local position estimate without orientation, useful for unit testing w/o FMU */
struct lpos_t {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
} lpos = {0};

/**
  * @brief  Increment boot_time_ms variable and decrement timer array.
  * @param  None
  * @retval None
  */
void timer_update_ms(void)
{
	boot_time_ms++;

	/* each timer decrements every millisecond if > 0 */
	for (unsigned i = 0; i < NTIMERS; i++)
		if (timer[i] > 0)
			timer[i]--;


	if (timer[TIMER_LED] == 0)
	{
		/* blink activitiy */
		LEDToggle(LED_ACT);
		timer[TIMER_LED] = LED_TIMER_COUNT;
	}

	if (timer[TIMER_SONAR] == 0)
	{
		sonar_trigger();
		timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	}

	if (timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT;
	}

	if (timer[TIMER_PARAMS] == 0)
	{
		send_params_now = true;
		timer[TIMER_PARAMS] = PARAMS_COUNT;
	}

	if (timer[TIMER_IMAGE] == 0)
	{
		send_image_now = true;
		timer[TIMER_IMAGE] = TIMER_IMAGE_COUNT;
	}

	if (timer[TIMER_LPOS] == 0)
	{
		send_lpos_now = true;
		timer[TIMER_LPOS] = LPOS_TIMER_COUNT;
	}
}

/**
  * @brief  Increment boot_time10_us variable and decrement millisecond timer, triggered by timer interrupt
  * @param  None
  * @retval None
  */
void timer_update(void)
{
	boot_time10_us++;

	/*  decrements every 10 microseconds*/
	timer_ms--;

	if (timer_ms == 0)
	{
		timer_update_ms();
		timer_ms = MS_TIMER_COUNT;
	}
}


uint32_t get_boot_time_ms(void)
{
	return boot_time_ms;
}

uint32_t get_boot_time_us(void)
{
	return boot_time10_us*10;// *10 to return microseconds
}

void delay(unsigned msec)
{
	timer[TIMER_DELAY] = msec;
	while (timer[TIMER_DELAY] > 0) {};
}

void buffer_reset(void) {
	buffer_reset_needed = 1;
}

extern uint8_t usb_image_buffer[USB_IMAGE_PIXELS * USB_IMAGE_PIXELS];
unsigned usb_image_pos = 0;
#define MAX_TRANSFER (1023*64)

void send_image_step() {
	unsigned size = USB_IMAGE_PIXELS*USB_IMAGE_PIXELS - usb_image_pos;
	if (size > MAX_TRANSFER) size = MAX_TRANSFER;

	DCD_EP_Tx(&USB_OTG_dev, IMAGE_IN_EP, &usb_image_buffer[usb_image_pos], size);
	if (size == 0 || size % 64 != 0) {
		// We sent a short packet at the end of the transfer. When it completes, we're done.
		usb_image_pos = -1;
	} else {
		usb_image_pos += size;
	}
}

void start_send_image() {
	LEDOn(LED_COM);
	usb_image_transfer_active = true;
	usb_image_pos = 0;
	DCD_EP_Flush(&USB_OTG_dev, IMAGE_IN_EP);
	send_image_step();
}

void send_image_completed(void) {
	if (usb_image_pos != -1) {
		send_image_step();
	} else {
		usb_image_transfer_active = false;
		LEDOff(LED_COM);
	}
}

int main(void)
{
	/* load settings and parameters */
	global_data_reset_param_defaults();
	global_data_reset();

	/* init led */
	LEDInit(LED_ACT);
	LEDInit(LED_COM);
	LEDInit(LED_ERR);
	LEDOff(LED_ACT);
	LEDOff(LED_COM);
	LEDOff(LED_ERR);

	/* enable FPU on Cortex-M4F core */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

	/* init clock */
	if (SysTick_Config(SystemCoreClock / 100000))/*set timer to trigger interrupt every 10 microsecond */
	{
		/* capture clock error */
		LEDOn(LED_ERR);
		while (1);
	}

	/* init usb */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&custom_composite_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();

	/* enable image capturing */
	enable_image_capture();

	/* gyro config */
	gyro_config();

	/* init and clear fast image buffers */
	for (int i = 0; i < global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]; i++)
	{
		image_buffer_8bit_1[i] = 0;
		image_buffer_8bit_2[i] = 0;
	}

	uint8_t * current_image = image_buffer_8bit_1;
	uint8_t * previous_image = image_buffer_8bit_2;

	/* usart config*/
	usart_init();

    /* i2c config*/
    i2c_init();

	/* sonar config*/
	float sonar_distance_filtered = 0.0f; // distance in meter
	float sonar_distance_raw = 0.0f; // distance in meter
	bool distance_valid = false;
	sonar_config();

	/* reset/start timers */
	timer[TIMER_SONAR] = SONAR_TIMER_COUNT;
	timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	timer[TIMER_RECEIVE] = SYSTEM_STATE_COUNT / 2;
	timer[TIMER_PARAMS] = PARAMS_COUNT;
	timer[TIMER_IMAGE] = global_data.param[PARAM_VIDEO_RATE];

	/* variables */
	uint32_t counter = 0;
	uint8_t qual = 0;

	/* bottom flow variables */
	float pixel_flow_x = 0.0f;
	float pixel_flow_y = 0.0f;
	float pixel_flow_x_sum = 0.0f;
	float pixel_flow_y_sum = 0.0f;
	float velocity_x_sum = 0.0f;
	float velocity_y_sum = 0.0f;
	float velocity_x_lp = 0.0f;
	float velocity_y_lp = 0.0f;
	int valid_frame_count = 0;
	int pixel_flow_count = 0;

	static float accumulated_flow_x = 0;
	static float accumulated_flow_y = 0;
	static float accumulated_gyro_x = 0;
	static float accumulated_gyro_y = 0;
	static float accumulated_gyro_z = 0;
	static uint16_t accumulated_framecount = 0;
	static uint16_t accumulated_quality = 0;
	static uint32_t integration_timespan = 0;
	static uint32_t lasttime = 0;
	uint32_t time_since_last_sonar_update= 0;


	/* main loop */
	while (1)
	{

		/* forward flow from other sensors */
		if (counter % 2)
		{
			communication_receive_forward();
		}

		/* send system state, receive commands */
		if (send_system_state_now)
		{
			/* every second */
			if (global_data.param[PARAM_SYSTEM_SEND_STATE])
			{
				communication_system_state_send();
			}
			send_system_state_now = false;
		}

		/* receive commands */
		if (receive_now)
		{
			/* test every second */
			communication_receive();
			communication_receive_usb();
			receive_now = false;
		}

		/* sending debug msgs and requested parameters */
		if (send_params_now)
		{
			debug_message_send_one();
			communication_parameter_send();
			send_params_now = false;
		}

		/* send local position estimate, for testing only, doesn't account for heading */
		if (send_lpos_now)
		{
			if (global_data.param[PARAM_SYSTEM_SEND_LPOS])
			{
				mavlink_msg_local_position_ned_send(MAVLINK_COMM_2, timer_ms, lpos.x, lpos.y, lpos.z, lpos.vx, lpos.vy, lpos.vz);
			}
			send_lpos_now = false;
		}

		/*  transmit raw 8-bit image */
		if (send_image_now && !usb_image_transfer_active)
		{
			capture_one_frame();
			start_send_image();
			send_image_now = false;
		}
		else if (!global_data.param[PARAM_USB_SEND_VIDEO])
		{
			LEDOff(LED_COM);
		}
	}
}
