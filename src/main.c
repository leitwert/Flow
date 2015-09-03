/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *           Simon Laube <simon@leitwert.ch>
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
#include "filter.h"
#include "result_accumulator.h"
#include "flow.h"
#include "timer.h"
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


__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/* timer constants */
#define SONAR_POLL_MS	 	100	/* steps in milliseconds ticks */
#define SYSTEM_STATE_MS		1000/* steps in milliseconds ticks */
#define PARAMS_MS			100	/* steps in milliseconds ticks */
#define LPOS_TIMER_COUNT 	100	/* steps in milliseconds ticks */

/* local position estimate without orientation, useful for unit testing w/o FMU */
struct lpos_t {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
} lpos = {0};

static camera_ctx cam_ctx;
static camera_img_param img_stream_param;

uint8_t snapshot_buffer_mem[128 * 128];

static camera_image_buffer snapshot_buffer;
	
void sonar_update_fn(void) {
	sonar_trigger();
}

void system_state_send_fn(void) {
	/* every second */
	if (global_data.param[PARAM_SYSTEM_SEND_STATE])
	{
		communication_system_state_send();
	}
}

void system_receive_fn(void) {
	/* test every 0.5s */
	communication_receive();
	communication_receive_usb();
}

const camera_image_buffer *previous_image = NULL;

void mavlink_send_image(const camera_image_buffer *image) {
	uint32_t img_size = (uint32_t)image->param.p.size.x * (uint32_t)image->param.p.size.y;
	mavlink_msg_data_transmission_handshake_send(
			MAVLINK_COMM_2,
			MAVLINK_DATA_STREAM_IMG_RAW8U,
			img_size,
			image->param.p.size.x,
			image->param.p.size.y,
			img_size / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
			MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
			100);
	uint16_t frame = 0;
	for (frame = 0; frame < img_size / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1; frame++) {
		mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, &((uint8_t *)image->buffer)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
	}
}

void send_video_fn(void) {
	/* update the rate */
	timer_update(send_video_fn, global_data.param[PARAM_VIDEO_RATE]);

	if (previous_image == NULL) return;
	
	/*  transmit raw 8-bit image */
	if (global_data.param[PARAM_USB_SEND_VIDEO] && !global_data.param[PARAM_VIDEO_ONLY])
	{
		mavlink_send_image(previous_image);
		LEDToggle(LED_COM);
	}
	else if (!global_data.param[PARAM_USB_SEND_VIDEO])
	{
		LEDOff(LED_COM);
	}
}

void send_params_fn(void) {
	debug_message_send_one();
	communication_parameter_send();
}

/*void switch_params_fn(void) {
	switch (img_stream_param.binning) {
		case 1: img_stream_param.binning = 4; break;
		case 2: img_stream_param.binning = 1; break;
		case 4: img_stream_param.binning = 2; break;
	}
	camera_img_stream_schedule_param_change(&cam_ctx, &img_stream_param);
}*/

static volatile bool snap_capture_done = false;
static volatile bool snap_capture_success = false;
static bool snap_ready = true;

void snapshot_captured_fn(bool success) {
	snap_capture_done = true;
	snap_capture_success = success;
}

void take_snapshot_fn(void) {
	if (global_data.param[PARAM_USB_SEND_VIDEO] && global_data.param[PARAM_VIDEO_ONLY] && snap_ready) {
		static camera_img_param snapshot_param;
		snapshot_param.size.x = 128;
		snapshot_param.size.y = 128;
		snapshot_param.binning = 1;
		if (camera_snapshot_schedule(&cam_ctx, &snapshot_param, &snapshot_buffer, snapshot_captured_fn)) {
			snap_ready = false;
		}
	}
}

void notify_changed_camera_parameters() {
	camera_reconfigure_general(&cam_ctx);
}

void notify_changed_image_parameters() {
	uint16_t bin = global_data.param[PARAM_IMAGE_BINNING];
	switch (bin) {
		case 1:
		case 2:
		case 4: img_stream_param.binning = bin; break;
		default: return;
	}
	camera_img_stream_schedule_param_change(&cam_ctx, &img_stream_param);
}

#define FLOW_IMAGE_SIZE (64)

static uint8_t image_buffer_8bit_1[FLOW_IMAGE_SIZE * FLOW_IMAGE_SIZE] __attribute__((section(".ccm")));
static uint8_t image_buffer_8bit_2[FLOW_IMAGE_SIZE * FLOW_IMAGE_SIZE] __attribute__((section(".ccm")));
static uint8_t image_buffer_8bit_3[FLOW_IMAGE_SIZE * FLOW_IMAGE_SIZE] __attribute__((section(".ccm")));
static uint8_t image_buffer_8bit_4[FLOW_IMAGE_SIZE * FLOW_IMAGE_SIZE] __attribute__((section(".ccm")));
static uint8_t image_buffer_8bit_5[FLOW_IMAGE_SIZE * FLOW_IMAGE_SIZE] __attribute__((section(".ccm")));

static flow_klt_image flow_klt_images[2] __attribute__((section(".ccm")));

/**
  * @brief  Main function.
  */
int main(void)
{
	__enable_irq();
	snapshot_buffer = BuildCameraImageBuffer(snapshot_buffer_mem);

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

	/* init timers */
	timer_init();

	/* init usb */
	USBD_Init(	&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb);

	/* init mavlink */
	communication_init();

	/* initialize camera: */
	img_stream_param.size.x = FLOW_IMAGE_SIZE;
	img_stream_param.size.y = FLOW_IMAGE_SIZE;
	img_stream_param.binning = (uint16_t)global_data.param[PARAM_IMAGE_BINNING];
	{
		camera_image_buffer buffers[5] = {
			BuildCameraImageBuffer(image_buffer_8bit_1),
			BuildCameraImageBuffer(image_buffer_8bit_2),
			BuildCameraImageBuffer(image_buffer_8bit_3),
			BuildCameraImageBuffer(image_buffer_8bit_4),
			BuildCameraImageBuffer(image_buffer_8bit_5)
		};
		camera_init(&cam_ctx, mt9v034_get_sensor_interface(), dcmi_get_transport_interface(), 
					mt9v034_get_clks_per_row(64, 4) * 1, mt9v034_get_clks_per_row(64, 4) * 64, 4.0,
					&img_stream_param, buffers, 5);
	}

	/* gyro config */
	gyro_config();

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
	timer_register(sonar_update_fn, SONAR_POLL_MS);
	timer_register(system_state_send_fn, SYSTEM_STATE_MS);
	timer_register(system_receive_fn, SYSTEM_STATE_MS / 2);
	timer_register(send_params_fn, PARAMS_MS);
	timer_register(send_video_fn, global_data.param[PARAM_VIDEO_RATE]);
	timer_register(take_snapshot_fn, 500);
	//timer_register(switch_params_fn, 2000);

	/* variables */
	uint32_t counter = 0;

	result_accumulator_ctx mavlink_accumulator;

	result_accumulator_init(&mavlink_accumulator);
	
	uint32_t fps_timing_start = get_boot_time_us();
	uint16_t fps_counter = 0;
	uint16_t fps_skipped_counter = 0;
	
	uint32_t last_frame_index = 0;
	
	uint32_t last_loop_end = 0;
	
	/* main loop */
	while (1)
	{
		uint32_t ts_loop_start;
		uint32_t ts_gyro_and_dist;
		uint32_t ts_get_image;
		uint32_t ts_frame;
		uint32_t ts_preparation;
		uint32_t ts_computation;
		uint32_t ts_result_extract;
		uint32_t ts_result_accu;
		uint32_t ts_finished;
		uint32_t ts_loop_end;
		
		ts_loop_start = get_boot_time_us();
		
		/* check timers */
		timer_check();
		
		if (snap_capture_done) {
			snap_capture_done = false;
			camera_snapshot_acknowledge(&cam_ctx);
			snap_ready = true;
			if (snap_capture_success) {
				/* send the snapshot! */
				LEDToggle(LED_COM);
				mavlink_send_image(&snapshot_buffer);
			}
		}

		ts_gyro_and_dist = get_boot_time_us();
		
		/* new gyroscope data */
		float x_rate_sensor, y_rate_sensor, z_rate_sensor;
		int16_t gyro_temp;
		gyro_read(&x_rate_sensor, &y_rate_sensor, &z_rate_sensor,&gyro_temp);

		/* gyroscope coordinate transformation to flow sensor coordinates */
		float x_rate =   y_rate_sensor; // change x and y rates
		float y_rate = - x_rate_sensor;
		float z_rate =   z_rate_sensor; // z is correct

		/* get sonar data */
		distance_valid = sonar_read(&sonar_distance_filtered, &sonar_distance_raw);
		/* reset to zero for invalid distances */
		if (!distance_valid) {
			sonar_distance_filtered = 0.0f;
			sonar_distance_raw = 0.0f;
		}

		bool use_klt = global_data.param[PARAM_ALGORITHM_CHOICE] != 0;

		ts_get_image = get_boot_time_us();
		
		/* get recent images */
		camera_image_buffer *frames[2];
		camera_img_stream_get_buffers(&cam_ctx, frames, 2, true);
		
		ts_preparation = get_boot_time_us();
		
		int frame_delta = ((int32_t)frames[0]->frame_number - (int32_t)last_frame_index);
		last_frame_index = frames[0]->frame_number;
		fps_skipped_counter += frame_delta - 1;
		
		ts_frame = frames[0]->timestamp;
		
		flow_klt_image *klt_images[2] = {NULL, NULL};
		{
			/* make sure that the new images get the correct treatment */
			/* this algorithm will still work if both images are new */
			int i;
			bool used_klt_image[2] = {false, false};
			for (i = 0; i < 2; ++i) {
				if (frames[i]->frame_number != frames[i]->meta) {
					// the image is new. apply pre-processing:
					/* filter the new image */
					if (global_data.param[PARAM_ALGORITHM_IMAGE_FILTER]) {
						filter_image(frames[i]->buffer, frames[i]->param.p.size.x);
					}
					/* update meta data to mark it as an up-to date image: */
					frames[i]->meta = frames[i]->frame_number;
				} else {
					// the image has the preprocessing already applied.
					if (use_klt) {
						int j;
						/* find the klt image that matches: */
						for (j = 0; j < 2; ++j) {
							if (flow_klt_images[j].meta == frames[i]->frame_number) {
								used_klt_image[j] = true;
								klt_images[i] = &flow_klt_images[j];
							}
						}
					}
				}
			}
			if (use_klt) {
				/* only for KLT: */
				/* preprocess the images if they are not yet preprocessed */
				for (i = 0; i < 2; ++i) {
					if (klt_images[i] == NULL) {
						// need processing. find unused KLT image:
						int j;
						for (j = 0; j < 2; ++j) {
							if (!used_klt_image[j]) {
								used_klt_image[j] = true;
								klt_images[i] = &flow_klt_images[j];
								break;
							}
						}
						klt_preprocess_image(frames[i]->buffer, klt_images[i]);
						klt_images[i]->meta = frames[i]->frame_number;
					}
				}
			}
		}
		
		const float frame_dt = (frames[0]->timestamp - frames[1]->timestamp) * 0.000001f;
		
		/* calculate focal_length in pixel */
		const float focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / 
									  ((float)frames[0]->param.p.binning * 0.006f);	// pixel-size: 6um
		
		ts_computation = get_boot_time_us();
		
		/* extract the raw flow from the images: */
		flow_raw_result flow_rslt[32];
		uint16_t flow_rslt_count = 0;
		float flow_iter_count __attribute__((unused)) = 0;
		float flow_real_comp_t __attribute__((unused)) = 0;
		/* make sure both images are taken with same binning mode: */
		if (frames[0]->param.p.binning == frames[1]->param.p.binning) {
			/* compute gyro rate in pixels and change to image coordinates */
			float x_rate_px = - y_rate * (focal_length_px * frame_dt);
			float y_rate_px =   x_rate * (focal_length_px * frame_dt);
			float z_rate_fr = - z_rate * frame_dt;

			/* compute optical flow in pixels */
			if (!use_klt) {
				flow_rslt_count = compute_flow(frames[1]->buffer, frames[0]->buffer, x_rate_px, y_rate_px, z_rate_fr, 
											   flow_rslt, sizeof(flow_rslt) / sizeof(flow_rslt[0]));
				flow_iter_count = flow_rslt_count;
			} else {
				flow_rslt_count =  compute_klt(klt_images[1], klt_images[0],         x_rate_px, y_rate_px, z_rate_fr, 
											   flow_rslt, sizeof(flow_rslt) / sizeof(flow_rslt[0]));
				if (flow_rslt_count > 0) {
					flow_iter_count = flow_rslt[sizeof(flow_rslt) / sizeof(flow_rslt[0]) - 1].quality;
					flow_real_comp_t = flow_rslt[sizeof(flow_rslt) / sizeof(flow_rslt[0]) - 1].x;
				}
			}
		} else {
			/* no result for this frame. */
			flow_rslt_count = 0;
		}
		/* determine capability: */
		flow_capability flow_rslt_cap;
		if (!use_klt) {
			get_flow_capability(&flow_rslt_cap);
		} else {
			get_flow_klt_capability(&flow_rslt_cap);
		}
		
		ts_result_extract = get_boot_time_us();
		
		/* calculate flow value from the raw results */
		float pixel_flow_x;
		float pixel_flow_y;
		float outlier_threshold = global_data.param[PARAM_ALGORITHM_OUTLIER_THR_RATIO];
		float min_outlier_threshold = 0;
		if(global_data.param[PARAM_ALGORITHM_CHOICE] == 0)
		{
			min_outlier_threshold = global_data.param[PARAM_ALGORITHM_OUTLIER_THR_BLOCK];
		}else
		{
			min_outlier_threshold = global_data.param[PARAM_ALGORITHM_OUTLIER_THR_KLT];
		}
		uint8_t qual = flow_extract_result(flow_rslt, flow_rslt_count, &pixel_flow_x, &pixel_flow_y, 
							outlier_threshold,  min_outlier_threshold);

		/* create flow image if needed (previous_image is not needed anymore)
		 * -> can be used for debugging purpose
		 */
		previous_image = frames[1];
		if (global_data.param[PARAM_USB_SEND_VIDEO])
		{
			uint16_t frame_size = global_data.param[PARAM_IMAGE_WIDTH];
			uint8_t *prev_img = previous_image->buffer;
			for (int i = 0; i < flow_rslt_count; i++) {
				if (flow_rslt[i].quality > 0) {
					prev_img[flow_rslt[i].at_y * frame_size + flow_rslt[i].at_x] = 255;
					int ofs = (int)floor(flow_rslt[i].at_y + flow_rslt[i].y * 2 + 0.5f) * frame_size + (int)floor(flow_rslt[i].at_x + flow_rslt[i].x * 2 + 0.5f);
					if (ofs >= 0 && ofs < frame_size * frame_size) {
						prev_img[ofs] = 200;
					}
				}
			}
		}

		/* return the image buffers */
		camera_img_stream_return_buffers(&cam_ctx, frames, 2);
		
		ts_result_accu = get_boot_time_us();
		
		/* decide which distance to use */
		float ground_distance = 0.0f;

		if(global_data.param[PARAM_SONAR_FILTERED])
		{
			ground_distance = sonar_distance_filtered;
		}
		else
		{
			ground_distance = sonar_distance_raw;
		}

		/* update I2C transmit buffer */
		update_TX_buffer(frame_dt, 
						 x_rate, y_rate, z_rate, gyro_temp, 
						 qual, pixel_flow_x, pixel_flow_y, &flow_rslt_cap, 1.0f / focal_length_px, 
						 distance_valid, ground_distance, get_time_delta_us(get_sonar_measure_time()));

		/* accumulate the results */
		result_accumulator_feed(&mavlink_accumulator, frame_dt, 
								x_rate, y_rate, z_rate, gyro_temp, 
								qual, pixel_flow_x, pixel_flow_y, &flow_rslt_cap, 1.0f / focal_length_px, 
								distance_valid, ground_distance, get_time_delta_us(get_sonar_measure_time()));

		ts_finished = get_boot_time_us();
		
		counter++;
		fps_counter++;

        /* serial mavlink  + usb mavlink output throttled */
		if (counter % (uint32_t)global_data.param[PARAM_FLOW_SERIAL_THROTTLE_FACTOR] == 0)//throttling factor
		{
			float fps = 0;
			float fps_skip = 0;
			if (fps_counter + fps_skipped_counter > 100) {
				uint32_t dt = get_time_delta_us(fps_timing_start);
				fps_timing_start += dt;
				fps = (float)fps_counter / ((float)dt * 1e-6f);
				fps_skip = (float)fps_skipped_counter / ((float)dt * 1e-6f);
				fps_counter = 0;
				fps_skipped_counter = 0;

				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "TIMING", get_boot_time_us(), calculate_time_delta_us(ts_finished, ts_preparation), fps, fps_skip);
			}
			
			mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "EXPOSURE", get_boot_time_us(), 
					frames[0]->param.exposure, frames[0]->param.analog_gain, cam_ctx.last_brightness);
			
			/* calculate the output values */
			result_accumulator_output_flow output_flow;
			result_accumulator_output_flow_rad output_flow_rad;
			int min_valid_ratio = global_data.param[PARAM_ALGORITHM_MIN_VALID_RATIO];
			result_accumulator_calculate_output_flow(&mavlink_accumulator, min_valid_ratio, &output_flow);
			result_accumulator_calculate_output_flow_rad(&mavlink_accumulator, min_valid_ratio, &output_flow_rad);

			static double flow_var_e_x2 = 0;
			static double flow_var_e_x = 0;
			static double flow_var_e_y2 = 0;
			static double flow_var_e_y = 0;
			if (output_flow_rad.integration_time > 0) {
				const double k = 0.95;

				float flow_x = output_flow_rad.integrated_x / (output_flow_rad.integration_time * 1e-6f);
				float flow_y = output_flow_rad.integrated_y / (output_flow_rad.integration_time * 1e-6f);

				flow_var_e_x  = k * flow_var_e_x  + (1.0 - k) *  flow_x;
				flow_var_e_x2 = k * flow_var_e_x2 + (1.0 - k) * (flow_x * flow_x);

				flow_var_e_y  = k * flow_var_e_y  + (1.0 - k) *  flow_y;
				flow_var_e_y2 = k * flow_var_e_y2 + (1.0 - k) * (flow_y * flow_y);

				float var_x = flow_var_e_x2 - flow_var_e_x * flow_var_e_x;
				float var_y = flow_var_e_y2 - flow_var_e_y * flow_var_e_y;
				
				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "VAR", get_boot_time_us(), 
						sqrt(flow_x * flow_x + flow_y * flow_y), 
						sqrt(flow_var_e_x * flow_var_e_x + flow_var_e_y * flow_var_e_y), 
						sqrt(var_x + var_y));
			}
			
			mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "MAXVEL", get_boot_time_us(), 
					mavlink_accumulator.flow_cap_mvx_rad, mavlink_accumulator.flow_cap_mvy_rad, frame_dt);
			
			// calculate RMS:
			{
				uint8_t *img = frames[0]->buffer;
				//int i;
				//int size = (int)frames[0]->param.p.size.x * (int)frames[0]->param.p.size.y;
				float avg = 0;
				int x, y;
				int sx = frames[0]->param.p.size.x;
				for (y = 16; y < 48; y++) {
					for (x = 16; x < 48; x++) {
						avg += img[x + sx * y];
					}
				}
				avg = avg / (float)(32 * 32);
				float rms = 0;
				for (y = 16; y < 48; y++) {
					for (x = 16; x < 48; x++) {
						float d = ((float)img[x + sx * y] - avg);
						rms += d * d;
					}
				}
				rms = sqrt(rms / (float)(32 * 32));
				
				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "RMS", get_boot_time_us(), 
					rms, avg, 0);
			}
			
			// send flow
			mavlink_msg_optical_flow_send(MAVLINK_COMM_0, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
					output_flow.flow_x, output_flow.flow_y,
					output_flow.flow_comp_m_x, output_flow.flow_comp_m_y, 
					output_flow.quality, output_flow.ground_distance);

			mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_0, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
					output_flow_rad.integration_time, 
					output_flow_rad.integrated_x, output_flow_rad.integrated_y,
					output_flow_rad.integrated_xgyro, output_flow_rad.integrated_ygyro, output_flow_rad.integrated_zgyro,
					output_flow_rad.temperature, output_flow_rad.quality,
					output_flow_rad.time_delta_distance_us,output_flow_rad.ground_distance);

			if (global_data.param[PARAM_USB_SEND_FLOW] && (output_flow.quality > 0 || global_data.param[PARAM_USB_SEND_QUAL_0]))
			{
				mavlink_msg_optical_flow_send(MAVLINK_COMM_2, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
						output_flow.flow_x, output_flow.flow_y,
						output_flow.flow_comp_m_x, output_flow.flow_comp_m_y, 
						output_flow.quality, output_flow.ground_distance);

				mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_2, get_boot_time_us(), global_data.param[PARAM_SENSOR_ID],
						output_flow_rad.integration_time, 
						output_flow_rad.integrated_x, output_flow_rad.integrated_y,
						output_flow_rad.integrated_xgyro, output_flow_rad.integrated_ygyro, output_flow_rad.integrated_zgyro,
						output_flow_rad.temperature, output_flow_rad.quality,
						output_flow_rad.time_delta_distance_us,output_flow_rad.ground_distance);
			}

			if(global_data.param[PARAM_USB_SEND_GYRO])
			{
				mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "GYRO", get_boot_time_us(), x_rate, y_rate, z_rate);
			}

			result_accumulator_reset(&mavlink_accumulator);
		}

		/* forward flow from other sensors */
		if (counter % 2)
		{
			communication_receive_forward();
		}
		
		ts_loop_end = get_boot_time_us();
		
		/*uint32_t ts_loop_start;
		uint32_t ts_gyro_and_dist;
		uint32_t ts_get_image;
		uint32_t ts_frame;
		uint32_t ts_preparation;
		uint32_t ts_computation;
		uint32_t ts_result_extract;
		uint32_t ts_result_accu;
		uint32_t ts_finished;
		uint32_t ts_loop_end;*/
		mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "T1", get_boot_time_us(),
				calculate_time_delta_us(ts_gyro_and_dist, ts_loop_start), 
				calculate_time_delta_us(ts_get_image, ts_gyro_and_dist), 
				calculate_time_delta_us(ts_preparation, ts_get_image));
		mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "T2", get_boot_time_us(), 
				calculate_time_delta_us(ts_preparation, ts_frame),
				calculate_time_delta_us(ts_computation, ts_preparation), 
				/*flow_real_comp_t*/calculate_time_delta_us(ts_result_extract, ts_computation));
		mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "T3", get_boot_time_us(), 
				calculate_time_delta_us(ts_result_accu, ts_result_extract), 
				calculate_time_delta_us(ts_finished, ts_result_accu), 
				calculate_time_delta_us(ts_loop_end, ts_finished));
		mavlink_msg_debug_vect_send(MAVLINK_COMM_2, "T4", get_boot_time_us(), 
				frame_delta, 
				flow_iter_count, 
				calculate_time_delta_us(ts_loop_start, last_loop_end));
		
		last_loop_end = ts_loop_end;
	}
}
