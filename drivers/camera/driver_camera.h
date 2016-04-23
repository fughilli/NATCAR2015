/*
 * camera.h
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdint.h>

#define CAMERA_SAMPLES (128)

#define CAMERA_NUMCAMERAS (2)

void camera_init();

typedef uint16_t camera_sample_t;

typedef enum
{
	CAMERA_BUFFER_A = 0,
	CAMERA_BUFFER_B = 1
} camera_buffer_index_t;

extern volatile camera_sample_t* camera_buffers[CAMERA_NUMCAMERAS];
extern volatile camera_sample_t camera_ibuffers[CAMERA_NUMCAMERAS + 1][CAMERA_SAMPLES];

// Callback type:
// arg0 -> sample buffer
// arg1 -> number of samples
typedef void(*camera_callback_t)(camera_sample_t*, uint32_t);

void camera_registerCallback(camera_callback_t callback);

#endif /* CAMERA_H_ */
