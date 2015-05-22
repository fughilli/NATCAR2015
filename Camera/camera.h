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

enum Camera_t
{
    FAR = 1,
    NEAR = 0
};

void camera_init();

typedef uint16_t camera_sample_t;

extern camera_sample_t* camera_buffer;
extern enum Camera_t current_Camera;

// Callback type:
// arg0 -> sample buffer
// arg1 -> number of samples
typedef void(*camera_callback_t)(camera_sample_t*, uint32_t);

void camera_registerCallback(camera_callback_t callback);

#endif /* CAMERA_H_ */
