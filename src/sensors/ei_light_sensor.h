/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EI_LIGHT_SENSOR_H
#define EI_LIGHT_SENSOR_H

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
#define LIGHTSENSOR_VALUES_IN_SAMPLE          4
#define SIZEOF_LIGHTSENSOR_VALUES_IN_SAMPLE   (sizeof(float) * LIGHTSENSOR_VALUES_IN_SAMPLE)

/* Function prototypes ----------------------------------------------------- */
bool ei_lightsensor_init(void);
float *ei_fusion_light_read_data(int n_samples);

static const ei_device_fusion_sensor_t light_sensor = {
    // name of sensor module to be displayed in fusion list
    "Light",
    // number of sensor module axis
    LIGHTSENSOR_VALUES_IN_SAMPLE,
    // sampling frequencies
    { 5.0f, 1.0f},
    // axis name and units payload (must be same order as read in)
    { { "R ", "lux" }, { "G ", "lux" }, { "B ", "lux" }, { "I ", "lux" }, },
    // reference to read data function
    &ei_fusion_light_read_data,
    0
};

#endif /* EI_LIGHT_SENSOR_H */
