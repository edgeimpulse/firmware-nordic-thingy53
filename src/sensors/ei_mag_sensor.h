/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EI_MAG_SENSOR_H
#define EI_MAG_SENSOR_H

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
#define MAG_AXIS_SAMPLED            3

/* Function prototypes ----------------------------------------------------- */
bool ei_mag_init(void);
float *ei_fusion_mag_read_data(int n_samples);

static const ei_device_fusion_sensor_t mag_sensor = {
    // name of sensor module to be displayed in fusion list
    "Magnetometer",
    // number of sensor module axis
    MAG_AXIS_SAMPLED,
    // sampling frequencies
    { 5.0f, 20.0f },
    // axis name and units payload (must be same order as read in)
    { { "magX", "uT  " }, { "magY", "uT  " }, { "magZ", "uT  " } },
    // reference to read data function
    &ei_fusion_mag_read_data,
    0
};

#endif /* EI_MAG_SENSOR_H */
