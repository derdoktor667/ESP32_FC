// BiquadFilter.h
//
// This file defines a biquad (second-order) filter class.
// It can be configured as a low-pass filter and is commonly used in flight controllers
// for noise reduction on sensor data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#include <Arduino.h>

class BiquadFilter
{
public:
    // Constructor: Initializes the filter as a low-pass filter.
    // @param cutoffFreq The cutoff frequency in Hz.
    // @param sampleFreq The sample frequency in Hz.
    // @param Q The Q factor of the filter (typically 0.707 for Butterworth).
    BiquadFilter(float cutoffFreq, float sampleFreq, float Q = 0.707f);

    // Updates the filter with a new raw value and returns the filtered value.
    float update(float rawValue);

private:
    float _a0, _a1, _a2, _b0, _b1, _b2; // Filter coefficients
    float _x1, _x2, _y1, _y2;           // Filter state variables
};

#endif // BIQUAD_FILTER_H