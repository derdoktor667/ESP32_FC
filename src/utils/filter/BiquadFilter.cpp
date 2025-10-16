// BiquadFilter.cpp
//
// This file implements a biquad (second-order) filter class.
// It can be configured as a low-pass filter and is commonly used in flight controllers
// for noise reduction on sensor data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "BiquadFilter.h"
#include <math.h> // Required for tanf, cosf, sinf

// Constructor: Initializes the filter as a low-pass filter.
// @param cutoffFreq The cutoff frequency in Hz.
// @param sampleFreq The sample frequency in Hz.
// @param Q The Q factor of the filter (typically 0.707 for Butterworth).
BiquadFilter::BiquadFilter(float cutoffFreq, float sampleFreq, float Q)
    : _x1(0.0f), _x2(0.0f), _y1(0.0f), _y2(0.0f)
{
    // Calculate coefficients for a low-pass Butterworth filter
    // Based on: https://www.earlevel.com/main/2012/11/26/biquad-c-code/
    float K = tanf(PI * cutoffFreq / sampleFreq);
    float norm = 1.0f / (1.0f + K / Q + K * K);
    _b0 = K * K * norm;
    _b1 = 2.0f * _b0;
    _b2 = _b0;
    _a1 = 2.0f * (K * K - 1.0f) * norm;
    _a2 = (1.0f - K / Q + K * K) * norm;
}

// Updates the filter with a new raw value and returns the filtered value.
float BiquadFilter::update(float rawValue)
{
    // Direct Form 2 Transposed Biquad implementation
    float result = _b0 * rawValue + _b1 * _x1 + _b2 * _x2 - _a1 * _y1 - _a2 * _y2;

    _x2 = _x1;
    _x1 = rawValue;
    _y2 = _y1;
    _y1 = result;

    return result;
}