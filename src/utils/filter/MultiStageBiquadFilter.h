// MultiStageBiquadFilter.h
//
// This file defines a class for a multi-stage biquad filter.
// It allows cascading multiple BiquadFilter instances to achieve a steeper roll-off
// and more effective noise reduction, similar to filtering strategies in Betaflight.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef MULTI_STAGE_BIQUAD_FILTER_H
#define MULTI_STAGE_BIQUAD_FILTER_H

#include <Arduino.h>
#include "BiquadFilter.h"

class MultiStageBiquadFilter
{
public:
    static constexpr float DEFAULT_BUTTERWORTH_Q = 0.707f;

    // Constructor: Initializes the multi-stage filter.
    // @param cutoffFreq The cutoff frequency in Hz for each stage.
    // @param sampleFreq The sample frequency in Hz.
    // @param numStages The number of biquad filter stages to cascade.
    // @param Q The Q factor for each biquad stage (typically 0.707 for Butterworth).
    // @param type The type of filter (LPF, HPF, NOTCH) for all stages.
    MultiStageBiquadFilter(float cutoffFreq, float sampleFreq, uint8_t numStages, float Q = DEFAULT_BUTTERWORTH_Q, FilterType type = LPF);

    // Destructor: Cleans up dynamically allocated BiquadFilter instances.
    ~MultiStageBiquadFilter();

    // Updates the filter with a new raw value and returns the filtered value.
    float update(float rawValue);

private:
    BiquadFilter **_filters; // Array of pointers to BiquadFilter instances
    uint8_t _numStages;      // Number of cascaded filter stages
};

#endif // MULTI_STAGE_BIQUAD_FILTER_H