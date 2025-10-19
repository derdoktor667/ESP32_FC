// MultiStageBiquadFilter.cpp
//
// This file implements a class for a multi-stage biquad filter.
// It allows cascading multiple BiquadFilter instances to achieve a steeper roll-off
// and more effective noise reduction, similar to filtering strategies in Betaflight.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "MultiStageBiquadFilter.h"

// Constructor: Initializes the multi-stage filter.
MultiStageBiquadFilter::MultiStageBiquadFilter(float cutoffFreq, float sampleFreq, uint8_t numStages, float Q, FilterType type)
    : _numStages(numStages)
{
    _filters = new BiquadFilter *[_numStages];
    for (uint8_t i = 0; i < _numStages; ++i)
    {
        _filters[i] = new BiquadFilter(cutoffFreq, sampleFreq, Q, type);
    }
}

// Destructor: Cleans up dynamically allocated BiquadFilter instances.
MultiStageBiquadFilter::~MultiStageBiquadFilter()
{
    for (uint8_t i = 0; i < _numStages; ++i)
    {
        delete _filters[i];
    }
    delete[] _filters;
}

// Updates the filter with a new raw value and returns the filtered value.
float MultiStageBiquadFilter::update(float rawValue)
{
    float filteredValue = rawValue;
    for (uint8_t i = 0; i < _numStages; ++i)
    {
        filteredValue = _filters[i]->update(filteredValue);
    }
    return filteredValue;
}