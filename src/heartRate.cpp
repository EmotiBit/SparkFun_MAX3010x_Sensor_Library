/*
 Optical Heart Rate Detection (PBA Algorithm)
 By: Nathan Seidle
 SparkFun Electronics
 Date: October 2nd, 2016
 
 Given a series of IR samples from the MAX30105 we discern when a heart beat is occurring

 Let's have a brief chat about what this code does. We're going to try to detect
 heart-rate optically. This is tricky and prone to give false readings. We really don't
 want to get anyone hurt so use this code only as an example of how to process optical
 data. Build fun stuff with our MAX30105 breakout board but don't use it for actual
 medical diagnosis.

 Excellent background on optical heart rate detection:
 http://www.ti.com/lit/an/slaa655/slaa655.pdf

 Good reading:
 http://www.techforfuture.nl/fjc_documents/mitrabaratchi-measuringheartratewithopticalsensor.pdf
 https://fruct.org/publications/fruct13/files/Lau.pdf

 This is an implementation of Maxim's PBA (Penpheral Beat Amplitude) algorithm. It's been 
 converted to work within the Arduino framework.
*/

/* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
* 
*/

#include "heartRate.h"
#include "DigitalFilter.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

#ifdef EMOTIBIT_PPG_100HZ
// Sampling frequency
const double f_s = 100; // Hz
#else
// Sampling frequency
const double f_s = 25; // Hz
#endif
// Cut-off frequency (-3 dB)
const double f_c = 3; // Hz
// Normalized cut-off frequency
const double f_n = 2 * f_c / f_s;

auto filter = butter<2>(f_n);  //< LPF to smooth output of respiration removed signal. The output of this filter is fed to the peak-detector
DigitalFilter acSignalAmplitudeFilter(DigitalFilter::FilterType::IIR_LOWPASS, 10, 1); //< create normalized cutoff of 0.1. Chosen based on testing and analysis in "Heartbeat Det AC Filter Calculator" testing sheet
DigitalFilter acRangeFitler(DigitalFilter::FilterType::IIR_LOWPASS, 10, 1); //< create normalized cutoff of 0.1. Chosen based on testing and analysis in "Heartbeat Det AC Filter Calculator" testing sheet
const float AC_RANGE_MULTIPLIER = 1.f/1.5;  // Chosen based on testing and analysis in "Heartbeat Det AC Filter Calculator" testing sheet

const int16_t IR_AC_MIN_AMP = 20;  // ABS MIN to consider it as a valid AC signal. EmotiBit with no finger usually presents AC noise in the 0-10 range
const int16_t IR_AC_MAX_AMP = 10000;  // ABS MAX to consider it as a valid AC signal


int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;
int16_t IR_AC_amplitude;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t filteredAcAmp;
int16_t acRange;
int16_t acAmpUpperBound;
int16_t acAmpLowerBound;

//  Heart Rate Monitor functions takes a sample value and the sample number
//  Returns true if a beat is detected
bool checkForBeat(int32_t sample, int16_t &iirFiltData, bool dcRemoved)
{
  bool beatDetected = false;

  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;
  
  //  Process next data sample
  if(!dcRemoved)
  {
    IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
    sample -= IR_Average_Estimated;
  }

  IR_AC_Signal_Current = lowPassIIRFitler((float)sample);  // sample is already IIR high pass filtered in EmotiBit cpp
  iirFiltData = IR_AC_Signal_Current;

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0))
  {
   
    IR_AC_amplitude = IR_AC_Signal_max - IR_AC_Signal_min;
    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;
    
    filteredAcAmp = acSignalAmplitudeFilter.filter(IR_AC_amplitude);
    acRange = acRangeFitler.filter(IR_AC_amplitude);
    acAmpUpperBound = filteredAcAmp + (acRange * AC_RANGE_MULTIPLIER);  // Upper bound is adjusted based on acRange and range multiplier
    acAmpLowerBound = filteredAcAmp - (acRange * AC_RANGE_MULTIPLIER);  // Lower bound is adjusted based on acRange and range multiplier
 
    if ((IR_AC_amplitude > IR_AC_MIN_AMP) && (IR_AC_amplitude < IR_AC_MAX_AMP))
    {
      // signal is within ABS bounds
      if(IR_AC_amplitude > acAmpLowerBound && IR_AC_amplitude < acAmpUpperBound)
      {
        // signal is within the filtered signal bounds
        //Heart beat!!!
        beatDetected = true;
      }
      else
      {
        // Ac signal is out of permissible range. Do nothing.
      }
    }
    else
    {
      // Ac signal is noise. Do nothing.
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0))
  {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge && (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge && (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }
  
  return(beatDetected);
}

//  Average DC Estimator
int16_t averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += ((((long) x << 15) - *p) >> 4);
  return (*p >> 15);
}


int16_t lowPassIIRFitler(float sample)
{
  return (int16_t)filter(sample);
}

//  Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}
