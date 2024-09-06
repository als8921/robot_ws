#!/usr/bin/python
# -*- coding:utf-8 -*-


"""
/*******************************************************************************
Copyright (c) 1983-2021 Advantech Co., Ltd.
********************************************************************************
THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY INFORMATION
WHICH IS THE PROPERTY OF ADVANTECH CORP., ANY DISCLOSURE, USE, OR REPRODUCTION,
WITHOUT WRITTEN AUTHORIZATION FROM ADVANTECH CORP., IS STRICTLY PROHIBITED.
================================================================================
REVISION HISTORY
--------------------------------------------------------------------------------
$Log: $
--------------------------------------------------------------------------------
$NoKeywords:  $
*/
/******************************************************************************
*
* Windows Example:
*    StaticAO.py
*
* Example Category:
*    AO
*
* Description:
*    This example demonstrates how to use Static AO voltage function.
*
* Instructions for Running:
*    1  Set the 'deviceDescription' for opening the device.
*    2  Set the 'profilePath' to save the profile path of being initialized device.
*    3  Set the 'channelStart' as the first channel for analog data output.
*    4  Set the 'channelCount' to decide how many sequential channels to output analog data.
*
* I/O Connections Overview:
*    Please refer to your hardware reference manual.
*
******************************************************************************/
"""
import time
import math
import sys
sys.path.append('..')
from CommonUtils import kbhit

from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.BDaqApi import AdxEnumToString, BioFailed, AdxGetValueRangeInformation
from Automation.BDaq.Utils import CreateArray

ONE_WAVE_POINT_COUNT = 512    # define how many data to makeup a waveform period

deviceDescription = "USB-4716,BID#0"
profilePath = u"../../profile/DemoDevice.xml"
channelStart = 0
channelCount = 1

class WaveStyle(object):
    Sine = 0
    Sawtooh = 1
    Square = 2

def GenerateWaveform(instantAoObj, channelStart, channelCount, samplesCount, waveStyle):
    ret = ErrorCode.Success
    waveBuffer = [0.0] * samplesCount
    mathIntervalRanges = CreateArray(MathInterval, 64)

    channelCountMax = instantAoObj.features.channelCountMax
    channels_list = instantAoObj.channels

    for i in range(channelCountMax):
        aoChannel = channels_list[i]
        valRange = aoChannel.valueRange

        if (ValueRange.V_ExternalRefBipolar == valRange) or (ValueRange.V_ExternalRefUnipolar == valRange):
            if instantAoObj.features.externalRefAntiPolar:
                if valRange == ValueRange.V_ExternalRefBipolar:
                    referenceValue = aoChannel.extRefBipolar
                    if referenceValue >= 0:
                        mathIntervalRanges[i].Max = referenceValue
                        mathIntervalRanges[i].Min = 0 - referenceValue
                    else:
                        mathIntervalRanges[i].Max = 0 - referenceValue
                        mathIntervalRanges[i].Min = referenceValue
                else:
                    referenceValue = aoChannel.extRefUnipolar
                    if referenceValue > 0:
                        mathIntervalRanges[i].Max = 0
                        mathIntervalRanges[i].Min = 0 - referenceValue
                    else:
                        mathIntervalRanges[i].Max = 0 - referenceValue
                        mathIntervalRanges[i].Min = 0
            else:
                if valRange == ValueRange.V_ExternalRefBipolar:
                    referenceValue = aoChannel.extRefBipolar
                    if referenceValue >= 0:
                        mathIntervalRanges[i].Max = referenceValue
                        mathIntervalRanges[i].Min = 0 - referenceValue
                    else:
                        mathIntervalRanges[i].Max = 0 - referenceValue
                        mathIntervalRanges[i].Min = referenceValue
                else:
                    referenceValue = aoChannel.extRefUnipolar
                    if referenceValue >= 0:
                        mathIntervalRanges[i].Max = referenceValue
                        mathIntervalRanges[i].Min = 0
                    else:
                        mathIntervalRanges[i].Max = 0
                        mathIntervalRanges[i].Min = referenceValue
        else:
            ret = AdxGetValueRangeInformation(valRange, 0, None, mathIntervalRanges[i], None)
            if BioFailed(ret):
                return ret, None

    # generate waveform data and put them into the buffer which the parameter 'waveBuffer' give in, the Amplitude these waveform
    oneWaveSamplesCount = samplesCount // channelCount
    waveBufferIndex = 0
    for i in range(oneWaveSamplesCount):
        for j in range(channelStart, channelStart + channelCount):
            channel = j % channelCountMax
            amplitude = (mathIntervalRanges[channel].Max - mathIntervalRanges[channel].Min) / 2
            offset = (mathIntervalRanges[channel].Max + mathIntervalRanges[channel].Min) / 2
            if waveStyle == WaveStyle.Sine:
                waveBuffer[waveBufferIndex] = amplitude * math.sin((i * 2.0 * 3.14159) / oneWaveSamplesCount) + offset
                waveBufferIndex += 1
            elif waveStyle == WaveStyle.Sawtooh:
                if (i >= 0) and (i < (oneWaveSamplesCount / 4.0)):
                    waveBuffer[waveBufferIndex] = amplitude * (i / oneWaveSamplesCount / 4.0) + offset
                    waveBufferIndex += 1
                else:
                    if (i >= oneWaveSamplesCount / 4.0) and (i < 3 * (oneWaveSamplesCount / 4.0)):
                        waveBuffer[waveBufferIndex] = amplitude * ((2.0 * (oneWaveSamplesCount / 4.0) - i) / (oneWaveSamplesCount/4.0)) + offset
                        waveBufferIndex += 1
                    else:
                        waveBuffer[waveBufferIndex] = amplitude * ((i - oneWaveSamplesCount)/ (oneWaveSamplesCount/4.0)) + offset
                        waveBufferIndex += 1
            elif waveStyle == WaveStyle.Square:
                if (i >= 0) and (i < (oneWaveSamplesCount / 2)):
                    waveBuffer[waveBufferIndex] = amplitude * 1 + offset
                    waveBufferIndex += 1
                else:
                    waveBuffer[waveBufferIndex] = amplitude * (-1) + offset
                    waveBufferIndex += 1
            else:
                print("Invalid wave style, generate waveform error!")
                ret = ErrorCode.ErrorUndefined
    return ret, waveBuffer

def AdvInstantAO():
    # Step 1: Create a 'InstantAoCtrl' for Static AO function
    # Select a device by device number or device description and specify the access mode.
    # in this example we use ModeWrite mode so that we can fully control the device,
    # including configuring, sampling, etc.
    instantAo = InstantAoCtrl(deviceDescription)
    for _ in range(1):
        instantAo.loadProfile = profilePath   # Loads a profile to initialize the device

        # Step 2: Output data
        # Generate waveform data
        ret, waveform = GenerateWaveform(instantAo, channelStart, channelCount, channelCount * ONE_WAVE_POINT_COUNT, WaveStyle.Sine)
        if BioFailed(ret):
            break

        print("Outputting data...")

        while(1):
            for i in range(ONE_WAVE_POINT_COUNT):
                writeData = waveform[channelCount * i:channelCount * i + channelCount]
                print(writeData)
                ret = instantAo.writeAny(channelStart, channelCount, None, writeData)
                if BioFailed(ret) or kbhit():
                    break
                # time.sleep(0.01)
    instantAo.dispose()

    if BioFailed(ret):
        enumStr = AdxEnumToString("ErrorCode", ret.value, 256)
        print("Some error occurred, And the last error code is %#x. [%s]" % (ret.value, enumStr))

    return 0


if __name__ == '__main__':
    AdvInstantAO()