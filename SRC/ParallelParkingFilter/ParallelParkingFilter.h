﻿/*
 * Date 16.02.2016
 */ 
/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/


#ifndef _PARALLEL_PARKING_FILTER
#define _PARALLEL_PARKING_FILTER

struct tValueStruct {
    tFloat32 timeStamp;
    tFloat32 f32Value1;
    tFloat32 f32Value2;
};

#define __guid "adtf.aadc.aadc_ParallelParkingFilter"

#include "stdafx.h"

/*!
This filter can be used to generate two waveforms in Media Samples of the type tSignalValue.
The waveforms can be specified in table in the GUI of the filter by specify a timestamp and corresponding two values which are transmitted on the output pins. The filter does a linear interpolation between the given points and the sample rate of the output samples must be set in the property Actuator Update Rate [Hz] .
The filter can also work with a default file with predefined values. The file can be selected in the properties and the values can be loaded with the button Load Defaults. After editing values the button Save can be used to save the values to a new file or also back to the file with the default values used in the properties.
The values in the properties Default Value 1 and Default Value 2 are transmitted always except when the defined waveform is transmitted, i.e. before the waveform starts and after the waveform ends.
The file must be a simple file containing in one line first the timestamp, followed by the value 1 and value 2. A file looks like:
1000 0 80 
8000 3 110 
15000 3 90
*/


class ParallelParkingFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "Parallel Parking Filter", OBJCAT_DataFilter, "Parallel Parking Filter", 1, 0, 0, "BFFT GmbH");



public: // construction
    ParallelParkingFilter(const tChar *);
    virtual ~ParallelParkingFilter();

    /*! overrides cFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);

    /*! overrides cFilter */
    virtual tResult Start(__exception = NULL);

    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);

    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);   

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

    /*! overrides cFilter */
    tResult PropertyChanged(const tChar* strName);

protected:

    /*! the output pin to send the value for speed controller */
    cOutputPin m_oSpeedOutput;

    /*! the output pin to send the value for steering controller */
    cOutputPin m_oSteeringOutput;

    /*! the output pin to send hazzard light */
    cOutputPin m_oHazzardLightOutput;

    /*! the output pin to send turn-right-light */
    cOutputPin m_oTurnRightOutput;

    /*! the output pin to send reverse-light */
    cOutputPin m_oReverseLightOutput;

	/*! the output pin to send the value for finish parking */
    cOutputPin m_oFinishParkingOutput;

    /* Input pin start parking */
    cInputPin m_iStartParking;


    /*! descriptor for actuator values data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput; 
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDOutputF32Value; 
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDOutputArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOutputSet;

    /*! descriptor for actuator values data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput2; 
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDOutput2F32Value; 
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDOutput2ArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOutput2Set;

    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);


private:

	struct tProperties {
		tFloat32 fMinAcc;
		tFloat32 fMaxAcc;
		tFloat32 fRmin;
		tFloat32 fb;
		tFloat32 fl;
		tFloat32 fXe;
		tFloat32 fYe;
		tFloat32 fXc;
		tFloat32 fYc;
		tFloat32 fv;
		tFloat32 fXmin;

		tProperties()
		{
		// min. Accelaration
		fMinAcc = -3.5f;
		// max. Accelaration
		fMaxAcc = 3.5f;
		// min. turning circle
		fRmin = 0.65f;
		// 1/2 width car
		fb = 0.15f;
		//  distance rear-wheel and end of car
		fl = 0.30f-0.18f;
		// parking spot x-coord., right top edge, marking
		fXe = 0.7653f/2.0f + 0.18f;
		// parking spot y-coord., right top edge, marking
		fYe = 0.45f/2.0f;
		// start car x-pos.
		fXc = 0.7127f;  //calculated
		// start car y-pos.
		fYc = 0.225f+0.03f+0.22f;
		// start car velocity
		fv = 0.0f;
		// minimal x-coord car (end of parking spot minus tolerance)
		fXmin=-0.7653f/2.0f-0.02f+0.18f-0.05f;
		}
	} m_sProperties;

	// calculation: switchingpoints
	void calc();

    // StartParkingInput
	cObjectPtr<IMediaTypeDescription> m_pStartParkingInput;
	tBufferID m_szIDBoolValueStartParkingInput;
	tBufferID m_szIDTimestampStartParkingInput;


	cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
	/*! the id for the bool value output of the media description */
    tBufferID m_szIDBoolValueOutput;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolValueOutput;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDArduinoTimestampOutput;

	/*! helper function to toggle a bool signal via the specified pin
    @param the destination pin to toggle
    */
    tResult TransmitBoolValue(cOutputPin* pin, tBool value);
    cCriticalSection m_oTransmitBoolCritSection;

	// flags for light
	tBool m_hazzardON, m_turnRightON, m_reverseON;

	// flag for finish parking
	tBool m_finishParking;

	// flag for parking mode
	tBool m_isParking;



	// x-pos midpoint 1st circle (parking spot)
	tFloat32 fX1;
	// y-pos midpoint 1st circle (parking spot)
	tFloat32 fY1;

	// x-pos midpoint 2nd circle (parking spot)
	tFloat32 fX2;
	// y-pos midpoint 2nd circle (parking spot)
	tFloat32 fY2;

    // x-pos midpoint 3rd circle (parking spot, parallel parking)
	tFloat32 fX3;
	// y-pos midpoint 3rd circle (parking spot, parallel parking)
	tFloat32 fY3;

	// x-pos intersection point (1st and 2nd circle)
	tFloat32 fXi;
	// y-pos intersection point (1st and 2nd circle)
	tFloat32 fYi;

    // x-pos intersection point 2(2nd and 3rd circle, parallel parking)
	tFloat32 fX2i;
	// y-pos intersection point 2(2nd and 3rd circle, parallel parking)
	tFloat32 fY2i;

	// angles of circle segments
	tFloat32 fAlpha1;
	tFloat32 fAlpha2;
    tFloat32 fAlpha3;  //parallel parking

	// switching points
	tFloat32 fSwitchingPointsOrig[6];
	tFloat32 fSwitchingPoints[6];
	
	// driven distances
	tFloat32 fDistance[3];


       
	/*! the default output for value 1 */
	tFloat32 m_f32DefaultValue1;

	/*! the default output for value 2 */
	tFloat32 m_f32DefaultValue2;

	/*! the default output for time stretch factor */
	tFloat32 m_fTSF;

	/*! holds the timestamp when the list was started*/
	tUInt32 m_tInt32StartTimeStamp;

	/*! gets the current value from the vector for the given timestamp
	@param timestamp the current timestamp of which the value has to be get
	@param valueId the id of which the value has to be get (only 1 or 2 are valid by now: columns in vector)
	*/
	tFloat32 getCurrentValue(tFloat32 f32Timestamp, tInt8 i8ValueID);

	/*! gets the default value for the given id
	@param valueId the id of which the value has to be get (only 1 or 2 are valid by now: columns in vector)
	*/
	tFloat32 getDefaultValue(tInt8 i8ValueID);

	void parallelParking(tBool start);
};

#endif
