/*
 * Date 25.01.2016
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


#ifndef _TURNING_FILTER
#define _TURNING_FILTER



#define __guid "adtf.aadc.aadc_TurningFilter"

#include "stdafx.h"
#include "../../include/aadc_structs.h"


/*
This filter can be used to..

*/
class TurningFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "Turning Filter", OBJCAT_DataFilter, "Turning Filter", 1, 0, 0, "BFFT GmbH");    


public: // construction
    TurningFilter(const tChar *);
    virtual ~TurningFilter();

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

protected:


	/* Input pin start turning */
	cInputPin m_iStartTurning;

	/* Input pin for driven distance */
	cInputPin m_iDistance;

	/* Input pin for driven distance right wheel */
	cInputPin m_iDistanceRight;

	/* Input pin for driven distance left wheel */
	cInputPin m_iDistanceLeft;

	/* Input pin sensor struct */
	cInputPin m_iUltraSonic;

	cInpurPin m_itestLF;

    /*! the output pin to send the value for speed controller while turning*/
    cOutputPin m_oSpeed;

    /*! the output pin to send the value for steering controller */
    cOutputPin m_oSteering;

	/*! the output pin to send turn-left-light */
    cOutputPin m_oTurnLeftOutput;

	/*! the output pin to send turn-right-light */
    cOutputPin m_oTurnRightOutput;
    
    cOutputPin m_oBreaklight;

	/*! the output pin to send the value for finish turning */
    cOutputPin m_oFinishTurning;

	/* start the ES */
	cOutputPin m_oStartES;

	



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


	// StartTurningInput
	cObjectPtr<IMediaTypeDescription> m_pStartTurningInput;
	tBufferID m_szIDManeuvreStartTurningInput;
	tBool m_bIDsStartTurningSet;


	// UltraSonic Input
	cObjectPtr<IMediaTypeDescription> m_pUltraSonicInput;
	tBufferID m_szIDFrontLeftUltraSonicInput;
	tBufferID m_szIDFrontCenterLeftUltraSonicInput;
	tBufferID m_szIDFrontCenterUltraSonicInput;
	tBufferID m_szIDFrontCenterRightUltraSonicInput;
	tBufferID m_szIDFrontRightUltraSonicInput;
    tBool m_bIDsUltraSonicSet;

	// DistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBufferID m_szIDTimestampDistanceInput;
	tBool m_bIDsDistanceSet;

        // DistanceInputLeft
	cObjectPtr<IMediaTypeDescription> m_pDistanceInputLeft;
	tBufferID m_szIDDistanceInputLeft;
	tBufferID m_szIDTimestampDistanceInputLeft;
	tBool m_bIDsDistanceLeftSet;


        // DistanceInputRight
	cObjectPtr<IMediaTypeDescription> m_pDistanceInputRight;
	tBufferID m_szIDDistanceInputRight;
	tBufferID m_szIDTimestampDistanceInputRight;
	tBool m_bIDsDistanceRightSet;


    /*! in this struct all the ultrasonic signals are collected and transmitted togeher 
    tUltrasonicStruct m_ultraSonicDataStruct;*/

	tBool m_ultraSonicReceived;

	cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
	/*! the id for the bool value output of the media description */
    tBufferID m_szIDBoolValueOutput;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolValueOutput;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDArduinoTimestampOutput;


	// flag for activating filter (true while either stopping, waiting or turning)
	tBool m_filterActive;

	tBool m_testLT;

	tBool m_stopTurning;

	// flags for turning-light
	tBool m_turnLeftON, m_turnRightON;

	tBool m_finishTurning;
	// flag for turning mode (true while car is turning)
	tBool m_isTurning;

	// turning maneuvre -> left, straight on, right
	tInt32 m_turningManeuvre;

	// distance offset
	tFloat32 m_distanceOffset;

	// current driving mode ( 0=straight, 1=left, 2=right)
	tInt32 m_drivingMode;

	// true if car has stopped
	tBool m_emergencyStop;

	// counter for good sensor frames
	tInt32 m_good_sensor_count;

	tResult turning();

	tResult ProcessTurningSpeed(IMediaSample* pMediaSample);

	tResult ProcessTurningStruct(IMediaSample* pMediaSample);

	tResult ProcessUltraSonic(IMediaSample* pMediaSample);

	tResult StartTurning();

    tResult TransmitBoolValue(cOutputPin* oPin, bool value);

	tResult changeSpeed(tFloat32 fSpeed, tFloat32 fSteering);

	tResult InitialState();

	void finishTurning();
};

#endif
