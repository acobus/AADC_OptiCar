/*
 * Date 17.02.2016
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


#ifndef _STOP_TURNING_ZEITMESSUNG_FILTER
#define _STOP_TURNING_ZEITMESSUNG_FILTER



#define __guid "adtf.aadc.aadc_StopTurningFilterZeitmessung"

#include "stdafx.h"
#include "../../include/aadc_structs.h"


/*
This filter can be used to..

*/
class StopTurningFilterZeitmessung : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "Stop Turning Filter Zeitmessung", OBJCAT_DataFilter, "Stop Turning Filter Zeitmessung", 1, 0, 0, "BFFT GmbH");


public: // construction
    StopTurningFilterZeitmessung(const tChar *);
    virtual ~StopTurningFilterZeitmessung();

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

    /*! the output pin to send the value for speed (lane tracking)*/
    cOutputPin m_oVelocity;
	cOutputPin m_oStopLF;
	cOutputPin m_oStopES;
	cOutputPin m_oDriveStraight;
    cOutputPin m_oStartTurning;

    /*! the output pin to send the value for speed controller before turning*/
    cOutputPin m_oSpeed;

	/*! the output pin to send turn-left-light */
    cOutputPin m_oTurnLeftOutput;

	/*! the output pin to send turn-right-light */
    cOutputPin m_oTurnRightOutput;


      /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;

 	// implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:

    /*! descriptor for actuator values data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionOutput; 
    /*! the id for the f32value of the media description for the pins */
    tBufferID m_szIDOutputF32Value; 
    /*! the id for the arduino time stamp of the media description for the pins */
    tBufferID m_szIDOutputArduinoTimestamp;         
    /*! indicates if bufferIDs were set */
    tBool m_bIDsOutputSet;


	// StartParkingInput
	cObjectPtr<IMediaTypeDescription> m_pStartTurningInput;
	tBufferID m_szIDBoolValueStartTurningInput;
	tBufferID m_szIDModeStartTurningInput;
	tBufferID m_szIDTypeStartTurningInput;
	tBufferID m_szIDGiveWayStartTurningInput;
	tBufferID m_szIDDistanceStartTurningInput;
	tBufferID m_szIDTimestampStartTurningInput;
	tBufferID m_szIDVelocityStartTurningInput;
	tBufferID m_szIDStopSignStartTurningInput;
	tBufferID m_szIDBoolValuePullOutModeInput;
	tBool m_bIDsStartTurningSet;


	// DistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBufferID m_szIDTimestampDistanceInput;
	tBool m_bIDsDistanceSet;

    // StartTurningOutput
	cObjectPtr<IMediaTypeDescription> m_pStartTurningOutput;
	tBufferID m_szIDStartTurning;
	tBool m_bIDsStartTurningOutputSet;

 	// VelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pVelocityOutput;
	tBufferID m_szIDMinVelocityOutput;
	tBufferID m_szIDMaxVelocityOutput;
	tBufferID m_szIDVelocityTimestampOutput;
	tBool m_bIDsVelocityOutput;

	
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
    cCriticalSection    m_oTransmitBoolCritSection;

	// flag for activating filter (true while either stopping, waiting or turning)
	tBool m_filterActive;

	// flags for turning-light
	tBool m_turnLeftON, m_turnRightON;
	// flag for stopping mode (true while car slows down)
	tBool m_stopTurningActive;


	// DriveStraight
	tBool m_DriveStraightSet;

	tBool m_waiting;
	tUInt32 m_startWait;

	tUInt32 m_i32TimeStampHoldLine;


	// Lookup-table turning modes
	tInt32 turningModeLookup[4][3][3];

	// turning mode: siehe Zettel (1-12)
	tInt32 m_turningMode;

	tInt32 m_giveWay;

	tInt32 m_turningType;

	tBool m_stopSign;

	tBool m_pullOutMode;


	// turning maneuvre -> left, straight on, right
	tInt32 m_turningManeuvre;

    /*! holds the timestamp when the list was started*/
    tUInt32 m_i32StartTimeStamp;
    
	// velocity at t=0
	tFloat32 m_velocityZero;
	// distance to stopping position at t=0
	tFloat32 m_distanceZero;
	// distance offset
	tFloat32 m_distanceOffset;
	// distance to crossing
	tFloat32 m_dist2crossing;


	void InitialState();


	tResult ProcessStoppingSpeed(IMediaSample* pMediaSample);

	tResult ProcessTurningStruct(IMediaSample* pMediaSample);

	tResult ChangeSpeed(tFloat32 fSpeed);

	tResult StartTurning();

	bool Wait();
};

#endif
