/*
 * Date 11.03.2016
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


#ifndef _PULLOUT_FILTER
#define _PULLOUT_FILTER

#define __guid "adtf.aadc.aadc_PullOutFilter"

#include "stdafx.h"
#include "../../include/aadc_structs.h"

/*
This filter can be used to..

*/
class PullOutFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "PullOut Filter", OBJCAT_DataFilter, "PullOut Filter", 1, 0, 0, "BFFT GmbH");    


public: // construction
    PullOutFilter(const tChar *);
    virtual ~PullOutFilter();

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

	tResult PropertyChanged(const char* strProperty);

protected:


	cVideoPin m_iDepthimagePin;

	/* Input pin start pull out */
	cInputPin m_iStartPullOut;

	/* Input pin for driven distance */
	cInputPin m_iDistance;
	
	/* Input pin for nextSpotFree */
	cInputPin m_iNextSpotFree;

   /* Input pin for driven distance right wheel */
	cInputPin m_iDistanceRight;

    /* Input pin for driven distance left wheel */
	cInputPin m_iDistanceLeft;

	/* Input pin sensor struct */
	cInputPin m_iUltraSonic;

    /*! the output pin to send the value for speed controller while pull out*/
    cOutputPin m_oSpeed;

    /*! the output pin to send the value for steering controller */
    cOutputPin m_oSteering;

	/*! the output pin to send turn-left-light */
    cOutputPin m_oTurnLeftOutput;

	/*! the output pin to send turn-right-light */
    cOutputPin m_oTurnRightOutput;
    
    cOutputPin m_oBrakelight;

	cOutputPin m_oStopLF;

	cOutputPin m_oStopES;

	cOutputPin m_oGCLOutput;

	/*! the output pin to send the value for finish pull out */
    cOutputPin m_oFinish;

	cOutputPin m_oStartCrossPullOut;



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

	// StateTurningOutput
	cObjectPtr<IMediaTypeDescription> m_pStateTurningOutput;
	tBufferID m_szIDTurningStructModeOutput;
	tBufferID m_szIDTurningStructTypeOutput;
	tBufferID m_szIDTurningStructGiveWayOutput;
	tBufferID m_szIDTurningStructStopSignOutput;
	tBufferID m_szIDTurningStructDistanceOutput;
	tBufferID m_szIDTurningStructVelocityOutput;
	tBufferID m_szIDTurningStructStatusOutput;
	tBufferID m_szIDTurningStructPullOutModeOutput;
	tBufferID m_szIDTurningStructRotationDegreeOutput;
	tBufferID m_szIDTurningStructCircleOutput;
	tBool m_bTurningOutputSet;

    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;

 	// implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:


	// StartInput
	cObjectPtr<IMediaTypeDescription> m_pStartInput;
	tBufferID m_szIDManeuvreStartInput;
	tBool m_bIDsStartSet;


	// UltraSonic Input
	cObjectPtr<IMediaTypeDescription> m_pUltraSonicInput;
	tBufferID m_szIDFrontLeftUltraSonicInput;
	tBufferID m_szIDFrontCenterLeftUltraSonicInput;
	tBufferID m_szIDFrontCenterUltraSonicInput;
	tBufferID m_szIDFrontCenterRightUltraSonicInput;
	tBufferID m_szIDFrontRightUltraSonicInput;
	tBufferID m_szIDSideLeftUltraSonicInput;
	tBufferID m_szIDSideRightUltraSonicInput;
	tBufferID m_szIDRearLeftUltraSonicInput;
	tBufferID m_szIDRearCenterUltraSonicInput;
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
    
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolNextSpotFree;
	/*! the id for the bool value output of the media description */
    tBufferID m_szIDBoolValueNextSpotFreeOutput;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolValueNextSpotFreeOutput;
    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDArduinoTimestampNextSpotFreeOutput;


	// bitmap format of input pin
    tBitmapFormat m_sInputFormatDepthimage;
    
	tBool m_bFirstFrameDepthimage;
	tBool m_imageReceived;

	tUInt32 m_c2, m_c6, m_c7, m_c8, m_c8_l, m_c8_r;
	tUInt32 m_c2all, m_c6all, m_c7all, m_c8all, m_c8_lall, m_c8_rall;

	std::vector<tInt32> aux2, aux6,aux7,aux8, auxObs;

	tUInt32 m_good_frame_count, m_bad_frame_count;
	tUInt32 m_bad_frame_8_count;

	tUInt32 m_good_us_count, m_bad_us_count;

	tBool m_test_minibox;

	tFloat32 m_waiting_time;
	tUInt32 m_tInt32StartTimeStamp;

	tUInt32 m_tInt32TimeStampStart;

	tFloat32 m_dist2crossing;

	// flag for activating filter (true while either stopping, waiting or turning)
	tBool m_filterActive;

	// flags for turning-light
	tBool m_turnLeftON, m_turnRightON;

	tBool m_finishTurning;
	// flag for turning mode (true while car is turning)
	tBool m_isTurning;

	// turning maneuvre -> left, straight on, right
	tInt32 m_maneuvre;

	// distance offsets
	tFloat32 m_distanceOffsetLeft;
	tFloat32 m_distanceOffsetRight;

	tFloat32 m_drivenDist;

	// current driving mode ( 0=straight, 1=left, 2=right)
	tInt32 m_drivingMode;

	// true if car has stopped
	tBool m_emergencyStop;

	// counter for good sensor frames
	tUInt32 m_good_sensor_count;
	// counter for bad sensor frames
	tUInt32 m_bad_sensor_count;
	
	// counters for sensor frames for next spot free
	tUInt32 m_NSF_good_count;
	tUInt32 m_NSF_bad_count;
	
	tBool m_nextSpotAvailable;
	tBool m_nextSpotFree;

	tInt32 m_pullOutType;

	tBool m_noTraffic;

	tFloat32 m_fMaxAcc, m_fMinAcc;
	double m_fSwitchingPointsParallel[6];
	
	tBool m_showGCL;

	// true if pull out process has started
	tBool m_isPullingOut;
	
	tUInt32 m_timeStamp;

	tResult CreateAndTransmitGCL();

	tResult ReadSwitchingPoints();
	
	tResult InitialState();
	
	tResult ProcessNextSpotFree(IMediaSample* pMediaSample);

	tResult ProcessPullOutStruct(IMediaSample* pMediaSample);

	tResult ProcessUltraSonic(IMediaSample* pMediaSample);
	tResult ProcessUltraSonicES(IMediaSample* pMediaSample);

	tResult ProcessInputDepth(IMediaSample* pMediaSample);

	void countObstacles(Point3f &coordinates, int i, int j);
	tBool checkObstacles();
	
	tFloat32 getCurrentValue(tFloat32 fTime, tInt8 i8ValueID);
	
	tResult ProcessSCurve(IMediaSample* pMediaSample, IPin* pSource);

	tResult ProcessCrossNormal(IMediaSample* pMediaSample, IPin* pSource);

	tResult ProcessCrossPullOut(tInt32 mode, tBool bValue);
	tResult ProcessCrossRight();

	tResult finishPullOut();


    tResult TransmitBoolValue(cOutputPin* oPin, bool value);
    cCriticalSection    m_oBoolValueCritSection;

	tResult changeControl(tFloat32 fSpeed, tFloat32 fSteering);
};

#endif
