/*
 * Date 11.02.2016
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


#ifndef _STOP_ZEBRA_STRIPES_FILTER
#define _STOP_ZEBRA_STRIPES_FILTER



#define __guid "adtf.aadc.aadc_StopZebraStripesFilter"

#include "stdafx.h"
#include "../../include/aadc_structs.h"


/*
This filter can be used to..

*/
class StopZebraStripesFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "Stop Zebra Stripes Filter", OBJCAT_DataFilter, "Stop Zebra Stripes Filter", 1, 0, 0, "BFFT GmbH");


public: // construction
    StopZebraStripesFilter(const tChar *);
    virtual ~StopZebraStripesFilter();

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

	/* contains current speed and distance to zebra stripes */
	cInputPin	m_iStopStruct;

	/* Input pin for driven distance */
	cInputPin m_iDistance;

	/* Input pin sensor struct */
	cInputPin m_iUltraSonic;

    /*! the output pin to send the value for speed controller before turning*/
   // cOutputPin m_oSpeed;

	/* Output pin for LT velocity */
	cOutputPin m_oVelocity;
    
	/*! the output pin to send the value for finish turning */
    cOutputPin m_oFinishFilter;

    cOutputPin m_oGCLOutput;



      /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;

 	// implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:

	// StopStruct (Input)
	cObjectPtr<IMediaTypeDescription> m_pStopStructInput;
	tBufferID m_szIDVelocityStructInput;
	tBufferID m_szIDDistanceStructInput;
	tBufferID m_szIDFarDistStructInput;
	tBufferID m_szIDBoolStructInput;
	tBool m_pStopStructInputSet;

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

 	// VelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pVelocityOutput;
	tBufferID m_szIDMinVelocityOutput;
	tBufferID m_szIDMaxVelocityOutput;
	tBufferID m_szIDVelocityTimestampOutput;
	tBool m_bIDsVelocityOutput;
	

    /*! in this struct all the ultrasonic signals are collected and transmitted togeher 
    tUltrasonicStruct m_ultraSonicDataStruct;*/

	
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

	// flag for activating filter (true while zebra stripe is in front of us)
	tBool m_filterActive;

	
	// flag for stopping mode (true while zebra stripe not reached)
	tBool m_stoppingProcessActive;
	// flag for no traffic / no pedestrian (sensor)
	tBool m_noTrafficSensor;
	tBool m_noTrafficCamera;


    /*! holds the timestamp when the list was started*/
    tUInt32 m_i32StartTimeStamp;
    
    // holds the timestamp when zebra stripe has been reached
    tUInt32 m_i32TimeStampZebraStripe;

    // bitmap format of input pin
    tBitmapFormat m_sInputFormatDepthimage;
    
	tBool m_bFirstFrameDepthimage;

	tBool m_showGCL;

	// velocity at t=0
	tFloat32 m_velocityZero;
	// distance to stopping position at t=0
	tFloat32 m_distanceZero;
	// distance offset
	tFloat32 m_distanceOffset;
	// distance to crossing
	tFloat32 m_dist2crossing;

	// counter for obstacles
	tInt32 m_c3, m_c6, m_c7, m_c9;
	tInt32 m_c3all, m_c6all, m_c7all, m_c9all;
	
	// counter for frames without obstacle
	tInt32 m_good_frame_count;

	// counter for frames with obstacle
	tInt32 m_bad_frame_count;

	// waiting time between two frames
	tFloat32 m_waiting_time;

	// counter for frames without obstacle (ultrasound)
	tInt32 m_good_us_count;

	// counter for frames with obstacle (ultrasound)
	tInt32 m_bad_us_count;

	tFloat32 m_US_FL, m_US_FCL, m_US_FC, m_US_FCR;
	
	// Length of zebra stripes
	tFloat32 m_zebraLength;

	void InitialState();

	void countObstacles(Point3f &coordinates, int i, int j);
	tBool checkObstacles();

	tResult ProcessStoppingSpeed(IMediaSample* pMediaSample);

	tResult ProcessZebraStripeStruct(IMediaSample* pMediaSample);

	tResult ProcessInputDepth(IMediaSample* pSample);

	tResult ProcessUltraSonic(IMediaSample* pMediaSample);

	tResult CreateAndTransmitGCL();

	tResult SendVelocityLT(tFloat32 fSpeed);

	tResult stopFilter();

	std::vector<int> aux3,aux6,aux7,aux9,auxObs;

};

#endif
