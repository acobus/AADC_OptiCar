/*
 * Date 17.03.2016
 */ 

#include <math.h>
#include "stdafx.h"
#include "PullOutParking.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

#include "../../include/action_enum.h"
#include "../../include/cross_type.h"
#include "../../include/cross_sign_type.h"

#include "../../include/pullout_enum.h"

#define OBJECT_HEIGHT 0.05
#define OFFSET_X 0.09
#define LANE_OFFSET_X 0.08
#define LANE_OFFSET_Z 0.05
#define SCURVE_LEFT 0.60
#define SCURVE_RIGHT 0.60
#define CROSS_NORMAL_FIRST 1.25
#define CROSS_NORMAL_SECOND 1.72
#define MAX_WAITING_TIME 10.00 //Aendern

#define POP_PROP_SHOW_GCL "Common::Show GCL Output"

enum pullOutType {PULLOUTTYPE_UNKNOWN, PARALLEL_S_CURVE, PARALLEL_NORMAL, CROSS_NORMAL};

using namespace cv;

ADTF_FILTER_PLUGIN("PullOutFilter", __guid, PullOutFilter);


PullOutFilter::PullOutFilter(const tChar* __info) 
{
	
	SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

	SetPropertyBool(POP_PROP_SHOW_GCL, tFalse);
    SetPropertyStr(POP_PROP_SHOW_GCL NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");
}

PullOutFilter::~PullOutFilter()
{

}


tResult PullOutFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)    {

        //get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        //get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		//get description for sensor data pins
        tChar const * strDescIntSignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");
        RETURN_IF_POINTER_NULL(strDescIntSignalValue);
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeIntSignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescIntSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description Sensors
		tChar const * UltraSonicDesc = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(UltraSonicDesc);
		cObjectPtr<IMediaType> pTypeUltraSonicDesc = new cMediaType(0, 0, 0, "tUltrasonicStruct", UltraSonicDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description TurningState
        tChar const * TurningStateDesc = pDescManager->GetMediaDescription("tTurningStopStruct");
        RETURN_IF_POINTER_NULL(TurningStateDesc);
        cObjectPtr<IMediaType> pTypeTurningStateDesc = new cMediaType(0, 0, 0, "tTurningStopStruct", TurningStateDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput2));

		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBoolNextSpotFree));
		RETURN_IF_FAILED(pTypeIntSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartInput)); 
		RETURN_IF_FAILED(pTypeUltraSonicDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUltraSonicInput)); 

		RETURN_IF_FAILED(pTypeTurningStateDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateTurningOutput));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInputRight));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInputLeft));

		//create input pin for starting
    	RETURN_IF_FAILED(m_iStartPullOut.Create("StartPullOut", pTypeIntSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartPullOut));

		//create input pin for US
    	RETURN_IF_FAILED(m_iUltraSonic.Create("us_struct", pTypeUltraSonicDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonic));
    	
		//create input pin for nextSpotFree
    	RETURN_IF_FAILED(m_iNextSpotFree.Create("next_spot_free", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iNextSpotFree));
    	
		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));

        // DistanceInput
		RETURN_IF_FAILED(m_iDistanceRight.Create("Distance_right_wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistanceRight));

        // DistanceInput
		RETURN_IF_FAILED(m_iDistanceLeft.Create("Distance_left_wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistanceLeft));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));


		//create output pin for turning struct
		RETURN_IF_FAILED(m_oStartCrossPullOut.Create("StartTurningStruct", pTypeTurningStateDesc, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oStartCrossPullOut));
		
        //create output pin for speed output data
        RETURN_IF_FAILED(m_oSpeed.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSpeed));

        //create output pin for steering output data
        RETURN_IF_FAILED(m_oSteering.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteering));

		//create output pin for turn-right light output data
        RETURN_IF_FAILED(m_oTurnRightOutput.Create("turn-right_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnRightOutput));

		//create output pin for turn-left light output data
        RETURN_IF_FAILED(m_oTurnLeftOutput.Create("turn-left_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnLeftOutput));
        
		//create output pin for brake light light output data
        RETURN_IF_FAILED(m_oBrakelight.Create("Break_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oBrakelight));

		// Start LaneTracking Output
        RETURN_IF_FAILED(m_oStopLF.Create("Start_LF", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStopLF));

		// Start EmergencySTOP Output
        RETURN_IF_FAILED(m_oStopES.Create("Start_ES", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStopES));

		//create output pin for finish turning output data
        RETURN_IF_FAILED(m_oFinish.Create("finish_pullOut", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oFinish));

		//GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));


        RETURN_NOERROR;
    }
    else if(eStage == StageNormal)
    {
    }
    else if(eStage == StageGraphReady)
    {
        // no ids were set so far
		m_bIDsOutputSet					= tFalse;
		m_bIDsOutput2Set				= tFalse;
		m_bIDsBoolValueOutput 			= tFalse;
		m_bIDsUltraSonicSet 			= tFalse;
		m_bIDsDistanceSet				= tFalse;
		m_bIDsDistanceRightSet			= tFalse;
		m_bIDsDistanceLeftSet			= tFalse;
		m_bIDsStartSet 					= tFalse;
		m_bTurningOutputSet				= tFalse;
		m_bIDsBoolValueNextSpotFreeOutput = tFalse;
		m_bFirstFrameDepthimage			= tTrue;

		m_showGCL = GetPropertyBool(POP_PROP_SHOW_GCL);
		
		m_fMaxAcc = 3.5f;
		m_fMinAcc = -3.5f;

		m_fSwitchingPointsParallel[0] = 0.663386;
		m_fSwitchingPointsParallel[1] = 1.32677;
		m_fSwitchingPointsParallel[2] = 1.91307;
		m_fSwitchingPointsParallel[3] = 2.49936;
		m_fSwitchingPointsParallel[4] = 2.80975;
		m_fSwitchingPointsParallel[5] = 3.12013;
		
		InitialState();
    }

    RETURN_NOERROR;
}


tResult PullOutFilter::PropertyChanged(const char* strProperty)
{

	if (NULL == strProperty || cString::IsEqual(strProperty, POP_PROP_SHOW_GCL)) {
		m_showGCL = GetPropertyBool(POP_PROP_SHOW_GCL);
	}

	RETURN_NOERROR;
}

tResult PullOutFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
    	RETURN_IF_POINTER_NULL(pMediaSample);

		if (pSource == &m_iStartPullOut) {
			ProcessPullOutStruct(pMediaSample);
		} else if (pSource == &m_iNextSpotFree) {
			ProcessNextSpotFree(pMediaSample);
		} else if (pSource == &m_iDistanceLeft && m_pullOutType == PARALLEL_S_CURVE && m_drivenDist >= SCURVE_LEFT) {
			ProcessSCurve(pMediaSample, pSource);
        } else if (pSource == &m_iDistanceRight && m_pullOutType == PARALLEL_S_CURVE && m_drivenDist < SCURVE_LEFT) {
            ProcessSCurve(pMediaSample, pSource);
        } else if (pSource == &m_iDistanceLeft && m_pullOutType == CROSS_NORMAL && m_drivenDist < CROSS_NORMAL_FIRST) {
        	ProcessCrossNormal(pMediaSample, pSource);
        } else if (pSource == &m_iDistanceRight && m_pullOutType == CROSS_NORMAL && m_drivenDist >= CROSS_NORMAL_FIRST) {
            ProcessCrossNormal(pMediaSample, pSource);
		} else if (&m_iUltraSonic == pSource && m_filterActive && m_pullOutType == PULLOUTTYPE_UNKNOWN) {
			//LOG_INFO(cString::Format("USStruct"));
			ProcessUltraSonic(pMediaSample);
		} else if (&m_iUltraSonic == pSource && m_pullOutType != PULLOUTTYPE_UNKNOWN && m_pullOutType != PARALLEL_NORMAL) {
			//LOG_INFO(cString::Format("POPF: Process ES"));
			ProcessUltraSonicES(pMediaSample);
		} else if (pSource == &m_iDepthimagePin && m_filterActive && m_pullOutType == PULLOUTTYPE_UNKNOWN && m_maneuvre == CROSS_RIGHT) {

			
		    //Videoformat
		    if (m_bFirstFrameDepthimage)
		    {        
		        cObjectPtr<IMediaType> pType;
		        RETURN_IF_FAILED(m_iDepthimagePin.GetMediaType(&pType));
		        cObjectPtr<IMediaTypeVideo> pTypeVideo;
		        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
		        const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
		        if (pFormat == NULL)
		        {
		            //LOG_ERROR("No Bitmap information found on pin \"input\"");
		            RETURN_ERROR(ERR_NOT_SUPPORTED);
		        }
		        m_sInputFormatDepthimage.nPixelFormat = pFormat->nPixelFormat;
		        m_sInputFormatDepthimage.nWidth = pFormat->nWidth;
		        m_sInputFormatDepthimage.nHeight =  pFormat->nHeight;
		        m_sInputFormatDepthimage.nBitsPerPixel = pFormat->nBitsPerPixel;
		        m_sInputFormatDepthimage.nBytesPerLine = pFormat->nBytesPerLine;
		        m_sInputFormatDepthimage.nSize = pFormat->nSize;
		        m_sInputFormatDepthimage.nPaletteSize = pFormat->nPaletteSize;
			    m_bFirstFrameDepthimage = false;

				m_tInt32StartTimeStamp = 0;
			}		
		
				if (m_tInt32StartTimeStamp==0) {
					m_tInt32StartTimeStamp = _clock->GetStreamTime();
				}
				tInt32 fTime = _clock->GetStreamTime();

				if ((fTime-m_tInt32StartTimeStamp)/1000000.0 > m_waiting_time){
					//LOG_INFO(cString::Format("SFT: waitingTime %f", m_waiting_time));
					//LOG_INFO(cString::Format("POPF: Process Cam"));
					ProcessInputDepth(pMediaSample);
					m_tInt32StartTimeStamp = fTime;
				}

        }
	}
    RETURN_NOERROR;
}

tResult PullOutFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult PullOutFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */){


	if (m_pullOutType != PARALLEL_NORMAL) {
		return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);	
	}
	
    if (nActivationCode == IRunnable::RUN_TIMER)
    {        
        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput)
        {
			tFloat32 fSpeed = getCurrentValue(_clock->GetStreamTime(),1);
			tFloat32 fSteering = getCurrentValue(_clock->GetStreamTime(),2);
        	changeControl(fSpeed, fSteering);
        }
		if (m_pullOutType == PULLOUTTYPE_UNKNOWN) {
			finishPullOut();
		}
    }
	
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tFloat32 PullOutFilter::getCurrentValue(tFloat32 fTime, tInt8 i8ValueID)
{

    //get time difference in seconds
    fTime = (_clock->GetStreamTime()-m_timeStamp)/1000000.0f;

	//Parallel Parking
    switch (i8ValueID)
    {
    case 1: {
    	if (fTime>=0 && fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[4]) {
    		return m_fMinAcc*fTime;
    	} else if (fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[2]) {	
    		return m_fMaxAcc*fTime+m_fMinAcc*(m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[3]);
        } else if (fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[1]) {
        	return m_fMinAcc*fTime+m_fMaxAcc*(m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[1]);
        } else if (fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[0]) {
            return m_fMaxAcc*fTime+m_fMinAcc*(m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[1]);
        } else if (fTime<m_fSwitchingPointsParallel[5]) {
            return m_fMinAcc*fTime+m_fMaxAcc*m_fSwitchingPointsParallel[5];
        } else {
        	return 0.0;
        }
    	break;}
    case 2: {
    	if (fTime>=0 && fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[3]) {
    		return 120.0;
    	} else if (fTime<m_fSwitchingPointsParallel[5]-m_fSwitchingPointsParallel[1]) {	
    		return 60.0;
        } else if (fTime<m_fSwitchingPointsParallel[5]) {	
    		return 120.0;
        } else {
        	m_pullOutType = PULLOUTTYPE_UNKNOWN;
            return 90.0;
        }
        break;}
    }

    //should never happen (only if valueID is not 1 or 2:
    return 0.0;
}


tResult PullOutFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult PullOutFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult PullOutFilter::ProcessNextSpotFree(IMediaSample* pMediaSample){


	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionBoolNextSpotFree, pMediaSample, pCoder);    
		
		if (!m_bIDsBoolValueNextSpotFreeOutput) {						  
			pCoder->GetID("bValue", m_szIDBoolValueNextSpotFreeOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestampNextSpotFreeOutput);
			m_bIDsBoolValueNextSpotFreeOutput=tTrue;
		}

			pCoder->Get(m_szIDBoolValueOutput, (tVoid*)&m_nextSpotAvailable); 
	}
	RETURN_NOERROR;
}


tResult PullOutFilter::ProcessUltraSonicES(IMediaSample* pMediaSample){

			tSignalValue frontLeft, frontCenterLeft, frontCenter, frontCenterRight, frontRight, rearLeft, rearCenter;

			{
				__adtf_sample_read_lock_mediadescription(m_pUltraSonicInput, pMediaSample, pCoder);    
				
				if (!m_bIDsUltraSonicSet) {						  
					pCoder->GetID("tFrontLeft", m_szIDFrontLeftUltraSonicInput);
					pCoder->GetID("tFrontCenterLeft", m_szIDFrontCenterLeftUltraSonicInput);
					pCoder->GetID("tFrontCenter", m_szIDFrontCenterUltraSonicInput);
					pCoder->GetID("tFrontCenterRight", m_szIDFrontCenterRightUltraSonicInput);
					pCoder->GetID("tFrontRight", m_szIDFrontRightUltraSonicInput);
					pCoder->GetID("tSideLeft", m_szIDSideLeftUltraSonicInput);
					pCoder->GetID("tSideRight", m_szIDSideRightUltraSonicInput);
					pCoder->GetID("tRearLeft", m_szIDRearLeftUltraSonicInput);
					pCoder->GetID("tRearCenter", m_szIDRearCenterUltraSonicInput);
					m_bIDsUltraSonicSet=tTrue;
				}

  				pCoder->Get(m_szIDFrontLeftUltraSonicInput, (tVoid*)&frontLeft); 
  				pCoder->Get(m_szIDFrontCenterLeftUltraSonicInput, (tVoid*)&frontCenterLeft);   
  				pCoder->Get(m_szIDFrontCenterUltraSonicInput, (tVoid*)&frontCenter);   
  				pCoder->Get(m_szIDFrontCenterRightUltraSonicInput, (tVoid*)&frontCenterRight);   
  				pCoder->Get(m_szIDFrontRightUltraSonicInput, (tVoid*)&frontRight); 
  				pCoder->Get(m_szIDRearLeftUltraSonicInput, (tVoid*)&rearLeft);
  				pCoder->Get(m_szIDRearCenterUltraSonicInput, (tVoid*)&rearCenter);
			}


			tBool emergencyStop = tFalse;
			tFloat32 fSpeed = 1.5;
			tFloat32 fSteering = 90.0;


			switch (m_drivingMode) {
				case ACTION_STRAIGHT:   
					if (frontCenterLeft.f32Value < 0.1 || frontCenter.f32Value < 0.2 || frontCenterRight.f32Value < 0.1) {
						emergencyStop = tTrue;
					}
					break;
				case ACTION_LEFT: 
					fSteering = 60.0;
					if (frontLeft.f32Value < 0.1 || frontCenterLeft.f32Value < 0.2 || frontCenter.f32Value < 0.1 ) {
						emergencyStop = tTrue;
					}
					break;

				case ACTION_RIGHT:  
					fSteering = 120.0;
					if ( frontCenter.f32Value < 0.1 || frontCenterRight.f32Value < 0.2 || frontRight.f32Value < 0.1) {
						emergencyStop = tTrue;
					}
					break;
					
				case ACTION_BACKWARDS:  
					fSteering = 60.0;
					if ( rearCenter.f32Value < 0.2 || rearLeft.f32Value < 0.1) {
						emergencyStop = tTrue;
					}
					break;
                }


                if (emergencyStop && !m_emergencyStop) {
						LOG_INFO(cString::Format("POPF: ES, steering %f", fSteering));
                        // stop car
                        fSpeed = 0.0;
						m_emergencyStop = tTrue;
						TransmitBoolValue(&m_oBrakelight, tTrue);
                        changeControl(fSpeed, fSteering);
                } else if ( !emergencyStop && m_emergencyStop) {
                        ++m_good_sensor_count;

						if (m_good_sensor_count >= 5){
							LOG_INFO(cString::Format("POPF: STOP ES, steering %f", fSteering));
                        	m_emergencyStop = tFalse;
							m_good_sensor_count = 0;

							if (m_drivingMode == ACTION_BACKWARDS){
								fSpeed *= -1.0;
							}
							TransmitBoolValue(&m_oBrakelight, tFalse);
							
                        	changeControl(fSpeed, fSteering);
						}
                }
                RETURN_NOERROR;
		}



tResult PullOutFilter::ProcessUltraSonic(IMediaSample* pMediaSample){

			tSignalValue frontLeft, frontCenterLeft, frontCenter, frontCenterRight, frontRight, sideLeft, sideRight, rearLeft;

			{
				__adtf_sample_read_lock_mediadescription(m_pUltraSonicInput, pMediaSample, pCoder);    
				
				if (!m_bIDsUltraSonicSet) {						  
					pCoder->GetID("tFrontLeft", m_szIDFrontLeftUltraSonicInput);
					pCoder->GetID("tFrontCenterLeft", m_szIDFrontCenterLeftUltraSonicInput);
					pCoder->GetID("tFrontCenter", m_szIDFrontCenterUltraSonicInput);
					pCoder->GetID("tFrontCenterRight", m_szIDFrontCenterRightUltraSonicInput);
					pCoder->GetID("tFrontRight", m_szIDFrontRightUltraSonicInput);
					pCoder->GetID("tSideLeft", m_szIDSideLeftUltraSonicInput);
					pCoder->GetID("tSideRight", m_szIDSideRightUltraSonicInput);
					pCoder->GetID("tRearLeft", m_szIDRearLeftUltraSonicInput);
					pCoder->GetID("tRearCenter", m_szIDRearCenterUltraSonicInput);
					m_bIDsUltraSonicSet=tTrue;
				}

  				pCoder->Get(m_szIDFrontLeftUltraSonicInput, (tVoid*)&frontLeft); 
  				pCoder->Get(m_szIDFrontCenterLeftUltraSonicInput, (tVoid*)&frontCenterLeft);   
  				pCoder->Get(m_szIDFrontCenterUltraSonicInput, (tVoid*)&frontCenter);   
  				pCoder->Get(m_szIDFrontCenterRightUltraSonicInput, (tVoid*)&frontCenterRight);   
  				pCoder->Get(m_szIDFrontRightUltraSonicInput, (tVoid*)&frontRight);
  				pCoder->Get(m_szIDSideLeftUltraSonicInput, (tVoid*)&sideLeft);
  				pCoder->Get(m_szIDSideRightUltraSonicInput, (tVoid*)&sideRight);
  				pCoder->Get(m_szIDRearLeftUltraSonicInput, (tVoid*)&rearLeft);
			}


			tBool goodStruct = tFalse;

			switch (m_maneuvre) {
				case PARALLEL_RIGHT:
					if (m_nextSpotAvailable){
						if (frontCenter.f32Value > 0.60) {
							++m_NSF_good_count;
							if (m_NSF_good_count >= 4){
								m_nextSpotFree = tTrue;
								m_nextSpotAvailable = tFalse;
							}
						} else {
							++m_NSF_bad_count;
						}
					} 
					//LOG_INFO(cString::Format("POP: frontLeft %f, rearLeft %f, sideLeft %f", frontLeft.f32Value, rearLeft.f32Value, sideLeft.f32Value));
					if (frontLeft.f32Value>0.60 && rearLeft.f32Value>0.60 && sideLeft.f32Value>0.3) {
						goodStruct = tTrue;
					}
					break;
				case CROSS_RIGHT:
					if (m_nextSpotAvailable){
						if (sideRight.f32Value > 0.3) {
							++m_NSF_good_count;
							if (m_NSF_good_count >= 3){
								ProcessCrossPullOut(ACTION_RIGHT, tTrue);
								InitialState();
								RETURN_NOERROR;
							}
						} else {
							++m_NSF_bad_count;
						}
					} 
					if (frontCenterLeft.f32Value>0.50 && frontCenter.f32Value>0.4 && frontCenterRight.f32Value>0.40) {
						goodStruct = tTrue;
					}
					break;
			}
			
			if (m_NSF_bad_count >= 2) {
				m_NSF_good_count = m_NSF_bad_count = 0;
				m_nextSpotAvailable = tFalse;
			} 

				if (goodStruct) {
					++m_good_us_count;
				} else {
					++m_bad_us_count;
					if (m_bad_us_count >=2) {
						m_good_us_count = 0;
						m_bad_us_count = 0;
					}
				}
				tFloat32 waitingTime =( _clock->GetStreamTime() - m_tInt32TimeStampStart)/1000000.0;
				//LOG_INFO(cString::Format("POPF: waitingTime %f, timeStamp %f", waitingTime, m_tInt32TimeStampStart/1000000.0));
				
				if (m_good_us_count >= 5 || waitingTime>MAX_WAITING_TIME)  {
					if (m_maneuvre == PARALLEL_RIGHT && !m_nextSpotAvailable){
						if (m_nextSpotFree) {
							m_pullOutType = PARALLEL_S_CURVE;

							changeControl(0.5,60);
						} else {
							ReadSwitchingPoints();
							m_timeStamp=_clock->GetStreamTime();
							m_pullOutType = PARALLEL_NORMAL;
						}
					} else if (m_maneuvre == CROSS_RIGHT && (m_noTraffic || waitingTime>MAX_WAITING_TIME) && !m_nextSpotAvailable){
						m_pullOutType = CROSS_NORMAL;
					}
				}

	LOG_INFO(cString::Format("POP: m_noTraffic %i, m_nextSpotAvailable %i, m_pullOutType %i, m_filterActive %i, m_good_us_count %i", m_noTraffic, m_nextSpotAvailable,m_pullOutType,m_filterActive, m_good_us_count));

	RETURN_NOERROR;
}

tResult PullOutFilter::ProcessSCurve(IMediaSample* pMediaSample, IPin* pSource ) {
	
	tFloat32 fDistance = 0.0;
	
	{
        __adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

        if (!m_bIDsDistanceSet) {
	        pCoder->GetID("f32Value", m_szIDDistanceInput);
	        pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput);
	        m_bIDsDistanceLeftSet = tTrue;
        }

        pCoder->Get(m_szIDDistanceInput, (tVoid*)&(fDistance));
    }
	
	if (pSource == &m_iDistanceRight) {
		if(m_distanceOffsetLeft == 0.0) {
			m_distanceOffsetLeft = fDistance;
		}
		fDistance -= m_distanceOffsetLeft;
	} else {
		if(m_distanceOffsetRight == 0.0) {
			m_distanceOffsetRight = fDistance;
		}
		fDistance -= m_distanceOffsetRight;
		fDistance += SCURVE_LEFT;
	}

	m_drivenDist = fDistance;
	tFloat32 fSpeed = 0.0;
	tFloat32 fSteering = 90.0;

	if (fDistance < SCURVE_LEFT) {
		m_drivingMode = ACTION_LEFT;
		fSteering = 60.0;
		fSpeed = 1.5;
	} else if (fDistance < SCURVE_LEFT + SCURVE_RIGHT) {
		m_drivingMode = ACTION_RIGHT;
		fSteering = 120.0;
		fSpeed = 1.5;
	} else {
		m_drivingMode = ACTION_STRAIGHT;
		fSteering = 90.0;
		fSpeed = 0.50; //Anpassen
		finishPullOut();
	}

	if (!m_emergencyStop){
		changeControl(fSpeed, fSteering);
	}
	
	RETURN_NOERROR;
}
	
tResult PullOutFilter::ProcessCrossNormal(IMediaSample* pMediaSample, IPin* pSource) {
	
	tFloat32 fDistance = 0.0;
	
	{
        __adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

        if (!m_bIDsDistanceSet) {
	        pCoder->GetID("f32Value", m_szIDDistanceInput);
	        pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput);
	        m_bIDsDistanceLeftSet = tTrue;
        }

        pCoder->Get(m_szIDDistanceInput, (tVoid*)&(fDistance));
    }
	
	if (pSource == &m_iDistanceRight) {
		if(m_distanceOffsetLeft == 0.0) {
			m_distanceOffsetLeft = fDistance;
		}
		fDistance -= m_distanceOffsetLeft;
		fDistance += CROSS_NORMAL_FIRST;
	} else {
		if(m_distanceOffsetRight == 0.0) {
			m_distanceOffsetRight = fDistance;
		}
		fDistance -= m_distanceOffsetRight;
	}

	m_drivenDist = fDistance;
	tFloat32 fSpeed = 0.0;
	tFloat32 fSteering = 90.0;

	
	if (fDistance < 0.2) {
		m_drivingMode = ACTION_STRAIGHT;
		fSteering = 90.0;
		fSpeed = 1.5;
	} else if (fDistance < CROSS_NORMAL_FIRST - 0.05) {
		m_drivingMode = ACTION_RIGHT;		
		fSteering = 120.0;
		fSpeed = 1.5;
	} else if (fDistance < CROSS_NORMAL_FIRST) {
		m_drivingMode = ACTION_RIGHT;
		fSteering = 120.0;
		fSpeed = 0.4;
	} else if (fDistance < CROSS_NORMAL_SECOND - 0.05) {
		m_drivingMode = ACTION_BACKWARDS;
		fSteering = 60.0;
		fSpeed = -1.5;
	} else if (fDistance < CROSS_NORMAL_SECOND) {
		m_drivingMode = ACTION_BACKWARDS;
		fSteering = 120.0;
		fSpeed = -0.4;
	} else {
		m_drivingMode = ACTION_STRAIGHT;
		fSteering = 90.0;
		fSpeed = 0.50; //Anpassen 1.5
		finishPullOut();
	}	
	
	if (!m_emergencyStop){
		changeControl(fSpeed, fSteering);
	}
	
	/*if (fDistance < 0.2) {
		m_drivingMode = ACTION_STRAIGHT;
		changeControl(1.5,90);
	} else if (fDistance < 2.0) {
		m_drivingMode = ACTION_RIGHT;
		changeControl(1.5,120);
	} else if (fDistance < 2.2) {
		m_drivingMode = ACTION_LEFT;
		changeControl(1.5,60);
	} else {
		m_drivingMode = ACTION_STRAIGHT;
		changeControl(1.5,90);
		finishPullOut();
	}*/
	
	RETURN_NOERROR;
}


tResult PullOutFilter::ProcessInputDepth(IMediaSample* pSample) {

	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	const tVoid* l_pSrcBuffer;

	IplImage* oImg = cvCreateImage(cvSize(m_sInputFormatDepthimage.nWidth, m_sInputFormatDepthimage.nHeight), IPL_DEPTH_16U, 3);
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	oImg->imageData = (char*)l_pSrcBuffer;
	Mat image(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	pSample->Unlock(l_pSrcBuffer);

	Mat depthImage = Mat(m_sInputFormatDepthimage.nHeight,m_sInputFormatDepthimage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthimage.nBytesPerLine);

	m_c2 = m_c6 = m_c7 = m_c8 = m_c8_r = m_c8_l = 0;
	
	m_c2all = m_c6all = m_c7all = m_c8all = m_c8_lall = m_c8_rall = 0;

	aux2.clear();
	aux6.clear();
	aux7.clear();
	aux8.clear();
	auxObs.clear();


	for (int j=50;j<200;j++) {
		for (int i=0; i<320; i++) {
			Point3f coordinates = Util::ComputeWorldCoordinate(2*i,2*j,depthImage.at<ushort>(j, i), 0,0);
			coordinates.x += OFFSET_X;			
			countObstacles(coordinates, i,j);
		}
	}				
	
	m_noTraffic = checkObstacles();
	//LOG_INFO(cString::Format("POPF: m_noTraffic %i", m_noTraffic));

	/* fstream f;
	f.open("Hinderniszaehler.dat",ios::out|ios::app);
	f << m_c1 << ", " << m_c2 <<  ", " <<m_c4 << ", " <<m_c5 << ", " <<m_c6 << ", " <<m_c7 << ", " <<m_c8 << ", " <<m_c9  <<"\n";
	f << m_c1all << ", " << m_c2all <<  ", " <<m_c4all << ", " <<m_c5all << ", " <<m_c6all << ", " <<m_c7all << ", " <<m_c8all << ", " <<m_c9all  <<"\n";
	f << m_c1/(tFloat32)m_c1all << ", " << m_c2/(tFloat32)m_c2all <<  ", " << m_c4/(tFloat32)m_c4all << ", " << m_c5/(tFloat32)m_c5all << ", " << m_c6/(tFloat32)m_c6all << ", " << m_c7/(tFloat32)m_c7all << ", " << m_c8/(tFloat32)m_c8all << ", " << m_c9/(tFloat32)m_c9all  <<"\n";
	f << m_turningMode << "\n";
	f << m_c8_lall << ", " << m_c8_rall << "\n";
	f << m_c8_l << ", " << m_c8_r << "\n";
	f << m_c8_l/(tFloat32)m_c8_lall << ", " << m_c8_r/(tFloat32)m_c8_rall << "\n";
	f.close();

	fstream f2;
	f2.open("stopTurningFilter.dat",ios::out|ios::app);
	f2 << " m_waitingTime " << m_waiting_time  << "\n";
	f2.close();*/


	if (!m_imageReceived) {
		m_imageReceived = tTrue;
	}

	CreateAndTransmitGCL();

	RETURN_NOERROR;
}

void PullOutFilter::countObstacles(Point3f &p, int i, int j) {


	if (p.x < 0.28 - LANE_OFFSET_X && p.x > -0.185 + LANE_OFFSET_X && p.z > 0.2 && p.z < 1.26 + m_dist2crossing) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c2;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c2all;
		aux2.push_back(i);
		aux2.push_back(j);
	} else if (p.x > -0.65 - 1.5 && p.x < -0.185 && p.z > 0.345 + m_dist2crossing + LANE_OFFSET_Z && p.z < 0.8 + m_dist2crossing - LANE_OFFSET_Z) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c6;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c6all;
		aux6.push_back(i);
		aux6.push_back(j);
	} else if (p.x > 0.28 && p.x < 0.28 + 0.6/*car*/ && p.z > 0.345 + m_dist2crossing + LANE_OFFSET_Z && p.z < 0.8 + m_dist2crossing - LANE_OFFSET_Z) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c7;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c7all;
		if(m_showGCL) {aux7.push_back(i); aux7.push_back(j);}

	} else if (p.x > 0.28 && p.x < 0.28 + 1.5 && p.z > 0.82 + m_dist2crossing + LANE_OFFSET_Z && p.z < 1.26 + m_dist2crossing - LANE_OFFSET_Z) {

		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c8;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c8all;

		if (p.x > 0.55) {
			if (p.x > 0.65) {
				//box r
				++m_c8_rall;

				if (p.y < 0.22 - OBJECT_HEIGHT) {
					++m_c8_r;
				}

			} else {
				//box l
				++m_c8_lall;

				if (p.y < 0.22 - OBJECT_HEIGHT) {
					++m_c8_l;
				}
			}
		}
		
		if(m_showGCL) {aux8.push_back(i); aux8.push_back(j);}
	}
}

tBool PullOutFilter::checkObstacles() {
	
	static const float pObs = 0.1;
	tBool testBox8 = tFalse;
	tBool noObstacle = tFalse;

	++m_c2all;
	++m_c6all;
	++m_c7all;
	++m_c8all;

	if (m_c2/(tFloat32)m_c2all < pObs && m_c6/(tFloat32)m_c6all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
		testBox8 = tTrue;
	}

		
/*LOG_INFO(cString::Format("SFT: box 6=%f",m_c6/(tFloat32)m_c6all));
LOG_INFO(cString::Format("SFT: box 7=%f",m_c7/(tFloat32)m_c7all));
LOG_INFO(cString::Format("SFT: goodFrameCount=%i",m_good_frame_count)); */


/*LOG_INFO(cString::Format("SFT: test box 8 bool=%i",testBox8));
LOG_INFO(cString::Format("SFT: %f, %f",m_c8_l/(tFloat32)(m_c8_lall+1), m_c8_r/(tFloat32)(m_c8_rall+1)));

LOG_INFO(cString::Format("SFT: test box 5 bool=%i",testBox5));
LOG_INFO(cString::Format("SFT: %f, %f", m_c5_b/(tFloat32)(m_c5_ball+1), m_c5_f/(tFloat32)(m_c5_fall+1)));*/

	if (testBox8 && m_test_minibox) {
		// test box 8
		if (m_c8_l/(tFloat32)(m_c8_lall+1) > pObs || m_c8_r/(tFloat32)(m_c8_rall+1) > pObs) {
			++m_bad_frame_8_count;
			++m_bad_frame_count;
			m_waiting_time = 0.5 * m_bad_frame_8_count;
		} else {
			noObstacle = tTrue;
		}
	} 

	if (m_bad_frame_8_count >= 5){ //5 Bilder hintereinander keine Veränderung ( 2 sec)
		// Annahme: kein Hindernis
		m_test_minibox = tFalse;
		if (testBox8){
			noObstacle = tTrue;
		}
	}

	if (noObstacle) {
		++m_good_frame_count;	
		m_waiting_time = 0.08;	
	} else {
		++m_bad_frame_count;
		if (!testBox8) {
			m_waiting_time = 0.1;
		}
	}

	if (m_bad_frame_count >= 2) { // mindestens zwei schlechte dicht hintereinander
		m_good_frame_count = 0; // fange von vorne an
		m_bad_frame_count = 0;
		return tFalse;
	} else if (m_good_frame_count >= 4) { // 4 gute mit maximal einem schlechten dazwischen
		m_bad_frame_count	= 0;
		return tTrue;
	} else {
		return tFalse;
	}

}

tResult PullOutFilter::changeControl(tFloat32 fSpeed, tFloat32 fSteering){

			  //create new media sample for speed controller
  			  {
     			   cObjectPtr<IMediaSample> pMediaSampleValue1;
      			  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

		        //allocate memory with the size given by the descriptor
		        cObjectPtr<IMediaSerializer> pSerializer1;
		        m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer1);
		        tInt nSize = pSerializer1->GetDeserializedSize();
		        RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));
		        {   // focus for sample write lock
		            //write date to the media sample with the coder of the descriptor
		            __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput,pMediaSampleValue1,pCoder);
		            // get the IDs for the items in the media sample
		            if(!m_bIDsOutputSet)
		            {
		                pCoder->GetID("f32Value", m_szIDOutputF32Value);
		                pCoder->GetID("ui32ArduinoTimestamp", m_szIDOutputArduinoTimestamp);
		                m_bIDsOutputSet = tTrue;
		            }
		            pCoder->Set(m_szIDOutputF32Value, (tVoid*)&fSpeed);
		        }

				//LOG_INFO(cString::Format("POF speed = %f",fSpeed));

		        //transmit media sample over output pin

		        RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));
		        RETURN_IF_FAILED(m_oSpeed.Transmit(pMediaSampleValue1));
			}


			//create new media sample for steering controller
			{
				cObjectPtr<IMediaSample> pMediaSampleValue2;
				RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue2));

				//allocate memory with the size given by the descriptor
				cObjectPtr<IMediaSerializer> pSerializerSpeed;
				m_pDescriptionOutput2->GetMediaSampleSerializer(&pSerializerSpeed);
				tInt nSizeSpeed = pSerializerSpeed->GetDeserializedSize();
				RETURN_IF_FAILED(pMediaSampleValue2->AllocBuffer(nSizeSpeed));

                {   // focus for sample write lock
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput2,pMediaSampleValue2,pCoder2);

                    // get the IDs for the items in the media sample
                    if(!m_bIDsOutput2Set)
                    {
						pCoder2->GetID("f32Value", m_szIDOutput2F32Value);
						pCoder2->GetID("ui32ArduinoTimestamp", m_szIDOutput2ArduinoTimestamp);
						m_bIDsOutput2Set = tTrue;
					}

                    pCoder2->Set(m_szIDOutput2F32Value, (tVoid*)&fSteering);
                }

                //transmit media sample over output pin
                RETURN_IF_FAILED(pMediaSampleValue2->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oSteering.Transmit(pMediaSampleValue2));
            }		
//LOG_INFO(cString::Format("PUF: speed %f, steering %f", fSpeed,fSteering));
	RETURN_NOERROR;

}

tResult PullOutFilter::ProcessPullOutStruct(IMediaSample* pMediaSample){
	
	tBool filterActive = tFalse;	

	if (m_maneuvre != PULLOUT_UNKNOWN){
		filterActive = tTrue;
	}

	{
		__adtf_sample_read_lock_mediadescription(m_pStartInput, pMediaSample, pCoder);    

		if (!m_bIDsStartSet) {
			pCoder->GetID("intValue", m_szIDManeuvreStartInput);
			m_bIDsStartSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDManeuvreStartInput, (tVoid*)&m_maneuvre);
	}

	if (m_maneuvre == PULLOUT_UNKNOWN){ //shutdown filter
		TransmitBoolValue(&m_oStopES, tTrue);
		TransmitBoolValue(&m_oStopLF, tTrue);
		InitialState();
		if (m_pullOutType == PARALLEL_S_CURVE || m_pullOutType == CROSS_NORMAL || m_pullOutType == PARALLEL_NORMAL){
			changeControl(0.0, 90.0);
		} else {
			ProcessCrossPullOut(ACTION_UNKNOWN, tFalse); // stop turning filter
		}
		RETURN_NOERROR;
	} else if (filterActive){ // filter active, don't start again
		RETURN_NOERROR;
	}

	TransmitBoolValue(&m_oStopES, tFalse);
	TransmitBoolValue(&m_oStopLF, tFalse);

	LOG_INFO(cString::Format("PUF: Start  maneuvre "));

	LOG_INFO(cString::Format("PUF: Start  maneuvre %i", m_maneuvre) );

	m_tInt32TimeStampStart = _clock->GetStreamTime();

	switch (m_maneuvre) {
		case CROSS_LEFT:
			LOG_INFO(cString::Format("POPF: ProcessCrossPullOut"));
			ProcessCrossPullOut(ACTION_LEFT, tTrue);
			InitialState();
			break;
		case CROSS_RIGHT:
			m_nextSpotAvailable = tTrue;
			m_filterActive = tTrue;
	        TransmitBoolValue(&m_oTurnRightOutput, true);
	        m_turnRightON = true;
			break;
		case PARALLEL_RIGHT:
			m_filterActive = tTrue;
	        TransmitBoolValue(&m_oTurnLeftOutput, true);
	        m_turnLeftON = true;
			break;
	}

	RETURN_NOERROR;
}


tResult PullOutFilter::ProcessCrossPullOut(tInt32 mode, tBool bValue) {
		
	cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

	 //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;

	tInt32 type = DEADENDFRONT;
	tInt32 giveWay = GIVEWAY;
	tBool stopSign = tFalse;
	tFloat32 distance = 0.05;
	tFloat32 velocity = 0.0;
	tFloat32 rotationDegree = 0.0;
	tBool bValuePullOutMode = tTrue;
	tBool bCircle = tFalse;

	m_pStateTurningOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	//write date to the media sample with the coder of the descriptor
	{
   		__adtf_sample_write_lock_mediadescription(m_pStateTurningOutput, pMediaSample, pCoderOutput);    

    	// set the id if not already done
    	if(!m_bTurningOutputSet)
    	{
			pCoderOutput->GetID("mode", m_szIDTurningStructModeOutput);
			pCoderOutput->GetID("type", m_szIDTurningStructTypeOutput);
			pCoderOutput->GetID("giveWay", m_szIDTurningStructGiveWayOutput);
			pCoderOutput->GetID("stopSign", m_szIDTurningStructStopSignOutput);
			pCoderOutput->GetID("fDistance", m_szIDTurningStructDistanceOutput);
			pCoderOutput->GetID("fVelocity", m_szIDTurningStructVelocityOutput);
			pCoderOutput->GetID("bStart", m_szIDTurningStructStatusOutput);
			pCoderOutput->GetID("bPullOutMode", m_szIDTurningStructPullOutModeOutput);
			pCoderOutput->GetID("rotationDegree", m_szIDTurningStructRotationDegreeOutput);
			pCoderOutput->GetID("bCircle", m_szIDTurningStructCircleOutput);
        	m_bTurningOutputSet = tTrue;
    	}      
        
    	// set value from sample
    	pCoderOutput->Set(m_szIDTurningStructModeOutput, (tVoid*)&mode);
		pCoderOutput->Set(m_szIDTurningStructTypeOutput, (tVoid*)&type);
		pCoderOutput->Set(m_szIDTurningStructGiveWayOutput, (tVoid*)&giveWay);
		pCoderOutput->Set(m_szIDTurningStructStopSignOutput, (tVoid*)&stopSign);
		pCoderOutput->Set(m_szIDTurningStructDistanceOutput, (tVoid*)&distance);
		pCoderOutput->Set(m_szIDTurningStructVelocityOutput, (tVoid*)&velocity);
		pCoderOutput->Set(m_szIDTurningStructStatusOutput, (tVoid*)&bValue);
		pCoderOutput->Set(m_szIDTurningStructPullOutModeOutput, (tVoid*)&bValuePullOutMode);
		pCoderOutput->Set(m_szIDTurningStructRotationDegreeOutput, (tVoid*)&rotationDegree);
		pCoderOutput->Set(m_szIDTurningStructCircleOutput, (tVoid*)&bCircle);
	}

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStartCrossPullOut.Transmit(pMediaSample);

	changeControl(0.8, 90.0);

	RETURN_NOERROR;
}


tResult PullOutFilter::TransmitBoolValue(cOutputPin* oPin, bool value)
{
	
	//__synchronized_obj(m_oBoolValueCritSection);
	
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = value;
    tUInt32 ui32TimeStamp = 0;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);    

        // set the id if not already done
        if(!m_bIDsBoolValueOutput)
        {
            pCoderOutput->GetID("bValue", m_szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestampOutput);
            m_bIDsBoolValueOutput = tTrue;
        }      

        // set value from sample
        pCoderOutput->Set(m_szIDBoolValueOutput, (tVoid*)&bValue);     
        pCoderOutput->Set(m_szIDArduinoTimestampOutput, (tVoid*)&(ui32TimeStamp));     
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult PullOutFilter::ReadSwitchingPoints() {

	// dont change this!
	fstream f;
	f.open("parallelParkingSwitchingPoints",ios::in);
	string aux;
	tInt32 counter = 0;

	while (getline(f, aux)){
		replace(aux.begin(),aux.end(),'.',',');
		m_fSwitchingPointsParallel[counter] = atof(aux.c_str());
		++counter;
	}

	f.close();

	RETURN_NOERROR;
}

tResult PullOutFilter::CreateAndTransmitGCL()
{
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput.IsConnected() || !m_showGCL)
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(1024*2048));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc; 


	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 185, 15).GetRGBA());	
	for (size_t i = 0; i < aux2.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux2[i]*2-1, aux2[i+1]*2-1, aux2[i]*2+1, aux2[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	for (size_t i = 0; i < aux6.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux6[i]*2-1, aux6[i+1]*2-1, aux6[i]*2+1, aux6[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100, 255, 255).GetRGBA());	
	for (size_t i = 0; i < aux7.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux7[i]*2-1, aux7[i+1]*2-1, aux7[i]*2+1, aux7[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 153, 0).GetRGBA());	
	for (size_t i = 0; i < aux8.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux8[i]*2-1, aux8[i+1]*2-1, aux8[i]*2+1, aux8[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 255).GetRGBA());	
	for (size_t i = 0; i < auxObs.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, auxObs[i]*2-1, auxObs[i+1]*2-1, auxObs[i]*2+1, auxObs[i+1]*2+1); 
	}


    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

tResult PullOutFilter::finishPullOut() {

	TransmitBoolValue(&m_oStopLF, tTrue);
	TransmitBoolValue(&m_oStopES, tTrue);	
	TransmitBoolValue(&m_oTurnLeftOutput,tFalse);
	TransmitBoolValue(&m_oTurnRightOutput,tFalse);
	TransmitBoolValue(&m_oFinish,tTrue);


	InitialState();
	
    RETURN_NOERROR;
}

tResult PullOutFilter::InitialState() {

	m_nextSpotFree					= tFalse;
	m_nextSpotAvailable				= tFalse;
	m_emergencyStop					= tFalse;
	m_turnRightON					= tFalse;
	m_turnLeftON					= tFalse;
	m_filterActive					= tFalse;
	m_noTraffic						= tFalse;
	m_test_minibox					= tTrue;			
	m_maneuvre						= PULLOUT_UNKNOWN;
	m_drivingMode					= ACTION_UNKNOWN;
	m_pullOutType					= PULLOUTTYPE_UNKNOWN;
	m_waiting_time					= 0.08;
	m_dist2crossing					= 0.0;
	m_drivenDist					= 0.0;
	m_distanceOffsetLeft			= 0.0;
	m_distanceOffsetRight			= 0.0;
	m_timeStamp						= 0;
	m_tInt32TimeStampStart			= 0;
	m_tInt32StartTimeStamp			= 0;
	m_good_us_count					= 0;
	m_bad_us_count					= 0;
	m_bad_sensor_count				= 0;
	m_good_sensor_count				= 0;
	m_NSF_good_count				= 0;
	m_NSF_bad_count					= 0;
	m_bad_frame_8_count				= 0;
	m_good_frame_count				= 0;
		
    RETURN_NOERROR;
}

