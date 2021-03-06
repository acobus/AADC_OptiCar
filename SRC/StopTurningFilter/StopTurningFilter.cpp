/*
 * Date 17.03.2016
 */ 

#include <math.h>
#include "stdafx.h"
#include "StopTurningFilter.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"
#include <fstream>

#include "../../include/action_enum.h"
#include "../../include/cross_type.h"
#include "../../include/cross_sign_type.h"

#define MIN_SPEED 0.5
#define MIDDLE_SPEED 1.0
#define OBJECT_HEIGHT 0.07
#define OFFSET_X 0.09
#define LANE_OFFSET_X 0.08
#define LANE_OFFSET_Z 0.03
#define MAX_WAITING_TIME 10.0

#define STF_PROP_SHOW_GCL "Common::Show GCL Output"

using namespace cv;

ADTF_FILTER_PLUGIN("StopTurningFilter", __guid, StopTurningFilter);


StopTurningFilter::StopTurningFilter(const tChar* __info)
{

	SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

	SetPropertyBool(STF_PROP_SHOW_GCL, tFalse);
    SetPropertyStr(STF_PROP_SHOW_GCL NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

	turningModeLookup[CROSSROAD][ACTION_LEFT][GIVEWAY] = 1;	// cross, left, give way
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][GIVEWAY] = 2; // cross, straight on, give way
	turningModeLookup[CROSSROAD][ACTION_RIGHT][GIVEWAY] = 3; // cross, right, give way
	turningModeLookup[CROSSROAD][ACTION_LEFT][HAVEWAY] = 4; // cross, left, have way
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][HAVEWAY] = 5; // cross, straight on, have way
	turningModeLookup[CROSSROAD][ACTION_RIGHT][HAVEWAY] = 6; // cross, right, have way
	turningModeLookup[CROSSROAD][ACTION_LEFT][RIGHTBEFORELEFT] = 13;// cross, left, right before left
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 14;// cross, straight on, right before left
	turningModeLookup[CROSSROAD][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // cross, right, right before left

	turningModeLookup[DEADENDLEFT][ACTION_LEFT][GIVEWAY] = 1;//0; // dead end left, left, give way
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][GIVEWAY] = 2;//7; // dead end left, straight on, give way
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][GIVEWAY] = 3;//8; // dead end left, right, give way
	turningModeLookup[DEADENDLEFT][ACTION_LEFT][HAVEWAY] = 4;//0; // dead end left, left, have way
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][HAVEWAY] = 5; // dead end left, straight on, have way
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][HAVEWAY] = 6; // dead end left, right, have way
	turningModeLookup[DEADENDLEFT][ACTION_LEFT][RIGHTBEFORELEFT] = 13;//0; // dead end left, left, right before left
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 14;// dead end left, straight on, right before left
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // dead end left, right, right before left

	turningModeLookup[DEADENDFRONT][ACTION_LEFT][GIVEWAY] = 9; // dead end front, left, give way
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][GIVEWAY] = 2;//0; // dead end front, straight on, give way
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][GIVEWAY] = 3; // dead end front, right, give way
	turningModeLookup[DEADENDFRONT][ACTION_LEFT][HAVEWAY] = 4;//10;// dead end front, left, have way
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][HAVEWAY] = 5;//0; // dead end front, straight on, have way
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][HAVEWAY] = 6; // dead end front, right, have way
	turningModeLookup[DEADENDFRONT][ACTION_LEFT][RIGHTBEFORELEFT] = 15;// dead end front, left, right before left
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 14;//0; // dead end front, straight on, right before left
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // dead end front, right, right before left

	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][GIVEWAY] = 1; //11;// dead end right, left, give way (!!)
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][GIVEWAY] = 2;//12;// dead end right, straight on, give way
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][GIVEWAY] = 3;//0; // dead end right, right, give way
	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][HAVEWAY] = 4; // dead end right, left, have way
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][HAVEWAY] = 5; // dead end right, straight on, have way
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][HAVEWAY] = 6;//0; // dead end right, right, have way
	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][RIGHTBEFORELEFT] = 4;// dead end right, left, right before left                
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 5; // dead end right, straight on, right before left
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][RIGHTBEFORELEFT] = 6;//0; // dead end right, right, right before left
}

StopTurningFilter::~StopTurningFilter()
{

}


tResult StopTurningFilter::Init(tInitStage eStage, __exception)
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

		tChar const * strDescTurningStartValue = pDescManager->GetMediaDescription("tTurningStartStruct");
        RETURN_IF_POINTER_NULL(strDescTurningStartValue);
        cObjectPtr<IMediaType> pTypeTurningStartValue = new cMediaType(0, 0, 0, "tTurningStartStruct", strDescTurningStartValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description Turning
		tChar const * TurningDesc = pDescManager->GetMediaDescription("tTurningStopStruct");
		RETURN_IF_POINTER_NULL(TurningDesc);
		cObjectPtr<IMediaType> pTypeTurningDesc = new cMediaType(0, 0, 0, "tTurningStopStruct", TurningDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description Sensors
		tChar const * UltraSonicDesc = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(UltraSonicDesc);
		cObjectPtr<IMediaType> pTypeUltraSonicDesc = new cMediaType(0, 0, 0, "tUltrasonicStruct", UltraSonicDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Media Description LaneTracking Velocity
        tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
        RETURN_IF_POINTER_NULL(LTVelocityDescValue);
        cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		

		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
		RETURN_IF_FAILED(pTypeTurningDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTurningInput)); 
		RETURN_IF_FAILED(pTypeUltraSonicDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUltraSonicInput)); 
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));
		RETURN_IF_FAILED(pTypeTurningStartValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTurningOutput));

		//create input pin for starting
    	RETURN_IF_FAILED(m_iStartTurning.Create("StartTurningStruct", pTypeTurningDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartTurning));

		//create input pin for US
    	RETURN_IF_FAILED(m_iUltraSonic.Create("us_struct", pTypeUltraSonicDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonic));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));

        // Start Turning Output
		RETURN_IF_FAILED(m_oStartTurning.Create("StartTurning", pTypeTurningStartValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oStartTurning));

        //create output pin for speed output data
        RETURN_IF_FAILED(m_oSpeed.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSpeed));

		// Stop LaneTracking Output
        RETURN_IF_FAILED(m_oStopLF.Create("Stop_LF", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStopLF));
		//RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopLTOutput));

		// Stop EmergencySTOP Output
        RETURN_IF_FAILED(m_oStopES.Create("Stop_ES", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStopES));
		//RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopESOutput));

		// VelocityOutput
		RETURN_IF_FAILED(m_oVelocity.Create("Velocity", pTypeLTVelocityValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oVelocity));
		RETURN_IF_FAILED(pTypeLTVelocityValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pVelocityOutput));


		//create output pin for turn-right light output data
        RETURN_IF_FAILED(m_oTurnRightOutput.Create("turn-right_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnRightOutput));

		//create output pin for turn-left light output data
        RETURN_IF_FAILED(m_oTurnLeftOutput.Create("turn-left_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnLeftOutput));
        
        // Drive Straight Output
        RETURN_IF_FAILED(m_oDriveStraight.Create("Drive_Straight", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oDriveStraight));


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
		m_showGCL = GetPropertyBool(STF_PROP_SHOW_GCL);

        // no ids were set so far
		m_bFirstFrameDepthimage			= tTrue;
		m_bIDsBoolValueOutput			= tFalse;
		m_bIDsOutputSet					= tFalse;
		m_bIDsUltraSonicSet				= tFalse;
		m_bIDsStartTurningSet			= tFalse;
		m_bIDsStartTurningOutputSet		= tFalse;
		m_bIDsDistanceSet				= tFalse;
		m_bIDsVelocityOutput			= tFalse;
		m_szIDBoolValuePullOutModeInput	= tFalse;
		m_counter = 0;

		InitialState();
    }

    RETURN_NOERROR;
}

tResult StopTurningFilter::PropertyChanged(const char* strProperty)
{

	if (NULL == strProperty || cString::IsEqual(strProperty, STF_PROP_SHOW_GCL)) {
		m_showGCL = GetPropertyBool(STF_PROP_SHOW_GCL);
	}

	RETURN_NOERROR;
}

tResult StopTurningFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}


tResult StopTurningFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult StopTurningFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult StopTurningFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult StopTurningFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
    	RETURN_IF_POINTER_NULL(pMediaSample);
		if (m_stopSign){
			if (m_waiting) {
				//m_noTraffic = tFalse;
				if(Wait()) {
					// LOG_INFO(cString::Format("SFT: Warten für Stopschild"));
					RETURN_NOERROR;
				} 
			}
		}

		/*fstream f;
		f.open("stopTurningFilter2.dat",ios::out|ios::app);
		f << "m_dist2crossing " << m_dist2crossing << " , " << "m_Circle " << m_circle << "\n";
		f.close();*/

		if (&m_iStartTurning == pSource) {
			ProcessTurningStruct(pMediaSample);
		} else if (pSource == &m_iDistance && m_stopTurningActive){
			//LOG_INFO(cString::Format("SFT: pSource distance&stopTurningActive" ));
			ProcessStoppingSpeed(pMediaSample); // slow down car speed
		} else if (pSource == &m_iDepthimagePin && m_filterActive) {
			//LOG_INFO(cString::Format("SFT: pSource depthImage" ));
			
			if (m_circle){ // circle, no traffic
				m_noTraffic = tTrue;
				RETURN_NOERROR;
			}			

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

				m_i32StartTimeStamp = 0;
        	}		
			
			if (m_dist2crossing < 0.5) {
				if (m_i32StartTimeStamp==0){
					m_i32StartTimeStamp = _clock->GetStreamTime();
				} 
				tUInt32 fTime = _clock->GetStreamTime();

				if ((fTime-m_i32StartTimeStamp)/1000000.0 > m_waiting_time){
					//LOG_INFO(cString::Format("SFT: waitingTime %f", m_waiting_time));
					ProcessInputDepth(pMediaSample);
					m_i32StartTimeStamp = fTime;
				}
			} 

        } else if (&m_iUltraSonic == pSource && m_dist2crossing < 0.05 && m_filterActive) {

			if (m_circle){ // circle, no traffic
				m_good_us_count = 3;
				RETURN_NOERROR;
			}	

			//LOG_INFO(cString::Format("SFT: pSource ultraSonic" ));
			ProcessUltraSonic(pMediaSample);
		}
	}
    RETURN_NOERROR;
}

tResult StopTurningFilter::ProcessUltraSonic(IMediaSample* pMediaSample){

	tSignalValue frontLeft, frontCenterLeft, frontCenter, frontCenterRight, frontRight;

	{
		__adtf_sample_read_lock_mediadescription(m_pUltraSonicInput, pMediaSample, pCoder);    
		
		if (!m_bIDsUltraSonicSet) {						  
			pCoder->GetID("tFrontLeft", m_szIDFrontLeftUltraSonicInput);
			pCoder->GetID("tFrontCenterLeft", m_szIDFrontCenterLeftUltraSonicInput);
			pCoder->GetID("tFrontCenter", m_szIDFrontCenterUltraSonicInput);
			pCoder->GetID("tFrontCenterRight", m_szIDFrontCenterRightUltraSonicInput);
			pCoder->GetID("tFrontRight", m_szIDFrontRightUltraSonicInput);
			m_bIDsUltraSonicSet=tTrue;
		}

		pCoder->Get(m_szIDFrontLeftUltraSonicInput, (tVoid*)&frontLeft); 
		pCoder->Get(m_szIDFrontCenterLeftUltraSonicInput, (tVoid*)&frontCenterLeft);   
		pCoder->Get(m_szIDFrontCenterUltraSonicInput, (tVoid*)&frontCenter);   
		pCoder->Get(m_szIDFrontCenterRightUltraSonicInput, (tVoid*)&frontCenterRight);   
		pCoder->Get(m_szIDFrontRightUltraSonicInput, (tVoid*)&frontRight);    
	}



	tBool goodStruct = tFalse;

	switch (m_turningMode) {
		case 1: 
			if (frontLeft.f32Value>0.30 && frontCenterLeft.f32Value>1.10 && frontCenter.f32Value>1.00 /*&& frontCenterRight.f32Value>1.15*/) { // eventuell noch ganz rechten
				//m_turningManeuvre = ACTION_LEFT;	
				goodStruct = tTrue;
			}
			break;
		case 2: 
			if (frontLeft.f32Value>0.30 && frontCenterLeft.f32Value>0.60 && frontCenter.f32Value>1.0 && frontCenterRight.f32Value>1.0) {
				//m_turningManeuvre = ACTION_STRAIGHT;
				goodStruct = tTrue;	
			}
			break;
		case 3: 
			if ((frontLeft.f32Value>0.30 && frontCenterLeft.f32Value>0.50 && frontCenter.f32Value>0.4 && frontCenterRight.f32Value>0.40) || ( frontCenterLeft.f32Value>0.50 && frontCenter.f32Value>0.4 && frontCenterRight.f32Value>0.40 && m_pullOutMode)) {
				//m_turningManeuvre = ACTION_RIGHT;	
				goodStruct = tTrue;
			}
			break;
		case 4: {

	/*fstream fi;
	fi.open("HinderniszaehlerUS.dat",ios::out|ios::app);
	fi << frontCenterLeft.f32Value <<  " frontCenterLeft.f32Value 1.10, " <<frontCenter.f32Value << " frontCenter.f32Value 1.00 "  <<"\n";
	fi.close();*/

			if (frontCenterLeft.f32Value>1.10 && frontCenter.f32Value>1.00) {
				//m_turningManeuvre = ACTION_LEFT;	
				goodStruct = tTrue;
			}
				}
			break;
		case 5: 
			if (frontCenter.f32Value>1.00) {
				//m_turningManeuvre = ACTION_STRAIGHT;
				goodStruct = tTrue;
			}
			break;
		case 6: 
			if (frontCenterRight.f32Value>0.40 /*&& frontRight.f32Value>0.50*/) {
				//m_turningManeuvre = ACTION_RIGHT;	
				goodStruct = tTrue;
			}
			break;
		case 7: 
			if (frontCenter.f32Value>1.00 && frontCenterRight.f32Value>1.0) {
				//m_turningManeuvre = ACTION_STRAIGHT;
				goodStruct = tTrue;
			}
			break;
		case 8: 
			if (frontCenter.f32Value>0.4 && frontCenterRight.f32Value>0.4 /*&& frontRight.f32Value>0.5*/) {
				//m_turningManeuvre = ACTION_RIGHT;	
				goodStruct = tTrue;
			}
			break;
		case 9: 
			if ((frontLeft.f32Value>0.30 /*1.1*/ && frontCenterLeft.f32Value>1.0 && frontCenter.f32Value>1.0 && frontCenterRight.f32Value>1.0) || (frontCenterLeft.f32Value>1.0 && frontCenter.f32Value>1.0 && frontCenterRight.f32Value>1.0 && m_pullOutMode)){ //eventuell noch ganz rechten
				//m_turningManeuvre = ACTION_LEFT;	
				goodStruct = tTrue;
				/*fstream fi;
				fi.open("HinderniszaehlerUSAusparken.dat",ios::out|ios::app);
				fi << frontCenterLeft.f32Value <<  " frontCenterLeft.f32Value 1.00, " <<frontCenter.f32Value << " frontCenter.f32Value 1.00 " << frontCenterRight.f32Value << " frontCenterRight.f32Value 1.0 " <<"\n";
				fi.close();*/
			}
			break;
		case 10: 
			if ( frontCenterLeft.f32Value>1.00 && frontCenter.f32Value>1.00) { //eventuell frontCenterLeft raus
				//m_turningManeuvre = ACTION_LEFT;	
				goodStruct = tTrue;
			}
			break;
		case 11:
			if (frontLeft.f32Value>0.30 && frontCenterLeft.f32Value>1.10 && frontCenter.f32Value>1.15 ) {
				//m_turningManeuvre = ACTION_LEFT;
				goodStruct = tTrue;
			} 
			break;
		case 12: 
			if (frontLeft.f32Value>0.30 && frontCenter.f32Value>1.00) {
				//m_turningManeuvre = ACTION_STRAIGHT;
				goodStruct = tTrue;
			}
			break;
		case 13: 
			if (frontCenterLeft.f32Value>0.30 /*1.1*/ && frontCenter.f32Value>1.00 && frontCenterRight.f32Value>1.10) { // eventuell noch ganz rechten
				//m_turningManeuvre = ACTION_LEFT;	
				goodStruct = tTrue;
			}
			break;
		case 14: 
			if (frontCenter.f32Value>1.00 && frontCenterRight.f32Value>1.10) { // eventuell noch ganz rechten
				//m_turningManeuvre = ACTION_STRAIGHT;
				goodStruct = tTrue;
			}
			break;
		case 15: 
			if (frontCenterLeft.f32Value>1.10 && frontCenter.f32Value>1.00 && frontCenterRight.f32Value>1.10) { // eventuell noch ganz rechten
				//m_turningManeuvre = ACTION_LEFT;
				goodStruct = tTrue;
			}
			break;
		}

		if (goodStruct) {
			++m_good_us_count;
		} else {
			++m_bad_us_count;
			if (m_bad_us_count >=1) {
				m_good_us_count = 0;
				m_bad_us_count = 0;
			}
		}

	tFloat32 fWaitingTime = (_clock->GetStreamTime() - m_i32TimeStampHoldLine) /1000000.0;

	/*fstream f;
	f.open("stopTurningFilter2.dat",ios::out|ios::app);
	f << "fWaitingTime " << fWaitingTime << " , " << " m_stopTurningActive " << m_stopTurningActive <<  " , " << " m_good_us_count " <<  m_good_us_count << " , " << " m_bad_us_count " << m_bad_us_count << " , " << " m_noTraffic "<< m_noTraffic << "\n";
	f.close();*/


				
	//LOG_INFO(cString::Format("TF sensoren: fWaitingTime %f, m_stopTurningActive %i, m_good_us_count %i, m_bad_us_count %i, m_noTraffic %i, %i",fWaitingTime, m_stopTurningActive, m_good_us_count, m_bad_us_count, m_noTraffic, m_counter));

	if (((m_good_us_count >= 3 && m_noTraffic && !m_waiting) || (fWaitingTime >= MAX_WAITING_TIME)) && !m_stopTurningActive) {
		LOG_INFO(cString::Format("STF: start Turning, %i", m_counter)); 

		/*f.open("stopTurningFilter2.dat",ios::out|ios::app);
		f << "startTurning " << "\n";
		f.close();*/
		StartTurning();

		
	} else if (!m_stopTurningActive) {
		TransmitVelocityLT(0.0);
		//ChangeSpeed(0.0);
	}
			
			
	RETURN_NOERROR;

}

tResult StopTurningFilter::ChangeSpeed(tFloat32 fSpeed){

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

		LOG_INFO(cString::Format("STF speed = %f",fSpeed));

        //transmit media sample over output pin

        RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oSpeed.Transmit(pMediaSampleValue1));
	}
	RETURN_NOERROR;
}


tResult StopTurningFilter::ProcessTurningStruct(IMediaSample* pMediaSample){
	m_counter++;

	/*fstream f2;
	f2.open("Start_stopTurningFilter.dat",ios::out|ios::trunc);
	f2 << " gestartet!! ";
	f2.close();

	f2.open("Hinderniszaehler.dat",ios::out|ios::trunc);
	f2 << "gestartet";
	f2.close();

	f2.open("stopTurningFilter.dat",ios::out|ios::trunc);
	f2 << "gestartet";
	f2.close();*/

	tBool filterActive = m_filterActive;	

	tFloat32 	velocity = 0.0;
	tFloat32 	distance = -1.0;
	tInt32		mode = -1;
	tInt32		type = -1;
	tInt32		giveWay = -1;
	tFloat32	rotationDegree = 0.0;
	tBool		stopSign = tFalse;
	tBool 		bValue = tFalse;
	tBool 		bValuePullOutMode = tFalse;
	tBool		bCircle = tFalse;
	tBool		bIsInCircle = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pStartTurningInput, pMediaSample, pCoder);    

		if (!m_bIDsStartTurningSet){
			pCoder->GetID("fVelocity", m_szIDVelocityStartTurningInput);
			pCoder->GetID("fDistance", m_szIDDistanceStartTurningInput);
			pCoder->GetID("type", m_szIDTypeStartTurningInput);
			pCoder->GetID("mode", m_szIDModeStartTurningInput);
			pCoder->GetID("giveWay", m_szIDGiveWayStartTurningInput);
			pCoder->GetID("stopSign", m_szIDStopSignStartTurningInput);
			pCoder->GetID("bStart", m_szIDBoolValueStartTurningInput);
			pCoder->GetID("bPullOutMode", m_szIDBoolValuePullOutModeInput);
			pCoder->GetID("rotationDegree", m_szIDBoolValueRotationDegreeInput);
			pCoder->GetID("bCircle", m_szIDBoolValueCircleInput);
			pCoder->GetID("bIsInCircle", m_szIDBoolValueIsInCircleInput);
			m_bIDsStartTurningSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDVelocityStartTurningInput, (tVoid*)&velocity);
		pCoder->Get(m_szIDDistanceStartTurningInput, (tVoid*)&distance);
		pCoder->Get(m_szIDTypeStartTurningInput, (tVoid*)&type);      
		pCoder->Get(m_szIDModeStartTurningInput, (tVoid*)&mode);   
		pCoder->Get(m_szIDGiveWayStartTurningInput, (tVoid*)&giveWay);   
		pCoder->Get(m_szIDStopSignStartTurningInput, (tVoid*)&stopSign);   
		pCoder->Get(m_szIDBoolValueStartTurningInput, (tVoid*)&bValue);     
		pCoder->Get(m_szIDBoolValuePullOutModeInput, (tVoid*)&bValuePullOutMode);
		pCoder->Get(m_szIDBoolValueRotationDegreeInput, (tVoid*)&rotationDegree);
		pCoder->Get(m_szIDBoolValueCircleInput, (tVoid*)&bCircle);
		pCoder->Get(m_szIDBoolValueIsInCircleInput, (tVoid*)&bIsInCircle);
	}

	if (!bValue) {
		TransmitTurningStruct(ACTION_UNKNOWN); //stop turning and shutdown this filter
		if (m_DriveStraightSet){
			TransmitBoolValue(&m_oDriveStraight, tFalse);
			m_DriveStraightSet 	= tFalse;
		}
		TransmitBoolValue(&m_oTurnRightOutput, tFalse);
		TransmitBoolValue(&m_oTurnLeftOutput, tFalse);
		TransmitBoolValue(&m_oStopES, tTrue);
		InitialState();
		LOG_INFO(cString::Format("STF: shutdown stop turning filter, %i", m_counter));
		RETURN_NOERROR;		
	} else if (filterActive){
		LOG_INFO(cString::Format("STF: filter is active, don't start, %i", m_counter));
		RETURN_NOERROR;	// filter is active, don't start again
	}

	TransmitBoolValue(&m_oStopLF, tTrue);
	TransmitBoolValue(&m_oStopES, tTrue);



	m_velocityZero 		= velocity;
	m_distanceZero 		= distance;
	m_stopSign 			= stopSign;
	m_giveWay 			= giveWay;
	m_turningType		= type;
	m_turningMode 		= turningModeLookup[type][mode][giveWay]; 
	m_dist2crossing 	= distance;
	m_pullOutMode		= bValuePullOutMode;
	m_rotationDegree 	= rotationDegree;
	m_circle 			= bCircle;
	m_isInCircle		= bIsInCircle;

	/*fstream f2;
	f2.open("stopTurningFilterDistances.dat",ios::out|ios::app);
	f2 << distance << "\n";
	f2.close();*/

	switch (m_turningMode) {
		case 1: 
		case 4: 
		case 9: 
		case 10: 
		case 11: 
		case 13: 
		case 15: 
			m_turningManeuvre = ACTION_LEFT;
			break;
		case 2: 
		case 5: 
		case 7: 
		case 12: 
		case 14: 
			m_turningManeuvre = ACTION_STRAIGHT;
			break;
		case 3: 
		case 6:
		case 8:
			m_turningManeuvre = ACTION_RIGHT;	
			break;
	}



	if (m_distanceZero > 0.0) { // no stopping if distance to crossing is zero
		if (m_velocityZero == 0.0){ // starte Auto manuell
			m_velocityZero = MIDDLE_SPEED;
			ChangeSpeed(MIDDLE_SPEED);
		}
		m_stopTurningActive = bValue;
		LOG_INFO(cString::Format("STF: m_stopTurningActive %i, %i", bValue, m_counter)); 
	} else {
		LOG_INFO(cString::Format("STF: m_distanceZero < 0 %f, %i", m_distanceZero, m_counter)); 
		m_i32TimeStampHoldLine = _clock->GetStreamTime();

		if (m_stopSign){
			m_i32TimeStampHoldLine += 3000000;
			Wait();
		} 
        TransmitBoolValue(&m_oStopLF, tFalse);
	}

	/*fstream f;
	f.open("stopTurningFilter.dat",ios::out|ios::app);
	f << " m_distanceZero "<< m_distanceZero <<"\n";
	f << "stopSign " << stopSign <<"\n";
	f << "m_giveWay " << m_giveWay  <<"\n";
	f.close();*/

	LOG_INFO(cString::Format("STF: StartTurning stopSign %i | giveWay %i | type %i | mode %i | filterActive %i | Distance = %f | circle = %i | %i", m_stopSign, m_giveWay, m_turningType, m_turningMode, bValue, m_distanceZero, m_circle, m_counter) );

	m_filterActive 	= bValue;

	RETURN_NOERROR;
}

tResult StopTurningFilter::StartTurning()
{
	TransmitBoolValue(&m_oStopES, tFalse);
	TransmitBoolValue(&m_oStopLF, tFalse);

	if (m_DriveStraightSet){
		TransmitBoolValue(&m_oDriveStraight, tFalse);
		m_DriveStraightSet 	= tFalse;
	}
	
	TransmitTurningStruct(m_turningManeuvre);
	LOG_INFO(cString::Format("STF: Biege ab!!, %i", m_counter));
	InitialState();

	RETURN_NOERROR;
}

tResult StopTurningFilter::TransmitTurningStruct(tInt32 turningManeuvre){

	//create new media sample for start turning
    {
        cObjectPtr<IMediaSample> pMediaSampleValue;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue));

        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pStartTurningOutput->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        RETURN_IF_FAILED(pMediaSampleValue->AllocBuffer(nSize));


        {   // focus for sample write lock
            //write date to the media sample with the coder of the descriptor
            __adtf_sample_write_lock_mediadescription(m_pStartTurningOutput,pMediaSampleValue,pCoder);

            // get the IDs for the items in the media sample
            if(!m_bIDsStartTurningOutputSet)
            {
                pCoder->GetID("mode", m_szIDStartTurningMode);
                pCoder->GetID("bPullOutMode", m_szIDStartTurningPullOut);
				pCoder->GetID("rotationDegree", m_szIDStartTurningRotationDegree);
				pCoder->GetID("bCircle", m_szIDStartTurningCircle);
				m_bIDsStartTurningOutputSet = tTrue;
            }

            pCoder->Set(m_szIDStartTurningMode, (tVoid*)&turningManeuvre);
            pCoder->Set(m_szIDStartTurningPullOut, (tVoid*)&m_pullOutMode);
			pCoder->Set(m_szIDStartTurningRotationDegree, (tVoid*)&m_rotationDegree);
			pCoder->Set(m_szIDStartTurningCircle, (tVoid*)&m_circle);
        }

        //transmit media sample over output pin
        RETURN_IF_FAILED(pMediaSampleValue->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oStartTurning.Transmit(pMediaSampleValue));
    }

	RETURN_NOERROR;
}

tResult StopTurningFilter::ProcessStoppingSpeed(IMediaSample* pMediaSample){

    tFloat32 fDistance = 0.0;
	tUInt32 fTimeStamp = 0;
	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

		if (!m_bIDsDistanceSet) {
			pCoder->GetID("f32Value", m_szIDDistanceInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput); 
			m_bIDsDistanceSet = tTrue;
		}  

		pCoder->Get(m_szIDDistanceInput, (tVoid*)&fDistance);
		pCoder->Get(m_szIDTimestampDistanceInput, (tVoid*)&fTimeStamp);
	
	}
	// calc Speed

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = fDistance;
	}

	fDistance -= m_distanceOffset;
		//LOG_INFO(cString::Format("STF: fdistance %f",fDistance));
	m_dist2crossing = m_distanceZero - fDistance;
	tFloat32 faux = 1.0-fDistance/m_distanceZero;

	tFloat32 fSpeed;


	// turn on lights if necessary
	if (fDistance<=2) {

		if (m_turningManeuvre == ACTION_RIGHT && !m_turnRightON && (!m_circle || m_isInCircle)) {

			m_turnRightON = tTrue;
			TransmitBoolValue(&m_oTurnRightOutput, tTrue);

		} else if (m_turningManeuvre == ACTION_LEFT && !m_turnLeftON && (!m_circle || m_isInCircle)) {

			m_turnLeftON = tTrue;
			TransmitBoolValue(&m_oTurnLeftOutput, tTrue); 
		}
	}

		/*fstream f;
		f.open("AAAstopTurningFilter.dat",ios::out|ios::app);
		f << m_dist2crossing <<"\n";
		f.close();*/

	if (faux < 1e-4) {
		LOG_INFO(cString::Format("STF: an Haltelinie, %i", m_counter) );
		//TransmitBoolValue(&m_oStopLF, tFalse);
		m_turnRightON = tFalse;
		m_turnLeftON = tFalse;
		m_stopTurningActive = tFalse;
		fSpeed = 0.0;
		m_i32TimeStampHoldLine = _clock->GetStreamTime();
		if (m_stopSign){
			m_waiting = tTrue;
			m_i32TimeStampHoldLine += 3000000;
		}
		m_dist2crossing = 0.0;

		if (m_circle){
			LOG_INFO(cString::Format("STF: biege ab, circle, %i", m_counter) );
			TransmitVelocityLT(fSpeed);
			StartTurning();
		}

		/* fstream f;
		f.open("stopTurningFilter.dat",ios::out|ios::app);
		f << "STD: ProcessStoppingSpeed faux < 1e-4"<<"\n";
		f.close();*/

	} else if ((m_distanceZero-fDistance) < 0.15) {

		if (!m_DriveStraightSet){
			TransmitBoolValue(&m_oDriveStraight, tTrue);
			m_DriveStraightSet 	= tTrue;
		}

		fSpeed = MIN_SPEED;
	} else {
		fSpeed = sqrt(faux)*m_velocityZero;

		if (fSpeed < MIDDLE_SPEED) {
			fSpeed = MIDDLE_SPEED;
		}
	}


	if (m_stopTurningActive || m_good_us_count < 3 || !m_noTraffic) {
		TransmitVelocityLT(fSpeed);
	} else {
	
		if (m_stopSign){
			TransmitVelocityLT(fSpeed);
			if(Wait()) {
				RETURN_NOERROR;
			}
		} 
	}

	RETURN_NOERROR;
}

bool StopTurningFilter::Wait() {

	if (m_startWait == 0) {
		m_startWait = _clock->GetStreamTime();
	}

	if ((_clock->GetStreamTime() - m_startWait)/1000000.0 < 3) {
		return true;	
	} else {
		m_startWait = 0;
		m_waiting = tFalse;
		return false;
	}
}

tResult StopTurningFilter::TransmitVelocityLT(tFloat32 fSpeed)
{
	cObjectPtr<IMediaSample> pNewMediaSample;
	AllocMediaSample((tVoid**)&pNewMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pVelocityOutput->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	pNewMediaSample->AllocBuffer(nSize);

   //write date to the media sample with the coder of the descriptor
	{
	    __adtf_sample_write_lock_mediadescription(m_pVelocityOutput, pNewMediaSample, pCoderOutput);

		// set the id if not already done
	    if(!m_bIDsVelocityOutput)
	    {
	    	pCoderOutput->GetID("fMinVel", m_szIDMinVelocityOutput);
			pCoderOutput->GetID("fMaxVel", m_szIDMaxVelocityOutput);
	    	//pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDVelocityTimestampOutput);
	    	m_bIDsVelocityOutput = tTrue;
	    }

		pCoderOutput->Set(m_szIDMinVelocityOutput, (tVoid*)&(fSpeed));
		pCoderOutput->Set(m_szIDMaxVelocityOutput, (tVoid*)&(fSpeed));
		//pCoderOutput->Set(m_szIDVelocityTimestampOutput, (tVoid*)&nTimeStamp);

		pNewMediaSample->SetTime(pNewMediaSample->GetTime());
	}

	m_oVelocity.Transmit(pNewMediaSample);

	if (fSpeed==0.0){
		//LOG_INFO(cString::Format("STF: fSpeed = 0.0 gesendet"));
	}

	RETURN_NOERROR;

}

tResult StopTurningFilter::ProcessInputDepth(IMediaSample* pSample)
{

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

	m_c1 = m_c2 = m_c4 = m_c5 = m_c6 = m_c7 = m_c8 = m_c9 = m_c8_r = m_c8_l = m_c5_b = m_c5_f= 0;
	m_c1all = m_c2all = m_c4all = m_c5all = m_c6all = m_c7all = m_c8all = m_c9all= m_c8_lall = m_c8_rall = m_c5_ball = m_c5_fall = 0;

	aux1.clear();
	aux2.clear();
	aux4.clear();
	aux5.clear();
	aux6.clear();
	aux7.clear();
	aux8.clear();
	auxObs.clear();

	for (int j=25;j<180;j++) {
		for (int i=0; i<320; i++) {
			Point3f coordinates = Util::ComputeWorldCoordinate(2*i,2*j,depthImage.at<ushort>(j, i), 0,0);
			coordinates.x += OFFSET_X;			
			countObstacles(coordinates, i,j);
		}
	}			
	
	m_noTraffic = checkObstacles();

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


	CreateAndTransmitGCL();

	
	RETURN_NOERROR;            
}

void StopTurningFilter::countObstacles(Point3f &p, int i, int j) {


	if (p.x < 0.28 - LANE_OFFSET_X && p.x > -0.185 + LANE_OFFSET_X && p.z > 0.2 && p.z < 1.26 + m_dist2crossing + 0.6/*car*/) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c1;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c1all;
		if(m_showGCL) {aux1.push_back(i); aux1.push_back(j);}
		
		if (p.z < 1.26 + m_dist2crossing) {
			if (p.y < 0.22 - OBJECT_HEIGHT) {
				++m_c2;
			}
			++m_c2all;
		}
	} else if (p.x > -0.65 - 0.6/*car*/ && p.x < -0.185 && p.z > 0.82 + m_dist2crossing + LANE_OFFSET_Z && p.z < 1.26 + m_dist2crossing - LANE_OFFSET_Z) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c4;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c4all;
		
		if(m_showGCL) {aux4.push_back(i); aux4.push_back(j);}
		
	} else if (p.x > -0.65 + LANE_OFFSET_X && p.x < -0.25 - LANE_OFFSET_X && p.z > 1.26 + m_dist2crossing && p.z < 1.26 + m_dist2crossing + 1.5) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c5;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}

		if (p.z > 1.26 + m_dist2crossing + 1.00) {
			if (p.z > 1.26 + m_dist2crossing + 1.25) {
				if (p.y < 0.22 - OBJECT_HEIGHT) {
					++m_c5_b;
				}
				++m_c5_ball;
			} else {
				if (p.y < 0.22 - OBJECT_HEIGHT) {
					++m_c5_f;
				}
				++m_c5_fall;
			}
		}
		
		++m_c5all;
		
		if(m_showGCL) {aux5.push_back(i); aux5.push_back(j);}
		
	} else if (p.x > -0.65 - 1.5 && p.x < -0.185 && p.z > 0.345 + m_dist2crossing + LANE_OFFSET_Z && p.z < 0.8 + m_dist2crossing - LANE_OFFSET_Z) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c6;
			if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_c6all;
		
		if(m_showGCL) {aux6.push_back(i); aux6.push_back(j);}
		
		if (p.x > -0.65) {
			if (p.y < 0.22 - OBJECT_HEIGHT) {
				++m_c9;
			}
			++m_c9all;
		}
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

tBool StopTurningFilter::checkObstacles() {
	
	static const float pObs = 0.1;
	tBool testBox8 = tFalse;
	tBool testBox5 = tFalse;
	tBool noObstacle = tFalse;

	++m_c1all;
	++m_c2all;
	++m_c4all;
	++m_c5all;
	++m_c6all;
	++m_c7all;
	++m_c8all;
	++m_c9all;


	switch (m_turningMode) {
		case 1:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs  && m_c5/(tFloat32)m_c5all < pObs && m_c6/(tFloat32)m_c6all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
				testBox8 = tTrue;
				testBox5 = tTrue;
			}
			break;
		case 2:
			if (m_c1/(tFloat32)m_c1all < pObs && m_c6/(tFloat32)m_c6all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
				testBox8 = tTrue;
			}
			break;
		case 3:
			if (m_c6/(tFloat32)m_c6all < pObs && m_c7/(tFloat32)m_c7all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 4:
			/*{fstream f2;
			f2.open("Hinderniszaehler.dat",ios::out|ios::app);

			f2 << m_c2 <<  ", " <<m_c4 << ", " <<m_c5 <<  ", " <<m_c9  <<"\n";
			f2 << m_c2all <<  ", " <<m_c4all << ", " <<m_c5all << ", " <<m_c9all  <<"\n";
			f2 << m_c2/(tFloat32)m_c2all <<  ", " << m_c4/(tFloat32)m_c4all << ", "  << m_c9/(tFloat32)m_c9all  <<"\n";
			f2 << "_____" << "\n";*/

			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c5/(tFloat32)m_c5all < pObs && m_c9/(tFloat32)m_c9all < pObs) {
				testBox5 = tTrue;
				//f2 << "testBox5" << "\n";
			}

			//f2.close()};

			break;
		case 5:
			if (m_c1/(tFloat32)m_c1all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 6:
			if (m_c7/(tFloat32)m_c7all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 7:
			if (m_c1/(tFloat32)m_c1all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
				testBox8 = tTrue;
			}
			break;
		case 8:
			if (m_c7/(tFloat32)m_c7all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 9:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c6/(tFloat32)m_c6all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
				testBox8 = tTrue;
			}
			break;
		case 10:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c9/(tFloat32)m_c9all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 11:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c5/(tFloat32)m_c5all < pObs && m_c6/(tFloat32)m_c6all < pObs) {
				testBox5 = tTrue;
			}
			break;
		case 12:
			if (m_c1/(tFloat32)m_c1all < pObs && m_c6/(tFloat32)m_c6all < pObs) {
				noObstacle = tTrue;
			}
			break;
		case 13:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c5/(tFloat32)m_c5all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs  && m_c9/(tFloat32)m_c9all < pObs) {
				testBox8 = tTrue;
				testBox5 = tTrue;
			}
			break;
		case 14:
			if (m_c1/(tFloat32)m_c1all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs) {
				testBox8 = tTrue;
			}
			break;
		case 15:
			if (m_c2/(tFloat32)m_c2all < pObs && m_c4/(tFloat32)m_c4all < pObs && m_c7/(tFloat32)m_c7all < pObs && m_c8/(tFloat32)m_c8all < pObs && m_c9/(tFloat32)m_c9all < pObs) {
				testBox8 = tTrue;
			}
			break;
	}
/*LOG_INFO(cString::Format("SFT: box 6=%f",m_c6/(tFloat32)m_c6all));
LOG_INFO(cString::Format("SFT: box 7=%f",m_c7/(tFloat32)m_c7all));
LOG_INFO(cString::Format("SFT: goodFrameCount=%i",m_good_frame_count)); */


/*LOG_INFO(cString::Format("SFT: test box 8 bool=%i",testBox8));
LOG_INFO(cString::Format("SFT: %f, %f",m_c8_l/(tFloat32)(m_c8_lall+1), m_c8_r/(tFloat32)(m_c8_rall+1)));

LOG_INFO(cString::Format("SFT: test box 5 bool=%i",testBox5));
LOG_INFO(cString::Format("SFT: %f, %f", m_c5_b/(tFloat32)(m_c5_ball+1), m_c5_f/(tFloat32)(m_c5_fall+1)));*/

	if (testBox8 && m_text_minibox) {
		// test box 8
		if (m_c8_l/(tFloat32)(m_c8_lall+1) > pObs || m_c8_r/(tFloat32)(m_c8_rall+1) > pObs) {
			++m_bad_frame_8_count;
			++m_bad_frame_count;
			m_waiting_time = 0.5 * m_bad_frame_8_count;
		} else if (testBox8){
			noObstacle = tTrue;
		}
	}

	if (testBox5 && m_text_minibox) {
		if (m_c5_b/(tFloat32)(m_c5_ball+1) > pObs || m_c5_f/(tFloat32)(m_c5_fall+1) > pObs) {

			/*fstream f9;
			f9.open("Hinderniszaehler.dat",ios::out|ios::app);
			f9 << "box 5 nicht frei!"  <<"\n";
			f9.close();*/

			++m_bad_frame_5_count;
			++m_bad_frame_count;
			m_waiting_time = 0.5 * m_bad_frame_5_count;

				if (m_bad_frame_8_count*0.5 > m_waiting_time) {
					m_waiting_time = m_bad_frame_8_count*0.5;
				}
			noObstacle = tFalse;
		} else {
			if ((testBox8 && noObstacle) || !testBox8){
				noObstacle = tTrue;
			}
		}
	}

	if (m_bad_frame_8_count >= 5 || m_bad_frame_5_count >= 5){ //5 Bilder hintereinander keine Veränderung ( 2 sec)
		// Annahme: kein Hindernis
		noObstacle=tTrue;
		m_text_minibox = tFalse;
	}

	if (noObstacle) {
		++m_good_frame_count;	
		m_waiting_time = 0.08;	
	} else {
		++m_bad_frame_count;
		if (!testBox8 && !testBox5) {
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

tResult StopTurningFilter::CreateAndTransmitGCL()
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
	for (size_t i = 0; i < aux1.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux1[i]*2-1, aux1[i+1]*2-1, aux1[i]*2+1, aux1[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	for (size_t i = 0; i < aux4.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux4[i]*2-1, aux4[i+1]*2-1, aux4[i]*2+1, aux4[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 255).GetRGBA());	
	for (size_t i = 0; i < aux5.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, aux5[i]*2-1, aux5[i+1]*2-1, aux5[i]*2+1, aux5[i+1]*2+1); 
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 70, 0).GetRGBA());	
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

	tFloat64 text = m_distanceZero;
	cString strText = cString::FromFloat64(text);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr());


    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}


tResult StopTurningFilter::TransmitBoolValue(cOutputPin* oPin, tBool value)
{
	__synchronized_obj(m_oTransmitBoolCritSection);
	
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = value;
    tUInt32 ui32TimeStamp = _clock->GetStreamTime();

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

void StopTurningFilter::InitialState(){

	m_text_minibox					= tTrue;
	m_stopTurningActive				= tFalse;
	m_turnRightON					= tFalse;
	m_turnLeftON					= tFalse;
	m_DriveStraightSet				= tFalse;
	m_waiting						= tFalse;
	m_filterActive					= tFalse;
	m_noTraffic						= tFalse;
	m_pullOutMode					= tFalse;
	m_turningManeuvre				= ACTION_UNKNOWN;
	m_waiting_time					= 0.08;
	m_distanceOffset				= 0.0;
	m_velocityZero					= 0.0;
	m_distanceZero					= 0.0;
	m_dist2crossing 				= 0.0;
	m_i32TimeStampHoldLine			= 0;
	m_i32StartTimeStamp				= 0;
	m_startWait						= 0;
	m_turningMode					= 0;
	m_stopSign						= 0;
	m_good_frame_count				= 0;
	m_bad_frame_count				= 0;
	m_bad_frame_8_count				= 0;
	m_good_us_count					= 0;
	m_bad_us_count					= 0;
	m_bad_frame_5_count				= 0;
	m_rotationDegree				= 0.0;
	m_giveWay						= -1;
	m_turningType					= -1;
	m_circle						= tFalse;
	m_isInCircle 					= tFalse;

}



