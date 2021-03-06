/*
 * Date 17.01.16   
 */   
 
#include "stdafx.h"
#include "LaneTracking.h"
#include "/home/aadc/AADC/src/aadcUser/include/lane_type.h"

#include <fstream>

#define WIDTH_MIN_FAR 0
#define WIDTH_MIN_NEAR 0
#define NEAR_FIELD 0
#define FAR_FIELD 1

ADTF_FILTER_PLUGIN("LaneTracking", OID_ADTF_LaneTracking, cLaneTracking)

// define the ADTF property names to avoid errors 
#define LT_PROP_NEAR_LINE "LaneDetection::Near area::Near Line"
#define LT_PROP_NEAR_LINE_MAX_OFFSET "LaneDetection::Near area::Max offset for near line"
#define LT_PROP_LANE_WIDTH_MIN_NEAR "LaneDetection::Near area::Lane width min near"
#define LT_PROP_LANE_WIDTH_MAX_NEAR "LaneDetection::Near area::Lane width max near"

#define LT_PROP_FAR_LINE "LaneDetection::Far area::Far Line"
#define LT_PROP_LANE_WIDTH_MIN_FAR "LaneDetection::Far area::Lane width min far"
#define LT_PROP_LANE_WIDTH_MAX_FAR "LaneDetection::Far area::Lane width max far"

#define LT_PROP_CAMERA_OFFSET "LaneDetection::Camera Offset"
#define LT_PROP_TRESHOLD "LaneDetection::ThresholdValue"


#define LT_PROP_MAX_ACCELERATION "LongitudinalControl::Max Acceleration"
#define LT_PROP_MIN_ACCELERATION "LongitudinalControl::Min Acceleration"
#define LT_PROP_ACCELERATION_FAR_NEAR_DIFF "LongitudinalControl::Far Near difference"

#define LT_PROP_SHOW_DEBUG "Common::Show Debug"
#define LT_PROP_ENABLE_LIGHTBEAM_TRIGGER "Common::Enable Lightbeam Trigger"
#define LT_PROP_DRIVE_TIME "Common::Drive Time in milliseconds"
#define LT_PROP_EMERGENCY_STOP_TIME "Common::Emergency Stop Time in milliseconds"

#define LT_PROP_CONTR_PROPORTIONAL_GAIN "PID::Controller Proportional Gain"
#define LT_PROP_CONTR_INTEGRAL_GAIN "PID::Controller Integral Gain"
#define LT_PROP_CONTR_DIFFERENTIAL_GAIN "PID::Controller Differential Gain"

#define LT_PROP_PT1_TAU "PT1::Tau"
#define LT_PROP_PT1_SAMPLE_TIME "PT1::Sample_Time"
#define LT_PROP_PT1_INPUT_FACTOR "PT1::InputFactor"

#define LT_PROP_ImagecutWidthLeftHough "LT::ImagecutWidthLeftHough"
#define LT_PROP_ImagecutWidthRightHough "LT::ImagecutWidthRightHough"
#define LT_PROP_ImagecutHeightUpHough "LT::ImagecutHeightUpHough"
#define LT_PROP_ImagecutHeightDownHough "LT::ImagecutHeightDownHough"
#define LT_PROP_ThresholdValueCanny "LT::ThresholdValueCanny"
#define LT_PROP_ThresholdValueHough "LT::ThresholdValueHough"


// id and name definitions for signal registry (increase the id for new signals)
#define LT_SIGREG_ID_SCALED_PT1_INPUT 0
#define LT_SIGREG_NAME_SCALED_PT1_INPUT "scaled pt1 input"
#define LT_SIGREG_UNIT_SCALED_PT1_INPUT "pixel"

#define LT_SIGREG_ID_CONTROLLER_INPUT 1
#define LT_SIGREG_NAME_CONTROLLER_INPUT "controller input"
#define LT_SIGREG_UNIT_CONTROLLER_INPUT "pixel"



#define MAX_DEVIATION 150

// used to identify the braking timer
const tUInt8 g_ui8StopTimerIdNegative = 77;
const tUInt8 g_ui8StopTimerIdZero = 78;
const tUInt8 g_ui8EmergencyStopTimerIdNegative = 87;
const tUInt8 g_ui8EmergencyStopTimerIdZero = 88;
const tUInt8 g_ui8EmergencyStopTimerIdResume = 89;

//#define LT_ENABLE_CANNY_WINDOWS


cLaneTracking::cLaneTracking(const tChar* __info) : cFilter(__info), 
    m_hStopTimerNegative(NULL),
    m_hStopTimerZero(NULL),
    m_hEmergencyStopTimerNegative(NULL),
    m_hEmergencyStopTimerZero(NULL),
    m_hEmergencyStopTimerResume(NULL)
{
    SetPropertyInt(LT_PROP_NEAR_LINE, 350);
    SetPropertyBool(LT_PROP_NEAR_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_NEAR_LINE NSSUBPROP_DESCRIPTION, "The y value of the near line used for lateral control.");

    SetPropertyInt(LT_PROP_NEAR_LINE_MAX_OFFSET, -25);
    SetPropertyBool(LT_PROP_NEAR_LINE_MAX_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_NEAR_LINE_MAX_OFFSET NSSUBPROP_DESCRIPTION, "The maximum offset to adjust near line to the current speed.");

    SetPropertyInt(LT_PROP_FAR_LINE, 250);
    SetPropertyBool(LT_PROP_FAR_LINE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_FAR_LINE NSSUBPROP_DESCRIPTION, "The y value of the far line used for longitudinal control.");
    
    SetPropertyBool(LT_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(LT_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

    SetPropertyBool(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER, tFalse);
    SetPropertyStr(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER NSSUBPROP_DESCRIPTION, "If true, start_trigger pin will be used to start driving.");
    
    SetPropertyInt(LT_PROP_DRIVE_TIME, 25000);
    SetPropertyInt(LT_PROP_DRIVE_TIME NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_DRIVE_TIME NSSUBPROP_DESCRIPTION, "If enable lightbeam trigger is set to true, this value will be used to stop driving after the given time." \
                                                                "If the value is 0, the stop trigger is disabled.");

    SetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME, 0);
    SetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_EMERGENCY_STOP_TIME NSSUBPROP_DESCRIPTION, "If enable lightbeam trigger is set to true, this value will be used to perform an emergency stop after the given time." \
        "If the value is 0, the emergency stop trigger is disabled.");

    SetPropertyFloat(LT_PROP_MAX_ACCELERATION, 1.5);
    SetPropertyBool(LT_PROP_MAX_ACCELERATION NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_MAX_ACCELERATION NSSUBPROP_DESCRIPTION, "Acceleration value used on a straight.");
    
    SetPropertyFloat(LT_PROP_MIN_ACCELERATION, 1.0);
    SetPropertyBool(LT_PROP_MIN_ACCELERATION NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_MIN_ACCELERATION NSSUBPROP_DESCRIPTION, "Acceleration value used in a turn.");

    SetPropertyInt(LT_PROP_ACCELERATION_FAR_NEAR_DIFF, 40);
    SetPropertyBool(LT_PROP_ACCELERATION_FAR_NEAR_DIFF NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ACCELERATION_FAR_NEAR_DIFF NSSUBPROP_DESCRIPTION, "The difference between far and near point in pixel.");

    SetPropertyFloat(LT_PROP_CAMERA_OFFSET, 15);
    SetPropertyBool(LT_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");

    SetPropertyFloat(LT_PROP_CONTR_PROPORTIONAL_GAIN, 0.15);
    SetPropertyBool(LT_PROP_CONTR_PROPORTIONAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_CONTR_PROPORTIONAL_GAIN NSSUBPROP_DESCRIPTION, "The proportional gain of the PID controller.");
    
    SetPropertyFloat(LT_PROP_CONTR_INTEGRAL_GAIN, 0.01);
    SetPropertyBool(LT_PROP_CONTR_INTEGRAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_CONTR_INTEGRAL_GAIN NSSUBPROP_DESCRIPTION, "The integral gain of the PID controller.");
    
    SetPropertyFloat(LT_PROP_CONTR_DIFFERENTIAL_GAIN, 0.015);
    SetPropertyBool(LT_PROP_CONTR_DIFFERENTIAL_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_CONTR_DIFFERENTIAL_GAIN NSSUBPROP_DESCRIPTION, "The differential gain of the PID controller.");

    SetPropertyInt(LT_PROP_TRESHOLD, 150);
    SetPropertyBool(LT_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The threshold value for canny to detect lines.");
    
    SetPropertyFloat(LT_PROP_PT1_TAU, 0.1);
    SetPropertyBool(LT_PROP_PT1_TAU NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_PT1_TAU NSSUBPROP_DESCRIPTION, "The tau value of the PT1 controller.");
    
    SetPropertyFloat(LT_PROP_PT1_SAMPLE_TIME, 0.9);
    SetPropertyBool(LT_PROP_PT1_SAMPLE_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_PT1_SAMPLE_TIME NSSUBPROP_DESCRIPTION, "The sample time of the PT1 controller.");

    SetPropertyFloat(LT_PROP_PT1_INPUT_FACTOR, 0.1);
    SetPropertyBool(LT_PROP_PT1_INPUT_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_PT1_INPUT_FACTOR NSSUBPROP_DESCRIPTION, "The factor to normalize the input value (difference of the lane center and the place to be) to the range of the output values.");

    SetPropertyInt(LT_PROP_LANE_WIDTH_MIN_NEAR, 320);
    SetPropertyBool(LT_PROP_LANE_WIDTH_MIN_NEAR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_LANE_WIDTH_MIN_NEAR NSSUBPROP_DESCRIPTION, "The Minimum value for the near lane width. If the calculated lane width is smaller than this value the blind count will be increased.");

    SetPropertyInt(LT_PROP_LANE_WIDTH_MAX_NEAR, 430);
    SetPropertyBool(LT_PROP_LANE_WIDTH_MAX_NEAR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_LANE_WIDTH_MAX_NEAR NSSUBPROP_DESCRIPTION, "The Maximum value for the near lane width. If the calculated lane width is greater than this value the blind count will be increased.");


    SetPropertyInt(LT_PROP_LANE_WIDTH_MIN_FAR, 120);
    SetPropertyBool(LT_PROP_LANE_WIDTH_MIN_FAR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_LANE_WIDTH_MIN_FAR NSSUBPROP_DESCRIPTION, "The Minimum value for the far lane width. If the calculated lane width is smaller than this value the blind count will be increased.");

    SetPropertyInt(LT_PROP_LANE_WIDTH_MAX_FAR, 290);
    SetPropertyBool(LT_PROP_LANE_WIDTH_MAX_FAR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_LANE_WIDTH_MAX_FAR NSSUBPROP_DESCRIPTION, "The Maximum value for the far lane width. If the calculated lane width is greater than this value the blind count will be increased.");

	SetPropertyInt(LT_PROP_ImagecutWidthLeftHough, 0);
	SetPropertyInt(LT_PROP_ImagecutWidthLeftHough NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_ImagecutWidthLeftHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(LT_PROP_ImagecutWidthRightHough, 640);
	SetPropertyInt(LT_PROP_ImagecutWidthRightHough NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_ImagecutWidthRightHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough... both Left and Right -> max Val 320");

	SetPropertyInt(LT_PROP_ImagecutHeightUpHough, 240);
	SetPropertyInt(LT_PROP_ImagecutHeightUpHough NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_ImagecutHeightUpHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(LT_PROP_ImagecutHeightDownHough, 440);
	SetPropertyInt(LT_PROP_ImagecutHeightDownHough NSSUBPROP_MIN, 0);
    SetPropertyStr(LT_PROP_ImagecutHeightDownHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

    SetPropertyFloat(LT_PROP_ThresholdValueCanny, 100);
    SetPropertyBool(LT_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(LT_PROP_ThresholdValueHough, 62);
    SetPropertyBool(LT_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");

    m_pISignalRegistry = NULL;
}

cLaneTracking::~cLaneTracking()
{
}

tResult cLaneTracking::GetInterface(const tChar* idInterface,
    tVoid** ppvObject)
{
    if (idmatch(idInterface, IID_ADTF_SIGNAL_PROVIDER))
    {
        *ppvObject = static_cast<ISignalProvider*> (this);
    }
    else
    {
        return cFilter::GetInterface(idInterface, ppvObject);
    }

    Ref();

    RETURN_NOERROR;
}

tUInt cLaneTracking::Ref()
{
    return cFilter::Ref();
}

tUInt cLaneTracking::Unref()
{
    return cFilter::Unref();
}

tVoid cLaneTracking::Destroy()
{
    delete this;
}

tResult cLaneTracking::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cLaneTracking::Stop(__exception)
{

#ifdef LT_ENABLE_CANNY_WINDOWS
    if(m_bShowDebug)
    {
        destroyWindow("Canny Near");
        destroyWindow("Canny Far");
        //destroyWindow("RGB Image");
        //destroyWindow("GaussianBlur");
        //destroyWindow("GreyScale Image");
        //destroyWindow("Binary Image");
        //destroyWindow("Canny Image");
        //destroyWindow("LaneTracking");  
    }  
#endif    

    return cFilter::Stop(__exception_ptr);
}
tResult cLaneTracking::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // Media Description Signal
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);        
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
        
        // Media Description Bool
        tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
        cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBool));

        // Media Description tInt32SignalValue
        tChar const * strDesctInt32SignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");
        RETURN_IF_POINTER_NULL(strDesctInt32SignalValue);
        cObjectPtr<IMediaType> pTypetInt32SignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDesctInt32SignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypetInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDesctInt32SignalValue));
        
        // Media Description LaneTracking Velocity
        tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
        RETURN_IF_POINTER_NULL(LTVelocityDescValue);
        cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION); 

		// Media Description Lane
		tChar const * LaneDesc = pDescManager->GetMediaDescription("tParkingLotStruct");
		RETURN_IF_POINTER_NULL(LaneDesc);
		cObjectPtr<IMediaType> pTypeLaneDesc = new cMediaType(0, 0, 0, "tParkingLotStruct", LaneDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);       
     

        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        // Start/Stop-Pin
        RETURN_IF_FAILED(m_oInputStart.Create("start", new cMediaType(0, 0, 0, "tBoolSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputStart));
        
        // Current Velocity Input
        RETURN_IF_FAILED(m_oCurrentVelocity.Create("CurrentVelocity", pTypeLTVelocityValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oCurrentVelocity));
        RETURN_IF_FAILED(pTypeLTVelocityValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCurrentVelocityInput));

        //Acceleration Output
        RETURN_IF_FAILED(m_oAccelerateOutput.Create("Acceleration", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oAccelerateOutput));

        //Steering Angle Output
        RETURN_IF_FAILED(m_oSteeringAngleOutput.Create("Steering_Angle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringAngleOutput));

        //Steering LaneType Output
        RETURN_IF_FAILED(m_oLaneTypeOutput.Create("LaneType", pTypetInt32SignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oLaneTypeOutput));

        //Steering Angle PT1 Output
        RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Create("Steering_Angle_PT1", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringAnglePT1Output));

		//LaneDetails Output
        RETURN_IF_FAILED(m_oLaneDetailsOutput.Create("LaneDetails", pTypeLaneDesc, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oLaneDetailsOutput));
		RETURN_IF_FAILED(pTypeLaneDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLane));

        //HeadLights Output
        RETURN_IF_FAILED(m_oHeadLightsOutput.Create("HeadLights", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oHeadLightsOutput));

		// Drive Straight Input
        RETURN_IF_FAILED(m_iDriveStraight.Create("Drive_Straight", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDriveStraight));
        RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolDriveStraightInput));


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

        //GLC Output
        cObjectPtr<IMediaType> pCmdType2 = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType2, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput2.Create("GLC_Output_alt",pCmdType2, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput2));


              
    }
    else if (eStage == StageNormal)
    {
		m_bLFStatusSet 		= tFalse;
		m_bDriveStraightSet = tFalse;

		m_DriveStraight 	= tFalse;
		

        m_bFirstFrame = true;
        m_ui8Imagecount = 0;
        m_bActive = tFalse;

		m_bCurrentVelocityInputSet = tFalse;
		m_bLaneOutputSet 			= tFalse;

        ReadProperties(NULL);

        if(m_bLightBeamTriggerEnabled == tFalse)
        {
            m_bActive = tTrue;
        }
        
        m_nCenterFromLeft = 0;
        m_nCenterFromRight = 0;
        m_i16ErrorSum = 0;
        m_i16ErrorOld = 0; 
        m_f32Ts = 0.033f;

        m_i16FarLaneCenter = 320;
        m_i16FarLaneWidth = 200;
        m_f32AccelerateOut = 0.0f;
        m_i16FarLaneWidthCalc = 0;

        m_ui8InitCtrl = 0;
        
        m_f32SteeringAngle = 0.0f;
        m_i16LaneWidth = 390;
        m_nBlindCounter = 0;
        m_nBlindCounterFar = 0;
        
        m_nCurrentNearLine = m_nNearLine;

		LaneType 	= UNKNOWN_LANE;
        
        if (m_bShowDebug)
        {
            // create a kernel mutex 
            THROW_IF_FAILED(m_oLock.Create(adtf_util::cString(OIGetInstanceName()) + ".active_signals"));

            // get the signal registry object
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
                IID_ADTF_SIGNAL_REGISTRY_EXTENDED,
                (tVoid**) &m_pISignalRegistry,
                __exception_ptr));

            // register the provider at the registry
            RETURN_IF_FAILED(m_pISignalRegistry->RegisterProvider(this, OIGetInstanceName(), __exception_ptr));

            RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(this,
                LT_SIGREG_ID_SCALED_PT1_INPUT,
                LT_SIGREG_NAME_SCALED_PT1_INPUT,
                LT_SIGREG_UNIT_SCALED_PT1_INPUT,
                "The variation of the 'lane center' and 'place to be' with the factor given by the PT1 input factor property.",
                -35,
                +35.0,
                __exception_ptr));

            RETURN_IF_FAILED(m_pISignalRegistry->RegisterSignal(this,
                LT_SIGREG_ID_CONTROLLER_INPUT,
                LT_SIGREG_NAME_CONTROLLER_INPUT,
                LT_SIGREG_UNIT_CONTROLLER_INPUT,
                "The variation of the 'lane center' and 'place to be' (simple difference)",
                -50.0,
                +50.0,
                __exception_ptr));
        }

    }
    RETURN_NOERROR;
}

tResult cLaneTracking::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cLaneTracking::ReadProperties(const tChar* strPropertyName)
{
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_NEAR_LINE))
    {
        m_nNearLine = GetPropertyInt(LT_PROP_NEAR_LINE);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_NEAR_LINE_MAX_OFFSET))
    {
        m_nNearLineMaxOffset = GetPropertyInt(LT_PROP_NEAR_LINE_MAX_OFFSET);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ImagecutWidthLeftHough))
    {
        m_nImagecutWidthLeftHough = GetPropertyInt(LT_PROP_ImagecutWidthLeftHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ImagecutWidthRightHough))
    {
        m_nImagecutWidthRightHough = GetPropertyInt(LT_PROP_ImagecutWidthRightHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ImagecutHeightUpHough))
    {
        m_nImagecutHeightUpHough = GetPropertyInt(LT_PROP_ImagecutHeightUpHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ImagecutHeightDownHough))
    {
        m_nImagecutHeightDownHough = GetPropertyInt(LT_PROP_ImagecutHeightDownHough);
	}

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_FAR_LINE))
    {
        m_nFarLine = GetPropertyInt(LT_PROP_FAR_LINE);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_MAX_ACCELERATION))
    {
        m_f32AccelerationMax = static_cast<tFloat32> (GetPropertyFloat(LT_PROP_MAX_ACCELERATION));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_MIN_ACCELERATION))
    {
        m_f32AccelerationMin = static_cast<tFloat32> (GetPropertyFloat(LT_PROP_MIN_ACCELERATION));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ACCELERATION_FAR_NEAR_DIFF))
    {
        m_nAccelerationFarNearDiff = GetPropertyInt(LT_PROP_ACCELERATION_FAR_NEAR_DIFF);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CAMERA_OFFSET))
    {
        m_f64CamOffset = GetPropertyFloat(LT_PROP_CAMERA_OFFSET);
        m_sLaneCenterNear.x = 320 + static_cast<tInt16> (m_f64CamOffset);
        m_sPlaceToBe.x = 320 + static_cast<tInt16> (m_f64CamOffset);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_PROPORTIONAL_GAIN))
    {
        m_f64Kp = GetPropertyFloat(LT_PROP_CONTR_PROPORTIONAL_GAIN);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_INTEGRAL_GAIN))
    {
        m_f64Ki = GetPropertyFloat(LT_PROP_CONTR_INTEGRAL_GAIN);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_CONTR_DIFFERENTIAL_GAIN))
    {
        m_f64Kd = GetPropertyFloat(LT_PROP_CONTR_DIFFERENTIAL_GAIN);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_TRESHOLD))
    {
        m_nThresholdValue = GetPropertyInt(LT_PROP_TRESHOLD);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MIN_NEAR))
    {
        m_i16LaneWidthMinNear = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MIN_NEAR));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MAX_NEAR))
    {
        m_i16LaneWidthMaxNear = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MAX_NEAR));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MIN_FAR))
    {
        m_i16LaneWidthMinFar = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MIN_FAR));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_LANE_WIDTH_MAX_FAR))
    {
        m_i16LaneWidthMaxFar = static_cast<tInt16> (GetPropertyInt(LT_PROP_LANE_WIDTH_MAX_FAR));
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_TAU))
    {
        m_f64PT1Tau = GetPropertyFloat(LT_PROP_PT1_TAU);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_SAMPLE_TIME))
    {
        m_f64PT1Sample = GetPropertyFloat(LT_PROP_PT1_SAMPLE_TIME);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_INPUT_FACTOR))
    {
        m_f64PT1InputFactor = GetPropertyFloat(LT_PROP_PT1_INPUT_FACTOR);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_PT1_TAU) || cString::IsEqual(strPropertyName, LT_PROP_PT1_SAMPLE_TIME))
    {
        m_f64PT1Gain = m_f64PT1Tau / m_f64PT1Sample;
        m_f32PT1LastSteeringOut = 0.0f;
        m_f32PT1SteeringOut = 0.0f;
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ENABLE_LIGHTBEAM_TRIGGER))
    {
        m_bLightBeamTriggerEnabled = GetPropertyBool(LT_PROP_ENABLE_LIGHTBEAM_TRIGGER);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(LT_PROP_SHOW_DEBUG);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_DRIVE_TIME))
    {
        m_nDriveTime = GetPropertyInt(LT_PROP_DRIVE_TIME);
    }
    
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_EMERGENCY_STOP_TIME))
    {
        m_nEmergencyStopTime = GetPropertyInt(LT_PROP_EMERGENCY_STOP_TIME);
    }

		if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ThresholdValueHough))
    {
        m_nThresholdValueHough = GetPropertyInt(LT_PROP_ThresholdValueHough);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(LT_PROP_ThresholdValueCanny);
    }


    RETURN_NOERROR;
}

tResult cLaneTracking::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{
    __synchronized_obj(m_oRunCritSection);
    if(IRunnable::RUN_TIMER == nActivationCode)
    {
        LOG_INFO(cString::Format("Run timer called with user data size: %d", szUserDataSize).GetPtr());
        if(szUserDataSize == sizeof(tUInt8))
        {
            tUInt8* nUserData = (tUInt8*) pvUserData;
            LOG_INFO(cString::Format("Run timer called with user data value: %u", *nUserData));
            if(*nUserData == g_ui8StopTimerIdNegative)
            {
                LOG_INFO("Set active to false and break (negative acceleration).");
                m_bActive = tFalse;

                // transmit negative acceleration for breaking
                TransmitAcceleration(0.0f, _clock->GetStreamTime());
                
                // destroy this timer
                _kernel->TimerDestroy(m_hStopTimerNegative);
                m_hStopTimerNegative = NULL;

                m_hStopTimerZero = _kernel->TimerCreate(-1, 500000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8StopTimerIdZero, sizeof(tUInt8), 0, cString::Format("%s.stoptimerZero", OIGetInstanceName()));
            }
            else if(*nUserData == g_ui8StopTimerIdZero)
            {
                LOG_INFO("Set active to false, set acceleration to zero and destroy timer.");
                m_bActive = tFalse;

                // transmit negative acceleration for breaking
                TransmitAcceleration(0.0f, _clock->GetStreamTime());

                // destroy this timer
                _kernel->TimerDestroy(m_hStopTimerZero);
                m_hStopTimerZero = NULL;
            }
            else if(*nUserData == g_ui8EmergencyStopTimerIdNegative)
            {
                LOG_INFO("Set active to false, performing an emergency break (acceleration negative) and destroy timer.");
                m_bActive = tFalse;

                // transmit negative acceleration for breaking
                // break hard for 1 second
                TransmitAcceleration(0.0f, _clock->GetStreamTime());
                
                // cleanup this timer
                _kernel->TimerDestroy(m_hEmergencyStopTimerNegative);
                m_hEmergencyStopTimerNegative = NULL;

                m_hEmergencyStopTimerZero = _kernel->TimerCreate(-1, 1000000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8EmergencyStopTimerIdZero, sizeof(tUInt8), 0, cString::Format("%s.emergencystoptimerZero", OIGetInstanceName()));
            }
            else if(*nUserData == g_ui8EmergencyStopTimerIdZero)
            {
                LOG_INFO("Set active to false, performing an emergency break, set active to true and destroy timer.");
                m_bActive = tFalse;

                // stay on that position for 3 seconds
                TransmitAcceleration(0.0f, _clock->GetStreamTime());

                // cleanup this timer
                _kernel->TimerDestroy(m_hEmergencyStopTimerZero);
                m_hEmergencyStopTimerZero = NULL;

                m_hEmergencyStopTimerResume = _kernel->TimerCreate(-1, 3000000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8EmergencyStopTimerIdResume, sizeof(tUInt8), 0, cString::Format("%s.emergencystoptimerResume", OIGetInstanceName()));
            }

            else if(*nUserData == g_ui8EmergencyStopTimerIdResume)
            {
                LOG_INFO("Set active to true after performing an emergency break and destroy timer.");
                
                // reenable the longitudinal controller
                m_bActive = tTrue;

                // cleanup this timer
                _kernel->TimerDestroy(m_hEmergencyStopTimerResume);
                m_hEmergencyStopTimerResume = NULL;

            }
        }
    }

    RETURN_NOERROR;
}

tResult cLaneTracking::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if(m_bShowDebug && eStage == cFilter::StageNormal)
    {        
        ucom::cObjectPtr<ISignalRegistry> pSignalRegistry;
        if (IS_OK(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
            IID_ADTF_SIGNAL_REGISTRY,
            (tVoid**)&pSignalRegistry)))
        {
            // Unregister the provider
            pSignalRegistry->UnregisterSignalProvider(this);
        }
        m_oActive.clear();

        m_oLock.Release();
    }

    if(m_hStopTimerZero != NULL)
    {
        _kernel->TimerDestroy(m_hStopTimerZero);
        m_hStopTimerZero = NULL;
    }
    if(m_hStopTimerNegative != NULL)
    {
        _kernel->TimerDestroy(m_hStopTimerNegative);
        m_hStopTimerNegative = NULL;
    }
    
    if(m_hEmergencyStopTimerNegative != NULL)
    {
        _kernel->TimerDestroy(m_hEmergencyStopTimerNegative);
        m_hEmergencyStopTimerNegative = NULL;
    }

    if(m_hEmergencyStopTimerZero != NULL)
    {
        _kernel->TimerDestroy(m_hEmergencyStopTimerZero);
        m_hEmergencyStopTimerZero = NULL;
    }

    if(m_hEmergencyStopTimerResume != NULL)
    {
        _kernel->TimerDestroy(m_hEmergencyStopTimerResume);
        m_hEmergencyStopTimerResume = NULL;
    }
    
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cLaneTracking::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
    	
        tTimeStamp InputTimeStamp;
        InputTimeStamp = pMediaSample->GetTime();

    	// Change of Velocity property
    	if (pSource == &m_oCurrentVelocity){
    		ChangeCurrentVelocity(pMediaSample);
    	}


        if(pSource == &m_oVideoInputPin)
        {
            //Videoformat
            if (m_bFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrame = false;
            }

            ProcessInput(pMediaSample, InputTimeStamp);
        }
		
		else if(pSource == &m_iDriveStraight){
			DriveStraight(pMediaSample);
		}
         
        else if(pSource == &m_oInputStart)
        {
            tBool bValue = tFalse;
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pCoderDescBool,pMediaSample,pCoder);

				if (!m_bLFStatusSet){

					pCoder->GetID("bValue", m_szIDBoolValueOutput);
					m_bLFStatusSet 	= tTrue;
				}

                pCoder->Get(m_szIDBoolValueOutput, (tVoid*)&bValue);            
            }
			LOG_INFO(cString::Format("LF: Status Received = %i", bValue) );
            if (bValue)
            {
                LOG_INFO("Receiving positive trigger.");
                if(m_hStopTimerNegative == NULL && m_hStopTimerZero == NULL && m_nDriveTime > 0)
                {
					ReadProperties(NULL);
                    m_bActive = tTrue;
					m_i16ErrorSum = 0.0; 	// Reset ErrorSum of PID controller
                    m_hStopTimerNegative = _kernel->TimerCreate(-1, m_nDriveTime * 1000, static_cast<IRunnable*>(this), NULL, (tVoid*) &g_ui8StopTimerIdNegative, sizeof(tUInt8), 0, cString::Format("%s.stoptimerNegative", OIGetInstanceName()));
                    LOG_INFO(cString::Format("Starting stopping timer with: %d us and %u", m_nDriveTime * 1000, g_ui8StopTimerIdNegative).GetPtr());
                }
                else if(m_hStopTimerNegative != NULL || m_hStopTimerZero != NULL)
                {
                    LOG_WARNING("There is already a stopping timer running. Will ignore this trigger.");
                }
                else if (m_nDriveTime <= 0)
                {
					ReadProperties(NULL);
                    m_bActive = tTrue;
                    LOG_INFO("There is no Drive time set. The car will not stop unless stopping ADTF.");
                }

                if(m_hEmergencyStopTimerNegative == NULL && m_hEmergencyStopTimerZero == NULL && m_hEmergencyStopTimerResume == NULL && m_nEmergencyStopTime > 0)
                {
                    m_hEmergencyStopTimerNegative = _kernel->TimerCreate(-1, m_nEmergencyStopTime * 1000, static_cast<IRunnable*>(this), NULL,(tVoid*) &g_ui8EmergencyStopTimerIdNegative, sizeof(tUInt8), 0, cString::Format("%s.emergencytimerNegative", OIGetInstanceName()));
                    LOG_INFO(cString::Format("Starting emergency timer with: %d us and %u", m_nEmergencyStopTime * 1000, g_ui8EmergencyStopTimerIdNegative).GetPtr());
                }
                else if(m_hEmergencyStopTimerNegative != NULL || m_hEmergencyStopTimerZero != NULL || m_hEmergencyStopTimerResume != NULL)
                {
                    LOG_WARNING("There is already a braking timer running. Will ignore this trigger.");
                }
                else if (m_nEmergencyStopTime <= 0)
                {
                    LOG_INFO("There is no emergency stop time set. This action will not be performed.");
                }
            }
            else
            {
                LOG_INFO("Receiving negative trigger. Will stop the car.");
                m_bActive = tFalse;
            }
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cLaneTracking::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
    if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
        TransmitAcceleration(0.0f, tsInputTime);
        TransmitSteeringAngle(0.0f, tsInputTime);
        TransmitSteeringAnglePT1(0.0f, tsInputTime);
        TransmitHeadLights(tTrue, tsInputTime);    // Switch on Headlights
        m_ui8InitCtrl++;
    }
    else
    {
        // VideoInput
        RETURN_IF_POINTER_NULL(pSample);

        const tVoid* l_pSrcBuffer;
    
        /*IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
    
        cvSetData(img, (tVoid*)l_pSrcBuffer, img->widthStep);
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
        pSample->Unlock(l_pSrcBuffer);*/
    
        IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
        oImg->imageData = (char*)l_pSrcBuffer;
        Mat image(cvarrToMat(oImg));
        cvReleaseImage(&oImg);
        pSample->Unlock(l_pSrcBuffer);
      
      
        // Transform nearfield (Nearfield is used for lateral control)
        Mat matImageCutNear= image(cv::Range(m_nCurrentNearLine - 20, m_nCurrentNearLine + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image
		//Mat matImageCutNear= image(cv::Range(350 - 20, 350 + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image        
        GaussianBlur(matImageCutNear, matImageCutNear, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
        cvtColor(matImageCutNear, m_matGreyNear ,CV_RGB2GRAY);// Grey Image
        threshold(m_matGreyNear, m_matGreyThreshNear, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
        Canny(m_matGreyThreshNear, m_matLineCannyNear, 0, 2, 3, false);// Detect Edges
    
        // search points within the canny near
        Search(m_asAllpointsNear, &m_ui8NearPointsCount, 40, m_matLineCannyNear, NEAR_FIELD );
        
    
        // Transform farfield (farfield is used for longitudinal control)
        Mat matImageCutFar= image(cv::Range(m_nFarLine - 20, m_nFarLine + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image
		//Mat matImageCutFar= image(cv::Range(250 - 20, 250 + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image                
        GaussianBlur(matImageCutFar, matImageCutFar, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
        cvtColor(matImageCutFar, m_matGreyFar ,CV_RGB2GRAY);// Grey Image
        threshold(m_matGreyFar, m_matGreyThreshFar, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
        Canny(m_matGreyThreshFar, m_matLineCannyFar, 0, 2, 3, false);// Detect Edges
        
        // search points within the canny far
        Search(m_asAllpointsFar, &m_ui8FarPointsCount, 40, m_matLineCannyFar, FAR_FIELD);

		// OptiCar preproecessing
		Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_nImagecutWidthLeftHough, m_nImagecutWidthRightHough)).clone(); //Cut Image 
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 		
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
/*		fstream f2;
		f2.open("norm.dat",ios::out);
		f2 << m_matCutHough << "\n";
		f2.close();*/
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO); // delete dark noise
/*		fstream f3;
		f3.open("thres.dat",ios::out);
		f3 << m_matCutHough << "\n";
		f3.close();	*/
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
/*		fstream f4;
		f4.open("median.dat",ios::out);
		f4 << m_matCutHough << "\n";
		f4.close(); */
        Canny(m_matCutHough, m_matCannyHough,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges ---------Nur Canny "funktioniert"-------
/*		fstream f5;
		f5.open("canny.dat",ios::out);
		f5 << m_matCannyHough << "\n";
		f5.close(); */

		//Alles auf initial setzen
		m_lines.clear();
		m_HoughGroup1.clear();
		m_HoughGroup2.clear();
		m_HoughGroup3.clear();
		m_HoughpointsNear[0].x = 0;
		m_HoughpointsNear[1].x = 0;
		m_HoughpointsNear[2].x = 0;
		m_HoughpointsNear[0].y = 0;
		m_HoughpointsNear[1].y = 0;
		m_HoughpointsNear[2].y = 0;
		m_HoughpointsFar[0].x = 0;
		m_HoughpointsFar[1].x = 0;
		m_HoughpointsFar[2].x = 0;
		m_HoughpointsFar[0].y = 0;
		m_HoughpointsFar[1].y = 0;
		m_HoughpointsFar[2].y = 0;
		m_nearRightBuffer.clear();m_farRightBuffer.clear();
		m_nearMidBuffer.clear();m_farMidBuffer.clear();
		m_group3Buffer.clear();

		HoughLines(m_matCannyHough,m_lines,1,CV_PI/180,m_nThresholdValueHough,0,0);

/*		fstream f2;
		f2.open("Hough.dat",ios::out|ios::app);
	for(size_t i=0; i<m_lines.size();i++)
		{
					f2 << m_lines[i][0] << "," << m_lines[i][1]*180/M_PI << "\n";
		}
		f2.close();
*/

#ifdef LT_ENABLE_CANNY_WINDOWS
        //**************** Show Image *************************
        if(m_bShowDebug)
        {
            if(m_ui8Imagecount > 2)
            {
                m_ui8Imagecount=0;
                //imshow("RGB Image", image);
                //imshow("GaussianBlur", imagegauss);
                //imshow("GreyScale Image", greyNear);
                //imshow("Binary Image", greythreshNear);
                //imshow("Canny Image", linecanny);*/
                imshow("Canny Near", m_matLineCannyNear);
                imshow("Canny Far", m_matLineCannyFar);
                waitKey(1);
            }
            m_ui8Imagecount++;
        }
        //********************************************************
#endif

        //********** Find horizontal line (for calibration only)**************
        //tInt edgerow = 0;
        //for(tInt row=cannysize.height;row>=1;row--)                                                                                                                                                                                  
        //{            
        //    if(linecanny.at<uchar>(row,cannysize.width/2)!=0) 
        //    {    
        //        edgerow = row;
        //        row = 0;
        //    }
        //}
        //LOG_INFO(cString::Format("Found edge at row: %i", edgerow));
        //********************************************************************    

        //LOG_INFO(cString::Format("lfdnr1: %i",lfdnr1));


		//Decide which kind of lane we have:
		if(!m_lines.empty())
		{
			vector<Point2f> GroupLeft;
			vector<Point2f> GroupRight;
			float average = 0.0;
				for(size_t i=0; i<m_lines.size();i++)
				{					
					if((m_lines[i][1]*180/M_PI<70)) //Eigentlich sollte die grenze bei 90°(waagerecht) sein aber die  erfahrung hat gezeigt, dass 95 besser funktioniert,weil manchmal ein paar lines über 90° hinaus kommen trotz rechtskurve.
					{
						GroupLeft.push_back(Point2f(m_lines[i][0],m_lines[i][1]));	
						average = average + m_lines[i][1]*180/M_PI;	
					}
					else if((m_lines[i][1]*180/M_PI>110))
					{
						GroupRight.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	
						average = average + m_lines[i][1]*180/M_PI;
					}
				}
				average = average/(float)(GroupLeft.size()+GroupRight.size()); 

				// In the if clause: Look on the average group size to identify if we are in LANE_Straight mode
			if(GetAverage( GroupLeft.size(), m_LaneTypeBufferLeft)>=1 && GetAverage( GroupRight.size(), m_LaneTypeBufferRight)>1) 
			{
				//andert sich der LaneType, muessen einige Buffer auf 0 gesetzt werden
				if(LaneType!=LANE_STRAIGHT)
				{
					m_nearRightBuffer.clear();m_farRightBuffer.clear();
					m_nearMidBuffer.clear();m_farMidBuffer.clear();
					m_group3Buffer.clear();
				}
				LaneType = LANE_STRAIGHT;
			}
			// otherwise use the average angle to identify which direction the curve has
			else
			{
				if(average<90) 
				{
					//andert sich der LaneType, muessen einige Buffer auf 0 gesetzt werden
					if(LaneType!=LANE_RIGHT)
					{
						m_nearRightBuffer.clear();m_farRightBuffer.clear();
						m_nearMidBuffer.clear();m_farMidBuffer.clear();
						m_group3Buffer.clear();
					}
					LaneType = LANE_RIGHT;
				}
				if(average>90) 
				{
					//andert sich der LaneType, muessen einige Buffer auf 0 gesetzt werden
					if(LaneType!=LANE_LEFT)
					{
						m_nearRightBuffer.clear();m_farRightBuffer.clear();
						m_nearMidBuffer.clear();m_farMidBuffer.clear();
						m_group3Buffer.clear();
					}
					LaneType = LANE_LEFT;
				}			
			}		
		}
		TransmitLaneType(LaneType, tsInputTime);

//		LOG_INFO(cString::Format("LaneType: %i",LaneType)); // OutputLanetype

		SearchHough(m_lines);

		if(LaneType == LANE_STRAIGHT)//if we drive straight we use our lanetracking
		{
			if(m_HoughGroup1.empty() && m_HoughGroup2.empty() && m_HoughGroup3.empty())//...but only if we found lines
			{
        		LateralControl(m_asAllpointsNear, &m_ui8NearPointsCount, tsInputTime);
	    		LongitudinalControl(m_asAllpointsFar, &m_ui8FarPointsCount, tsInputTime);
			}
			else//...if we didnt find lines use aadclanetracking
			{
				LateralControl(m_HoughpointsNear, &m_HoughLineSizeCounter, tsInputTime);
	    		LongitudinalControl(m_HoughpointsFar, &m_HoughLineSizeCounter, tsInputTime);
			}
    	}
		else if(LaneType == LANE_LEFT || LaneType == LANE_RIGHT)//in curves we use aadclanetracking
		{
			if(&m_ui8NearPointsCount != 0)//..but only if we found points
			{
				LateralControl(m_asAllpointsNear, &m_ui8NearPointsCount, tsInputTime);
	    		LongitudinalControl(m_asAllpointsFar, &m_ui8FarPointsCount, tsInputTime);
			}
			else//otherwise we use our lanetracking and hope we are in a good position
			{
				LateralControl(m_HoughpointsNear, &m_HoughLineSizeCounter, tsInputTime);
				/*if(LaneType == LANE_LEFT)
				{
					LateralControl(m_HoughpointsNear, &m_HoughLineSizeCounter, tsInputTime);
				}
				else if(LaneType == LANE_RIGHT)
				{
					LateralControl(m_HoughpointsNear, &m_HoughLineSizeCounter, tsInputTime);
				}*/
			}
		}
	}
    RETURN_NOERROR;            
}

tResult cLaneTracking::ProcessFound()
{        

    RETURN_NOERROR;
}

tResult cLaneTracking::ProcessOutput()
{

    RETURN_NOERROR;
}

tResult cLaneTracking::SearchHough(vector<Vec2f> m_lines)
{	
	m_HoughLineSizeCounter = 0;
	for(size_t i=0; i<m_lines.size();i++)
		{
			if((m_lines[i][1]*180/M_PI<60) && (m_lines[i][1]*180/M_PI>35))
				{
					// HoughGroup1(mid line)
					m_HoughGroup1.push_back(Point2f(m_lines[i][0],m_lines[i][1]));		
				}
				else if((m_lines[i][1]*180/M_PI<145) && (m_lines[i][1]*180/M_PI>120))
				{
					// HoughGroup2(right line)
					m_HoughGroup2.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	
				}
				else if((m_lines[i][1]*180/M_PI<100) && (m_lines[i][1]*180/M_PI>65))
				{
					// HoughGroup3(left line)
					if(LaneType == LANE_RIGHT)//bei einer rechtskurve wollen wir nur die nahen lines, also die lines mit großem radius
					{
						if((m_lines[i][0]>150)) m_HoughGroup3.push_back(Point2f(m_lines[i][0],m_lines[i][1]));
					}
				}
		}
/*
//Calculate representer for HoughGroup1(mid line)
if(!m_HoughGroup1.empty())
{
float averageHG1 = 0;
int max1 = 0;
m_representerHG1 = m_HoughGroup1[0]; 
	for(size_t i=0; i<m_HoughGroup1.size();i++)
		{
			averageHG1 = averageHG1 + m_HoughGroup1[i].y*180/M_PI;
		}
			averageHG1 = averageHG1/m_HoughGroup1.size();
	/*for(size_t i=0; i<m_HoughGroup1.size();i++)
		{
			if(abs(m_representerHG1.y*180/M_PI-averageHG1)>abs(m_HoughGroup1[i].y*180/M_PI-averageHG1))
			{
				m_representerHG1 = m_HoughGroup1[i];
			}
		}*/
/*	for(size_t i=0; i<m_HoughGroup1.size();i++)
	{
		if(m_HoughGroup1[i].x>max1)
		{ 
			max1 = m_HoughGroup1[i].x;
			m_representerHG1 = m_HoughGroup1[i];
		}
	}
}
//Calculate representer for HoughGroup2(right line)
if(!m_HoughGroup2.empty())
{
float averageHG2 = 0;
int max2 = 0;
m_representerHG2 = m_HoughGroup2[0]; 
	for(size_t i=0; i<m_HoughGroup2.size();i++)
		{
			averageHG2 = averageHG2 + m_HoughGroup2[i].y*180/M_PI;
		}
			averageHG2 = averageHG2/m_HoughGroup2.size();
	/*for(size_t i=0; i<m_HoughGroup2.size();i++)
		{
			if(abs(m_representerHG2.y*180/M_PI-averageHG2)>abs(m_HoughGroup2[i].y*180/M_PI-averageHG2))
			{
				m_representerHG2 = m_HoughGroup2[i];
			}
		}*/
/*	for(size_t i=0; i<m_HoughGroup2.size();i++)
	{
		if(m_HoughGroup2[i].x>max2)
		{ 
			max2 = m_HoughGroup2[i].x;
			m_representerHG2 = m_HoughGroup2[i];
		}
	}
}
//Calculate representer for HoughGroup3(left line)
if(!m_HoughGroup3.empty())
{
float averageHG3 = 0;
m_representerHG3 = m_HoughGroup3[0]; 
	for(size_t i=0; i<m_HoughGroup3.size();i++)
		{
			averageHG3 = averageHG3 + m_HoughGroup3[i].y*180/M_PI;
		}
			averageHG3 = averageHG3/m_HoughGroup3.size();
	for(size_t i=0; i<m_HoughGroup3.size();i++)
		{
			if(abs(m_representerHG3.y*180/M_PI-averageHG3)>abs(m_HoughGroup3[i].y*180/M_PI-averageHG3))
			{
				m_representerHG3 = m_HoughGroup3[i];
			}
		}
}*/
//create linear function with houghlines
	if(!m_HoughGroup1.empty() && LaneType == LANE_STRAIGHT)
		{	
			tInt16 x1,x2;
			x1=0;
			x2=0;
			
			for(size_t i=0; i<m_HoughGroup1.size();i++)
			{
				Point2f pt1m,pt2m;
				pt1m.x = cos(m_HoughGroup1[i].y)*m_HoughGroup1[i].x + 1000*(-sin(m_HoughGroup1[i].y));
				pt1m.y = sin(m_HoughGroup1[i].y)*m_HoughGroup1[i].x + 1000*(cos(m_HoughGroup1[i].y));
				pt2m.x = cos(m_HoughGroup1[i].y)*m_HoughGroup1[i].x - 1000*(-sin(m_HoughGroup1[i].y));
				pt2m.y = sin(m_HoughGroup1[i].y)*m_HoughGroup1[i].x - 1000*(cos(m_HoughGroup1[i].y));

				if(!(pt1m.x == pt2m.x) || !(pt1m.y == pt2m.y) )				
				{
					float m = (pt2m.y-pt1m.y)/(pt2m.x-pt1m.x);
					double b = pt2m.y - m*pt2m.x;
				
					tInt16 temp1=(m_nNearLine-m_nImagecutHeightUpHough-b)/m;
					tInt16 temp2=(m_nFarLine-m_nImagecutHeightUpHough-b)/m;

					if(temp1>x1)
					{
						x1=temp1;
					}
					if(temp2>x2)
					{
						x2=temp2;
					}
				}


			}
			m_HoughpointsNear[0].y = m_nNearLine;
			m_HoughpointsNear[0].x = GetAverage(x1, m_nearMidBuffer);
			m_HoughpointsFar[0].y = m_nFarLine; 
			m_HoughpointsFar[0].x = GetAverage(x2, m_farMidBuffer);
			m_HoughLineSizeCounter++;
/*
				Point2f pt1m,pt2m;
				pt1m.x = cos(m_representerHG1.y)*m_representerHG1.x + 1000*(-sin(m_representerHG1.y));
				pt1m.y = sin(m_representerHG1.y)*m_representerHG1.x + 1000*(cos(m_representerHG1.y));
				pt2m.x = cos(m_representerHG1.y)*m_representerHG1.x - 1000*(-sin(m_representerHG1.y));
				pt2m.y = sin(m_representerHG1.y)*m_representerHG1.x - 1000*(cos(m_representerHG1.y));

				if(pt1m.x == pt2m.x || pt1m.y == pt2m.y )				
				{
				}
				else
				{
				float m1 = (pt2m.y-pt1m.y)/(pt2m.x-pt1m.x);
				double b1 = pt2m.y - m1*pt2m.x;
				m_HoughpointsNear[0].y = m_nNearLine;
				m_HoughpointsNear[0].x = (m_nNearLine-m_nImagecutHeightUpHough-b1)/m1;
				m_HoughpointsFar[0].y = m_nFarLine; 
				m_HoughpointsFar[0].x = (m_nFarLine-m_nImagecutHeightUpHough-b1)/m1;
				m_HoughLineSizeCounter++;
*/
		}
	if(!m_HoughGroup2.empty() && !(LaneType == LANE_RIGHT))
		{
			tInt16 x1,x2;
			x1=640;
			x2=640;
			
			for(size_t i=0; i<m_HoughGroup2.size();i++)
			{
				Point2f pt1m,pt2m;
				pt1m.x = cos(m_HoughGroup2[i].y)*m_HoughGroup2[i].x + 1000*(-sin(m_HoughGroup2[i].y));
				pt1m.y = sin(m_HoughGroup2[i].y)*m_HoughGroup2[i].x + 1000*(cos(m_HoughGroup2[i].y));
				pt2m.x = cos(m_HoughGroup2[i].y)*m_HoughGroup2[i].x - 1000*(-sin(m_HoughGroup2[i].y));
				pt2m.y = sin(m_HoughGroup2[i].y)*m_HoughGroup2[i].x - 1000*(cos(m_HoughGroup2[i].y));

				if(!(pt1m.x == pt2m.x) || !(pt1m.y == pt2m.y) )				

				{
					float m = (pt2m.y-pt1m.y)/(pt2m.x-pt1m.x);
					double b = pt2m.y - m*pt2m.x;
				
					tInt16 temp1=(m_nNearLine-m_nImagecutHeightUpHough-b)/m;
					tInt16 temp2=(m_nFarLine-m_nImagecutHeightUpHough-b)/m;

					if(temp1<x1)
					{
						x1=temp1;
					}
					if(temp2<x2)
					{
						x2=temp2;
					}
				}


			}
				
			if(LaneType == LANE_LEFT)
			{
				m_HoughpointsNear[0].y = m_nNearLine;//Bei 350 Pixeln berechnet deren lanetracking die spurenbreite
				m_HoughpointsNear[0].x = GetAverage(x1, m_nearRightBuffer);
				}
			else if (LaneType == LANE_STRAIGHT)
			{
				if(!m_HoughGroup1.empty())
				{
					m_HoughpointsNear[1].y = m_nNearLine;//Bei 350 Pixeln berechnet deren lanetracking die spurenbreite
					m_HoughpointsNear[1].x = GetAverage(x1, m_nearRightBuffer);//Hier findet die Berechnung nur bei m_nNearLine-m_nImagecutHeightUpHough weil wir das bild bei m_nImagecutHeightUpHough cutten
					m_HoughpointsFar[1].y = m_nFarLine; 
					m_HoughpointsFar[1].x = GetAverage(x2, m_farRightBuffer);
					m_HoughLineSizeCounter++;
					vector<tFloat32> LaneDetails;
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsNear[1].x));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsNear[1].y));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsFar[1].x));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsFar[1].y));
					TransmitLaneDetails(LaneDetails);
				}
				else
				{
					m_HoughpointsNear[0].y = m_nNearLine;//Bei 350 Pixeln berechnet deren lanetracking die spurenbreite
					m_HoughpointsNear[0].x = GetAverage(x1, m_nearRightBuffer);
					m_HoughpointsFar[0].y = m_nFarLine; 
					m_HoughpointsFar[0].x = GetAverage(x2, m_farRightBuffer);
					m_HoughLineSizeCounter++;
					vector<tFloat32> LaneDetails;
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsNear[0].x));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsNear[0].y));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsFar[0].x));
					LaneDetails.push_back(static_cast<tFloat32>(m_HoughpointsFar[0].y));
					TransmitLaneDetails(LaneDetails);
				}
			}
		}
	if(m_HoughGroup3.size()!=0 && LaneType == LANE_RIGHT)
		{
			tInt16 x1;
			x1=0;
			
			for(size_t i=0; i<m_HoughGroup3.size();i++)
			{
				Point2f pt1m,pt2m;
				pt1m.x = cos(m_HoughGroup3[i].y)*m_HoughGroup3[i].x + 1000*(-sin(m_HoughGroup3[i].y));
				pt1m.y = sin(m_HoughGroup3[i].y)*m_HoughGroup3[i].x + 1000*(cos(m_HoughGroup3[i].y));
				pt2m.x = cos(m_HoughGroup3[i].y)*m_HoughGroup3[i].x - 1000*(-sin(m_HoughGroup3[i].y));
				pt2m.y = sin(m_HoughGroup3[i].y)*m_HoughGroup3[i].x - 1000*(cos(m_HoughGroup3[i].y));

				if(!(pt1m.x == pt2m.x) || !(pt1m.y == pt2m.y) )				
				{
					float m = (pt2m.y-pt1m.y)/(pt2m.x-pt1m.x);
					double b = pt2m.y - m*pt2m.x;
				
					tInt16 temp1=(m_nNearLine-m_nImagecutHeightUpHough-b)/m;

					if(temp1>x1)
					{
						x1=temp1;
					}
				}

				m_HoughpointsNear[0].y = m_nNearLine;//Bei 350 Pixeln berechnet deren lanetracking die spurenbreite
				m_HoughpointsNear[0].x = GetAverage(x1,m_group3Buffer); 
				m_HoughLineSizeCounter++;
			}
/*
				Point2f pt1l,pt2l;
				pt1l.x = cos(m_representerHG3.y)*m_representerHG3.x + 1000*(-sin(m_representerHG3.y));
				pt1l.y = sin(m_representerHG3.y)*m_representerHG3.x + 1000*(cos(m_representerHG3.y));
				pt2l.x = cos(m_representerHG3.y)*m_representerHG3.x - 1000*(-sin(m_representerHG3.y));
				pt2l.y = sin(m_representerHG3.y)*m_representerHG3.x - 1000*(cos(m_representerHG3.y));

				if(pt1l.x == pt2l.x || pt1l.y == pt2l.y )				
				{
				}
				else
				{
				float m3 = (pt2l.y-pt1l.y)/(pt2l.x-pt1l.x);
				double b3 = pt2l.y - m3*pt2l.x;
					if(LaneType == LANE_RIGHT)
					{
						m_HoughpointsNear[0].y = m_nNearLine;//Bei 350 Pixeln berechnet deren lanetracking die spurenbreite
						m_HoughpointsNear[0].x = (m_nNearLine-m_nImagecutHeightUpHough-b3)/m3; 
						//m_HoughpointsFar[0].y = 250; 
						//m_HoughpointsFar[0].x = (-50-b3)/m3;
						m_HoughLineSizeCounter++;
						//LOG_INFO(cString::Format("x=: %d",m_HoughpointsNear[0].x));
					}
				}

		*/		
		}
//if(m_HoughpointsNear && m_HoughpointsFar)
//{
fstream ff;
ff.open("hallo.dat",ios::out);
ff << m_HoughpointsNear[0].x << "," << m_HoughpointsNear[0].y << "\n" << m_HoughpointsNear[1].x << "," << m_HoughpointsNear[1].y << "\n" << m_HoughpointsFar[0].x << "," << m_HoughpointsFar[0].y << "\n" << m_HoughpointsFar[1].x << "," << m_HoughpointsFar[1].y << LaneType << "\n\n" ;
ff.close();
//}
 CreateAndTransmitGCL();
 CreateAndTransmitGCL2();
	
	RETURN_NOERROR;
}

tInt16 cLaneTracking::GetAverage(tInt16 x, std::deque<tInt16> &Buffer)
{
	Buffer.push_back(x);
	size_t sBuffer = Buffer.size();
	if(sBuffer>5)
	{
		Buffer.pop_front();
		sBuffer--;
	}

	tInt16 xav=0;
	for(size_t i=0; i<sBuffer;i++)
	{
		xav+=Buffer.at(i);
	}
	xav=floor(xav/sBuffer);
	return xav;
}

tResult cLaneTracking::Search(sPoint *psPoints, tUInt8 *pui8PointsCount, tUInt8 ui8Limit, cv::Mat matCannyImg, int mode)
{
    A_UTIL_ASSERT(NULL != psPoints);
    A_UTIL_ASSERT(NULL != pui8PointsCount);

    *pui8PointsCount    = 0;
    tInt nColumnLast    = -1;
    Size szCannySize      = matCannyImg.size();

    for(tInt nColumn=0; nColumn < szCannySize.width; nColumn++)
    {                    
        if(matCannyImg.at<uchar>(szCannySize.height/2, nColumn)!=0) 
        {    
            if(abs(nColumnLast-nColumn)>=5 && abs(nColumnLast-nColumn) < ui8Limit && nColumnLast!=-1)
            {
                psPoints[*pui8PointsCount].x = static_cast<tInt>(nColumn-(abs(nColumnLast-nColumn)/2));

				if (mode == NEAR_FIELD) psPoints[*pui8PointsCount].x += WIDTH_MIN_NEAR;
				else if (mode == FAR_FIELD) psPoints[*pui8PointsCount].x += WIDTH_MIN_FAR;

                (*pui8PointsCount)++;            
                nColumnLast=-1;
            }
            else
            {
                nColumnLast=nColumn;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cLaneTracking::LateralControl(sPoint *psPoints, tUInt8 *pui8PointsCount, tTimeStamp tsInputTime)
{

    if(*pui8PointsCount == 2)
    {
        tInt16 i16LaneWidthCalc = psPoints[1].x - psPoints[0].x;
        //LOG_INFO(cString::Format("Point0: %i, Point1: %i, LaneWidth calc: %i", points[0].x, points[1].x, LaneWidth_calc));
        tInt nLaneCenterCalc = psPoints[0].x + (static_cast<tInt>(i16LaneWidthCalc/2));

        if ((i16LaneWidthCalc > m_i16LaneWidthMinNear) && (i16LaneWidthCalc < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
            m_i16LaneWidth = i16LaneWidthCalc;
            m_sLaneCenterNear.x = nLaneCenterCalc;
            m_nBlindCounter = 0;
        }
        else
        {
            m_nBlindCounter++;
        }        
    }
    else if(*pui8PointsCount == 3)
    {
        // If three points are found the correct lane is chosen according to the calculated lanewidth
        tInt16 i16Width1 = psPoints[2].x - psPoints[1].x;
        tInt16 i16Width2 = psPoints[1].x - psPoints[0].x;
        tInt nLaneCenterCalc1 = psPoints[1].x + (static_cast<tInt>(i16Width1/2));
        tInt nLaneCenterCalc2 = psPoints[0].x + (static_cast<tInt>(i16Width2/2));
        
        if((i16Width1 > m_i16LaneWidthMinNear) && (i16Width1 < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc1 - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
            m_i16LaneWidth = i16Width1;
            m_sLaneCenterNear.x = nLaneCenterCalc1;
            m_nBlindCounter = 0;
        }
        else if((i16Width2 > m_i16LaneWidthMinNear) && (i16Width2 < m_i16LaneWidthMaxNear) && abs(nLaneCenterCalc2 - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
            m_i16LaneWidth = i16Width2;
            m_sLaneCenterNear.x = nLaneCenterCalc2;
            m_nBlindCounter = 0;
        }
        else
        {
            m_nBlindCounter++;
        }        
    }
    else if(*pui8PointsCount == 1)
    {
        // If just one point is found the lane center shall be calculated based on this single point and the half of the previously calculated lane width
        // If the right point of the lane was found LaneWidth/2 must be subtracted from the position of the point, if the left point was found LaneWidth/2
        // must be added. Whether to add or to subtract LaneWidth/2 is determined by comparing the calculated new LaneCenter with the old LaneCenter.

        m_nCenterFromLeft = psPoints[0].x + (static_cast<tInt>(m_i16LaneWidth/2));

        m_nCenterFromRight = psPoints[0].x - (static_cast<tInt>(m_i16LaneWidth/2));

        if(abs(m_nCenterFromLeft - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
            m_sLaneCenterNear.x = m_nCenterFromLeft;
            m_nBlindCounter = 0;
        }
        else if(abs(m_nCenterFromRight - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
            m_sLaneCenterNear.x = m_nCenterFromRight;
            m_nBlindCounter = 0;
        }
        else
        {
            m_nBlindCounter++;
        }                
    }
    else
    {    
        // If more than three or no point (=lane) is detected, continue with the previously calculated values
        // Due to safety reasons a counter is implemented, that will cause the car to stop, if the car did not find any lane for more than X times consecutively (see parameter in logitudinal control)
    
        m_nBlindCounter++;
    }

    //if (m_sLaneCenterNear.x < 0 || m_sLaneCenterNear.x > m_sInputFormat.nWidth)
    //{
    //    m_sLaneCenterNear.x = static_cast<tInt16> (m_sInputFormat.nWidth / 2);
    //}

    
    //PID - Controller for Steering Angle
    m_i16Error = m_sLaneCenterNear.x - m_sPlaceToBe.x;
    m_i16ErrorSum = m_i16ErrorSum + m_i16Error;    
    m_f32SteeringAngle = static_cast<tFloat32>(m_f64Kp*m_i16Error + m_f64Ki*m_f32Ts*m_i16ErrorSum + (m_f64Kd*(m_i16Error-m_i16ErrorOld))/m_f32Ts);
    m_i16ErrorOld = m_i16Error;

//  LOG_INFO(cString::Format("Kp: %f, Ki: %f, Kd: %f", m_Kp, m_Ki, m_Kd).GetPtr());
    
    // Limitation of Steering Angle
    if (m_f32SteeringAngle > 30)
    {
//      LOG_INFO(cString::Format("Error greater: LaneCenter %d Place2Be %d Steering %f error_val: %d error_sum: %d", m_LaneCenter.x, m_PlaceToBe.x, m_SteeringAngle, error, e_sum).GetPtr());
        m_f32SteeringAngle = 30.0;
    }
    else if (m_f32SteeringAngle < -30)
    {
//      LOG_INFO(cString::Format("Error smaller: LaneCenter %d Place2Be %d Steering %f error_val: %d error_sum: %d", m_LaneCenter.x, m_PlaceToBe.x, m_SteeringAngle, error, e_sum).GetPtr());
        m_f32SteeringAngle = -30.0;
    }


    /*********************************
    *
    * PT 1 discrete algorithm
    *
    *               Tau
    *       In +    ---  * LastOut
    *             Tsample
    * Out = ---------------------
    *               Tau
    *       1 +     ---
    *             Tsample
    *
    *                           Tau
    * here with     Factor =    ---
    *                         Tsample
    *
    ********************************************/
    m_f64PT1ScaledError = m_i16Error * m_f64PT1InputFactor/*/ 5*/;
    m_f32PT1SteeringOut = static_cast<tFloat32> ((m_f64PT1ScaledError + m_f64PT1Gain * m_f32PT1LastSteeringOut) / (1 + m_f64PT1Gain));
    m_f32PT1LastSteeringOut = m_f32PT1SteeringOut;

    if (m_f32PT1SteeringOut > 30.0f)
    {
        m_f32PT1SteeringOut = 30.0f;
    }
    else if (m_f32PT1SteeringOut < -30.0f)
    {
        m_f32PT1SteeringOut = -30.0f;
    }

	if (m_DriveStraight){
		m_f32SteeringAngle 		= 0.0f;
		m_f32PT1SteeringOut 	= 0.0f;
	}
  
    if(m_bActive){ // (Andi)
    	TransmitSteeringAngle(m_f32SteeringAngle, tsInputTime);        // Send data to output pin
    	TransmitSteeringAnglePT1(m_f32PT1SteeringOut, tsInputTime);
    }

    CreateAndTransmitGCL();

    if (m_bShowDebug)
    {
        __synchronized_kernel(m_oLock);
        tTimeStamp tsStreamTime = _clock->GetStreamTime();

        for (tActiveSignals::iterator oSignal = m_oActive.begin();
            oSignal != m_oActive.end();
            ++oSignal)
        {
            if (*oSignal == LT_SIGREG_ID_CONTROLLER_INPUT)
            {
                tSignalValue sValue;
                sValue.nRawValue = 0;
                sValue.nTimeStamp = tsStreamTime;
                sValue.strTextValue = 0;
                sValue.f64Value = m_i16Error;

                m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
            }
            else if (*oSignal == LT_SIGREG_ID_SCALED_PT1_INPUT)
            {
                tSignalValue sValue;
                sValue.nRawValue = 0;
                sValue.nTimeStamp = tsStreamTime;
                sValue.strTextValue = 0;
                sValue.f64Value = m_f64PT1ScaledError;

                m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
            }
        }
    }
    
    RETURN_NOERROR;
}

/**
 *   Returns the current value of a Signal.
 */
tResult cLaneTracking::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
    if (nSignalID == LT_SIGREG_ID_CONTROLLER_INPUT)
    {
        pValue->f64Value = m_i16Error;
        tTimeStamp nStreamTime = _clock->GetStreamTime();
        pValue->nRawValue = 0;
        pValue-> nTimeStamp = nStreamTime;
        pValue->strTextValue = 0;
    }
    else if (nSignalID == LT_SIGREG_ID_SCALED_PT1_INPUT)
    {
        pValue->f64Value = m_f64PT1ScaledError;
        tTimeStamp nStreamTime = _clock->GetStreamTime();
        pValue->nRawValue = 0;
        pValue-> nTimeStamp = nStreamTime;
        pValue->strTextValue = 0;
    }
    else
    {
        RETURN_ERROR(ERR_NOT_FOUND);
    }


    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cLaneTracking::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{
    if (nSignalID == LT_SIGREG_ID_CONTROLLER_INPUT || nSignalID == LT_SIGREG_ID_SCALED_PT1_INPUT)
    {
        __synchronized_kernel(m_oLock);
        m_oActive.insert(nSignalID);
    }
    else
    {
        RETURN_ERROR(ERR_NOT_FOUND);
    }
        
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cLaneTracking::DeactivateSignalEvents(tSignalID nSignalID)
{
    __synchronized_kernel(m_oLock);
    m_oActive.erase(nSignalID);
    RETURN_NOERROR;
}

tResult cLaneTracking::LongitudinalControl(sPoint *psPointsFar, tUInt8 *pui8PointsCountFar, tTimeStamp tsInputTime)
{
        
    if(*pui8PointsCountFar == 2)
    {
        m_i16FarLaneWidthCalc = psPointsFar[1].x - psPointsFar[0].x;

        //LOG_INFO(cString::Format("FarLaneWidth calc: %i", FarLaneWidth_calc));

        if ((m_i16FarLaneWidthCalc > m_i16LaneWidthMinFar) && (m_i16FarLaneWidthCalc < m_i16LaneWidthMaxFar))
        {
            m_i16FarLaneWidth = m_i16FarLaneWidthCalc;
            m_i16FarLaneCenter = psPointsFar[0].x + (static_cast<tInt>(m_i16FarLaneWidth/2));
            m_nBlindCounterFar = 0;
        } 
         
         //LOG_INFO(cString::Format("FarLaneCenter: %i", FarLaneCenter));
         //LOG_INFO(cString::Format("FarLaneWidth: %i", FarLaneWidth));
    }
    else if(*pui8PointsCountFar == 1)
    {
         tInt nFarCenterLeft = psPointsFar[0].x + (static_cast<tInt>(m_i16FarLaneWidth/2));

         tInt nFarCenterRight = psPointsFar[0].x - (static_cast<tInt>(m_i16FarLaneWidth/2));

         if(abs(nFarCenterRight - m_i16FarLaneCenter) < MAX_DEVIATION)
         {
             m_i16FarLaneCenter = nFarCenterRight;
             m_nBlindCounterFar = 0;
         } 
         else if(abs(nFarCenterLeft - m_i16FarLaneCenter) < MAX_DEVIATION)
         {
             m_i16FarLaneCenter = nFarCenterLeft;
             m_nBlindCounterFar = 0;
         }
         
    } 
    else if(*pui8PointsCountFar == 3)
    {
        tInt16 i16FarWidth1 = psPointsFar[1].x - psPointsFar[0].x;
        tInt16 i16FarWidth2 = psPointsFar[2].x - psPointsFar[1].x;
        
        if((i16FarWidth2 > m_i16LaneWidthMinFar) && (i16FarWidth2 < m_i16LaneWidthMaxFar))
        {
           m_i16FarLaneCenter = psPointsFar[1].x + (static_cast<tInt>(i16FarWidth2/2)); 
           m_nBlindCounterFar = 0;
        }
        else if((i16FarWidth1 > m_i16LaneWidthMinFar) && (i16FarWidth1 < m_i16LaneWidthMaxFar))
        {
           m_i16FarLaneCenter = psPointsFar[0].x + (static_cast<tInt>(i16FarWidth1/2));
           m_nBlindCounterFar = 0;
        }
    }
    else
    {    // If no or more than 3 points are found, it is supposed that the car is located in a curve, looking anywhere. 
        // increase the blind counter
        m_nBlindCounterFar++; 
    }
    
    if (m_nBlindCounterFar > 5)
    {
        // For this reason the variable Lanecenter is set to 0, since this will 
        // cause the car to continue driving at m_AccelerationMin.
        m_i16FarLaneCenter = 0;
    }


    if (m_nBlindCounter > 30 && !m_DriveStraight) //(Andi)
    {
         m_f32AccelerateOut = 0.0;        // If the car can't find lanes anymore --> stop
    }
    else
    {
//        tInt16 i16Difference = abs(m_i16FarLaneCenter - m_sPlaceToBe.x);
         tInt16 i16Difference = abs(m_i16FarLaneCenter - m_sLaneCenterNear.x);
         tFloat32 f32AccelerationDifference = m_f32AccelerationMax - m_f32AccelerationMin;
         tFloat32 f32Factor = (m_nAccelerationFarNearDiff - i16Difference) > 0 ? (m_nAccelerationFarNearDiff - i16Difference) / (tFloat32) m_nAccelerationFarNearDiff : 0.0f;
         m_f32AccelerateOut = m_f32AccelerationMin + (f32Factor * f32AccelerationDifference);
         // Adjust current near line according to absolute output acceleration (speed) 
         // higher speed results in farther distance => look ahead!
         f32Factor = m_f32AccelerateOut / 100;
         m_nCurrentNearLine = (tInt) (f32Factor * m_nNearLineMaxOffset) + m_nNearLine;
    }
/*    else if(abs(m_i16FarLaneCenter - m_sLaneCenterNear.x) <= m_nAccelerationFarNearDiff)    // This parameter determines how early the car brakes before and accelerates after a curve
    {
         m_f32AccelerateOut = m_f32AccelerationMax;
    }
    else if(abs(m_i16FarLaneCenter - m_sLaneCenterNear.x) > m_nAccelerationFarNearDiff)
    {
         m_f32AccelerateOut = m_f32AccelerationMin;
    }
*/
    //LOG_INFO(cString::Format("Acceleration: %f", AccelerateOut));
    if (m_bActive)
    {
        // transmit acceleration out only if the "global" active flag is set to true
        TransmitAcceleration(m_f32AccelerateOut, tsInputTime);        // Send data to output pin
    }
    else if (m_hEmergencyStopTimerNegative == NULL && 
            m_hEmergencyStopTimerZero == NULL && 
            //m_hEmergencyStopTimerResume == NULL && 
            m_hStopTimerNegative == NULL &&
            m_hStopTimerZero == NULL)
    {
        TransmitAcceleration(0.0f, tsInputTime);
    }

    RETURN_NOERROR;
}

tResult cLaneTracking::TransmitAcceleration(tFloat32 f32Acceleration, tTimeStamp tsInputTime)
{
	//if (m_bActive){	

		__synchronized_obj(m_oTransmitAccelCritSection);        
		
		/*static tFloat32 AccelLastTransmitted = 1000.0;

		if (fabs(Acceleration - AccelLastTransmitted)< 0.001f)
		{
		    RETURN_NOERROR;
		} 
		
		AccelLastTransmitted = Acceleration;*/
		
		//create new media sample
		cObjectPtr<IMediaSample> pSampleAccel;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSampleAccel));

		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));
		    
		{
		      __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pSampleAccel, pCoderOutput);
	 
		      pCoderOutput->Set("f32Value", (tVoid*)&f32Acceleration);    
		      pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
		}
		      
		 pSampleAccel->SetTime(tsInputTime);
		 RETURN_IF_FAILED(m_oAccelerateOutput.Transmit(pSampleAccel));
		 
	//}

	RETURN_NOERROR;
}


tResult cLaneTracking::TransmitSteeringAngle(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitSteerCritSection);
    
    /*static tFloat32 SteerLastTransmitted = 100.0;

    if (fabs(SteeringAngle - SteerLastTransmitted)< 0.00001f)
    {
        RETURN_NOERROR;
    }
    
    SteerLastTransmitted = SteeringAngle;*/
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
    
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oSteeringAngleOutput.Transmit(pNewMediaSample));

    //LOG_INFO(cString::Format("Sending SteeringAngle: %f", SteeringAngle).GetPtr());

    RETURN_NOERROR;
    
}

tResult cLaneTracking::TransmitSteeringAnglePT1(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitSteerCritSection);
    
    /*static tFloat32 SteerLastTransmitted = 100.0;

    if (fabs(SteeringAngle - SteerLastTransmitted)< 0.00001f)
    {
        RETURN_NOERROR;
    }
    
    SteerLastTransmitted = SteeringAngle;*/
    tFloat32 f32SteeringAngle1 = f32SteeringAngle+90.f;
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle1);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
    
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Transmit(pNewMediaSample));

        //LOG_INFO(cString::Format("Sending SteeringAnglePT1: %f", SteeringAngle).GetPtr());

    RETURN_NOERROR;
    
}

tResult cLaneTracking::TransmitHeadLights(const tBool bHeadLights, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitLightCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBool->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBool, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bHeadLights);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oHeadLightsOutput.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}

tResult cLaneTracking::TransmitLaneType(const tInt32 bLaneType, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitLightCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDesctInt32SignalValue->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDesctInt32SignalValue, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("intValue", (tVoid*)&bLaneType);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oLaneTypeOutput.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}

tResult cLaneTracking::TransmitLaneDetails(vector<tFloat32> details)
{

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pCoderDescLane->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pCoderDescLane, pMediaSample, pCoderOutput); 
	
			if(!m_bLaneOutputSet){
				pCoderOutput->GetID("fz1", m_szIDSignalValueLaneSpot1XOutput);
				pCoderOutput->GetID("fz2", m_szIDSignalValueLaneSpot1YOutput);
				pCoderOutput->GetID("fz3", m_szIDSignalValueLaneSpot2XOutput);
				pCoderOutput->GetID("fz4", m_szIDSignalValueLaneSpot2YOutput);
				m_bLaneOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szIDSignalValueLaneSpot1XOutput, (tVoid*)&(details[0]));
			pCoderOutput->Set(m_szIDSignalValueLaneSpot1YOutput, (tVoid*)&(details[1]));
			pCoderOutput->Set(m_szIDSignalValueLaneSpot2XOutput, (tVoid*)&(details[2]));
			pCoderOutput->Set(m_szIDSignalValueLaneSpot2YOutput, (tVoid*)&(details[3]));
		
		}

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oLaneDetailsOutput.Transmit(pMediaSample);

	RETURN_NOERROR;
    
}

tResult cLaneTracking::CreateAndTransmitGCL()
{
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput.IsConnected() || !m_bShowDebug)
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(8192));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;

    // draw rectangle to scale the video display correctly
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());	
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, m_sInputFormat.nWidth, m_sInputFormat.nHeight);

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 153, 0).GetRGBA());		
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nImagecutHeightUpHough, m_nImagecutWidthRightHough, m_nImagecutHeightDownHough);

    // draw near and far area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
    //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nCurrentNearLine - 20, m_sInputFormat.nWidth, m_nCurrentNearLine + 20);
    //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nFarLine - 20, m_sInputFormat.nWidth, m_nFarLine + 20);
    //cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());
	if(m_HoughpointsFar[0].x != 0) cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_HoughpointsFar[0].x-4, m_HoughpointsFar[0].y-4, m_HoughpointsFar[0].x+4, m_HoughpointsFar[0].y+4);
	if(m_HoughpointsFar[1].x != 0) cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_HoughpointsFar[1].x-4, m_HoughpointsFar[1].y-4, m_HoughpointsFar[1].x+4, m_HoughpointsFar[1].y+4);
	if(m_HoughpointsNear[0].x != 0) cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_HoughpointsNear[0].x-4, m_HoughpointsNear[0].y-4, m_HoughpointsNear[0].x+4, m_HoughpointsNear[0].y+4);
	if(m_HoughpointsNear[1].x != 0) cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_HoughpointsNear[1].x-4, m_HoughpointsNear[1].y-4, m_HoughpointsNear[1].x+4, m_HoughpointsNear[1].y+4);

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
	for(size_t i=0; i<m_HoughGroup1.size();i++)
		{
			float rho = m_HoughGroup1[i].x;
			float theta = m_HoughGroup1[i].y;
			//float rho = m_representerHG3.x;
			//float theta = m_representerHG3.y;
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
		 if((m_HoughGroup1[i].y*180/M_PI<60)&& (m_HoughGroup1[i].y*180/M_PI>35))
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
		}
		else if((m_HoughGroup1[i].y*180/M_PI<145) && (m_HoughGroup1[i].y*180/M_PI>120))
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
		}
		else if((m_HoughGroup1[i].y*180/M_PI<100) && (m_HoughGroup1[i].y*180/M_PI>65)) //&&  (m_lines[i][0]>150))
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,255).GetRGBA());
			if(LaneType == LANE_RIGHT)
			{
				if(m_HoughGroup1[i].x>150)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}
			else
			{
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}
		}
		}
	for(size_t i=0; i<m_HoughGroup2.size();i++)
		{
			float rho = m_HoughGroup2[i].x;
			float theta = m_HoughGroup2[i].y;
			//float rho = m_representerHG3.x;
			//float theta = m_representerHG3.y;
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
		 if((m_HoughGroup2[i].y*180/M_PI<60)&& (m_HoughGroup2[i].y*180/M_PI>35))
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
		}
		else if((m_HoughGroup2[i].y*180/M_PI<145) && (m_HoughGroup2[i].y*180/M_PI>120))
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
		}
		else if((m_HoughGroup2[i].y*180/M_PI<100) && (m_HoughGroup2[i].y*180/M_PI>65)) //&&  (m_lines[i][0]>150))
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,255).GetRGBA());
			if(LaneType == LANE_RIGHT)
			{
				if(m_HoughGroup2[i].x>150)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}
			else
			{
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}
		}
		}/*
			//plot hough lines responsible for lanetype choice
			vector<Point2f> GroupLeft;
			vector<Point2f> GroupRight;
				for(size_t i=0; i<m_lines.size();i++)
				{
					if((m_lines[i][1]*180/M_PI<70)) //Eigentlich sollte die grenze bei 90°(waagerecht) sein aber die  erfahrung hat gezeigt, dass 95 besser funktioniert,weil manchmal ein paar lines über 90° hinaus kommen trotz rechtskurve.
					{
						GroupLeft.push_back(Point2f(m_lines[i][0],m_lines[i][1]));		
					}
					else if((m_lines[i][1]*180/M_PI>110))
					{
						GroupRight.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	
					}
				}
			for(size_t i=0; i<GroupLeft.size();i++)
			{
				float rho = GroupLeft[i].x;
				float theta = GroupLeft[i].y;
				Point pt1,pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000*(-b));
				pt1.y = cvRound(y0 + 1000*(a));
				pt2.x = cvRound(x0 - 1000*(-b));
				pt2.y = cvRound(y0 - 1000*(a));
				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}
			for(size_t i=0; i<GroupRight.size();i++)
			{
				float rho = GroupRight[i].x;
				float theta = GroupRight[i].y;
				Point pt1,pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000*(-b));
				pt1.y = cvRound(y0 + 1000*(a));
				pt2.x = cvRound(x0 - 1000*(-b));
				pt2.y = cvRound(y0 - 1000*(a));
				cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,100,255).GetRGBA());
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
			}*/


/*
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());

	Point2f pt1r,pt2r;
				pt1r.x = cos(m_representerHG2.y)*m_representerHG2.x + 1000*(-sin(m_representerHG2.y));
				pt1r.y = sin(m_representerHG2.y)*m_representerHG2.x + 1000*(cos(m_representerHG2.y));
				pt2r.x = cos(m_representerHG2.y)*m_representerHG2.x - 1000*(-sin(m_representerHG2.y));
				pt2r.y = sin(m_representerHG2.y)*m_representerHG2.x - 1000*(cos(m_representerHG2.y));
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2r.x, pt2r.y+m_nImagecutHeightUpHough, pt1r.x, pt1r.y+m_nImagecutHeightUpHough);

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,100).GetRGBA());
	Point2f pt1l,pt2l;
				pt1l.x = cos(m_representerHG1.y)*m_representerHG1.x + 1000*(-sin(m_representerHG1.y));
				pt1l.y = sin(m_representerHG1.y)*m_representerHG1.x + 1000*(cos(m_representerHG1.y));
				pt2l.x = cos(m_representerHG1.y)*m_representerHG1.x - 1000*(-sin(m_representerHG1.y));
				pt2l.y = sin(m_representerHG1.y)*m_representerHG1.x - 1000*(cos(m_representerHG1.y));
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2l.x, pt2l.y+m_nImagecutHeightUpHough, pt1l.x, pt1l.y+m_nImagecutHeightUpHough);
	*/
    
    // draw the min and max lane width
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());
    //cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMaxFar/2, m_nFarLine, m_i16FarLaneCenter + m_i16LaneWidthMaxFar/2, m_nFarLine + 5);
    //cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMinFar/2, m_nFarLine - 5, m_i16FarLaneCenter + m_i16LaneWidthMinFar/2, m_nFarLine);

   // cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMaxNear/2, m_nCurrentNearLine, m_sLaneCenterNear.x + m_i16LaneWidthMaxNear/2, m_nCurrentNearLine + 5);
   // cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMinNear/2, m_nCurrentNearLine - 5, m_sLaneCenterNear.x + m_i16LaneWidthMinNear/2, m_nCurrentNearLine);

    // draw near and far line
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nCurrentNearLine, m_sInputFormat.nWidth, m_nCurrentNearLine);
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nFarLine, m_sInputFormat.nWidth, m_nFarLine);	
    
    // draw the lines for place to be and the detected lanecenter
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());
    //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sLaneCenterNear.x, m_nCurrentNearLine - 20, m_sLaneCenterNear.x, m_nCurrentNearLine + 20);
    //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sPlaceToBe.x, m_nCurrentNearLine - 10, m_sPlaceToBe.x, m_nCurrentNearLine + 10);
    //cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_i16FarLaneCenter, m_nFarLine - 20, m_i16FarLaneCenter, m_nFarLine + 20);

    // draw a rect for longitudinal control
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_i16FarLaneCenter - m_nAccelerationFarNearDiff, m_nFarLine - 20, m_i16FarLaneCenter + m_nAccelerationFarNearDiff, m_nCurrentNearLine + 20);
    
    
    // draw the found near lines
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,50,100).GetRGBA());
    for (tInt nIdx = 0; nIdx < m_ui8NearPointsCount; ++nIdx)
    {
        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine - 20, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine + 20);    
    }

    // draw the found far lines
    for (tInt nIdx = 0; nIdx < m_ui8FarPointsCount; ++nIdx)
    {
        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsFar[nIdx].x, m_nFarLine - 20, m_asAllpointsFar[nIdx].x, m_nFarLine + 20);    
    }

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

tResult cLaneTracking::CreateAndTransmitGCL2()
{
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput2.IsConnected() || !m_bShowDebug)
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(8192));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;

    // draw rectangle to scale the video display correctly
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());	
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, m_sInputFormat.nWidth, m_sInputFormat.nHeight);

    // draw near and far area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 255).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nCurrentNearLine - 20, m_sInputFormat.nWidth, m_nCurrentNearLine + 20);
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, m_nFarLine - 20, m_sInputFormat.nWidth, m_nFarLine + 20);
    
    // draw the min and max lane width
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMaxFar/2, m_nFarLine, m_i16FarLaneCenter + m_i16LaneWidthMaxFar/2, m_nFarLine + 5);
    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_i16FarLaneCenter - m_i16LaneWidthMinFar/2, m_nFarLine - 5, m_i16FarLaneCenter + m_i16LaneWidthMinFar/2, m_nFarLine);

    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMaxNear/2, m_nCurrentNearLine, m_sLaneCenterNear.x + m_i16LaneWidthMaxNear/2, m_nCurrentNearLine + 5);
    cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, m_sLaneCenterNear.x - m_i16LaneWidthMinNear/2, m_nCurrentNearLine - 5, m_sLaneCenterNear.x + m_i16LaneWidthMinNear/2, m_nCurrentNearLine);

    // draw near and far line
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nCurrentNearLine, m_sInputFormat.nWidth, m_nCurrentNearLine);
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, 0, m_nFarLine, m_sInputFormat.nWidth, m_nFarLine);	
    
    // draw the lines for place to be and the detected lanecenter
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sLaneCenterNear.x, m_nCurrentNearLine - 20, m_sLaneCenterNear.x, m_nCurrentNearLine + 20);
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_sPlaceToBe.x, m_nCurrentNearLine - 10, m_sPlaceToBe.x, m_nCurrentNearLine + 10);
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_i16FarLaneCenter, m_nFarLine - 20, m_i16FarLaneCenter, m_nFarLine + 20);

    // draw a rect for longitudinal control
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_i16FarLaneCenter - m_nAccelerationFarNearDiff, m_nFarLine - 20, m_i16FarLaneCenter + m_nAccelerationFarNearDiff, m_nCurrentNearLine + 20);
    
    
    // draw the found near lines
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,50,100).GetRGBA());
    for (tInt nIdx = 0; nIdx < m_ui8NearPointsCount; ++nIdx)
    {
        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine - 20, m_asAllpointsNear[nIdx].x, m_nCurrentNearLine + 20);    
    }

    // draw the found far lines
    for (tInt nIdx = 0; nIdx < m_ui8FarPointsCount; ++nIdx)
    {
        cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, m_asAllpointsFar[nIdx].x, m_nFarLine - 20, m_asAllpointsFar[nIdx].x, m_nFarLine + 20);    
    }

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput2.Transmit(pSample));
    RETURN_NOERROR;
}

/*
 * Function to change the velocity properties of this filter
 */
tResult cLaneTracking::ChangeCurrentVelocity(IMediaSample* pMediaSample){
	
	tFloat32 fMinVel = m_f32AccelerationMin;
	tFloat32 fMaxVel = m_f32AccelerationMax;
	
	{
		__adtf_sample_read_lock_mediadescription(m_pCurrentVelocityInput, pMediaSample, pCoder);    

		if (!m_bCurrentVelocityInputSet){
			pCoder->GetID("fMinVel", m_szIDSignalValueCurrentVelocityMinInput);
			pCoder->GetID("fMaxVel", m_szIDSignalValueCurrentVelocityMaxInput);
			m_bCurrentVelocityInputSet = tTrue;
		}

		pCoder->Get(m_szIDSignalValueCurrentVelocityMinInput, (tVoid*)&fMinVel);
		pCoder->Get(m_szIDSignalValueCurrentVelocityMaxInput, (tVoid*)&fMaxVel);
	}

	LOG_INFO(cString::Format("LF: fMinVel: %f, fMaxVel: %f", fMinVel, fMaxVel) );
	
	if (fMinVel > fMaxVel){
		LOG_WARNING(cString::Format("fMinVel > fMaxVel not allowed!"));
	}else{

		// Ignore a value if it is not valid
		m_f32AccelerationMin 	= ( fMinVel < 0.0 || fMinVel > 5.0 ) ? m_f32AccelerationMin : fMinVel;
		m_f32AccelerationMax 	= ( fMaxVel < 0.0 || fMaxVel > 5.0 ) ? m_f32AccelerationMax : fMaxVel;

 	}

	RETURN_NOERROR;
}

tResult cLaneTracking::DriveStraight(IMediaSample* pMediaSample){

	tBool bValue;

	{
    	__adtf_sample_read_lock_mediadescription(m_pBoolDriveStraightInput, pMediaSample, pCoder);

		if (!m_bDriveStraightSet){

			pCoder->GetID("bValue", m_szIDBoolValueDriveStraightInput);
			m_bDriveStraightSet 	= tTrue;

		}

   		pCoder->Get(m_szIDBoolValueDriveStraightInput, (tVoid*)&bValue); 

	}

	m_DriveStraight 	= bValue;  

	LOG_INFO(cString::Format("LF: DriveStraightReceived: %i", m_DriveStraight) );

	RETURN_NOERROR;
}
