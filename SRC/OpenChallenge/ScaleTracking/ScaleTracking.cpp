/*
 * Date 17.03.16   
 */   
 
#include "stdafx.h" 
#include "ScaleTracking.h"

#include <fstream>
#include <iomanip>

#define WIDTH_MIN_FAR 0
#define WIDTH_MIN_NEAR 0
#define NEAR_FIELD 0
#define FAR_FIELD 1

#define X_DEVIATION 25 
//15


#define X_WIDTH 100

#define LAST_DISTANCE 0.15

#define AVERAGING_COUNT 2

#define MID_DIST 10



ADTF_FILTER_PLUGIN("ScaleTracking", OID_ADTF_ScaleTracking, cScaleTracking)

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


cScaleTracking::cScaleTracking(const tChar* __info) : cFilter(__info), 
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

    SetPropertyInt(LT_PROP_TRESHOLD, 160);
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

    SetPropertyFloat(LT_PROP_ThresholdValueCanny, 230);
    SetPropertyBool(LT_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(LT_PROP_ThresholdValueHough, 65);
    SetPropertyBool(LT_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");

    m_pISignalRegistry = NULL;
}

cScaleTracking::~cScaleTracking()
{
}

tResult cScaleTracking::GetInterface(const tChar* idInterface,
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

tUInt cScaleTracking::Ref()
{
    return cFilter::Ref();
}

tUInt cScaleTracking::Unref()
{
    return cFilter::Unref();
}

tVoid cScaleTracking::Destroy()
{
    delete this;
}

tResult cScaleTracking::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cScaleTracking::Stop(__exception)
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
tResult cScaleTracking::Init(tInitStage eStage, __exception )
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
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSteeringAngle)); 
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSteeringPT1));
        
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSpeed)); 
        
        // Media Description Bool
        tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
        cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        

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

        //Steering Angle PT1 Output
        RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Create("Steering_Angle_PT1", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringAnglePT1Output))

        
        //HeadLights Output
        RETURN_IF_FAILED(m_oHeadLightsOutput.Create("HeadLights", pTypeSignalBoolValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oHeadLightsOutput));
        RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolHeadlight));


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

              
    }
    else if (eStage == StageNormal)
    {
		m_bLFStatusSet 				= tFalse;
		m_bStartCalcAreaOutputSet 	= tFalse;


		m_lastPointX 				= -1;

		m_numberGoodPoints			= 0;

        m_bFirstFrame = true;
        m_ui8Imagecount = 0;
        m_bActive = tFalse;

		m_bCurrentVelocityInputSet = tFalse;

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

tResult cScaleTracking::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cScaleTracking::ReadProperties(const tChar* strPropertyName)
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

tResult cScaleTracking::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
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

tResult cScaleTracking::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cScaleTracking::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cScaleTracking::ProcessInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{
    if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
        TransmitAcceleration(0.0f, tsInputTime);
        TransmitSteeringAngle(0.0f, tsInputTime);
        TransmitSteeringAnglePT1(0.0f, tsInputTime);
        TransmitHeadLights(tTrue, tsInputTime);    // Switch on Headlights
        m_ui8InitCtrl++;
    }
    else if(m_bActive)
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
//        Mat matImageCutNear= image(cv::Range(m_nCurrentNearLine - 20, m_nCurrentNearLine + 20), cv::Range(m_sInputFormat.nWidth/2-X_WIDTH, m_sInputFormat.nWidth/2+X_WIDTH)).clone(); //Cut Image
		Mat matImageCutNear= image(cv::Range(m_nCurrentNearLine - 20, m_nCurrentNearLine + 20), cv::Range(m_sInputFormat.nWidth/2-X_WIDTH, m_sInputFormat.nWidth/2+X_WIDTH));
/*//Mat matImageCutNear= image(cv::Range(350 - 20, 350 + 20), cv::Range(0, m_sInputFormat.nWidth)).clone(); //Cut Image        
        GaussianBlur(matImageCutNear, matImageCutNear, Size(11,11), 0, 0, BORDER_DEFAULT); // Filter
        cvtColor(matImageCutNear, m_matGreyNear ,CV_RGB2GRAY);// Grey Image
        threshold(m_matGreyNear, m_matGreyThreshNear, m_nThresholdValue, 500,THRESH_BINARY);// Generate Binary Image
        Canny(m_matGreyThreshNear, m_matLineCannyNear, 0, 2, 3, false);// Detect Edges
    */

        // Set Blue and purple colors to dark grey
		Size matNearSize	= matImageCutNear.size();

		// First get current max values
		/*uchar max1 	= 0;
		uchar max2 	= 0;
		for (int i = 0; i < matNearSize.height; ++i) {
			for (int j = 0; j < matNearSize.width; ++j) {
				Vec3b color = matImageCutNear.at<Vec3b>(Point(j,i));
	
				max1 = color[1] > max1 ? color[1] : max1;
				max2 = color[2] > max2 ? color[2] : max2;
			}
		}*/

		//LOG_INFO(cString::Format("LiT: max1 = %i, max2 = %i", max1, max2) );
		
        
		for (int i = 0; i < matNearSize.height; ++i) {
			for (int j = 0; j < matNearSize.width; ++j) {
				Vec3b color = matImageCutNear.at<Vec3b>(Point(j,i));

				if ( color[0] > color[1] * 1.3 || color[2] < 60){
					color[0] = 0;
					color[1] = 0;
					color[2] = 0;
				}
				matImageCutNear.at<Vec3b>(Point(j,i)) = color;
			}
		}

		//matImageCutNear 	= cv::Scalar::all(255)-matImageCutNear;
        
        
        cvtColor(matImageCutNear, matImageCutNear ,CV_RGB2GRAY);// Grey Image 	
		threshold(matImageCutNear ,matImageCutNear ,60, 255,THRESH_TOZERO); // delete dark noise
		matImageCutNear=matImageCutNear-60;
/*		fstream f3;
		f3.open("thres.dat",ios::out);
		f3 << matImageCutNear << "\n";
		f3.close();	*/
		normalize(matImageCutNear,matImageCutNear, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(matImageCutNear, matImageCutNear); // Konveriere in 8Bit array. Nehme aus abs
		threshold(matImageCutNear, matImageCutNear, 45, 255,THRESH_TOZERO); // delete dark noise
        medianBlur(matImageCutNear, matImageCutNear,3); // reduce bright noise with edge-preserving filter
        Canny(matImageCutNear, m_matLineCannyNear,m_nThresholdValue, m_nThresholdValueCanny, 3, false);// Detect Edges ---------Nur Canny "funktioniert"-------

/*		fstream f5;
		f5.open("LineTrackingCanny.dat",ios::out);
		f5 << m_matLineCannyNear << "\n";
		f5.close(); */

        // search points within the canny near
        Search(m_asAllpointsNear, &m_ui8NearPointsCount, 40, m_matLineCannyNear, NEAR_FIELD );

		LateralControl(m_asAllpointsNear, &m_ui8NearPointsCount, tsInputTime);

		// Search for horizontal line
		Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_nImagecutWidthLeftHough, m_nImagecutWidthRightHough)).clone(); //Cut Image 
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 	
		threshold(m_matCutHough ,m_matCutHough ,60, 255,THRESH_TOZERO); // delete dark noise
		m_matCutHough=m_matCutHough-60;
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCannyHough,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges ---------Nur Canny "funktioniert"-------

		CreateAndTransmitGCL();
        

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


		
	}
    RETURN_NOERROR;            
}

tResult cScaleTracking::ProcessFound()
{        

    RETURN_NOERROR;
}

tResult cScaleTracking::ProcessOutput()
{

    RETURN_NOERROR;
}


tInt16 cScaleTracking::GetAverage(tInt16 x, std::deque<tInt16> &Buffer)
{
	Buffer.push_back(x);
	size_t sBuffer = Buffer.size();
	if(sBuffer>AVERAGING_COUNT)
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

/*
 * Algorithmn to search for a certain point that represents the line
 */
tResult cScaleTracking::Search(sPoint *psPoints, tUInt8 *pui8PointsCount, tUInt8 ui8Limit, cv::Mat matCannyImg, int mode)
{
	
	// If we are here for the first time, set old x value to the middle of the image
	if (m_lastPointX == -1){
		m_lastPointX 		= m_sInputFormat.nWidth/2;
	}

    A_UTIL_ASSERT(NULL != psPoints);
    A_UTIL_ASSERT(NULL != pui8PointsCount);

    *pui8PointsCount    = 0;
    Size szCannySize    = matCannyImg.size();

	m_goodXPoints.clear();
	m_veryGoodXPoints.clear();
	

	for(tInt nColumn = 0; nColumn < szCannySize.width; nColumn++){
		// Save strong values
		if (matCannyImg.at<uchar>(szCannySize.height/2, nColumn)==255){
			m_goodXPoints.push_back(nColumn);			
		}
	}

	m_numberGoodPoints 	= m_goodXPoints.size();

	tInt16 xValue = -10000;

	if (m_goodXPoints.size()==0){
		//LOG_INFO(cString::Format("LiT: No Point Found") );
		// use old point
		xValue = m_lastPointX;
	}
	else if(m_goodXPoints.size()==1){
		//LOG_INFO(cString::Format("LiT: Only One Point Found") );
		xValue 	= m_goodXPoints.at(0);
	}
	else{
		
		vector<tInt16> xDistances;
		// Calculate distances between all points
		for (size_t i = 0; i < m_goodXPoints.size()-1; i++){
			xDistances.push_back(m_goodXPoints.at(i+1)-m_goodXPoints.at(i));			
		}


		vector<tInt16> smallDistancesInd;
		for (size_t j = 0; j < xDistances.size(); j++){
			//line size should be 29pt
			if (xDistances.at(j) < 35 && xDistances.at(j) > 25){
				smallDistancesInd.push_back(j);
			}		
		}
				

		// Take that point, which is closest to the last one we had
		for (size_t i = 0; i < smallDistancesInd.size(); i++){

			// Save very good points to plot with gcl
			m_veryGoodXPoints.push_back(m_goodXPoints.at(smallDistancesInd.at(i)));

			if ( abs(m_goodXPoints.at(smallDistancesInd.at(i)) + 15-m_lastPointX) < abs(xValue - m_lastPointX) ){
				xValue = m_goodXPoints.at(smallDistancesInd.at(i)) + 15;
			}
		}

		// No new point found -> take old one again
		xValue = xValue == -10000 ? m_lastPointX : xValue;

	}

	// get average of xValue
	xValue 	= GetAverage(xValue, m_xValueBuffer);

	psPoints[*pui8PointsCount].x = static_cast<tInt>(xValue+m_sInputFormat.nWidth/2-X_WIDTH);
	psPoints[*pui8PointsCount].y = static_cast<tInt>(m_nCurrentNearLine);
	// remember value
	m_lastPointX = xValue;
	(*pui8PointsCount)++;
	RETURN_NOERROR;
}

tResult cScaleTracking::LateralControl(sPoint *psPoints, tUInt8 *pui8PointsCount, tTimeStamp tsInputTime)
{

    if(*pui8PointsCount == 2)
    {
		//LOG_INFO(cString::Format("LiT: 2 Points!") );
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
			//LOG_WARNING(cString::Format("LF: LaneWidthNear not correct: i16LaneWidthCalc | m_i16LaneWidthMinNear | m_i16LaneWidthMaxNear | abs(nLaneCenterCalc - m_sLaneCenterNear.x) | MAX_DEVIATION = %i | %i | %i | %i | %i", i16LaneWidthCalc, m_i16LaneWidthMinNear, m_i16LaneWidthMaxNear, abs(nLaneCenterCalc - m_sLaneCenterNear.x), MAX_DEVIATION) );
            m_nBlindCounter++;
        }        
    }
    else if(*pui8PointsCount == 3)
    {
		//LOG_INFO(cString::Format("LiT: 3 Points!") );
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
		//LOG_INFO(cString::Format("LiT: 1 Points!") );
        // If just one point is found the lane center shall be calculated based on this single point and the half of the previously calculated lane width
        // If the right point of the lane was found LaneWidth/2 must be subtracted from the position of the point, if the left point was found LaneWidth/2
        // must be added. Whether to add or to subtract LaneWidth/2 is determined by comparing the calculated new LaneCenter with the old LaneCenter.

/*        m_nCenterFromLeft = psPoints[0].x + (static_cast<tInt>(m_i16LaneWidth/2));

        m_nCenterFromRight = psPoints[0].x - (static_cast<tInt>(m_i16LaneWidth/2));

        if(abs(m_nCenterFromLeft - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
			LOG_INFO(cString::Format("LiT: CASE1") );
            m_sLaneCenterNear.x = m_nCenterFromLeft;
            m_nBlindCounter = 0;
        }
        else if(abs(m_nCenterFromRight - m_sLaneCenterNear.x) < MAX_DEVIATION)
        {
			LOG_INFO(cString::Format("LiT: CASE2") );
            m_sLaneCenterNear.x = m_nCenterFromRight;
            m_nBlindCounter = 0;
        }
        else
        {
			LOG_INFO(cString::Format("LiT: CASE3") );
            m_nBlindCounter++;
        } */   

		m_sLaneCenterNear.x = psPoints[0].x;	
            
    }
    else
    {    
		//LOG_INFO(cString::Format("LiT: 0 Points!") );
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

  
    if(m_bActive){ // (Andi)
    	TransmitSteeringAngle(m_f32SteeringAngle, tsInputTime);        // Send data to output pin
    	TransmitSteeringAnglePT1(m_f32PT1SteeringOut, tsInputTime);
		TransmitAcceleration(m_f32AccelerationMin ,tsInputTime);
    }
    

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
tResult cScaleTracking::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
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
tResult cScaleTracking::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
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
tResult cScaleTracking::DeactivateSignalEvents(tSignalID nSignalID)
{
    __synchronized_kernel(m_oLock);
    m_oActive.erase(nSignalID);
    RETURN_NOERROR;
}


tResult cScaleTracking::TransmitAcceleration(tFloat32 f32Acceleration, tTimeStamp tsInputTime)
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
		m_pCoderDescSignalSpeed->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		RETURN_IF_FAILED(pSampleAccel->AllocBuffer(nSize));
		    
		{
		      __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalSpeed, pSampleAccel, pCoderOutput);
	 
		      pCoderOutput->Set("f32Value", (tVoid*)&f32Acceleration);    
		      pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
		}

	/*fstream fff;
	fff.open("LF_Acceleration", ios::out|ios::app);
	fff << f32Acceleration << "\n";
	fff.close(); */
		      
		 pSampleAccel->SetTime(tsInputTime);
		 RETURN_IF_FAILED(m_oAccelerateOutput.Transmit(pSampleAccel));
		 
	//}

	RETURN_NOERROR;
}


tResult cScaleTracking::TransmitSteeringAngle(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
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
    m_pCoderDescSignalSteeringAngle->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalSteeringAngle, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
    
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oSteeringAngleOutput.Transmit(pNewMediaSample));

    //LOG_INFO(cString::Format("Sending SteeringAngle: %f", SteeringAngle).GetPtr());

    RETURN_NOERROR;
    
}

tResult cScaleTracking::TransmitSteeringAnglePT1(const tFloat32 f32SteeringAngle, tTimeStamp tsInputTime)
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
    m_pCoderDescSignalSteeringPT1->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalSteeringPT1, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("f32Value", (tVoid*)&f32SteeringAngle1);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
    
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oSteeringAnglePT1Output.Transmit(pNewMediaSample));

        //LOG_INFO(cString::Format("Sending SteeringAnglePT1: %f", SteeringAngle).GetPtr());

    RETURN_NOERROR;
    
}

tResult cScaleTracking::TransmitHeadLights(const tBool bHeadLights, tTimeStamp tsInputTime)
{
    __synchronized_obj(m_oTransmitLightCritSection);
    
    //create new media sample
    cObjectPtr<IMediaSample> pNewMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pNewMediaSample));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolHeadlight->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    RETURN_IF_FAILED(pNewMediaSample->AllocBuffer(nSize));

    {
          __adtf_sample_write_lock_mediadescription(m_pCoderDescBoolHeadlight, pNewMediaSample, pCoderOutput);    
 
          pCoderOutput->Set("bValue", (tVoid*)&bHeadLights);    
          pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&m_tsArduinoTime);    
    }
              
    pNewMediaSample->SetTime(tsInputTime);
    RETURN_IF_FAILED(m_oHeadLightsOutput.Transmit(pNewMediaSample));

    RETURN_NOERROR;
    
}


tResult cScaleTracking::CreateAndTransmitGCL()
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

    // draw searching area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(m_sInputFormat.nWidth/2-X_WIDTH) , static_cast<tInt16>(m_nCurrentNearLine + 20), static_cast<tInt16>(m_sInputFormat.nWidth/2+X_WIDTH), static_cast<tInt16>(m_nCurrentNearLine - 20));

	// draw hough searching area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100, 100, 100).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(m_nImagecutWidthLeftHough) , static_cast<tInt16>(m_nImagecutHeightUpHough), static_cast<tInt16>(m_nImagecutWidthRightHough), static_cast<tInt16>(m_nImagecutHeightDownHough));

	// draw found hough lines
	for(size_t i=0; i<m_lines.size();i++)
	{
		float rho = m_lines[i][0];
		float theta = m_lines[i][1];
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

	// draw found point
	sPoint point = m_asAllpointsNear[0];
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 255, 255).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(point.x-3) , static_cast<tInt16>(point.y + 3), static_cast<tInt16>(point.x+3), static_cast<tInt16>(point.y - 3));

	// draw all possible points
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100, 100, 100).GetRGBA());
	for (size_t i = 0; i < m_goodXPoints.size(); i++){		
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(m_goodXPoints.at(i)-2+m_sInputFormat.nWidth/2-X_WIDTH) , static_cast<tInt16>(m_nCurrentNearLine+2), static_cast<tInt16>(m_goodXPoints.at(i)+2+m_sInputFormat.nWidth/2-X_WIDTH), static_cast<tInt16>(m_nCurrentNearLine-2));	
	}

	// draw all possible points with near neighbours
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 255).GetRGBA());
	for (size_t i = 0; i < m_veryGoodXPoints.size(); i++){		
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(m_veryGoodXPoints.at(i)-2+m_sInputFormat.nWidth/2-X_WIDTH) , static_cast<tInt16>(m_nCurrentNearLine+2), static_cast<tInt16>(m_veryGoodXPoints.at(i)+2+m_sInputFormat.nWidth/2-X_WIDTH), static_cast<tInt16>(m_nCurrentNearLine-2));	
	}

	// Plot number of found points Output
	cString strText = cString::FromInt32(m_numberGoodPoints);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 


    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    
    RETURN_NOERROR;

}


/*
 * Function to change the velocity properties of this filter
 */
tResult cScaleTracking::ChangeCurrentVelocity(IMediaSample* pMediaSample){
	
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

	if (!m_bActive && fMaxVel > 0.0 && fMaxVel < 5.0 && fMinVel > 0.0 && fMinVel < 5.0  ) {
		ReadProperties(LT_PROP_CAMERA_OFFSET);
		m_bActive = tTrue;


		m_xValueBuffer.clear();		// Reset all buffers!!
	

		m_i16ErrorSum = 0.0; 
	}
	
	
	if (fMinVel > fMaxVel){
		LOG_WARNING(cString::Format("fMinVel > fMaxVel not allowed!"));
	}else{

		// Ignore a value if it is not valid
		m_f32AccelerationMin 	= ( fMinVel < 0.0 || fMinVel > 5.0 ) ? m_f32AccelerationMin : fMinVel;
		m_f32AccelerationMax 	= ( fMaxVel < 0.0 || fMaxVel > 5.0 ) ? m_f32AccelerationMax : fMaxVel;

 	}

	if (fMaxVel == 0 && fMinVel == 0) {
		TransmitAcceleration(0.0f, _clock->GetStreamTime());
		m_bActive = tFalse;
	}

	//LOG_INFO(cString::Format("LF (Wtd): fMinVel: %f, fMaxVel: %f", fMinVel, fMaxVel) );
	//LOG_INFO(cString::Format("LF (Curr): fMinVel: %f, fMaxVel: %f", m_f32AccelerationMin, m_f32AccelerationMax) );

	RETURN_NOERROR;
}
