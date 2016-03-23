/*
 * Date 17.03.2016  
 */ 

#include "AdaptiveCruiseControl.h"
#include "../Util/Util.h"

#include <fstream>

#include "../../include/lane_type.h"

#define IMAGE_CUT_LEFT 0
#define IMAGE_CUT_RIGHT 320
#define IMAGE_CUT_UP 10
#define IMAGE_CUT_DOWN 140

#define NUMBER_SQUARES 20
#define NUMBER_SQUARES_CONTRAFLOW 60

#define BUFFER_MAX_SIZE 10
#define SET_ELEMENTS 3

#define CONTRAFLOW_BUFFER_MAX_SIZE 50

// Percentage to evaluate found squares
#define FILLING 0.9

// Minimum-distance in [m]
#define CRITICAL_DISTANCE 0.7

// Maximum-distance in [m]
#define UNCRITICAL_DISTANCE 1.5

#define STANDING_THRESHOLD 5

// Radius of curves
#define RADIUS_RO 2.1
#define RADIUS_RI 1.35

#define RADIUS_LO 2.6
#define RADIUS_LI 1.8

#define RADIUS_SRO 1.35
#define RADIUS_SRI 1.1
#define RADIUS_BRO 2.35
#define RADIUS_BRI 2.1
#define RADIUS_SLO 1.80
#define RADIUS_SLI 1.6
#define RADIUS_BLO 2.8
#define RADIUS_BLI 2.6

// Steering Angles
#define LOWER_STEERING 77
#define MIDDLE_STEERING_MIN 85 // TODO: are these two
#define MIDDLE_STEERING_MAX 97 //       values correct?
#define UPPER_STEERING 107

#define HOUGH_CUT_X1 0
#define HOUGH_CUT_X2 280
#define HOUGH_CUT_Y1 35
#define HOUGH_CUT_Y2 160

// define properties to avoid errors
// define the ADTF property names to avoid errors 
#define LT_PROP_ThresholdValueCanny "Hough::Canny Threshold value"
#define LT_PROP_ThresholdValueHough "Hough::Hough Threshold value"


ADTF_FILTER_PLUGIN("AdaptiveCruiseControl", __guid, cAdaptiveCruiseControl);


cAdaptiveCruiseControl::cAdaptiveCruiseControl(const tChar* __info)
{
    SetPropertyInt(LT_PROP_ThresholdValueCanny, 40);
    SetPropertyBool(LT_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyInt(LT_PROP_ThresholdValueHough, 100);
    SetPropertyBool(LT_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(LT_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines - length of a hough line");

}

cAdaptiveCruiseControl::~cAdaptiveCruiseControl()
{

}

tResult cAdaptiveCruiseControl::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cAdaptiveCruiseControl::ReadProperties(const tChar* strPropertyName)
{
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(LT_PROP_ThresholdValueCanny);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, LT_PROP_ThresholdValueHough))
    {
        m_nThresholdValueHough = GetPropertyInt(LT_PROP_ThresholdValueHough);
    }

	RETURN_NOERROR;
}

tResult cAdaptiveCruiseControl::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	// pins need to be created at StageFirst
	if (eStage == StageFirst)    {

		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
		
		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for SignalValue 
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");   
		RETURN_IF_POINTER_NULL(strDescSignalValue);    
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
			
		// Get description for tInt32
		tChar const * strDescInt32SignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");	
		RETURN_IF_POINTER_NULL(strDescInt32SignalValue);	 
		cObjectPtr<IMediaType> pTypeInt32SignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescInt32SignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
		// Media Description Overtaking
		tChar const * OvertakingDesc = pDescManager->GetMediaDescription("tOvertakingStruct");
		RETURN_IF_POINTER_NULL(OvertakingDesc);
		cObjectPtr<IMediaType> pTypeOvertakingDesc = new cMediaType(0, 0, 0, "tOvertakingStruct", OvertakingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// State ofACC
		RETURN_IF_FAILED(m_iStateACC.Create("Set_State", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStateACC));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateACCInput));

		// Depthimage Input
		RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

		// create Inputpin for Steering Angle (Input)
		RETURN_IF_FAILED(m_iSteeringAngle.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iSteeringAngle));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalSteeringAngleInput));

		// create Inputpin for desired CarSpeed (Input)
		RETURN_IF_FAILED(m_iCarSpeed.Create("Desired_Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iCarSpeed));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalCarSpeedInput));

		// Current Lanetype (Input)
		RETURN_IF_FAILED(m_iCurrentLanetype.Create("Lane_Type", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iCurrentLanetype));
		RETURN_IF_FAILED(pTypeInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCurrentLanetypeInput));
		
		// Overtaking Allowed (Input)
		RETURN_IF_FAILED(m_iOvertakingAllowed.Create("OvertakingAllowed", pTypeOvertakingDesc, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_iOvertakingAllowed));
		RETURN_IF_FAILED(pTypeOvertakingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pOvertakingInput));

		//create pin for left side ultrasonic sensor
		/*RETURN_IF_FAILED(m_iUSSensorLeft.Create("US_Sensor_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iUSSensorLeft));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSLeftInput));*/

		//create pin for front left ultrasonic sensor
		RETURN_IF_FAILED(m_iUSSensorFrontLeft.Create("US_Sensor_Front_Left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iUSSensorFrontLeft));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSFrontLeftInput));

        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput)); 

		// create Output pin for CarSpeed 
		RETURN_IF_FAILED(m_oCarSpeed.Create("Adaptive_Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oCarSpeed));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalSpeedOutput));

		// Output pin to start overtaking
		RETURN_IF_FAILED(m_oStartDriveRound.Create("drive_round", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStartDriveRound));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateDROutput));

		// create Pin for Breaklight (Output)
		RETURN_IF_FAILED(m_oBreaklight.Create("Breaklight", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oBreaklight));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBreaklightOutput));
		

	}
	else if(eStage == StageNormal)
	{

	}
	else if(eStage == StageGraphReady)
	{

		m_bFirstFrameDepthimage 	= tTrue;
		m_bFirstFrame 				= tTrue;

		m_iframeCounter 			= 0;

		m_referenceSpeed 			= 0.0;

		m_currentSteering 			= 90.0;

		m_currDistance 				= 0.0;

		m_USFrontBufferValue		= 0.0;
		//m_USSideBufferValue			= 0.0;

		m_derivation 			 	= 0.0;

		m_standingCounter			= 0;
		
		m_ACCActive 				= tTrue;

		m_startLookingForContraflow = tFalse;
		m_noContraflow 				= tFalse;

		m_LongStraightLane 			= tFalse;
		m_lines.clear();
		m_lines_corr.clear();

		// Overtaking-Allowed flags
		m_OvertakingAlowed_RoadSign 	= tTrue;
		m_OvertakingAlowed_Maneuvre		= tTrue;
		m_OvertakingAlowed_Pedestrain 	= tTrue;

		m_LeftFrontFree				= tFalse;
		//m_LeftSideFree				= tFalse;

		m_breaking 					= tFalse;

		// Default lanetype = straight
		m_Lanetype 					= LANE_STRAIGHT;
		
		// ID's set
		m_bStateACCInputSet 		= tFalse;
		m_bSpeedOutputSet			= tFalse;
		m_bCarSpeedInputSet			= tFalse;
		m_bLanetypeInputSet 		= tFalse;
		m_bSteeringAngleInputSet 	= tFalse;
		m_bStateDROutputSet 		= tFalse;
		m_bOvertakingInputSet		= tFalse;
		m_bUSFrontLeftInputSet		= tFalse;
		//m_bUSLeftInputSet			= tFalse;

		m_bBreaklightOutputSet		= tFalse;

	}
	
	RETURN_NOERROR;
}
 


tResult cAdaptiveCruiseControl::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{

	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		tTimeStamp InputTimeStamp;
		InputTimeStamp = pMediaSample->GetTime();

		if (pSource == &m_iDepthimagePin){

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
					LOG_ERROR("No Bitmap information found on pin \"input\"");
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
			}

			trackTheCar(pMediaSample, InputTimeStamp);
		}
		else if (pSource == &m_iStateACC){
			
			setACCState(pMediaSample);
		}		
		else if (pSource == &m_iSteeringAngle){
			processSteeringAngle(pMediaSample);
		}
		else if (pSource == &m_iCarSpeed){

			processVelocity(pMediaSample);
		}
		else if (pSource == &m_iCurrentLanetype){
				
			processLanetype(pMediaSample);
		}
		else if (pSource == &m_iOvertakingAllowed){
			
			processOvertakingAllowed(pMediaSample);
		}
		else if (m_startLookingForContraflow){
			/*if (pSource == &m_iUSSensorLeft){
				processUSLeft(pMediaSample);
			}*/
			if (pSource == &m_iUSSensorFrontLeft){
				processUSFrontLeft(pMediaSample);
			}
		    else if(pSource == &m_iVideoInputPin)
		    {
		        //Videoformat
		        if (m_bFirstFrame)
		        {        
		            cObjectPtr<IMediaType> pType;
		            RETURN_IF_FAILED(m_iVideoInputPin.GetMediaType(&pType));
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
		            m_bFirstFrame = tFalse;
				}	

		        checkLongStraightLane(pMediaSample, InputTimeStamp);
		    }
		}
		
	}

	RETURN_NOERROR;
}

/*
 * Method to activate/deactivate the ACC
 */
tResult cAdaptiveCruiseControl::setACCState(IMediaSample* pMediaSample){
	
	tBool bValue = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolStateACCInput, pMediaSample, pCoder);    

		if (!m_bStateACCInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStateACCInput);		
			m_bStateACCInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueStateACCInput, (tVoid*)&bValue);         
	}

	m_ACCActive = bValue;

	resetValues();

	RETURN_NOERROR;
}


/*
 * This method processes an incoming change of the lanetype
 */
tResult cAdaptiveCruiseControl::processLanetype(IMediaSample* pMediaSample){
	
	tInt32 lanetype = UNKNOWN_LANE;

	{
		__adtf_sample_read_lock_mediadescription(m_pCurrentLanetypeInput, pMediaSample, pCoder);    

		if (!m_bLanetypeInputSet){
			pCoder->GetID("intValue", m_szIDLanetypeInput);		
			m_bLanetypeInputSet = tTrue;
		}

		pCoder->Get(m_szIDLanetypeInput, (tVoid*)&lanetype);         
	}
/*
	fstream fff;
	fff.open("ACC_Lanetype", ios::out|ios::app);
	fff << lanetype << "\n";
	fff.close();  */

	// Ignore unknown lanetype	
	m_Lanetype = 	(lanetype == UNKNOWN_LANE) ? m_Lanetype : lanetype;

	RETURN_NOERROR;
}

/*
 * This method filters the current steering angle
 */
tResult cAdaptiveCruiseControl::processSteeringAngle(IMediaSample* pMediaSample){

	tFloat32 fSteering = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pSignalSteeringAngleInput, pMediaSample, pCoderOutput);

		if (!m_bSteeringAngleInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueSteeringAngleInput);
			m_bSteeringAngleInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueSteeringAngleInput, (tVoid*)&(fSteering));
	}

	m_steeringBuffer.push_back(fSteering);

	int bufferSize 	= m_steeringBuffer.size();

	// Compare 10 last elements
	if (bufferSize > 10){
		m_steeringBuffer.pop_front();
		bufferSize--;
	}

	// Set actual steering as mean of last measures
	tFloat32 buffer = 0.0;
	for (int i = 0; i < bufferSize; i++){
		buffer 	+= m_steeringBuffer.at(i);
	}

	m_currentSteering 	= buffer/bufferSize;

	RETURN_NOERROR;
}

/*
 * Process incoming Velocity of the lane following module
 */
tResult cAdaptiveCruiseControl::processVelocity(IMediaSample* pMediaSample){

	tFloat32 fSpeed = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pSignalCarSpeedInput,pMediaSample, pCoderOutput);
		
		if (!m_bCarSpeedInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueCarSpeedInput);
			m_bCarSpeedInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueCarSpeedInput, (tVoid*)&(fSpeed));
	}

	if (fSpeed >= 0.0 && fSpeed < 5.0){

		m_referenceSpeed 	= fSpeed;
	}else{
		LOG_WARNING(cString::Format("ACC: Invalid speed input from lane following: %f", fSpeed) );
	}

	if (!m_ACCActive) 	setVelocity(m_referenceSpeed);
	else 				setVelocity(speedControl());

	RETURN_NOERROR;
}

/*
 * Method to process the "Overtaking allowed mode" from lane following
 */
tResult cAdaptiveCruiseControl::processOvertakingAllowed(IMediaSample* pMediaSample){
	
	{
		__adtf_sample_read_lock_mediadescription(m_pOvertakingInput ,pMediaSample, pCoder);    

		if (!m_bOvertakingInputSet){
			pCoder->GetID("RoadSign", m_szIDOvertakingRoadSignInput);			
			pCoder->GetID("Maneuvre", m_szIDOvertakingManeuvreInput);
			pCoder->GetID("Pedestrain", m_szIDOvertakingPedestrainInput);
			m_bOvertakingInputSet = tTrue;
		}

		pCoder->Get( m_szIDOvertakingRoadSignInput, (tVoid*)&m_OvertakingAlowed_RoadSign);
		pCoder->Get( m_szIDOvertakingManeuvreInput, (tVoid*)&m_OvertakingAlowed_Maneuvre);
		pCoder->Get( m_szIDOvertakingPedestrainInput, (tVoid*)&m_OvertakingAlowed_Pedestrain);
	}

	// TODO: Delete this!
	/*m_OvertakingAlowed_RoadSign 	= tFalse;
	m_OvertakingAlowed_Maneuvre		= tFalse;
	m_OvertakingAlowed_Pedestrain 	= tFalse;*/

	//LOG_INFO(cString::Format("ACC: Received OvertakingAllowed Signal: %i | %i | %i", m_OvertakingAlowed_RoadSign, m_OvertakingAlowed_Maneuvre, m_OvertakingAlowed_Pedestrain) );
	
	RETURN_NOERROR;
}

/*
 * Method to check if there is an obstacle beside us, before we start
 * overtaking
 */
/*tResult cAdaptiveCruiseControl::processUSLeft(IMediaSample* pMediaSample){

	tFloat32 distance = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pUSLeftInput, pMediaSample, pCoderOutput);

		if (!m_bUSLeftInputSet){
			pCoderOutput->GetID("f32Value", m_szIDUSLeftInput);
			m_bUSLeftInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDUSLeftInput, (tVoid*)&(distance));
	}
	//LOG_INFO(cString::Format("DR: US_Distance = %f", distance) );

	m_USLeftBuffer.push_back(distance);

	size_t bufferSize 	= m_USLeftBuffer.size();

	// Compare at least 10 elements
	if (bufferSize < BUFFER_MAX_SIZE){
		RETURN_NOERROR;
	}
	
	// Compare maximal the 10 last elements
	else if (bufferSize > BUFFER_MAX_SIZE){
		m_USLeftBuffer.pop_front();
		bufferSize--;
	}

	// Calcluate three smalest value:
	tFloat32 smallElements[SET_ELEMENTS] 	= {500, 500, 500};
	for (size_t i = 0; i < bufferSize; i++){
		for (int j = 0; j < SET_ELEMENTS; j++){
			if(m_USLeftBuffer.at(i) < smallElements[j]){
				switch (j){
					case 0:
						smallElements[j+1]	= smallElements[j];
						smallElements[j] 	= m_USLeftBuffer.at(i);
						break;

					case 1:
						smallElements[j+1]	= smallElements[j];
						smallElements[j] 	= m_USLeftBuffer.at(i);
						break;

					case 2:
						smallElements[j] 	= m_USLeftBuffer.at(i);
						break;										
				}
			}
		}
	}
	
	// Calculate mean
	tFloat32 buffer 	= 0.0;
	for (int j = 0; j < SET_ELEMENTS; j++){
		buffer += smallElements[j];
	}

	m_USSideBufferValue 	= buffer / SET_ELEMENTS;
	
	// Set USLeftFlag
	m_LeftSideFree 	= m_USSideBufferValue < 0.4 ? tFalse : tTrue;
	
	RETURN_NOERROR;
}*/

/*
 * Method to check if there is an obstacle beside us, before we start
 * overtaking
 */
tResult cAdaptiveCruiseControl::processUSFrontLeft(IMediaSample* pMediaSample){

	tFloat32 distance = 0.0;

	{
		__adtf_sample_read_lock_mediadescription(m_pUSFrontLeftInput ,pMediaSample, pCoder);    

		if (!m_bUSFrontLeftInputSet){
			pCoder->GetID("f32Value", m_szIDUSFrontLeftInput);		
			m_bUSFrontLeftInputSet = tTrue;
		}

		pCoder->Get(m_szIDUSFrontLeftInput, (tVoid*)&distance);         
	}

	m_USFrontLeftBuffer.push_back(distance);

	size_t bufferSize 	= m_USFrontLeftBuffer.size();

	// Compare at least 10 elements
	if (bufferSize < BUFFER_MAX_SIZE){
		RETURN_NOERROR;
	}

	// Compare maximal the 10 last elements
	else if (bufferSize > BUFFER_MAX_SIZE){
		m_USFrontLeftBuffer.pop_front();
		bufferSize--;
	}

	// Calcluate three smalest value:
	tFloat32 smallElements[SET_ELEMENTS] 	= {500, 500, 500};
	for (size_t i = 0; i < bufferSize; i++){
		for (int j = 0; j < SET_ELEMENTS; j++){
			if(m_USFrontLeftBuffer.at(i) < smallElements[j]){
				switch (j){
					case 0:
						smallElements[j+1]	= smallElements[j];
						smallElements[j] 	= m_USFrontLeftBuffer.at(i);
						break;

					case 1:
						smallElements[j+1]	= smallElements[j];
						smallElements[j] 	= m_USFrontLeftBuffer.at(i);
						break;

					case 2:
						smallElements[j] 	= m_USFrontLeftBuffer.at(i);
						break;										
				}
			}
		}
	}
	
	// Calculate mean
	tFloat32 buffer 	= 0.0;
	for (int j = 0; j < SET_ELEMENTS; j++){
		buffer += smallElements[j];
	}

	m_USFrontBufferValue 	= buffer / SET_ELEMENTS;

	// Set USFrontLeftFlag
	m_LeftFrontFree 	= m_USFrontBufferValue < 0.5 ? tFalse : tTrue;
	
	RETURN_NOERROR;
}

/*
 * Check if there is a curve or crossing before we overtake
 */
tResult cAdaptiveCruiseControl::checkLongStraightLane(IMediaSample* pSample, tTimeStamp tsInputTime){

	const tVoid* l_pSrcBuffer;
	
	// Get image out of mediaSample
	IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
   	oImg->imageData = (char*)l_pSrcBuffer;
  	Mat image_new(cvarrToMat(oImg));
   	cvReleaseImage(&oImg);
   	pSample->Unlock(l_pSrcBuffer);

	m_lines.clear();
	m_lines_corr.clear();

	Mat matCutHough = image_new(cv::Range(HOUGH_CUT_Y1, HOUGH_CUT_Y2), cv::Range(HOUGH_CUT_X1, HOUGH_CUT_X2)).clone(); //Cut Image 
    cvtColor(matCutHough, matCutHough ,CV_RGB2GRAY);// Grey Image 	
	threshold(matCutHough, matCutHough, 45, 255,THRESH_TOZERO); // delete dark noise
	matCutHough = matCutHough-45;
	/*fstream f3;
	f3.open("thres.dat",ios::out);
	f3 << matCutHough << "\n";
	f3.close();	*/	
	normalize(matCutHough,matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
	convertScaleAbs(matCutHough, matCutHough); // Konveriere in 8Bit array. Nehme aus abs
	/*fstream f2;
	f2.open("norm.dat",ios::out);
	f2 << matCutHough << "\n";
	f2.close();*/
    medianBlur(matCutHough, matCutHough,3); // reduce bright noise with edge-preserving filter
	/*fstream f4;
	f4.open("median.dat",ios::out);
	f4 << matCutHough << "\n";
	f4.close(); */
    Canny(matCutHough, matCutHough,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false); // edges
	/*fstream f5;
	f5.open("canny.dat",ios::out);
	f5 << matCutHough << "\n";
	f5.close(); */

	HoughLines(matCutHough,m_lines,1,CV_PI/180,m_nThresholdValueHough,0,0);

	for(size_t i=0; i<m_lines.size();i++)
		{
			if((m_lines[i][1]*180/M_PI<80) && (m_lines[i][1]*180/M_PI>60))
				{
					// HoughGroup1(mid line)
					m_lines_corr.push_back(m_lines[i]);	
				}

		}

	m_LongStraightLane 	= m_lines_corr.size()!=0 ? tTrue : tFalse;

	RETURN_NOERROR;
}

/*
 * Method that reads an incoming depth image an tracks obstacles within the lane
 */
tResult cAdaptiveCruiseControl::trackTheCar(IMediaSample* pSample, tTimeStamp tsInputTime){

	if (m_ACCActive){

		m_iframeCounter++;

		// dont use every frame
		if(m_iframeCounter == 2){

			m_iframeCounter 	= 0;
			//m_ACCActive 		= tFalse;
			m_foundSquares.clear();
			m_foundSquaresContraflow.clear();
		
			// VideoInput
			RETURN_IF_POINTER_NULL(pSample);

			const tVoid* l_pSrcBuffer;

			IplImage* oImg 		= cvCreateImage(cvSize(m_sInputFormatDepthimage.nWidth, m_sInputFormatDepthimage.nHeight), IPL_DEPTH_16U, 3);
			RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
			oImg->imageData 	= (char*)l_pSrcBuffer;
			Mat image(cvarrToMat(oImg));
			cvReleaseImage(&oImg);
			pSample->Unlock(l_pSrcBuffer);

			Mat depthImage 		= Mat(m_sInputFormatDepthimage.nHeight,m_sInputFormatDepthimage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthimage.nBytesPerLine);

			tInt imageCutDown 	= static_cast<tInt>(IMAGE_CUT_DOWN);
			tInt imageCutUp 	= static_cast<tInt>(IMAGE_CUT_UP);
			tInt imageCutLeft 	= static_cast<tInt>(IMAGE_CUT_LEFT);
			tInt imageCutRight 	= static_cast<tInt>(IMAGE_CUT_RIGHT);
			
			m_ImageDepth		= depthImage(cv::Range(imageCutUp, imageCutDown), cv::Range(imageCutLeft, imageCutRight)).clone(); //Cut Image

			m_ImageZ			= Mat::zeros(imageCutDown-imageCutUp, imageCutRight-imageCutLeft, CV_32FC1);
			m_ImageCF			= Mat::zeros(imageCutDown-imageCutUp, imageCutRight-imageCutLeft, CV_32FC1);

			//m_BinImage 			= Mat::zeros(imageCutDown-imageCutUp, imageCutRight-imageCutLeft, CV_8UC1);

			// Get World Coordinates of each point	
			//fstream f;
			//f.open("AAC_Points.dat",ios::out);

			// Calculations with respect to the current lanetype

			// if we have very small steering angles, than we probably are at a straight lane
			if (m_currentSteering > MIDDLE_STEERING_MIN && m_currentSteering < MIDDLE_STEERING_MAX){
				m_Lanetype = LANE_STRAIGHT;
			}		

			// Straight lane
			if (m_Lanetype == LANE_STRAIGHT){
				for (int i = 0; i < imageCutDown-imageCutUp; i++){
					for (int j = 0; j < imageCutRight-imageCutLeft; j++){
				
						Point3f point 		= Util::ComputeWorldCoordinate(j*2, i*2, m_ImageDepth.at<ushort>(i, j), imageCutUp*2, imageCutLeft*2);					
				
						// Save critical points
						if (point.x < 0.2 && point.x > -0.10 && point.y < 0.15 && point.y > -0.1 && point.z < 1.5 && point.z > 0.3){

							//m_BinImage.at<uchar>(i,j) 	= 1;
							m_ImageZ.at<float>(i,j) 	= point.z;
						}

						// Look for contraflow
						if (m_startLookingForContraflow && point.x < -0.32 && point.x > -0.70 && point.y < 0.15 && point.y > -0.1 && point.z < 4 && point.z > 0.25){
							m_ImageCF.at<float>(i,j) 	= point.z;
						}
					}			
				}
			}

			// Curve Right direction
			else if (m_Lanetype == LANE_RIGHT){

				for (int i = 0; i < imageCutDown-imageCutUp; i++){
					for (int j = 0; j < imageCutRight-imageCutLeft; j++){
				
						Point3f point 		= Util::ComputeWorldCoordinate(j*2, i*2, m_ImageDepth.at<ushort>(i, j), imageCutUp*2, imageCutLeft*2);					
				
						// possible points on curve
						if (point.y < 0.15 && point.y > -0.1 && point.z > 0.3 && point.x > -0.2 ){

							// Calculate radius dependend on steering angle
							
							float innerRadius 	= RADIUS_RI; 	// Average default values
							float outerRadius 	= RADIUS_RO;
							if (m_currentSteering >= UPPER_STEERING){
								innerRadius  = RADIUS_SRI;
								outerRadius  = RADIUS_SRO;								
							} 
							else if (m_currentSteering >= MIDDLE_STEERING_MAX){
								innerRadius  = ((float)(RADIUS_BRI-RADIUS_SRI))/((float)(UPPER_STEERING-MIDDLE_STEERING_MAX)) * (UPPER_STEERING-m_currentSteering) + RADIUS_SRI;
								outerRadius  = ((float)(RADIUS_BRO-RADIUS_SRO))/((float)(UPPER_STEERING-MIDDLE_STEERING_MAX)) * (UPPER_STEERING-m_currentSteering) + RADIUS_SRO;
							}

							float xSmallBound 	= -0.2 + outerRadius - cos(asin(point.z/outerRadius))*outerRadius;
							float xHighBound 	= 0.25 + innerRadius - cos(asin(point.z/innerRadius))*innerRadius;

							if (point.x > xSmallBound && point.x < xHighBound){
								//m_BinImage.at<uchar>(i,j) 	= 1;
								m_ImageZ.at<float>(i,j) 	= sqrt(point.x*point.x + point.z*point.z); // Norm within x-z-plane
							}

							//LOG_INFO(cString::Format("ACC: RightCurveRADII: %f | %f", innerRadius, outerRadius));
						}

						// Look for contraflow
						if (m_startLookingForContraflow && point.x < -0.32 && point.x > -0.70 && point.y < 0.15 && point.y > -0.1 && point.z < 4 && point.z > 0.25){
							m_ImageCF.at<float>(i,j) 	= point.z;
						}
					}			
				}
			}

			// Curve Left direction
			else if (m_Lanetype == LANE_LEFT){

				for (int i = 0; i < imageCutDown-imageCutUp; i++){
					for (int j = 0; j < imageCutRight-imageCutLeft; j++){
				
						Point3f point 		= Util::ComputeWorldCoordinate(j*2, i*2, m_ImageDepth.at<ushort>(i, j), imageCutUp*2, imageCutLeft*2);					
				
						// possible points on curve
						if (point.y < 0.15 && point.y > -0.1 && point.z > 0.3 && point.x < 0.25 ){

							// Calculate radius dependend on steering angle
							
							float innerRadius 	= RADIUS_LI; 	// Average default values
							float outerRadius 	= RADIUS_LO;
							if (m_currentSteering <= LOWER_STEERING){
								innerRadius  = RADIUS_SLI;
								outerRadius  = RADIUS_SLO;								
							} 
							else if (m_currentSteering <= MIDDLE_STEERING_MIN){
								innerRadius  = ((float)(RADIUS_BLI-RADIUS_SLI))/((float)(MIDDLE_STEERING_MIN-LOWER_STEERING)) * (m_currentSteering-LOWER_STEERING) + RADIUS_SLI;
								outerRadius  = ((float)(RADIUS_BLO-RADIUS_SLO))/((float)(MIDDLE_STEERING_MIN-LOWER_STEERING)) * (m_currentSteering-LOWER_STEERING) + RADIUS_SLO;
							}

							float xSmallBound 	= -0.2 - innerRadius + cos(asin(point.z/innerRadius))*innerRadius;
							float xHighBound 	= 0.25 - outerRadius + cos(asin(point.z/outerRadius))*outerRadius;

							if (point.x > xSmallBound && point.x < xHighBound){
								//m_BinImage.at<uchar>(i,j) 	= 1;
								m_ImageZ.at<float>(i,j) 	= sqrt(point.x*point.x + point.z*point.z); // Norm within x-z-plane
							}

							//LOG_INFO(cString::Format("ACC: LeftCurveRADII: %f | %f", innerRadius, outerRadius));
						}

						// Look for contraflow
						if (m_startLookingForContraflow && point.x < -0.32 && point.x > -0.70 && point.y < 0.15 && point.y > -0.1 && point.z < 4 && point.z > 0.25){
							m_ImageCF.at<float>(i,j) 	= point.z;
						}
					}			
				}
			}


			//f << m_BinImage;

			//f.close();

			/*fstream f2;
			f2.open("ACC_Distances",ios::out);
			f2 << m_ImageZ;
			f2.close();			*/		

			m_currDistance  	= distanceToEnemy(imageCutDown-imageCutUp, imageCutRight-imageCutLeft);

			m_noContraflow 		= m_startLookingForContraflow ? checkContraflow(imageCutDown-imageCutUp, imageCutRight-imageCutLeft) : tFalse;

/*
			if (m_currDistance < 1e6){
				LOG_INFO(cString::Format("ACC: Distance: %f", m_currDistance) );
			}*/

			CreateAndTransmitGCL();

		}

	}

	RETURN_NOERROR;
}

/*
 * Find distance to a car in front of us
 */
tFloat32 cAdaptiveCruiseControl::distanceToEnemy(int height, int width)
{

	// Ignore some of the values at borders if the following does not fit
	int squareHeight 	= floor(height/NUMBER_SQUARES);
	int squareWidth 	= floor(width/NUMBER_SQUARES);

	float maxHits 		= squareHeight*squareWidth;

	vector<Point3f> verifiedSquares;
	
	// Calclulate squares width a lot of "1" in it and the corresponding minimal Distance
	for (size_t i = 0; i < NUMBER_SQUARES; i++){
		for (size_t j = 0; j < NUMBER_SQUARES; j++){

			//Mat tmpMat 		= m_BinImage( cv::Range(i*squareHeight, (i+1)*squareHeight), cv::Range(j*squareWidth, (j+1)*squareWidth) ).clone(); //Cut Image
			Mat tmpDist 	= m_ImageZ  ( cv::Range(i*squareHeight, (i+1)*squareHeight), cv::Range(j*squareWidth, (j+1)*squareWidth) ).clone();
			
			int tmpHits 	= 0;

			float minDist 	= 1e6;
			float meanDist 	= 0.0;

			for (int ii = 0; ii < squareHeight; ii++){
				for (int jj = 0; jj < squareWidth; jj++){
			
					if (tmpDist.at<float>(ii,jj) > 0){

						float aktDist 	= tmpDist.at<float>(ii,jj);
	
						tmpHits++;
						minDist 		=  aktDist < minDist ? aktDist : minDist;
						meanDist 		+= aktDist;

						//if(aktDist == 0) LOG_WARNING(cString::Format("ACC: Zero-Distance not allowed here!") );
					}
				}
			}

			meanDist 	/= tmpHits;

			// Filter possible wrong values
			float dist = meanDist-minDist < 0.05 ? minDist : meanDist;

			if ( (float)tmpHits/maxHits > FILLING )	{

				// Store to plot with GCL
				m_foundSquares.push_back((j*squareWidth+IMAGE_CUT_LEFT)*2);
				m_foundSquares.push_back((i*squareHeight+IMAGE_CUT_UP)*2);		
				m_foundSquares.push_back(((j+1)*squareWidth+IMAGE_CUT_LEFT)*2);		
				m_foundSquares.push_back(((i+1)*squareHeight+IMAGE_CUT_UP)*2);

				verifiedSquares.push_back( Point3f(i,j,dist) );
			}	 	
		}
	}

	float resultDistance 	= 1e6;	

	// Compare the verified squares
	for (size_t i = 0; i < verifiedSquares.size(); i++){

		// ATM: Take the absolute minimum distance of all squares
		resultDistance  	= verifiedSquares[i].z < resultDistance ? verifiedSquares[i].z : resultDistance;
	}

	return resultDistance;
	
}

/*
 * Check whethere there is a contraflow (so we better should not start to overtake) or not
 */
tBool cAdaptiveCruiseControl::checkContraflow(int height, int width){

	tBool noContraflow 	= tFalse;

	// Ignore some of the values at borders if the following does not fit
	int squareHeight 	= floor(height/NUMBER_SQUARES_CONTRAFLOW);
	int squareWidth 	= floor(width/NUMBER_SQUARES_CONTRAFLOW);

	float maxHits 		= squareHeight*squareWidth;

	vector<Point3f> verifiedSquares;
	
	// Calclulate squares width a lot of "1" in it and the corresponding minimal Distance
	for (size_t i = 0; i < NUMBER_SQUARES_CONTRAFLOW; i++){
		for (size_t j = 0; j < NUMBER_SQUARES_CONTRAFLOW; j++){

			Mat tmpDist 	= m_ImageCF( cv::Range(i*squareHeight, (i+1)*squareHeight), cv::Range(j*squareWidth, (j+1)*squareWidth) ).clone();
			
			int tmpHits 	= 0;

			for (int ii = 0; ii < squareHeight; ii++){
				for (int jj = 0; jj < squareWidth; jj++){
			
					if (tmpDist.at<float>(ii,jj) > 0){
	
						tmpHits++;
					}
				}
			}

			if ( (float)tmpHits/maxHits > FILLING )	{

				// Store to plot with GCL
				m_foundSquaresContraflow.push_back((j*squareWidth+IMAGE_CUT_LEFT)*2);
				m_foundSquaresContraflow.push_back((i*squareHeight+IMAGE_CUT_UP)*2);		
				m_foundSquaresContraflow.push_back(((j+1)*squareWidth+IMAGE_CUT_LEFT)*2);		
				m_foundSquaresContraflow.push_back(((i+1)*squareHeight+IMAGE_CUT_UP)*2);

			}	 	
		}
	}

	m_ContraflowBuffer.push_back((tFloat32)m_foundSquaresContraflow.size());

	size_t bufferSize 	= m_ContraflowBuffer.size();

	// Compare maximal the 50 last elements
	if (bufferSize > CONTRAFLOW_BUFFER_MAX_SIZE){
		m_ContraflowBuffer.pop_front();
		bufferSize--;
	}

	// Calculate variation
	if (bufferSize == CONTRAFLOW_BUFFER_MAX_SIZE){

		tFloat32 mean 		= 0.0;
		tFloat32 mean_sq 	= 0.0;
		for (size_t i = 0; i < m_ContraflowBuffer.size(); i++){
			mean 	+= m_ContraflowBuffer.at(i);
			mean_sq += m_ContraflowBuffer.at(i)*m_ContraflowBuffer.at(i);
		}
		mean 	= (mean/bufferSize)*(mean/bufferSize);
		mean_sq = mean_sq/bufferSize;
		m_derivation 	= mean_sq - mean > 0 ? mean_sq - mean : mean - mean_sq; 

	}

	// Found Contraflow 						// no movement in found obstacle
	if ( m_foundSquaresContraflow.empty() || (m_derivation > 0 && m_derivation < 100) ){
			//LOG_INFO(cString::Format("ACC: Contraflow = %i | %i", m_foundSquaresContraflow.empty(), (m_derivation > 0 && m_derivation < 100) ) );
			noContraflow 	= tTrue;
	}

	return noContraflow;
}

/*
 * Calculate the output speed
 */
tFloat32 cAdaptiveCruiseControl::speedControl()
{

	tFloat32 outputVelocity 	= m_referenceSpeed;

	if (m_currDistance < CRITICAL_DISTANCE){

		outputVelocity	= 0.0;

	}else if(m_currDistance < UNCRITICAL_DISTANCE){

		outputVelocity 	= m_referenceSpeed/((float)(UNCRITICAL_DISTANCE-CRITICAL_DISTANCE)) * ((float)(m_currDistance-CRITICAL_DISTANCE));
	}

	if (outputVelocity < 0.2 && m_referenceSpeed > 0.2){
		m_standingCounter++;
	}
	else m_standingCounter = 0;
	
	// Overtake driving and standig obstacles
	if (m_OvertakingAlowed_RoadSign && m_OvertakingAlowed_Maneuvre && m_OvertakingAlowed_Pedestrain){// && m_Lanetype == LANE_STRAIGHT && outputVelocity < m_referenceSpeed/2){
		if (m_Lanetype == LANE_STRAIGHT && m_standingCounter >= STANDING_THRESHOLD){ //outputVelocity < m_referenceSpeed/3){ //2){
			if (m_startLookingForContraflow){
				if (m_noContraflow && m_LeftFrontFree && m_LongStraightLane){ // && m_LeftSideFree){
					resetValues();
					startOvertaking(tTrue);
					return -0.1;
				}
			}
			else m_startLookingForContraflow 	= tTrue;
		}
		/*if (outputVelocity <= 0.2 && m_referenceSpeed > 0.2){
			if (m_startLookingForContraflow){
				if (m_noContraflow && m_LeftFrontFree){ // && m_LeftSideFree){ // && m_LongStraightLane){
					resetValues();
					startOvertaking(tTrue);
					return -0.1;
				}
			}
			else m_startLookingForContraflow 	= tTrue;		
		}*/
	}else resetValues();
		
	
	// if velocity is to small, than the motor can't handle it
	outputVelocity 	= (outputVelocity < 0.2) ? 0.0 : outputVelocity;

	// Set breaklight signal
	if (outputVelocity == 0.0 && m_referenceSpeed > 0.0 && !m_breaking){
	
		m_breaking 	= tTrue;
		TransmitBreaklightValue(tTrue);
	} else if (outputVelocity > 0.0 && m_breaking){

		m_breaking 	= tFalse;
		TransmitBreaklightValue(tFalse);
	}


	return outputVelocity;
}

tResult cAdaptiveCruiseControl::resetValues(){

	m_startLookingForContraflow 	= tFalse;
	m_noContraflow 					= tFalse;
	m_LeftFrontFree					= tFalse;	
	//m_LeftSideFree 					= tFalse;
	m_LongStraightLane 				= tFalse;
	//m_USLeftBuffer.clear();
	m_USFrontLeftBuffer.clear();
	m_ContraflowBuffer.clear();
	m_USFrontBufferValue			= 0.0;
	m_USSideBufferValue				= 0.0;
	m_standingCounter 				= 0;

	RETURN_NOERROR;
}

/*
 * Method to transmit the output velocity
 */ 
tResult cAdaptiveCruiseControl::setVelocity(tFloat32 speed){

	if (speed >= 0.0){	

		tUInt32 nTimeStamp = 0;

		cObjectPtr<IMediaSample> pNewSample;
		AllocMediaSample((tVoid**)&pNewSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pSignalSpeedOutput->GetMediaSampleSerializer(&pSerializer);
		pNewSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pSignalSpeedOutput, pNewSample, pCoderOutput); 
		
			if (!m_bSpeedOutputSet){
				pCoderOutput->GetID("f32Value", m_szIDSignalValueSpeedOutput);
				pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampSpeedOutput);
				m_bSpeedOutputSet = tTrue;
			}
		
			pCoderOutput->Set(m_szIDSignalValueSpeedOutput, (tVoid*)&(speed));
			pCoderOutput->Set(m_szIDTimestampSpeedOutput, (tVoid*)&nTimeStamp);
	
		}

		pNewSample->SetTime(pNewSample->GetTime());
		m_oCarSpeed.Transmit(pNewSample);

	}

	RETURN_NOERROR;
}

/*
 * This method sends the reference velocity to the DriveRound module to start overtaking
 */
tResult cAdaptiveCruiseControl::startOvertaking(tBool state){
	
	cObjectPtr<IMediaSample> pNewSample;
	AllocMediaSample((tVoid**)&pNewSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStateDROutput->GetMediaSampleSerializer(&pSerializer);
	pNewSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pStateDROutput, pNewSample, pCoderOutput); 
		
		if (!m_bStateDROutputSet){
			pCoderOutput->GetID("bValue", m_szIDStateDROutput);
			m_bStateDROutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDStateDROutput, (tVoid*)&(state));
	
	}

	pNewSample->SetTime(pNewSample->GetTime());
	m_oStartDriveRound.Transmit(pNewSample);

	RETURN_NOERROR;
}

/*
 * this function sends the breakinglight signal to the baseconfig.
 */
tResult cAdaptiveCruiseControl::TransmitBreaklightValue(tBool bValue){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBreaklightOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = 0;

	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(m_pBreaklightOutput, pMediaSample, pCoderOutput);    

		// set the id if not already done
		if(!m_bBreaklightOutputSet)
		{
			pCoderOutput->GetID("bValue", m_szIDBoolValueBreaklightOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampBreaklightOutput);
			m_bBreaklightOutputSet = tTrue;
		}      

		// set value from sample
		pCoderOutput->Set(m_szIDBoolValueBreaklightOutput, (tVoid*)&bValue);     
		pCoderOutput->Set(m_szIDTimestampBreaklightOutput, (tVoid*)&ui32TimeStamp);     
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	//transmit media sample over output pin
	m_oBreaklight.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * GCL-Debug Output
 */
tResult cAdaptiveCruiseControl::CreateAndTransmitGCL()
{

    // just draw gcl if the pin is connected
    if (!m_oGCLOutput.IsConnected())
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(1024*1024));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;

    // draw near and far area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 100, 255).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(IMAGE_CUT_LEFT*2) , static_cast<tInt16>(IMAGE_CUT_UP*2), static_cast<tInt16>(IMAGE_CUT_RIGHT*2), static_cast<tInt16>(IMAGE_CUT_DOWN*2));

	// Plot found squares
	for (size_t i = 0; i < m_foundSquares.size()/4; i++){

		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_foundSquares[i*4+0] , m_foundSquares[i*4+1], m_foundSquares[i*4+2], m_foundSquares[i*4+3]);
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());

	// Plot found Contraflow squares
	for (size_t i = 0; i < m_foundSquaresContraflow.size()/4; i++){

		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, m_foundSquaresContraflow[i*4+0] , m_foundSquaresContraflow[i*4+1], m_foundSquaresContraflow[i*4+2], m_foundSquaresContraflow[i*4+3]);
	}

	// Plot Lanetype Output
	cString strText = cString::FromInt32(m_Lanetype);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	cString strText2 = cString::FromBool(m_noContraflow);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 40, strText2.GetLength());
	cGCLWriter::StoreData(pc, strText2.GetLength(), strText2.GetPtr()); 

	cString strText3 = cString::FromBool(m_LeftFrontFree);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 60, strText3.GetLength());
	cGCLWriter::StoreData(pc, strText3.GetLength(), strText3.GetPtr()); 

	tFloat64 text3 = m_USFrontBufferValue;
	cString strText33 = cString::FromFloat64(text3);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 70, strText33.GetLength());
	cGCLWriter::StoreData(pc, strText33.GetLength(), strText33.GetPtr()); 

	/*cString strText4 = cString::FromBool(m_LeftSideFree);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 90, strText4.GetLength());
	cGCLWriter::StoreData(pc, strText4.GetLength(), strText4.GetPtr()); 

	tFloat64 text4 = m_USSideBufferValue;
	cString strText44 = cString::FromFloat64(text4);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 100, strText44.GetLength());
	cGCLWriter::StoreData(pc, strText44.GetLength(), strText44.GetPtr());*/ 

	cString strText5 = cString::FromBool(m_LongStraightLane);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 90, strText5.GetLength());
	cGCLWriter::StoreData(pc, strText5.GetLength(), strText5.GetPtr()); 

	tFloat64 text9 = m_derivation;
	cString strText9 = cString::FromFloat64(text9);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 110, strText9.GetLength());
	cGCLWriter::StoreData(pc, strText9.GetLength(), strText9.GetPtr()); 

	cString strText10 = cString::FromInt32(m_standingCounter);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 130, strText10.GetLength());
	cGCLWriter::StoreData(pc, strText10.GetLength(), strText10.GetPtr()); 

	cString strTexta = cString::FromBool(m_OvertakingAlowed_RoadSign);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 300, strTexta.GetLength());
	cGCLWriter::StoreData(pc, strTexta.GetLength(), strTexta.GetPtr()); 

	cString strTextb = cString::FromBool(m_OvertakingAlowed_Maneuvre);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 320, strTextb.GetLength());
	cGCLWriter::StoreData(pc, strTextb.GetLength(), strTextb.GetPtr()); 

	cString strTextc = cString::FromBool(m_OvertakingAlowed_Pedestrain);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 340, strTextc.GetLength());
	cGCLWriter::StoreData(pc, strTextc.GetLength(), strTextc.GetPtr()); 




	// Plot Hough lines in order to detect if there is enough place to overtake

 	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(160, 255, 255).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, static_cast<tInt16>(HOUGH_CUT_X1) , static_cast<tInt16>(HOUGH_CUT_Y1), static_cast<tInt16>(HOUGH_CUT_X2), static_cast<tInt16>(HOUGH_CUT_Y2));

	for(size_t i=0; i<m_lines_corr.size();i++)
	{
		float rho = m_lines_corr[i][0];
		float theta = m_lines_corr[i][1];
		Point pt1,pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+HOUGH_CUT_Y1, pt1.x, pt1.y+HOUGH_CUT_Y1);
	}

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    
    RETURN_NOERROR;

}
