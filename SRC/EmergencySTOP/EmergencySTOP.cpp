/*
 * Date 17.03.2016
 */

#include "stdafx.h"
#include "EmergencySTOP.h"
#include <fstream>

#define FREE_BUFFER 3

// Create filter shell
ADTF_FILTER_PLUGIN("EmergencySTOP", OID_ADTF_Emergency_STOP, EmergencySTOP);

#define PROP_MIN_DISTANCE "Minimal Distance"

EmergencySTOP::EmergencySTOP(const tChar* __info) : cFilter(__info)
{
	//set booleans for all sensors and the breakinglight:
	SetPropertyFloat(PROP_MIN_DISTANCE, m_sProperties.fMinDistance);
	breakingModeFrontCenter = false;
	breakingModeFrontCenterRight = false;
	breakingModeFrontCenterLeft = false;
	breakingModeFrontRight = false;
	breakingModeFrontLeft = false;
	breakingModeRearCenter = false;
	breakingModeRearLeft = false;
	breakingModeRearRight = false;
	driveForward = tTrue;
	AngleID = 3;
	
	m_emergencyBreak 	= tFalse;
	m_stateChanged 		= tFalse;

}

EmergencySTOP::~EmergencySTOP()
{

}

tResult EmergencySTOP::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    																		// in StageFirst you can create and register your static pins.
			if (eStage == StageFirst)
			{
				// get a media type for the input pin
				cObjectPtr<IMediaDescriptionManager> pDescManager;
				RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager, __exception_ptr));

				// Get description for bool values
				tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
				RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
				cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

				// Get description for SignalValue
				tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");   
				RETURN_IF_POINTER_NULL(strDescSignalValue);    
				cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

				// Media Description Sensors
				tChar const * UltraSonicDesc = pDescManager->GetMediaDescription("tUltrasonicStruct");
				RETURN_IF_POINTER_NULL(UltraSonicDesc);
				cObjectPtr<IMediaType> pTypeUltraSonicDesc = new cMediaType(0, 0, 0, "tUltrasonicStruct", UltraSonicDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
				//create input pin for US
    			RETURN_IF_FAILED(m_iUltraSonic.Create("us_struct", pTypeUltraSonicDesc, static_cast<IPinEventSink*> (this)));
    			RETURN_IF_FAILED(RegisterPin(&m_iUltraSonic));
				RETURN_IF_FAILED(pTypeUltraSonicDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUltraSonicInput));

				// create Inputpin for ServoAngle (Input)
				RETURN_IF_FAILED(m_iServoAngle.Create("ServoAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_iServoAngle));
				RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalServoAngleInput));

				// create Inputpin for CarSpeed (Input)
				RETURN_IF_FAILED(m_iCarSpeed.Create("CarSpeed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_iCarSpeed));
				RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalCarSpeedInput));

				// create Inputpin for SpeedController (Input)
				RETURN_IF_FAILED(m_iSpeedController.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_iSpeedController));
				RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalSpeedControllerInput));

				// create Input Pin for StatusES
				RETURN_IF_FAILED(m_iStatusES.Create("Status", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_iStatusES));
				RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStatusESInput));

				// create Pin for Breaklight (Output)
				RETURN_IF_FAILED(m_oBreaklight.Create("Breaklight", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oBreaklight));
				RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBreaklightOutput));
				
				// create Pin for DecisionMaking (Output)
				RETURN_IF_FAILED(m_oDecisionMaking.Create("Decision_Making", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oDecisionMaking));
				RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDecisionMakingOutput));

				// Create Pin for Speed (Output)
				RETURN_IF_FAILED(m_oSpeedOut.Create("SpeedOut", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
				RETURN_IF_FAILED(RegisterPin(&m_oSpeedOut));
				RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalSpeedOutOutput));

			}
			else if (eStage == StageNormal)
			{
				// In this stage you would do further initialisation and/or create your dynamic pins.
				// Please take a look at the demo_dynamicpin example for further reference.
				m_sProperties.fMinDistance = GetPropertyFloat(PROP_MIN_DISTANCE);
			}
			else if (eStage == StageGraphReady)
			{
				// ID's not set yet

				m_bIDsUltraSonicSet 			= tFalse;
				m_bServoAngleInputSet			= tFalse;
				m_bCarSpeedInputSet				= tFalse;
				m_bSpeedControllerInputSet		= tFalse;
				m_bBreaklightOutputSet			= tFalse;
				m_bDecisionMakingOutputSet		= tFalse;
				m_bSpeedOutOutputSet			= tFalse;
				m_bStatusESSet 					= tFalse;
		
				// Activate EmergencySTOP at beginning
				m_ESActivated 					= tTrue;
			}

	RETURN_NOERROR;
}

tResult EmergencySTOP::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception: 
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult EmergencySTOP::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	// first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		if (&m_iStatusES == pSource){
			ProcessStatus(pMediaSample);
		}

		if (m_ESActivated){
			if (&m_iCarSpeed == pSource) {
				ProcessCarSpeed(pMediaSample);
			}
			if (&m_iUltraSonic == pSource){
				ProcessUltrasonicInput(pMediaSample);
			}
			if (&m_iServoAngle == pSource) {
				ProcessServoAngle(pMediaSample);
			}
	
			if ( &m_iSpeedController == pSource) {
				TransmitSpeed(pMediaSample);
			}
		}
	}

	RETURN_NOERROR;
}

/*
 * The function with the measured speed:
 */
tResult EmergencySTOP::ProcessCarSpeed(IMediaSample* pMediaSample) {

	tFloat32 fSpeed = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pSignalCarSpeedInput,pMediaSample, pCoderOutput);
		
		if (!m_bCarSpeedInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueCarSpeedInput);
			m_bCarSpeedInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueCarSpeedInput, (tVoid*)&fSpeed); 		// Absturz?? (Andi)
	}

	////////////////////////////////////
	//decide if we drive forward or backward.fSpeed is not a global variabel...So we need another boolean:

	if (!breakingModeFrontCenter && !breakingModeFrontCenterLeft && !breakingModeFrontCenterRight && !breakingModeFrontLeft && !breakingModeFrontRight  && !breakingModeRearCenter && !breakingModeRearLeft && !breakingModeRearRight) {
		if (fSpeed < 0) {
			driveForward = false;
		} 
		else {
			driveForward = true;
		}
	}

	//define fMinDistance
	if(0.5*fSpeed*fSpeed < GetPropertyFloat(PROP_MIN_DISTANCE)){
		m_sProperties.fMinDistance = GetPropertyFloat(PROP_MIN_DISTANCE);
	}else{
		m_sProperties.fMinDistance = 0.5*fSpeed*fSpeed;
	}

	RETURN_NOERROR;
}

tResult EmergencySTOP::ProcessServoAngle(IMediaSample* pMediaSample) {
//RETURN_NOERROR; // Warnungen wegen 'unlocked description' von hier?! (andi)
	tFloat32 fAngle = 90;
	
	{
		__adtf_sample_read_lock_mediadescription(m_pSignalServoAngleInput,pMediaSample, pCoderOutput);

		if (!m_bServoAngleInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueServoAngleInput);
			m_bServoAngleInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueServoAngleInput, (tVoid*)&fAngle);
	}
	
	//divide the steeringangles into 5 parts for the front and into 3 parts for the rear and dustribute them to the single sensors.
	if(driveForward)
	{
		if (fAngle<65)//
		{
			AngleID=1;//SensorFrontLeft
		};

		if (fAngle<80 && fAngle >=65)
		{
			AngleID=2;//SensorFrontCenterLeft
		};

		if (fAngle<=100 && fAngle >=80)
		{
			AngleID=3;//SensorFrontCenter
		};
		if (fAngle<=115 && fAngle >100)
		{
			AngleID=4;//SensorFrontCenterRight
		};
		if (fAngle>115)
		{
			AngleID=5;//SensorFrontRight
		};
	}else{
		if (fAngle<75)//
		{
			AngleID=6;//SensorRearLeft
		};


		if (fAngle<=100 && fAngle >=80)
		{
			AngleID=7;//SensorRearCenter
		};

		if (fAngle>105)
		{
			AngleID=8;//SensorRearRight
		};
	}

	RETURN_NOERROR;

}

tResult EmergencySTOP::ProcessUltrasonicInput(IMediaSample* pMediaSample){

		tSignalValue frontLeft, frontCenterLeft, frontCenter, frontCenterRight, frontRight, rearLeft, rearCenter, rearRight;

		{
			__adtf_sample_read_lock_mediadescription(m_pUltraSonicInput, pMediaSample, pCoder);    
			
			if (!m_bIDsUltraSonicSet) {						  
				pCoder->GetID("tFrontLeft", m_szIDFrontLeftUltraSonicInput);
				pCoder->GetID("tFrontCenterLeft", m_szIDFrontCenterLeftUltraSonicInput);
				pCoder->GetID("tFrontCenter", m_szIDFrontCenterUltraSonicInput);
				pCoder->GetID("tFrontCenterRight", m_szIDFrontCenterRightUltraSonicInput);
				pCoder->GetID("tFrontRight", m_szIDFrontRightUltraSonicInput);
				pCoder->GetID("tRearLeft", m_szIDRearLeftUltraSonicInput);
				pCoder->GetID("tRearCenter", m_szIDRearCenterUltraSonicInput);
				pCoder->GetID("tRearRight", m_szIDRearRightUltraSonicInput);
				m_bIDsUltraSonicSet=tTrue;
			}

  			pCoder->Get(m_szIDFrontLeftUltraSonicInput, (tVoid*)&frontLeft); 
  			pCoder->Get(m_szIDFrontCenterLeftUltraSonicInput, (tVoid*)&frontCenterLeft);   
  			pCoder->Get(m_szIDFrontCenterUltraSonicInput, (tVoid*)&frontCenter);   
  			pCoder->Get(m_szIDFrontCenterRightUltraSonicInput, (tVoid*)&frontCenterRight);   
  			pCoder->Get(m_szIDFrontRightUltraSonicInput, (tVoid*)&frontRight);    
			pCoder->Get(m_szIDRearRightUltraSonicInput, (tVoid*)&rearRight);
			pCoder->Get(m_szIDRearCenterUltraSonicInput, (tVoid*)&rearCenter);
			pCoder->Get(m_szIDRearLeftUltraSonicInput, (tVoid*)&rearLeft);
		}

		// FrontLeft
		{
			tFloat32 fDistance 	= frontLeft.f32Value;

			if(AngleID == 1){

				//if the measured distance is less than the mindistance:set the breaking mode true.if the sensors doesnt see anything or if the barrier left set the breakingmode false.- 
				if (fDistance < m_sProperties.fMinDistance) {
						breakingModeFrontLeft = true;
				} else {
						breakingModeFrontLeft = false;
				}
			}
			//if the steeringangle is less than 80 but more or equal 65 check this sensor but just with 30% of the distance:
			if(AngleID == 2){
				if (fDistance < 0.3*m_sProperties.fMinDistance) {
					breakingModeFrontLeft = true;

				} else {
					breakingModeFrontLeft = false;
				}
			}
			if(AngleID == 3){
				//if the steeringangle is less than 100 but more or equal 80 check this sensor but just with a distance of 10cm to avoid sidecrahses:
				if (fDistance < 0.05) {
					breakingModeFrontLeft = true;
				} else {
					breakingModeFrontLeft = false;
				}
			}
		}

		// FrontCenterLeft
		{
			tFloat32 fDistance 	= frontCenterLeft.f32Value;

			if(AngleID == 2){
				if (fDistance < m_sProperties.fMinDistance) {
					breakingModeFrontCenterLeft = true;
				} else {
					breakingModeFrontCenterLeft = false;
				}
			}
			//if the steeringangle is less than 100 but more or equal 80 or less than 65 check this sensor but just with a distance of 30%:
			if(AngleID == 3 || AngleID == 1){
				if (fDistance < 0.3*m_sProperties.fMinDistance) {
					breakingModeFrontCenterLeft = true;
				} else {
					breakingModeFrontCenterLeft = false;
				}
			}
		}

		// FrontCenter
		{
			tFloat32 fDistance 	= frontCenter.f32Value;

			if(AngleID == 3){

				if (fDistance < m_sProperties.fMinDistance) {
					breakingModeFrontCenter = true;
				} else {
					breakingModeFrontCenter = false;
				}
			}
			//if the steeringangle is less than 115 but more or equal 100 or less than 80 and more or equal 65 check this sensor but just with a distance of 30%:
			if(AngleID == 2 || AngleID == 4){

				if (fDistance < 0.3*m_sProperties.fMinDistance) {
					breakingModeFrontCenter = true;
				} else {
					breakingModeFrontCenter = false;
				}
			}
		}

		// FrontCenterRight
		{
			tFloat32 fDistance 	= frontCenterRight.f32Value;
	
			if(AngleID == 4){

				if (fDistance < m_sProperties.fMinDistance) {
					breakingModeFrontCenterRight = true;

				} else {
					breakingModeFrontCenterRight = false;

				}
			}
			//if the steeringangle is less than 100 but more or equal 80 or more than 115 check this sensor but just with a distance of 30%:
			if(AngleID == 3 || AngleID == 5){

				if (fDistance < 0.3*m_sProperties.fMinDistance) {
					breakingModeFrontCenterRight = true;
				}else {
					breakingModeFrontCenterRight = false;
				}
			}
		}

		// FrontRight
		{
			tFloat32 fDistance 	= frontRight.f32Value;

			if(AngleID == 5){
				if (fDistance < m_sProperties.fMinDistance) {
					breakingModeFrontRight = true;
				} else {
					breakingModeFrontRight = false;
				}
			}
			//if the steeringangle is less than 115 but more or equal 110 check this sensor but just with 30% of the distance:
			if(AngleID == 4){

				if (fDistance < 0.3*m_sProperties.fMinDistance) {
					breakingModeFrontRight = true;
				} else {
					breakingModeFrontRight = false;
				}
			}
			//if the steeringangle is less than 100 but more or equal 80 check this sensor but just with a distance of 10cm to avoid sidecrahses:
			if(AngleID == 3){		

				if (fDistance < 0.05) {
					breakingModeFrontRight = true;
				} else {
					breakingModeFrontRight = false;
				}
			}
		}

		// RearLeft
		{
			tFloat32 fDistance 	= rearLeft.f32Value;			

			if (fDistance < m_sProperties.fMinDistance) {
				breakingModeRearLeft = true;
			} else {
				breakingModeRearLeft = false;
			}
		}

		// RearCenter
		{
			tFloat32 fDistance 	= rearCenter.f32Value;
			
			if (fDistance < m_sProperties.fMinDistance) {
				breakingModeRearCenter = true;
			} else {
				breakingModeRearCenter = false;
			}			
		}

		// RearRight
		{
			tFloat32 fDistance 	= rearRight.f32Value;
	
			if (fDistance < m_sProperties.fMinDistance) {
				breakingModeRearRight = true;
			} else {
				breakingModeRearRight = false;
			}	
		}

	RETURN_NOERROR;
}


/*
 * this is the function where we set the speed.
 * if one of the breakingmodes of the frontsensors is set to true AND if the car is driving forwards->set the speed to 0! 
 * if one of the breakingmodes of the rearsensors is set to true AND if the car is driving backwards->set the speed to 0! Otherwise 
 * send the current speed to output.
 */
tResult EmergencySTOP::TransmitSpeed(IMediaSample* pMediaSample) {

	if (m_freeBuffer.size() == FREE_BUFFER) m_freeBuffer.pop_front();

	if ( ( ( breakingModeFrontCenter || breakingModeFrontCenterLeft || breakingModeFrontCenterRight || breakingModeFrontLeft || breakingModeFrontRight)
			&& driveForward) || ( ( breakingModeRearCenter || breakingModeRearLeft || breakingModeRearRight) && !driveForward ) )  {

		m_freeBuffer.push_back(1);

	} else m_freeBuffer.push_back(0);
	
	tInt16 sum 	= 0;
	for (size_t i = 0; i < m_freeBuffer.size(); i++){
		sum 	+= m_freeBuffer.at(i);
	}

	fstream f;
	f.open("ESErrorDetect", ios::out|ios::app);
	f << sum << "\n";

	// Obstacle
	//if ( (sum == FREE_BUFFER && !m_emergencyBreak) || (sum > 0 && m_emergencyBreak) ){
	if ( (sum == 1 && !m_emergencyBreak) || (sum > 0 && m_emergencyBreak) ){
		// Set current state
		if (!m_emergencyBreak){
			m_stateChanged 	= tTrue;
			LOG_INFO(cString::Format("EMERGENCY-BREAK! %i | %i | %i | %i | %i | %i | %i | %i", breakingModeFrontLeft, breakingModeFrontCenterLeft, breakingModeFrontCenter, breakingModeFrontCenterRight, breakingModeFrontRight, breakingModeRearLeft, breakingModeRearCenter, breakingModeRearRight) );
			f << "EMERGENCY-Break\n";
		}
		//else LOG_INFO(cString::Format("ES: Error Detect = %i | %i | %i | %i | %i | %i | %i | %i", breakingModeFrontLeft, breakingModeFrontCenterLeft, breakingModeFrontCenter, breakingModeFrontCenterRight, breakingModeFrontRight, breakingModeRearLeft, breakingModeRearCenter, breakingModeRearRight) );
		m_emergencyBreak 	= tTrue;

		tUInt32 nTimeStamp = _clock->GetStreamTime();

		tFloat32 fZeroSpeed = 90;
		cObjectPtr<IMediaSample> pNewSample;
		AllocMediaSample((tVoid**)&pNewSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pSignalSpeedOutOutput->GetMediaSampleSerializer(&pSerializer);
		pNewSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pSignalSpeedOutOutput, pNewSample, pCoderOutput); 
			
			if (!m_bSpeedOutOutputSet){
				pCoderOutput->GetID("f32Value", m_szIDSignalValueSpeedOutput);
				pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampSpeedOutput);
				m_bSpeedOutOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szIDSignalValueSpeedOutput, (tVoid*)&fZeroSpeed);
			pCoderOutput->Set(m_szIDTimestampSpeedOutput, (tVoid*)&nTimeStamp);
		}

		pNewSample->SetTime(pNewSample->GetTime());
		m_oSpeedOut.Transmit(pNewSample);

	}else if ( (sum == 0 && m_emergencyBreak) || (sum < FREE_BUFFER && !m_emergencyBreak) ) {
		m_oSpeedOut.Transmit(pMediaSample);
		
		if (m_emergencyBreak){
			m_stateChanged 	= tTrue;	
			f << "FREE\n";
		}
		m_emergencyBreak 	= tFalse;
	}
//LOG_INFO(cString::Format("Before Sending Values!\tStateChanged: %i\tStatus: %i", m_stateChanged, m_emergencyBreak) );	
	// Only send booleans if the stop-status changed
	if (m_stateChanged){
//LOG_INFO(cString::Format("While Sending Values!\tStateChanged: %i\tStatus: %i", m_stateChanged, m_emergencyBreak) );	
		m_stateChanged  = tFalse;
		TransmitBreaklightValue(m_emergencyBreak);
		TransmitToDecisionMaking(m_emergencyBreak);			
	}
//LOG_INFO(cString::Format("After Sending Values!\tStateChanged: %i\tStatus: %i", m_stateChanged, m_emergencyBreak) );		

	RETURN_NOERROR;
}

/*
 * this function sends the breakinglight signal to the baseconfig.
 */
tResult EmergencySTOP::TransmitBreaklightValue(tBool bValue){

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
 * this function sends the Emergeny-Status to DecisionMaking
 */
tResult EmergencySTOP::TransmitToDecisionMaking(tBool bValue){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDecisionMakingOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = 0;

	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(m_pDecisionMakingOutput, pMediaSample, pCoderOutput);    

		// set the id if not already done
		if(!m_bDecisionMakingOutputSet)
		{
			pCoderOutput->GetID("bValue",  m_szIDBoolValueDecisionMakingOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampDecisionMakingOutput);
			m_bDecisionMakingOutputSet = tTrue;
		}      

		// set value from sample
		pCoderOutput->Set(m_szIDBoolValueDecisionMakingOutput, (tVoid*)&bValue);     
		pCoderOutput->Set(m_szIDTimestampDecisionMakingOutput, (tVoid*)&ui32TimeStamp);     
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	//transmit media sample over output pin
	m_oDecisionMaking.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * Function to set the state of this filter
 */
tResult EmergencySTOP::ProcessStatus(IMediaSample* pMediaSample){

	tBool bValue = tFalse;
	
	{
		__adtf_sample_read_lock_mediadescription(m_pStatusESInput ,pMediaSample, pCoder);    

		if (!m_bStatusESSet){
			pCoder->GetID("bValue", m_szIDBoolValueStatusESInput);		
			m_bStatusESSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueStatusESInput, (tVoid*)&bValue);         
	}

	LOG_INFO(cString::Format("ES: Received signal %i", bValue) );

	m_ESActivated = bValue;

//Nau auf Lauras Wunsch!! (sonst fährt Auto nicht ausParklück, falls etwas dahinter, da vorher rückwärts gefahren)
	if (!m_ESActivated){		

		breakingModeFrontCenter = breakingModeFrontCenterLeft = breakingModeFrontCenterRight = breakingModeFrontLeft = breakingModeFrontRight = breakingModeRearCenter = breakingModeRearLeft = breakingModeRearRight = tFalse;
		
		driveForward 	= tTrue;

		m_stateChanged 	= tFalse;

		TransmitBreaklightValue(tFalse);
		TransmitToDecisionMaking(tFalse);	

		m_emergencyBreak 	= tFalse;

		m_freeBuffer.clear();

	}

	RETURN_NOERROR;
}
