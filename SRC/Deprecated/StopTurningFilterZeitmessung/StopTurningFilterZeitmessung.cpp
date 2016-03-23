/*
 * Date 18.02.2016
 */ 

#include <math.h>
#include "stdafx.h"
#include "StopTurningFilterZeitmessung.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"
#include <fstream>

#include "../../include/action_enum.h"
#include "../../include/cross_type.h"
#include "../../include/cross_sign_type.h"

#define SPEED 1.5

ADTF_FILTER_PLUGIN("StopTurningFilterZeitmessung", __guid, StopTurningFilterZeitmessung);


StopTurningFilterZeitmessung::StopTurningFilterZeitmessung(const tChar* __info)
{

	SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)");
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0);
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

	turningModeLookup[CROSSROAD][ACTION_LEFT][GIVEWAY] = 1;	// cross, left, give way
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][GIVEWAY] = 2; // cross, straight on, give way
	turningModeLookup[CROSSROAD][ACTION_RIGHT][GIVEWAY] = 3; // cross, right, give way
	turningModeLookup[CROSSROAD][ACTION_LEFT][HAVEWAY] = 4; // cross, left, have way
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][HAVEWAY] = 5; // cross, straight on, have way
	turningModeLookup[CROSSROAD][ACTION_RIGHT][HAVEWAY] = 6; // cross, right, have way
	turningModeLookup[CROSSROAD][ACTION_LEFT][RIGHTBEFORELEFT] = 13;// cross, left, right before left
	turningModeLookup[CROSSROAD][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 14;// cross, straight on, right before left
	turningModeLookup[CROSSROAD][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // cross, right, right before left

	turningModeLookup[DEADENDLEFT][ACTION_LEFT][GIVEWAY] = 0; // dead end left, left, give way
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][GIVEWAY] = 7; // dead end left, straight on, give way
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][GIVEWAY] = 8; // dead end left, right, give way
	turningModeLookup[DEADENDLEFT][ACTION_LEFT][HAVEWAY] = 0; // dead end left, left, have way
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][HAVEWAY] = 5; // dead end left, straight on, have way
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][HAVEWAY] = 6; // dead end left, right, have way
	turningModeLookup[DEADENDLEFT][ACTION_LEFT][RIGHTBEFORELEFT] = 0; // dead end left, left, right before left
	turningModeLookup[DEADENDLEFT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 14;// dead end left, straight on, right before left
	turningModeLookup[DEADENDLEFT][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // dead end left, right, right before left

	turningModeLookup[DEADENDFRONT][ACTION_LEFT][GIVEWAY] = 9; // dead end front, left, give way
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][GIVEWAY] = 0; // dead end front, straight on, give way
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][GIVEWAY] = 3; // dead end front, right, give way
	turningModeLookup[DEADENDFRONT][ACTION_LEFT][HAVEWAY] = 10;// dead end front, left, have way
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][HAVEWAY] = 0; // dead end front, straight on, have way
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][HAVEWAY] = 6; // dead end front, right, have way
	turningModeLookup[DEADENDFRONT][ACTION_LEFT][RIGHTBEFORELEFT] = 15;// dead end front, left, right before left
	turningModeLookup[DEADENDFRONT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 0; // dead end front, straight on, right before left
	turningModeLookup[DEADENDFRONT][ACTION_RIGHT][RIGHTBEFORELEFT] = 6; // dead end front, right, right before left

	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][GIVEWAY] = 11;// dead end right, left, give way
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][GIVEWAY] = 12;// dead end right, straight on, give way
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][GIVEWAY] = 0; // dead end right, right, give way
	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][HAVEWAY] = 4; // dead end right, left, have way
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][HAVEWAY] = 5; // dead end right, straight on, have way
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][HAVEWAY] = 0; // dead end right, right, have way
	turningModeLookup[DEADENDRIGHT][ACTION_LEFT][RIGHTBEFORELEFT] = 4;// dead end right, left, right before left                
	turningModeLookup[DEADENDRIGHT][ACTION_STRAIGHT][RIGHTBEFORELEFT] = 5; // dead end right, straight on, right before left
	turningModeLookup[DEADENDRIGHT][ACTION_RIGHT][RIGHTBEFORELEFT] = 0; // dead end right, right, right before left
}

StopTurningFilterZeitmessung::~StopTurningFilterZeitmessung()
{

}


tResult StopTurningFilterZeitmessung::Init(tInitStage eStage, __exception)
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

		// Media Description Turning
		tChar const * TurningDesc = pDescManager->GetMediaDescription("tTurningStopStruct");
		RETURN_IF_POINTER_NULL(TurningDesc);
		cObjectPtr<IMediaType> pTypeTurningDesc = new cMediaType(0, 0, 0, "tTurningStopStruct", TurningDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Media Description LaneTracking Velocity
        tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
        RETURN_IF_POINTER_NULL(LTVelocityDescValue);
        cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
		RETURN_IF_FAILED(pTypeTurningDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTurningInput));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));
		RETURN_IF_FAILED(pTypeIntSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTurningOutput));

		//create input pin for starting
    	RETURN_IF_FAILED(m_iStartTurning.Create("StartTurningStruct", pTypeTurningDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartTurning));

		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));

        // Start Turning Output
		RETURN_IF_FAILED(m_oStartTurning.Create("StartTurning", pTypeIntSignalValue, static_cast<IPinEventSink*> (this)));
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



        RETURN_NOERROR;
    }
    else if(eStage == StageNormal)
    {
    }
    else if(eStage == StageGraphReady)
    {

        // no ids were set so far
		m_bIDsBoolValueOutput			= tFalse;
		m_bIDsOutputSet					= tFalse;
		m_bIDsStartTurningSet			= tFalse;
		m_bIDsStartTurningOutputSet		= tFalse;
		m_bIDsDistanceSet				= tFalse;
		m_bIDsVelocityOutput			= tFalse;

		InitialState();
    }

    RETURN_NOERROR;
}


tResult StopTurningFilterZeitmessung::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}


tResult StopTurningFilterZeitmessung::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult StopTurningFilterZeitmessung::Stop(__exception)
{
    if (m_hTimerOutput)
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult StopTurningFilterZeitmessung::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult StopTurningFilterZeitmessung::OnPinEvent(IPin* pSource,
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

		if (&m_iStartTurning == pSource) {
			ProcessTurningStruct(pMediaSample);
		} else if (pSource == &m_iDistance && m_stopTurningActive){
			//LOG_INFO(cString::Format("SFT: pSource distance&stopTurningActive" ));
			ProcessStoppingSpeed(pMediaSample); // slow down car speed
		}
	}
    RETURN_NOERROR;
}



tResult StopTurningFilterZeitmessung::ChangeSpeed(tFloat32 fSpeed){

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

        //transmit media sample over output pin

        RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oSpeed.Transmit(pMediaSampleValue1));
	}
	RETURN_NOERROR;
}


tResult StopTurningFilterZeitmessung::ProcessTurningStruct(IMediaSample* pMediaSample){

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

	TransmitBoolValue(&m_oStopLF, tTrue);
	TransmitBoolValue(&m_oStopES, tTrue);
			
	tFloat32 	velocity = 0;
	tFloat32 	distance = -1;
	tInt32		mode = -1;
	tInt32		type = -1;
	tInt32		giveWay = -1;
	tBool		stopSign = tFalse;
	tBool 		bValue = tFalse;
	tBool 		bValuePullOutMode = tFalse;


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
	}

	m_filterActive 	= bValue;
	m_velocityZero 	= velocity;
	m_distanceZero 	= distance;
	m_stopSign 		= stopSign;
	m_giveWay 		= giveWay;
	m_turningType	= type;
	m_turningMode 	= turningModeLookup[type][mode][giveWay]; 
	m_dist2crossing = distance;
	m_pullOutMode	= bValuePullOutMode;



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
			m_velocityZero = SPEED;
			ChangeSpeed(SPEED);
		}
		m_stopTurningActive = bValue;
		LOG_INFO(cString::Format("STF: m_stopTurningActive")); 
	} else {
		m_i32TimeStampHoldLine = _clock->GetStreamTime();

		if (m_stopSign){
			m_i32TimeStampHoldLine += 3000000;
			Wait();
		} 
        TransmitBoolValue(&m_oStopLF, tFalse);
	}



	//LOG_INFO(cString::Format("STF: StartTurning stopSign %i | giveWay %i | type %i | mode %i", m_stopSign, m_giveWay, m_turningType, m_turningMode) );

	//LOG_INFO(cString::Format("SFT: Distance: %f", m_distanceZero) );

	RETURN_NOERROR;
}

tResult StopTurningFilterZeitmessung::StartTurning()
{
	TransmitBoolValue(&m_oStopES, tFalse);

	if (m_DriveStraightSet){
		TransmitBoolValue(&m_oDriveStraight, tFalse);
		m_DriveStraightSet 	= tFalse;
	}
	
 LOG_INFO(cString::Format("STF:Biege ab!!") );
// losfahren

			// stop drive straight

        	//start TurningFilter

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
                        pCoder->GetID("intValue", m_szIDStartTurning);
						m_bIDsStartTurningOutputSet = tTrue;
                    }

                    pCoder->Set(m_szIDStartTurning, (tVoid*)&m_turningManeuvre);
                }

                //transmit media sample over output pin
                RETURN_IF_FAILED(pMediaSampleValue->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oStartTurning.Transmit(pMediaSampleValue));
            }

	InitialState();

	RETURN_NOERROR;
}



tResult StopTurningFilterZeitmessung::ProcessStoppingSpeed(IMediaSample* pMediaSample){

    tFloat32 fDistance = 0.0;
	tUInt32 fTimeStamp = 0;
	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

		if (!m_bIDsDistanceSet) {
			pCoder->GetID("f32Value", m_szIDDistanceInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput); 
			m_bIDsDistanceSet = tTrue;
		}  

		pCoder->Get(m_szIDDistanceInput, (tVoid*)&(fDistance));
		pCoder->Get(m_szIDTimestampDistanceInput, (tVoid*)&(fTimeStamp));
	
	}
	// calc Speed

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = fDistance;
	}

	fDistance -= m_distanceOffset;
		//LOG_INFO(cString::Format("STF: fdistance %f",fDistance));
	m_dist2crossing = m_distanceZero - fDistance;


    
	tFloat32 faux = fDistance/m_distanceZero;

	tFloat32 fSpeed;


	// turn on lights if necessary
	if (fDistance<=2) {

		if (m_turningManeuvre==ACTION_RIGHT && !m_turnRightON){

			m_turnRightON = tTrue;
			TransmitBoolValue(&m_oTurnRightOutput, true);

		} else if (m_turningManeuvre==ACTION_LEFT && !m_turnLeftON){

			m_turnLeftON = tTrue;
			TransmitBoolValue(&m_oTurnLeftOutput, true);
		}
	}

        fSpeed = sqrt(faux)*(SPEED-m_velocityZero)+m_velocityZero;

	if (faux > 1.0 -(1e-4)) {
                LOG_INFO(cString::Format("SF: an Haltelinie") );
		//TransmitBoolValue(&m_oStopLF, tFalse);
		m_turnRightON = tFalse;
		m_turnLeftON = tFalse;
		//TransmitBoolValue(&m_oTurnLeftOutput, false);		
		//TransmitBoolValue(&m_oTurnRightOutput, false);
		m_stopTurningActive = tFalse;
		fSpeed = 0.0;
		m_dist2crossing = 0.0;

	} else if ((m_distanceZero-fDistance)<0.15) {

		if (!m_DriveStraightSet){
			TransmitBoolValue(&m_oDriveStraight, tTrue);
			m_DriveStraightSet 	= tTrue;
		}
	}



	if (m_stopTurningActive) {

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


	} else {

		TransmitBoolValue(&m_oStopLF, tFalse);

		if (m_stopSign){
			if(Wait()) {
				RETURN_NOERROR;
			}
		}
                
		StartTurning();
	}

	RETURN_NOERROR;
}

bool StopTurningFilterZeitmessung::Wait() {
/*
LOG_INFO(cString::Format("SFT: START_WAIT!") );
	tFloat32 arrival = _clock->GetStreamTime();
	tFloat32 fTime = 0.0;

	while (fTime<3){
		fTime = (_clock->GetStreamTime()-arrival)/1000000;
		//LOG_INFO(cString::Format("fTime %f",fTime));

	}
*/

	if (m_startWait == 0) {
		m_startWait = _clock->GetStreamTime();
		m_waiting = tTrue;
	}

	if ((_clock->GetStreamTime() - m_startWait)/1000000.0 < 3) {
		return true;	
	} else {
		m_startWait = 0;
		m_waiting = tFalse;
		return false;
	}
}


tResult StopTurningFilterZeitmessung::TransmitBoolValue(cOutputPin* oPin, bool value)
{
	__synchronized_obj(m_oTransmitBoolCritSection);

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

void StopTurningFilterZeitmessung::InitialState(){


	m_stopTurningActive				= tFalse;
	m_turnRightON					= tFalse;
	m_turnLeftON					= tFalse;
	m_DriveStraightSet				= tFalse;
	m_waiting						= tFalse;
	m_filterActive					= tFalse;
	m_pullOutMode					= tFalse;
	m_turningManeuvre				= ACTION_UNKNOWN;
	m_distanceOffset				= 0.0;
	m_velocityZero					= 0.0;
	m_distanceZero					= 0.0;
	m_dist2crossing 				= 0.0;
	m_i32TimeStampHoldLine			= 0;
	m_startWait						= 0;
	m_turningMode					= 0;
	m_stopSign						= 0;
	m_giveWay						= -1;
	m_turningType					= -1;

}



