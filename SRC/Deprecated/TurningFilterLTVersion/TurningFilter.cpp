/*
 * Date 25.01.2016
 */ 

#include <math.h>
#include "stdafx.h"
#include "TurningFilter.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"
#include <fstream>

#include "../../include/action_enum.h"
#include "../../include/cross_type.h"
#include "../../include/cross_sign_type.h"


using namespace cv;

ADTF_FILTER_PLUGIN("TurningFilter", __guid, TurningFilter);


TurningFilter::TurningFilter(const tChar* __info)
{

}

TurningFilter::~TurningFilter()
{

}


tResult TurningFilter::Init(tInitStage eStage, __exception)
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



        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput2));

		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeIntSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTurningInput)); 
		RETURN_IF_FAILED(pTypeUltraSonicDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUltraSonicInput)); 

		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInputRight));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInputLeft));

		//create input pin for starting
    	RETURN_IF_FAILED(m_iStartTurning.Create("StartTurning", pTypeIntSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartTurning));

		//create input pin for US
    	RETURN_IF_FAILED(m_iUltraSonic.Create("us_struct", pTypeUltraSonicDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonic));

		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));

        // DistanceInput
		RETURN_IF_FAILED(m_iDistanceRight.Create("Distance_right_wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistanceRight));

        // DistanceInput
		RETURN_IF_FAILED(m_iDistanceLeft.Create("Distance_left_wheel", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistanceLeft));

		//
        RETURN_IF_FAILED(m_iTestLT.Create("StopTurning_LT", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_iTestLT));



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
        
		//create output pin for break light light output data
        RETURN_IF_FAILED(m_oBreaklight.Create("Break_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oBreaklight));

		//create output pin for finish turning output data
        RETURN_IF_FAILED(m_oFinishTurning.Create("finish_Turning", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oFinishTurning));

		// Start EmergencySTOP Output
        RETURN_IF_FAILED(m_oStartES.Create("Start_ES", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStartES));


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
		m_bIDsStartTurningSet 			= tFalse;

		InitialState();

    }

    RETURN_NOERROR;
}


tResult TurningFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult TurningFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult TurningFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult TurningFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult TurningFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
    	RETURN_IF_POINTER_NULL(pMediaSample);

		if (&m_iStartTurning == pSource) {
			ProcessTurningStruct(pMediaSample);
		} else if (pSource == &m_iDistanceLeft && m_isTurning && m_turningManeuvre==ACTION_RIGHT){
			// Steuerung beim Abbiegen
			ProcessTurningSpeed(pMediaSample);
        } else if (pSource == &m_iDistanceRight && m_isTurning && m_turningManeuvre==ACTION_LEFT){
            ProcessTurningSpeed(pMediaSample);
        } else if (pSource == &m_iDistance && m_isTurning && m_turningManeuvre==ACTION_STRAIGHT){
            ProcessTurningSpeed(pMediaSample);
		} else if (&m_iUltraSonic == pSource && m_isTurning) {
			//LOG_INFO(cString::Format("SFT: pSource ultraSonic" ));
			ProcessUltraSonic(pMediaSample);
		} else if (&m_itestLT == pSource && m_testLT){
			m_stopTurning = tTrue;
		}
	}
    RETURN_NOERROR;
}
tResult TurningFilter::ProcessUltraSonic(IMediaSample* pMediaSample){

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



			tBool emergencyStop = tFalse;
			tFloat32 fSpeed = 0.5;
			tFloat32 fSteering = 90.0;


			switch (m_drivingMode) {
				case ACTION_STRAIGHT:   // straight
					if (frontCenterLeft.f32Value < 0.1 || frontCenter.f32Value < 0.2 || frontCenterRight.f32Value < 0.1) {
						emergencyStop = tTrue;
					}
					break;
				case ACTION_LEFT:   //left
					fSteering = 60.0;
					if (frontLeft.f32Value < 0.1 || frontCenterLeft.f32Value < 0.2 || frontCenter.f32Value < 0.1 ) {
						emergencyStop = tTrue;
					}
					break;

				case ACTION_RIGHT:  //right
					fSteering = 120.0;
					if ( frontCenter.f32Value < 0.1 || frontCenterRight.f32Value < 0.2 || frontRight.f32Value < 0.1) {
						emergencyStop = tTrue;
					}
					break;
                }


                if (emergencyStop && !m_emergencyStop) {
                        // stop car
                        fSpeed = 0.0;
						m_emergencyStop = tTrue;
						TransmitBoolValue(&m_oBreaklight, tTrue);
                        changeSpeed(fSpeed, fSteering);
                } else if ( !emergencyStop && m_emergencyStop) {
                        ++m_good_sensor_count;

						if (m_good_sensor_count >= 5){
                        	m_emergencyStop = tFalse;
							m_good_sensor_count = 0;
							TransmitBoolValue(&m_oBreaklight, tFalse);
                        	changeSpeed(fSpeed, fSteering);
						}
                }
                RETURN_NOERROR;
		}

	tResult TurningFilter::changeSpeed(tFloat32 fSpeed, tFloat32 fSteering){

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
			
			
	RETURN_NOERROR;

}

tResult TurningFilter::ProcessTurningStruct(IMediaSample* pMediaSample){

	tInt32	maneuvre = ACTION_UNKNOWN;

	{
		__adtf_sample_read_lock_mediadescription(m_pStartTurningInput, pMediaSample, pCoder);    

		if (!m_bIDsStartTurningSet){
			pCoder->GetID("intValue", m_szIDManeuvreStartTurningInput);
			m_bIDsStartTurningSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDManeuvreStartTurningInput, (tVoid*)&maneuvre);
	}

	m_turningManeuvre = maneuvre;


	//LOG_INFO(cString::Format("STF: StartTurning  maneuvre %i", m_turningManeuvre) );
	StartTurning();
	turning();

	RETURN_NOERROR;
}

tResult TurningFilter::StartTurning()
{
// losfahren
	m_isTurning = tTrue;

	RETURN_NOERROR;
}



tResult TurningFilter::ProcessTurningSpeed(IMediaSample* pMediaSample){

    tFloat32 fDistance = 0.0;
	tUInt32 fTimeStamp = 0;
	tFloat32 fSpeed = 0.0;
	tFloat32 fSteering = 90.0;

	if (m_turningManeuvre==ACTION_RIGHT) {
                // turn on turnRight-light
		if (!m_turnRightON) {
			TransmitBoolValue(&m_oTurnRightOutput, true);
			m_turnRightON = true;
		}

	        {
		        __adtf_sample_read_lock_mediadescription(m_pDistanceInputLeft,pMediaSample, pCoder);

		        if (!m_bIDsDistanceLeftSet) {
			        pCoder->GetID("f32Value", m_szIDDistanceInputLeft);
			        pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInputLeft);
			        m_bIDsDistanceLeftSet = tTrue;
		        }

		        pCoder->Get(m_szIDDistanceInputLeft, (tVoid*)&(fDistance));
		        pCoder->Get(m_szIDTimestampDistanceInputLeft, (tVoid*)&(fTimeStamp));

	        }

                // calc Speed % steering

	        if (m_distanceOffset == 0.0) {
		        m_distanceOffset = fDistance;
	        }

	        fDistance -= m_distanceOffset;

                if (fDistance >= 0.0 && fDistance <= 1.20) { //1.50
                        fSpeed = 1.5;
                        fSteering = 120.0;
                        m_drivingMode = ACTION_RIGHT;

						if (fDistance > 1.20) {
							m_testLT = tTrue;
						}

						if (m_stopTurning){
							finishTurning();
						}

                } else  {
                        fSpeed = 0.0;//ANPASSEN!
                        fSteering = 90.0;

						finishTurning();
                }


        } else if (m_turningManeuvre==ACTION_LEFT){
                // turn on turnLeft-light
		if (!m_turnLeftON) {
			TransmitBoolValue(&m_oTurnLeftOutput, true);
			m_turnLeftON = true;
		}


	        {
		        __adtf_sample_read_lock_mediadescription(m_pDistanceInputRight,pMediaSample, pCoder);

		        if (!m_bIDsDistanceRightSet) {
			        pCoder->GetID("f32Value", m_szIDDistanceInputRight);
			        pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInputRight);
			        m_bIDsDistanceRightSet = tTrue;
		        }

		        pCoder->Get(m_szIDDistanceInputRight, (tVoid*)&(fDistance));
		        pCoder->Get(m_szIDTimestampDistanceInputRight, (tVoid*)&(fTimeStamp));

	        }

                // calc Speed % steering

	        if (m_distanceOffset == 0.0) {
		        m_distanceOffset = fDistance;
	        }

	        fDistance -= m_distanceOffset;

                if (fDistance >= 0 && fDistance <= 0.3) {
                        fSpeed = 2.0;
                        fSteering = 90.0;
                        m_drivingMode = ACTION_STRAIGHT;
                } else if (fDistance > 0.3 && fDistance <= 0.55) {
                        fSpeed = 0.5;
                        fSteering = 90.0;
                        m_drivingMode = ACTION_STRAIGHT;
                } else if (fDistance > 0.55 && fDistance <= 2.00) {
                        fSpeed = 1.5;
                        fSteering = 60.0;
                        m_drivingMode = ACTION_LEFT;

						if (fDistance > 1.70) {
							m_testLT = tTrue;
						}

						if (m_stopTurning){
							finishTurning();
						}

                } else  {
                        fSpeed = 0.0;//ANPASSEN
                        fSteering = 90.0;

						finishTurning();
                }

        } else if (m_turningManeuvre==ACTION_STRAIGHT) {
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
                // calc Speed % steering

	        if (m_distanceOffset == 0.0) {
		        m_distanceOffset = fDistance;
	        }

	        fDistance -= m_distanceOffset;

                if (fDistance >= 0 && fDistance <= 1.60) {
                        fSpeed = 2.0;
                        fSteering = 90.0;
                        m_drivingMode = ACTION_STRAIGHT;

						if (fDistance > 0.60) {
							m_testLT = tTrue;
						}

						if (m_stopTurning){
							finishTurning();
						}

                } else  {
                        fSpeed = 1.5;
                        fSteering = 90.0;
						finishTurning();
                }

        }
		//LOG_INFO(cString::Format("SFT: emergencystop %i", m_emergencyStop ));
        if (!m_emergencyStop) {
			changeSpeed(fSpeed, fSteering);
        }
        
	RETURN_NOERROR;
}

void TurningFilter::finishTurning() {

    // turn off turnRight-light if on
    if (m_turnRightON) {
            TransmitBoolValue(&m_oTurnRightOutput, false);
    }

    // turn off turnLeft-light if on
    if (m_turnLeftON) {
            TransmitBoolValue(&m_oTurnLeftOutput, false);
    }

	InitialState();
	TransmitBoolValue(&m_oStartES, true);
    TransmitBoolValue(&m_oFinishTurning, true);
}

tResult TurningFilter::turning() {

	tFloat32 fSpeed = 0.5;

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


tResult TurningFilter::TransmitBoolValue(cOutputPin* oPin, bool value)
{
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

tResult TurningFilter::InitialState()
{
	m_stopTurning					= tFalse;
	m_testLT						= tFalse;
	m_emergencyStop					= tFalse;
	m_isTurning 					= tFalse;
	m_turnRightON					= tFalse;
	m_turnLeftON					= tFalse;
	m_turningManeuvre				= ACTION_UNKNOWN;
	m_drivingMode					= ACTION_UNKNOWN;
	m_distanceOffset				= 0.0;
	m_good_sensor_count				= 0;

    RETURN_NOERROR;
}


