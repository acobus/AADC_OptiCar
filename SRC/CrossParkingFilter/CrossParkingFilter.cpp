/*
 * Date 16.02.2016
 */ 

#include <math.h>
#include "stdafx.h"
#include "CrossParkingFilter.h"

#define FACTOR_LAST_PART 0.98
//0.9


ADTF_FILTER_PLUGIN("CrossParkingFilter", __guid, CrossParkingFilter);


CrossParkingFilter::CrossParkingFilter(const tChar* __info) : m_hazzardON(false), m_turnRightON(false), m_reverseON(false), m_finishParking(false), m_isParking(false), m_stateHold(false)
{
	SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

	SetPropertyFloat("Default Speed",0);
	SetPropertyStr("Default Speed" NSSUBPROP_DESCRIPTION, "Defines the default speed [m/s] which is transmitted when nothing is read from the list");
	SetPropertyBool("Default Speed" NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyFloat("Default Steering",90);
	SetPropertyStr("Default Steering" NSSUBPROP_DESCRIPTION, "Defines the default steering [servo angle] which is transmitted when nothing is read from the list");
	SetPropertyFloat("Default Steering" NSSUBPROP_MIN, 60); 
	SetPropertyFloat("Default Steering" NSSUBPROP_MAX, 120);
	SetPropertyBool("Default Steering" NSSUBPROP_ISCHANGEABLE, tTrue); 

	SetPropertyFloat("Time Stretch Factor",1.52);
	SetPropertyStr("Time Stretch Factor" NSSUBPROP_DESCRIPTION, "Time Stretch Factor");
	SetPropertyFloat("Time Stretch Factor" NSSUBPROP_MIN, 0.0);
	SetPropertyBool("Time Stretch Factor" NSSUBPROP_ISCHANGEABLE, tTrue);

	calc();
}

CrossParkingFilter::~CrossParkingFilter()
{
}

void CrossParkingFilter::calc() {
	
	fX2m = m_sProperties.fRmin;
	fY2m = m_sProperties.fYe-sqrt((m_sProperties.fRmin-m_sProperties.fb)*(m_sProperties.fRmin-m_sProperties.fb) - (m_sProperties.fXe-fX2m)*(m_sProperties.fXe-fX2m));
	
	fY1m = m_sProperties.fYc+m_sProperties.fRmin;
	fX1m = fX2m - sqrt(4.0*m_sProperties.fRmin*m_sProperties.fRmin - (fY1m-fY2m)*(fY1m-fY2m));

	fXi = fX1m + 0.5*(fX2m-fX1m);
	fYi = fY1m + 0.5*(fY2m-fY1m);

    // auxiliary memory
    tFloat32 faux = sqrt((fXi-fX1m)*(fXi-fX1m) + (fYi-m_sProperties.fYc)*(fYi-m_sProperties.fYc));

	fAlpha1 = acos((2.0*m_sProperties.fRmin*m_sProperties.fRmin-faux*faux)/(2.0*m_sProperties.fRmin*m_sProperties.fRmin));

	fDistance[0] = m_sProperties.fRmin*fAlpha1+0.15;//+fX1m;//-m_sProperties.fXc;

	faux = sqrt((fXi-0.0)*(fXi-0.0) + (fYi-fY2m)*(fYi-fY2m));

	fAlpha2 = acos((2.0*m_sProperties.fRmin*m_sProperties.fRmin-faux*faux)/(2.0*m_sProperties.fRmin*m_sProperties.fRmin));

	fDistance[1] = m_sProperties.fRmin*fAlpha2+fY2m-0.0;

	faux = m_sProperties.fMinAcc-m_sProperties.fMaxAcc;

	fSwitchingPointsOrig[0] = m_sProperties.fv/(faux-m_sProperties.fMinAcc)+sqrt((m_sProperties.fv*m_sProperties.fv)/((m_sProperties.fMinAcc-faux)*(m_sProperties.fMinAcc-faux))+(2.0*fDistance[0]*m_sProperties.fMinAcc+m_sProperties.fv*m_sProperties.fv)/(m_sProperties.fMinAcc*faux-faux*faux));

	fSwitchingPointsOrig[1] = (-m_sProperties.fv+faux*fSwitchingPointsOrig[0])/m_sProperties.fMinAcc;

	fSwitchingPointsOrig[2] = sqrt(2.0*fDistance[1]*m_sProperties.fMinAcc/(m_sProperties.fMinAcc*faux-faux*faux));

	fSwitchingPointsOrig[4] = faux*fSwitchingPointsOrig[2]/m_sProperties.fMinAcc;	
	fSwitchingPointsOrig[4] += fSwitchingPointsOrig[1];

	
	fSwitchingPointsOrig[3] = faux*fSwitchingPointsOrig[2]/m_sProperties.fMinAcc-sqrt(-faux/m_sProperties.fMinAcc*fSwitchingPointsOrig[2]*fSwitchingPointsOrig[2]+(faux*fSwitchingPointsOrig[2]/m_sProperties.fMinAcc)*(faux*fSwitchingPointsOrig[2]/m_sProperties.fMinAcc)+(fDistance[1]-(fY2m-0.0))*2.0/m_sProperties.fMinAcc);
	fSwitchingPointsOrig[3] += fSwitchingPointsOrig[1];

	fSwitchingPointsOrig[2] += fSwitchingPointsOrig[1];

	m_fTSF = static_cast<tFloat32>(GetPropertyFloat("Time Stretch Factor"));

	for (int i=0;i<5;i++){
		fSwitchingPoints[i] = fSwitchingPointsOrig[i] * m_fTSF;
	}
	LOG_INFO(cString::Format("CP: Stretch-Faktor %f",m_fTSF));
}

tResult CrossParkingFilter::Init(tInitStage eStage, __exception)
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

		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput2));

		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartParkingInput));


		//create input pin for starting
    	RETURN_IF_FAILED(m_iStartParking.Create("StartCrossParking", pTypeBoolSignalValue, this));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartParking));

        //create output pin for speed output data
        RETURN_IF_FAILED(m_oSpeedOutput.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oSpeedOutput));

        //create output pin for steering output data
        RETURN_IF_FAILED(m_oSteeringOutput.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringOutput));	

		//create output pin for hazzard light output data
        RETURN_IF_FAILED(m_oHazzardLightOutput.Create("Hazzard_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oHazzardLightOutput));	

		//create output pin for turn-right light output data
        RETURN_IF_FAILED(m_oTurnRightOutput.Create("turn-right_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnRightOutput));

		//create output pin for reverse light output data
        RETURN_IF_FAILED(m_oReverseLightOutput.Create("reverse_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oReverseLightOutput));

		//create output pin for finish parking output data
        RETURN_IF_FAILED(m_oFinishParkingOutput.Create("finish_Parking", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oFinishParkingOutput));
    	

        RETURN_NOERROR;
    }
    else if(eStage == StageNormal)
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Speed"));
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Steering"));
		m_fTSF = static_cast<tFloat32>(GetPropertyFloat("Time Stretch Factor"));
    }
    else if(eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsOutputSet = tFalse;
        m_bIDsOutput2Set = tFalse;
		m_bIDsBoolValueOutput = tFalse;
		m_i32StartTimeStamp	= 0;
    }
    RETURN_NOERROR;
}


tResult CrossParkingFilter::PropertyChanged(const tChar* strName)
{
    //read from property if it was changed
    if (cString::IsEqual("Default Speed", strName))
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Speed"));
    }
    else if (cString::IsEqual("Default Steering", strName))
    {
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Steering"));
    }
	else if (cString::IsEqual("Time Stretch Factor", strName))
    {
        m_fTSF = static_cast<tFloat32>(GetPropertyFloat("Time Stretch Factor"));
		for (int i=0;i<5;i++){
			fSwitchingPoints[i]=fSwitchingPointsOrig[i] * m_fTSF;
		}
		//fSwitchingPoints[3]-=0.2;
		LOG_INFO(cString::Format("CP: Stretch-Faktor %f",m_fTSF));
    }

    RETURN_NOERROR;
}
tResult CrossParkingFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult CrossParkingFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
	if (!m_isParking)
	{	
		return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);	
	}
    if (nActivationCode == IRunnable::RUN_TIMER)
    {     

		tUInt32 time = _clock->GetStreamTime();   

		if ((time - m_i32StartTimeStamp)/1000000.0 > 0.5 && m_stateHold){
			m_stateHold = tFalse;
			m_i32StartTimeStamp = _clock->GetStreamTime();
			LOG_INFO(cString::Format("CPF: m_stateHold auf false, starte echtes Einparken"));
		}

        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput)
        {

            //create new media sample for steering controller	
            {                
                cObjectPtr<IMediaSample> pMediaSampleValue1;	
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

                //allocate memory with the size given by the descriptor	
                cObjectPtr<IMediaSerializer> pSerializer1;	
                m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer1);
                tInt nSize = pSerializer1->GetDeserializedSize();	
                RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));

				tFloat32 value1 = 0.0; 
				
				if (!m_stateHold){
                	value1 = getCurrentValue(_clock->GetStreamTime(),1);
				}

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
                    pCoder->Set(m_szIDOutputF32Value, (tVoid*)&value1);
                }

                //transmit media sample over output pin

                RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));


                RETURN_IF_FAILED(m_oSpeedOutput.Transmit(pMediaSampleValue1));
            }
            //create new media sample for speed controller	
            {                
                cObjectPtr<IMediaSample> pMediaSampleValue2;	
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue2));

                //allocate memory with the size given by the descriptor	
                cObjectPtr<IMediaSerializer> pSerializerSpeed;	
                m_pDescriptionOutput2->GetMediaSampleSerializer(&pSerializerSpeed);
                tInt nSizeSpeed = pSerializerSpeed->GetDeserializedSize();	
                RETURN_IF_FAILED(pMediaSampleValue2->AllocBuffer(nSizeSpeed));	

				tFloat32 value2 = 90.0; 
				
				if (!m_stateHold){
                	value2 = getCurrentValue(_clock->GetStreamTime(),2);
				}

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

                    pCoder2->Set(m_szIDOutput2F32Value, (tVoid*)&value2);
                }

                //transmit media sample over output pin	
                RETURN_IF_FAILED(pMediaSampleValue2->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oSteeringOutput.Transmit(pMediaSampleValue2));
            }
        }

    }
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult CrossParkingFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult CrossParkingFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cFilter::Shutdown(eStage, __exception_ptr);
}


void CrossParkingFilter::crossParking(tBool start)
{
	if (start && !m_isParking) {
		m_finishParking = false;
    	m_i32StartTimeStamp = _clock->GetStreamTime();
		LOG_INFO(cString::Format("%f| %f| %f| %f| %f",fSwitchingPoints[0],fSwitchingPoints[1],fSwitchingPoints[2],fSwitchingPoints[3],fSwitchingPoints[4]));
	}
	m_stateHold = start;
	m_isParking = start;
	
}

tFloat32 CrossParkingFilter::getDefaultValue(tInt8 i8ValueID)
{
    switch (i8ValueID)
    {
    case 1:
        return m_f32DefaultValue1;
        break;
    case 2:
        return m_f32DefaultValue2;
        break;
    default:                
        return m_f32DefaultValue1;
        break;
    }
}

tFloat32 CrossParkingFilter::getCurrentValue(tFloat32 fTime, tInt8 i8ValueID)
{

	// list empty or output disabled, return default
    if (m_isParking == tFalse) {
		return getDefaultValue(i8ValueID);
	}

	// turn on turnRight-light
	if (!m_turnRightON) {
		TransmitBoolValue(&m_oTurnRightOutput, true);
		m_turnRightON = true;
	}

	//get time difference in seconds
    fTime = (_clock->GetStreamTime()-m_i32StartTimeStamp)/1000000.0;

    switch (i8ValueID)
    {
    /*case 1: {
		if (fTime>=0 && fTime<fSwitchingPoints[0]) {
			return m_sProperties.fMaxAcc*fTime+m_sProperties.fv;
		} else if (fTime<fSwitchingPoints[1]) {
			return m_sProperties.fMinAcc*fTime-m_sProperties.fMinAcc*fSwitchingPoints[1];
		} else if (fTime<fSwitchingPoints[1]+0.2) {
			return 0.0;
		} else if (fTime<(fSwitchingPoints[2]+0.2)) {
			return m_sProperties.fMinAcc*(fTime-0.2)-m_sProperties.fMinAcc*fSwitchingPoints[1];
		} else if (fTime<(fSwitchingPoints[4]+0.2)) {
			return m_sProperties.fMaxAcc*(fTime)-m_sProperties.fMaxAcc*(fSwitchingPoints[4]+0.2);
		} else {
			return 0.0;
		}
        break;}*/
	case 1: {
		if (fTime>=0 && fTime<fSwitchingPoints[0]) {
			return m_sProperties.fMaxAcc*fTime+m_sProperties.fv;
		} else if (fTime<fSwitchingPoints[1]) {
			return m_sProperties.fMinAcc*fTime-m_sProperties.fMinAcc*fSwitchingPoints[1];
		} else if (fTime<(fSwitchingPoints[2]*FACTOR_LAST_PART)) {
			// turn on reverse-light
			if (!m_reverseON) {
				TransmitBoolValue(&m_oReverseLightOutput, true);
				m_reverseON = true;
			}
			return m_sProperties.fMinAcc*(fTime)-m_sProperties.fMinAcc*fSwitchingPoints[1];
		} else if (fTime<(fSwitchingPoints[4]*FACTOR_LAST_PART)) {
			return m_sProperties.fMaxAcc*(fTime)-m_sProperties.fMaxAcc*(fSwitchingPoints[4]);
		} else if (fTime<(fSwitchingPoints[4]*FACTOR_LAST_PART+3.0)) {
			// turn off reverse-light
			if (m_reverseON) {
				m_reverseON = false;
				TransmitBoolValue(&m_oReverseLightOutput, false);
			}
			// turn on hazzard-light
			if (!m_hazzardON) {
				m_hazzardON = true;
				TransmitBoolValue(&m_oHazzardLightOutput, true);
			}

			return 0.0;
		} else  {
			// turn off hazzard-light
			if (m_hazzardON) {
				TransmitBoolValue(&m_oHazzardLightOutput, false);
				m_hazzardON = false;
			}
			// turnRight-light switch off while hazzard light is turned on
			m_turnRightON = false;

			m_isParking = false;
			if (!m_finishParking) {
				LOG_INFO(cString::Format("CPF: Send signal finished") );
				TransmitBoolValue(&m_oFinishParkingOutput, true);
				m_finishParking = true;
			}
			return 0.0;
		}
        break;}
    case 2: {
		if (fTime>=0 && fTime<fSwitchingPoints[1]) { 
			return 60;  
		} else if (fTime<fSwitchingPoints[3]*FACTOR_LAST_PART+0.26) {
			return 120;  
		} else {
			return 90; 
		}
        break;}
    }

    //should never happen (only if valueID is not 1 or 2:
    return m_f32DefaultValue1;
}

tResult CrossParkingFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
    	RETURN_IF_POINTER_NULL(pMediaSample);

		if (&m_iStartParking == pSource) {
			
			tBool bValue = tFalse;

			{
				__adtf_sample_read_lock_mediadescription(m_pStartParkingInput ,pMediaSample, pCoder);    

				pCoder->GetID("bValue", m_szIDBoolValueStartParkingInput);
				pCoder->GetID("ui32ArduinoTimestamp",  m_szIDTimestampStartParkingInput);
 		
				pCoder->Get( m_szIDBoolValueStartParkingInput, (tVoid*)&bValue);         
			}

			if (bValue) LOG_INFO(cString::Format("PPF: Start Cross Parking"));
			else 		LOG_INFO(cString::Format("PPF: Shutdown Cross Parking"));

			crossParking(bValue);
			
		}
	}
    

    RETURN_NOERROR;
}


tResult CrossParkingFilter::TransmitBoolValue(cOutputPin* oPin, bool value)
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

