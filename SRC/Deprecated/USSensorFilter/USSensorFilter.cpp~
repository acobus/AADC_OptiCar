/*
 * Date 13.02.16
 */
 

#include <math.h>
#include "stdafx.h"
#include "USSensorFilter.h"
#include <fstream>



ADTF_FILTER_PLUGIN("US Sensor Filter", OID_US_SENSOR_FILTER, cUSSensorFilter);


cUSSensorFilter::cUSSensorFilter(const tChar* __info):cFilter(__info)
{
	SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

}

cUSSensorFilter::~cUSSensorFilter()
{

}

tResult cUSSensorFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {	
		cObjectPtr<IMediaDescriptionManager> pDescManager;
 		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             	IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             	(tVoid**)&pDescManager,
                                             	__exception_ptr));



	
        //get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

 	
        // US rear middle
	RETURN_IF_FAILED(m_iUltraSonicBack.Create("Sensor_middle_rear", pTypeSignalValue, static_cast <IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonicBack));

        // US right (side)
	RETURN_IF_FAILED(m_iUltraSonicRight.Create("Sensor_right", pTypeSignalValue, static_cast <IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonicRight));

        // US front middle 
	RETURN_IF_FAILED(m_iUltraSonicFrontMiddle.Create("Sensor_front_middle", pTypeSignalValue, static_cast <IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_iUltraSonicFrontMiddle));


        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSFrontMiddleInput));	
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSRightInput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSBackInput));
      

    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
		// ID not set yet
		m_bIDsUSBackSet         = tFalse;
		m_bIDsUSRightSet        = tFalse;
		m_bIDsUSFrontMiddleSet	= tFalse;
    	m_RearMiddleTimeStamp = _clock->GetStreamTime();
		m_SideRightTimeStamp = m_RearMiddleTimeStamp;
		m_FrontMiddleTimeStamp = m_RearMiddleTimeStamp;

		// last values
		m_lastRearMiddle = 4.30;
		m_lastSideRight = 4.30;
		m_lastFrontMiddle = 4.30;

		m_RearMiddleTimeDiff = 0.0;
		m_SideRightTimeDiff = 0.0;
		m_FrontMiddleTimeDiff = 0.0;
    }

    RETURN_NOERROR;
}

tResult cUSSensorFilter::Shutdown(tInitStage eStage, __exception)
{
   
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}



tResult cUSSensorFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}


tResult cUSSensorFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {        
        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput)
        {


	if ((m_RearMiddleTimeDiff/1000000.0)>0.3){
		//sende letzten Wert
	}

           /* //create new media sample for steering controller	
            {                
                cObjectPtr<IMediaSample> pMediaSampleValue1;	
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

                //allocate memory with the size given by the descriptor	
                cObjectPtr<IMediaSerializer> pSerializer1;	
                m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer1);
                tInt nSize = pSerializer1->GetDeserializedSize();	
                RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));	
                tFloat32 value1 = getCurrentValue(_clock->GetStreamTime(),1);
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


                RETURN_IF_FAILED(m_SpeedOutput.Transmit(pMediaSampleValue1));
            }*/
           
        }

    }
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult cUSSensorFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}




tResult cUSSensorFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

	if (pSource == &m_iUltraSonicRight){
		ProcessUltrasonicRightInput(pMediaSample);
	} else if (pSource == &m_iUltraSonicBack) {
		ProcessUltrasonicBackInput(pMediaSample);
	} else if (pSource == &m_iUltraSonicFrontMiddle) {
		ProcessUltrasonicFrontMiddleInput(pMediaSample);
	}
}

    RETURN_NOERROR;
}



tResult cUSSensorFilter::ProcessUltrasonicBackInput(IMediaSample* pMediaSample){

	{
		__adtf_sample_read_lock_mediadescription(m_pUSBackInput,pMediaSample, pCoder);

		if (!m_bIDsUSBackSet) {
			pCoder->GetID("f32Value", m_szIDUSBackInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampUSBackInput);
			m_bIDsUSBackSet = tTrue;
		}

		pCoder->Get(m_szIDUSBackInput, (tVoid*)&m_lastRearMiddle);
	}

	tUInt32 fTime = _clock->GetStreamTime();

	m_RearMiddleTimeDiff = fTime - m_RearMiddleTimeStamp;

	m_RearMiddleTimeStamp = fTime;

	fstream f;
	f.open("ValueUSBack.dat",ios::out|ios::app);
	f << "RearMiddle, " << m_lastRearMiddle << " , " << m_RearMiddleTimeDiff/1000000.0 << "\n";
	f.close();
       

	RETURN_NOERROR;
}

tResult cUSSensorFilter::ProcessUltrasonicRightInput(IMediaSample* pMediaSample){

	{
		__adtf_sample_read_lock_mediadescription(m_pUSRightInput,pMediaSample, pCoder);

		if (!m_bIDsUSRightSet) {
			pCoder->GetID("f32Value", m_szIDUSRightInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampUSRightInput);
			m_bIDsUSRightSet = tTrue;
		}

		pCoder->Get(m_szIDUSRightInput, (tVoid*)&m_lastSideRight);
	}

	tUInt32 fTime = _clock->GetStreamTime();

	m_SideRightTimeDiff = fTime - m_SideRightTimeStamp;

	m_SideRightTimeStamp = fTime;

	fstream f;
	f.open("ValueUSRight.dat",ios::out|ios::app);
	f << "Right, " << m_lastSideRight << " , " << m_SideRightTimeDiff/1000000.0 << "\n";
	f.close();
       

	RETURN_NOERROR;
}


tResult cUSSensorFilter::ProcessUltrasonicFrontMiddleInput(IMediaSample* pMediaSample){

	{
		__adtf_sample_read_lock_mediadescription(m_pUSFrontMiddleInput,pMediaSample, pCoder);

		if (!m_bIDsUSFrontMiddleSet) {
			pCoder->GetID("f32Value", m_szIDUSFrontMiddleInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampUSFrontMiddleInput);
			m_bIDsUSFrontMiddleSet = tTrue;
		}

		pCoder->Get(m_szIDUSFrontMiddleInput, (tVoid*)&m_lastFrontMiddle);
	}

	tUInt32 fTime = _clock->GetStreamTime();

	m_FrontMiddleTimeDiff = fTime - m_FrontMiddleTimeStamp;

	m_FrontMiddleTimeStamp = fTime;

	fstream f;
	f.open("ValueUSFrontMiddle.dat",ios::out|ios::app);
	f << "FrontMiddle, " << m_lastFrontMiddle << " , " << m_FrontMiddleTimeDiff/1000000.0 << "\n";
	f.close();
       

	RETURN_NOERROR;
}




