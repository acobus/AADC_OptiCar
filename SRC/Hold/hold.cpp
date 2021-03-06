/*
 * Date 20.12.2015
 */ 

#include "stdafx.h"
#include "hold.h"


ADTF_FILTER_PLUGIN("Hold Filter", OID_HOLD_FILTER, cHold);


cHold::cHold(const tChar* __info):cFilter(__info)
{

}

cHold::~cHold()
{

}

tResult cHold::Init(tInitStage eStage, __exception)
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

		//get description for bool values
    	tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
    	RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
    	cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
		//get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// StateHoldInput
		RETURN_IF_FAILED(m_iStateHold.Create("State_Hold", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStateHold));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateHoldInput));

		// ZeroVelocityOutput
		RETURN_IF_FAILED(m_oZeroVelocity.Create("ZeroVelocity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oZeroVelocity));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pZeroVelocityOutput));
		
		// create Pin for Breaklight (Output)
		RETURN_IF_FAILED(m_oBreaklight.Create("Breaklight", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oBreaklight));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBreaklightOutput));

    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
    	
    	m_HoldActive 			= tFalse;
    	m_StateChanged 			= tFalse;

    	m_bStateHoldSet 		= tFalse;
    	m_bVelocityOutputSet 	= tFalse;
    	m_bBreaklightOutputSet 	= tFalse;
    }

    RETURN_NOERROR;
}

tResult cHold::Shutdown(tInitStage eStage, __exception)
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

tResult cHold::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

		if (pSource == &m_iStateHold)
        {
			HoldWork(pMediaSample);
        }
    }

    RETURN_NOERROR;
}

tResult cHold::HoldWork(IMediaSample* pMediaSample){

	tBool bValue = tFalse;
	
	{
    	__adtf_sample_read_lock_mediadescription(m_pBoolStateHoldInput ,pMediaSample, pCoder);    
           
    	if (!m_bStateHoldSet){
    		pCoder->GetID("bValue", m_szIDBoolValueStateHoldInput);  	
    		m_bStateHoldSet 	= tTrue;
    	}
    	
		pCoder->Get(m_szIDBoolValueStateHoldInput, (tVoid*)&bValue);         
	}
	
	if (m_HoldActive != bValue){
		m_StateChanged 	= tTrue;
		m_HoldActive 	= bValue;
	}

	// Only send booleans once
	if (m_StateChanged){
		m_StateChanged 	= tFalse;
		if (m_HoldActive){ 	
			transmitHold();
			transmitBreaklight(tTrue);
		} else transmitBreaklight(tFalse);
	}

	RETURN_NOERROR;
}

tResult cHold::transmitHold(){

    tUInt32 nTimeStamp = 0;
    tFloat32 fZeroSpeed = 0.0;

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pZeroVelocityOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    
	//write date to the media sample with the coder of the descriptor
	{
		__adtf_sample_write_lock_mediadescription(m_pZeroVelocityOutput, pMediaSample, pCoderOutput);    

		// set the id if not already done
		if(!m_bVelocityOutputSet)
		{
			pCoderOutput->GetID("f32Value", m_szIDZeroVelocityOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampZeroVelocityOutput);
			m_bVelocityOutputSet = tTrue;
		}      

		// set value from sample
		pCoderOutput->Set(m_szIDZeroVelocityOutput, (tVoid*)&fZeroSpeed);     
		pCoderOutput->Set(m_szIDTimestampZeroVelocityOutput, (tVoid*)&(nTimeStamp));     
	}

    pMediaSample->SetTime(pMediaSample->GetTime());

    m_oZeroVelocity.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * this function sends the breakinglight signal to the baseconfig.
 */
tResult cHold::transmitBreaklight(tBool bValue){

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
		pCoderOutput->Set(m_szIDTimestampBreaklightOutput, (tVoid*)&(ui32TimeStamp));     
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	//transmit media sample over output pin
	m_oBreaklight.Transmit(pMediaSample);

	RETURN_NOERROR;
}
