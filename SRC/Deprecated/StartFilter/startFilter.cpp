#include <math.h>
#include "stdafx.h"
#include "startFilter.h"


ADTF_FILTER_PLUGIN("Start Filter", OID_START_FILTER, cStartFilter);


cStartFilter::cStartFilter(const tChar* __info):cFilter(__info), m_startActive(tFalse)
{
	m_distanceOffset = 0.0;
}

cStartFilter::~cStartFilter()
{

}

tResult cStartFilter::Init(tInitStage eStage, __exception)
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

		// MediaDescription of StopFilter-Struct
		tChar const* strDescStopStruct = pDescManager->GetMediaDescription("tStopStruct");
		RETURN_IF_POINTER_NULL(strDescStopStruct);
		cObjectPtr<IMediaType> pTypeStopStruct = new cMediaType(0, 0, 0, "tStopStruct", strDescStopStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));

		// VelocityInput
		RETURN_IF_FAILED(m_iVelocity.Create("RealVelocity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_iVelocity));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pVelocityInput));

		// StartFilter Structure
		RETURN_IF_FAILED(m_iStartStruct.Create("StartStruct", pTypeStopStruct, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStartStruct));
		RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartStructInput));

		// StopFilter Structure
		RETURN_IF_FAILED(m_oStopStruct.Create("StopStruct", pTypeStopStruct, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopStruct));
		RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopStructOutput));

		// VelocityOutput
		RETURN_IF_FAILED(m_oVelocity.Create("Velocity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oVelocity));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pVelocityOutput));

    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
		// ID not set yet
		m_pStopStructOutputSet = tFalse;
		m_pStartStructInputSet = tFalse;
    }

    RETURN_NOERROR;
}

tResult cStartFilter::Shutdown(tInitStage eStage, __exception)
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

tResult cStartFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

		if (pSource == &m_iStartStruct){
			ProcessStartStruct(pMediaSample);
        } else if (pSource == &m_iDistance && m_startActive){
			ProcessSpeed(pMediaSample);
		} else if (pSource == &m_iVelocity && m_startActive){
			ProcessRealVelocity(pMediaSample);
		}
    }

    RETURN_NOERROR;
}

tResult cStartFilter::ProcessStartStruct(IMediaSample* pMediaSample){

	tFloat32 	velocity;
	tFloat32 	distance;
	tBool 		bValue;

	{
		__adtf_sample_read_lock_mediadescription(m_pStartStructInput, pMediaSample, pCoder);    

		if (!m_pStartStructInputSet){
			pCoder->GetID("fVelocity", m_szIDVelocityStructInput);
			pCoder->GetID("fDistance", m_szIDDistanceStructInput);
			pCoder->GetID("bStart", m_szIDBoolStructInput);
			m_pStartStructInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDVelocityStructInput, (tVoid*)&velocity);
		pCoder->Get(m_szIDDistanceStructInput, (tVoid*)&distance);
		pCoder->Get(m_szIDBoolStructInput, (tVoid*)&bValue);        
	}

	m_startActive = bValue;
	m_velocity = velocity;
	m_distanceTotal = distance;

	RETURN_NOERROR;
}

tResult cStartFilter::ProcessRealVelocity(IMediaSample* pMediaSample){

    tFloat32 fRealVelocity = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pVelocityInput,pMediaSample, pCoder);
		
		pCoder->GetID("f32Value", m_szIDVelocityInput);
		pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampVelocityInput);    		
		pCoder->Get(m_szIDVelocityInput, (tVoid*)&(fRealVelocity));
	}
	m_realVelocity=fRealVelocity;

	RETURN_NOERROR;
}


tResult cStartFilter::ProcessSpeed(IMediaSample* pMediaSample){
	
    tFloat32 fDistance = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);
		
		pCoder->GetID("f32Value", m_szIDDistanceInput);
		pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput);    		
		pCoder->Get(m_szIDDistanceInput, (tVoid*)&(fDistance));
	}
	// calc Speed

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = fDistance;
		fDistance += 5e-1;
		//LOG_INFO(cString::Format("offset %f",m_distanceOffset));
	}
	
	fDistance -= m_distanceOffset;
	tFloat32 faux = fDistance/(m_distanceTotal*0.5);

	//LOG_INFO(cString::Format("faux %f",faux));

	tFloat32 fSpeed = sqrt(faux)*m_velocity;
	//LOG_INFO(cString::Format("Speed %f",fSpeed));

	if (faux > 1.0 + 1e-9) {
		m_startActive = tFalse;
		m_distanceOffset = 0.0;
		
	}

	//fSpeed = m_velocity;

	tUInt32 nTimeStamp = 0;

    cObjectPtr<IMediaSample> pNewMediaSample;
    AllocMediaSample((tVoid**)&pNewMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pVelocityOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pNewMediaSample->AllocBuffer(nSize);

    cObjectPtr<IMediaCoder> pCoderOutput;

    m_pVelocityOutput->WriteLock(pNewMediaSample, &pCoderOutput);

    pCoderOutput->Set("f32Value", (tVoid*)&(fSpeed));
    pCoderOutput->Set("ui32ArduinoTimeStamp", (tVoid*)&nTimeStamp);

	pNewMediaSample->SetTime(pNewMediaSample->GetTime());
    m_pVelocityOutput->Unlock(pCoderOutput);

    m_oVelocity.Transmit(pNewMediaSample);

	if (!m_startActive) {
		LOG_INFO(cString::Format("speed %f",fSpeed));
		TransmitStruct();
	}

	RETURN_NOERROR;
}

tResult cStartFilter::TransmitStruct() {
	cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
	m_pStopStructOutput->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
     

	tFloat32 	velocity = m_realVelocity;
	tFloat32 	distance = m_distanceTotal/2.0; 
    tBool bValue = tTrue;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pStopStructOutput, pMediaSample, pCoderOutput);    

        // set the id if not already done
        if(!m_pStopStructOutputSet)
        {
			pCoderOutput->GetID("fVelocity", m_szIDVelocityStructOutput);
			pCoderOutput->GetID("fDistance", m_szIDDistanceStructOutput);
			pCoderOutput->GetID("bStart", m_szIDBoolStructOutput);

            m_pStopStructOutputSet = tTrue;
        }      
            
        // set value from sample
        pCoderOutput->Set(m_szIDVelocityStructOutput, (tVoid*)&velocity);     
   		pCoderOutput->Set(m_szIDDistanceStructOutput, (tVoid*)&distance); 
		pCoderOutput->Set(m_szIDBoolStructOutput, (tVoid*)&bValue); 
    }

	LOG_INFO(cString::Format("speed %f",m_velocity));
	LOG_INFO(cString::Format("realSpeed %f",m_realVelocity));

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStopStruct.Transmit(pMediaSample);
	RETURN_NOERROR;
}

