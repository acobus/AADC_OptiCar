//date 17.03.16


// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include <math.h>
#include "stdafx.h"
#include "CalcAreaFilter.h"
#include <fstream>

#define SPEED 0.8;
#define MIN_SPEED 0.4;
#define STRETCH_FACTOR 0.25;


ADTF_FILTER_PLUGIN("CalcAreaFilter", OID_ADTF_CALC_AREA_FILTER, CalcAreaFilter)

    CalcAreaFilter::CalcAreaFilter(const tChar* __info) : cFilter(__info)
{
    

}

CalcAreaFilter::~CalcAreaFilter()
{
}

tResult CalcAreaFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
            cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);        
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		    // Media Description LaneTracking Velocity
		    tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
		    RETURN_IF_POINTER_NULL(LTVelocityDescValue);
		    cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			// Get description for bool values
			tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
			RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
			cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    		RETURN_IF_FAILED(m_iStart.Create("Start", pTypeBoolSignalValue, this));
    		RETURN_IF_FAILED(RegisterPin(&m_iStart));

    		RETURN_IF_FAILED(m_iShowResult.Create("ShowResult", pTypeBoolSignalValue, this));
    		RETURN_IF_FAILED(RegisterPin(&m_iShowResult));

            RETURN_IF_FAILED(m_iInputDistanceOverall.Create("distance_overall", pTypeSignalValue, this));
            RETURN_IF_FAILED(RegisterPin(&m_iInputDistanceOverall));

            RETURN_IF_FAILED(m_oOutputSpeed.Create("speed", pTypeLTVelocityValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeed));

            RETURN_IF_FAILED(m_oStartTickFilter.Create("startTickFilter", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oStartTickFilter));

			RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionStart));
			RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionShowResult));
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pInputDistanceOverall));
            RETURN_IF_FAILED(pTypeLTVelocityValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSpeed));
			RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartTickFilter));

        }
        else if (eStage == StageNormal)
        {
			InitialState();

    		m_bIDsSetBoolValueStart = tFalse;
			m_bIDsDistanceSet = tFalse; 
		   	m_bIDsSpeedSet = tFalse;  
			m_bStartStartTickFilterSet = tFalse;
			m_active = tFalse;
			m_showResultActive = tFalse;
			m_area = 0.0;
			m_distanceOffset = 0.0;
			m_drivenDistance = 0.0;
        }
        else if(eStage == StageGraphReady)
        {
        }

        RETURN_NOERROR;
}

tResult CalcAreaFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {
        if (pSource == &m_iStart)
        {
			tBool aux;

			{
				__adtf_sample_read_lock_mediadescription(m_pDescriptionStart ,pMediaSample, pCoder);    

				pCoder->GetID("bValue", m_szIDBoolValueStart);
				pCoder->GetID("ui32ArduinoTimestamp",  m_szIDArduinoTimestampStart);
 		
				pCoder->Get( m_szIDBoolValueStart, (tVoid*)&aux);
			}

			if (!m_active) {
				m_active = aux;
				if (m_active){
					TransmitBoolValue(tFalse); // deactivate tickfilter
					calcArea();
				}
			} else if (!aux) {
				InitialState();
			}

        } else if (pSource == &m_iShowResult){
			tBool aux;
			{
				__adtf_sample_read_lock_mediadescription(m_pDescriptionStart ,pMediaSample, pCoder);    

				pCoder->Get("bValue", (tVoid*)&aux);
			}
			showResult(aux);
		} else if (pSource == &m_iInputDistanceOverall && m_showResultActive){
			ProcessSpeed(pMediaSample);
		}
       
    }
    RETURN_NOERROR;
}

tResult CalcAreaFilter::ProcessSpeed(IMediaSample* pMediaSample){

	tFloat32 fDistance = 0.0;
	tFloat32 fSpeed = 0.0;

	{
		__adtf_sample_read_lock_mediadescription(m_pInputDistanceOverall,pMediaSample, pCoder);

		if (!m_bIDsDistanceSet) {
			pCoder->GetID("f32Value", m_szIDDistanceF32Value);
			m_bIDsDistanceSet = tTrue;
		}

		pCoder->Get(m_szIDDistanceF32Value, (tVoid*)&fDistance);
	}

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = fDistance;
	}

	fDistance -= m_distanceOffset;
	m_drivenDistance = fDistance;

	if (m_drivenDistance < m_area - 0.2){
		fSpeed = SPEED;
	} else if (m_drivenDistance < m_area){
		fSpeed = MIN_SPEED;
	} else {
		fSpeed = 0.0;
		InitialState();
		TransmitBoolValue(tFalse);
	}

	TransmitControl(fSpeed);
	

	RETURN_NOERROR;
}

tResult CalcAreaFilter::TransmitControl(tFloat32 fSpeed){

	  //create new media sample for speed controller
	  {
		   cObjectPtr<IMediaSample> pMediaSampleValue1;
		  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSerializer> pSerializer1;
        m_pDescriptionSpeed->GetMediaSampleSerializer(&pSerializer1);
        tInt nSize = pSerializer1->GetDeserializedSize();
        RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));
        {   // focus for sample write lock
            //write date to the media sample with the coder of the descriptor
            __adtf_sample_write_lock_mediadescription(m_pDescriptionSpeed,pMediaSampleValue1,pCoder);
            // get the IDs for the items in the media sample
            if(!m_bIDsSpeedSet)
            {
				pCoder->GetID("fMinVel", m_szIDMinVelocityOutput);
				pCoder->GetID("fMaxVel", m_szIDMaxVelocityOutput);
				m_bIDsSpeedSet = tTrue;
	    	}

			pCoder->Set(m_szIDMinVelocityOutput, (tVoid*)&(fSpeed));
			pCoder->Set(m_szIDMaxVelocityOutput, (tVoid*)&(fSpeed));
        }

		//LOG_INFO(cString::Format("CAF: Transmited speed = %f", fSpeed) );

        //transmit media sample over output pin

        RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));
        RETURN_IF_FAILED(m_oOutputSpeed.Transmit(pMediaSampleValue1));
	}
			
			
	RETURN_NOERROR;

}



tResult CalcAreaFilter::showResult(tBool aux) {
	
	if (m_area <= 0.0){
		LOG_INFO(cString::Format("CAF: no valid area"));
		RETURN_NOERROR;
	}

	m_showResultActive = aux;

	if (aux){
		TransmitControl(1.0); //start car
	}


	RETURN_NOERROR;
}

tResult CalcAreaFilter::calcArea() {
    tFloat32 wheelBase = 0.363;

	m_dist_Left.clear();
	m_dist_Right.clear();

	m_dist_Left.reserve(5000);
	m_dist_Right.reserve(5000);

	m_dist_Left.push_back(0.0);
	m_dist_Right.push_back(wheelBase);

	// dont change this!
	fstream f1, f2;
	f1.open("Dist_Left.dat",ios::in);
	f2.open("Dist_Right.dat",ios::in);

	string aux1, aux2;
	
	while (getline(f1, aux1) && getline(f2, aux2)) {
		replace(aux1.begin(),aux1.end(),'.',',');
		replace(aux2.begin(),aux2.end(),'.',',');

		double auxL = atof(aux1.c_str());
		double auxR = atof(aux2.c_str());

		//LOG_INFO(cString::Format("CAF: %f, %f", auxL, auxR));

		if (auxL != 0.0 || auxR != 0.0) {

			m_dist_Left.push_back(auxL);
			m_dist_Right.push_back(auxR);
		}
	}

	f1.close();
	f2.close();

	// calc position

	size_t last = m_dist_Right.size();

	tFloat32 axis = (m_dist_Right[last-1]-m_dist_Left[last-1])/(2*M_PI);

	// right -> x, left -> y

	tFloat32 auxL = 0.0, auxR = 0.0;

	tFloat32 xOld = 0.0;
	tFloat32 yOld = 0.0;

	tFloat32 stretch_factor2 = 1.0;//0.93;
	LOG_INFO(cString::Format("CAF: last = %i", last));

	for (size_t i=1; i<last; i++){
		
		tFloat32 rotAngle = (m_dist_Right[i]-m_dist_Left[i])/axis;
		tFloat32 diff = (m_dist_Right[i]-auxR+m_dist_Left[i]-auxL)/2.0*stretch_factor2;

		auxR = m_dist_Right[i];
		auxL = m_dist_Left[i];

		m_dist_Right[i] = xOld + cos(rotAngle)*(diff + wheelBase); // front axis
		m_dist_Left[i] = yOld + sin(rotAngle)*(diff + wheelBase);

		xOld += cos(rotAngle)*diff; // rear axis
		yOld += sin(rotAngle)*diff;
	}

	m_dist_Left.push_back(0.0);
	m_dist_Right.push_back(wheelBase);

	
	// calc area
	tFloat32 A = 0.0;

	for (size_t i=0; i<last; i++){
		A += (m_dist_Left[i]+m_dist_Left[i+1]) * (m_dist_Right[i]-m_dist_Right[i+1]);
	}

	A = 0.5*fabs(A);
	A *= 0.93;
	LOG_INFO(cString::Format("CAF: Area = %f", A));
	
	// scale area!
	m_area = A*STRETCH_FACTOR;

	RETURN_NOERROR;
}

tResult CalcAreaFilter::InitialState(){

	m_active = tFalse;
	m_showResultActive = tFalse;
	m_area = 0.0;
	m_distanceOffset = 0.0;
	m_drivenDistance = 0.0;

	RETURN_NOERROR;
}

tResult CalcAreaFilter::TransmitBoolValue(tBool state){

	tUInt32 nTimeStamp 	= 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStartTickFilter->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


	{
		__adtf_sample_write_lock_mediadescription(m_pStartTickFilter, pMediaSample, pCoderOutput); 

		if(!m_bStartStartTickFilterSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueStartTickFilter);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampStartTickFilter);
			m_bStartStartTickFilterSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueStartTickFilter, (tVoid*)&(state));
		pCoderOutput->Set(m_szIDTimestampStartTickFilter, (tVoid*)&nTimeStamp);
	
	}

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oStartTickFilter.Transmit(pMediaSample);

	RETURN_NOERROR;
}


