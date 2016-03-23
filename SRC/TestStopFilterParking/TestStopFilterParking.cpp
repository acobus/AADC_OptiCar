/*
 * Date 14.01.16
 */


#include <math.h>
#include "stdafx.h"
#include "TestStopFilterParking.h"
#include <fstream>

ADTF_FILTER_PLUGIN("Test Stop Filter for Parking", OID_TEST_STOP_FILTER_PARKING, cTestStopFilterParking);

cTestStopFilterParking::cTestStopFilterParking(const tChar* __info) :
		cFilter(__info) {

}

cTestStopFilterParking::~cTestStopFilterParking() {

}

tResult cTestStopFilterParking::Init(tInitStage eStage, __exception) {
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED (cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst) {
		cObjectPtr < IMediaDescriptionManager > pDescManager;
		RETURN_IF_FAILED(
				_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
						IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
						(tVoid**) &pDescManager, __exception_ptr));

		//get description for sensor data pins
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription(
				"tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		//get mediatype for ultrasonic sensor data pins
		cObjectPtr < IMediaType > pTypeSignalValue = new cMediaType(0, 0, 0,
				"tSignalValue", strDescSignalValue,
				IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// MediaDescription of StopFilter-Struct
		tChar const* strDescStopStruct = pDescManager->GetMediaDescription(
				"tStopStruct");
		RETURN_IF_POINTER_NULL(strDescStopStruct);
		cObjectPtr < IMediaType > pTypeStopStruct = new cMediaType(0, 0, 0,
				"tStopStruct", strDescStopStruct,
				IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for bool values
		tChar const * strDescBoolSignalValue =
				pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
		cObjectPtr < IMediaType > pTypeBoolSignalValue = new cMediaType(0, 0, 0,
				"tBoolSignalValue", strDescBoolSignalValue,
				IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// DistanceInput
		RETURN_IF_FAILED(
				m_iDistance.Create("Distance_overall", pTypeSignalValue,
						static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_iDistance));

		// Parking Spot Output
		RETURN_IF_FAILED(
				m_oParkingSpot.Create("Distance_parking_spot", pTypeSignalValue,
						static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oParkingSpot));

		// StopFilter Structure
		RETURN_IF_FAILED(
				m_oStopStruct.Create("StopStruct", pTypeStopStruct,
						static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopStruct));

		// StopFilter Structure
		RETURN_IF_FAILED(
				m_iStartBool.Create("StartBool", pTypeBoolSignalValue,
						static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStartBool));

		RETURN_IF_FAILED(
				pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
						(tVoid**) &m_pDistanceInput));
		RETURN_IF_FAILED(
				pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
						(tVoid**) &m_pDistanceSpotOutput));
		RETURN_IF_FAILED(
				pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION,
						(tVoid**) &m_pStopStructOutput));
		RETURN_IF_FAILED(
				pTypeBoolSignalValue->GetInterface(
						IID_ADTF_MEDIA_TYPE_DESCRIPTION,
						(tVoid**) &m_pDescriptionBool));

	} else if (eStage == StageNormal) {

	} else if (eStage == StageGraphReady) {
		// ID not set yet
		m_bIDsDistanceSpotSet = tFalse;
		m_bIDsBoolValueOutput = tFalse;
		m_bIDsDistanceSet = tFalse;
		m_pStopStructOutputSet = tFalse;
		SetToInitialState();
	}

	RETURN_NOERROR;
}

tResult cTestStopFilterParking::Shutdown(tInitStage eStage, __exception) {

	if (eStage == StageGraphReady) {
	} else if (eStage == StageNormal) {
	} else if (eStage == StageFirst) {
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cTestStopFilterParking::OnPinEvent(IPin* pSource, tInt nEventCode,
		tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		RETURN_IF_POINTER_NULL(pMediaSample);

		if (pSource == &m_iStartBool) {
			StartFilter();
		} else if (pSource == &m_iDistance && m_filterActive) {
			ProcessSpotDistance(pMediaSample);
		}
	}
	RETURN_NOERROR;
}

tResult	cTestStopFilterParking::SendStopStruct()
	{

		tFloat32 velocityZero = 0.8;
		tFloat32 distanceZero = spot.at(m_spotCounter);
		++m_spotCounter;
		tBool stopActive = tTrue;

		cObjectPtr < IMediaSample > pMediaSample;
		AllocMediaSample((tVoid**) &pMediaSample);

		cObjectPtr < IMediaSerializer > pSerializer;
		m_pStopStructOutput->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		pMediaSample->AllocBuffer(nSize);

		{
			__adtf_sample_read_lock_mediadescription(m_pStopStructOutput,
					pMediaSample, pCoder);

			if (!m_pStopStructOutputSet) {
				pCoder->GetID("fVelocity", m_szIDVelocityStructOutput);
				pCoder->GetID("fDistance", m_szIDDistanceStructOutput);
				pCoder->GetID("bStart", m_szIDBoolStructOutput);
				m_pStopStructOutputSet = tTrue;
			}

			pCoder->Set(m_szIDVelocityStructOutput, (tVoid*) &velocityZero);
			pCoder->Set(m_szIDDistanceStructOutput, (tVoid*) &distanceZero);
			pCoder->Set(m_szIDBoolStructOutput, (tVoid*) &stopActive);
		}

		m_oStopStruct.Transmit(pMediaSample);

		RETURN_NOERROR;
	}

tResult	cTestStopFilterParking::ProcessSpotDistance(IMediaSample * pMediaSample)
	{

		tFloat32 fDistance = 0.0;
		{
			__adtf_sample_read_lock_mediadescription(m_pDistanceSpotOutput,
					pMediaSample, pCoder);

			if (!m_bIDsDistanceSpotSet) {
				pCoder->GetID("f32Value", m_szIDDistanceSpotOutput);
				m_bIDsDistanceSpotSet = tTrue;
			}

			pCoder->Get(m_szIDDistanceSpotOutput, (tVoid*) &fDistance);
		}

		if (m_distanceOffset == 0.0) {
			m_distanceOffset = fDistance;
		}

		fDistance -= m_distanceOffset;

		LOG_INFO(cString::Format("TSFP: fdistance %f, spotSize %i, m_spotCounter %i", fDistance, spot.size(), m_spotCounter)); 

		if (spot.size() > m_spotCounter) {
			if (spot[m_spotCounter] - fDistance < 2.5) {
				SendSpotDistance (spot[m_spotCounter] - fDistance);
				++m_spotCounter;
			}

		} else {
			//runterfahren
			SetToInitialState();
		}

		RETURN_NOERROR;
	}

tResult	cTestStopFilterParking::StartFilter()
	{
		spot.clear();
		spot.push_back(0.70);
		spot.push_back(1.50);
		//spot.push_back(0.7653 + 1.50 + 0.03); //1.98);
		m_filterActive = tTrue;
		SendStopStruct();

		RETURN_NOERROR;
	}

tResult	cTestStopFilterParking::SendSpotDistance(tFloat32 distance) {

		cObjectPtr<IMediaSample> pNewMediaSample;
		AllocMediaSample((tVoid**)&pNewMediaSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistanceSpotOutput->GetMediaSampleSerializer(&pSerializer);
		tInt nSize = pSerializer->GetDeserializedSize();
		pNewMediaSample->AllocBuffer(nSize);

		//write date to the media sample with the coder of the descriptor
		{
			__adtf_sample_write_lock_mediadescription(m_pDistanceSpotOutput, pNewMediaSample, pCoderOutput);

			// set the id if not already done
			if(!m_bIDsDistanceSpotSet)
			{
				pCoderOutput->GetID("f32Value", m_szIDDistanceSpotOutput);
				m_bIDsDistanceSpotSet = tTrue;
			}

			pCoderOutput->Set(m_szIDDistanceSpotOutput, (tVoid*)&(distance));
			

			pNewMediaSample->SetTime(pNewMediaSample->GetTime());
		}

		m_oParkingSpot.Transmit(pNewMediaSample);
		LOG_INFO(cString::Format("TSFP: pin gesendet, distance %f", distance));

		RETURN_NOERROR;

	}

tResult	cTestStopFilterParking::SetToInitialState()
	{
		spot.clear();
		m_spotCounter = 0;
		m_distanceOffset = 0.0;
		m_filterActive = tFalse;

		RETURN_NOERROR;
	}

