/*
 * Date 12.01.16
 */
#ifndef _TEST_STOP_FILTER_PARKING_H_
#define _TEST_STOP_FILTER_PARKING_H_

#define OID_TEST_STOP_FILTER_PARKING "adtf.test_stop_filter_parking"


//*************************************************************************************************
class cTestStopFilterParking : public adtf::cFilter
{
    ADTF_FILTER(OID_TEST_STOP_FILTER_PARKING, "Test Stop Filter for Parking", adtf::OBJCAT_DataFilter);

protected:
	cInputPin		m_iDistance;
	cInputPin 		m_iStartBool;
	cOutputPin 		m_oParkingSpot;
	cOutputPin		m_oStopStruct;

public:
    cTestStopFilterParking(const tChar* __info);
    virtual ~cTestStopFilterParking();

private:

	// OverallDistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBufferID m_szIDTimestampDistanceInput;
	tBool m_bIDsDistanceSet;
	
	// Bool Output
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
	tBufferID m_szIDBoolValueOutput;
	tBufferID m_szIDArduinoTimestampOutput;
	tBool m_bIDsBoolValueOutput;

	// DistanceToParkingSpotOutput
	cObjectPtr<IMediaTypeDescription> m_pDistanceSpotOutput;
	tBufferID m_szIDDistanceSpotOutput;
	tBufferID m_szIDTimestampDistanceSpotOutput;
	tBool m_bIDsDistanceSpotSet;
	
	// StopStruct (Output)
	cObjectPtr<IMediaTypeDescription> m_pStopStructOutput;
	tBufferID m_szIDVelocityStructOutput; 
	tBufferID m_szIDDistanceStructOutput; 
	tBufferID m_szIDBoolStructOutput;
	tBool m_pStopStructOutputSet;


	tFloat32 m_distanceOffset;
    //distances to free parking spots
    std::vector<float> spot;
    
    tUInt32 m_spotCounter;
    tBool m_filterActive;

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    tResult SendStopStruct();
    tResult SendSpotDistance(tFloat32 distance);
    tResult ProcessSpotDistance(IMediaSample* pMediaSample);
    tResult StartFilter();
    tResult SetToInitialState();

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif 
