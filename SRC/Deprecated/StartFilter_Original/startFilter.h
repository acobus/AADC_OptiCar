#ifndef _START_FILTER_H_
#define _START_FILTER_H_

#define OID_START_FILTER "adtf.start_filter"


//*************************************************************************************************
class cStartFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_START_FILTER, "Start Filter", adtf::OBJCAT_DataFilter);

protected:

	cInputPin m_iStartStruct;
	cInputPin m_iDistance;

	cOutputPin m_oStopStruct;
	cOutputPin m_oVelocity;

public:
    cStartFilter(const tChar* __info);
    virtual ~cStartFilter();

private:

	// DistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput; 
	tBufferID m_szIDTimestampDistanceInput; 
	
	// VelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pVelocityOutput;
	tBufferID m_szIDVelocityOutput; 
	tBufferID m_szIDTimestampVelocityOutput; 

	// StartStruct (Input)
	cObjectPtr<IMediaTypeDescription> m_pStartStructInput;
	tBufferID m_szIDVelocityStructInput; 
	tBufferID m_szIDDistanceStructInput; 
	tBufferID m_szIDBoolStructInput;
	tBool m_pStartStructInputSet;

	// StopStruct (Output)
	cObjectPtr<IMediaTypeDescription> m_pStopStructOutput;
	tBufferID m_szIDVelocityStructOutput; 
	tBufferID m_szIDDistanceStructOutput; 
	tBufferID m_szIDBoolStructOutput;
	tBool m_pStopStructOutputSet;

	// stopping mode active
	tBool m_startActive;
	// velocity (max)
	tFloat32 m_velocity;
	// distance
	tFloat32 m_distanceTotal;
	// distance offset
	tFloat32 m_distanceOffset; 

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

	tResult ProcessSpeed(IMediaSample* pMediaSample);
	tResult ProcessStartStruct(IMediaSample* pMediaSample);
	tResult TransmitStruct();

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif 
