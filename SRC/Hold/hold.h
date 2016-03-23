#ifndef _HOLD_FILTER_H_
#define _HOLD_FILTER_H_

#define OID_HOLD_FILTER "adtf.hold_filter"


//*************************************************************************************************
class cHold : public adtf::cFilter
{
    ADTF_FILTER(OID_HOLD_FILTER, "Hold Filter", adtf::OBJCAT_DataFilter);

protected:
    cInputPin    	m_iStateHold;
    cOutputPin    	m_oZeroVelocity;
    cOutputPin 		m_oBreaklight;

public:
    cHold(const tChar* __info);
    virtual ~cHold();

private:
	// StateHoldInput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateHoldInput;
	tBufferID m_szIDBoolValueStateHoldInput; 
	tBool m_bStateHoldSet;
	
	// ZeroVelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pZeroVelocityOutput;
	tBufferID m_szIDZeroVelocityOutput; 
	tBufferID m_szIDTimestampZeroVelocityOutput;
	tBool m_bVelocityOutputSet;
	
	// BreaklightOutput
	cObjectPtr<IMediaTypeDescription> m_pBreaklightOutput;
	tBufferID m_szIDBoolValueBreaklightOutput;
	tBufferID m_szIDTimestampBreaklightOutput;
	tBool m_bBreaklightOutputSet;

	tBool m_HoldActive;
	tBool m_StateChanged;

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

	tResult HoldWork(IMediaSample* pMediaSample);
	tResult transmitHold();
	tResult transmitBreaklight(tBool bValue);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif // _TEMPLATE_PROJECT_FILTER_H_
