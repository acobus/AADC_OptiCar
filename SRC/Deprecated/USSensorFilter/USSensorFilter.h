/*
 * Date 13.02.16
 */
#ifndef _US_SENSOR_FILTER_H_
#define _US_SENSOR_FILTER_H_

#define OID_US_SENSOR_FILTER "adtf._US_Ssensor_Filter"


//*************************************************************************************************
class cUSSensorFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_US_SENSOR_FILTER, "US Sensor Filter", adtf::OBJCAT_DataFilter);

protected:
	cInputPin		m_iUltraSonicRight;
	cInputPin		m_iUltraSonicBack;
	cInputPin		m_iUltraSonicFrontMiddle;

public:
    cUSSensorFilter(const tChar* __info);
    virtual ~cUSSensorFilter();

private:

	// USRightInput
	cObjectPtr<IMediaTypeDescription> m_pUSRightInput;
	tBufferID m_szIDUSRightInput;
	tBufferID m_szIDTimestampUSRightInput;
	tBool m_bIDsUSRightSet;

	// USRearMiddleInput
	cObjectPtr<IMediaTypeDescription> m_pUSBackInput;
	tBufferID m_szIDUSBackInput;
	tBufferID m_szIDTimestampUSBackInput;
	tBool m_bIDsUSBackSet;

	// USFrontMiddleInput
	cObjectPtr<IMediaTypeDescription> m_pUSFrontMiddleInput;
	tBufferID m_szIDUSFrontMiddleInput;
	tBufferID m_szIDTimestampUSFrontMiddleInput;
	tBool m_bIDsUSFrontMiddleSet;




    // timestamps last sample
    tUInt32 m_RearMiddleTimeStamp;
	tUInt32 m_SideRightTimeStamp;
	tUInt32 m_FrontMiddleTimeStamp;

	// last values
	tFloat32 m_lastRearMiddle;
	tFloat32 m_lastSideRight;
	tFloat32 m_lastFrontMiddle;

	tUInt32 m_RearMiddleTimeDiff;
	tUInt32 m_SideRightTimeDiff;
	tUInt32 m_FrontMiddleTimeDiff;

   


protected:
    tResult Init(tInitStage eStage, __exception);
/*! overrides cFilter */
    virtual tResult Start(__exception = NULL);

    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);

    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);   

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

    tResult ProcessUltrasonicRightInput(IMediaSample* pMediaSample);
	tResult ProcessUltrasonicBackInput(IMediaSample* pMediaSample);
	tResult ProcessUltrasonicFrontMiddleInput(IMediaSample* pMediaSample);

 
    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;


    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif 
