#ifndef _CheckPullOut_FILTER_HEADER_
#define _CheckPullOut_FILTER_HEADER_

#define OID_ADTF_CheckPullOut  "adtf.aadc.CheckPullOut"
#define LT_POINTS_ARRAY_SIZE 640

class cCheckPullOut : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CheckPullOut, "CheckPullOut", OBJCAT_Tool, "CheckPullOut", 1, 0, 0, "BFFT GmbH");    
    
protected:
    //Eingang für RGB Bild
    cVideoPin           m_iVideoInputPin;
    cInputPin           m_iStart; 
	cInputPin			m_iUS_FrontCenter;
        
    cOutputPin          m_oGCLOutput;
	cOutputPin			m_oisCrossParking;


public:
            
    cCheckPullOut(const tChar*);
    virtual ~cCheckPullOut();

    // implements cFilter
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult PropertyChanged(const char* strProperty);

public: // implements ISignalProvider
    virtual tResult GetSignalValue(tSignalID nSignalID, tSignalValue* pValue);
    virtual tResult ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate = 0);
    virtual tResult DeactivateSignalEvents(tSignalID nSignalID);

public: // implements IObject
    tResult GetInterface(const tChar* idInterface, tVoid** ppvObject);
    tUInt Ref();
    tUInt Unref();
    tVoid Destroy();

private: // private methods
	tResult check();
    tResult ProcessInputRGB(IMediaSample* pSample, tTimeStamp);
    tResult ProcessFound();
    tResult ProcessOutput();
	tResult ProcessUSSensor();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();
	tResult TransmitIsCrossParking(tBool b);


protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);
public:


private:

	// for stop criteria
	tUInt8		m_iframeCounter;


    // offset of the camera to the mid of the vehicle
    tFloat64    m_f64CamOffset;   

    // active flag to enable driving
    tBool       m_bActive;
        


	// line detection
	cv::Mat		matCutHough;
	cv::Mat		m_matCanny;
	vector<Vec2f> 	m_lines;

	// decide
	int	m_iParallel;
	int	m_iFrontSensor;
	bool m_bGroupRisEmpty;

	tUInt32 m_startSensorTimeStamp;
	tInt32 m_no_car_count;
	tInt32 m_car_count;

    /*! handle for timer for sending actuator values*/
    tHandle m_hTimerOutput;



	// CheckPullOutLotOutput
	cObjectPtr<IMediaTypeDescription> m_pIsCrossParking;
	tBufferID m_szIsCrossParking;
	tBufferID m_szIDTimestampOutput;
	tBool m_bIsCrossParkingSet;


	//US Input
	cObjectPtr<IMediaTypeDescription> m_pSignalUS_FrontCenterInput;
	tBufferID m_szIDFrontCenterUltraSonicInput;
	tBool m_bIDsUltraSonicSet;

	// StartBoolInput
	cObjectPtr<IMediaTypeDescription> m_pStartBoolInput;
	tBufferID m_szIDValueStartBoolInput; 
	tBufferID m_szIDTimestampStartBoolInput;
	tBool m_bStartBoolInput;
 	
    // flag to check the first frame
    bool        m_bFirstFrame;
    bool        m_bFirstFrameDepthimage;
    // image counter 
    tUInt8      m_ui8Imagecount;
    // arduino timestamp value
    tUInt32     m_tsArduinoTime;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormatDepthimage;
    tBitmapFormat m_sInputFormat;
 
     
    //Properties:
    
	tInt        m_nThresholdValueCanny;
    tInt        m_nThresholdValueHough;
	//Imagecut for Zebradetectiondetection
	tInt		m_nImagecutWidthLeft;
	tInt		m_nImagecutWidthRight;
	tInt		m_nImagecutHeightUp;
	tInt		m_nImagecutHeightDown;
     
    tBool       m_bShowDebug;

    // members for signal registry
    typedef std::set<tSignalID> tActiveSignals;

    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    cKernelMutex                              m_oLock;
    tActiveSignals                            m_oActive;

    // caching the error values for signal registry
    tFloat64            m_f64PT1ScaledError;
    tInt16              m_i16Error;
        
};

#endif 
