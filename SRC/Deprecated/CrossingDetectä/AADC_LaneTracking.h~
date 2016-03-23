#ifndef _LaneTracking_FILTER_HEADER_
#define _LaneTracking_FILTER_HEADER_

#define OID_ADTF_LaneTracking  "adtf.aadc.aadc_laneTracking"
#define LT_POINTS_ARRAY_SIZE 640

class cLaneTracking : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LaneTracking, "AADC LaneTracking", OBJCAT_Tool, "AADC Lane Tracking", 1, 0, 0, "BFFT GmbH");    
    
protected:
    //Eingang für RGB Bild
    cVideoPin           m_oVideoInputPin;
    cInputPin           m_oInputStart; 
        
    cOutputPin          m_oGCLOutput;
    cOutputPin          m_oAccelerateOutput;
    cOutputPin          m_oSteeringAngleOutput;
    cOutputPin          m_oSteeringAnglePT1Output;
    cOutputPin          m_oHeadLightsOutput;

public:

    struct sPoint 
    {
        tInt16 x;
        tInt16 y;
    };

                
    cLaneTracking(const tChar*);
    virtual ~cLaneTracking();

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
    tResult Search(sPoint *points,tUInt8 *lfdnr,tUInt8 limit,cv::Mat cannyimage);
    tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteeringAngle(const tFloat32, tTimeStamp);
    tResult TransmitSteeringAnglePT1(const tFloat32, tTimeStamp);
    tResult TransmitHeadLights(const tBool, tTimeStamp);
    tResult LateralControl(sPoint*, tUInt8*, tTimeStamp);
    tResult LongitudinalControl(sPoint*, tUInt8*, tTimeStamp);
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();


protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

private:
    //für Search
    sPoint      m_asAllpointsNear[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8NearPointsCount;
    sPoint      m_asAllpointsFar[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8FarPointsCount;

    // points and values for lane detection
    sPoint      m_sLaneCenterNear;
    tInt16      m_i16LaneWidth;
    sPoint      m_sPlaceToBe;
    tInt16      m_i16LaneWidthMinNear;
    tInt16      m_i16LaneWidthMaxNear;  
    tInt16      m_i16LaneWidthMinFar;
    tInt16      m_i16LaneWidthMaxFar;  

    // offset of the camera to the mid of the vehicle
    tFloat64    m_f64CamOffset;
    
    tInt        m_nCurrentNearLine;

    // PID-Controller values
    tFloat64    m_f64Kp;
    tFloat64    m_f64Ki;
    tFloat64    m_f64Kd;
    tInt        m_nCenterFromLeft;
    tInt        m_nCenterFromRight;
    tInt16      m_i16ErrorSum;
    tInt16      m_i16ErrorOld; 
    tFloat32    m_f32Ts;
    tInt16      m_i16FarLaneCenter;
    tInt16      m_i16FarLaneWidth;
    tFloat32    m_f32AccelerateOut;
    tInt16      m_i16FarLaneWidthCalc;

    // PT1-Controller values
    tFloat64    m_f64PT1Tau;
    tFloat64    m_f64PT1Sample;
    tFloat64    m_f64PT1Gain;
    tFloat32    m_f32PT1LastSteeringOut;
    tFloat32    m_f32PT1SteeringOut;
    tFloat64    m_f64PT1InputFactor;



    // active flag to enable driving
    tBool       m_bActive;
        
    // opencv members for line detection
    cv::Mat     m_matLine;    
    cv::Mat     m_matGreyNear;
    cv::Mat     m_matGreyFar;
    cv::Mat     m_matGreyThreshNear;
    cv::Mat     m_matGreyThreshFar;
    cv::Size    m_szCannySize;
    //cv::Size  m_szCannySizeFar;
    cv::Mat     m_matLineCannyNear;
    cv::Mat     m_matLineCannyFar;

    // DDL descriptions
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBool;

    // flag to check the first frame
    bool        m_bFirstFrame;
    // image counter 
    tUInt8      m_ui8Imagecount;
    // arduino timestamp value
    tUInt32     m_tsArduinoTime;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormat;
        
    //Properties:
    
    tUInt8      m_ui8InitCtrl;
    tInt        m_nNearLine;
    tInt        m_nNearLineMaxOffset;
    tInt        m_nFarLine;
    tFloat32    m_f32AccelerationMax;
    tFloat32    m_f32AccelerationMin;
    tInt        m_nAccelerationFarNearDiff;
    tFloat32    m_f32SteeringAngle;
    tInt        m_nThresholdValue;
    tInt        m_nBlindCounter;
    tInt        m_nBlindCounterFar;
    tInt        m_nDriveTime;
    tInt        m_nEmergencyStopTime;
        
    tBool       m_bShowDebug;
    tBool       m_bLightBeamTriggerEnabled;
        
    tHandle     m_hStopTimerNegative;
    tHandle     m_hStopTimerZero;
    tHandle     m_hEmergencyStopTimerNegative;
    tHandle     m_hEmergencyStopTimerZero;
    tHandle     m_hEmergencyStopTimerResume;
        
    // Critical Sections
    cCriticalSection    m_oTransmitSteerCritSection;
    cCriticalSection    m_oTransmitAccelCritSection;
    cCriticalSection    m_oTransmitLightCritSection;
    cCriticalSection    m_oRunCritSection;

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
