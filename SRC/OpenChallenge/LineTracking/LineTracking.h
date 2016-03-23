#ifndef _LineTracking_FILTER_HEADER_
#define _LineTracking_FILTER_HEADER_

#define OID_ADTF_LineTracking  "adtf.line_tracking"
#define LT_POINTS_ARRAY_SIZE 640

class cLineTracking : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_FILTER(OID_ADTF_LineTracking, "LineTracking", adtf::OBJCAT_DataFilter);    
    
protected:
    //Eingang f�r RGB Bild
    cVideoPin           m_oVideoInputPin;
    cInputPin           m_oInputStart;
    cInputPin			m_oCurrentVelocity;
	cInputPin 			m_iDistance;
        
    cOutputPin          m_oGCLOutput;
    cOutputPin          m_oAccelerateOutput;
    cOutputPin          m_oSteeringAngleOutput;
    cOutputPin          m_oSteeringAnglePT1Output;
    cOutputPin          m_oHeadLightsOutput;
	cOutputPin			m_oStartCalcArea;


public:

    struct sPoint 
    {
        tInt16 x;
        tInt16 y;
    };

                
    cLineTracking(const tChar*);
    virtual ~cLineTracking();

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
    tResult Search(sPoint *points,tUInt8 *lfdnr,tUInt8 limit,cv::Mat cannyimage, int mode);
    tResult TransmitAcceleration(tFloat32, tTimeStamp);
    tResult TransmitSteeringAngle(const tFloat32, tTimeStamp);
    tResult TransmitSteeringAnglePT1(const tFloat32, tTimeStamp);
    tResult TransmitHeadLights(const tBool, tTimeStamp);
    tResult LateralControl(sPoint*, tUInt8*, tTimeStamp);
    tResult ProcessInput(IMediaSample* pSample, tTimeStamp);
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();
	tInt16	GetAverage(tInt16 x, std::deque<tInt16> &Buffer);
	tInt16	GetAverage2(tInt16 x, std::deque<tInt16> &Buffer);
	tResult SearchHorizontalLine();
	tResult startCalcArea(tBool state);
	tResult processOverallDistance(IMediaSample* pMediaSample);


protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

private:
    //f�r Search
    sPoint      m_asAllpointsNear[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8NearPointsCount;
    sPoint      m_asAllpointsFar[LT_POINTS_ARRAY_SIZE];
    tUInt8      m_ui8FarPointsCount;

    //f�r HoughSearch
    sPoint      m_HoughpointsNear[LT_POINTS_ARRAY_SIZE];
    sPoint      m_HoughpointsFar[LT_POINTS_ARRAY_SIZE];
	tUInt8		m_HoughLineSizeCounter;


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

	//HoughLines
	vector<Vec2f> 	m_lines;
	cv::Mat		m_matCannyHough;
	tInt        m_nThresholdValueCanny;
    tInt        m_nThresholdValueHough;
	//Point2f m_representerHG1;
	//Point2f m_representerHG2;
	//Point2f m_representerHG3;
	vector<Point2f> m_HoughGroup1;//mid line
	vector<Point2f> m_HoughGroup2;//right line
	vector<Point2f> m_HoughGroup3;//left line

	

//Imagecut for Houghlines
	tInt		m_nImagecutWidthLeftHough;
	tInt		m_nImagecutWidthRightHough;
	tInt		m_nImagecutHeightUpHough;
	tInt		m_nImagecutHeightDownHough;
        
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
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSteeringAngle;
	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSteeringPT1;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSpeed;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolHeadlight;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolStart;
	cObjectPtr<IMediaTypeDescription> m_pCoderDesctInt32SignalValue;

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
    
    //////////////////////////////////////////////////////////////////////////////////
    
    // Current Velocity Pin (Input)
    cObjectPtr<IMediaTypeDescription> m_pCurrentVelocityInput;
    tBufferID m_szIDSignalValueCurrentVelocityMinInput;
    tBufferID m_szIDSignalValueCurrentVelocityMaxInput;
    tBool m_bCurrentVelocityInputSet;
    
    tResult ChangeCurrentVelocity(IMediaSample* pMediaSample);

	tResult DriveStraight(IMediaSample* pMediaSample);

	tBufferID m_szIDBoolValueOutput;
	tBool m_bLFStatusSet;

	tBool m_checkLastDistance;
	tFloat32 m_distanceOffset;

	// remember last x value
	tInt16 m_lastPointX;

	// gcl for number of found points
	tInt32 m_numberGoodPoints;

	vector<tInt16> m_goodXPoints;
	vector<tInt16> m_veryGoodXPoints;

	// 
	cObjectPtr<IMediaTypeDescription> m_pStartCalcAreaOutput;
	tBufferID m_szIDBoolValueStartCalcAreaOutput; 
	tBufferID m_szIDTimestampStartCalcAreaOutput;
	tBool m_bStartCalcAreaOutputSet;


	// DistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBool m_bIDsDistanceSet;

	// Buffer
	deque<tInt16> m_xValueBuffer;
        
};

#endif 
