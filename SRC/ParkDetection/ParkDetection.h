#ifndef _ParkDetection_FILTER_HEADER_
#define _ParkDetection_FILTER_HEADER_

#define OID_ADTF_ParkDetection  "adtf.aadc.ParkDetection"
#define LT_POINTS_ARRAY_SIZE 640

class cParkDetection : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ParkDetection, "ParkDetection", OBJCAT_Tool, "ParkDetection", 1, 0, 0, "BFFT GmbH");    
    
protected:
    //Eingang für RGB Bild
    cVideoPin           m_iVideoInputPin;
	cVideoPin			m_iDepthimagePin;

    cInputPin           m_iStart;
	cInputPin			m_iLane;
	cInputPin			m_iStop;
        
    cOutputPin          m_oGCLOutput;
	cOutputPin			m_oDistanceToParkingSpot;
	cOutputPin			m_oDistanceToParkingSpotDM;

public:

                
    cParkDetection(const tChar*);
    virtual ~cParkDetection();

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
	tResult SearchCorners();
    tResult ProcessInputRGB(IMediaSample* pSample, tTimeStamp);
    tResult ProcessInputDepth(IMediaSample* pSample, tTimeStamp);
	tResult ProcessLane(IMediaSample* pMediaSample);
    tResult ProcessFound();
    tResult ProcessOutput(); 
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();
	tResult TransmitParkingSpot(float lots);
	tResult InitialState();


protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);
public:
	tInt		pui8PointsCount;
	vector<Point2f> m_PixelSpot;
	vector<Point2i> m_Points;
	vector<Point2i> m_PointsNeu;
	vector<Point2i> m_intersecPoint;

	tUInt32 tictoc;

private:
    //für Search
    tUInt8      m_ui8PointsCount;

	// for stop criteria
	tUInt8		m_iframeCounter;


    // offset of the camera to the mid of the vehicle
    tFloat64    m_f64CamOffset;
    

	//decide which videoinput we use
	tBool		RGBreceived;




    // active flag to enable driving
    tBool       m_bActive;
        
    // opencv members for line detection 

  	cv::Mat		m_matImageDepth;
    cv::Mat     m_matMedian;
    cv::Mat     m_matGrey;
    cv::Mat     m_matThres;
	cv::Mat     m_matThres2;
    cv::Mat     m_matCorner;
	cv::Mat		m_matNormalized;
	cv::Mat		m_matNormal;
	cv::Mat		m_matScaled;
	cv::Mat		image_GREY;

	//Houghlines
	vector<Vec2f> 	m_lines;
	vector<Point2f> m_HoughGroup2;
	cv::Mat		m_matCannyHough;
	tInt        m_nThresholdValueCanny;
    tInt        m_nThresholdValueHough;


    // DDL descriptions
    //cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
	//cObjectPtr<IMediaTypeDescription> m_pCoderDescPark;


	// Process found parking spot(Output)
    cObjectPtr<IMediaTypeDescription> m_pDistanceToParkingSpotOutput;
	tBufferID m_szIDParkingSpot1Output;
	tBufferID m_szIDTimestampParkingSpotOutput; 
	tBool m_parkingSpotOutputSet;

	// Process found parking spot(Output)
    cObjectPtr<IMediaTypeDescription> m_pDistanceToParkingSpotOutputDM;
	tBufferID m_szIDParkingSpot1OutputDM;
	tBufferID m_szIDTimestampParkingSpotOutputDM; 
	tBool m_parkingSpotOutputSetDM;

	// Process Start Pin (Input)
    cObjectPtr<IMediaTypeDescription> m_pStartInput;
	tBufferID m_szIDBoolValueStartInput;
	tBufferID m_szIDStartInput;
	tBool m_startInputSet;

	// Process Stop Pin (Input)
    cObjectPtr<IMediaTypeDescription> m_pStopInput;
	tBufferID m_szIDStopInput;
	tBool m_stopInputSet;

	// Process Lane Input
	cObjectPtr<IMediaTypeDescription> m_pCoderDescLane;
	tBufferID m_szIDSignalValueLaneSpot1XInput;
	tBufferID m_szIDSignalValueLaneSpot1YInput;
	tBufferID m_szIDSignalValueLaneSpot2XInput;
	tBufferID m_szIDSignalValueLaneSpot2YInput;
	tBool m_bLaneInputSet; 	
	tFloat32 m_LaneSpots[4];

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
    
    tUInt8      m_ui8InitCtrl;
    tFloat64    m_nThresholdValue;
	tFloat64	m_nThresholdValue2;
	tFloat64	m_nThresholdValueBeforeNorm;
    tFloat64    m_nCornerHarrisparamK;
    tInt        m_nCornerHarrisblockSize;
    tInt        m_nCornerHarrisksize;

	tInt		m_pointCloudSizeFar;
	tInt		m_LaneToleranceFar;
	tInt		m_LaneToleranceNear;
	tInt		m_pointCloudSizeNear;

	tInt		m_nImagecutWidthLeft;
	tInt		m_nImagecutWidthRight;
	tInt		m_nImagecutHeightUp;
	tInt		m_nImagecutHeightDown;
	tFloat32	m_nPointcloudTolerance;
        
    tBool       m_bShowDebug;

	tFloat32 m_lotDim[2][2];
	tInt32 m_parkLotPixelSize[2][2];
	tInt32 m_parkingType;

	tBool m_firstSpot;
	
        
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


	vector<Point3f> ParkPoint;
        
};

#endif 
