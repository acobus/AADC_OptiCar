#ifndef _CrossingDetect_FILTER_HEADER_
#define _CrossingDetect_FILTER_HEADER_

#define OID_ADTF_CrossingDetect  "adtf.aadc.CrossingDetect"
#define LT_POINTS_ARRAY_SIZE 640

class cCrossingDetect : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CrossingDetect, "CrossingDetect", OBJCAT_Tool, "CrossingDetect", 1, 0, 0, "BFFT GmbH");    
    
protected:
    //Eingang f�r RGB Bild
    cVideoPin           m_iVideoInputPin;
	cVideoPin			m_iDepthimagePin;
    cInputPin           m_iStart; 
        
    cOutputPin          m_oGCLOutput;
	cOutputPin			m_oDistanceAndTypeOutput;


public:

    cCrossingDetect(const tChar*);
    virtual ~cCrossingDetect();

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
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult ReadProperties(const tChar* strPropertyName = NULL);
    tResult CreateAndTransmitGCL();
	tResult TransmitCrossingSpot(float lot, tFloat32 rotationDegree);
	tResult InitialState();
	tResult ProcessCircle();

protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);
public:

	vector<Point2f> PixelSpot;
	vector<Point2f> m_Points;
	vector<Point2f> m_PointsCircle;


private:
	// for stop criteria
	tUInt32		m_iframeCounter;

	//decide which videoinput we use
	tBool		RGBreceived;

    // active flag to enable driving
    tBool       m_bActive;

	tBool 		m_BoolTrafficCircle;
        
    // members for corner detection
  	cv::Mat		m_matImageDepth;
    cv::Mat     m_matThres;
	cv::Mat     m_matThres2;
    cv::Mat     m_matCorner;
	cv::Mat		m_matNormalized;

	// line detection
	cv::Mat		matCutHough;
	cv::Mat		m_matCannyLeft;
	vector<Vec2f> 	m_lines;
	vector<Vec2f> 	m_linesL;
	vector<Vec2f> 	m_linesR;
	vector<Point2f> 	m_linesRSmall;
	vector<Point2f>     m_linesRSmallLane;
	vector<Vec2f>	m_linesCIRCLE;
	vector<Vec2f>	m_linesCIRCLEsmall;
	cv::Mat		m_matCannyRight;
	tUInt16 m_iImageHalfWidth;
	tBool	m_DeadEndFrontBool; 
	tBool	m_DeadEndLeftBool;
	tBool	m_DeadEndRightBool; 
	tBool 	m_SMALL;

	Point2f m_representerHLG1;
	Point2f m_representerHRG1;
	Point2f m_representerHRG2;
	Point2f m_representerHRGS;
	Point2f m_representerHRGSLane;
	Point2f m_representerHLGC;
	Point m_intersecpointR;
	Point m_intersecpointRS;
	Point3f m_crossFrontRightS;
	Point3f m_crossFrontRight;
	Point2i m_sentSpot;

	struct linesStruct{
		tFloat32 radius;
		tFloat32 angle;
		linesStruct() {};
		linesStruct(tFloat32 r, tFloat32 a) : radius(r), angle(a) {}
		bool operator<(const linesStruct& other) const {
			return (radius < other.radius);
		}
	};
    std::vector<linesStruct> lines;
    std::vector<linesStruct> linesBig;

	// crossing type
	int			m_CrossType;

    // DDL descriptions
  //cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
  //cObjectPtr<IMediaTypeDescription> m_pCoderDescPark;

	// CrossLotOutput
	cObjectPtr<IMediaTypeDescription> m_pDistanceAndCrossType;
	tBufferID m_szIDCrossingType;
	tBufferID m_szCrossDistance;
	tBufferID m_szIDTimestampCrossOutput;
	tBufferID m_szIDRotationDegreeCrossOutput;
	tBool m_crossingSpotOutputSet;

	// StartBoolIDInput
	cObjectPtr<IMediaTypeDescription> m_pStartBoolIDInput;
	tBufferID m_szIDValueStartBoolIDInput; 
	tBufferID m_szIDTimestampStartBoolIDInput;
	tBufferID m_szTrafficCircleBoolInput;
	tBufferID m_szIDDRIVING;
	tInt32 m_DRIVING_ID;
	tBool m_bStartBoolIDInput;
 	
 	

    // flag to check the first frame
    bool        m_bFirstFrame;
    bool        m_bFirstFrameDepthimage;
    // image counter 
    tUInt8      m_ui8Imagecount;
    // arduino timestamp value
    //tUInt32     m_tsArduinoTime;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormatDepthimage;
    tBitmapFormat m_sInputFormat;
        
    //Properties:
    
    //tUInt8      m_ui8InitCtrl;
    tFloat64    m_nThresholdValue;
	tFloat64	m_nThresholdValue2;
	tFloat64	m_nThresholdValueBeforeNorm;
    tFloat64    m_nCornerHarrisparamK;
    tInt        m_nCornerHarrisblockSize;
    tInt        m_nCornerHarrisksize;
	tInt        m_nThresholdValueCanny;
    tInt        m_nThresholdValueHough;
	tInt		m_nThresholdValueHoughSMALL;


//Imagecut for Cornerdetection
	tInt		m_nImagecutWidthLeft;
	tInt		m_nImagecutWidthRight;
	tInt		m_nImagecutHeightUp;
	tInt		m_nImagecutHeightDown;

//Imagecut for Houghlines
	tInt		m_nImagecutWidthLeftHough;
	tInt		m_nImagecutWidthRightHough;
	tInt		m_nImagecutHeightUpHough;
	tInt		m_nImagecutHeightDownHough;

	//CIRCLE PROPS
	tFloat64    m_nThresholdValueCIRCLE;
	tFloat64    m_nThresholdValueBeforeNormCIRCLE;
	tFloat64	m_nThresholdValue2CIRCLE;
	tInt        m_nThresholdValueCannyCIRCLE;
    tInt        m_nThresholdValueHoughCIRCLE;
     
    tBool       m_bShowDebug;      

    // members for signal registry
    typedef std::set<tSignalID> tActiveSignals;

    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    cKernelMutex                              m_oLock;
    tActiveSignals                            m_oActive;

    // caching the error values for signal registry
    tFloat64            m_f64PT1ScaledError;
    tInt16              m_i16Error;

	tFloat32 	m_GCLDistance;

	Point3f m_GCLCoordinates[30];
	tFloat32 	m_rotationAngle;
	tFloat32 	m_HrotationAngle;
	Point2i m_rotPt1, m_rotPt2;
	Point2i m_HrotPt1, m_HrotPt2;

	// deactivation timer
	tUInt32 	m_activationTime;

	// Flag to know that this filter is going to stop working!
	tBool m_shutdown;
        
};

#endif 
