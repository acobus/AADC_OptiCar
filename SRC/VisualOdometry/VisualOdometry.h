#ifndef _VisualOdometry_FILTER_HEADER_
#define _VisualOdometry_FILTER_HEADER_

#define OID_ADTF_VisualOdometry  "adtf.aadc.VisualOdometry"

class cVisualOdometry : public adtf::cFilter,
                      public adtf::ISignalProvider
{
    
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_VisualOdometry, "VisualOdometry", OBJCAT_Tool, "VisualOdometry", 1, 0, 0, "BFFT GmbH");    
    
protected:
    cVideoPin           m_iVideoInputPin;
    cVideoPin 			m_iDepthInputPin;
    cInputPin           m_iInputStart;  
    cOutputPin 			m_oDrivenDistance;

public:

    struct sPoint 
    {
        tInt16 x;
        tInt16 y;
    };

                
    cVisualOdometry(const tChar*);
    virtual ~cVisualOdometry();

    // implements cFilter
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);

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

    tResult ProcessRGBInput(IMediaSample* pSample, tTimeStamp);
    
    tResult ProcessDepthInput(IMediaSample* pSample);

	tResult ProcessState(IMediaSample* pMediaSample);
	
	tResult SendDrivenDistance();



protected:
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);

private:

	// Start Input
	cObjectPtr<IMediaTypeDescription> m_pStartInput;
	tBufferID m_szIDBoolValueStartInput;
	tBool m_bStartInputSet;
	
    // Driven Distance Output
    cObjectPtr<IMediaTypeDescription> m_pDrivenDistanceOutput;
    tBufferID m_szIDValueDrivenDistanceOutput;
    tBufferID m_szIDTimestampDrivenDistanceOutput;
    tBool m_bDrivenDistanceOutputSet;

    // active flag to enable driving
    tBool       m_bActive;   

    // opencv members
	cv::Mat			m_image;
	vector<Point2f> m_points;
	vector<Point3f> m_lastCoordinates;
	cv::Mat			m_R_f;
	cv::Mat			m_t_f;
	
	cv::Mat 		m_DepthImage;

    // flag to check the first frame
    tBool       	m_bFirstFrame;
    tBool 			m_bFirstFrameDepth;

    // bitmap format of input pin
    tBitmapFormat 	m_sInputFormat;
    tBitmapFormat 	m_sInputFormatDepth;
    
    tUInt8      	m_ui8InitCtrl;
    
    // Flag to control depth image input
    tBool 			m_getDepthImage;      

    // Driven distance since activation of this filter
    tFloat32		m_DrivenDistance;
               
};

#endif 
