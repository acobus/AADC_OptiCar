/*
 * Date 24.11.15
 */

#ifndef _UNDISTORT_H_
#define _UNDISTORT_H_

#define OID_ADTF_UNDISTORT "adtf.example.undistort"


//*************************************************************************************************
class cUndistort : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_UNDISTORT, "Undistort", adtf::OBJCAT_DataFilter);

protected:
    cVideoPin    	m_iDistortedImage;
    cVideoPin    	m_oUndistortedImage;

public:
    cUndistort(const tChar* __info);
    virtual ~cUndistort();

	tResult UndistortImage(IMediaSample* pMediaSample);
	tResult SendImage(Mat &image);
	tResult UpdateImageFormat(const tBitmapFormat* pFormat);
	void fixVideoFormat(Mat &image);


protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
private:
	
	const char* m_calibrationFile;

    // bitmap format of input pin
    tBitmapFormat m_sInputFormat;

    // flag to check the first frame
    tBool        m_bFirstFrame;

	// Camera and Distortion coefficients
	tBool 		m_CoefficientsSet;
	cv::Mat 	m_CameraMatrix;
	cv::Mat 	m_DistCoeffs;	

};

//*************************************************************************************************
#endif // _UNDISTORT_H_
