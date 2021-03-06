/*
 * Date 16.12.15
 */

#include "stdafx.h"
#include "Undistort.h"
//#include <fstream>

/// Create filter shell
ADTF_FILTER_PLUGIN("Undistort", OID_ADTF_UNDISTORT, cUndistort);


cUndistort::cUndistort(const tChar* __info):cFilter(__info)
{

}

cUndistort::~cUndistort()
{

}

tResult cUndistort::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

		// Distorted Video Input
        RETURN_IF_FAILED(m_iDistortedImage.Create("Image_Dist", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistortedImage));

		// Undistorted Video Output
        RETURN_IF_FAILED(m_oUndistortedImage.Create("Image_Undist", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oUndistortedImage));
    }
    else if (eStage == StageNormal)
    {
		m_bFirstFrame = tTrue;
		m_CoefficientsSet = tFalse;
		m_calibrationFile = "/home/aadc/AADC/src/aadcUser/calibration/xtion_intrinsic_calib.yml";

		m_sInputFormat.nWidth 			= 640;
		m_sInputFormat.nHeight 			= 480;
		m_sInputFormat.nBitsPerPixel 	= 24;
		m_sInputFormat.nPixelFormat		= cImage::PF_RGB_888;
		m_sInputFormat.nBytesPerLine 	= 640 * 3;
		m_sInputFormat.nSize 			= m_sInputFormat.nBytesPerLine * 480;
		m_sInputFormat.nPaletteSize 	= 0;
		m_oUndistortedImage.SetFormat(&m_sInputFormat, NULL);
    }
    else if (eStage == StageGraphReady)
    {

    }

    RETURN_NOERROR;
}

tResult cUndistort::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cUndistort::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_iDistortedImage)
        {
			//Videoformat
            if (m_bFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iDistortedImage.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrame = tFalse;
            }

			UndistortImage(pMediaSample);
        }
		if (nEventCode == IPinEventSink::PE_MediaTypeChanged){
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(m_iDistortedImage.GetMediaType(&pType));;
			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
			UpdateImageFormat(m_iDistortedImage.GetFormat());
		}
    }

    RETURN_NOERROR;
}

/*
 * This Method undistorts incoming images 
 */
tResult cUndistort::UndistortImage(IMediaSample* pMediaSample){

	const tVoid* l_pSrcBuffer;

	// Get image out of mediaSample
	IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
    RETURN_IF_FAILED(pMediaSample->Lock(&l_pSrcBuffer));
    oImg->imageData = (char*)l_pSrcBuffer;
    Mat image_distorted(cvarrToMat(oImg));
    cvReleaseImage(&oImg);
    pMediaSample->Unlock(l_pSrcBuffer);

	// Get configuration data
 	if (!m_CoefficientsSet){
		FileStorage fs( m_calibrationFile, FileStorage::READ );
		if (fs.isOpened()){
	        fs["camera_matrix"] >> m_CameraMatrix;
	        fs["distortion_coefficients"] >> m_DistCoeffs;
        	fs.release();
		}
	}

	Mat image_undistorted;

	undistort(image_distorted, image_undistorted, m_CameraMatrix, m_DistCoeffs);

	SendImage(image_undistorted);
//SendImage(image_distorted);

	RETURN_NOERROR;
}


tResult cUndistort::SendImage(Mat &image){

	cObjectPtr<IMediaSample> pNewRGBSample;

    if (IS_OK(AllocMediaSample(&pNewRGBSample)))
    {                            
     	tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
        pNewRGBSample->Update(tmStreamTime, image.data, tInt32(image.channels()*image.cols*image.rows), 0);
        fixVideoFormat(image);

        RETURN_IF_FAILED(m_oUndistortedImage.Transmit(pNewRGBSample));
    }    

	RETURN_NOERROR;
}

void cUndistort::fixVideoFormat(Mat &image)
{
    if(image.cols == m_sInputFormat.nWidth && image.rows == m_sInputFormat.nHeight && (image.channels()*8==m_sInputFormat.nBitsPerPixel))
        return;
    LOG_WARNING(adtf_util::cString::Format("Changing image format: [%dx%d], %d channels",image.cols,image.rows,image.channels()));
    m_sInputFormat.nWidth = image.cols;
    m_sInputFormat.nHeight = image.rows;
    m_sInputFormat.nBitsPerPixel = 8 * image.channels();
    m_sInputFormat.nPixelFormat = (image.channels() == 3) ? cImage::PF_RGB_888 : cImage::PF_GREYSCALE_8;
    m_sInputFormat.nBytesPerLine = image.cols * image.channels();
    m_sInputFormat.nSize = m_sInputFormat.nBytesPerLine * image.rows;
    m_sInputFormat.nPaletteSize = 0;
    m_oUndistortedImage.SetFormat(&m_sInputFormat, NULL);
}

tResult cUndistort::UpdateImageFormat(const tBitmapFormat* pFormat){
	
	if(pFormat != NULL)
		m_sInputFormat = (*pFormat);

	RETURN_NOERROR;
}
