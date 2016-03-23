/*
 * Date 20.12.15 
 */

#include "stdafx.h"
#include "CameraAngleCalibration.h"
#include "../../include/intrinsic_data.h"
#include <fstream>

#define X_MIDDLE 160
#define Y_MIDDLE 120
#define CAMERA_HEIGHT 220
#define POINT_HEIGHT 0
//154 // Todo
#define BUFFER_LENGTH 10


/// Create filter shell
ADTF_FILTER_PLUGIN("CameraAngleCalibration", OID_ADTF_CAMERAANGLECALIBRATION, cCameraAngleCalibration);


cCameraAngleCalibration::cCameraAngleCalibration(const tChar* __info):cFilter(__info)
{

}

cCameraAngleCalibration::~cCameraAngleCalibration()
{

}

tResult cCameraAngleCalibration::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {

    	cObjectPtr<IMediaDescriptionManager> pDescManager;
    	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
    										IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
    										(tVoid**)&pDescManager,
    										__exception_ptr));

    	// Get description for bool values
    	tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
    	RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
    	cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// RGB Input
        RETURN_IF_FAILED(m_iRGBImagePin.Create("RGB_Image_In", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iRGBImagePin));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("StartCalibrating", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartInput)); 

		// RGB Output
        RETURN_IF_FAILED(m_oRGBImagePin.Create("RGB_Image_Out", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oRGBImagePin));

        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

    }
    else if (eStage == StageNormal)
    {
		m_bFirstFrameRGB			= tTrue;
		m_bFirstFrameDepthimage 	= tTrue;

		m_sInputFormat.nWidth 			= 640;
		m_sInputFormat.nHeight 			= 480;
		m_sInputFormat.nBitsPerPixel 	= 24;
		m_sInputFormat.nPixelFormat		= cImage::PF_RGB_888;
		m_sInputFormat.nBytesPerLine 	= 640 * 3;
		m_sInputFormat.nSize 			= m_sInputFormat.nBytesPerLine * 480;
		m_sInputFormat.nPaletteSize 	= 0;
		m_oRGBImagePin.SetFormat(&m_sInputFormat, NULL);
    }
    else if (eStage == StageGraphReady)
    {
		m_startInputSet 		= tFalse;
		
		m_CalibrationActive 	= tFalse;
		m_CalibrationCounter 	= 0;
		m_angle 				= 0.0;
    }

    RETURN_NOERROR;
}

tResult cCameraAngleCalibration::Shutdown(tInitStage eStage, __exception)
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

tResult cCameraAngleCalibration::OnPinEvent(IPin* pSource,
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

        if (pSource == &m_iRGBImagePin)
        {
			//Videoformat
            if (m_bFirstFrameRGB)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iRGBImagePin.GetMediaType(&pType));
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
                m_bFirstFrameRGB = tFalse;
            }

			GetRGBImage(pMediaSample);
        }
        if(pSource == &m_iDepthimagePin )
        {
		        //Videoformat
		        if (m_bFirstFrameDepthimage)
		        {        
		            cObjectPtr<IMediaType> pType;
		            RETURN_IF_FAILED(m_iDepthimagePin.GetMediaType(&pType));
		            cObjectPtr<IMediaTypeVideo> pTypeVideo;
		            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
		            const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
		            if (pFormat == NULL)
		            {
		                LOG_ERROR("No Bitmap information found on pin \"input\"");
		                RETURN_ERROR(ERR_NOT_SUPPORTED);
		            }
		            m_sInputFormatDepthimage.nPixelFormat = pFormat->nPixelFormat;
		            m_sInputFormatDepthimage.nWidth = pFormat->nWidth;
		            m_sInputFormatDepthimage.nHeight =  pFormat->nHeight;
		            m_sInputFormatDepthimage.nBitsPerPixel = pFormat->nBitsPerPixel;
		            m_sInputFormatDepthimage.nBytesPerLine = pFormat->nBytesPerLine;
		            m_sInputFormatDepthimage.nSize = pFormat->nSize;
		            m_sInputFormatDepthimage.nPaletteSize = pFormat->nPaletteSize;
            	    m_bFirstFrameDepthimage = tFalse;
            	}

            	ProcessInputDepth(pMediaSample);

        }
		if(pSource == &m_iStart){
			work(pMediaSample);
		}
		if (nEventCode == IPinEventSink::PE_MediaTypeChanged){
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(m_iRGBImagePin.GetMediaType(&pType));;
			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
			UpdateImageFormat(m_iRGBImagePin.GetFormat());
		}
    }

    RETURN_NOERROR;
}

/*
 * This Method starts the calibration
 */
tResult cCameraAngleCalibration::work(IMediaSample* pMediaSample){

	tBool bValue = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pStartInput ,pMediaSample, pCoder);    

		if (!m_startInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStartInput);		
			m_startInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueStartInput, (tVoid*)&bValue);         
	}

	LOG_INFO(cString::Format("Received: %i", bValue) );

	m_CalibrationActive 	= bValue;
	m_angle 				= 0.0;
	m_CalibrationCounter  	= 0;

	RETURN_NOERROR;	
}

/*
 * Receive Depth Image
 */
tResult cCameraAngleCalibration::ProcessInputDepth(IMediaSample* pSample)
{
	//LOG_INFO(cString::Format("0"));
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	//LOG_INFO(cString::Format("1"));

	if (m_CalibrationActive){

	

		const tVoid* l_pSrcBuffer;

		IplImage* oImg = cvCreateImage(cvSize(m_sInputFormatDepthimage.nWidth, m_sInputFormatDepthimage.nHeight), IPL_DEPTH_16U, 3);
		RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
		oImg->imageData = (char*)l_pSrcBuffer;
		Mat image(cvarrToMat(oImg));
		cvReleaseImage(&oImg);
		pSample->Unlock(l_pSrcBuffer);

		Mat m_depthImage = Mat(m_sInputFormatDepthimage.nHeight,m_sInputFormatDepthimage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthimage.nBytesPerLine);		

		float distance 	= m_depthImage.at<ushort>(Y_MIDDLE, X_MIDDLE);

		LOG_INFO(cString::Format("CAC: %f", distance));
		
		if (distance > 500 && distance < 1500){

			double a = (Y_MIDDLE*2 - c_y) * distance / f_y;
			double b = distance;
			double c = CAMERA_HEIGHT-POINT_HEIGHT;

			// Solve a*cos(x) + b*sin(x) = c
			m_angle += 2 * atan((b-sqrt(a*a + b*b - c*c))/(a+c));
//			m_angle += 	asin( (CAMERA_HEIGHT-POINT_HEIGHT) / distance );
			m_CalibrationCounter++;

			if (m_CalibrationCounter == BUFFER_LENGTH){

				m_CalibrationCounter 	= 0;
				m_CalibrationActive 	= tFalse;
				m_angle 				/= BUFFER_LENGTH;

				LOG_INFO(cString::Format("Found Angle: %f", m_angle*180/M_PI) );

				fstream fs;
				fs.open("/home/aadc/Desktop/AADC Source/src/aadcUser/include/camera_angle.h",ios::out);
				fs << "#ifndef camera_angle_h\n";
				fs << "#define camera_angle_h\n";
				fs << "#include <cmath>\n";
				fs << "const float camera_angle=" << m_angle*180/M_PI << "; // in [deg]\n";
				fs << "const float camera_angle_rad=((camera_angle)*M_PI/180); // in [rad]\n";
				fs << "#endif";
				fs.close();

				m_angle 				= 0.0;
				
			}
		}

	}
	
	RETURN_NOERROR;            
}

/*
 * This Method receives RGB Image 
 */
tResult cCameraAngleCalibration::GetRGBImage(IMediaSample* pMediaSample){

	const tVoid* l_pSrcBuffer;

	// Get image out of mediaSample
	IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
    RETURN_IF_FAILED(pMediaSample->Lock(&l_pSrcBuffer));
    oImg->imageData = (char*)l_pSrcBuffer;
    Mat image(cvarrToMat(oImg));
    cvReleaseImage(&oImg);
    pMediaSample->Unlock(l_pSrcBuffer);

	CreateAndTransmitGCL();

	SendRGBImage(image);

	RETURN_NOERROR;
}

/*
 * Send RGB Image
 */
tResult cCameraAngleCalibration::SendRGBImage(Mat image){

	cObjectPtr<IMediaSample> pNewRGBSample;

    if (IS_OK(AllocMediaSample(&pNewRGBSample)))
    {                            
     	tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
        pNewRGBSample->Update(tmStreamTime, image.data, tInt32(image.channels()*image.cols*image.rows), 0);
        fixVideoFormat(image);

        RETURN_IF_FAILED(m_oRGBImagePin.Transmit(pNewRGBSample));
    }    

	RETURN_NOERROR;
}

void cCameraAngleCalibration::fixVideoFormat(Mat image)
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
    m_oRGBImagePin.SetFormat(&m_sInputFormat, NULL);
}

tResult cCameraAngleCalibration::UpdateImageFormat(const tBitmapFormat* pFormat){
	
	if(pFormat != NULL)
		m_sInputFormat = (*pFormat);

	RETURN_NOERROR;
}

tResult cCameraAngleCalibration::CreateAndTransmitGCL()
{
    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(8192));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;

    // draw rectangle to scale the video display correctly
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());	
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, m_sInputFormat.nWidth, m_sInputFormat.nHeight);

    // draw near and far area
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 100, 255).GetRGBA());

	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(X_MIDDLE*2-1), static_cast<tInt16>(Y_MIDDLE*2-1), static_cast<tInt16>(X_MIDDLE*2+1), static_cast<tInt16>(Y_MIDDLE*2+1));
	//cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(Y_MIDDLE*2-1), static_cast<tInt16>(X_MIDDLE*2-1), static_cast<tInt16>(Y_MIDDLE*2+1), static_cast<tInt16>(X_MIDDLE*2+1));


    // draw the min and max lane width
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(150,150,150).GetRGBA());


    // draw near and far line
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,70,0).GetRGBA());
	
    
    // draw the lines for place to be and the detected lanecenter
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,150,0).GetRGBA());

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}
