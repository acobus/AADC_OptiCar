/**
*
* VisualOdometry
*
* Date 28.11.15
*
*/
 
#include "stdafx.h"
#include "VisualOdometry.h"
#include "../Util/Util.h"
#include "../../include/intrinsic_data.h"
#include "../../include/camera_angle.h"

#define MIN_NUM_FEAT 150
#define X_MIN 20
#define X_MAX 620
#define Y_MIN 20
#define Y_MAX 460


ADTF_FILTER_PLUGIN("VisualOdometry", OID_ADTF_VisualOdometry, cVisualOdometry)


cVisualOdometry::cVisualOdometry(const tChar* __info) : cFilter(__info)
{

}

cVisualOdometry::~cVisualOdometry()
{

}

tResult cVisualOdometry::GetInterface(const tChar* idInterface,
    tVoid** ppvObject)
{
    if (idmatch(idInterface, IID_ADTF_SIGNAL_PROVIDER))
    {
        *ppvObject = static_cast<ISignalProvider*> (this);
    }
    else
    {
        return cFilter::GetInterface(idInterface, ppvObject);
    }

    Ref();

    RETURN_NOERROR;
}

tUInt cVisualOdometry::Ref()
{
    return cFilter::Ref();
}

tUInt cVisualOdometry::Unref()
{
    return cFilter::Unref();
}

tVoid cVisualOdometry::Destroy()
{
    delete this;
}

tResult cVisualOdometry::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cVisualOdometry::Stop(__exception)
{

    return cFilter::Stop(__exception_ptr);
}

tResult cVisualOdometry::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // Media Description Signal
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);        
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        
        // Media Description Bool
        tChar const * strDescSignalBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalBoolValue);
        cObjectPtr<IMediaType> pTypeSignalBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescSignalBoolValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
     
        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));
        
        // Depth Image Input
        RETURN_IF_FAILED(m_iDepthInputPin.Create("Depth_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthInputPin));

		// Start Input
        RETURN_IF_FAILED(m_iInputStart.Create("start", new cMediaType(0, 0, 0, "tBoolSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iInputStart));
		RETURN_IF_FAILED(pTypeSignalBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartInput));
        
        // Driven Distance Output
		RETURN_IF_FAILED(m_oDrivenDistance.Create("Driven_Distance", pTypeSignalValue, this));
		RETURN_IF_FAILED(RegisterPin(&m_oDrivenDistance));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDrivenDistanceOutput)); 
        
    }
    else if (eStage == StageNormal)
    {
        m_bFirstFrame 				= tTrue;
        m_bFirstFrameDepth 			= tTrue;

        m_ui8InitCtrl 				= 0; 

		m_bActive 					= tFalse;     
		
		m_getDepthImage 			= tTrue;
		
		m_bStartInputSet			= tFalse;
		m_bDrivenDistanceOutputSet 	= tFalse;

		m_DrivenDistance 			= 0.0;
    }

    RETURN_NOERROR;
}

tResult cVisualOdometry::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cVisualOdometry::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
  
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cVisualOdometry::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        tTimeStamp InputTimeStamp;
        InputTimeStamp = pMediaSample->GetTime();


        if(pSource == &m_iVideoInputPin)
        {
            //Videoformat
            if (m_bFirstFrame)
            {        
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iVideoInputPin.GetMediaType(&pType));
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

            ProcessRGBInput(pMediaSample, InputTimeStamp);
        }
        
        else if(pSource == &m_iDepthInputPin){
        	// Videoformat
        	if (m_bFirstFrameDepth){
        		
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iDepthInputPin.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();                                
                if (pFormat == NULL)
                {
                    LOG_ERROR("No Bitmap information found on pin \"input\"");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormatDepth.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormatDepth.nWidth = pFormat->nWidth;
                m_sInputFormatDepth.nHeight =  pFormat->nHeight;
                m_sInputFormatDepth.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormatDepth.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormatDepth.nSize = pFormat->nSize;
                m_sInputFormatDepth.nPaletteSize = pFormat->nPaletteSize;
                m_bFirstFrameDepth = tFalse;
        	}
        	
        	ProcessDepthInput(pMediaSample);
        }
         
        else if(pSource == &m_iInputStart)
        {
            ProcessState(pMediaSample);              
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

/*
 * Set working state of this filter
 */
tResult cVisualOdometry::ProcessState(IMediaSample* pMediaSample){

	tBool bValue = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pStartInput ,pMediaSample, pCoder);    

		if (!m_bStartInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStartInput);		
			m_bStartInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueStartInput, (tVoid*)&bValue);         
	}

	m_bActive = bValue;

	RETURN_NOERROR;
}

/*
 * If we received a RGB image and the working state is set to true,
 * than an approxiamation of driven distance since this filter set
 * to 'start' is calculated.
 */
tResult cVisualOdometry::ProcessRGBInput(IMediaSample* pSample, tTimeStamp tsInputTime)
{

    if(m_ui8InitCtrl < 150)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {     
        m_ui8InitCtrl++;
		LOG_INFO(cString::Format("VO: Zaehler %i, Value %i", m_ui8InitCtrl, m_bActive));	
    }
    else if( m_bActive && m_DepthImage.data ) // Only start if there is a depth image saved
    {	
		// Stop updating depth image
		m_getDepthImage 	= tFalse;

		const tVoid* l_pSrcBuffer;
		
		// Get image out of mediaSample
		IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
       	oImg->imageData = (char*)l_pSrcBuffer;
      	Mat image_new(cvarrToMat(oImg));
       	cvReleaseImage(&oImg);
       	pSample->Unlock(l_pSrcBuffer);

		if ( !m_image.data ){
			
			// Save as grey image
			cvtColor(image_new, m_image, COLOR_BGR2GRAY);
		}

		Mat image_old_grey, image_new_grey;

		m_image.copyTo(image_old_grey);

		cvtColor(image_new, image_new_grey, COLOR_BGR2GRAY);

		vector<Point2f> points_old, points_new;

		// Get points
		if ( m_points.size() < MIN_NUM_FEAT ){
			// Feature Detection
			
			/*TermCriteria termCrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
			const int MAX_COUNT 	= 500;
			Size subPixWinSize(10,10);

			goodFeaturesToTrack( image_old_grey, points_old, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04 );
			cornerSubPix(image_old_grey, points_old, subPixWinSize, Size(-1,-1), termCrit);*/
			
			//LOG_INFO(cString::Format( "VO: Calculating new tracking points" ) );

			vector<KeyPoint> keypoints_old;
			int fast_threshold = 30;
			bool nonmaxSuppression = true;
	

			FAST(image_old_grey, keypoints_old, fast_threshold, nonmaxSuppression);
			KeyPoint::convert( keypoints_old, points_old, vector<int>() );


			// Update last world coordinates too 
			m_lastCoordinates.clear();

			for (size_t i = 0; i < points_old.size(); i++){

				Point3f point = Util::ComputeWorldCoordinate( points_old[i].x, points_old[i].y, m_DepthImage.at<ushort>(floor(points_old[i].y*0.5), floor(points_old[i].x*0.5)) );
				m_lastCoordinates.push_back(point);			
			}

		}else{
			points_old 	= m_points;
		}

		// Don't start calculating with zero points -> could cause crahes!
		if (points_old.size() == 0){
			// Allways clear depth image at the end
			LOG_WARNING(cString::Format("VO: Couldn't find any tracking points!") );
			m_DepthImage.release();
			m_getDepthImage = tTrue;
			m_points.clear();
			m_image.release();
			RETURN_NOERROR;
		}

		LOG_INFO(cString::Format("VO: Found Points : %i", points_old.size() ) );

		// Feature Tracking
		vector<uchar> status;

		vector<float> err;
		Size winSize = Size(3,3);
		TermCriteria termCrit2(TermCriteria::COUNT|TermCriteria::EPS,30,0.003);

		calcOpticalFlowPyrLK(image_old_grey, image_new_grey, points_old, points_new, status, err, winSize, 
			3, termCrit2, 0, 0.001);

		int indexCorrection = 0;
		for ( unsigned int i = 0; i < status.size(); i++){

			Point2f point = points_new.at( i - indexCorrection );
			if ( (status.at(i) == 0) || (point.x<X_MIN) || (point.y<Y_MIN) || (point.x>X_MAX) || (point.y>Y_MAX) ){
			
				if ( (point.x<X_MIN) || (point.y<Y_MIN) || (point.x>X_MAX) || (point.y>Y_MAX) )	status.at(i) = 0;

				points_old.erase( points_old.begin() + i - indexCorrection );
				points_new.erase( points_new.begin() + i - indexCorrection );
				m_lastCoordinates.erase( m_lastCoordinates.begin() + i - indexCorrection );
				indexCorrection++;
			}
		}
		//LOG_INFO(cString::Format("CO: IndexCorrection = %i",indexCorrection) );

		// Consistency check
		if (points_old.size() != points_new.size() ) LOG_WARNING(cString::Format("VO: points_old.size() - points_new.size() = %i" ,points_old.size() - points_new.size() ) );

		size_t 	number_points 	= points_new.size();

		// Get coordinates of found points
		vector<Point3f> coordinates_new;


		for (size_t i = 0; i < number_points; i++){

			Point3f point = Util::ComputeWorldCoordinate( points_new[i].x, points_new[i].y, m_DepthImage.at<ushort>(floor(points_new[i].y*0.5), floor(points_new[i].x*0.5)) );
			coordinates_new.push_back(point);
			
		}

		// Consistency check
		if (coordinates_new.size() != number_points) LOG_WARNING(cString::Format("VO: coordinates_new.size() - number_points = %i" ,coordinates_new.size() - number_points) );

		// If there are coordinates to compare with, calculate driven distance
		if ( m_lastCoordinates.size() > 0){
			
			// Consistency check
			if (m_lastCoordinates.size() != number_points) LOG_WARNING(cString::Format("VO: m_lastCoordinates.size() - number_points = %i" ,m_lastCoordinates.size() - number_points) );

			// Calculate mean driven distance
			tFloat32 current_distance 	= 0.0;
			
			for (size_t i = 0; i < number_points; i++){
				current_distance += ( m_lastCoordinates[i].z - coordinates_new[i].z );
			}
			
			m_DrivenDistance 	+= 	current_distance / number_points;
			
			SendDrivenDistance();
		}
		
		// Allways clear depth image at the end
		m_DepthImage.release();
		m_getDepthImage = tTrue;
		
		// Save data for next calculation
		image_new_grey.copyTo(m_image);
		m_points 			= points_new;
		m_lastCoordinates 	= coordinates_new;
	

    } else { // Not active -> delete last values

		m_image.release();
		m_points.clear();
		m_lastCoordinates.clear();
		m_DrivenDistance 	= 0.0;
	}

    RETURN_NOERROR;            
}

/*
 * Receive and store depth image, if a new one is needed
 */
tResult cVisualOdometry::ProcessDepthInput(IMediaSample* pMediaSample){
	
	// Only get depth image if this is necessary
	if (m_getDepthImage && m_bActive){
		
		const tVoid* l_pSrcBuffer;
		
		// Get depth image out of mediaSample
		IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormatDepth.nWidth, m_sInputFormatDepth.nHeight), IPL_DEPTH_16U, 3);
        RETURN_IF_FAILED(pMediaSample->Lock(&l_pSrcBuffer));
       	oImg->imageData = (char*)l_pSrcBuffer;
      	Mat image(cvarrToMat(oImg));
       	cvReleaseImage(&oImg);
       	pMediaSample->Unlock(l_pSrcBuffer);
       	
       	m_DepthImage 		= image;

	}
	
	RETURN_NOERROR;
}

/*
 * Function to send the driven distance since activation as a SignalValue
 */
tResult cVisualOdometry::SendDrivenDistance(){
	
	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDrivenDistanceOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp 	= 0;
	tFloat32 f32Value 		= m_DrivenDistance;
	
	{
		__adtf_sample_write_lock_mediadescription(m_pDrivenDistanceOutput, pMediaSample, pCoder);    

		if (!m_bDrivenDistanceOutputSet){
			pCoder->GetID("f32Value", m_szIDValueDrivenDistanceOutput);
			pCoder->GetID("ui32Timestamp", m_szIDTimestampDrivenDistanceOutput);
			m_bDrivenDistanceOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDValueDrivenDistanceOutput, (tVoid*)&f32Value);     
		pCoder->Set(m_szIDTimestampDrivenDistanceOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oDrivenDistance.Transmit(pMediaSample);

	RETURN_NOERROR;
	
}


tResult cVisualOdometry::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
    
    RETURN_NOERROR;
}

tResult cVisualOdometry::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

tResult cVisualOdometry::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}
