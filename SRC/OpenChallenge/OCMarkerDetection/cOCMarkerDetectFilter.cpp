/*
 * Date 17.03.2016
 */

#include "stdafx.h"
#include "aadc_roadSign_enums.h"
#include "cOCMarkerDetectFilter.h"
#include <fstream>

#define NECESSARY_HITS 5


ADTF_FILTER_PLUGIN("OC Marker Detection Filter Plugin", OID_ADTF_OCMARKERDETECTFILTER, cOCMarkerDetectFilter)



cOCMarkerDetectFilter::cOCMarkerDetectFilter(const tChar* __info):cFilter(__info)
{
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start", m_oProcessStart);

    SetPropertyBool("Debug Output to Console", tFalse);    
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)."); 

    SetPropertyStr("Dictionary File For Markers",""); 
    SetPropertyBool("Dictionary File For Markers" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)"); 
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_DESCRIPTION, "Here you have to set the dicitionary file which holds the marker ids and their content"); 

    SetPropertyStr("Calibration File for used Camera",""); 
    SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)"); 
    SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION, "Here you have to set the file with calibration paraemters of the used camera"); 
    
    SetPropertyInt("Video Output Pin",1);
    SetPropertyStr("Video Output Pin" NSSUBPROP_VALUELIST, "1@None|2@Detected Signs|");
    SetPropertyStr("Video Output Pin" NSSUBPROP_DESCRIPTION, "If enabled the video stream with the highlighted markers is transmitted at the output pin (Warning: decreases performance)."); 

    SetPropertyFloat("Size of Markers",0.117f);
    SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION, "Size (length of one side) of markers in m"); 

    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End", m_oProcessEnd);    
}

cOCMarkerDetectFilter::~cOCMarkerDetectFilter()
{
}

tResult cOCMarkerDetectFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
        {
        //create the video rgb input pin
        RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this))); 
        RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

        //create the video rgb output pin
        RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

        // create the description manager
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager,__exception_ptr));
        
		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
		
        
		// State of StartDriving (Output)
		RETURN_IF_FAILED(m_oStartDriving.Create("Start_Driving", pTypeBoolSignalValue, this));
		RETURN_IF_FAILED(RegisterPin(&m_oStartDriving));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStartDrivingOutput));
		
		// State of StartScale (Output)
		RETURN_IF_FAILED(m_oStartScale.Create("Start_Scale", pTypeBoolSignalValue, this));
		RETURN_IF_FAILED(RegisterPin(&m_oStartScale));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStartScaleOutput));

        }
    else if (eStage == StageNormal)
        {
            // get the propeerties
            m_iOutputMode = GetPropertyInt("Video Output Pin");
            m_f32MarkerSize = static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));
            m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");


            m_bCamaraParamsLoaded = tFalse;

            //Get path of marker configuration file
            cFilename fileConfig = GetPropertyStr("Dictionary File For Markers");
            //create absolute path for marker configuration file
            ADTF_GET_CONFIG_FILENAME(fileConfig);
            fileConfig = fileConfig.CreateAbsolutePath(".");
            
            //check if marker configuration file exits
            if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
                {
                LOG_ERROR("Dictionary File for Markers not found");
                RETURN_ERROR(ERR_INVALID_FILE);
                }   
            else
                {
                //try to read the marker configuration file
                if(m_Dictionary.fromFile(string(fileConfig))==false)
                    {
                    RETURN_ERROR(ERR_INVALID_FILE);
                    LOG_ERROR("Dictionary File for Markers could not be read");
                    }
                if(m_Dictionary.size()==0)
                    {
                    RETURN_ERROR(ERR_INVALID_FILE);
                    LOG_ERROR("Dictionary File does not contain valid markers or could not be read sucessfully");
                    }
                //set marker configuration file to highlyreliable markers class
                if (!(HighlyReliableMarkers::loadDictionary(m_Dictionary)==tTrue))
                    {
                    LOG_ERROR("Dictionary File could not be read for highly reliable markers");
                    }
                } 
    
            //Get path of calibration file with camera paramters
            cFilename fileCalibration = GetPropertyStr("Calibration File for used Camera"); ;
        
            //Get path of calibration file with camera paramters
            ADTF_GET_CONFIG_FILENAME(fileCalibration);
            fileCalibration = fileCalibration.CreateAbsolutePath(".");
            //check if calibration file with camera paramters exits
            if (fileCalibration.IsEmpty() || !(cFileSystem::Exists(fileCalibration)))
                {
                LOG_ERROR("Calibration File for camera not found");
                }   
            else
                {
                // read the calibration file with camera paramters exits and save to member variable
                m_TheCameraParameters.readFromXMLFile(fileCalibration.GetPtr());
                cv::FileStorage camera_data (fileCalibration.GetPtr(),cv::FileStorage::READ);
                camera_data ["camera_matrix"] >> m_Intrinsics; 
                camera_data ["distortion_coefficients"] >> m_Distorsion;    
                m_bCamaraParamsLoaded = tTrue;
                }   
        }
    else if (eStage == StageGraphReady)
        {
    	
    	// Set Setter for ids
    	m_bStartDrivingOutputSet 	= tFalse;
    	m_bStartScaleOutputSet 		= tFalse;
    	
    	m_startCounter	 		= 0;
    	m_scaleCounter 			= 0;
    	
    	m_startTime 			= 0;
    	m_scaleTime 			= 0;
    	
        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
    
        // set the image format of the input video pin
        UpdateInputImageFormat(pTypeVideo->GetFormat());
        //TODO: why was the following line inserted?
        //m_oPinInputVideo.SetFormat(&m_sInputFormat, NULL);  
    
        // set the image format of the output video pin
        UpdateOutputImageFormat(pTypeVideo->GetFormat());  
    
        // set paramenter for the marker detector class
        // same parameters as in aruco sample aruco_hrm_test.cpp
        // effects of parameters has to be tested
        m_MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
        m_MDetector.setThresholdParams( 21, 7);
        m_MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        m_MDetector.setWarpSize((m_Dictionary[0].n()+2)*8);
        m_MDetector.setMinMaxSize(0.005f, 0.5f);

        }
    RETURN_NOERROR;
}

tResult cOCMarkerDetectFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cOCMarkerDetectFilter::OnPinEvent(IPin* pSource,
                                            tInt nEventCode,
                                            tInt nParam1,
                                            tInt nParam2,
                                            IMediaSample* pMediaSample)
{
    switch (nEventCode)
            {
            case IPinEventSink::PE_MediaSampleReceived:
                // a new image was received so the processing is started
                if (pSource == &m_oPinInputVideo)
                {                
                    UCOM_TIMING_SPOT(m_oProcessStart);
                    ProcessVideo(pMediaSample);        
                    UCOM_TIMING_SPOT(m_oProcessEnd);
                }                    
                break;
            case IPinEventSink::PE_MediaTypeChanged:
                if (pSource == &m_oPinInputVideo)
                {
                    //the input format was changed, so the imageformat has to changed in this filter also
                    cObjectPtr<IMediaType> pType;
                    RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

                    cObjectPtr<IMediaTypeVideo> pTypeVideo;
                    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

                    UpdateInputImageFormat(m_oPinInputVideo.GetFormat());    
                    UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());                
                }            
                break;
            default:
                break;
        }    
    RETURN_NOERROR;
}



tResult cOCMarkerDetectFilter::ProcessVideo(adtf::IMediaSample* pISample)
{
    
    RETURN_IF_POINTER_NULL(pISample);

    //creating new media sample for output
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));    
    
    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    //creating matrix for input image
    Mat TheInputImage;
    Mat TheOutputImage;
	
    //receiving data from input sample, and saving to TheInputImage
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
        {
        //convert to mat
        Mat FullImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);

		TheInputImage = FullImage.clone();//Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);

		/*
		for (int i = m_sInputFormat.nWidth-210; i < m_sInputFormat.nWidth; ++i) {
			for (int j = 0; j < m_sInputFormat.nHeight; ++j) {
				Vec3b color = FullImage.at<Vec3b>(Point(i,j));
				//if (i > m_sInputFormat.nWidth - 210) {	
					color[0] = 100;
					color[1] = 100;
					color[2] = 100;
				//}
				TheInputImage.at<Vec3b>(Point(i,j)) = color;
			}
		}
		*/ // Dont do this here!
		
   
		//Mat FullImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);
		//LOG_INFO(cString::Format("MARKER %i, %i",m_sInputFormat.nHeight,m_sInputFormat.nWidth));
		//TheInputImage = FullImage(cv::Range(0, 500), cv::Range(150, 300)).clone(); //Cut Image
//TheInputImage=FullImage;
        
        // doing the detection of markers in image
        m_MDetector.detect(TheInputImage,m_TheMarkers,m_TheCameraParameters,static_cast<float>(m_f32MarkerSize));
        }
    pISample->Unlock(l_pSrcBuffer); 

    // 1: nothing is drawn, 2: results are drawn so we need the have copy of the frame otherwise the orginal mediasample is modified
    if (m_iOutputMode==2) 
    {  
        // do a deep copy of the image, otherwise the orginal frame is modified
        TheOutputImage = TheInputImage.clone();
    }
   
    //print marker info and draw the markers in image    
    for (unsigned int i=0;i<m_TheMarkers.size();i++) 
        { 
        if (m_iOutputMode==2) 
            {                
                // draw the marker in the image
                m_TheMarkers[i].draw(TheOutputImage,Scalar(0,0,255),1);
                
                // draw cube in image
                //CvDrawingUtils::draw3dCube(TheInputImage,m_TheMarkers[i],m_TheCameraParameters);
                
                // draw 3d axis in image if the intrinsic params were loaded
                if (m_bCamaraParamsLoaded) CvDrawingUtils::draw3dAxis(TheOutputImage,m_TheMarkers[i],m_TheCameraParameters);                
            }
        
        // call the function to transmit a road sign sample with the detected marker
        if (m_TheMarkers[i].id == 0){
        	if (_clock->GetStreamTime() - m_startTime > 5000000) 	m_startCounter = 0;
        	m_startCounter++;
        	m_startTime = _clock->GetStreamTime();
        	if (m_startCounter == NECESSARY_HITS) 	{
        		SendStartDriving(tTrue);
        		m_startCounter 	= 0;
        	}
        }else if (m_TheMarkers[i].id == 1){
        	if (_clock->GetStreamTime() - m_scaleTime > 5000000) 	m_scaleCounter = 0;
        	m_scaleCounter++;
        	m_scaleTime = _clock->GetStreamTime();
        	if (m_scaleCounter == NECESSARY_HITS) {
        		SendStartScale(tTrue);
        		m_scaleCounter = 0;
        	}
        }
        
        
        
        // call the function to transmit a extended road sign sample with the detected marker if the Tvec in the marker was correctly set
        if (m_bCamaraParamsLoaded) 
            {                
                //get camera position

                if (m_TheMarkers[i].id == 2) 
                {
                    cv::Mat rotationVector( m_TheMarkers[i].Rvec);;
                    cv::Mat translationVector(m_TheMarkers[i].Tvec);;
                    tFloat32 aSignPosition[3];
                    aSignPosition[0] = 0;
                    aSignPosition[1] = 0;
                    aSignPosition[2] = 0;
                    vector<Point3d> m_worldCoordinatesSign;
                    tFloat32 halfSize = m_f32MarkerSize/2;
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]-halfSize,aSignPosition[1] - halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]+halfSize,aSignPosition[1] - halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]+halfSize,aSignPosition[1] + halfSize,aSignPosition[2]));
                    m_worldCoordinatesSign.push_back(Point3d(aSignPosition[0]-halfSize,aSignPosition[1] + halfSize,aSignPosition[2]));
                                      
                    
        

                   vector<Point2d> ImagePoints;
                    for (int c = 0; c < 4; c++) {   
                        
                        //LOG_INFO(cString::Format("Imagepoint %d: %f,%f",c,m_TheMarkers[i][c].x,m_TheMarkers[i][c].y));
                        ImagePoints.push_back(Point2d(m_TheMarkers[i][c].x,m_TheMarkers[i][c].y));
                    }


                    cv::solvePnP(m_worldCoordinatesSign,ImagePoints,m_Intrinsics,m_Distorsion,rotationVector,translationVector,true,0);

                    cv::Mat RodriguesMat;
                    
                    cv::Rodrigues(rotationVector,RodriguesMat);
                    RodriguesMat = RodriguesMat.t();

                    translationVector = -RodriguesMat * translationVector;

                    /*cv::Rodrigues(Rodrigues.t(),cameraRotationVector);
                    cv::Mat t= translationVector.t();
                    cameraTranslationVector= -cameraRotationVector * t;*/                 
                    
                }
            }

        
    }
    
    //update new media sample with image data if something has to be transmitted
    if (m_iOutputMode==2) 
        {
        pNewSample->Update(pISample->GetTime(), TheOutputImage.data, m_sOutputFormat.nSize, 0);
        m_oPinOutputVideo.Transmit(pNewSample);    
        }    

    RETURN_NOERROR;
}

tResult cOCMarkerDetectFilter::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));        
        
    }
    
    RETURN_NOERROR;
}

tResult cOCMarkerDetectFilter::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);    
        
        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
        
        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);                
    }
    
    RETURN_NOERROR;
}

tResult cOCMarkerDetectFilter::SendStartDriving(tBool state){
	
	tUInt32 nTimeStamp = 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStartDrivingOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStartDrivingOutput, pMediaSample, pCoderOutput); 

		if(!m_bStartDrivingOutputSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueStartDrivingOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampStartDrivingOutput);
			m_bStartDrivingOutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueStartDrivingOutput, (tVoid*)&(state));
		pCoderOutput->Set(m_szIDTimestampStartDrivingOutput, (tVoid*)&nTimeStamp);
	
	}
	
	LOG_INFO(cString::Format("OCMD: Sending start driving"));

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oStartDriving.Transmit(pMediaSample);
	
	RETURN_NOERROR;
}

tResult cOCMarkerDetectFilter::SendStartScale(tBool state){

	tUInt32 nTimeStamp = 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStartScaleOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStartScaleOutput, pMediaSample, pCoderOutput); 

		if(!m_bStartScaleOutputSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueStartScaleOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampStartScaleOutput);
			m_bStartScaleOutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueStartScaleOutput, (tVoid*)&(state));
		pCoderOutput->Set(m_szIDTimestampStartScaleOutput, (tVoid*)&nTimeStamp);
	
	}
	
	LOG_INFO(cString::Format("OCMD: Sending start scale mode"));

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oStartScale.Transmit(pMediaSample);
	
	RETURN_NOERROR;
}
