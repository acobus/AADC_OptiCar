/**
*
*ImageProcessing
*
* Date 17.03.2015
*
*/
 
 
#include "stdafx.h"
#include "ImageProcessing.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>


ADTF_FILTER_PLUGIN("ImageProcessing", OID_ADTF_ImageProcessing, cImageProcessing)

#define ImageProcessing_PROP_CAMERA_OFFSET "ImageProcessing::Camera Offset"
#define ImageProcessing_PROP_TRESHOLD "ImageProcessing::ThresholdValue"
#define ImageProcessing_PROP_TRESHOLD2 "ImageProcessing::ThresholdValue2"
#define ImageProcessing_PROP_CornerHarrisparamK "ImageProcessing::CornerHarrisparamK"
#define ImageProcessing_PROP_CornerHarrisblockSize "ImageProcessing::CornerHarrisblockSize"
#define ImageProcessing_PROP_CornerHarrisksize "ImageProcessing::CornerHarrisksize"

#define ImageProcessing_PROP_ImagecutWidthLeft "ImageProcessing::ImagecutWidthLeft"
#define ImageProcessing_PROP_ImagecutWidthRight "ImageProcessing::ImagecutWidthRight"
#define ImageProcessing_PROP_ImagecutHeightUp "ImageProcessing::ImagecutHeightUp"
#define ImageProcessing_PROP_ImagecutHeightDown "ImageProcessing::ImagecutHeightDown"

#define ImageProcessing_PROP_SHOW_DEBUG "Common::Show Debug"





#define MAX_DEVIATION 150


//TODO 1
//#define LT_ENABLE_CANNY_WINDOWS


cImageProcessing::cImageProcessing(const tChar* __info) : cFilter(__info)
{
	//psPoints.x=0;
	pui8PointsCount    = 0;
	RGBreceived 	   = false;

	// parallel parking
	lotDim[0][0] = 0.820;
	lotDim[0][1] = 0.78;
	// cross parking
	lotDim[1][0] = 0.52;
	lotDim[1][1] = 0.43;

    
    SetPropertyBool(ImageProcessing_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(ImageProcessing_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");


	SetPropertyInt(ImageProcessing_PROP_ImagecutWidthLeft, 60);
	SetPropertyInt(ImageProcessing_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(ImageProcessing_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ImageProcessing_PROP_ImagecutWidthRight, 120);
	SetPropertyInt(ImageProcessing_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(ImageProcessing_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ImageProcessing_PROP_ImagecutHeightUp, 240);
	SetPropertyInt(ImageProcessing_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(ImageProcessing_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ImageProcessing_PROP_ImagecutHeightDown, 480);
	SetPropertyInt(ImageProcessing_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(ImageProcessing_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");



    SetPropertyFloat(ImageProcessing_PROP_CAMERA_OFFSET, 15);
    SetPropertyBool(ImageProcessing_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");


    SetPropertyFloat(ImageProcessing_PROP_TRESHOLD, 200);
    SetPropertyBool(ImageProcessing_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The first threshold value.");
	
	SetPropertyFloat(ImageProcessing_PROP_TRESHOLD2, 200);
    SetPropertyBool(ImageProcessing_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The second threshold value.");

	SetPropertyInt(ImageProcessing_PROP_CornerHarrisblockSize, 2);
    SetPropertyBool(ImageProcessing_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(ImageProcessing_PROP_CornerHarrisksize, 3);
    SetPropertyBool(ImageProcessing_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(ImageProcessing_PROP_CornerHarrisparamK, 0.04);
    SetPropertyBool(ImageProcessing_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ImageProcessing_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");
    


    m_pISignalRegistry = NULL;
}

cImageProcessing::~cImageProcessing()
{
}

tResult cImageProcessing::GetInterface(const tChar* idInterface,
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

tUInt cImageProcessing::Ref()
{
    return cFilter::Ref();
}

tUInt cImageProcessing::Unref()
{
    return cFilter::Unref();
}

tVoid cImageProcessing::Destroy()
{
    delete this;
}

tResult cImageProcessing::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cImageProcessing::Stop(__exception)
{

// TODO 1
#ifdef LT_ENABLE_CANNY_WINDOWS
    if(m_bShowDebug)
    {
        //destroyWindow("Canny Near");
        //destroyWindow("Canny Far");
        //destroyWindow("RGB Image");
        //destroyWindow("Canny");
        //destroyWindow("GreyScale Image");
        //destroyWindow("Binary Image");
        //destroyWindow("Canny Image");
        //destroyWindow("LaneTracking");  
    }  
#endif    

    return cFilter::Stop(__exception_ptr);
}
tResult cImageProcessing::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// Media Description ParkingID
		tChar const * ParkingDesc = pDescManager->GetMediaDescription("tParkingStruct");
		RETURN_IF_POINTER_NULL(ParkingDesc);
		cObjectPtr<IMediaType> pTypeParkingDesc = new cMediaType(0, 0, 0, "tParkingStruct", ParkingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description Parking 
		tChar const * FoundParkingLotDesc = pDescManager->GetMediaDescription("tParkingLotStruct");
		RETURN_IF_POINTER_NULL(FoundParkingLotDesc);
		cObjectPtr<IMediaType> pTypeFoundParkingLotDesc = new cMediaType(0, 0, 0, "tParkingLotStruct", FoundParkingLotDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("Start", pTypeParkingDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeParkingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPark)); 


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output", pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

		// Create Pin for DistanceToParkingSpot (Output)
		RETURN_IF_FAILED(m_oDistanceToParkingSpot.Create("DistanceToParkingSpot", pTypeFoundParkingLotDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceToParkingSpot));
		RETURN_IF_FAILED(pTypeFoundParkingLotDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceToParkingSpotOutput)); 


              
    }
    else if (eStage == StageNormal)
    {
		m_startInputSet = tFalse;
		m_bActive = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
        m_ui8Imagecount = 0;

		m_parkingSpotOutputSet = tFalse;

        ReadProperties(NULL);

        m_ui8InitCtrl = 0;

		m_iframeCounter = 0;
        
        
        
        if (m_bShowDebug)
        {
        }

    }
    RETURN_NOERROR;
}

tResult cImageProcessing::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cImageProcessing::ReadProperties(const tChar* strPropertyName)
{
    
if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_CornerHarrisparamK))
    {
		float f;
		f = GetPropertyFloat(ImageProcessing_PROP_CornerHarrisparamK);
		m_nCornerHarrisparamK = static_cast<double>(f);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_CornerHarrisblockSize))
    {
        m_nCornerHarrisblockSize = GetPropertyInt(ImageProcessing_PROP_CornerHarrisblockSize);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_CornerHarrisksize))
    {
        m_nCornerHarrisksize = GetPropertyInt(ImageProcessing_PROP_CornerHarrisksize);
    }


    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_TRESHOLD))
    {
        m_nThresholdValue = GetPropertyInt(ImageProcessing_PROP_TRESHOLD);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_TRESHOLD))
    {
        m_nThresholdValue2 = GetPropertyInt(ImageProcessing_PROP_TRESHOLD2);
    }
	
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(ImageProcessing_PROP_SHOW_DEBUG);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_ImagecutWidthLeft))
    {
        m_nImagecutWidthLeft = GetPropertyInt(ImageProcessing_PROP_ImagecutWidthLeft);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_ImagecutWidthRight))
    {
        m_nImagecutWidthRight = GetPropertyInt(ImageProcessing_PROP_ImagecutWidthRight);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_ImagecutHeightUp))
    {
        m_nImagecutHeightUp = GetPropertyInt(ImageProcessing_PROP_ImagecutHeightUp);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ImageProcessing_PROP_ImagecutHeightDown))
    {
        m_nImagecutHeightDown = GetPropertyInt(ImageProcessing_PROP_ImagecutHeightDown);
	}


    RETURN_NOERROR;
}

tResult cImageProcessing::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cImageProcessing::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    if(m_bShowDebug && eStage == cFilter::StageNormal)
    {        
        ucom::cObjectPtr<ISignalRegistry> pSignalRegistry;
        if (IS_OK(_runtime->GetObject(OID_ADTF_SIGNAL_REGISTRY,
            IID_ADTF_SIGNAL_REGISTRY,
            (tVoid**)&pSignalRegistry)))
        {
            // Unregister the provider
            pSignalRegistry->UnregisterSignalProvider(this);
        }
        m_oActive.clear();

        m_oLock.Release();
    }

    
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cImageProcessing::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        tTimeStamp InputTimeStamp;
        InputTimeStamp = pMediaSample->GetTime();


        if(pSource == &m_iVideoInputPin)
        {
			if(!RGBreceived)//if RGB not received we want RGB
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
            	    m_bFirstFrame = false; 
            	}

            	ProcessInputRGB(pMediaSample, InputTimeStamp);
			}
        }
        if(pSource == &m_iDepthimagePin )
        {
			if(RGBreceived) //If RGB received we want depthimage
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
            	    m_bFirstFrameDepthimage = false;
            	}

            	ProcessInputDepth(pMediaSample, InputTimeStamp);
			}
        }

        else if(pSource == &m_iStart)
        {
            tBool bValue = tFalse;
			tInt32 type;
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pCoderDescPark,pMediaSample,pCoder);

				if (!m_startInputSet) {
					pCoder->GetID("bStart", m_szIDBoolValueStartInput);
					pCoder->GetID("ID", m_szIDStartInput);
					m_startInputSet = tTrue;
				}

                pCoder->Get(m_szIDBoolValueStartInput, (tVoid*)&bValue);
				pCoder->Get(m_szIDStartInput, (tVoid*)&type);
            }

			m_bActive = bValue;
			m_parkingType = type-3;
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cImageProcessing::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{ if (m_bActive){
/*    if(m_ui8InitCtrl < 10)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
        m_ui8InitCtrl++;
    }
    else 
    {*/
			m_iframeCounter++;
			
		    // VideoInput
		    RETURN_IF_POINTER_NULL(pSample);

		    const tVoid* l_pSrcBuffer;
		    //char* corners_window="Corners detected";
		
		    /*IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
		
		    cvSetData(img, (tVoid*)l_pSrcBuffer, img->widthStep);
		    Mat image(cvarrToMat(img));
		    cvReleaseImage(&img);
		    pSample->Unlock(l_pSrcBuffer);*/
		
		    IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
		    oImg->imageData = (char*)l_pSrcBuffer;
		    Mat image(cvarrToMat(oImg));
		    cvReleaseImage(&oImg);
		    pSample->Unlock(l_pSrcBuffer);

		
		    // Transform image
		    Mat m_matImageRGB= image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image
			// TODO compress

			cvtColor(m_matImageRGB, m_matGrey, CV_RGB2GRAY);// Grey Image
			medianBlur(m_matScaled, m_matScaled, 5); // reduce noise with edge-preserving filter
			cornerHarris(m_matGrey, m_matCorner, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			threshold(m_matCorner, m_matThres, m_nThresholdValue, 255,THRESH_TOZERO);
			normalize(m_matThres, m_matNormalized, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
			convertScaleAbs(m_matNormalized, m_matScaled); // Konveriere in 8Bit array. Nehme aus abs
/*stream f;
f.open("mat.dat",ios::out);
f << m_matScaled ;
f.close();*/
			threshold(m_matScaled, m_matThres2, m_nThresholdValue2, 255,THRESH_BINARY); 

			m_matNormal = m_matImageRGB;  // memeber var um in Search Corners verwendet zu werden

	#ifdef LT_ENABLE_CANNY_WINDOWS
		    //**************** Show Image *************************
		    if(m_bShowDebug)
		    {
		        if(m_ui8Imagecount > 2)
		        {
		            m_ui8Imagecount=0;
		            //imshow("RGB Image", image);
		            //imshow("Canny", m_matThres);
		            //imshow("GreyScale Image", greyNear);
		            //imshow("Binary Image", greythreshNear);
		            //imshow("Canny Image", linecanny);
		            //imshow("Canny Near", m_matLineCannyNear);
		            //imshow("Canny Far", m_matLineCannyFar);
		            waitKey(0);
					sleep(3);
		        }
		        m_ui8Imagecount++;
		    }
		    //********************************************************
	#endif

		    //********** Find horizontal line (for calibration only)**************
		    //tInt edgerow = 0;
		    //for(tInt row=cannysize.height;row>=1;row--)                                                                                                                                                                                  
		    //{            
		    //    if(linecanny.at<uchar>(row,cannysize.width/2)!=0) 
		    //    {    
		    //        edgerow = row;
		    //        row = 0;
		    //    }
		    //}
		    //LOG_INFO(cString::Format("Found edge at row: %i", edgerow));
		    //********************************************************************    

		    //LOG_INFO(cString::Format("lfdnr1: %i",lfdnr1));
		RGBreceived = true;
		m_bActive = tFalse; // damit nur ein RGB und Tiefenbild empfangen wird
		
	}
 //}
	RETURN_NOERROR;            
}

tResult cImageProcessing::ProcessInputDepth(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	const tVoid* l_pSrcBuffer;

	IplImage* oImg = cvCreateImage(cvSize(m_sInputFormatDepthimage.nWidth, m_sInputFormatDepthimage.nHeight), IPL_DEPTH_16U, 3);
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	oImg->imageData = (char*)l_pSrcBuffer;
	Mat image(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	pSample->Unlock(l_pSrcBuffer);

	Mat m_depthImage = Mat(m_sInputFormatDepthimage.nHeight,m_sInputFormatDepthimage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthimage.nBytesPerLine);

	// Transform image
	m_matImageDepth= m_depthImage(cv::Range(m_nImagecutHeightUp/2, m_nImagecutHeightDown/2), cv::Range(m_nImagecutWidthLeft/2, m_nImagecutWidthRight/2)).clone(); //Cut Image

	//cvtColor(image, image_GREY, CV_RGB2GRAY);// Grey Image 
	SearchCorners();
	CreateAndTransmitGCL();
	RGBreceived=false;	
	
	RETURN_NOERROR;            
}



tResult cImageProcessing::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cImageProcessing::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cImageProcessing::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cImageProcessing::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cImageProcessing::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

tResult cImageProcessing::SearchCorners()
{	

	pui8PointsCount    = 0;
    Size szImageCutThresSize = m_matThres2.size();
	Points.clear(); 
	
	// Write corners from Matrix to vector
    for(tInt nColumn=0; nColumn < szImageCutThresSize.width; nColumn++)
    {  
    	for(tInt nRow=0; nRow < szImageCutThresSize.height; nRow++)
    		{  
                 
        	if(m_matThres2.at<uchar>(nRow, nColumn)==255) 
        		{ 
				Points.push_back(Point2f(nColumn,nRow));         
            	}
        	}
    }

	vector<Point3f> ParkPoint;
	Point3f corner1;
	Point3f corner2;

	//elimininate pointcloud TODO Hough line hier einbauen
		vector<int> killUs;
		for(size_t i=0;i<Points.size();i++)
			{	
			corner1 = Util::ComputeWorldCoordinate(Points[i].x, Points[i].y,m_matImageDepth.at<ushort>(floor(Points[i].y*0.5),floor(Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			for(size_t j=i+1;j<Points.size();j++)
				{
					corner2 = Util::ComputeWorldCoordinate(Points[j].x, Points[j].y,m_matImageDepth.at<ushort>(floor(Points[j].y*0.5), floor(Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
					if(norm(corner1-corner2)<0.18)
						{
						if(norm(corner1)< norm(corner2))
							{ 
							if(find(killUs.begin(), killUs.end(), j) == killUs.end()) killUs.push_back(j);
							}
							else
							{
							if(find(killUs.begin(), killUs.end(), i) == killUs.end()) killUs.push_back(i);
							}						
						}			
				}
			}

	if(!killUs.empty()){
	sort(killUs.begin(),killUs.end(),greater<int>());
	for(size_t i=0;i<killUs.size();i++)
		{
		Points.erase(Points.begin()+killUs[i]);
		}
	}

	// Search actual parking lot
	 for(size_t i=0;i<Points.size();i++)
		{
		corner1 = Util::ComputeWorldCoordinate(Points[i].x, Points[i].y,m_matImageDepth.at<ushort>(floor(Points[i].y*0.5),floor(Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
	 	for(size_t j=i+1;j<Points.size();j++)
			{
			corner2 = Util::ComputeWorldCoordinate(Points[j].x, Points[j].y,m_matImageDepth.at<ushort>(floor(Points[j].y*0.5), floor(Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
			if(norm(corner1-corner2)<lotDim[m_parkingType][0] && norm(corner1-corner2)>lotDim[m_parkingType][1]){
					PixelSpot.push_back(Point2f(Points[i].y,Points[i].x));
					PixelSpot.push_back(Point2f(Points[j].y,Points[j].x));
					ParkPoint.push_back(corner1);
					ParkPoint.push_back(corner2);
					}
				}
			}



	int idx = -1;
	double buffer;
	double MIN1 = 10000;//norm(ParkPoint.at(0));
	for(size_t i=0; i<ParkPoint.size();i++)
		{
			buffer = norm(ParkPoint[i]);
			if(buffer<MIN1)
				{
					MIN1 = buffer;
					idx = i;
				}
		}
if(ParkPoint.size()!=0){
		/*fstream f;
f.open("Parklot.dat",ios::out|ios::app);
f << ParkPoint[idx].y ;
f.close();*/
}
	if(m_iframeCounter>400)
	{
				fstream f1;
/*f1.open("size1.dat",ios::out|ios::app);
f1 << ParkPoint.size() ;
f1.close();*/

		LOG_WARNING(cString::Format("!!!!!!ImageProcessingParking: Parkmaneuver cancelled. Timeframe of 12s reached!!!!!"));
		vector<float> temp;
		for(int i=0; i < 6; i++)
		{
			temp.push_back(0.0);
		}
		m_iframeCounter = 0;
		TransmitParkingSpot(temp);
		RETURN_NOERROR;
	}
	else if(ParkPoint.empty())
	{
		LOG_WARNING(cString::Format("ImageProcessingParking: No parking lot found!"));
		// Try again if failed
		m_bActive = tTrue;
	}
	else if(ParkPoint[idx].z > 2.3 || ParkPoint[idx].z < 0.8){
		LOG_WARNING(cString::Format("ImageProcessingParking: No valid parking lot found!"));
		// Try again if failed
		m_bActive = tTrue;
 	}
	else if(ParkPoint.size()<3 && ParkPoint[idx].z>1.0)
	{
				fstream f4;
f4.open("size4.dat",ios::out|ios::app);
f4 << ParkPoint.size() ;
f4.close();

		LOG_INFO(cString::Format("IP: Parking Lot found, but only one. I need at least two!") );
		m_bActive = tTrue;
	}
	else
	{
				/*fstream f5;
f5.open("size5.dat",ios::out|ios::app);
f5 << ParkPoint.size() << ParkPoint[0].z << ParkPoint[1].z;
f5.close();
				fstream f9;
f9.open("size9.dat",ios::out|ios::app);
f9 << ParkPoint[idx].z;
f9.close();*/

		// Schreibe gefundene Parkluecke in Outputpin	
 		if (!m_oDistanceToParkingSpot.IsConnected())
   		{
			LOG_WARNING(cString::Format("ImageProcessingParking: OutputPin for parking lot not connected!"));
    	    RETURN_NOERROR;
    	}
		// Send found parking spot
		vector<float> temp;
		for(size_t i=0; i < ParkPoint.size(); i++)
		{
			temp.push_back(ParkPoint[i].z);
		}
		sort(temp.begin(),temp.end());
		for(size_t i=temp.size(); i < 6; i++)
		{
			temp.push_back(0.0);
		}
		
		m_iframeCounter = 0;
		TransmitParkingSpot(temp);
	}
	LOG_INFO(cString::Format("IP: Finished Searching") );

	RETURN_NOERROR;
}


tResult cImageProcessing::CreateAndTransmitGCL()
{
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput.IsConnected() || !m_bShowDebug)
    {
        RETURN_NOERROR;
    }

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
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,m_nImagecutWidthLeft , m_nImagecutHeightUp, m_nImagecutWidthRight, m_nImagecutHeightDown);

	for(size_t i=0; i < Points.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(Points[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(Points[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(Points[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(Points[i].y+2) + m_nImagecutHeightUp);
		}
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
	for(size_t i=0; i < PixelSpot.size(); i++)
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(PixelSpot[i].y-2) + m_nImagecutWidthLeft , static_cast<tInt16>(PixelSpot[i].x-2) + m_nImagecutHeightUp, static_cast<tInt16>(PixelSpot[i].y+2) + m_nImagecutWidthLeft, static_cast<tInt16>(PixelSpot[i].x+2) + m_nImagecutHeightUp);
		}
	PixelSpot.clear();

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

tResult cImageProcessing::TransmitParkingSpot(vector<float> lots) {

		if(lots.size()!=6) LOG_WARNING(cString::Format("ImageProcessingParking: size of output pin data invalid!"));

		tUInt32 nTimeStamp = 0;

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistanceToParkingSpotOutput->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pDistanceToParkingSpotOutput, pMediaSample, pCoderOutput); 
	
			if(!m_parkingSpotOutputSet){
				pCoderOutput->GetID("fz1", m_szIDParkingSpot1Output);
				pCoderOutput->GetID("fz2", m_szIDParkingSpot2Output);
				pCoderOutput->GetID("fz3", m_szIDParkingSpot3Output);
				pCoderOutput->GetID("fz4", m_szIDParkingSpot4Output);
				pCoderOutput->GetID("fz5", m_szIDParkingSpot5Output);
				pCoderOutput->GetID("fz6", m_szIDParkingSpot6Output);
				pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampParkingSpotOutput);
				m_parkingSpotOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szIDParkingSpot1Output, (tVoid*)&(lots[0]));
			pCoderOutput->Set(m_szIDParkingSpot2Output, (tVoid*)&(lots[1]));
			pCoderOutput->Set(m_szIDParkingSpot3Output, (tVoid*)&(lots[2]));
			pCoderOutput->Set(m_szIDParkingSpot4Output, (tVoid*)&(lots[3]));
			pCoderOutput->Set(m_szIDParkingSpot5Output, (tVoid*)&(lots[4]));
			pCoderOutput->Set(m_szIDParkingSpot6Output, (tVoid*)&(lots[5]));
			pCoderOutput->Set(m_szIDTimestampParkingSpotOutput, (tVoid*)&nTimeStamp);
		
		}
/*fstream f1;
f1.open("output2IP.dat",ios::out|ios::app);
for(size_t i=0; i<6;i++)
{
	f1 << "\n" << lots[i] ;
}
f1.close();*/

		LOG_INFO(cString::Format("IP: Send Parking Lot to DM!!!!") );

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceToParkingSpot.Transmit(pMediaSample);

	RETURN_NOERROR;
}
