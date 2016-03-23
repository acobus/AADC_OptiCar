/**
*
*CrossingDetect
*
* Date 14.02.2016
*
*/

 
 
#include "stdafx.h"
#include "CrossingDetect.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"
#include "/home/aadc/AADC/src/aadcUser/include/cross_type.h"
#include "/home/aadc/AADC/src/aadcUser/include/action_enum.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>
#include <cmath>


ADTF_FILTER_PLUGIN("CrossingDetect", OID_ADTF_CrossingDetect, cCrossingDetect)

#define CrossingDetect_PROP_CAMERA_OFFSET "CrossingDetect::Camera Offset"
#define CrossingDetect_PROP_TRESHOLD "CrossingDetect::ThresholdValue"
#define CrossingDetect_PROP_ThresholdValueCanny "CrossingDetect::ThresholdValueCanny"
#define CrossingDetect_PROP_ThresholdValueHough "CrossingDetect::ThresholdValueHough"
#define CrossingDetect_PROP_TRESHOLD2 "CrossingDetect::ThresholdValue2"
#define CrossingDetect_PROP_CornerHarrisparamK "CrossingDetect::CornerHarrisparamK"
#define CrossingDetect_PROP_CornerHarrisblockSize "CrossingDetect::CornerHarrisblockSize"
#define CrossingDetect_PROP_CornerHarrisksize "CrossingDetect::CornerHarrisksize"

#define CrossingDetect_PROP_ImagecutWidthLeft "CrossingDetect::ImagecutWidthLeft"
#define CrossingDetect_PROP_ImagecutWidthRight "CrossingDetect::ImagecutWidthRight"
#define CrossingDetect_PROP_ImagecutHeightUp "CrossingDetect::ImagecutHeightUp"
#define CrossingDetect_PROP_ImagecutHeightDown "CrossingDetect::ImagecutHeightDown"

#define CrossingDetect_PROP_ImagecutWidthLeftHough "CrossingDetect::ImagecutWidthLeftHough"
#define CrossingDetect_PROP_ImagecutWidthRightHough "CrossingDetect::ImagecutWidthRightHough"
#define CrossingDetect_PROP_ImagecutHeightUpHough "CrossingDetect::ImagecutHeightUpHough"
#define CrossingDetect_PROP_ImagecutHeightDownHough "CrossingDetect::ImagecutHeightDownHough"

#define CrossingDetect_PROP_SHOW_DEBUG "Common::Show Debug"




#define MAX_FRAMES 400
#define MAX_DEVIATION 150


//TODO 1
//#define LT_ENABLE_CANNY_WINDOWS


cCrossingDetect::cCrossingDetect(const tChar* __info) : cFilter(__info)
{
	//psPoints.x=0;
	//pui8PointsCount    = 0;
	
    
    SetPropertyBool(CrossingDetect_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(CrossingDetect_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");


	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeft, 0);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRight, 640);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUp, 220);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDown, 400);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeftHough, 0);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeftHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthLeftHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRightHough, 320);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRightHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthRightHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough... both Left and Right -> max Val 320");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUpHough, 220);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUpHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightUpHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDownHough, 400);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDownHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightDownHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");



    SetPropertyFloat(CrossingDetect_PROP_CAMERA_OFFSET, 15);
    SetPropertyBool(CrossingDetect_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");


    SetPropertyFloat(CrossingDetect_PROP_TRESHOLD, 0.0001);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The first threshold value.");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueCanny, 50);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueHough, 80);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");
	
	SetPropertyFloat(CrossingDetect_PROP_TRESHOLD2, 40);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The second threshold value.");

	SetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize, 7);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(CrossingDetect_PROP_CornerHarrisksize, 5);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(CrossingDetect_PROP_CornerHarrisparamK, 0.0001);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");
    


    m_pISignalRegistry = NULL;
}

cCrossingDetect::~cCrossingDetect()
{
}

tResult cCrossingDetect::GetInterface(const tChar* idInterface,
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

tUInt cCrossingDetect::Ref()
{
    return cFilter::Ref();
}

tUInt cCrossingDetect::Unref()
{
    return cFilter::Unref();
}

tVoid cCrossingDetect::Destroy()
{
    delete this;
}

tResult cCrossingDetect::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cCrossingDetect::Stop(__exception)
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
tResult cCrossingDetect::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// Get description for bool values
    	tChar const * strDescBoolIDValue = pDescManager->GetMediaDescription("tStartCrossSearchingStruct");	
    	RETURN_IF_POINTER_NULL(strDescBoolIDValue);	
    	cObjectPtr<IMediaType> pTypeBoolIDValue = new cMediaType(0, 0, 0, "tStartCrossSearchingStruct", strDescBoolIDValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for SignalValue
    	tChar const * strDescCrossingStruct = pDescManager->GetMediaDescription("tCrossingStruct");   
    	RETURN_IF_POINTER_NULL(strDescCrossingStruct);    
    	cObjectPtr<IMediaType> pTypeCrossingStruct = new cMediaType(0, 0, 0, "tCrossingStruct", strDescCrossingStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("Start", pTypeBoolIDValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeBoolIDValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartBoolIDInput)); 


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

		// Create Pin for DistanceToCrossingSpot (Output)
		RETURN_IF_FAILED(m_oDistanceAndTypeOutput.Create("DistanceToCrossingSpot", pTypeCrossingStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceAndTypeOutput));
		RETURN_IF_FAILED(pTypeCrossingStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceAndCrossType)); 


              
    }
    else if (eStage == StageNormal)
    {
		RGBreceived 	   = false;
		m_iImageHalfWidth  = 0;
		m_bStartBoolIDInput = tFalse;
		m_bActive = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
        m_ui8Imagecount = 0;
		m_DeadEndFrontBool = tFalse;
		m_DeadEndLeftBool = tFalse;
		m_DeadEndRightBool = tFalse;
		m_CrossType = UNDEFINED_CROSSING;
		m_crossingSpotOutputSet = tFalse;
		//m_LefthorizontalHough = tFalse;//Brauchen wir damit er, wenn er eine horizontale houghline gefunden hat, sich im nachhinein nicht um entscheidet weil sie verschwindet
		//m_RighthorizontalHough = tFalse;
		m_SMALL = tFalse;

		//m_distHoughToCorner = tFalse;

        ReadProperties(NULL);

        //m_ui8InitCtrl = 0;

		m_iframeCounter = 0;
        
        
        
        if (m_bShowDebug)
        {
        }

    }
    RETURN_NOERROR;
}

tResult cCrossingDetect::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cCrossingDetect::ReadProperties(const tChar* strPropertyName)
{
    
if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisparamK))
    {
		float f;
		f = GetPropertyFloat(CrossingDetect_PROP_CornerHarrisparamK);
		m_nCornerHarrisparamK = static_cast<double>(f);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisblockSize))
    {
        m_nCornerHarrisblockSize = GetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_CornerHarrisksize))
    {
        m_nCornerHarrisksize = GetPropertyInt(CrossingDetect_PROP_CornerHarrisksize);
    }


    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLD))
    {
        m_nThresholdValue = GetPropertyInt(CrossingDetect_PROP_TRESHOLD);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLD2))
    {
        m_nThresholdValue2 = GetPropertyInt(CrossingDetect_PROP_TRESHOLD2);
    }
	
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(CrossingDetect_PROP_SHOW_DEBUG);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutWidthLeft))
    {
        m_nImagecutWidthLeft = GetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeft);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutWidthRight))
    {
        m_nImagecutWidthRight = GetPropertyInt(CrossingDetect_PROP_ImagecutWidthRight);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutHeightUp))
    {
        m_nImagecutHeightUp = GetPropertyInt(CrossingDetect_PROP_ImagecutHeightUp);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutHeightDown))
    {
        m_nImagecutHeightDown = GetPropertyInt(CrossingDetect_PROP_ImagecutHeightDown);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutWidthLeftHough))
    {
        m_nImagecutWidthLeftHough = GetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeftHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutWidthRightHough))
    {
        m_nImagecutWidthRightHough = GetPropertyInt(CrossingDetect_PROP_ImagecutWidthRightHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutHeightUpHough))
    {
        m_nImagecutHeightUpHough = GetPropertyInt(CrossingDetect_PROP_ImagecutHeightUpHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ImagecutHeightDownHough))
    {
        m_nImagecutHeightDownHough = GetPropertyInt(CrossingDetect_PROP_ImagecutHeightDownHough);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueHough))
    {
        m_nThresholdValueHough = GetPropertyInt(CrossingDetect_PROP_ThresholdValueHough);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(CrossingDetect_PROP_ThresholdValueCanny);
    }


    RETURN_NOERROR;
}

tResult cCrossingDetect::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cCrossingDetect::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cCrossingDetect::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pStartBoolIDInput,pMediaSample,pCoder);

				if (!m_bStartBoolIDInput) {
					pCoder->GetID("bStart", m_szIDValueStartBoolIDInput);
					pCoder->GetID("ID", m_szIDDRIVING);
					m_bStartBoolIDInput = tTrue;
				}

                pCoder->Get(m_szIDValueStartBoolIDInput, (tVoid*)&bValue);	
				pCoder->Get(m_szIDDRIVING, (tVoid*)&m_DRIVING_ID);
            }

			//LOG_INFO(cString::Format("Status: %i, ID: %i", bValue, m_DRIVING_ID) );

			m_bActive = bValue;
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cCrossingDetect::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{ if (m_bActive){
/*    if(m_ui8InitCtrl < 10)         // This loop is necessary to boot the car's controller (needs neutral signal for a certain time)
    {
        m_ui8InitCtrl++;
    }
    else 
    {*/
			m_bActive = tFalse; // damit nur ein RGB und Tiefenbild empfangen wird
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

			// Reset Values!
			m_representerHRG1 = Point2f (0,0);
			m_representerHRG2 = Point2f (0,0);
			m_representerHLG1 = Point2f (0,0);
			m_representerHRGS = Point2f (0,0);
			m_representerHRGSLane = Point2f (0,0);
			PixelSpot.clear();


//----------------Preprocessing-------------------------//
///////////////// Corner detection 
		
		    Mat m_matImage = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image
			// TODO compress
			cvtColor(m_matImage, m_matImage, CV_RGB2GRAY);// Grey Image
			medianBlur(m_matImage, m_matImage, 3); // reduce noise with edge-preserving filter
			cornerHarris(m_matImage, m_matCorner, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			threshold(m_matCorner, m_matThres, m_nThresholdValue, 255,THRESH_TOZERO);
			normalize(m_matThres, m_matNormalized, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
			convertScaleAbs(m_matNormalized, m_matNormalized); // Konveriere in 8Bit array. Nehme aus abs
			threshold(m_matNormalized, m_matThres2, m_nThresholdValue2, 255,THRESH_BINARY); 

//////////////// Line detection Left
		
		if(m_nImagecutWidthRightHough > 320)
			{
			LOG_ERROR(cString::Format("CrossDetection: Left Window of Hough detect is to bride") );
			RETURN_NOERROR; 
			}
		m_iImageHalfWidth = image.size().width*0.5;
		Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_nImagecutWidthLeftHough, m_nImagecutWidthRightHough)).clone(); //Cut Image 
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 		
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO); // delete dark noise	
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCannyLeft,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges 
		m_linesL.clear();
		HoughLines(m_matCannyLeft,m_linesL,1,CV_PI/180,m_nThresholdValueHough,0,0);
		/*fstream f3;
		f3.open("HoughL.dat",ios::out|ios::app);
	for(size_t i=0; i<m_linesL.size();i++)
		{
					f3 << m_linesL[i][0] << "," << m_linesL[i][1]*180/M_PI << "\n";
		}
		f3.close();*/
/////////////// Line detect right
		m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_iImageHalfWidth+m_nImagecutWidthLeftHough, m_iImageHalfWidth+m_nImagecutWidthRightHough)).clone(); //Cut Image    
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO);
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCannyRight,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges
		m_linesR.clear();
		HoughLines(m_matCannyRight,m_linesR,1,CV_PI/180,m_nThresholdValueHough,0,0);
		/*fstream f2;
		f2.open("HoughR.dat",ios::out|ios::app);
	for(size_t i=0; i<m_linesR.size();i++)
		{
					f2 << m_linesR[i][0] << "," << m_linesR[i][1]*180/M_PI << "\n";
		}
		f2.close();*/

/////////////////////// Line detect dead end Front
		m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(0, 640)).clone(); //Cut Image    
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO);
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCannyRight,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges ---------Nur Canny "funktioniert"-------		
		m_lines.clear();
		HoughLines(m_matCannyRight,m_lines,1,CV_PI/180,300,0,0);
		for(size_t i=0;i<m_lines.size();i++)
			{
				if(m_lines[i][1]*180/M_PI<100 && m_lines[i][1]*180/M_PI>80)
					{
						m_DeadEndFrontBool = tTrue;
						break;
					}
			}

/////////////////////// Line detect front right(small lines in front of car)
		m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_iImageHalfWidth+m_nImagecutWidthLeftHough, m_iImageHalfWidth+m_nImagecutWidthRightHough)).clone(); //Cut Image    
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO);
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCannyRight,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges
		m_linesRSmall.clear();
		HoughLines(m_matCannyRight,m_linesRSmall,1,CV_PI/180,20,0,0);
			
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
	}
 //}
	RETURN_NOERROR;            
}

tResult cCrossingDetect::ProcessInputDepth(IMediaSample* pSample, tTimeStamp tsInputTime)
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
	
	//Util::CalculateCameraAngle(m_depthImage.at<ushort>(108,160));

	// Transform image
	m_matImageDepth= m_depthImage(cv::Range(m_nImagecutHeightUp/2, m_nImagecutHeightDown/2), cv::Range(m_nImagecutWidthLeft/2, m_nImagecutWidthRight/2)).clone(); //Cut Image

	SearchCorners();
	CreateAndTransmitGCL();
	RGBreceived=false;	
	
	RETURN_NOERROR;            
}



tResult cCrossingDetect::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cCrossingDetect::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cCrossingDetect::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cCrossingDetect::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cCrossingDetect::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

tResult cCrossingDetect::SearchCorners()
{

//	pui8PointsCount    = 0;
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
//LOG_INFO(cString::Format("Number_points: %i", Points.size()) );
vector<Point3f> CrossPoint;
vector<Point3f> CrossPointHough;
	Point3f corner1;
	Point3f corner2;

	//elimininate pointcloud 
		vector<int> killUs;
		for(size_t i=0;i<Points.size();i++)
			{	
			corner1 = Util::ComputeWorldCoordinate(Points[i].x, Points[i].y,m_matImageDepth.at<ushort>(floor(Points[i].y*0.5),floor(Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			for(size_t j=i+1;j<Points.size();j++)
				{
					corner2 = Util::ComputeWorldCoordinate(Points[j].x, Points[j].y,m_matImageDepth.at<ushort>(floor(Points[j].y*0.5), floor(Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
					if(norm(corner1-corner2)<0.18)
						{
						if(norm(corner1) > norm(corner2))
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
		// Search actual crossing
		for(size_t i=0;i<Points.size();i++)
		{		
			corner1 = Util::ComputeWorldCoordinate(Points[i].x, Points[i].y,m_matImageDepth.at<ushort>(floor(Points[i].y*0.5),floor(Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
		 	for(size_t j=i+1;j<Points.size();j++)
				{
				corner2 = Util::ComputeWorldCoordinate(Points[j].x, Points[j].y,m_matImageDepth.at<ushort>(floor(Points[j].y*0.5), floor(Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
				if(((norm(corner1-corner2)<1.04 && norm(corner1-corner2)>0.89) && corner1.y>0.19 && corner1.y<0.23 && corner2.y>0.19 && corner2.y<0.23 && (corner1.x>0.0 || corner1.x<-0.45) && (corner2.x>0.0 || corner2.x<-0.45)) && (norm(corner1.z-corner2.z)<0.11 || norm(corner1.x-corner2.x)<0.11)){
					// Test if points are valid
					if(corner1.z < 3 && corner2.z <3 && corner1.y > 0.18 && corner2.y > 0.18){
						PixelSpot.push_back(Point2f(Points[i].y,Points[i].x));
						PixelSpot.push_back(Point2f(Points[j].y,Points[j].x));
						CrossPoint.push_back(corner1);
						CrossPoint.push_back(corner2);
					}
				}
			}
		}
	int idx = -1;
	double MIN1 = 10000;
	for(size_t i=0; i<CrossPoint.size();i++)
		{
			double buffer = norm(CrossPoint[i]);
			if(buffer<MIN1)
				{
					MIN1 = buffer;
					idx = i;
				}
		}
//eliminate Houghline-cloud of HoughL
vector<Point2f> HoughLGroup1;
vector<Point2f> HoughLGroup2;
	for(size_t i=0; i<m_linesL.size();i++)
		{
					if((m_linesL[i][1]*180/M_PI<80) && (m_linesL[i][1]*180/M_PI>40))
					{
							HoughLGroup1.push_back(Point2f(m_linesL[i][0],m_linesL[i][1]));		
					}
					else if((m_linesL[i][1]*180/M_PI<100) && (m_linesL[i][1]*180/M_PI>80))
					{
							HoughLGroup2.push_back(Point2f(m_linesL[i][0],m_linesL[i][1])); 	
					}
		}

if(HoughLGroup2.empty() && !m_DeadEndFrontBool) m_DeadEndLeftBool = tTrue; // && !m_LefthorizontalHough 
else
{
 m_DeadEndLeftBool = tFalse;
 //m_LefthorizontalHough = tTrue;//haben wir initial auf false gesetzt und wird nach einmaligem fund einer horizontalen houghline auf true gesetzt, damit er bei der entscheidung bleibt.
}
//Calculate representer for HoughLGroup1
if(!HoughLGroup1.empty())
{
float averageHLG1 = 0;
m_representerHLG1 = HoughLGroup1[0]; 
	for(size_t i=0; i<HoughLGroup1.size();i++)
		{
			averageHLG1 = averageHLG1 + HoughLGroup1[i].y*180/M_PI;
		}
			averageHLG1 = averageHLG1/HoughLGroup1.size();
	for(size_t i=0; i<HoughLGroup1.size();i++)
		{
			if(abs(m_representerHLG1.y*180/M_PI-averageHLG1)>abs(HoughLGroup1[i].y*180/M_PI-averageHLG1))
			{
				m_representerHLG1 = HoughLGroup1[i];
			}
		}
}

//eliminate Houghline-cloud of HoughR
vector<Point2f> HoughRGroup1;
vector<Point2f> HoughRGroup2;
	for(size_t i=0; i<m_linesR.size();i++)
		{
					if((m_linesR[i][1]*180/M_PI>115) && (m_linesR[i][1]*180/M_PI<150))
					{
							HoughRGroup1.push_back(Point2f(m_linesR[i][0],m_linesR[i][1]));		
					}
					else if((m_linesR[i][1]*180/M_PI<100) && (m_linesR[i][1]*180/M_PI>80))
					{
							HoughRGroup2.push_back(Point2f(m_linesR[i][0],m_linesR[i][1]));	
					}
		}
//LOG_INFO(cString::Format("Abbiegecheck: %i | %i | %i | %i", HoughRGroup2.empty(), !m_RighthorizontalHough, !m_DeadEndFrontBool, ACTION_RIGHT!=m_DRIVING_ID) );
if(HoughRGroup2.empty() && !m_DeadEndFrontBool && ACTION_RIGHT!=m_DRIVING_ID) m_DeadEndRightBool = tTrue; //&& !m_RighthorizontalHough
else
{
 m_DeadEndRightBool = tFalse;
 //m_RighthorizontalHough = tTrue;//haben wir initial auf false gesetzt und wird nach einmaligem fund einer horizontalen houghline auf true gesetzt, damit er bei der entscheidung bleibt.
}
//Calculate representer for HoughRGroup1
if(HoughRGroup1.size()!=0)
{
float averageHRG1 = 0;
m_representerHRG1 = HoughRGroup1[0]; 
	for(size_t i=0; i<HoughRGroup1.size();i++)
		{
			averageHRG1 = averageHRG1 + HoughRGroup1[i].y*180/M_PI;
		}
			averageHRG1 = averageHRG1/HoughRGroup1.size();

	for(size_t i=0; i<HoughRGroup1.size();i++)
		{
			if(abs(m_representerHRG1.y*180/M_PI-averageHRG1)>abs(HoughRGroup1[i].y*180/M_PI-averageHRG1))
			{
				m_representerHRG1 = HoughRGroup1[i];
			}
		}
}

//Calculate representer for HoughRGroup2
if(HoughRGroup2.size()!=0)
{
	vector<Point2f> averageGroup;
	averageGroup.push_back(HoughRGroup2[0]);
	tFloat32 averageRadius 	= HoughRGroup2[0].x;
	for(size_t i=1; i<HoughRGroup2.size();i++)
	{
		tFloat32 diff = averageRadius - HoughRGroup2[i].x;

		if (diff > 10){ // Line is far away
				continue;
		}		
		else if (diff < -10) {			// Line is near: reset averageGroup
				averageGroup.clear();
				averageGroup.push_back(HoughRGroup2[i]);
				averageRadius = HoughRGroup2[i].x;
		}
		else if (diff >= -10 && diff <= 10){ // append to averageGroup
				averageGroup.push_back(HoughRGroup2[i]);	
				averageRadius = 0.0;
				for (size_t j = 0; j < averageGroup.size(); j++){
					averageRadius += averageGroup[j].x;
				}
				averageRadius /= averageGroup.size();
		}
	}
	m_representerHRG2 = averageGroup[0];
	for(size_t i=1; i<averageGroup.size(); i++)
	{
		if(averageGroup[i].x<m_representerHRG2.x)
		{
			m_representerHRG2 = averageGroup[i];
		}
	}	
}
m_linesRSmallLane.clear();
//Calculate representer for HoughRGroupSmall
if(!m_linesRSmall.empty())
{
		for(size_t i=0;i<m_linesRSmall.size();i++)
			{
				if(m_linesRSmall[i].y*180/M_PI>100 || m_linesRSmall[i].y*180/M_PI<80)
					{
						if(m_linesRSmall[i].y*180/M_PI>115 && m_linesRSmall[i].y*180/M_PI<150)
							{
								m_linesRSmallLane.push_back(m_linesRSmall[i]);
							}
						m_linesRSmall.erase(m_linesRSmall.begin() + i);
					}
			}
	vector<Point2f> averageGroup;
	averageGroup.push_back(m_linesRSmall[0]);
	tFloat32 averageRadius 	= m_linesRSmall[0].x;
	tFloat32 averageAngle 	= m_linesRSmall[0].y;
	for(size_t i=1; i<m_linesRSmall.size();i++)
	{
		tFloat32 diff = averageRadius - m_linesRSmall[i].x;

		if (diff > 10){ // Line is far away
				continue;
		}		
		else if (diff < -10) {			// Line is near: reset averageGroup
				averageGroup.clear();
				averageGroup.push_back(m_linesRSmall[i]);
				averageRadius = m_linesRSmall[i].x;
				averageAngle = m_linesRSmall[i].y;
		}
		else if (diff >= -10 && diff <= 10){ // append to averageGroup
				averageGroup.push_back(m_linesRSmall[i]);	
				averageRadius = 0.0;
				averageAngle = 0.0;
				for (size_t j = 0; j < averageGroup.size(); j++){
					averageRadius += averageGroup[j].x;
					averageAngle += averageGroup[j].y;
				}
				averageRadius /= averageGroup.size();
				averageAngle /= averageGroup.size();
		}
	}
	m_representerHRGS = averageGroup[0];
/*	for(size_t i=1; i<averageGroup.size(); i++)
	{
		if(averageGroup[i].x<m_representerHRGS.x)
		{
			m_representerHRGS = averageGroup[i];
		}
	}*/	
	for(size_t i=1; i<averageGroup.size(); i++)
	{
		if(abs(averageGroup[i].y-averageAngle)<abs(averageAngle-m_representerHRGS.y))
		{
			m_representerHRGS = averageGroup[i];
		}
	}
//Calculate representer for HoughRGroup1SmallLane
if(m_linesRSmallLane.size()!=0)
{
float averageHRG1SmallLane = 0;
m_representerHRGSLane = m_linesRSmallLane[0]; 
	for(size_t i=0; i<m_linesRSmallLane.size();i++)
		{
			averageHRG1SmallLane = averageHRG1SmallLane + m_linesRSmallLane[i].y;
		}
			averageHRG1SmallLane = averageHRG1SmallLane/m_linesRSmallLane.size();

	for(size_t i=0; i<m_linesRSmallLane.size();i++)
		{
			if(abs(m_representerHRGSLane.y-averageHRG1SmallLane)>abs(m_linesRSmallLane[i].y-averageHRG1SmallLane))
			{
				m_representerHRGSLane = m_linesRSmallLane[i];
			}
		}
}

}
//calculate intersection of representer horizontal hougline and representer on right side(crossing corner right front)
		if(!m_linesRSmall.empty() && !m_linesRSmallLane.empty())
		{
		/*fstream f2;
		f2.open("Houghsmallsize.dat",ios::out|ios::app);
		f2 << m_linesRSmall.size() << "," << m_linesRSmallLane.size() << "\n";
		f2.close();*/
		}
m_SMALL = tFalse;
if(!m_DeadEndRightBool)
	{
	if(!m_linesRSmall.empty())
	{
		/*fstream f7;
		f7.open("TESTOR.dat",ios::out|ios::app);
		f7 <<m_representerHRGSLane.x << "," << m_representerHRGSLane.y << "," << m_representerHRGS.x << "," << m_representerHRGS.y << "\n";
		f7.close();*/
		Point2f pt1r,pt2r,pt1rS,pt2rS;
		pt1r.x = cos(m_representerHRGSLane.y)*m_representerHRGSLane.x + 1000*(-sin(m_representerHRGSLane.y));
		pt1r.y = sin(m_representerHRGSLane.y)*m_representerHRGSLane.x + 1000*(cos(m_representerHRGSLane.y));
		pt2r.x = cos(m_representerHRGSLane.y)*m_representerHRGSLane.x - 1000*(-sin(m_representerHRGSLane.y));
		pt2r.y = sin(m_representerHRGSLane.y)*m_representerHRGSLane.x - 1000*(cos(m_representerHRGSLane.y));

		pt1rS.x = cos(m_representerHRGS.y)*m_representerHRGS.x + 1000*(-sin(m_representerHRGS.y));
		pt1rS.y = sin(m_representerHRGS.y)*m_representerHRGS.x + 1000*(cos(m_representerHRGS.y));
		pt2rS.x = cos(m_representerHRGS.y)*m_representerHRGS.x - 1000*(-sin(m_representerHRGS.y));
		pt2rS.y = sin(m_representerHRGS.y)*m_representerHRGS.x - 1000*(cos(m_representerHRGS.y));
		if(pt1rS.x == pt2rS.x || pt1rS.y == pt2rS.y)
		{
		}
		else
		{
		float m1 = (pt2rS.y-pt1rS.y)/(pt2rS.x-pt1rS.x);
		float m2 = (pt2r.y-pt1r.y)/(pt2r.x-pt1r.x);
		double b1 = pt2rS.y - m1*pt2rS.x;
		double b2 = pt2r.y - m2*pt2r.x;
		m_intersecpointRS.x = cvRound((b2-b1)/(m1-m2));
		m_intersecpointRS.y = cvRound(m1*m_intersecpointRS.x+b1);;
		if(m_intersecpointRS.x>0 && m_intersecpointRS.y>0) m_crossFrontRightS = Util::ComputeWorldCoordinate(m_intersecpointRS.x,m_intersecpointRS.y,m_matImageDepth.at<ushort>(floor(m_intersecpointRS.y*0.5),floor(m_intersecpointRS.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUpHough,m_nImagecutWidthLeftHough+m_iImageHalfWidth);
		for(size_t i=0; i<CrossPoint.size();i++)
			{
LOG_INFO(cString::Format("CD: norm(CrossPoint[i]-m_crossFrontRightS = %f", norm(CrossPoint[i]-m_crossFrontRightS) ) );
				if(norm(CrossPoint[i]-m_crossFrontRightS)<0.05)
				{
					CrossPointHough.push_back(m_crossFrontRightS);
					m_SMALL = tTrue;	
				}					
			}
		}
	}
//LOG_INFO(cString::Format("CD: m_SMALL = %i", m_SMALL) );
	if(HoughRGroup2.size()!=0 && !m_SMALL)
		{
				Point2f pt1,pt2,pt1r,pt2r;
				pt1r.x = cos(m_representerHRG1.y)*m_representerHRG1.x + 1000*(-sin(m_representerHRG1.y));
				pt1r.y = sin(m_representerHRG1.y)*m_representerHRG1.x + 1000*(cos(m_representerHRG1.y));
				pt2r.x = cos(m_representerHRG1.y)*m_representerHRG1.x - 1000*(-sin(m_representerHRG1.y));
				pt2r.y = sin(m_representerHRG1.y)*m_representerHRG1.x - 1000*(cos(m_representerHRG1.y));

				pt1.x = cos(m_representerHRG2.y)*m_representerHRG2.x + 1000*(-sin(m_representerHRG2.y));
				pt1.y = sin(m_representerHRG2.y)*m_representerHRG2.x + 1000*(cos(m_representerHRG2.y));
				pt2.x = cos(m_representerHRG2.y)*m_representerHRG2.x - 1000*(-sin(m_representerHRG2.y));
				pt2.y = sin(m_representerHRG2.y)*m_representerHRG2.x - 1000*(cos(m_representerHRG2.y));
			
				if(pt1.x == pt2.x || pt1.y == pt2.y || pt1r.x == pt2r.x || pt1r.y == pt2r.y)
				{
				}
				else
				{
				float m1 = (pt2.y-pt1.y)/(pt2.x-pt1.x);
				float m2 = (pt2r.y-pt1r.y)/(pt2r.x-pt1r.x);
				double b1 = pt2.y - m1*pt2.x;
				double b2 = pt2r.y - m2*pt2r.x;
				m_intersecpointR.x = cvRound((b2-b1)/(m1-m2));
				m_intersecpointR.y = cvRound(m1*m_intersecpointR.x+b1);
				m_crossFrontRight = Util::ComputeWorldCoordinate(m_intersecpointR.x,m_intersecpointR.y,m_matImageDepth.at<ushort>(floor(m_intersecpointR.y*0.5),floor(m_intersecpointR.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUpHough,m_nImagecutWidthLeftHough+m_iImageHalfWidth);
				if(m_crossFrontRight.x!=0 && CrossPointHough.empty()){	
				 CrossPointHough.push_back(m_crossFrontRight);
					LOG_INFO(cString::Format("CD: Used normal line") );
				}					
					
				}
		}
	}
	else
	{
		//Calculate crosspoint if deadendright=true on left side with cornerdetection
		CrossPoint.clear();
		 for(size_t j=0;j<Points.size();j++)
			{
				Point3f corner = Util::ComputeWorldCoordinate(Points[j].x, Points[j].y,m_matImageDepth.at<ushort>(floor(Points[j].y*0.5), floor(Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
				if(corner.x < -0.45 && corner.y > 0.18 && corner.z < 2.0){
						CrossPoint.push_back(corner);
				}
			}
		//finde nun erneut wie am anfang die ecke, die am nahesten am auto ist.idx muss nicht neu deklariert werden, da er schon zu Beginn deklariert wird und wir ihn weiterverwenden müssen, da am ende CrossPoint[idx] übergeben wird.
		
		// Get farest corner of the nearest corner group
		idx 		= -1;
		if(!CrossPoint.empty()){
			m_DeadEndLeftBool 	= tFalse; // it is not DeadEndLeft if it is deadEndRight and we found a corner
			vector<float> nearGroup;
			float average = 0.0;
			for(size_t i=0; i<CrossPoint.size();i++)
			{
				if (nearGroup.empty() || fabs(average-CrossPoint[i].z)<0.07)
				{
					nearGroup.push_back(CrossPoint[i].z);
					average = 0.0;
					for(size_t j = 0; j < nearGroup.size(); j++){
						average+=nearGroup.at(j);
					}
					average/=nearGroup.size();
				}
				else if (CrossPoint[i].z-average<-0.07){
					nearGroup.clear();
					nearGroup.push_back(CrossPoint[i].z);
				}
			}
			
			double buffer2 	= 0.0;
			for (size_t j = 0; j < nearGroup.size(); j++){
				if (nearGroup.at(j) > buffer2){
					CrossPoint.push_back(Point3f (0,0,nearGroup.at(j)));
					buffer2 = nearGroup.at(j);
				}
			}
			
			idx = CrossPoint.size()-1;

//			LOG_INFO(cString::Format("CD: Crosspoint.z = %f", buffer2) );
		}
		
/*
		if(HoughLGroup2.size()!=0)
		{
		int idxH = 0;
		float max = 0;
			for(size_t i=0; i<HoughLGroup2.size();i++)
				{
					if(HoughLGroup2[i].x>max)
						{
							idxH = i;
							max = HoughLGroup2[i].x;
						}
				}
				Point2f pt1,pt2,pt1r,pt2r;
				pt1.x = cos(HoughLGroup2[idxH].y)*HoughLGroup2[idxH].x + 1000*(-sin(HoughLGroup2[idxH].y));
				pt1.y = sin(HoughLGroup2[idxH].y)*HoughLGroup2[idxH].x + 1000*(cos(HoughLGroup2[idxH].y));
				pt2.x = cos(HoughLGroup2[idxH].y)*HoughLGroup2[idxH].x - 1000*(-sin(HoughLGroup2[idxH].y));
				pt2.y = sin(HoughLGroup2[idxH].y)*HoughLGroup2[idxH].x - 1000*(cos(HoughLGroup2[idxH].y)); 

				pt1r.x = cos(m_representerHLG1.y)*m_representerHLG1.x + 1000*(-sin(m_representerHLG1.y));
				pt1r.y = sin(m_representerHLG1.y)*m_representerHLG1.x + 1000*(cos(m_representerHLG1.y));
				pt2r.x = cos(m_representerHLG1.y)*m_representerHLG1.x - 1000*(-sin(m_representerHLG1.y));
				pt2r.y = sin(m_representerHLG1.y)*m_representerHLG1.x - 1000*(cos(m_representerHLG1.y));

				if(pt1.x == pt2.x || pt1.y == pt2.y || pt1r.x == pt2r.x || pt1r.y == pt2r.y)
				{
				}
				else
				{
				float m1 = (pt2.y-pt1.y)/(pt2.x-pt1.x);
				float m2 = (pt2r.y-pt1r.y)/(pt2r.x-pt1r.x);
				double b1 = pt2.y - m1*pt2.x;
				double b2 = pt2r.y - m2*pt2r.x;
				m_intersecpointL.x = cvRound((b2-b1)/(m1-m2));
				m_intersecpointL.y = cvRound(m1*m_intersecpointL.x+b1);
				m_crossFrontLeft = Util::ComputeWorldCoordinate(m_intersecpointL.x,m_intersecpointL.y,m_matImageDepth.at<ushort>(floor(m_intersecpointL.y*0.5),floor(m_intersecpointL.x*0.5)),m_nImagecutHeightUpHough,m_nImagecutWidthLeftHough);
				if(m_crossFrontLeft.z>0) CrossPointHough.push_back(m_crossFrontLeft);
				}
		}
*/	
	}

//calculate if crosspoint is on houghline
//calculate first interceptionpoint of normale from crosspoint to houghline
//m_distHoughToCorner = tFalse;
/*if(!CrossPoint.empty() && !CrossPointHough.empty())
{
	Point2f inter;
	Point3f inter3d;
	float m1 = (m_pt2r.y-m_pt1r.y)/(m_pt2r.x-m_pt1r.x);
	double b1 = m_pt2r.y - m1*m_pt2r.x;
	inter.x = (PixelSpot[idx].y - PixelSpot[idx].x/m1-b1)/(m1+1/m1);
	inter.y = m1*inter.x+b1;
	inter3d = Util::ComputeWorldCoordinate(inter.x, inter.y,m_matImageDepth.at<ushort>(floor(inter.y*0.5), floor(inter.x*0.5)),m_nImagecutHeightUpHough,m_iImageHalfWidth+m_nImagecutWidthLeftHough);
	Point3f hough3d = Util::ComputeWorldCoordinate(PixelSpot[idx].x, PixelSpot[idx].y,m_matImageDepth.at<ushort>(floor(PixelSpot[idx].y*0.5), floor(PixelSpot[idx].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
	if(norm(inter3d-hough3d)<0.03) m_distHoughToCorner = tTrue;
	else m_distHoughToCorner = tFalse;
}*/
		/*if(!CrossPointHough.empty())
		{
		fstream f4;
		f4.open("HoughGroup.dat",ios::out|ios::app);
		f4 << CrossPointHough[0].z << "," << m_SMALL << "\n" ;
		f4.close();
		}*/
/*
		fstream f2;
		f2.open("Houghinter.dat",ios::out|ios::app);
		f2 << m_DeadEndLeftBool << "," << m_DeadEndRightBool << "," << m_DeadEndFrontBool << "\n" ;
		f2.close();
*/
//LOG_WARNING(cString::Format("CrossDetection: m_DeadEndLeftBool: %i,m_DeadEndRightBool: %i,m_DeadEndFrontBool: %i",m_DeadEndLeftBool,m_DeadEndRightBool,m_DeadEndFrontBool));
	if(m_iframeCounter>MAX_FRAMES)
	{
			m_CrossType =	UNDEFINED_CROSSING;
			//LOG_ERROR(cString::Format("CrossDetection: No Crossing found! UNDEFINED_CROSSING") );
			TransmitCrossingSpot(-1.0);
			m_iframeCounter = 0;
			RETURN_NOERROR;
	}
	else if((CrossPointHough.empty() && !m_DeadEndRightBool) || (CrossPoint.empty() && m_DeadEndRightBool) )
	{	
			//LOG_WARNING(cString::Format("CrossDetection: No Crossing found!"));
			// Try again if failed
			m_bActive = tTrue;
	}
	else if( (!m_DeadEndRightBool && (CrossPointHough[0].z > 2.0 || CrossPointHough[0].z < 0.4) ) || (m_DeadEndRightBool && idx > -1 && CrossPoint[idx].z > 2) )
	{
		//LOG_WARNING(cString::Format("CrossDetection: No valid Crossing lot found!"));
		// Try again if failed
		m_bActive = tTrue;
 	}
/*	else if(CrossPointHough[0].z>1.2)
	{
		//LOG_INFO(cString::Format("IP:Found but too far away....Try again") );
		m_bActive = tTrue;
	}*/
	else if((m_DeadEndFrontBool && m_DeadEndLeftBool) || (m_DeadEndRightBool && m_DeadEndFrontBool) || (m_DeadEndLeftBool && m_DeadEndRightBool))
	{
		//LOG_INFO(cString::Format("IP:Invalid Crosstype") );
		m_bActive = tTrue;
	}
	else
	{
		// Schreibe gefundene Kreuzung in Outputpin	
 		/*if (!m_oDistanceAndTypeOutput.IsConnected())
   		{
			LOG_WARNING(cString::Format("CrossDetection: OutputPin for crossing not connected!"));
    	    RETURN_NOERROR;
    	}*/
			if(m_DeadEndFrontBool)
			{
				m_CrossType	=	DEADENDFRONT;
				if(ACTION_STRAIGHT==m_DRIVING_ID) m_CrossType=CROSSROAD;
				
				//LOG_INFO(cString::Format("CrossDetection: Found DEADENDFRONT") );
			}
			else if(m_DeadEndRightBool)
			{
				if(ACTION_RIGHT==m_DRIVING_ID)
				{	
					m_CrossType=CROSSROAD;
					//LOG_INFO(cString::Format("CrossDetection: Found CROSSROAD") );
				}
				else

				{
					m_CrossType	=	DEADENDRIGHT;
					//LOG_INFO(cString::Format("CrossDetection: Found DEADENDRIGHT") );
				}
			}
			else if(m_DeadEndLeftBool)
			{
				if(ACTION_LEFT==m_DRIVING_ID)
				{
					m_CrossType=CROSSROAD;
					//LOG_INFO(cString::Format("CrossDetection: Found CROSSROADS") );
				}
				else
				{
					m_CrossType	=	DEADENDLEFT;
					//LOG_INFO(cString::Format("CrossDetection: Found DEADENDLEFT") );
				}
			}
			else if(!m_DeadEndFrontBool && !m_DeadEndLeftBool && !m_DeadEndRightBool)//Kreuzung!!
			{
				m_CrossType	=	CROSSROAD;
				//LOG_INFO(cString::Format("CrossDetection: Found CROSSROAD") );
			}
/*
if(CrossPoint.size()==2)//T-Kreuzung!!
		{
			if(CrossPoint[0].x*CrossPoint[1].x<0)
			{
				m_CrossType	=	DEADENDFRONT;
				if(ACTION_STRAIGHT==m_DRIVING_ID) m_CrossType=CROSSROAD;
				
				LOG_INFO(cString::Format("CrossDetection: Found DEADENDFRONT") );
			}
			else if(CrossPoint[0].x<0 && CrossPoint[1].x<0)
			{
				if(ACTION_RIGHT==m_DRIVING_ID)
				{	
					m_CrossType=CROSSROAD;
					LOG_INFO(cString::Format("CrossDetection: Found CROSSROAD") );
				}
				else

				{
					m_CrossType	=	DEADENDRIGHT;
					LOG_INFO(cString::Format("CrossDetection: Found DEADENDRIGHT") );
				}
			}
			else if(CrossPoint[0].x>0 && CrossPoint[1].x>0)
			{
				if(ACTION_LEFT==m_DRIVING_ID)
				{
					m_CrossType=CROSSROAD;
					LOG_INFO(cString::Format("CrossDetection: Found CROSSROADS") );
				}
				else
				{
					m_CrossType	=	DEADENDLEFT;
					LOG_INFO(cString::Format("CrossDetection: Found DEADENDLEFT") );
				}
			}
		}
		else if(CrossPoint.size()>2)//Kreuzung!!
		{
			m_CrossType	=	CROSSROAD;
			LOG_INFO(cString::Format("CrossDetection: Found CROSSROAD") );
		}
*/
		if(m_DeadEndRightBool) 
		{
			TransmitCrossingSpot(CrossPoint[idx].z); //Falls rechts "zu" müssen wir eine ecke von links nehmen.
			LOG_INFO(cString::Format("CD: Send CrossPoint") );
  		}else 
		{
			TransmitCrossingSpot(CrossPointHough[0].z);
			LOG_INFO(cString::Format("CD: Send CrossPointHough") );
		
		}
		/*if(m_distHoughToCorner) TransmitCrossingSpot(CrossPoint[idx].z);//Falls Ecke und Schnittpunikt der Houghs nah genug beieinander, nehmen wir lieber die ecke, da sie genauer ist.Habe dazu ein Bollean gesetzt.
		else TransmitCrossingSpot(CrossPointHough[0].z);*/
		m_iframeCounter = 0;
		//m_LefthorizontalHough = tFalse;//Brauchen wir damit er, wenn er eine horizontale houghline gefunden hat, sich im nachhinein nicht um entscheidet weil sie verschwindet
		//m_RighthorizontalHough = tFalse;
	}
//LOG_WARNING(cString::Format("CrossDetection: CrossType: %i!",m_CrossType));
	//LOG_INFO(cString::Format("IP: Finished Searching") );
	RETURN_NOERROR;
}


tResult cCrossingDetect::CreateAndTransmitGCL()
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

//LOG_INFO(cString::Format("GCL: Drawing!") );

    // draw rectangle to scale the video display correctly
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 0).GetRGBA());	
    cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, 0, 0, m_sInputFormat.nWidth, m_sInputFormat.nHeight);

    // draw near and far area
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 100, 255).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,m_nImagecutWidthLeft , m_nImagecutHeightUp, m_nImagecutWidthRight, m_nImagecutHeightDown);
	//if(m_intersecpointLB.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointLB.x)+m_nImagecutWidthLeftHough-4, cvRound(m_intersecpointLB.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointLB.x)+m_nImagecutWidthLeftHough+4, cvRound(m_intersecpointLB.y)+m_nImagecutHeightUpHough+4);   
	//if(m_intersecpointL.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointL.x)+m_nImagecutWidthLeftHough-4, cvRound(m_intersecpointL.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointL.x)+m_nImagecutWidthLeftHough+4, cvRound(m_intersecpointL.y)+m_nImagecutHeightUpHough+4);   
	if(m_intersecpointR.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointR.x)+m_nImagecutWidthLeftHough-4+m_iImageHalfWidth, cvRound(m_intersecpointR.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointR.x)+m_nImagecutWidthLeftHough+4+m_iImageHalfWidth, cvRound(m_intersecpointR.y)+m_nImagecutHeightUpHough+4);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,0,255).GetRGBA());	
	if(m_intersecpointRS.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointRS.x)+m_nImagecutWidthLeftHough-4+m_iImageHalfWidth, cvRound(m_intersecpointRS.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointRS.x)+m_nImagecutWidthLeftHough+4+m_iImageHalfWidth, cvRound(m_intersecpointRS.y)+m_nImagecutHeightUpHough+4);   	   	
	//if(m_intersecpointRB.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointRB.x)+m_nImagecutWidthLeftHough-4+m_iImageHalfWidth, cvRound(m_intersecpointRB.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointRB.x)+m_nImagecutWidthLeftHough+4+m_iImageHalfWidth, cvRound(m_intersecpointRB.y)+m_nImagecutHeightUpHough+4);   
	
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
	for(size_t i=0; i<m_linesL.size();i++)
		{
			float rho = m_linesL[i][0];
			float theta = m_linesL[i][1];
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
		}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,100).GetRGBA());
	for(size_t i=0; i<m_linesR.size();i++)
		{
			float rho = m_linesR[i][0];
			float theta = m_linesR[i][1];
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_iImageHalfWidth, pt2.y+m_nImagecutHeightUpHough, pt1.x+m_iImageHalfWidth, pt1.y+m_nImagecutHeightUpHough);
		}
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,0,255).GetRGBA());

	for(size_t i=0; i<m_linesRSmall.size();i++)
		{
			float rho = m_linesRSmall[i].x;
			float theta = m_linesRSmall[i].y;
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
		//cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_iImageHalfWidth, pt2.y+m_nImagecutHeightUpHough, pt1.x+m_iImageHalfWidth, pt1.y+m_nImagecutHeightUpHough);
		}
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
	for(size_t i=0; i < Points.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(Points[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(Points[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(Points[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(Points[i].y+2) + m_nImagecutHeightUp);
		}
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
	for(size_t i=0; i < PixelSpot.size(); i++)
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(PixelSpot[i].y-2) + m_nImagecutWidthLeft , static_cast<tInt16>(PixelSpot[i].x-2) + m_nImagecutHeightUp, static_cast<tInt16>(PixelSpot[i].y+2) + m_nImagecutWidthLeft, static_cast<tInt16>(PixelSpot[i].x+2) + m_nImagecutHeightUp);
		}
//	PixelSpot.clear();

	// Plot Crosstype Output
	cString strText = cString::FromInt32(m_CrossType);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	// Plot Detection type
	cString strText2 = cString::FromInt32(m_SMALL);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 40, strText2.GetLength());
	cGCLWriter::StoreData(pc, strText2.GetLength(), strText2.GetPtr()); 

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
	for(size_t i=0;i<m_lines.size();i++)
			{
				if(m_lines[i][1]*180/M_PI<100 && m_lines[i][1]*180/M_PI>80)
					{
						float rho = m_lines[i][0];
						float theta = m_lines[i][1];
						Point pt1,pt2;
						double a = cos(theta), b = sin(theta);
						double x0 = a*rho, y0 = b*rho;
						pt1.x = cvRound(x0 + 1000*(-b));
						pt1.y = cvRound(y0 + 1000*(a));
						pt2.x = cvRound(x0 - 1000*(-b));
						pt2.y = cvRound(y0 - 1000*(a)); 
						cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUpHough, pt1.x, pt1.y+m_nImagecutHeightUpHough);
					}
			}

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

tResult cCrossingDetect::TransmitCrossingSpot(float lot) {

		tUInt32 nTimeStamp = 0;

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistanceAndCrossType->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		/*fstream superf;
		superf.open("superTest", ios::out|ios::app);
		superf<< "\nbefore Sending\n";*/

		{
			__adtf_sample_write_lock_mediadescription(m_pDistanceAndCrossType, pMediaSample, pCoderOutput); 
	
			if(!m_crossingSpotOutputSet){
				pCoderOutput->GetID("distance", m_szCrossDistance);
				pCoderOutput->GetID("crosstype", m_szIDCrossingType);
				pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampCrossOutput);
				m_crossingSpotOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szCrossDistance, (tVoid*)&(lot));
			pCoderOutput->Set(m_szIDCrossingType, (tVoid*)&(m_CrossType));
			pCoderOutput->Set(m_szIDTimestampCrossOutput, (tVoid*)&nTimeStamp);
		
		}
/*
		superf << "IP: Send crossing Lot to DM!!!!";
		superf.close();
*/
		LOG_INFO(cString::Format("IP: Send crossing Lot to DM, distance = %f", m_szCrossDistance) );

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceAndTypeOutput.Transmit(pMediaSample);

	RETURN_NOERROR;
}
