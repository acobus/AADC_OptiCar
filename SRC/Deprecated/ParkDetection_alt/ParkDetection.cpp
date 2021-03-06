/**
*
*ParkDetection
*
* Date 11.02.2016
* 
*/
 
 
#include "stdafx.h"
#include "ParkDetection.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>


ADTF_FILTER_PLUGIN("ParkDetection", OID_ADTF_ParkDetection, cParkDetection)

#define ParkDetection_PROP_CAMERA_OFFSET "ParkDetection::Camera Offset"
#define ParkDetection_PROP_TRESHOLD "ParkDetection::ThresholdValue"
#define ParkDetection_PROP_TRESHOLD2 "ParkDetection::ThresholdValue2"
#define ParkDetection_PROP_CornerHarrisparamK "ParkDetection::CornerHarrisparamK"
#define ParkDetection_PROP_CornerHarrisblockSize "ParkDetection::CornerHarrisblockSize"
#define ParkDetection_PROP_CornerHarrisksize "ParkDetection::CornerHarrisksize"

#define ParkDetection_PROP_ImagecutWidthLeft "ParkDetection::ImagecutWidthLeft"
#define ParkDetection_PROP_ImagecutWidthRight "ParkDetection::ImagecutWidthRight"
#define ParkDetection_PROP_ImagecutHeightUp "ParkDetection::ImagecutHeightUp"
#define ParkDetection_PROP_ImagecutHeightDown "ParkDetection::ImagecutHeightDown"
#define ParkDetection_PROP_ThresholdValueCanny "ParkDetection::ThresholdValueCanny"
#define ParkDetection_PROP_PointcloudTolerance "ParkDetection::PointcloudTolerance"

#define ParkDetection_PROP_SHOW_DEBUG "Common::Show Debug"





#define MAX_DEVIATION 150


//TODO 1
//#define LT_ENABLE_CANNY_WINDOWS


cParkDetection::cParkDetection(const tChar* __info) : cFilter(__info)
{
	//psPoints.x=0;
	pui8PointsCount    = 0;
	RGBreceived 	   = false;

	// parallel parking
	m_lotDim[0][0] = 0.83;
	m_lotDim[0][1] = 0.77;
	// cross parking
	m_lotDim[1][0] = 0.505;// ;0.54;
	m_lotDim[1][1] = 0.435;//;0.41;

	/*// parallel parking free spot pixel
	m_parkLotPixelSize[0][0] = 30;
	m_parkLotPixelSize[0][1] = 21;

	// cross parking free spot pixel
	m_parkLotPixelSize[1][0] = 63;
	m_parkLotPixelSize[1][1] = 36;
  */  
    SetPropertyBool(ParkDetection_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(ParkDetection_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

	SetPropertyInt(ParkDetection_PROP_PointcloudTolerance, 0.1);
	SetPropertyInt(ParkDetection_PROP_PointcloudTolerance NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_PointcloudTolerance NSSUBPROP_DESCRIPTION, "PointcloudTolerance for");

	SetPropertyInt(ParkDetection_PROP_ImagecutWidthLeft, 320);
	SetPropertyInt(ParkDetection_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutWidthRight, 640);
	SetPropertyInt(ParkDetection_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutHeightUp, 220);
	SetPropertyInt(ParkDetection_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutHeightDown, 420);
	SetPropertyInt(ParkDetection_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");



    SetPropertyFloat(ParkDetection_PROP_CAMERA_OFFSET, 15);
    SetPropertyBool(ParkDetection_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");


    SetPropertyFloat(ParkDetection_PROP_TRESHOLD, 1e-4);
    SetPropertyBool(ParkDetection_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The first threshold value.");
	
	SetPropertyFloat(ParkDetection_PROP_TRESHOLD2, 90);
    SetPropertyBool(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The second threshold value.");

	SetPropertyInt(ParkDetection_PROP_CornerHarrisblockSize, 4);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(ParkDetection_PROP_CornerHarrisksize, 3);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(ParkDetection_PROP_CornerHarrisparamK, 0.05);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");

	SetPropertyFloat(ParkDetection_PROP_ThresholdValueCanny, 150);
    SetPropertyBool(ParkDetection_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    m_pISignalRegistry = NULL;
}

cParkDetection::~cParkDetection()
{
}

tResult cParkDetection::GetInterface(const tChar* idInterface,
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

tUInt cParkDetection::Ref()
{
    return cFilter::Ref();
}

tUInt cParkDetection::Unref()
{
    return cFilter::Unref();
}

tVoid cParkDetection::Destroy()
{
    delete this;
}

tResult cParkDetection::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cParkDetection::Stop(__exception)
{

// TODO 1
#ifdef LT_ENABLE_CANNY_WINDOWS
    if(m_bShowDebug)
    {
        //destroyWindow("Canny Near");
        //destroyWindow("Canny Far");
        //destroyWindow("RGB Image");
        destroyWindow("Canny");
        //destroyWindow("GreyScale Image");
        //destroyWindow("Binary Image");
        //destroyWindow("Canny Image");
        //destroyWindow("LaneTracking");  
    }  
#endif    

    return cFilter::Stop(__exception_ptr);
}
tResult cParkDetection::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		/* -----------------  Media Descriptions  -----------------*/
		// Media Description ParkingID
		tChar const * ParkingDesc = pDescManager->GetMediaDescription("tParkingStruct");
		RETURN_IF_POINTER_NULL(ParkingDesc);
		cObjectPtr<IMediaType> pTypeParkingDesc = new cMediaType(0, 0, 0, "tParkingStruct", ParkingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Media Description Signal
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);        
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    

		// Media Description Lane
		tChar const * LaneDesc = pDescManager->GetMediaDescription("tParkingLotStruct");
		RETURN_IF_POINTER_NULL(LaneDesc);
		cObjectPtr<IMediaType> pTypeLaneDesc = new cMediaType(0, 0, 0, "tParkingLotStruct", LaneDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		/*-----------------  INPUT ----------------*/
        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("Start", pTypeParkingDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeParkingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartInput)); 

		// Lane from LaneTracking (Input)
    	RETURN_IF_FAILED(m_iLane.Create("Lane", pTypeLaneDesc, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iLane));
		RETURN_IF_FAILED(pTypeLaneDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLane));
	
		// Finished Searching (Input)
		RETURN_IF_FAILED(m_iStop.Create("Stop", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStop));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopInput));

		/*--------------- OUTPUT -----------------*/
        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

		// Create Pin for DistanceToParkingSpot (Output)
		RETURN_IF_FAILED(m_oDistanceToParkingSpot.Create("Dist_To_Parking_Spot", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceToParkingSpot));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceToParkingSpotOutput)); 

		// Create Pin for DistanceToParkingSpot (Output)
		RETURN_IF_FAILED(m_oDistanceToParkingSpotDM.Create("Dist_To_Parking_Spot_DM", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceToParkingSpotDM));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceToParkingSpotOutputDM)); 

              
    }
    else if (eStage == StageNormal)
    {
		m_bLaneInputSet = tFalse;
		m_startInputSet = tFalse;
		m_stopInputSet = tFalse;
		m_parkingSpotOutputSet = tFalse;
		m_parkingSpotOutputSetDM = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
		m_firstSpot = tTrue;
		InitialState();

    }
    RETURN_NOERROR;
}

tResult cParkDetection::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cParkDetection::ReadProperties(const tChar* strPropertyName)
{
    
if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_CornerHarrisparamK))
    {
		float f;
		f = GetPropertyFloat(ParkDetection_PROP_CornerHarrisparamK);
		m_nCornerHarrisparamK = static_cast<double>(f);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_CornerHarrisblockSize))
    {
        m_nCornerHarrisblockSize = GetPropertyInt(ParkDetection_PROP_CornerHarrisblockSize);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_CornerHarrisksize))
    {
        m_nCornerHarrisksize = GetPropertyInt(ParkDetection_PROP_CornerHarrisksize);
    }


    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_TRESHOLD))
    {
        m_nThresholdValue = GetPropertyInt(ParkDetection_PROP_TRESHOLD);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_TRESHOLD))
    {
        m_nThresholdValue2 = GetPropertyInt(ParkDetection_PROP_TRESHOLD2);
    }
	
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(ParkDetection_PROP_SHOW_DEBUG);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_ImagecutWidthLeft))
    {
        m_nImagecutWidthLeft = GetPropertyInt(ParkDetection_PROP_ImagecutWidthLeft);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_ImagecutWidthRight))
    {
        m_nImagecutWidthRight = GetPropertyInt(ParkDetection_PROP_ImagecutWidthRight);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_ImagecutHeightUp))
    {
        m_nImagecutHeightUp = GetPropertyInt(ParkDetection_PROP_ImagecutHeightUp);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_ImagecutHeightDown))
    {
        m_nImagecutHeightDown = GetPropertyInt(ParkDetection_PROP_ImagecutHeightDown);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(ParkDetection_PROP_ThresholdValueCanny);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_PointcloudTolerance))
    {
        m_nPointcloudTolerance = GetPropertyInt(ParkDetection_PROP_PointcloudTolerance);
    }



    RETURN_NOERROR;
}

tResult cParkDetection::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cParkDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cParkDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
                __adtf_sample_read_lock_mediadescription(m_pStartInput,pMediaSample,pCoder);

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


			if(!m_bActive) InitialState();
        }

		else if(pSource == &m_iStop)
        {
			{   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pStopInput,pMediaSample,pCoder);

				if (!m_stopInputSet) {
					pCoder->GetID("bValue", m_szIDStopInput);
					m_stopInputSet = tTrue;
				}

				//pCoder->Get(m_szIDStopInput, (tVoid*)&m_bActive);
            }

			m_bActive = tFalse;
			InitialState();
        }

		else if(pSource == &m_iLane)
        {
			ProcessLane(pMediaSample);
		}
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cParkDetection::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{ if (m_bActive){
		tictoc = _clock->GetStreamTime();

		//if(!m_bLaneInputSet)RETURN_NOERROR; 
        m_ui8InitCtrl++;

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

			m_matNormal = m_matImageRGB;  // memeber var um in Search Corners verwendet zu werden
			// detect houghlines
			Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image 
		    cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 		
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat()); // normalize (helps to compensate different illumination)
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. 
			/*fstream f1;
			f1.open("scaled.dat",ios::out);
			f1 << m_matCutHough<< "\n";
			f1.close();*/
			threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO); // delete dark noise	
			/*fstream f2;
			f2.open("thres.dat",ios::out);
			f2 << m_matCutHough<< "\n";
			f2.close();*/
		    medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
			/*fstream f3;
			f3.open("median.dat",ios::out);
			f3 << m_matCutHough<< "\n";
			f3.close();*/
		    Canny(m_matCutHough, m_matCannyHough,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);
			/*fstream f4;
			f4.open("canny.dat",ios::out);
			f4 << m_matCannyHough<< "\n";
			f4.close();*/

			threshold(m_matCutHough, m_matThres2, 0, 255,THRESH_BINARY); 
			cornerHarris(m_matThres2, m_matThres2, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			/*fstream f5;
			f5.open("corner.dat",ios::out);
			f5 << m_matThres2<< "\n";
			f5.close();*/
			threshold(m_matThres2, m_matThres2, m_nThresholdValue, 255,THRESH_TOZERO); // delete dark noise
			/*fstream f4;
			f4.open("thres.dat",ios::out);
			f4 << m_matThres2<< "\n";
			f4.close();*/
			normalize(m_matThres2,m_matThres2, 0, 255, NORM_MINMAX, CV_32FC1,Mat()); // normalize (helps to compensate different illumination)
			convertScaleAbs(m_matThres2, m_matThres2); // Konveriere in 8Bit array.			
			/*fstream f2;
			f2.open("scaled.dat",ios::out);
			f2 << m_matThres2<< "\n";
			f2.close();*/
			threshold(m_matThres2, m_matThres2, m_nThresholdValue2, 255,THRESH_BINARY); 
			/*fstream f1;
			f1.open("thres2.dat",ios::out);
			f1 << m_matThres2<< "\n";
			f1.close();*/

		RGBreceived = true;
		 // damit nur ein RGB und Tiefenbild empfangen wird
		
	}
 //}
	RETURN_NOERROR;            
}

tResult cParkDetection::ProcessInputDepth(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	if(!m_bActive) RETURN_NOERROR; 

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

tResult cParkDetection::ProcessLane(IMediaSample* pMediaSample)
{
	{ // sample read lock
	__adtf_sample_read_lock_mediadescription(m_pCoderDescLane, pMediaSample, pCoder);    
		if(!m_bLaneInputSet)
		{
			pCoder->GetID("fz1", m_szIDSignalValueLaneSpot1XInput);
			pCoder->GetID("fz2", m_szIDSignalValueLaneSpot1YInput);
			pCoder->GetID("fz3", m_szIDSignalValueLaneSpot2XInput);
			pCoder->GetID("fz4", m_szIDSignalValueLaneSpot2YInput);
			m_bLaneInputSet = tTrue;
		}
		// set value from sample
		pCoder->Get(m_szIDSignalValueLaneSpot1XInput, (tVoid*)&m_LaneSpots[0]); 
		pCoder->Get(m_szIDSignalValueLaneSpot1YInput, (tVoid*)&m_LaneSpots[1]); 
		pCoder->Get(m_szIDSignalValueLaneSpot2XInput, (tVoid*)&m_LaneSpots[2]); 
		pCoder->Get(m_szIDSignalValueLaneSpot2YInput, (tVoid*)&m_LaneSpots[3]); 
	}	

	m_LaneSpots[0] 	-= m_nImagecutWidthLeft;
	m_LaneSpots[1] 	-= m_nImagecutHeightUp;
	m_LaneSpots[2] 	-= m_nImagecutWidthLeft;
	m_LaneSpots[3] 	-= m_nImagecutHeightUp;

	m_ui8InitCtrl=0;

    RETURN_NOERROR;
}

tResult cParkDetection::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cParkDetection::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cParkDetection::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cParkDetection::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cParkDetection::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

tResult cParkDetection::SearchCorners()
{	

	pui8PointsCount    = 0;
    Size szImageCutThresSize = m_matThres2.size();
	m_Points.clear(); 
	m_PixelSpot.clear();
	
	// Write corners from Matrix to vector
    for(tInt nColumn=0; nColumn < szImageCutThresSize.width; nColumn++)
    {  
    	for(tInt nRow=0; nRow < szImageCutThresSize.height; nRow++)
    		{  
                 
        	if(m_matThres2.at<uchar>(nRow, nColumn)==255) 
        		{ 
				m_Points.push_back(Point2f(nColumn,nRow));         
            	}
        	}
    }

	// Calc Lane from Infos sent from LaneTracking
	if( m_ui8InitCtrl<20)
	{
		double m = 0.0;
		double b = 0.0;

		if(!(m_LaneSpots[0] == m_LaneSpots[2]) || !(m_LaneSpots[1] == m_LaneSpots[3]) )				
		{
			m = (m_LaneSpots[3]-m_LaneSpots[1])/(m_LaneSpots[2]-m_LaneSpots[0]);
			b = m_LaneSpots[3] - m*m_LaneSpots[2];			
		}

		tUInt i = 0;
		m_intersecPoint.clear();
		while(i<m_Points.size())
		{
			tInt16 xint = cvRound((m_Points[i].y-b)/m);
			tInt16 yint = m_Points[i].y;
			m_intersecPoint.push_back(Point2i(xint,yint));
			Point3f inter3d = Util::ComputeWorldCoordinate(static_cast<float>(xint), static_cast<float>(yint),m_matImageDepth.at<ushort>(floor(static_cast<float>(yint)*0.5), floor(static_cast<float>(yint)*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			// calc 3d coords of potential park point
			Point3f corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5), floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);			
			// delete all corners that are too far off the Lane or delete if the z value is invalid
			if(abs(corner1.x-inter3d.x)>0.09 || corner1.y>0.25 || corner1.y<0.20 || corner1.z<0.2 || inter3d.z<0.2)
			{
				m_Points.erase(m_Points.begin()+i);	
				i--;
			}
			i++;
		}

	}

/*
//elimininate pointcloud 
		vector<int> killUs;
		size_t i=0;
		size_t j=0;
		while(i<m_Points.size())
		{	

			j=i+1;
			corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5),floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			while(j<m_Points.size())
			{
				
				corner2 = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
				if(norm(corner1-corner2)<0.17)
				{
					if(norm(corner1)< norm(corner2))
					{ 
						m_Points.erase(m_Points.begin()+j);
					}
					else
					{
						m_Points.erase(m_Points.begin()+i);
					}						
				}
				j++;
			}
			i++;
		}*/
	//elimininate pointcloud 
		vector<int> killUs;
		
		for(size_t i=0;i<m_Points.size();i++)
			{	
			Point3f corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5),floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			for(size_t j=i+1;j<m_Points.size();j++)
				{
					Point3f corner2 = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
					if(norm(corner1-corner2)<m_nPointcloudTolerance)
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

	if(!killUs.empty())
	{
		sort(killUs.begin(),killUs.end(),greater<int>());
		for(size_t i=0;i<killUs.size();i++)
			{
				m_Points.erase(m_Points.begin()+killUs[i]);
			}
	}




	// Search actual parking lot
	vector<Point3f> ParkPoint;
	 for(size_t i=0;i<m_Points.size();i++)
		{
		Point3f corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5),floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
	 	for(size_t j=i+1;j<m_Points.size();j++)
			{
			Point3f corner2 = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
			if(norm(corner1-corner2)<m_lotDim[m_parkingType][0] && norm(corner1-corner2)>m_lotDim[m_parkingType][1]){
					m_PixelSpot.push_back(Point2f(m_Points[i].y,m_Points[i].x));
					m_PixelSpot.push_back(Point2f(m_Points[j].y,m_Points[j].x));
					ParkPoint.push_back(corner1);
					ParkPoint.push_back(corner2);
					}
				}
		}


/*
//Check for free ParkingSpot
tUInt j=0;
if(!m_PixelSpot.empty())
{
while(j<m_PixelSpot.size()-1)
	{
		//midField.push_back(Point2f(cvRound((m_PixelSpot[j].x + m_PixelSpot[j+1].x)/2),cvRound((m_PixelSpot[j].y + m_PixelSpot[j+1].y)/2)));
		Point2f midField = Point2f(cvRound((m_PixelSpot[j].x + m_PixelSpot[j+1].x)/2),cvRound((m_PixelSpot[j].y + m_PixelSpot[j+1].y)/2));
		Point3f midField3d = Util::ComputeWorldCoordinate(midField.x, midField.y,m_matImageDepth.at<ushort>(floor(midField.y*0.5),floor(midField.x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
		
		// in case of a near parking lot we have better resolution, hence we need to count more pixels to look into the lot
		if(midField3d.z<1.5)
		{
			for(int i=0;i<m_parkLotPixelSize[m_parkingType][0];i+=3)
			{
				Point3f midFieldPoint = Util::ComputeWorldCoordinate(midField.x, midField.y,m_matImageDepth.at<ushort>(floor(midField.y*0.5),floor(midField.x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);;
				// if there is a car, but in the previous parking lot, we cannot say if there is also a car in this parking lot
				if((midFieldPoint.y<0.23 && midFieldPoint.y>0.19 && midFieldPoint.z < ParkPoint[j].z) || midFieldPoint.z<0.2) 
				{
				//	LOG_WARNING(cString::Format("%f",midFieldPoint.y));
				}
				// if there is a car && it is in our it is in this parking lot -> erase
				else
				{
					// delete the nearer ParkPoint, since the nearer is the representer for the parking lot
					if(ParkPoint[j].z>ParkPoint[j+1].z)
					{
						//LOG_WARNING(cString::Format("Parkspotfirst:Full"));
						ParkPoint.erase(ParkPoint.begin()+j+1);
						m_PixelSpot.erase(m_PixelSpot.begin()+j+1);
						j--;
						break;
					}
					else
					{
						//LOG_WARNING(cString::Format("Parkspotfirst:Full"));
						ParkPoint.erase(ParkPoint.begin()+j);
						m_PixelSpot.erase(m_PixelSpot.begin()+j);
						j--;
						break;
					}
				}
				// if we haven't found a car anywhere the lot is not occupied -> do nothing
			}//for
		}//if
		else
		{
			for(int i=0;i<m_parkLotPixelSize[m_parkingType][1];i+=3)
			{
				Point3f midFieldPoint = Util::ComputeWorldCoordinate(midField.x, midField.y,m_matImageDepth.at<ushort>(floor(midField.y*0.5),floor(midField.x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);;
				// if there is a car, but in the previous parking lot, we cannot say if there is also a car in this parking lot
				if((midFieldPoint.y<0.23 && midFieldPoint.y>0.19 && midFieldPoint.z < ParkPoint[j].z) || midFieldPoint.z<0.2) 
				{
				}
				// if there is a car && it is in our it is in this parking lot -> erase
				else
				{
				// delete the nearer ParkPoint, since the nearer is the representer for the parking lot
					if(ParkPoint[j].z>ParkPoint[j+1].z)
					{
						//LOG_WARNING(cString::Format("ParkspotSecond:Full"));
						ParkPoint.erase(ParkPoint.begin()+j+1);
						m_PixelSpot.erase(m_PixelSpot.begin()+j+1);
						j--;
						break;
					}
					else
					{
						//LOG_WARNING(cString::Format("ParkspotSecond:Full"));
						ParkPoint.erase(ParkPoint.begin()+j);
						m_PixelSpot.erase(m_PixelSpot.begin()+j);
						j--;
						break;
					}
				}
				// if we haven't found a car anywhere the lot is not occupied -> do nothing
			}//for

		}	
		j++;
	}
}
*/
/*
if(!ParkPoint.empty())
{
			fstream fff;
			fff.open("ParkPoints.dat",ios::out);
			fff << ParkPoint<< "\n";
			fff.close();
}*/

//HIER PARKSPOTS MIT HOUGH SUCHEN UND DANN MIT DEN ECKEN VERLEICHEN
//eliminate Houghline-cloud of Hough


/*m_HoughGroup2.clear();
vector<Point2f> m_HoughGroup2;
	for(size_t i=0; i<m_lines.size();i++)
	{
		if((m_lines[i][1]*180/M_PI<100) && (m_lines[i][1]*180/M_PI>80))
		{
			m_HoughGroup2.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	
		}
	}
*/
/*
//Calculate representer for HoughGroup2

//GIBT MEHRERE qUERLINIEN. WIE UNTERSCHEIDEN?
if(HoughGroup2.size()!=0)
{
float averageHG2 = 0;
m_representerHG2 = m_HoughGroup2[0]; 
	for(size_t i=0; i<m_HoughGroup2.size();i++)
		{
			averageHG2 = averageHG2 + m_HoughGroup2[i].y*180/M_PI;
		}
			averageHG2 = averageHG2/m_HoughGroup2.size();

	for(size_t i=0; i<HoughGroup2.size();i++)
		{
			if(abs(m_representerHG2.y*180/M_PI-averageHG2)>abs(HoughGroup2[i].y*180/M_PI-averageHG2))
			{
				m_representerHG2 = HoughGroup2[i];
			}
		}
}
//calculate intersection point of representer of horizontal houghline and representer of right lane(first parkingspot) 
	if(HoughGroup2.size()!=0)
		{
		int idxH = 0;
		float max = 0;
			for(size_t i=0; i<HoughRGroup2.size();i++)
				{
					if(HoughRGroup2[i].x>max)
						{
							idxH = i;
							max = HoughRGroup2[i].x;
						}
				}

				Point2f pt1,pt2,pt1r,pt2r;
				pt1.x = cos(HoughRGroup2[idxH].y)*HoughRGroup2[idxH].x + 1000*(-sin(HoughRGroup2[idxH].y));
				pt1.y = sin(HoughRGroup2[idxH].y)*HoughRGroup2[idxH].x + 1000*(cos(HoughRGroup2[idxH].y));
				pt2.x = cos(HoughRGroup2[idxH].y)*HoughRGroup2[idxH].x - 1000*(-sin(HoughRGroup2[idxH].y));
				pt2.y = sin(HoughRGroup2[idxH].y)*HoughRGroup2[idxH].x - 1000*(cos(HoughRGroup2[idxH].y));

				Point2f pt1,pt2,pt1r,pt2r;
				pt1.x = cos(m_representerHG2.y)*m_representerHG2.x + 1000*(-sin(m_representerHG2.y));
				pt1.y = sin(m_representerHG2.y)*m_representerHG2.x + 1000*(cos(m_representerHG2.y));
				pRERE qUERLINIEN. WIE UNTERSCHEIDEN?
if(HoughGroup2.size()!=0)
{

float averageHG2 = 0;
m_representerHG2 = m_HoughGroup2[0]; 
	for(size_t i=0; i<m_HoughGroup2.size();i++)
		{

			averageHG2 = averageHG2 + m_HoughGroup2[i].y*180/M_PI;
		}
			averageHG2 = averageHG2/m_HoughGroup2.size();

	for(size_t i=0; i<HoughGroup2.size();i++)
		{
			if(abs(m_representerHG2.y*180/M_PI-averageHG2)>abs(HoughGroup2[i].y*180/M_PI-averageHG2))

			{
				m_representerHG2 = Hout2.x = cos(m_representerHG2.y)*m_representerHG2.x - 1000*(-sin(m_representerHG2.y));
				pt2.y = sin(m_representerHG2.y)*m_representerHG2.x - 1000*(cos(m_representerHG2.y));

				pt1r.x = cos(m_representerHG1.y)*m_representerHG1.x + 1000*(-sin(m_representerHG1.y));
				pt1r.y = sin(m_representerHG1.y)*m_representerHG1.x + 1000*(cos(m_representerHG1.y));
				pt2r.x = cos(m_representerHG1.y)*m_representerHG1.x - 1000*(-sin(m_representerHG1.y));
				pt2r.y = sin(m_representerHG1.y)*m_representerHG1.x - 1000*(cos(m_representerHG1.y));

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
				m_ParkPointHough = Util::ComputeWorldCoordinate(m_intersecpointR.x,m_intersecpointR.y,m_matImageDepth.at<ushort>(floor(m_intersecpointR.y*0.5),floor(m_intersecpointR.x*0.5)),m_nImagecutHeightUpHough,m_nImagecutWidthLeftHough);
				}
		}
*/
///////////////////////////


	int idx = -1;
	double buffer;
	double MAX1 = 0;//norm(ParkPoint.at(0));
	for(size_t i=0; i<ParkPoint.size();i++)
		{
			buffer = norm(ParkPoint[i]);
			if(buffer>MAX1)
				{
					MAX1 = buffer;
					idx = i;
				}
		}
	if(m_iframeCounter>500)
	{

		//LOG_WARNING(cString::Format("!!!!!!ParkD: Parkmaneuver cancelled. Timeframe of 12s reached!!!!!"));
		m_iframeCounter = 0;
		TransmitParkingSpot(0.0);
		m_bActive = tFalse;
		RETURN_NOERROR;
	}
	else if(ParkPoint.empty())
	{
		//LOG_WARNING(cString::Format("ParkD: No parking lot found!"));
		// Try again if failed
	}
	else
	{	

		//m_bActive = tTrue;
		//LOG_INFO(cString::Format("ParkD: Parking lots sent to DM! :)") );

		// Schreibe gefundene Parkluecke in Outputpin	
 		if (!m_oDistanceToParkingSpot.IsConnected())
   		{
			LOG_WARNING(cString::Format("ParkD: OutputPin for parking lot not connected!"));
    	    RETURN_NOERROR;
    	}
		// Send found parking spot
		vector<float> temp;
		for(size_t i=0; i < ParkPoint.size(); i++)
		{
			if(ParkPoint[i].z<1.7 || ParkPoint[i].z>0.58){
				temp.push_back(ParkPoint[i].z);
			}
		}
		sort(temp.begin(),temp.end());
		
		m_iframeCounter = 0;
		for(size_t i=0; i < temp.size(); i++)
		{
		//	TransmitParkingSpot(temp[i]);
		}
		if(temp.size()>0) TransmitParkingSpot(temp[0]);

	}
	//LOG_INFO(cString::Format("ParkD: Finished Searching") );
	RETURN_NOERROR;
}


tResult cParkDetection::CreateAndTransmitGCL()
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
    cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 80, 80).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,m_nImagecutWidthLeft , m_nImagecutHeightUp, m_nImagecutWidthRight, m_nImagecutHeightDown);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_LaneSpots[0]-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_LaneSpots[1]-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_LaneSpots[0]+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_LaneSpots[1]+2) + m_nImagecutHeightUp);	
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_LaneSpots[2]-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_LaneSpots[3]-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_LaneSpots[2]+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_LaneSpots[3]+2) + m_nImagecutHeightUp);
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, (static_cast<tInt16>(m_LaneSpots[2])+m_nImagecutWidthLeft), (static_cast<tInt16>(m_LaneSpots[3])+m_nImagecutHeightUp), static_cast<tInt16>((m_LaneSpots[0]+m_nImagecutWidthLeft)), static_cast<tInt16>((m_LaneSpots[1]+m_nImagecutHeightUp)));

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 0).GetRGBA());
	for(size_t i=0; i < m_Points.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_Points[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_Points[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_Points[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_Points[i].y+2) + m_nImagecutHeightUp);
		}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
	for(size_t i=0; i < m_intersecPoint.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_intersecPoint[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_intersecPoint[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_intersecPoint[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_intersecPoint[i].y+2) + m_nImagecutHeightUp);
		}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
	for(size_t i=0; i < m_PixelSpot.size(); i++)
		{
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_PixelSpot[i].y-4) + m_nImagecutWidthLeft , static_cast<tInt16>(m_PixelSpot[i].x-4) + m_nImagecutHeightUp, static_cast<tInt16>(m_PixelSpot[i].y+4) + m_nImagecutWidthLeft, static_cast<tInt16>(m_PixelSpot[i].x+4) + m_nImagecutHeightUp);
		}
	//m_PixelSpot.clear();
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 255).GetRGBA());
	for(size_t i=0; i < m_intersecPoint.size(); i++)
		{
		//cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_intersecPoint[i].x-4) + m_nImagecutWidthLeft , static_cast<tInt16>(m_intersecPoint[i].y-4) + m_nImagecutHeightUp, static_cast<tInt16>(m_intersecPoint[i].x+4) + m_nImagecutWidthLeft, static_cast<tInt16>(m_intersecPoint[i].y+4) + m_nImagecutHeightUp);
		}

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

tResult cParkDetection::TransmitParkingSpot(float lot) {

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	if (m_firstSpot){

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistanceToParkingSpotOutputDM->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pDistanceToParkingSpotOutputDM, pMediaSample, pCoderOutput); 
	
			if(!m_parkingSpotOutputSetDM){
				pCoderOutput->GetID("f32Value", m_szIDParkingSpot1OutputDM);
				m_parkingSpotOutputSetDM = tTrue;
			}
			
			pCoderOutput->Set(m_szIDParkingSpot1OutputDM, (tVoid*)&(lot));
		
		}

		//LOG_INFO(cString::Format("PD: Send spot to DM!") );

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceToParkingSpotDM.Transmit(pMediaSample);

		m_firstSpot 	= tFalse;

	}else{

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistanceToParkingSpotOutput->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pDistanceToParkingSpotOutput, pMediaSample, pCoderOutput); 
	
			if(!m_parkingSpotOutputSet){
				pCoderOutput->GetID("f32Value", m_szIDParkingSpot1Output);
				m_parkingSpotOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szIDParkingSpot1Output, (tVoid*)&(lot));
		
		}

		//LOG_INFO(cString::Format("PD: Send spot to SFP!") );

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceToParkingSpot.Transmit(pMediaSample);
	}

	RETURN_NOERROR;
}

tResult cParkDetection::InitialState(){

        m_ui8Imagecount = 0;
		m_bActive = tFalse;

        ReadProperties(NULL);

        m_ui8InitCtrl = 0;

		m_iframeCounter = 0;

		m_firstSpot 	= tTrue;

RETURN_NOERROR;
}
