/**
*
*ParkDetection
*
* Date 17.03.2016
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

#define ParkDetection_PROP_TRESHOLD "ParkDetection::ThresholdValue"
#define ParkDetection_PROP_TRESHOLD2 "ParkDetection::ThresholdValue2"
#define ParkDetection_PROP_TRESHOLDBEFORENORM "ParkDetection::ThresholdValueBeforeNorm"
#define ParkDetection_PROP_CornerHarrisparamK "ParkDetection::CornerHarrisparamK"
#define ParkDetection_PROP_CornerHarrisblockSize "ParkDetection::CornerHarrisblockSize"
#define ParkDetection_PROP_CornerHarrisksize "ParkDetection::CornerHarrisksize"

#define ParkDetection_PROP_PointCloudSizeNear "ParkDetection::PointCloudSizes::PointCloudSizeNear"
#define ParkDetection_PROP_LaneToleranceNear "ParkDetection::PointCloudSizes::LaneToleranceNear"
#define ParkDetection_PROP_PointCloudSizeFar "ParkDetection::PointCloudSizes::PointCloudSizeFar"
#define ParkDetection_PROP_LaneToleranceFar "ParkDetection::PointCloudSizes::LaneToleranceFar"

#define ParkDetection_PROP_ImagecutWidthLeft "ParkDetection::ImagecutWidthLeft"
#define ParkDetection_PROP_ImagecutWidthRight "ParkDetection::ImagecutWidthRight"
#define ParkDetection_PROP_ImagecutHeightUp "ParkDetection::ImagecutHeightUp"
#define ParkDetection_PROP_ImagecutHeightDown "ParkDetection::ImagecutHeightDown"

#define ParkDetection_PROP_SHOW_DEBUG "Common::Show Debug"

#define MAX_DEVIATION 150

cParkDetection::cParkDetection(const tChar* __info) : cFilter(__info)
{
	//psPoints.x=0;
	pui8PointsCount    = 0;
	RGBreceived 	   = false;

	// parallel parking
	m_lotDim[0][0] = 0.83;
	m_lotDim[0][1] = 0.74;
	// cross parking
	m_lotDim[1][0] = 0.505;// ;0.54;
	m_lotDim[1][1] = 0.42;//;0.41;

	/*// parallel parking free spot pixel
	m_parkLotPixelSize[0][0] = 30;
	m_parkLotPixelSize[0][1] = 21;

	// cross parking free spot pixel
	m_parkLotPixelSize[1][0] = 63;
	m_parkLotPixelSize[1][1] = 36;
  */  
    SetPropertyBool(ParkDetection_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(ParkDetection_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

	SetPropertyInt(ParkDetection_PROP_ImagecutWidthLeft, 320);
	SetPropertyInt(ParkDetection_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutWidthRight, 625); 
	SetPropertyInt(ParkDetection_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutHeightUp, 70);
	SetPropertyInt(ParkDetection_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ParkDetection_PROP_ImagecutHeightDown, 245);
	SetPropertyInt(ParkDetection_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(ParkDetection_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");

    SetPropertyInt(ParkDetection_PROP_TRESHOLD, 40);
    SetPropertyBool(ParkDetection_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The first threshold value (deletes noise).");
	
	SetPropertyInt(ParkDetection_PROP_TRESHOLD2, 120);
    SetPropertyBool(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The second threshold value (determines number of corners).");

	SetPropertyFloat(ParkDetection_PROP_TRESHOLDBEFORENORM, 8000);
    SetPropertyBool(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The threshold before normalize (applied on the corner Matrix!).");

	SetPropertyInt(ParkDetection_PROP_CornerHarrisblockSize, 5);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(ParkDetection_PROP_CornerHarrisksize, 11);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(ParkDetection_PROP_CornerHarrisparamK, 0.08);
    SetPropertyBool(ParkDetection_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");

	SetPropertyInt(ParkDetection_PROP_PointCloudSizeNear, 8);
    SetPropertyBool(ParkDetection_PROP_PointCloudSizeNear NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_PointCloudSizeNear NSSUBPROP_DESCRIPTION, "Erase distance to other corners for Pointcloudsize");

	SetPropertyInt(ParkDetection_PROP_PointCloudSizeFar, 4);
    SetPropertyBool(ParkDetection_PROP_PointCloudSizeFar NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_PointCloudSizeFar NSSUBPROP_DESCRIPTION, "Erase distance to other corners for Pointcloudsize");

	SetPropertyInt(ParkDetection_PROP_LaneToleranceNear, 80);
    SetPropertyBool(ParkDetection_PROP_LaneToleranceNear NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_LaneToleranceNear NSSUBPROP_DESCRIPTION, "Erase distance to other corners for Pointcloudsize");

	SetPropertyInt(ParkDetection_PROP_LaneToleranceFar, 35);
    SetPropertyBool(ParkDetection_PROP_LaneToleranceFar NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ParkDetection_PROP_LaneToleranceFar NSSUBPROP_DESCRIPTION, "Erase distance to other corners for Pointcloudsize");

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
	
/*		// Finished Searching (Input)
		RETURN_IF_FAILED(m_iStop.Create("Stop", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStop));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopInput));
*/
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

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_TRESHOLD2))
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

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_PointCloudSizeNear))
    {
		m_pointCloudSizeNear = GetPropertyInt(ParkDetection_PROP_PointCloudSizeNear);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_LaneToleranceNear))
    {
		m_LaneToleranceNear = GetPropertyInt(ParkDetection_PROP_LaneToleranceNear);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_PointCloudSizeFar))
    {
		m_pointCloudSizeFar = GetPropertyInt(ParkDetection_PROP_PointCloudSizeFar);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_LaneToleranceFar))
    {
		m_LaneToleranceFar = GetPropertyInt(ParkDetection_PROP_LaneToleranceFar);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ParkDetection_PROP_TRESHOLDBEFORENORM))
    {
        m_nThresholdValueBeforeNorm = GetPropertyInt(ParkDetection_PROP_TRESHOLDBEFORENORM);
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

			if(bValue){
				LOG_INFO(cString::Format("PD: Starting ParkDetection with ID = %f", m_szIDStartInput));
				InitialState();
			}
			else LOG_INFO(cString::Format("PD: Shutdown ParkDetection") );

			m_bActive = bValue;
			m_parkingType = type-3;
			
        }

/*		else if(pSource == &m_iStop)
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
*/
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

		    IplImage* oImg = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
		    RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
		    oImg->imageData = (char*)l_pSrcBuffer;
		    Mat image(cvarrToMat(oImg));
		    cvReleaseImage(&oImg);
		    pSample->Unlock(l_pSrcBuffer);

			m_matCorner =  image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image 
			cvtColor(m_matCorner, m_matCorner, CV_RGB2GRAY);// Grey Image
			/*fstream f;
			f.open("color.dat",ios::out);
			f << m_matCorner<< "\n";
			f.close();*/
			threshold(m_matCorner, m_matThres, m_nThresholdValue, 255,THRESH_TOZERO);
			m_matThres=m_matThres-m_nThresholdValue;
			/*fstream f1;
			f1.open("thres1.dat",ios::out);
			f1 << m_matThres<< "\n";
			f1.close();*/
			medianBlur(m_matThres, m_matCorner, 3); // reduce noise with edge-preserving filter
			/*fstream f2;
			f2.open("median.dat",ios::out);
			f2 << m_matCorner<< "\n";
			f2.close();*/
			cornerHarris(m_matCorner, m_matCorner, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			/*fstream f6;
			f6.open("corner.dat",ios::out);
			f6 << m_matCorner<< "\n";
			f6.close();*/
			threshold(m_matCorner, m_matThres, 0.01, 255,THRESH_TOZERO); // in case of no real corners set everything to zero - otherwise everywhere corners because of normalize
			threshold(m_matThres, m_matThres, m_nThresholdValueBeforeNorm, 255,THRESH_TRUNC);
			/*fstream f3;
			f3.open("BeforeNormalized.dat",ios::out);
			f3 << m_matThres<< "\n";
			f3.close();*/
			normalize(m_matThres, m_matNormalized, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
			convertScaleAbs(m_matNormalized, m_matNormalized); // Konveriere in 8Bit array. Nehme aus abs
			/*fstream f8;
			f8.open("scaled.dat",ios::out);
			f8 << m_matNormalized<< "\n";
			f8.close();*/
			threshold(m_matNormalized, m_matThres2,  m_nThresholdValue2, 255,THRESH_BINARY); 
			/*fstream f4;
			f4.open("thres3.dat",ios::out);
			f4 << m_matThres2<< "\n";
			f4.close();*/

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
    for(tInt nColumn=3; nColumn < szImageCutThresSize.width-3; nColumn++)
    {  
    	for(tInt nRow=3; nRow < szImageCutThresSize.height-3; nRow++)
    		{  
                 
        	if(m_matThres2.at<uchar>(nRow, nColumn)==255) 
        		{ 
				m_Points.push_back(Point2i(nColumn,nRow));         
            	}
        	}
    }

	for(size_t i=0;i<m_Points.size();i++)
	{	
		for(size_t j=i+1;j<m_Points.size();j++)
		{	
			if( (m_Points[i].y <= cvRound((m_nImagecutHeightDown-m_nImagecutHeightUp)*0.4) && (norm(m_Points[i]-m_Points[j])<m_pointCloudSizeFar )) ||
				(m_Points[i].y > cvRound((m_nImagecutHeightDown-m_nImagecutHeightUp)*0.4) && (norm(m_Points[i]-m_Points[j])<m_pointCloudSizeNear )) )
			{
				if(m_Points[i].y > m_Points[j].y)
				{ 
					m_Points.erase(m_Points.begin()+j);
					j--;
				}
				else
				{
					m_Points.erase(m_Points.begin()+i);
					i--;
					break;
				}						
			}		
		}
	}

	m_PointsNeu.clear();
	for(size_t i=0;i<m_Points.size();i++)
	{
		m_PointsNeu.push_back(m_Points[i]);
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
			if( (m_Points[i].y <= cvRound((m_nImagecutHeightDown-m_nImagecutHeightUp)*0.4) && (abs(m_Points[i].x-xint)>m_LaneToleranceFar)) ||
				(m_Points[i].y > cvRound((m_nImagecutHeightDown-m_nImagecutHeightUp)*0.4) && (abs(m_Points[i].x-xint)>m_LaneToleranceNear)) )
			{
				//LOG_INFO(cString::Format("abs: %i,x:%i",abs(m_Points[i].x-xint),xint));
				m_Points.erase(m_Points.begin()+i);	
				i--;
			}
			else if(fabs(corner1.x-inter3d.x)>0.11 || corner1.y>0.25 || corner1.y<0.20 || corner1.z<0.4 || inter3d.z<0.4)
			{
				m_Points.erase(m_Points.begin()+i);	
				i--;
			}
			i++;
		}

	}

	// Search actual parking lot
//	vector<Point3f> ParkPoint; ist jetzt global
	ParkPoint.clear();
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
///////////////////////////


	/*int idx = -1;
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
		}*/

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
			if(ParkPoint[i].z<2.0 || ParkPoint[i].z>0.20){
				temp.push_back(ParkPoint[i].z);
			}
		}
		sort(temp.begin(),temp.end());
		
		m_iframeCounter = 0;
		for(size_t i=0; i < temp.size(); i++)
		{
			TransmitParkingSpot(temp[i]);
		}
		//if(temp.size()>0) TransmitParkingSpot(temp[0]);

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

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
	for(size_t i=0; i < m_PointsNeu.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_PointsNeu[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_PointsNeu[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_PointsNeu[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_PointsNeu[i].y+2) + m_nImagecutHeightUp);
		}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 0).GetRGBA());
	for(size_t i=0; i < m_Points.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_Points[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_Points[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_Points[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_Points[i].y+2) + m_nImagecutHeightUp);
		}

/*	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());
	for(size_t i=0; i < m_intersecPoint.size(); i++) 
		{
			//cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_intersecPoint[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_intersecPoint[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_intersecPoint[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_intersecPoint[i].y+2) + m_nImagecutHeightUp);
		}
*/
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
	for(size_t i=0; i < m_PixelSpot.size(); i++)
	{

		tFloat64 text = ParkPoint[i].z;
		cString strText = cString::FromFloat64(text);
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, static_cast<tInt16>(m_PixelSpot[i].y+10) + m_nImagecutWidthLeft, static_cast<tInt16>(m_PixelSpot[i].x-10) + m_nImagecutHeightUp, strText.GetLength());
		cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_PixelSpot[i].y-4) + m_nImagecutWidthLeft , static_cast<tInt16>(m_PixelSpot[i].x-4) + m_nImagecutHeightUp, static_cast<tInt16>(m_PixelSpot[i].y+4) + m_nImagecutWidthLeft, static_cast<tInt16>(m_PixelSpot[i].x+4) + m_nImagecutHeightUp);
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

	m_firstSpot = tTrue;

	RETURN_NOERROR;
}
