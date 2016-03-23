/**
*
*ZebraStripes
*
* Date 17.03.2016
*
*/
 
 
#include "stdafx.h"
#include "ZebraStripes.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>
#include <cmath>

#define MAX_LENGTH 0.6


ADTF_FILTER_PLUGIN("ZebraStripes", OID_ADTF_ZebraStripes, cZebraStripes)

#define ZebraStripes_PROP_CAMERA_OFFSET "ZebraStripes::Camera Offset"
#define ZebraStripes_PROP_ThresholdValueCanny "ZebraStripes::ThresholdValueCanny"
#define ZebraStripes_PROP_ThresholdValueHough "ZebraStripes::ThresholdValueHough"
#define ZebraStripes_PROP_ImagecutWidthLeft "ZebraStripes::ImagecutWidthLeft"
#define ZebraStripes_PROP_ImagecutWidthRight "ZebraStripes::ImagecutWidthRight"
#define ZebraStripes_PROP_ImagecutHeightUp "ZebraStripes::ImagecutHeightUp"
#define ZebraStripes_PROP_ImagecutHeightDown "ZebraStripes::ImagecutHeightDown"


#define ZebraStripes_PROP_SHOW_DEBUG "Common::Show Debug"




#define MAX_FRAMES 400


//TODO 1
//#define LT_ENABLE_CANNY_WINDOWS


cZebraStripes::cZebraStripes(const tChar* __info) : cFilter(__info)
{
	RGBreceived 	   = false;


    
    SetPropertyBool(ZebraStripes_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(ZebraStripes_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");


	SetPropertyInt(ZebraStripes_PROP_ImagecutWidthLeft, 0);
	SetPropertyInt(ZebraStripes_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(ZebraStripes_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ZebraStripes_PROP_ImagecutWidthRight, 640);
	SetPropertyInt(ZebraStripes_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(ZebraStripes_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ZebraStripes_PROP_ImagecutHeightUp, 70);
	SetPropertyInt(ZebraStripes_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(ZebraStripes_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(ZebraStripes_PROP_ImagecutHeightDown, 255);
	SetPropertyInt(ZebraStripes_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(ZebraStripes_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");


    SetPropertyFloat(ZebraStripes_PROP_CAMERA_OFFSET, 15);
    SetPropertyBool(ZebraStripes_PROP_CAMERA_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ZebraStripes_PROP_CAMERA_OFFSET NSSUBPROP_DESCRIPTION, "The offset of the camera in relation to the center of the car.");

    SetPropertyFloat(ZebraStripes_PROP_ThresholdValueCanny, 120);
    SetPropertyBool(ZebraStripes_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ZebraStripes_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(ZebraStripes_PROP_ThresholdValueHough, 73);
    SetPropertyBool(ZebraStripes_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(ZebraStripes_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");
	
    


    m_pISignalRegistry = NULL;
}

cZebraStripes::~cZebraStripes()
{
}

tResult cZebraStripes::GetInterface(const tChar* idInterface,
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

tUInt cZebraStripes::Ref()
{
    return cFilter::Ref();
}

tUInt cZebraStripes::Unref()
{
    return cFilter::Unref();
}

tVoid cZebraStripes::Destroy()
{
    delete this;
}

tResult cZebraStripes::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cZebraStripes::Stop(__exception)
{   
    return cFilter::Stop(__exception_ptr);
}
tResult cZebraStripes::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// Get description for bool values
    	tChar const * strDescBoolValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
    	RETURN_IF_POINTER_NULL(strDescBoolValue);	
    	cObjectPtr<IMediaType> pTypeBoolValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for Zebra Stripes Struct
    	tChar const * strDescZebraStripes = pDescManager->GetMediaDescription("tZebraStripesStruct");   
    	RETURN_IF_POINTER_NULL(strDescZebraStripes);    
    	cObjectPtr<IMediaType> pTypeZebraStripes = new cMediaType(0, 0, 0, "tZebraStripesStruct", strDescZebraStripes, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));

		// Depthimage Input
        RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));

		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("Start", pTypeBoolValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartBoolInput)); 

		// Distance Input
		RETURN_IF_FAILED(m_iDistance.Create("Distance", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

		// Create Pin for DistanceToZebrastripes (Output)
		RETURN_IF_FAILED(m_oDistanceOutput.Create("DistanceToZebraStripes", pTypeZebraStripes, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oDistanceOutput));
		RETURN_IF_FAILED(pTypeZebraStripes->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistance)); 


              
    }
    else if (eStage == StageNormal)
    {
		m_bStartBoolInput = tFalse;
		m_bActive = tFalse;
		m_filterActive = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
		m_ZebraStripesOutputSet = tFalse;
		m_DistanceInputSet = tFalse;

        ReadProperties(NULL);
		m_iframeCounter = 0;

    	m_fDistance = 0.0;
		m_distanceOffset = 0.0;
        
        m_ZebraEmpty 	= tFalse;
        
        if (m_bShowDebug)
        {
        }

    }
    RETURN_NOERROR;
}

tResult cZebraStripes::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cZebraStripes::ReadProperties(const tChar* strPropertyName)
{	
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(ZebraStripes_PROP_SHOW_DEBUG);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ImagecutWidthLeft))
    {
        m_nImagecutWidthLeft = GetPropertyInt(ZebraStripes_PROP_ImagecutWidthLeft);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ImagecutWidthRight))
    {
        m_nImagecutWidthRight = GetPropertyInt(ZebraStripes_PROP_ImagecutWidthRight);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ImagecutHeightUp))
    {
        m_nImagecutHeightUp = GetPropertyInt(ZebraStripes_PROP_ImagecutHeightUp);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ImagecutHeightDown))
    {
        m_nImagecutHeightDown = GetPropertyInt(ZebraStripes_PROP_ImagecutHeightDown);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ThresholdValueHough))
    {
        m_nThresholdValueHough = GetPropertyInt(ZebraStripes_PROP_ThresholdValueHough);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, ZebraStripes_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(ZebraStripes_PROP_ThresholdValueCanny);
    }


    RETURN_NOERROR;
}

tResult cZebraStripes::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cZebraStripes::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cZebraStripes::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
                __adtf_sample_read_lock_mediadescription(m_pStartBoolInput,pMediaSample,pCoder);

				if (!m_bStartBoolInput) {
					pCoder->GetID("bValue", m_szIDValueStartBoolInput);
					m_bStartBoolInput = tTrue;
				}

                pCoder->Get(m_szIDValueStartBoolInput, (tVoid*)&bValue);	
            }

			m_bActive = bValue;
			m_filterActive = bValue;
        }

		if (pSource == &m_iDistance && m_filterActive){

			RETURN_IF_POINTER_NULL(pMediaSample);
			ProcessSpeed(pMediaSample);
		}

		}          
        
    RETURN_NOERROR;
}

tResult cZebraStripes::ProcessSpeed(IMediaSample* pMediaSample){
	
    m_fDistance = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);
		
		if(!m_DistanceInputSet)
		{
			pCoder->GetID("f32Value", m_szIDDistanceInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput); 
			m_DistanceInputSet=tTrue; 
	  	}
		pCoder->Get(m_szIDDistanceInput, (tVoid*)&(m_fDistance));
	}
	// calc Speed

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = m_fDistance;
	}
	
	m_fDistance -= m_distanceOffset;
	
	RETURN_NOERROR;
}

tResult cZebraStripes::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{ if (m_bActive){

			m_bActive = tFalse; // damit nur ein RGB und Tiefenbild empfangen wird
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


//----------------Preprocessing-------------------------//
//////////////// Line detection
		
		Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image 
        cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 		
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		threshold(m_matCutHough, m_matCutHough, 45, 255,THRESH_TOZERO); // delete dark noise	
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCanny,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges 
		m_lines.clear();
		HoughLines(m_matCanny,m_lines,1,CV_PI/180,m_nThresholdValueHough,0,0);


		RGBreceived = true;
	}
	RETURN_NOERROR;            
}

tResult cZebraStripes::ProcessInputDepth(IMediaSample* pSample, tTimeStamp tsInputTime)
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

	SearchCorners();
	CreateAndTransmitGCL();
	RGBreceived=false;	
	
	RETURN_NOERROR;            
}


tResult cZebraStripes::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cZebraStripes::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cZebraStripes::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cZebraStripes::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cZebraStripes::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

tResult cZebraStripes::SearchCorners()
{

	m_intersecPointsR.clear();
	m_intersecPointsL.clear();
	m_ZebraStripes.clear();
	


	//eliminate Houghline-cloud of Hough
	vector<Point2f> HoughGroupL;
	vector<Point2f> HoughGroupR;
	vector<Point2f> HoughGroupH;
	for(size_t i=0; i<m_lines.size();i++)
		{
			if((m_lines[i][1]*180/M_PI<150) && (m_lines[i][1]*180/M_PI>115))
			{
					HoughGroupR.push_back(Point2f(m_lines[i][0],m_lines[i][1]));		
			}
			else if((m_lines[i][1]*180/M_PI<95) && (m_lines[i][1]*180/M_PI>85))
			{
					HoughGroupH.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	
			}
			else if((m_lines[i][1]*180/M_PI<55) && (m_lines[i][1]*180/M_PI>35))
			{
					HoughGroupL.push_back(Point2f(m_lines[i][0],m_lines[i][1])); 	

			}
		}
if(HoughGroupH.size()<2 || (HoughGroupR.empty() && HoughGroupL.empty()))
{
	m_bActive = tTrue;
	RETURN_NOERROR;
}


//Calculate representer for HoughGroupR
if(!HoughGroupR.empty())
{
float averageHGR = 0;
m_representerHGR = HoughGroupR[0]; 
	for(size_t i=0; i<HoughGroupR.size();i++)
		{
			averageHGR = averageHGR + HoughGroupR[i].y*180/M_PI;
		}
			averageHGR = averageHGR/HoughGroupR.size();
	for(size_t i=0; i<HoughGroupR.size();i++)
		{
			if(fabs(m_representerHGR.y*180/M_PI-averageHGR)>fabs(HoughGroupR[i].y*180/M_PI-averageHGR))
			{
				m_representerHGR = HoughGroupR[i];
			}
		}
}

//Calculate representer for HoughGroupL
if(!HoughGroupL.empty())
{
float averageHGL = 0;
m_representerHGL = HoughGroupL[0]; 
	for(size_t i=0; i<HoughGroupL.size();i++)
		{
			averageHGL = averageHGL + HoughGroupL[i].y*180/M_PI;
		}
			averageHGL = averageHGL/HoughGroupL.size();

	for(size_t i=0; i<HoughGroupL.size();i++)
		{
			if(fabs(m_representerHGL.y*180/M_PI-averageHGL)>fabs(HoughGroupL[i].y*180/M_PI-averageHGL))
			{
				m_representerHGL = HoughGroupL[i];
			}
		}
}

//calculate intersection of representer horizontal hougline and representer on right side(crossing corner right front)

	if(!HoughGroupR.empty() &&  !HoughGroupH.empty())
	{
		Point2f pt1R,pt2R;

		size_t r=0;
		pt1R.x = cos(m_representerHGR.y)*m_representerHGR.x + 1000*(-sin(m_representerHGR.y));
		pt1R.y = sin(m_representerHGR.y)*m_representerHGR.x + 1000*(cos(m_representerHGR.y));
		pt2R.x = cos(m_representerHGR.y)*m_representerHGR.x - 1000*(-sin(m_representerHGR.y));
		pt2R.y = sin(m_representerHGR.y)*m_representerHGR.x - 1000*(cos(m_representerHGR.y));

		for(size_t i=0; i<HoughGroupH.size();i++)
		{
			Point2f pt1H,pt2H;
			pt1H.x = cos(HoughGroupH[i].y)*HoughGroupH[i].x + 1000*(-sin(HoughGroupH[i].y));
			pt1H.y = sin(HoughGroupH[i].y)*HoughGroupH[i].x + 1000*(cos(HoughGroupH[i].y));
			pt2H.x = cos(HoughGroupH[i].y)*HoughGroupH[i].x - 1000*(-sin(HoughGroupH[i].y));
			pt2H.y = sin(HoughGroupH[i].y)*HoughGroupH[i].x - 1000*(cos(HoughGroupH[i].y));

			if(pt1R.x == pt2R.x || pt1R.y == pt2R.y || pt1H.x == pt2H.x)
			{
			}
			else
			{
				float m1 = (pt2R.y-pt1R.y)/(pt2R.x-pt1R.x);
				float m3 = (pt2H.y-pt1H.y)/(pt2H.x-pt1H.x);
				double b1 = pt2R.y - m1*pt2R.x;
				double b3 = pt2H.y - m3*pt2H.x;

				int Pixelx1 = cvRound((b3-b1)/(m1-m3));
				int Pixely1= cvRound(m1*Pixelx1+b1);

				if(Pixelx1 > 0  && Pixely1 > 0 &&  Pixelx1 < (m_nImagecutWidthRight-m_nImagecutWidthLeft) && Pixely1 < (m_nImagecutHeightDown-m_nImagecutHeightUp))
				{
					m_intersecPointsR.push_back(Point2f(Pixelx1,Pixely1));
					r++;
				}
			}
		}
	}
	else if(HoughGroupR.empty() && !HoughGroupL.empty() && !HoughGroupH.empty())
	{
	//calculate intersection of representer horizontal hougline and representer on right side(crossing corner right front)
		Point2f pt1R,pt2R,pt1L,pt2L;
		size_t l=0;
		pt1L.x = cos(m_representerHGL.y)*m_representerHGL.x + 1000*(-sin(m_representerHGL.y));
		pt1L.y = sin(m_representerHGL.y)*m_representerHGL.x + 1000*(cos(m_representerHGL.y));
		pt2L.x = cos(m_representerHGL.y)*m_representerHGL.x - 1000*(-sin(m_representerHGL.y));
		pt2L.y = sin(m_representerHGL.y)*m_representerHGL.x - 1000*(cos(m_representerHGL.y));

		for(size_t i=0; i<HoughGroupH.size();i++)
		{
			Point2f pt1H,pt2H;
			pt1H.x = cos(HoughGroupH[i].y)*HoughGroupH[i].x + 1000*(-sin(HoughGroupH[i].y));
			pt1H.y = sin(HoughGroupH[i].y)*HoughGroupH[i].x + 1000*(cos(HoughGroupH[i].y));
			pt2H.x = cos(HoughGroupH[i].y)*HoughGroupH[i].x - 1000*(-sin(HoughGroupH[i].y));
			pt2H.y = sin(HoughGroupH[i].y)*HoughGroupH[i].x - 1000*(cos(HoughGroupH[i].y));


			if( pt1L.x == pt2L.x || pt1L.y == pt2L.y || pt1H.x == pt2H.x)
			{
			}
			else
			{
				float m2 = (pt2L.y-pt1L.y)/(pt2L.x-pt1L.x);
				float m3 = (pt2H.y-pt1H.y)/(pt2H.x-pt1H.x);
				double b2 = pt2L.y - m2*pt2L.x;
				double b3 = pt2H.y - m3*pt2H.x;
				int Pixelx2 = cvRound((b3-b2)/(m2-m3));
				int Pixely2 = cvRound(m2*Pixelx2+b2);

				if(Pixelx2 > 0  && Pixely2 > 0  && Pixelx2 < (m_nImagecutWidthRight-m_nImagecutWidthLeft) && Pixely2 < (m_nImagecutHeightDown-m_nImagecutHeightUp))
				{
					m_intersecPointsL.push_back(Point2f(Pixelx2,Pixely2));
					l++;
				}
			}
		}
	}

//skip bad intersectionpoints and use proportion of zebrastripes to decide which intersectionpoints of the midline might be useful 
	for(size_t i=0;i<m_intersecPointsL.size();i++)
		{
			Point3f corner1 = Util::ComputeWorldCoordinate(m_intersecPointsL[i].x, m_intersecPointsL[i].y,m_matImageDepth.at<ushort>(floor(m_intersecPointsL[i].y*0.5),floor(m_intersecPointsL[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);	
			
			if (corner1.z > 0.2 && corner1.z<1.5) m_ZebraStripes.push_back(corner1.z);		
/*		for(size_t j=i+1;j<m_intersecPointsL.size();j++)
			{
			 	Point3f corner2 = Util::ComputeWorldCoordinate(m_intersecPointsL[j].x, m_intersecPointsL[j].y,m_matImageDepth.at<ushort>(floor(m_intersecPointsL[j].y*0.5),floor(m_intersecPointsL[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);			
				
				if(corner1.z>2.5 || corner1.z<0.4 || corner2.z>2.5 || corner2.z<0.4)continue;
				if(fabs(corner1.z-corner2.z)<MAX_LENGTH && fabs(corner1.z-corner2.z)>0.25)
					{
						m_ZebraStripes.push_back(corner1.z);
						m_ZebraStripes.push_back(corner2.z);
					}
			}*/
		}

//skip bad intersectionpoints and use proportion of zebrastripes to decide which intersectionpoints of the rightline might be useful 
	for(size_t i=0;i<m_intersecPointsR.size();i++)
		{
			Point3f corner1 = Util::ComputeWorldCoordinate(m_intersecPointsR[i].x, m_intersecPointsR[i].y,m_matImageDepth.at<ushort>(floor(m_intersecPointsR[i].y*0.5),floor(m_intersecPointsR[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);	

			if (corner1.z > 0.2 && corner1.z<1.5) m_ZebraStripes.push_back(corner1.z);		
			/*for(size_t j=i+1;j<m_intersecPointsR.size();j++)
			{
			 	Point3f corner2 = Util::ComputeWorldCoordinate(m_intersecPointsR[j].x, m_intersecPointsR[j].y,m_matImageDepth.at<ushort>(floor(m_intersecPointsR[j].y*0.5),floor(m_intersecPointsR[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);			
				//LOG_INFO(cString::Format("Zebra:corner1.z:%f, corner2.z:%f",corner1.z,corner2.z) );
				if(corner1.z>2.5 || corner1.z<0.4 || corner2.z>2.5 || corner2.z<0.4)continue;
				if(fabs(corner1.z-corner2.z)<MAX_LENGTH && fabs(corner1.z-corner2.z)>0.25)
					{
						m_ZebraStripes.push_back(corner1.z);
						m_ZebraStripes.push_back(corner2.z);
					}
			}*/
		}

if(m_ZebraStripes.empty())
{
	//LOG_WARNING(cString::Format("Zebra: Nothing found!"));
	m_ZebraEmpty 	= tTrue;
	m_bActive = tTrue;
	RETURN_NOERROR;
}
	m_ZebraEmpty 	= tFalse;
	sort(m_ZebraStripes.begin(),m_ZebraStripes.end());
	/*size_t index=m_ZebraStripes.size()-1;
	while(index >=0 && fabs(m_ZebraStripes[index]-m_ZebraStripes[0])>MAX_LENGTH){ // Same value as above!
	index--;}*/

	if(m_iframeCounter>MAX_FRAMES)
	{
			//LOG_ERROR(cString::Format("Zebra: Timeout!") );
			m_iframeCounter = 0;
			RETURN_NOERROR;
	}
	else
	{
		// Use buffer in order to have a more robust decision
		if(m_bufZebraNear.size()<5)
		{
			m_bufZebraNear.push_back(m_ZebraStripes[0]+m_fDistance);
			LOG_INFO(cString::Format("m_ZebraStripes[0]+m_fDistance = %f + %f = %f", m_ZebraStripes[0],m_fDistance, m_ZebraStripes[0]+m_fDistance) );
			//m_bufZebraFar.push_back(m_ZebraStripes[index]+m_fDistance);
			m_bActive = tTrue;

		} else {
			/*vector<int> votesZebraNear;
			//int votesZebraNear [m_bufZebraNear.size()];
			for(size_t i=0;i<m_bufZebraNear.size();i++)
			{
				votesZebraNear.push_back(0);
			}
			for(size_t i=0;i<m_bufZebraNear.size();i++){
				for(size_t j=0;j<m_bufZebraNear.size();j++){
					if(fabs(m_bufZebraNear[i]-m_bufZebraNear[j])<0.03){
						votesZebraNear[i]++;
					}
				}
			}

			//LOG_INFO(cString::Format("Zebra: Searching %i %i %i %i %i ", votesZebraNear[0],votesZebraNear[1], votesZebraNear[2],votesZebraNear[3],votesZebraNear[4]) );
			//int votesZebraFar [m_bufZebraFar.size()];
			vector<int> votesZebraFar;
			for(size_t i=0;i<m_bufZebraFar.size();i++)
			{
				votesZebraFar.push_back(0);
			}
			for(size_t i=0;i<m_bufZebraFar.size();i++){
				for(size_t j=0;j<m_bufZebraFar.size();j++){
					if(fabs(m_bufZebraFar[i]-m_bufZebraFar[j])<0.03){
						votesZebraFar[i]++;
					}
				}
			}

			// find Zebra Points with most votes
			int idxNear = 0;
			int maxNear = 0;
			for(size_t i=0; i<m_bufZebraNear.size();i++)
			{
				if(votesZebraNear[i]>maxNear)
				{
					idxNear = i;
					maxNear = votesZebraNear[i];
				}
			}

			int idxFar = 0;
			int maxFar = 0;
			for(size_t i=0; i<m_bufZebraFar.size();i++)
			{
				if(votesZebraFar[i]>maxFar)
				{
					idxFar = i;
					maxFar = votesZebraFar[i];
				}
			}

			// check for min number of occurences of the chosen points
			fstream f;
			f.open("ZebraVotes",ios::out|ios::app);
			f<<maxNear << "\t" << maxFar << "\n";
			f.close();*/
/*			if(maxNear>=4 && maxFar>=4){
				TransmitZebraSpot(m_bufZebraNear[idxNear]-m_fDistance,m_bufZebraFar[idxFar]-m_fDistance);
				m_iframeCounter = 0;*/

				
			// Average of found points:
			float average 	= 0;
			for (size_t i = 0; i<m_bufZebraNear.size(); i++){
				average	+= m_bufZebraNear.at(i);
			}
			average /= m_bufZebraNear.size();

			TransmitZebraSpot(average-m_fDistance,average-m_fDistance+0.4);
			m_iframeCounter = 0;

			/*} else {
				// Try again if failed
				m_bufZebraNear.push_back(m_ZebraStripes[0]+m_fDistance);
				m_bufZebraFar.push_back(m_ZebraStripes[index]+m_fDistance);
				m_bActive = tTrue;
			}*/
		}
		
	}
	//LOG_INFO(cString::Format("IP: Finished Searching") );
	RETURN_NOERROR;
}


tResult cZebraStripes::CreateAndTransmitGCL()
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

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,255).GetRGBA());	
		
	for(size_t i=0; i < m_intersecPointsR.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_intersecPointsR[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_intersecPointsR[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_intersecPointsR[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_intersecPointsR[i].y+2) + m_nImagecutHeightUp);
		}

	for(size_t i=0; i < m_intersecPointsL.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_intersecPointsL[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_intersecPointsL[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_intersecPointsL[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_intersecPointsL[i].y+2) + m_nImagecutHeightUp);
		}

	for(size_t i=0; i < m_lines.size(); i++) 
		{
			if((m_lines[i][1]*180/M_PI<150) && (m_lines[i][1]*180/M_PI>115))
			{
						cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
						float rho = m_lines[i][0];
						float theta = m_lines[i][1];
						Point pt1,pt2;
						double a = cos(theta), b = sin(theta);
						double x0 = a*rho, y0 = b*rho;
						pt1.x = cvRound(x0 + 1000*(-b));
						pt1.y = cvRound(y0 + 1000*(a));
						pt2.x = cvRound(x0 - 1000*(-b));
						pt2.y = cvRound(y0 - 1000*(a)); 
						cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUp, pt1.x, pt1.y+m_nImagecutHeightUp);
			}
			else if((m_lines[i][1]*180/M_PI<95) && (m_lines[i][1]*180/M_PI>85))
			{
						cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,255,0).GetRGBA());
						float rho = m_lines[i][0];
						float theta = m_lines[i][1];
						Point pt1,pt2;
						double a = cos(theta), b = sin(theta);
						double x0 = a*rho, y0 = b*rho;
						pt1.x = cvRound(x0 + 1000*(-b));
						pt1.y = cvRound(y0 + 1000*(a));
						pt2.x = cvRound(x0 - 1000*(-b));
						pt2.y = cvRound(y0 - 1000*(a)); 
						cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUp, pt1.x, pt1.y+m_nImagecutHeightUp);
			}
			else if((m_lines[i][1]*180/M_PI<45) && (m_lines[i][1]*180/M_PI>35))
			{
						cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,0,255).GetRGBA());
						float rho = m_lines[i][0];
						float theta = m_lines[i][1];
						Point pt1,pt2;
						double a = cos(theta), b = sin(theta);
						double x0 = a*rho, y0 = b*rho;
						pt1.x = cvRound(x0 + 1000*(-b));
						pt1.y = cvRound(y0 + 1000*(a));
						pt2.x = cvRound(x0 - 1000*(-b));
						pt2.y = cvRound(y0 - 1000*(a)); 
						cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x, pt2.y+m_nImagecutHeightUp, pt1.x, pt1.y+m_nImagecutHeightUp);
			}
		}

	cString strText2 = cString::FromBool(m_ZebraEmpty);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText2.GetLength());
	cGCLWriter::StoreData(pc, strText2.GetLength(), strText2.GetPtr()); 



    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

tResult cZebraStripes::TransmitZebraSpot(float spot1,float spot2) {

		tUInt32 nTimeStamp = 0;

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDistance->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


		{
			__adtf_sample_write_lock_mediadescription(m_pDistance, pMediaSample, pCoderOutput); 
	
			if(!m_ZebraStripesOutputSet) {
				pCoderOutput->GetID("neardist", m_sznearDistance);
				pCoderOutput->GetID("fardist", m_szfarDistance);
				pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampOutput);
				m_ZebraStripesOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_sznearDistance, (tVoid*)&(spot1));
			pCoderOutput->Set(m_szfarDistance, (tVoid*)&(spot2));
			pCoderOutput->Set(m_szIDTimestampOutput, (tVoid*)&nTimeStamp);
		
		}

		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceOutput.Transmit(pMediaSample);

		// Prepare filter for next activation
		m_distanceOffset = 0.0;
		m_bufZebraNear.clear();
		m_bufZebraFar.clear();
		m_filterActive = tFalse;
		// TODO fragen!!
		m_bActive = tFalse;

	RETURN_NOERROR;
}
