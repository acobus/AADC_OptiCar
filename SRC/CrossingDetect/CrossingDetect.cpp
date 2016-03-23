/**
*
*CrossingDetect
*
* Date 17.03.2016
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

#define CrossingDetect_PROP_TRESHOLD "CrossingDetect::ThresholdValue"
#define CrossingDetect_PROP_ThresholdValueCanny "CrossingDetect::ThresholdValueCanny"
#define CrossingDetect_PROP_ThresholdValueHough "CrossingDetect::ThresholdValueHough"
#define CrossingDetect_PROP_ThresholdValueHoughSMALL "CrossingDetect::ThresholdValueHoughSMALL"
#define CrossingDetect_PROP_TRESHOLD2 "CrossingDetect::ThresholdValue2"
#define CrossingDetect_PROP_BEFORENORM "CrossingDetect::ThresholdBeforeNorm"
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

#define CrossingDetect_PROP_TRESHOLD2CIRCLE "CrossingDetect::ThresholdValue2Circle"
#define CrossingDetect_PROP_TRESHOLDCIRCLE "CrossingDetect::ThresholdValueCircle"
#define CrossingDetect_PROP_TRESHOLDBEFORECIRCLE "CrossingDetect::ThresholdValueBeforeNormCircle"
#define CrossingDetect_PROP_ThresholdValueCannyCIRCLE "CrossingDetect::ThresholdValueCannyCircle"
#define CrossingDetect_PROP_ThresholdValueHoughCIRCLE "CrossingDetect::ThresholdValueHoughCircle"

#define CrossingDetect_PROP_SHOW_DEBUG "Common::Show Debug"


#define MAX_FRAMES 400
#define MIN_FRAMES 2
#define MAX_DEVIATION 150
#define CROSS_POINT_OFFSET 0.10
#define CROSS_HOUGH_OFFSET 0.10
//-0.35

#define CIRCLE_OFFSET 0.05

#define WORKING_TIME_MAX 5.0


cCrossingDetect::cCrossingDetect(const tChar* __info) : cFilter(__info)
{

    SetPropertyBool(CrossingDetect_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(CrossingDetect_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");


	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeft, 17);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRight, 630);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUp, 70);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDown, 245);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeftHough, 0);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthLeftHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthLeftHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRightHough, 320);
	SetPropertyInt(CrossingDetect_PROP_ImagecutWidthRightHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutWidthRightHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough... both Left and Right -> max Val 320");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUpHough, 70);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightUpHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightUpHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDownHough, 245);
	SetPropertyInt(CrossingDetect_PROP_ImagecutHeightDownHough NSSUBPROP_MIN, 0);
    SetPropertyStr(CrossingDetect_PROP_ImagecutHeightDownHough NSSUBPROP_DESCRIPTION, "Cuts the Image for Hough...");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueCanny, 150);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueHough, 80);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");

    SetPropertyInt(CrossingDetect_PROP_ThresholdValueHoughSMALL, 35);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueHoughSMALL NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueHoughSMALL NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");
	
    SetPropertyFloat(CrossingDetect_PROP_TRESHOLD, 50);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD NSSUBPROP_DESCRIPTION, "The first threshold value.");

	SetPropertyFloat(CrossingDetect_PROP_TRESHOLD2, 120);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD2 NSSUBPROP_DESCRIPTION, "The third threshold value.");

	SetPropertyInt(CrossingDetect_PROP_BEFORENORM, 40);
    SetPropertyBool(CrossingDetect_PROP_BEFORENORM NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_BEFORENORM NSSUBPROP_DESCRIPTION, "The threshold value (2nd) before the normalization.");

	SetPropertyInt(CrossingDetect_PROP_CornerHarrisblockSize, 5);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisblockSize NSSUBPROP_DESCRIPTION, "Harris Neighborhood size");

	SetPropertyInt(CrossingDetect_PROP_CornerHarrisksize, 11);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisksize NSSUBPROP_DESCRIPTION, "Harris aperture parameter for the sobel operator");

	SetPropertyFloat(CrossingDetect_PROP_CornerHarrisparamK, 0.08);
    SetPropertyBool(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_CornerHarrisparamK NSSUBPROP_DESCRIPTION, "Harris detectos free parameter k");


	SetPropertyFloat(CrossingDetect_PROP_TRESHOLD2CIRCLE, 150);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLD2CIRCLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLD2CIRCLE NSSUBPROP_DESCRIPTION, "The second threshold value.");

    SetPropertyInt(CrossingDetect_PROP_TRESHOLDBEFORECIRCLE, 40);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLDBEFORECIRCLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLDBEFORECIRCLE NSSUBPROP_DESCRIPTION, "The threshold value (2nd) before the normalization.");

    SetPropertyFloat(CrossingDetect_PROP_TRESHOLDCIRCLE, 50);
    SetPropertyBool(CrossingDetect_PROP_TRESHOLDCIRCLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_TRESHOLDCIRCLE NSSUBPROP_DESCRIPTION, "The first threshold value.");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueCannyCIRCLE, 150);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueCannyCIRCLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueCannyCIRCLE NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(CrossingDetect_PROP_ThresholdValueHoughCIRCLE, 90);
    SetPropertyBool(CrossingDetect_PROP_ThresholdValueHoughCIRCLE NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CrossingDetect_PROP_ThresholdValueHoughCIRCLE NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");
    


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
		InitialState();
		m_bStartBoolIDInput = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
		m_crossingSpotOutputSet = tFalse;


        ReadProperties(NULL);

       // m_ui8InitCtrl = 0;

		m_iframeCounter = 0;

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

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_BEFORENORM))
    {
        m_nThresholdValueBeforeNorm = GetPropertyInt(CrossingDetect_PROP_BEFORENORM);
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

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueHoughSMALL))
    {
        m_nThresholdValueHoughSMALL = GetPropertyInt(CrossingDetect_PROP_ThresholdValueHoughSMALL);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(CrossingDetect_PROP_ThresholdValueCanny);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueCannyCIRCLE))
    {
        m_nThresholdValueCannyCIRCLE = GetPropertyInt(CrossingDetect_PROP_ThresholdValueCannyCIRCLE);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLDCIRCLE))
    {
        m_nThresholdValueCIRCLE = GetPropertyInt(CrossingDetect_PROP_TRESHOLDCIRCLE);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLDBEFORECIRCLE))
    {
        m_nThresholdValueBeforeNormCIRCLE = GetPropertyInt(CrossingDetect_PROP_TRESHOLDBEFORECIRCLE);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_TRESHOLD2CIRCLE))
    {
        m_nThresholdValue2CIRCLE = GetPropertyInt(CrossingDetect_PROP_TRESHOLD2CIRCLE);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CrossingDetect_PROP_ThresholdValueHoughCIRCLE))
    {
        m_nThresholdValueHoughCIRCLE = GetPropertyInt(CrossingDetect_PROP_ThresholdValueHoughCIRCLE);
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
			tBool bisCircle = tFalse;
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pStartBoolIDInput,pMediaSample,pCoder);

				if (!m_bStartBoolIDInput) {
					pCoder->GetID("bStart", m_szIDValueStartBoolIDInput);
					pCoder->GetID("bIsCircleMode", m_szTrafficCircleBoolInput);
					pCoder->GetID("ID", m_szIDDRIVING);
					m_bStartBoolIDInput = tTrue;
				}

                pCoder->Get(m_szIDValueStartBoolIDInput, (tVoid*)&bValue);	
				pCoder->Get(m_szTrafficCircleBoolInput, (tVoid*)&bisCircle);
				pCoder->Get(m_szIDDRIVING, (tVoid*)&m_DRIVING_ID);
            }


			if (!bValue){
				LOG_INFO(cString::Format("CD: Shutdown Cross Detection") );	
				m_shutdown = tTrue;			
			} else{
				LOG_INFO(cString::Format("CD: Start Cross Detection with Status: %i, ID: %i, IsCircle: %i", bValue, m_DRIVING_ID, bisCircle) );
				InitialState();	
				m_activationTime 	= _clock->GetStreamTime();
			}
			m_bActive = bValue;
			m_BoolTrafficCircle=bisCircle;
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}

tResult cCrossingDetect::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{ 
	if (m_bActive){
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
		m_representerHLGC = Point2f(0,0);
		PixelSpot.clear();



//----------------Preprocessing-------------------------//
///////////////// Corner detection 
		
		if(!m_BoolTrafficCircle)	
		{
			Mat m_matImage = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image
			cvtColor(m_matImage, m_matImage, CV_RGB2GRAY);// Grey Image
			threshold(m_matImage, m_matImage, m_nThresholdValue, 255,THRESH_TOZERO);
			m_matImage=m_matImage-m_nThresholdValue;
			medianBlur(m_matImage, m_matImage, 3); // reduce noise with edge-preserving filter
			/*fstream f2;
			f2.open("median.dat",ios::out);
			f2 << m_matCorner<< "\n";
			f2.close();*/
			cornerHarris(m_matImage, m_matImage, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			/*fstream f6;
			f6.open("corner.dat",ios::out);
			f6 << m_matImage<< "\n";
			f6.close();*/
			threshold(m_matImage, m_matImage, 0.01, 255,THRESH_TOZERO); // in case of no real corners set everything to zero - otherwise everywhere corners because of normalize
			threshold(m_matImage, m_matImage, m_nThresholdValueBeforeNorm, 255,THRESH_TRUNC);
			/*fstream f1;
			f1.open("BeforeNormalized.dat",ios::out);
			f1 << m_matImage<< "\n";
			f1.close();*/
			normalize(m_matImage, m_matNormalized, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
			convertScaleAbs(m_matNormalized, m_matNormalized); // Konveriere in 8Bit array. Nehme aus abs
			/*fstream f8;
			f8.open("scaled.dat",ios::out);
			f8 << m_matNormalized<< "\n";
			f8.close();*/
			threshold(m_matNormalized, m_matThres2,  m_nThresholdValue2, 255,THRESH_BINARY); 
			/*fstream f2;
			f2.open("canny.dat",ios::out);
			f2 << m_matThres2<< "\n";
			f2.close();*/

			//////////////// Line detection Left
		
			if(m_nImagecutWidthRightHough > 320)
			{
				LOG_ERROR(cString::Format("CrossDetection: Left Window of Hough detect is to bride") );
				RETURN_NOERROR; 
			}
			m_iImageHalfWidth = floor(image.size().width*0.5);
			Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_nImagecutWidthLeftHough, m_nImagecutWidthRightHough)).clone(); //Cut Image 
		    cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 	
			threshold(m_matCutHough, m_matCutHough, 50, 255,THRESH_TOZERO); // delete dark noise
			m_matCutHough=m_matCutHough-50;
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		    medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
		    Canny(m_matCutHough, m_matCannyLeft,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges 
			m_linesL.clear();
			HoughLines(m_matCannyLeft,m_linesL,1,CV_PI/180,30,0,0);
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
			threshold(m_matCutHough, m_matCutHough, 50, 255,THRESH_TOZERO);
			m_matCutHough=m_matCutHough-50;		
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
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
			/*fstream f2;
			f2.open("color.dat",ios::out);
			f2 << m_matCutHough<< "\n";
			f2.close();*/
			threshold(m_matCutHough, m_matCutHough, 15, 255,THRESH_TOZERO);
			m_matCutHough=m_matCutHough-15;
			/*fstream f3;
			f3.open("thres.dat",ios::out);
			f3 << m_matCutHough<< "\n";
			f3.close();*/
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		    medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
		    Canny(m_matCutHough, m_matCannyRight,m_nThresholdValueCanny-20, m_nThresholdValueCanny, 3, false);// Detect Edges ---------Nur Canny "funktioniert"-------		
			/*fstream f4;
			f4.open("canny.dat",ios::out);
			f4 << m_matCannyRight<< "\n";
			f4.close();
			m_lines.clear();*/
			HoughLines(m_matCannyRight,m_lines,1,CV_PI/180,300,0,0);
			for(size_t i=0;i<m_lines.size();i++)
				{
					if(m_lines[i][1]*180/M_PI<105 && m_lines[i][1]*180/M_PI>75 && m_lines[i][0]<25)
						{
							m_DeadEndFrontBool = tTrue;
							break;
						}
				}

			/////////////////////// Line detect front right(small lines in front of car)
			m_matCutHough = image(cv::Range(m_nImagecutHeightUpHough, m_nImagecutHeightDownHough), cv::Range(m_iImageHalfWidth+m_nImagecutWidthLeftHough, m_iImageHalfWidth+m_nImagecutWidthRightHough)).clone(); //Cut Image    
		    cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 
			/*fstream f2;
			f2.open("color.dat",ios::out);
			f2 << m_matCutHough<< "\n";
			f2.close();*/
			threshold(m_matCutHough, m_matCutHough, 50, 255,THRESH_TOZERO);
			m_matCutHough=m_matCutHough-50;
			/*fstream f3;
			f3.open("thres.dat",ios::out);
			f3 << m_matCutHough<< "\n";
			f3.close();*/
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
		    medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
		    Canny(m_matCutHough, m_matCannyRight,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges
			m_linesRSmall.clear();
			HoughLines(m_matCannyRight,m_linesRSmall,1,CV_PI/180,m_nThresholdValueHoughSMALL,0,0);
		}
		else
		{
			// detect houghlines
			Mat m_matCutHough = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image 
		    cvtColor(m_matCutHough, m_matCutHough ,CV_RGB2GRAY);// Grey Image 
			threshold(m_matCutHough, m_matCutHough, 50, 255,THRESH_TOZERO); // delete dark noise	
			m_matCutHough=m_matCutHough-50;
			normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat()); // normalize (helps to compensate different illumination)
			convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. 
		    medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
		    Canny(m_matCutHough, m_matCutHough,m_nThresholdValueCannyCIRCLE, m_nThresholdValueCannyCIRCLE, 3, false);
			m_linesCIRCLE.clear();
			HoughLines(m_matCutHough,m_linesCIRCLE,1,CV_PI/180,m_nThresholdValueHoughCIRCLE,0,0);

			m_matCutHough = image(cv::Range(m_nImagecutHeightUp, m_nImagecutHeightDown), cv::Range(m_nImagecutWidthLeft, m_nImagecutWidthRight)).clone(); //Cut Image 
			cvtColor(m_matCutHough, m_matCutHough, CV_RGB2GRAY);// Grey Image
			threshold(m_matCutHough, m_matCutHough, m_nThresholdValueCIRCLE, 255,THRESH_TOZERO);
			m_matCutHough=m_matCutHough-m_nThresholdValue;
			medianBlur(m_matCutHough, m_matCutHough, 3); // reduce noise with edge-preserving filter
			/*fstream f2;
			f2.open("median.dat",ios::out);
			f2 << m_matCutHough<< "\n";
			f2.close();*/
			cornerHarris(m_matCutHough, m_matCorner, m_nCornerHarrisblockSize, m_nCornerHarrisksize, m_nCornerHarrisparamK, BORDER_DEFAULT);// preprocess corners
			/*fstream f6;
			f6.open("corner.dat",ios::out);
			f6 << m_matCutHough<< "\n";
			f6.close();*/
			threshold(m_matCorner, m_matThres, 0.01, 255,THRESH_TOZERO); // in case of no real corners set everything to zero - otherwise everywhere corners because of normalize
			threshold(m_matThres, m_matThres, m_nThresholdValueBeforeNormCIRCLE, 255,THRESH_TRUNC);
			/*fstream f1;
			f1.open("BeforeNormalized.dat",ios::out);
			f1 << m_matThres<< "\n";
			f1.close();*/
			normalize(m_matThres, m_matNormalized, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
			convertScaleAbs(m_matNormalized, m_matNormalized); // Konveriere in 8Bit array. Nehme aus abs
			/*fstream f8;
			f8.open("scaled.dat",ios::out);
			f8 << m_matNormalized<< "\n";
			f8.close();*/
			threshold(m_matNormalized, m_matThres2, m_nThresholdValue2CIRCLE, 255,THRESH_BINARY); 
		/*	fstream f2;
			f2.open("canny.dat",ios::out);
			f2 << m_matThres2<< "\n";
			f2.close();*/
		}
			
		RGBreceived = true;
	}
 
	RETURN_NOERROR;            
}

tResult cCrossingDetect::ProcessInputDepth(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	if(RGBreceived){ 

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
		if(!m_BoolTrafficCircle)SearchCorners();
		else ProcessCircle();
		CreateAndTransmitGCL();
		RGBreceived=false;
	}	
	
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
	m_Points.clear(); 
	
	// Write corners from Matrix to vector
    for(tInt nColumn=4; nColumn < szImageCutThresSize.width-4; nColumn++)
    {  
    	for(tInt nRow=4; nRow < szImageCutThresSize.height-4; nRow++)
    		{  
                 
        	if(m_matThres2.at<uchar>(nRow, nColumn)==255) 
        		{ 
				m_Points.push_back(Point2f(nColumn,nRow));         
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
		for(size_t i=0;i<m_Points.size();i++)
			{	
			corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5),floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			for(size_t j=i+1;j<m_Points.size();j++)
				{
					corner2 = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
					if(norm(corner1-corner2)<0.085)
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


	if(!killUs.empty())
	{
	sort(killUs.begin(),killUs.end(),greater<int>());
	for(size_t i=0;i<killUs.size();i++)
		{
		m_Points.erase(m_Points.begin()+killUs[i]);
		}
	}
		// Search actual crossing
		for(size_t i=0;i<m_Points.size();i++)
		{		
			// Check if we got valid points!
			if (m_Points[i].y*0.5 < 0 || m_Points[i].y*0.5 > (m_nImagecutHeightDown-m_nImagecutHeightUp)/2 || m_Points[i].x*0.5 < 0 || m_Points[i].x*0.5 > (m_nImagecutWidthRight-m_nImagecutWidthLeft)/2 || isnan(m_Points[i].x) || isnan(m_Points[i].y) ) continue;

			/*fstream fff;
			fff.open("CDErrorDetect2", ios::out|ios::app);	
			fff << m_Points[i].y*0.5 << "\t" << m_Points[i].x*0.5 << "\n";
			fff.close();*/

			corner1 = Util::ComputeWorldCoordinate(m_Points[i].x, m_Points[i].y,m_matImageDepth.at<ushort>(floor(m_Points[i].y*0.5),floor(m_Points[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
		 //	for(size_t j=i+1;j<m_Points.size();j++)
		//	{
			//	corner2 = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
				if( corner1.y>0.19 && corner1.y<0.23  && corner1.z < 2){ //&& corner2.z <2 && corner2.y>0.19 && corner2.y<0.23 ){				
				// Test if m_Points are valid
					PixelSpot.push_back(Point2f(m_Points[i].y,m_Points[i].x));
					//PixelSpot.push_back(Point2f(m_Points[j].y,m_Points[j].x));
					CrossPoint.push_back(corner1);
			//		CrossPoint.push_back(corner2);
				}
		//	}
		}


	for (size_t i = 0; i<30; i++){
	
		if (i < CrossPoint.size()) m_GCLCoordinates[i] = CrossPoint.at(i);

		else m_GCLCoordinates[i] = Point3f(0.0,0.0,0.0);
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

if(HoughLGroup2.empty() && !m_DeadEndFrontBool && ACTION_LEFT!=m_DRIVING_ID) m_DeadEndLeftBool = tTrue; // && !m_LefthorizontalHough 
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
m_linesRSmallLane.clear();
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

	for(size_t i=0;i<m_linesRSmall.size();i++)
		{
			if(m_linesRSmall[i].y*180/M_PI>100 || m_linesRSmall[i].y*180/M_PI<80)
				{
					if(m_linesRSmall[i].y*180/M_PI>115 && m_linesRSmall[i].y*180/M_PI<150)
						{
							m_linesRSmallLane.push_back(m_linesRSmall[i]);
						}
					m_linesRSmall.erase(m_linesRSmall.begin() + i);
					i--;
				}
		}
//LOG_INFO(cString::Format("Abbiegecheck: %i | %i | %i | %i", HoughRGroup2.empty(), !m_RighthorizontalHough, !m_DeadEndFrontBool, ACTION_RIGHT!=m_DRIVING_ID) );
if(m_linesRSmall.empty() && !m_DeadEndFrontBool && ACTION_RIGHT!=m_DRIVING_ID) m_DeadEndRightBool = tTrue; //&& !m_RighthorizontalHough
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
	float averageHRG2radius = 0;
	float averageHRG2angle = 0;
	if (HoughRGroup2.size()==1){
		averageHRG2radius 	= HoughRGroup2[0].x;
		averageHRG2angle	= HoughRGroup2[0].y;
	} else{
		linesBig.clear();
		for(size_t i=0;i<HoughRGroup2.size();i++)
		{
			linesBig.push_back(linesStruct(HoughRGroup2[i].x,HoughRGroup2[i].y));
		}
		sort(linesBig.begin(),linesBig.end());

		int sizecounter = 0;
		for(size_t i=linesBig.size()-1;i>0;i--)
		{
			if(fabs(linesBig[i].radius-linesBig[i-1].radius)<9)
			{
				averageHRG2angle+=linesBig[i].angle;
				averageHRG2radius+=linesBig[i].radius;
				sizecounter++;
			}
			else break;
		}
		averageHRG2angle/=sizecounter;
		averageHRG2radius/=sizecounter;
	}
	
	m_representerHRG2 = Point2f(averageHRG2radius,averageHRG2angle);
}

//Calculate representer for HoughRGroupSmall
	if(!m_linesRSmall.empty())
	{
	/*
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
	/*
	for(size_t i=1; i<averageGroup.size(); i++)
	{
		if(abs(averageGroup[i].y-averageAngle)<abs(averageAngle-m_representerHRGS.y))
		{
			m_representerHRGS = averageGroup[i];
		}
	}
	*/

	lines.clear();
	float averageHRGSradius = 0;
	float averageHRGSangle = 0;

	int sizecounter = 0;
	
	if (m_linesRSmall.size() == 1){
		averageHRGSradius 	= m_linesRSmall[0].x;
		averageHRGSangle 	= m_linesRSmall[0].y;
	} else{
		for(size_t i=0;i<m_linesRSmall.size();i++)
		{
			lines.push_back(linesStruct(m_linesRSmall[i].x,m_linesRSmall[i].y));
		}
		sort(lines.begin(),lines.end());


		for(size_t i=lines.size()-1;i>0;i--)
		{
			if(fabs(lines[i].radius-lines[i-1].radius)<9)
			{
				averageHRGSangle+=lines[i].angle;
				averageHRGSradius+=lines[i].radius;
				sizecounter++;
			}
			else break;
		}
		averageHRGSangle/=sizecounter;
		averageHRGSradius/=sizecounter;
	}
	
	m_representerHRGS = Point2f(averageHRGSradius,averageHRGSangle);

	if(m_linesRSmall.size() > 1 && sizecounter == 0) m_representerHRGS=Point2f(lines[0].radius, lines[0].angle); // Fehler: Abbiegen ohne horizontale Linie
	}
//Calculate representer for HoughRGroup1SmallLane
	if(!m_linesRSmallLane.empty())
	{
		float averageHRG1SmallLane = 0;
		float averageHGR1SmallLaneRadius 	= 0;
		m_representerHRGSLane = m_linesRSmallLane[0]; 
		for(size_t i=0; i<m_linesRSmallLane.size();i++)
		{
			averageHRG1SmallLane = averageHRG1SmallLane + m_linesRSmallLane[i].y;
			averageHGR1SmallLaneRadius 	= averageHGR1SmallLaneRadius + m_linesRSmallLane[i].x;
		}
		averageHRG1SmallLane = averageHRG1SmallLane/m_linesRSmallLane.size();
		averageHGR1SmallLaneRadius = averageHGR1SmallLaneRadius/m_linesRSmallLane.size();

/*		for(size_t i=0; i<m_linesRSmallLane.size();i++)
			{
				if(abs(m_representerHRGSLane.y-averageHRG1SmallLane)>abs(m_linesRSmallLane[i].y-averageHRG1SmallLane))
				{
					m_representerHRGSLane = m_linesRSmallLane[i];
				}
			}*/
		m_representerHRGSLane	= Point2f(averageHGR1SmallLaneRadius, averageHRG1SmallLane);
	}

//calculate intersection of representer horizontal hougline and representer on right side(crossing corner right front)
m_SMALL = tFalse;
m_sentSpot.x=0;
m_sentSpot.y=0;
if(!m_DeadEndRightBool)
	{
	if(!m_linesRSmall.empty() && !m_linesRSmallLane.empty())
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
		if(pt1rS.x != pt2rS.x && pt1rS.y != pt2rS.y)
		{
			float m1 = (pt2rS.y-pt1rS.y)/(pt2rS.x-pt1rS.x);
			float m2 = (pt2r.y-pt1r.y)/(pt2r.x-pt1r.x);
			double b1 = pt2rS.y - m1*pt2rS.x;
			double b2 = pt2r.y - m2*pt2r.x;
			m_intersecpointRS.x = cvRound((b2-b1)/(m1-m2));
			m_intersecpointRS.y = cvRound(m1*m_intersecpointRS.x+b1);;
			if(m_intersecpointRS.x>0 && m_intersecpointRS.y>0) m_crossFrontRightS = Util::ComputeWorldCoordinate(m_intersecpointRS.x,m_intersecpointRS.y,m_matImageDepth.at<ushort>(floor(m_intersecpointRS.y*0.5),floor(m_intersecpointRS.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUpHough,m_iImageHalfWidth);
			float max = 0;
			int index = 0;	
			for(size_t i=0; i<CrossPoint.size();i++)
			{
	//LOG_INFO(cString::Format("CD: norm(CrossPoint[i]-m_crossFrontRightS = %f", norm(CrossPoint[i]-m_crossFrontRightS) ) );
				if(norm(CrossPoint[i]-m_crossFrontRightS)<0.17) // 0.085
				{
					if(CrossPoint[i].z>max)
					{
						max = CrossPoint[i].z;
						index = i;
						m_sentSpot = PixelSpot[i];
					}
					//LOG_INFO(cString::Format("CD: CrossPoint[i] = %f %f %f", CrossPoint[i].x,CrossPoint[i].y,CrossPoint[i].z ) );	
				}				
			}
			if(max!=0)
			{
			CrossPointHough.push_back(CrossPoint[index]);
			m_SMALL = tTrue;
			}
		}
	}
//LOG_INFO(cString::Format("CD: m_SMALL = %i", m_SMALL) );
	if(!HoughRGroup2.empty() && !m_SMALL && m_iframeCounter>MIN_FRAMES)
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
				m_crossFrontRight  = Point3f(0.0, 0.0, 0.0);
				if(m_intersecpointR.y*0.5 > 0 && m_intersecpointR.y*0.5 < (m_nImagecutHeightDown-m_nImagecutHeightUp)/2 && m_intersecpointR.x*0.5+m_iImageHalfWidth*0.5 > 0 && m_intersecpointR.x*0.5+m_iImageHalfWidth*0.5 < (m_nImagecutWidthRight-m_nImagecutWidthLeft)/2 && !isnan(m_intersecpointR.x) && !isnan(m_intersecpointR.y)){	
					m_crossFrontRight = Util::ComputeWorldCoordinate(m_intersecpointR.x,m_intersecpointR.y,m_matImageDepth.at<ushort>(floor(m_intersecpointR.y*0.5),floor(m_intersecpointR.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUpHough,m_nImagecutWidthLeftHough+m_iImageHalfWidth);
				}
				if(m_crossFrontRight.x!=0 && CrossPointHough.empty()) CrossPointHough.push_back(m_crossFrontRight);
			}
		}
	}
	else
	{
		//Calculate crosspoint if deadendright=true on left side with cornerdetection
		CrossPoint.clear();
		if(m_iframeCounter>MIN_FRAMES){
			vector<size_t> PointInd;
			 for(size_t j=0;j<m_Points.size();j++)
				{
					Point3f corner = Util::ComputeWorldCoordinate(m_Points[j].x, m_Points[j].y,m_matImageDepth.at<ushort>(floor(m_Points[j].y*0.5), floor(m_Points[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			
					if(corner.x < -0.45 && corner.y > 0.19 && corner.y < 0.23 && corner.z < 2.0 && corner.z > 1.10){
							CrossPoint.push_back(corner);
							PointInd.push_back(j);
					}
				}
			//finde nun erneut wie am anfang die ecke, die am nahesten am auto ist.idx muss nicht neu deklariert werden, da er schon zu Beginn deklariert wird und wir ihn weiterverwenden müssen, da am ende CrossPoint[idx] übergeben wird.
		
			// Get farest corner of the nearest corner group
			idx 		= -1;
			if(!CrossPoint.empty()){
				m_DeadEndLeftBool 	= tFalse; // it is not DeadEndLeft if it is deadEndRight and we found a corner
				vector<float> nearGroup;
				float average = 0.0;
				size_t goodInd;
				for(size_t i=0; i<CrossPoint.size();i++)
				{
					if (nearGroup.empty() || fabs(average-CrossPoint[i].z)<0.07)
					{
						goodInd 	= i;
						nearGroup.push_back(CrossPoint[i].z);
						average = 0.0;
						for(size_t j = 0; j < nearGroup.size(); j++){
							average+=nearGroup.at(j);
						}
						average/=nearGroup.size();
					}
					else if (CrossPoint[i].z-average<-0.07){
						goodInd 	= i;
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

				m_sentSpot 	= Point2i(m_Points.at(PointInd.at(goodInd)).y, m_Points.at(PointInd.at(goodInd)).x);

	//			LOG_INFO(cString::Format("CD: Crosspoint.z = %f", buffer2) );
			}
		}
	}


//LOG_WARNING(cString::Format("CrossDetection: m_DeadEndLeftBool: %i,m_DeadEndRightBool: %i,m_DeadEndFrontBool: %i",m_DeadEndLeftBool,m_DeadEndRightBool,m_DeadEndFrontBool));
	if(m_iframeCounter>MAX_FRAMES || (_clock->GetStreamTime()-m_activationTime)/1000000.0 > WORKING_TIME_MAX )
	{
			m_CrossType =	UNDEFINED_CROSSING;
			//LOG_ERROR(cString::Format("CrossDetection: No Crossing found! UNDEFINED_CROSSING") );
			TransmitCrossingSpot(-1.0, 0.0);
			m_iframeCounter = 0;
			RETURN_NOERROR;
	}
	else if((CrossPointHough.empty() && !m_DeadEndRightBool) || (CrossPoint.empty() && m_DeadEndRightBool) )
	{	
			//LOG_WARNING(cString::Format("CrossDetection: No Crossing found!"));
			// Try again if failed
			if (!m_shutdown) m_bActive = tTrue;
	}
	else if( (!m_DeadEndRightBool && (CrossPointHough[0].z > 1.3 || CrossPointHough[0].z < 0.25) ) || (m_DeadEndRightBool && idx > -1 && CrossPoint[idx].z > 2) )
	{
		//LOG_WARNING(cString::Format("CrossDetection: No valid Crossing lot found!"));
		// Try again if failed
		if (!m_shutdown) m_bActive = tTrue;
 	}
	else if((m_DeadEndFrontBool && m_DeadEndLeftBool) || (m_DeadEndRightBool && m_DeadEndFrontBool) || (m_DeadEndLeftBool && m_DeadEndRightBool))
	{
		/*fstream f;
		f.open("testtestest.dat",ios::out);
		f << "verkackt";
		f.close();*/
		//LOG_INFO(cString::Format("IP:Invalid Crosstype") );
		if (!m_shutdown) m_bActive = tTrue;
	}
	else
	{
		// Schreibe gefundene Kreuzung in Outputpin	
 		if (!m_oDistanceAndTypeOutput.IsConnected())
   		{
			LOG_WARNING(cString::Format("CrossDetection: OutputPin for crossing not connected!"));
		    //RETURN_NOERROR;
		}
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

		// Calculate vertikal Lane Angle
		m_rotPt1			= Point2i(0,0);
		m_rotPt2			= Point2i(0,0);
		Point3f pt1World	= Point3f(5.0, 5.0, 5.0);
		Point3f pt2World	= Point3f(0.0, 0.0, 0.0);

		tInt32 	firstValue 	= 60;
		tInt32 	secondValue	= 0;
		int 	i 			= 0;

		bool canceled 		= false;

		int counter  		= 0;

		while (m_rotPt1.y 	< 50 || pt1World.z < 0.2){
			m_rotPt1.x 		= floor(cos(m_representerHRGSLane.y)*m_representerHRGSLane.x - (firstValue+i)*(-sin(m_representerHRGSLane.y)));
			m_rotPt1.y 		= floor(sin(m_representerHRGSLane.y)*m_representerHRGSLane.x - (firstValue+i)*(cos(m_representerHRGSLane.y)));
			if (m_rotPt1.x*0.5+m_iImageHalfWidth*0.5 > (m_nImagecutWidthRight-m_nImagecutWidthLeft)/2 || m_rotPt1.y*0.5 > (m_nImagecutHeightDown-m_nImagecutHeightUp)/2 || counter > 40){
				canceled 	= true;
				LOG_WARNING(cString::Format("CD: Cancled calculation of first lane angle with timer = %i!", counter) );
				break;
			}	
			if (m_rotPt1.x*0.5+m_iImageHalfWidth*0.5 < 0 || m_rotPt1.y*0.5 < 0){				
				i+=5;
				counter++;
				continue;	
			}
						
			pt1World 		= Util::ComputeWorldCoordinate(m_rotPt1.x, m_rotPt1.y,m_matImageDepth.at<ushort>(floor(m_rotPt1.y*0.5),floor(m_rotPt1.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUp,m_iImageHalfWidth);
			secondValue 	= firstValue+i + 50;
			i+=5;
			counter++;			
		}	
		//LOG_INFO(cString::Format("firstValue+i = %i", firstValue+i) );
		if (!canceled){
			i 					= 0;
			counter 			= 0;
			while (pt2World.z 	< 0.2 && pt1World.z!=pt2World.z){ // Don't devide by zero to get the rotation angle
				m_rotPt2.x 	= floor(cos(m_representerHRGSLane.y)*m_representerHRGSLane.x - (secondValue+i)*(-sin(m_representerHRGSLane.y)));
				m_rotPt2.y 	= floor(sin(m_representerHRGSLane.y)*m_representerHRGSLane.x - (secondValue+i)*(cos(m_representerHRGSLane.y)));
				if (m_rotPt2.x*0.5+m_iImageHalfWidth*0.5 > (m_nImagecutWidthRight-m_nImagecutWidthLeft)/2 || m_rotPt2.y*0.5 > (m_nImagecutHeightDown-m_nImagecutHeightUp)/2 || counter > 40){
					canceled 	= true;
					LOG_WARNING(cString::Format("CD: Cancled calculation of second lane angle with timer = %i!", counter) );
					break;
				}	
				if (m_rotPt2.x*0.5+m_iImageHalfWidth*0.5 < 0 || m_rotPt2.y*0.5 < 0){					
					i+=5;
					counter++;
					continue;	
				}
				pt2World 	= Util::ComputeWorldCoordinate(m_rotPt2.x, m_rotPt2.y,m_matImageDepth.at<ushort>(floor(m_rotPt2.y*0.5),floor(m_rotPt2.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUp,m_iImageHalfWidth);
				i+=5;
				counter++;
			}
		}

		//LOG_INFO(cString::Format("secondValue+i = %i", secondValue+i) );

		//LOG_INFO(cString::Format("pt1World = (%f, %f)", pt1World.z, pt1World.x));
		//LOG_INFO(cString::Format("pt2World = (%f, %f)", pt2World.z, pt2World.x));

		if (canceled) 	m_rotationAngle 	= 0.0;
		else 			m_rotationAngle		= -atan((pt1World.x-pt2World.x)/(pt1World.z-pt2World.z))*180/M_PI;

		LOG_INFO(cString::Format("CD: rotationAngle = %f", m_rotationAngle) );

		tFloat32 rotationDegree = 0.0;
		if (m_rotationAngle > 10.0) 		rotationDegree	= - 1.0;
		else if (m_rotationAngle <-0.5) 	rotationDegree	=   1.0;

		/*fstream f;
		f.open("CDAngleAuswertung", ios::out|ios::app);
		f << m_rotationAngle  << "\n";
		f.close();*/


		// Calculate distance to crossing		
		tFloat32 foundPoint 	= 0.0;

		// Correct driven distance!
		if(m_DeadEndRightBool) foundPoint 	= CrossPoint[idx].z-CROSS_POINT_OFFSET*rotationDegree; //Falls rechts "zu" müssen wir eine ecke von links nehmen.

//		else TransmitCrossingSpot(CrossPointHough[0].z);

		else {
			// Check if we are rotated within the lane
			foundPoint 	= CrossPointHough[0].z - CROSS_HOUGH_OFFSET*fabs(rotationDegree);
		}

		//f3.close();

		TransmitCrossingSpot(foundPoint, rotationDegree);
//m_bActive = tTrue;
/*
		// Horizontal Angle
		m_HrotPt1			= Point2i(0,0);
		m_HrotPt2			= Point2i(0,0);
		pt1World			= Point3f(5.0, 5.0, 5.0);
		pt2World			= Point3f(0.0, 0.0, 0.0);

		firstValue 			= 50;
		secondValue			= 0;
		i 					= 0;

		while (m_HrotPt1.y 	< 50 || pt1World.z < 0.2){
			m_HrotPt1.x 		= floor(cos(m_representerHRGS.y)*m_representerHRGS.x - (firstValue+i)*(-sin(m_representerHRGS.y)));
			m_HrotPt1.y 		= floor(sin(m_representerHRGS.y)*m_representerHRGS.x - (firstValue+i)*(cos(m_representerHRGS.y)));
			pt1World 			= Util::ComputeWorldCoordinate(m_HrotPt1.x, m_HrotPt1.y,m_matImageDepth.at<ushort>(floor(m_HrotPt1.y*0.5),floor(m_HrotPt1.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUp,m_iImageHalfWidth);
			secondValue 		= firstValue+i + 60;
			i+=5;			
		}	
		//LOG_INFO(cString::Format("firstValue+i = %i", firstValue+i) );
		i 					= 0;
		while (pt2World.z 	< 0.2 && pt1World.z!=pt2World.z){ // Don't devide by zero to get the rotation angle
			m_HrotPt2.x 	= floor(cos(m_representerHRGS.y)*m_representerHRGS.x - (secondValue+i)*(-sin(m_representerHRGS.y)));
			m_HrotPt2.y 	= floor(sin(m_representerHRGS.y)*m_representerHRGS.x - (secondValue+i)*(cos(m_representerHRGS.y)));
			pt2World 		= Util::ComputeWorldCoordinate(m_HrotPt2.x, m_HrotPt2.y,m_matImageDepth.at<ushort>(floor(m_HrotPt2.y*0.5),floor(m_HrotPt2.x*0.5+m_iImageHalfWidth*0.5)),m_nImagecutHeightUp,m_iImageHalfWidth);
			i+=5;
		}
		//LOG_INFO(cString::Format("secondValue+i = %i", secondValue+i) );

		//LOG_INFO(cString::Format("pt1World = (%f, %f)", pt1World.z, pt1World.x));
		//LOG_INFO(cString::Format("pt2World = (%f, %f)", pt2World.z, pt2World.x));

		m_HrotationAngle		= -atan((pt1World.x-pt2World.x)/(pt1World.z-pt2World.z))*180/M_PI;*/
		

		//else TransmitCrossingSpot(CrossPointHough[0].z); 
		//LOG_INFO(cString::Format("CD: Transmit: = %f ", CrossPointHough[0].z) );	
		//LOG_INFO(cString::Format("ecke: %f", CrossPointHough[0].z));
		/*m_iframeCounter = 0;
		fstream ff;
		ff.open("m_representerHRGSLane.y_Statistik", ios::out|ios::app);
		ff << m_representerHRGSLane.y*180/M_PI << "\n";
		ff.close(); */
		
	}
//LOG_WARNING(cString::Format("CrossDetection: CrossType: %i!",m_CrossType));
	//LOG_INFO(cString::Format("IP: Finished Searching") );
	m_intersecpointRS.x = 0.0;
	m_intersecpointRS.y = 0.0;
	m_intersecpointR.x = 0.0;
	m_intersecpointR.y = 0.0;
	RETURN_NOERROR;
}


tResult cCrossingDetect::ProcessCircle()
{

	/*fstream f3;
	f3.open("CDErrorDetect", ios::out|ios::app);
	f3 << _clock->GetStreamTime() << "\tMarker CIRCLE_PROCESSING\n";
	f3.close();*/

    Size szImageCutThresSize = m_matThres2.size();
	m_PointsCircle.clear(); 

	//m_bActive = tTrue;
	
	// Write corners from Matrix to vector
    for(tInt nColumn=0; nColumn < szImageCutThresSize.width; nColumn++)
    {  
    	for(tInt nRow=0; nRow < szImageCutThresSize.height; nRow++)
    		{  
                 
        	if(m_matThres2.at<uchar>(nRow, nColumn)==255) 
        		{ 
				m_PointsCircle.push_back(Point2f(nColumn,nRow));         
            	}
        	}
    }

	vector<Point3f> CrossPoint;

	//elimininate pointcloud 
	vector<int> killUs;
	for(size_t i=0;i<m_PointsCircle.size();i++)
		{	
		Point3f corner1 = Util::ComputeWorldCoordinate(m_PointsCircle[i].x, m_PointsCircle[i].y,m_matImageDepth.at<ushort>(floor(m_PointsCircle[i].y*0.5),floor(m_PointsCircle[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
		for(size_t j=i+1;j<m_PointsCircle.size();j++)
			{
				Point3f corner2 = Util::ComputeWorldCoordinate(m_PointsCircle[j].x, m_PointsCircle[j].y,m_matImageDepth.at<ushort>(floor(m_PointsCircle[j].y*0.5), floor(m_PointsCircle[j].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
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

	if(!killUs.empty())
	{
		sort(killUs.begin(),killUs.end(),greater<int>());
		for(size_t i=0;i<killUs.size();i++)
		{
			m_PointsCircle.erase(m_PointsCircle.begin()+killUs[i]);
		}
	}

	vector<Point2f> HoughLGroupCIRCLE;
	for(size_t i=0; i<m_linesCIRCLE.size();i++)
	{
		if((m_linesCIRCLE[i][1]*180/M_PI<88) && (m_linesCIRCLE[i][1]*180/M_PI>78))
		{
				HoughLGroupCIRCLE.push_back(Point2f(m_linesCIRCLE[i][0],m_linesCIRCLE[i][1]));		
		}
	}

	if(HoughLGroupCIRCLE.empty() || m_PointsCircle.empty())
	{
		if (!m_shutdown) m_bActive = tTrue;
		RETURN_NOERROR;
	}

	m_representerHLGC=Point2f(0,0);
	//Calculate representer for HoughLGroup1
	if(!HoughLGroupCIRCLE.empty())
	{
		float averageHLGC = 0;
		m_representerHLGC = HoughLGroupCIRCLE[0]; 
		for(size_t i=0; i<HoughLGroupCIRCLE.size();i++)
		{
			averageHLGC = averageHLGC + HoughLGroupCIRCLE[i].y*180/M_PI;
		}
			averageHLGC = averageHLGC/HoughLGroupCIRCLE.size();
		for(size_t i=0; i<HoughLGroupCIRCLE.size();i++)
		{
				if(abs(m_representerHLGC.y*180/M_PI-averageHLGC)>abs(HoughLGroupCIRCLE[i].y*180/M_PI-averageHLGC))
				{
					m_representerHLGC = HoughLGroupCIRCLE[i];
				}
		}
	}

	Point2f pt1,pt2;
	pt1.x = cos(m_representerHLGC.y)*m_representerHLGC.x + 1000*(-sin(m_representerHLGC.y));
	pt1.y = sin(m_representerHLGC.y)*m_representerHLGC.x + 1000*(cos(m_representerHLGC.y));
	pt2.x = cos(m_representerHLGC.y)*m_representerHLGC.x - 1000*(-sin(m_representerHLGC.y));
	pt2.y = sin(m_representerHLGC.y)*m_representerHLGC.x - 1000*(cos(m_representerHLGC.y));

	for (size_t i = 0; i<30; i++){

		m_GCLCoordinates[i] = Point3f(0.0,0.0,0.0);
	}
	
	if(pt1.x != pt2.x && pt1.y != pt2.y)
	{
		float m1 = (pt2.y-pt1.y)/(pt2.x-pt1.x);
		double b1 = pt2.y - m1*pt2.x;

		// Search actual crossing
		for(size_t i=0;i<m_PointsCircle.size();i++) 
		{		
			Point3f corner1 = Util::ComputeWorldCoordinate(m_PointsCircle[i].x, m_PointsCircle[i].y,m_matImageDepth.at<ushort>(floor(m_PointsCircle[i].y*0.5),floor(m_PointsCircle[i].x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
			m_GCLCoordinates[i] 	= corner1;
			// Test if m_PointsCircle are valid
			if(corner1.z < 1.8 &&  corner1.z > 0.4 && corner1.y > 0.19 && corner1.y < 0.23) // && corner1.x > -0.25)
			{
				
				float m2 = -1/m1;
				float b2 =  -m_PointsCircle[i].x*m2+m_PointsCircle[i].y;
				float x = (b2-b1)/(m1-m2);
				float y = m1*x+b1;

				Point3f corner2 = Util::ComputeWorldCoordinate(x, y,m_matImageDepth.at<ushort>(floor(y*0.5),floor(x*0.5)),m_nImagecutHeightUp,m_nImagecutWidthLeft);
				if(corner2.z < 1.8 && corner2.z > 0.4 && corner2.y > 0.19 && corner2.y < 0.23 && norm(corner1-corner2) < 0.05)
				{
					PixelSpot.push_back(Point2f(m_PointsCircle[i].y,m_PointsCircle[i].x));
					CrossPoint.push_back(corner1);
					LOG_INFO(cString::Format("CrossDetection: Ausfahrt %f", corner1.z) );
				}
			}
		}
	}

	m_sentSpot 	= Point2i(0,0);

	int idx = -1;
	double buffer;
	double MIN1 = 10000;
	for(size_t i=0; i<CrossPoint.size();i++)
	{
		buffer = CrossPoint[i].z; //norm(CrossPoint[i]);
		if(buffer<MIN1)
			{
				MIN1 = buffer;
				idx = i;
				m_sentSpot = PixelSpot[i];
			}
	}

	if(m_iframeCounter>MAX_FRAMES)
	{
		m_CrossType =	UNDEFINED_CROSSING;
		LOG_ERROR(cString::Format("CrossDetection: No Crossing found! UNDEFINED_CROSSING") );
		TransmitCrossingSpot(-1.0, 0.0);
		m_iframeCounter = 0;
		RETURN_NOERROR;
	}
	else if(CrossPoint.empty())
	{
		if (!m_shutdown) m_bActive = tTrue;
	}
	else
	{
		// Calculate Circle offset
		float r 		= 1.7;
		float z_sq  	= CrossPoint[idx].z*CrossPoint[idx].z + CrossPoint[idx].x*CrossPoint[idx].x;
		float alpha 	= acos(1-z_sq/(2*r*r));
		float z_circ	= alpha * r;

		LOG_INFO(cString::Format("CD: z_sq = %f", z_sq) );	
		LOG_INFO(cString::Format("CD: CrossPoint[idx].x = %f", CrossPoint[idx].x) );	
		LOG_INFO(cString::Format("CD: alpha = %f", alpha) );
	
		LOG_INFO(cString::Format("CrossDetection: CrossPoint[idx].z = %f, z_circ = %f, diff= %f", CrossPoint[idx].z,z_circ, fabs(CrossPoint[idx].z-z_circ)) );

//		TransmitCrossingSpot(CrossPoint[idx].z + CIRCLE_OFFSET, 0.0);
		TransmitCrossingSpot(z_circ, 0.0);
	}
	
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
if(!m_BoolTrafficCircle)
	{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 100, 255).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,m_nImagecutWidthLeft , m_nImagecutHeightUp, m_nImagecutWidthRight, m_nImagecutHeightDown);


		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,255,0).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(cvRound(m_sentSpot.y-4)) + m_nImagecutWidthLeft , static_cast<tInt16>(cvRound(m_sentSpot.x)-4) + m_nImagecutHeightUp, static_cast<tInt16>(cvRound(m_sentSpot.y)+4) + m_nImagecutWidthLeft, static_cast<tInt16>(cvRound(m_sentSpot.x)+4) + m_nImagecutHeightUp);		
		if(m_intersecpointRS.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointRS.x)+m_nImagecutWidthLeftHough-4+m_iImageHalfWidth, cvRound(m_intersecpointRS.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointRS.x)+m_nImagecutWidthLeftHough+4+m_iImageHalfWidth, cvRound(m_intersecpointRS.y)+m_nImagecutHeightUpHough+4);   	   	
		//if(m_intersecpointRB.x!=0)cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT, cvRound(m_intersecpointRB.x)+m_nImagecutWidthLeftHough-4+m_iImageHalfWidth, cvRound(m_intersecpointRB.y)+m_nImagecutHeightUpHough-4, cvRound(m_intersecpointRB.x)+m_nImagecutWidthLeftHough+4+m_iImageHalfWidth, cvRound(m_intersecpointRB.y)+m_nImagecutHeightUpHough+4);   
	
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(100,255,255).GetRGBA());
			float rho = m_representerHRGS.x;
			float theta = m_representerHRGS.y;
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_iImageHalfWidth, pt2.y+m_nImagecutHeightUpHough, pt1.x+m_iImageHalfWidth, pt1.y+m_nImagecutHeightUpHough);

				

		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,255,0).GetRGBA());

			float rho1 = m_representerHRGSLane.x;
			float theta1 = m_representerHRGSLane.y;
			Point pt11,pt21;
			double a1 = cos(theta1), b1 = sin(theta1);
			double x01 = a1*rho1, y01 = b1*rho1;
			pt11.x = cvRound(x01 + 1000*(-b1));
			pt11.y = cvRound(y01 + 1000*(a1));
			pt21.x = cvRound(x01 - 1000*(-b1));
			pt21.y = cvRound(y01 - 1000*(a1)); 
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt21.x+m_iImageHalfWidth, pt21.y+m_nImagecutHeightUpHough, pt11.x+m_iImageHalfWidth, pt11.y+m_nImagecutHeightUpHough);

		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,255,100).GetRGBA());
		/*for(size_t i=0; i<m_linesR.size();i++)
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
		}*/

		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,0,255).GetRGBA());
		/*for(size_t i=0; i<m_linesRSmall.size();i++)
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
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_iImageHalfWidth, pt2.y+m_nImagecutHeightUpHough, pt1.x+m_iImageHalfWidth, pt1.y+m_nImagecutHeightUpHough);
		}*/

/*		for(size_t i=0; i<m_linesRSmallLane.size();i++)
		{
			float rho = m_linesRSmallLane[i].x;
			float theta = m_linesRSmallLane[i].y;
			Point pt1,pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_iImageHalfWidth, pt2.y+m_nImagecutHeightUpHough, pt1.x+m_iImageHalfWidth, pt1.y+m_nImagecutHeightUpHough);
		}
*/
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
		for(size_t i=0; i < m_Points.size(); i++) 
			{
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_Points[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_Points[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_Points[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_Points[i].y+2) + m_nImagecutHeightUp);
			}
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());

		for(size_t i=0; i < PixelSpot.size(); i++)
			{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(PixelSpot[i].y-2) + m_nImagecutWidthLeft , static_cast<tInt16>(PixelSpot[i].x-2) + m_nImagecutHeightUp, static_cast<tInt16>(PixelSpot[i].y+2) + m_nImagecutWidthLeft, static_cast<tInt16>(PixelSpot[i].x+2) + m_nImagecutHeightUp);
			}
	}
	else
	{
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 100, 255).GetRGBA());
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,m_nImagecutWidthLeft , m_nImagecutHeightUp, m_nImagecutWidthRight, m_nImagecutHeightDown);

		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 0).GetRGBA());

		for(size_t i=0; i<m_linesCIRCLE.size();i++)
		{
			//if((m_linesCIRCLE[i][1]*180/M_PI>10) || (m_linesCIRCLE[i][1]*180/M_PI<40))continue;
			if((m_linesCIRCLE[i][1]*180/M_PI<88) && (m_linesCIRCLE[i][1]*180/M_PI>78)){
				float rho = m_linesCIRCLE[i][0];
				float theta = m_linesCIRCLE[i][1];
				Point pt1,pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000*(-b));
				pt1.y = cvRound(y0 + 1000*(a));
				pt2.x = cvRound(x0 - 1000*(-b));
				pt2.y = cvRound(y0 - 1000*(a)); 
				cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_nImagecutWidthLeft, pt2.y+m_nImagecutHeightUp, pt1.x+m_nImagecutWidthLeft, pt1.y+m_nImagecutHeightUp);
			}

			/*if((m_linesCIRCLEsmall[i][1]*180/M_PI>88) || (m_linesCIRCLEsmall[i][1]*180/M_PI<78))continue;
			rho = m_linesCIRCLEsmall[i][0];
			theta = m_linesCIRCLEsmall[i][1];
	
			a = cos(theta), b = sin(theta);
			x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a)); 
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWLINE, pt2.x+m_nImagecutWidthLeft, pt2.y+m_nImagecutHeightUp, pt1.x+m_nImagecutWidthLeft, pt1.y+m_nImagecutHeightUp);*/
		}

		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
		for(size_t i=0; i < m_PointsCircle.size(); i++) 
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_PointsCircle[i].x-2) + m_nImagecutWidthLeft , static_cast<tInt16>(m_PointsCircle[i].y-2) + m_nImagecutHeightUp, static_cast<tInt16>(m_PointsCircle[i].x+2) + m_nImagecutWidthLeft, static_cast<tInt16>(m_PointsCircle[i].y+2) + m_nImagecutHeightUp);
		}
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());
		for(size_t i=0; i < PixelSpot.size(); i++)
		{
			cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(PixelSpot[i].y-2) + m_nImagecutWidthLeft , static_cast<tInt16>(PixelSpot[i].x-2) + m_nImagecutHeightUp, static_cast<tInt16>(PixelSpot[i].y+2) + m_nImagecutWidthLeft, static_cast<tInt16>(PixelSpot[i].x+2) + m_nImagecutHeightUp);
		}
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0,255,0).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(cvRound(m_sentSpot.y-4)) + m_nImagecutWidthLeft , static_cast<tInt16>(cvRound(m_sentSpot.x)-4) + m_nImagecutHeightUp, static_cast<tInt16>(cvRound(m_sentSpot.y)+4) + m_nImagecutWidthLeft, static_cast<tInt16>(cvRound(m_sentSpot.x)+4) + m_nImagecutHeightUp);		
	}
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

	tFloat64 text3 = m_GCLDistance;
	cString strText3 = cString::FromFloat64(text3);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 60, strText3.GetLength());
	cGCLWriter::StoreData(pc, strText3.GetLength(), strText3.GetPtr());  

	cString strText4 = cString::FromInt32(PixelSpot.size());
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 80, strText4.GetLength());
	cGCLWriter::StoreData(pc, strText4.GetLength(), strText4.GetPtr());

	tFloat64 text45 = m_representerHRGSLane.y*180/M_PI;
	cString strText45 = cString::FromFloat64(text45);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 100, strText45.GetLength());
	cGCLWriter::StoreData(pc, strText45.GetLength(), strText45.GetPtr());

	tFloat64 text475 = m_representerHRGS.y*180/M_PI;
	cString strText475 = cString::FromFloat64(text475);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 120, strText475.GetLength());
	cGCLWriter::StoreData(pc, strText475.GetLength(), strText475.GetPtr());


	// Punkte zur Berechnung der Rotationswinkels in der spur
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(50, 100, 150).GetRGBA());
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_rotPt1.x-2+m_iImageHalfWidth) , static_cast<tInt16>(m_rotPt1.y-2+m_nImagecutHeightUp), static_cast<tInt16>(m_rotPt1.x+2+m_iImageHalfWidth), static_cast<tInt16>(m_rotPt1.y+2+m_nImagecutHeightUp));
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_rotPt2.x-2+m_iImageHalfWidth) , static_cast<tInt16>(m_rotPt2.y-2+m_nImagecutHeightUp), static_cast<tInt16>(m_rotPt2.x+2+m_iImageHalfWidth), static_cast<tInt16>(m_rotPt2.y+2+m_nImagecutHeightUp));
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_HrotPt1.x-2+m_iImageHalfWidth) , static_cast<tInt16>(m_HrotPt1.y-2+m_nImagecutHeightUp), static_cast<tInt16>(m_HrotPt1.x+2+m_iImageHalfWidth), static_cast<tInt16>(m_HrotPt1.y+2+m_nImagecutHeightUp));
	cGCLWriter::StoreCommand(pc, GCL_CMD_DRAWRECT,static_cast<tInt16>(m_HrotPt2.x-2+m_iImageHalfWidth) , static_cast<tInt16>(m_HrotPt2.y-2+m_nImagecutHeightUp), static_cast<tInt16>(m_HrotPt2.x+2+m_iImageHalfWidth), static_cast<tInt16>(m_HrotPt2.y+2+m_nImagecutHeightUp));

	// plot -arctan(1/m)
	tFloat64 text49 = m_rotationAngle;
	cString strText49 = cString::FromFloat64(text49);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 140, strText49.GetLength());
	cGCLWriter::StoreData(pc, strText49.GetLength(), strText49.GetPtr());

	// plot -arctan(1/mH)
	/*tFloat64 text495 = m_HrotationAngle;
	cString strText495 = cString::FromFloat64(text495);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 160, strText495.GetLength());
	cGCLWriter::StoreData(pc, strText495.GetLength(), strText495.GetPtr());*/

	

	cString strText5 = cString::FromBool(m_DeadEndFrontBool);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 400, strText5.GetLength());
	cGCLWriter::StoreData(pc, strText5.GetLength(), strText5.GetPtr()); 

	cString strText6 = cString::FromBool(m_DeadEndLeftBool);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 420, strText6.GetLength());
	cGCLWriter::StoreData(pc, strText6.GetLength(), strText6.GetPtr());
 
	cString strText7 = cString::FromBool(m_DeadEndRightBool);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 600, 440, strText7.GetLength());
	cGCLWriter::StoreData(pc, strText7.GetLength(), strText7.GetPtr()); 

	tFloat64 text4x = m_crossFrontRightS.x;
	cString strText4x = cString::FromFloat64(text4x);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 200, 190, strText4x.GetLength());
	cGCLWriter::StoreData(pc, strText4x.GetLength(), strText4x.GetPtr());
	tFloat64 text4z = m_crossFrontRightS.z;
	cString strText4z = cString::FromFloat64(text4z);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 0, 0).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 300, 190, strText4z.GetLength());
	cGCLWriter::StoreData(pc, strText4z.GetLength(), strText4z.GetPtr());  

	for (size_t i = 0; i < 30; i++){
		tFloat64 textx = m_GCLCoordinates[i].x;
		cString strTextx = cString::FromFloat64(textx);
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 150, 200 + i*10, strTextx.GetLength());
		cGCLWriter::StoreData(pc, strTextx.GetLength(), strTextx.GetPtr()); 

		tFloat64 texty = m_GCLCoordinates[i].y;
		cString strTexty = cString::FromFloat64(texty);
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 250, 200 + i*10, strTexty.GetLength());
		cGCLWriter::StoreData(pc, strTexty.GetLength(), strTexty.GetPtr()); 

		tFloat64 textz = m_GCLCoordinates[i].z;
		cString strTextz = cString::FromFloat64(textz);
		cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
		cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 350, 200 + i*10, strTextz.GetLength());
		cGCLWriter::StoreData(pc, strTextz.GetLength(), strTextz.GetPtr()); 
	}

	// Dead End front representer
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255,0,0).GetRGBA());
	for(size_t i=0;i<m_lines.size();i++)
	{
		if(m_lines[i][1]*180/M_PI<105 && m_lines[i][1]*180/M_PI>75 && m_lines[i][0]<25)
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

tResult cCrossingDetect::TransmitCrossingSpot(float lot, tFloat32 rotationDegree) {

		m_GCLDistance = lot;

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
				pCoderOutput->GetID("rotationDegree", m_szIDRotationDegreeCrossOutput);			
				m_crossingSpotOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szCrossDistance, (tVoid*)&(lot));
			pCoderOutput->Set(m_szIDCrossingType, (tVoid*)&(m_CrossType));
			pCoderOutput->Set(m_szIDTimestampCrossOutput, (tVoid*)&nTimeStamp);
			pCoderOutput->Set(m_szIDRotationDegreeCrossOutput, (tVoid*)&rotationDegree);
		
		}
/*
		superf << "IP: Send crossing Lot to DM!!!!";
		superf.close();

		LOG_INFO(cString::Format("IP: Send crossing Lot to DM!!!!") );
*/
		pMediaSample->SetTime(pMediaSample->GetTime());
		m_oDistanceAndTypeOutput.Transmit(pMediaSample);

	RETURN_NOERROR;
}

tResult cCrossingDetect::InitialState(){
		m_GCLDistance 	= 0.0;
		m_iframeCounter = 0;
	    m_ui8Imagecount = 0;
		m_DeadEndFrontBool = tFalse;
		m_DeadEndLeftBool = tFalse;
		m_DeadEndRightBool = tFalse;
		RGBreceived=false;
		m_CrossType = UNDEFINED_CROSSING;
		m_SMALL = tFalse;
		m_bActive = tFalse;
		m_BoolTrafficCircle = tFalse;
		PixelSpot.clear();
		m_intersecpointRS.x = 0;
		m_intersecpointRS.y = 0;
		m_intersecpointR.x = 0;
		m_intersecpointR.y = 0;
		m_PointsCircle.clear();
		m_Points.clear();
		lines.clear();
		m_rotationAngle 	= 0.0;
		m_HrotationAngle 	= 0.0;
		m_shutdown 			= tFalse;
		m_activationTime 	= 0;
		RETURN_NOERROR;
}
