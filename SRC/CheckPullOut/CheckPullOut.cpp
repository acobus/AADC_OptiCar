/**
*
*CheckPullOut
*
* Date 17.03.2016
*
*/
 
 
#include "stdafx.h"
#include "CheckPullOut.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>
#include <cmath>


ADTF_FILTER_PLUGIN("CheckPullOut", OID_ADTF_CheckPullOut, cCheckPullOut)

#define CheckPullOut_PROP_ThresholdValueCanny "CheckPullOut::ThresholdValueCanny"
#define CheckPullOut_PROP_ThresholdValueHough "CheckPullOut::ThresholdValueHough"
#define CheckPullOut_PROP_ImagecutWidthLeft "CheckPullOut::ImagecutWidthLeft"
#define CheckPullOut_PROP_ImagecutWidthRight "CheckPullOut::ImagecutWidthRight"
#define CheckPullOut_PROP_ImagecutHeightUp "CheckPullOut::ImagecutHeightUp"
#define CheckPullOut_PROP_ImagecutHeightDown "CheckPullOut::ImagecutHeightDown"

#define CheckPullOut_PROP_SHOW_DEBUG "Common::Show Debug"

cCheckPullOut::cCheckPullOut(const tChar* __info) : cFilter(__info)
{
  
    SetPropertyBool(CheckPullOut_PROP_SHOW_DEBUG, tFalse);
    SetPropertyStr(CheckPullOut_PROP_SHOW_DEBUG NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");


	SetPropertyInt(CheckPullOut_PROP_ImagecutWidthLeft, 0);
	SetPropertyInt(CheckPullOut_PROP_ImagecutWidthLeft NSSUBPROP_MIN, 0);
    SetPropertyStr(CheckPullOut_PROP_ImagecutWidthLeft NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CheckPullOut_PROP_ImagecutWidthRight, 320);
	SetPropertyInt(CheckPullOut_PROP_ImagecutWidthRight NSSUBPROP_MIN, 0);
    SetPropertyStr(CheckPullOut_PROP_ImagecutWidthRight NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CheckPullOut_PROP_ImagecutHeightUp, 80);
	SetPropertyInt(CheckPullOut_PROP_ImagecutHeightUp NSSUBPROP_MIN, 0);
    SetPropertyStr(CheckPullOut_PROP_ImagecutHeightUp NSSUBPROP_DESCRIPTION, "Cuts the Image...");

	SetPropertyInt(CheckPullOut_PROP_ImagecutHeightDown, 260);
	SetPropertyInt(CheckPullOut_PROP_ImagecutHeightDown NSSUBPROP_MIN, 0);
    SetPropertyStr(CheckPullOut_PROP_ImagecutHeightDown NSSUBPROP_DESCRIPTION, "Cuts the Image...");

    SetPropertyFloat(CheckPullOut_PROP_ThresholdValueCanny, 210);
    SetPropertyBool(CheckPullOut_PROP_ThresholdValueCanny NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CheckPullOut_PROP_ThresholdValueCanny NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Edgedetection");

    SetPropertyFloat(CheckPullOut_PROP_ThresholdValueHough, 95);
    SetPropertyBool(CheckPullOut_PROP_ThresholdValueHough NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(CheckPullOut_PROP_ThresholdValueHough NSSUBPROP_DESCRIPTION, "The Thresholdvalue for the Houghlines");
	
    SetPropertyInt("Actuator Update Rate [Hz]",30);
	SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
	SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

    m_pISignalRegistry = NULL;
}

cCheckPullOut::~cCheckPullOut()
{
}

tResult cCheckPullOut::GetInterface(const tChar* idInterface,
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

tUInt cCheckPullOut::Ref()
{
    return cFilter::Ref();
}

tUInt cCheckPullOut::Unref()
{
    return cFilter::Unref();
}

tVoid cCheckPullOut::Destroy()
{
    delete this;
}

tResult cCheckPullOut::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult cCheckPullOut::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}
tResult cCheckPullOut::Init(tInitStage eStage, __exception )
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

		// Get description for bool values
    	tChar const * strDescBoolValueOut = pDescManager->GetMediaDescription("tBoolSignalValue");	
    	RETURN_IF_POINTER_NULL(strDescBoolValueOut);	
    	cObjectPtr<IMediaType> pTypeBoolValueOut = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolValueOut, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for SignalValue
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");   
		RETURN_IF_POINTER_NULL(strDescSignalValue);    
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Video Input
        RETURN_IF_FAILED(m_iVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideoInputPin));


		// Start (Input)
    	RETURN_IF_FAILED(m_iStart.Create("Start", pTypeBoolValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_iStart));
		RETURN_IF_FAILED(pTypeBoolValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartBoolInput)); 

		// create Inputpin for US_FrontCenter (Input)
		RETURN_IF_FAILED(m_iUS_FrontCenter.Create("US_FrontCenter", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iUS_FrontCenter));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalUS_FrontCenterInput));


        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

		// Create Pin for DistanceToCheckPullOut (Output)
		RETURN_IF_FAILED(m_oisCrossParking.Create("bIsCrossParking", pTypeBoolValueOut, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oisCrossParking));
		RETURN_IF_FAILED(pTypeBoolValueOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pIsCrossParking)); 


              
    }
    else if (eStage == StageNormal)
    {
		m_bStartBoolInput = tFalse;
        m_bFirstFrame = true;
		m_bFirstFrameDepthimage = true;
		m_bIsCrossParkingSet = tFalse;  
		m_bIDsUltraSonicSet = tFalse;
		m_bGroupRisEmpty = false;
		m_startSensorTimeStamp = 0;
		m_no_car_count = 0;
		m_car_count = 0;
		m_iFrontSensor = 0;
		m_iParallel = 0;

		m_bActive = tFalse;

    }
    RETURN_NOERROR;
}

tResult cCheckPullOut::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cCheckPullOut::ReadProperties(const tChar* strPropertyName)
{	
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_SHOW_DEBUG))
    {
        m_bShowDebug = GetPropertyBool(CheckPullOut_PROP_SHOW_DEBUG);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ImagecutWidthLeft))
    {
        m_nImagecutWidthLeft = GetPropertyInt(CheckPullOut_PROP_ImagecutWidthLeft);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ImagecutWidthRight))
    {
        m_nImagecutWidthRight = GetPropertyInt(CheckPullOut_PROP_ImagecutWidthRight);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ImagecutHeightUp))
    {
        m_nImagecutHeightUp = GetPropertyInt(CheckPullOut_PROP_ImagecutHeightUp);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ImagecutHeightDown))
    {
        m_nImagecutHeightDown = GetPropertyInt(CheckPullOut_PROP_ImagecutHeightDown);
	}

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ThresholdValueHough))
    {
        m_nThresholdValueHough = GetPropertyInt(CheckPullOut_PROP_ThresholdValueHough);
    }

	if (NULL == strPropertyName || cString::IsEqual(strPropertyName, CheckPullOut_PROP_ThresholdValueCanny))
    {
        m_nThresholdValueCanny = GetPropertyInt(CheckPullOut_PROP_ThresholdValueCanny);
    }

    RETURN_NOERROR;
}

tResult cCheckPullOut::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {        
        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput)
        {
			if (m_bGroupRisEmpty){

				if (m_startSensorTimeStamp == 0){
					m_startSensorTimeStamp= _clock->GetStreamTime();	
				}

				tUInt32 fTime = _clock->GetStreamTime();
	
				if ((fTime-m_startSensorTimeStamp)/1000000.0 > 0.5){
					m_bGroupRisEmpty = false;
					ProcessUSSensor();
				}	
		
			}
		}
	}

	return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cCheckPullOut::ProcessUSSensor(){
	tUInt32 countAll = m_no_car_count + m_car_count;

	if (countAll==0){
		TransmitIsCrossParking(tTrue);
	} else if (m_no_car_count/(tFloat32)countAll < 0.9){
		TransmitIsCrossParking(tFalse);
	} else {
		TransmitIsCrossParking(tTrue);
	}
    RETURN_NOERROR;
}

tResult cCheckPullOut::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cCheckPullOut::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
            	    m_bFirstFrame = false; 
            	}

            	ProcessInputRGB(pMediaSample, InputTimeStamp);
        }

        else if(pSource == &m_iStart)
        {

			//LOG_INFO(cString::Format("CPO: Received state signal!") );

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
        }
		else if(pSource == &m_iUS_FrontCenter && m_bGroupRisEmpty)
		{
			tFloat32 frontCenter;

			{
				__adtf_sample_read_lock_mediadescription(m_pSignalUS_FrontCenterInput, pMediaSample, pCoder);    
			
				if (!m_bIDsUltraSonicSet) {						  
					pCoder->GetID("f32Value", m_szIDFrontCenterUltraSonicInput);
					m_bIDsUltraSonicSet=tTrue;
				}
	  
	  			pCoder->Get(m_szIDFrontCenterUltraSonicInput, (tVoid*)&frontCenter);   
			}
			
			//LOG_INFO(cString::Format("CheckPullOut: frontCenter %f",frontCenter) );
			if (frontCenter > 0.6){
				m_no_car_count++;
				
			} else {
				m_car_count++;
			}
		}      
	}    
        
    RETURN_NOERROR;
}

tResult cCheckPullOut::ProcessInputRGB(IMediaSample* pSample, tTimeStamp tsInputTime)
{
	if (m_bActive){

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
		threshold(m_matCutHough, m_matCutHough, 50, 255,THRESH_TOZERO); // delete dark noise	
		m_matCutHough=m_matCutHough-50;
		normalize(m_matCutHough,m_matCutHough, 0, 255, NORM_MINMAX, CV_8UC1,Mat());
		convertScaleAbs(m_matCutHough, m_matCutHough); // Konveriere in 8Bit array. Nehme aus abs
        medianBlur(m_matCutHough, m_matCutHough,3); // reduce bright noise with edge-preserving filter
        Canny(m_matCutHough, m_matCanny,m_nThresholdValueCanny, m_nThresholdValueCanny, 3, false);// Detect Edges 
		m_lines.clear();
		HoughLines(m_matCanny,m_lines,1,CV_PI/180,m_nThresholdValueHough,0,0);

		check();
		CreateAndTransmitGCL();
	}
	RETURN_NOERROR;            
}


tResult cCheckPullOut::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cCheckPullOut::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cCheckPullOut::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cCheckPullOut::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cCheckPullOut::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

tResult cCheckPullOut::check()
{
	//eliminate Houghline-cloud of Hough
	vector<Point2f> HoughGroupR;
	for(size_t i=0; i<m_lines.size();i++)
		{
			if((m_lines[i][1]*180/M_PI<50) && (m_lines[i][1]*180/M_PI>30))
			{
					HoughGroupR.push_back(Point2f(m_lines[i][0],m_lines[i][1]));		
			}
		}

	// Process Us Sensor only if no lane line was found
	if(m_iFrontSensor > 10){
		// Us Sensor not blocked -> cross Parking lot, but check for Horizontal line (HoughGroupH) for safety reasons
		m_bGroupRisEmpty = true;
		RETURN_NOERROR;
	}else if (m_iParallel > 10){
		TransmitIsCrossParking(tFalse);
		RETURN_NOERROR;
	}

	if(HoughGroupR.empty()){
		m_iFrontSensor++;
		m_bActive = tTrue;
	}else{
		m_iParallel++;
		m_bActive = tTrue;
	}
	
	RETURN_NOERROR;
}


tResult cCheckPullOut::CreateAndTransmitGCL()
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
	
	for(size_t i=0; i < m_lines.size(); i++) 
		{
			if((m_lines[i][1]*180/M_PI<50) && (m_lines[i][1]*180/M_PI>30))
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
		}


    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

tResult cCheckPullOut::TransmitIsCrossParking(tBool b) {

	tUInt32 nTimeStamp = 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pIsCrossParking->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


	{
		__adtf_sample_write_lock_mediadescription(m_pIsCrossParking, pMediaSample, pCoderOutput); 

		if(!m_bIsCrossParkingSet){
			pCoderOutput->GetID("bValue", m_szIsCrossParking);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampOutput);
			m_bIsCrossParkingSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIsCrossParking, (tVoid*)&(b));
		pCoderOutput->Set(m_szIDTimestampOutput, (tVoid*)&nTimeStamp);
	
	}

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oisCrossParking.Transmit(pMediaSample);

	// Prepare for next use of the filter
	m_iParallel=0;
	m_iFrontSensor=0;
	m_bGroupRisEmpty = false;
	m_startSensorTimeStamp = 0.0;
	m_no_car_count = 0;
	m_car_count = 0;

	//LOG_INFO(cString::Format("CheckPullOut: Finished Searching with decsion %i",b) );

	RETURN_NOERROR;
}
