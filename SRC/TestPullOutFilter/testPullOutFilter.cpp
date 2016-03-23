/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/
#include "stdafx.h"
#include "testPullOutFilter.h"
#include "../../include/pullout_enum.h"


ADTF_FILTER_PLUGIN("Test PullOut Filter", __guid, testPullOutFilter);




testPullOutFilter::testPullOutFilter(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info)
{

    SetPropertyInt("Type",0);
    SetPropertyBool("Type" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("Type" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("Type" NSSUBPROP_DESCRIPTION, "pullOut type"); 

}

testPullOutFilter::~testPullOutFilter()
{
}

tHandle testPullOutFilter::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);            

    // make the qt connections
    connect(m_pWidget->m_btSendValueNextSpotFree, SIGNAL(clicked()), this, SLOT(OnTransmitBoolValue()));
    connect(m_pWidget->m_btSendValueCrossLeft, SIGNAL(clicked()), this, SLOT(transmit1()));
    connect(m_pWidget->m_btSendValueCrossRight, SIGNAL(clicked()), this, SLOT(transmit2()));
    connect(m_pWidget->m_btSendValueParallelRight, SIGNAL(clicked()), this, SLOT(transmit3()));

    return (tHandle)m_pWidget;
}

tResult testPullOutFilter::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult testPullOutFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)    
    {        
        //get the media description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
      
		//get description for sensor data pins
        tChar const * strDescIntSignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");
        RETURN_IF_POINTER_NULL(strDescIntSignalValue);
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeIntSignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescIntSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
     	
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
		RETURN_IF_FAILED(pTypeIntSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartInput)); 
       

		//create input pin for starting
    	RETURN_IF_FAILED(m_oStartPullOut.Create("StartPullOut", pTypeIntSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_oStartPullOut));
		//create input pin for nextSpotFree
    	RETURN_IF_FAILED(m_oNextSpotFree.Create("next_spot_free", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_oNextSpotFree));

        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
        // media descriptions ids not set by now
		m_bIDsBoolValueOutput = tFalse;
		m_bIDsStartSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult testPullOutFilter::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));    

    RETURN_NOERROR;
}

tResult testPullOutFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult testPullOutFilter::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult testPullOutFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

void testPullOutFilter::transmit1() {
	OnTransmitIntValue(CROSS_LEFT);
}
void testPullOutFilter::transmit2() {
	OnTransmitIntValue(CROSS_RIGHT);
}
void testPullOutFilter::transmit3() {
	OnTransmitIntValue(PARALLEL_RIGHT);
}

void testPullOutFilter::OnTransmitBoolValue()
{	
	LOG_INFO(cString::Format("TPOF: send nextSpotFree") ); 
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = tTrue;
    tUInt32 ui32TimeStamp = 0;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);    

        // set the id if not already done
        if(!m_bIDsBoolValueOutput)
        {
            pCoderOutput->GetID("bValue", m_szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDArduinoTimestampOutput);
            m_bIDsBoolValueOutput = tTrue;
        }      

        // set value from sample
        pCoderOutput->Set(m_szIDBoolValueOutput, (tVoid*)&bValue);     
        pCoderOutput->Set(m_szIDArduinoTimestampOutput, (tVoid*)&(ui32TimeStamp));     
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    //transmit media sample over output pin
    m_oNextSpotFree.Transmit(pMediaSample);

}

void testPullOutFilter::OnTransmitIntValue(tInt32 pullOutType)
{

	LOG_INFO(cString::Format("TPOF: Start  pullOutType1 %i", pullOutType) ); 
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pStartInput->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pStartInput, pMediaSample, pCoder);    

		if (!m_bIDsStartSet) {
			pCoder->GetID("intValue", m_szIDManeuvreStartInput);
			m_bIDsStartSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDManeuvreStartInput, (tVoid*)&pullOutType);
	}

	LOG_INFO(cString::Format("TPOF: Start  pullOutType2 %i", pullOutType) ); 
 //transmit media sample over output pin
    m_oStartPullOut.Transmit(pMediaSample);
LOG_INFO(cString::Format("TPOF: Start  pullOutType3 %i", pullOutType) ); 
}
