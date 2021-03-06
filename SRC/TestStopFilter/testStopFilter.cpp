/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/
#include "stdafx.h"
#include "testStopFilter.h"


ADTF_FILTER_PLUGIN("Test Stop Filter", __guid, testStopFilter);




testStopFilter::testStopFilter(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info)
{
    SetPropertyInt("Occasion",0);
    SetPropertyBool("Occasion" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("Occasion" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("Occasion" NSSUBPROP_DESCRIPTION, "0: turning, 1: parking"); 

    SetPropertyInt("Type",0);
    SetPropertyBool("Type" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("Type" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("Type" NSSUBPROP_DESCRIPTION, "Turning type"); 

    SetPropertyInt("Mode",0);
    SetPropertyBool("Mode" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("Mode" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("Mode" NSSUBPROP_DESCRIPTION, "Turning mode"); 

    SetPropertyInt("GiveWay",0);
    SetPropertyBool("GiveWay" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("GiveWay" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("GiveWay" NSSUBPROP_DESCRIPTION, "Turning give way"); 

    SetPropertyBool("StopSign",tFalse);
    SetPropertyBool("StopSign" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool("StopSign" NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr("StopSign" NSSUBPROP_DESCRIPTION, "Turning stop sign"); 
}

testStopFilter::~testStopFilter()
{
}

tHandle testStopFilter::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);            

    // make the qt connections
    connect(m_pWidget->m_btSendValueFalse, SIGNAL(clicked()), this, SLOT(OnTransmitValueFalse()));
    connect(m_pWidget->m_btSendValueTrue, SIGNAL(clicked()), this, SLOT(OnTransmitValueTrue()));

    return (tHandle)m_pWidget;
}

tResult testStopFilter::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult testStopFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)    
    {        
        //get the media description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
      
		// MediaDescription of StopFilter-Struct
		tChar const* strDescStopStruct = pDescManager->GetMediaDescription("tStopStruct");
		RETURN_IF_POINTER_NULL(strDescStopStruct);
		cObjectPtr<IMediaType> pTypeStopStruct = new cMediaType(0, 0, 0, "tStopStruct", strDescStopStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description TurningState
        tChar const * TurningStateDesc = pDescManager->GetMediaDescription("tTurningStopStruct");
        RETURN_IF_POINTER_NULL(TurningStateDesc);
        cObjectPtr<IMediaType> pTypeTurningStateDesc = new cMediaType(0, 0, 0, "tTurningStopStruct", TurningStateDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        
  		// StopFilter Structure
		RETURN_IF_FAILED(m_oStopStructOutputPin.Create("StopStructParking", pTypeStopStruct, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopStructOutputPin));
		RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionStopStruct)); 

		// State of Turning (Output)
		RETURN_IF_FAILED(m_oStateTurning.Create("StopStructTurning", pTypeTurningStateDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateTurning));
		RETURN_IF_FAILED(pTypeTurningStateDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateTurningOutput));

     
       
        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
        // media descriptions ids not set by now
        m_bIDStopStructOutput = tFalse;
		m_bTurningOutputSet = tFalse;
		m_intOccasion = static_cast<tInt32>(GetPropertyFloat("Occasion"));
    }
    RETURN_NOERROR;
}

tResult testStopFilter::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));    

    RETURN_NOERROR;
}

tResult testStopFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult testStopFilter::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult testStopFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult testStopFilter::PropertyChanged(const tChar* strName)
{
    if (cString::IsEqual("Occasion", strName))
    {
		m_intOccasion = static_cast<tInt32>(GetPropertyFloat("Occasion"));
    } else if (cString::IsEqual("Type", strName)) {
		m_type = static_cast<tInt32>(GetPropertyFloat("Type"));
	} else if (cString::IsEqual("Mode", strName)) {
		m_mode = static_cast<tInt32>(GetPropertyFloat("Mode"));
	} else if (cString::IsEqual("GiveWay", strName)) {
		m_giveWay = static_cast<tInt32>(GetPropertyFloat("GiveWay"));
	} else if (cString::IsEqual("StopSign", strName)) {
		m_stopSign = static_cast<tBool>(GetPropertyFloat("StopSign"));
	}

    RETURN_NOERROR;
}


void testStopFilter::OnTransmitValueFalse()
{
}

void testStopFilter::OnTransmitValueTrue()
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
  

	tFloat32 	velocity = 1.0f;
	tFloat32 	distance = 1.0f;  
    tBool bValue = tTrue;


	if (m_intOccasion == 0) {// turning

		m_pStateTurningOutput->GetMediaSampleSerializer(&pSerializer);
    	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

   		//write date to the media sample with the coder of the descriptor
    	{
       		__adtf_sample_write_lock_mediadescription(m_pStateTurningOutput, pMediaSample, pCoderOutput);    

        	// set the id if not already done
        	if(!m_bTurningOutputSet)
        	{
				pCoderOutput->GetID("mode", m_szIDTurningStructModeOutput);
				pCoderOutput->GetID("type", m_szIDTurningStructTypeOutput);
				pCoderOutput->GetID("giveWay", m_szIDTurningStructGiveWayOutput);
				pCoderOutput->GetID("stopSign", m_szIDTurningStructStopSignOutput);
				pCoderOutput->GetID("fDistance", m_szIDTurningStructDistanceOutput);
				pCoderOutput->GetID("fVelocity", m_szIDTurningStructVelocityOutput);
				pCoderOutput->GetID("bStart", m_szIDTurningStructStatusOutput);


            	m_bTurningOutputSet = tTrue;
        	}      
            
        	// set value from sample
        	pCoderOutput->Set(m_szIDTurningStructModeOutput, (tVoid*)&m_mode);     
   			pCoderOutput->Set(m_szIDTurningStructTypeOutput, (tVoid*)&m_type); 
			pCoderOutput->Set(m_szIDTurningStructGiveWayOutput, (tVoid*)&m_giveWay); 
			pCoderOutput->Set(m_szIDTurningStructStopSignOutput, (tVoid*)&m_stopSign); 
			pCoderOutput->Set(m_szIDTurningStructDistanceOutput, (tVoid*)&distance); 
			pCoderOutput->Set(m_szIDTurningStructVelocityOutput, (tVoid*)&velocity); 
			pCoderOutput->Set(m_szIDTurningStructStatusOutput, (tVoid*)&bValue); 
    	}

    	pMediaSample->SetTime(_clock->GetStreamTime());
    
    	//transmit media sample over output pin
    	m_oStateTurning.Transmit(pMediaSample);

	} else if (m_intOccasion == 1) { // parking

		m_pDescriptionStopStruct->GetMediaSampleSerializer(&pSerializer);
    	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    	//write date to the media sample with the coder of the descriptor
    	{
       		__adtf_sample_write_lock_mediadescription(m_pDescriptionStopStruct, pMediaSample, pCoderOutput);    

        	// set the id if not already done
        	if(!m_bIDStopStructOutput)
        	{
				pCoderOutput->GetID("fVelocity", m_szIDVelocityStructOutput);
				pCoderOutput->GetID("fDistance", m_szIDDistanceStructOutput);
				pCoderOutput->GetID("bStart", m_szIDBoolStructOutput);

            	m_bIDStopStructOutput = tTrue;
        	}      
            
        	// set value from sample
        	pCoderOutput->Set(m_szIDVelocityStructOutput, (tVoid*)&velocity);     
   			pCoderOutput->Set(m_szIDDistanceStructOutput, (tVoid*)&distance); 
			pCoderOutput->Set(m_szIDBoolStructOutput, (tVoid*)&bValue); 
    	}

    	pMediaSample->SetTime(_clock->GetStreamTime());
    
    	//transmit media sample over output pin
    	m_oStopStructOutputPin.Transmit(pMediaSample);
	}
}
