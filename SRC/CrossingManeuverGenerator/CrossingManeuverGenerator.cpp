/*
 * Date 20.12.2015
 */
#include "stdafx.h"
#include "CrossingManeuverGenerator.h"
#include "/home/aadc/AADC/src/aadcUser/include/action_enum.h"


ADTF_FILTER_PLUGIN("Crossing Maneuver Generator", __guid, CrossingManeuverGenerator);




CrossingManeuverGenerator::CrossingManeuverGenerator(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info)
{
}

CrossingManeuverGenerator::~CrossingManeuverGenerator()
{
}

tHandle CrossingManeuverGenerator::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);            

    // make the qt connections
    connect(m_pWidget->m_tSendStraight, SIGNAL(clicked()), this, SLOT(OnTransmitStraight()));
    connect(m_pWidget->m_btSendValueFalse, SIGNAL(clicked()), this, SLOT(OnTransmitValueFalse()));
    connect(m_pWidget->m_btSendValueTrue, SIGNAL(clicked()), this, SLOT(OnTransmitValueTrue()));
    connect(m_pWidget->m_tSendValueCircle, SIGNAL(clicked()), this, SLOT(OnTransmitCircle()));

    return (tHandle)m_pWidget;
}

tResult CrossingManeuverGenerator::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult CrossingManeuverGenerator::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)    
    {        
        //get the media description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
      
// MediaDescription of ImageProcessing-Struct
		tChar const* strDescCrossingStruct = pDescManager->GetMediaDescription("tStartCrossSearchingStruct");
		RETURN_IF_POINTER_NULL(strDescCrossingStruct);
		cObjectPtr<IMediaType> pTypeCrossingDesc = new cMediaType(0, 0, 0, "tStartCrossSearchingStruct", strDescCrossingStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        
  // Crossing Structure
		RETURN_IF_FAILED(m_oStopStructOutputPin.Create("CrossingStruct", pTypeCrossingDesc, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopStructOutputPin));
		RETURN_IF_FAILED(pTypeCrossingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionStopStruct)); 

     
       
        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
        // media descriptions ids not set by now
        m_bIDStopStructOutput = tFalse;
    }
    RETURN_NOERROR;
}

tResult CrossingManeuverGenerator::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));    

    RETURN_NOERROR;
}

tResult CrossingManeuverGenerator::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult CrossingManeuverGenerator::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult CrossingManeuverGenerator::Shutdown(tInitStage eStage, __exception)
{ 
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

void CrossingManeuverGenerator::OnTransmitCircle()
{
	cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
	m_pDescriptionStopStruct->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
     

	tBool status = tTrue; 
	tBool isCircle = tTrue;
	//not important for circle - set to some value
    tInt32 ID = ACTION_STRAIGHT;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionStopStruct, pMediaSample, pCoder);    

        // set the id if not already done
        if(!m_bIDStopStructOutput)
        {
			pCoder->GetID("ID", m_szIDSearchCrossingSpotOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchCrossingSpotOutput);
			pCoder->GetID("bIsCircleMode", m_szIDBoolValueIsCircle);

            m_bIDStopStructOutput = tTrue;
        }      
            
        // set value from sample
		pCoder->Set(m_szIDSearchCrossingSpotOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchCrossingSpotOutput, (tVoid*)&status);
		pCoder->Set(m_szIDBoolValueIsCircle, (tVoid*)&isCircle);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStopStructOutputPin.Transmit(pMediaSample);
}

void CrossingManeuverGenerator::OnTransmitStraight()
{
	cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
	m_pDescriptionStopStruct->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
     

	tBool status = tTrue; 
	tBool isCircle = tFalse;
    tInt32 ID = ACTION_STRAIGHT;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionStopStruct, pMediaSample, pCoder);    

        // set the id if not already done
        if(!m_bIDStopStructOutput)
        {
			pCoder->GetID("ID", m_szIDSearchCrossingSpotOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchCrossingSpotOutput);
			pCoder->GetID("bIsCircleMode", m_szIDBoolValueIsCircle);

            m_bIDStopStructOutput = tTrue;
        }      
            
        // set value from sample
		pCoder->Set(m_szIDSearchCrossingSpotOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchCrossingSpotOutput, (tVoid*)&status);
		pCoder->Set(m_szIDBoolValueIsCircle, (tVoid*)&isCircle);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStopStructOutputPin.Transmit(pMediaSample);
}

void CrossingManeuverGenerator::OnTransmitValueFalse()
{
	cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
	m_pDescriptionStopStruct->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
     

	tBool status = tTrue; 
	tBool isCircle = tFalse;
    tInt32 ID = ACTION_RIGHT;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionStopStruct, pMediaSample, pCoder);    

        // set the id if not already done
        if(!m_bIDStopStructOutput)
        {
			pCoder->GetID("ID", m_szIDSearchCrossingSpotOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchCrossingSpotOutput);
			pCoder->GetID("bIsCircleMode", m_szIDBoolValueIsCircle);

            m_bIDStopStructOutput = tTrue;
        }      
            
        // set value from sample
		pCoder->Set(m_szIDSearchCrossingSpotOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchCrossingSpotOutput, (tVoid*)&status);
		pCoder->Set(m_szIDBoolValueIsCircle, (tVoid*)&isCircle);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStopStructOutputPin.Transmit(pMediaSample);
}

void CrossingManeuverGenerator::OnTransmitValueTrue()
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    
	m_pDescriptionStopStruct->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
     

	tBool status = tTrue; 
	tBool isCircle = tFalse;
    tInt32 ID = ACTION_LEFT;

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionStopStruct, pMediaSample, pCoder);    

        // set the id if not already done
        if(!m_bIDStopStructOutput)
        {
			pCoder->GetID("ID", m_szIDSearchCrossingSpotOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchCrossingSpotOutput);
			pCoder->GetID("bIsCircleMode", m_szIDBoolValueIsCircle);

            m_bIDStopStructOutput = tTrue;
        }      
            
        // set value from sample
		pCoder->Set(m_szIDSearchCrossingSpotOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchCrossingSpotOutput, (tVoid*)&status);
		pCoder->Set(m_szIDBoolValueIsCircle, (tVoid*)&isCircle);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    
    //transmit media sample over output pin
    m_oStopStructOutputPin.Transmit(pMediaSample);
}
