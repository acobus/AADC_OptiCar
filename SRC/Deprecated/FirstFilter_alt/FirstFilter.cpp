#include "stdafx.h"
#include "FirstFilter.h"
#include <template_data.h>
#include <fstream>

/// Create filter shell
ADTF_FILTER_PLUGIN("First Filter", OID_ADTF_FIRST_FILTER, FirstFilter);

#define PROP_MIN_DISTANCE "Minimal Distance"

FirstFilter::FirstFilter(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat(PROP_MIN_DISTANCE, m_sProperties.fMinDistance);
    breakingModeFront = breakingModeRear = false;
    breakinglight = false; // decide if light on(true) or off(false)
}

FirstFilter::~FirstFilter()
{

}

tResult FirstFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // get a media type for the input pin
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager, __exception_ptr));
        
        // create and register the input pin
        RETURN_IF_FAILED(m_oUSFrontInput.Create("US_Front", new cMediaType(0,0,0,"tSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oUSFrontInput));

        RETURN_IF_FAILED(m_oUSRearInput.Create("US_Rear", new cMediaType(0,0,0,"tSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oUSRearInput));

        RETURN_IF_FAILED(m_oCarSpeedInput.Create("CarSpeed", new cMediaType(0,0,0,"tSignalValue"), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oCarSpeedInput));
    	
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
 	cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(m_oSpeedController.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oSpeedController));

        // get a media type for the output pin
	RETURN_IF_POINTER_NULL(strDescSignalValue);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));
        
        // create and register the output pin
        RETURN_IF_FAILED(m_oSpeedOut.Create("Speed_Output", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSpeedOut));

///////////////////////////////////////////////////////// 
	//get description for bool light sensors
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        RETURN_IF_FAILED(m_oBrakeLightOutputPin.Create("Breaklight", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oBrakeLightOutputPin));
////////////////////////////////////////////////////////
	//get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionActuatorOutput));
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
	m_sProperties.fMinDistance = GetPropertyFloat(PROP_MIN_DISTANCE);
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    }

    RETURN_NOERROR;
}

tResult FirstFilter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult FirstFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

	if (&m_oCarSpeedInput == pSource) {
	    ProcessCarSpeedInput(pMediaSample);
	}
	if (&m_oUSFrontInput == pSource) {
	    ProcessUSFrontInput(pMediaSample);
	}
	if (&m_oUSRearInput == pSource) {
	    ProcessUSRearInput(pMediaSample);
	}
	if (&m_oSpeedController == pSource) {
	    TransmitSpeed(pMediaSample);
	}
    }

    RETURN_NOERROR;
}

///////////////////////////////////////////////////////////////////////

tResult FirstFilter::ProcessCarSpeedInput(IMediaSample* pMediaSample) {

tFloat32 fSpeed = 0;

cObjectPtr<IMediaCoder> pCoder;
LOG_INFO(cString::Format("Lock2"));
RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

pCoder->Get("f32Value", (tVoid*)&(fSpeed));
LOG_INFO(cString::Format("UnLock2"));
m_pCoderDescSignal->Unlock(pCoder);

if (!breakingModeFront && !breakingModeRear) {
	if (fSpeed < 0) {
	    driveForward = false;
	} else {
	    driveForward = true;
	}
}

RETURN_NOERROR;

}


tResult FirstFilter::ProcessUSFrontInput(IMediaSample* pMediaSample) {

//tUInt32 nTimeStamp = 0;
tFloat32 fDistance = 0;

cObjectPtr<IMediaCoder> pCoder;
LOG_INFO(cString::Format("Lock3"));
RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

pCoder->Get("f32Value", (tVoid*)&(fDistance));
//pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&(nTimeStamp));
LOG_INFO(cString::Format("UnLock3"));
m_pCoderDescSignal->Unlock(pCoder);

/*
	fstream f;
	f.open("test.dat",ios::out|ios::app);
	f << fDistance << "\t" << m_sProperties.fMinDistance << "\t" << speedZero << "\n";
	f.close();	
*/

if (fDistance < m_sProperties.fMinDistance) {
    breakingModeFront = true;
    speedZeroFront = true;
} else {
    breakingModeFront = false;
    speedZeroFront = false;
}

RETURN_NOERROR;

}


tResult FirstFilter::ProcessUSRearInput(IMediaSample* pMediaSample) {

//tUInt32 nTimeStamp = 0;
tFloat32 fDistance = 0;

cObjectPtr<IMediaCoder> pCoder;
LOG_INFO(cString::Format("Lock4"));
RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

pCoder->Get("f32Value", (tVoid*)&(fDistance));
//pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&(nTimeStamp));
LOG_INFO(cString::Format("UnLock4"));
m_pCoderDescSignal->Unlock(pCoder);

if (fDistance < m_sProperties.fMinDistance) {
    breakingModeRear = true;
    speedZeroRear = true;
} else {
    breakingModeRear = false;
    speedZeroRear = false;
}

RETURN_NOERROR;

}


///////////////////////////////////////////////////////////////////////

tResult FirstFilter::TransmitSpeed(IMediaSample* pMediaSample) {

//fstream f;
//f.open("tollername2.dat",ios::out|ios::app);

//f << "transmitspeed" << "\n";


if ((speedZeroFront && driveForward) || (speedZeroRear && !driveForward)) {

    tUInt32 nTimeStamp = 0;

    cObjectPtr<IMediaCoder> pCoder;
// Following could cause errors!!!
//LOG_INFO(cString::Format("Lock5"));
    //RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoder));

    //pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&(nTimeStamp));
//LOG_INFO(cString::Format("UnLock5"));
    //m_pCoderDescSignal->Unlock(pCoder);

    tFloat32 fZeroSpeed = 90;
    cObjectPtr<IMediaSample> pNewSample;
    AllocMediaSample((tVoid**)&pNewSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pNewSample->AllocBuffer(nSize);

    cObjectPtr<IMediaCoder> pCoderOutput;
LOG_INFO(cString::Format("Lock1"));
    m_pCoderDescSignal->WriteLock(pNewSample, &pCoderOutput);

    pCoderOutput->Set("f32Value", (tVoid*)&(fZeroSpeed));
    pCoderOutput->Set("ui32ArduinoTimeStamp", (tVoid*)&nTimeStamp);
LOG_INFO(cString::Format("UnLock1"));
    m_pCoderDescSignal->Unlock(pCoderOutput);

    pNewSample->SetTime(pMediaSample->GetTime());
    m_oSpeedOut.Transmit(pNewSample);


////////////////////////////////////////////////////////////////////////////

    if (!breakinglight) {
	TransmitBoolValue(&m_oBrakeLightOutputPin, true);
        breakinglight = true;
    }
   //LOG_INFO(cString::Format("Bremsen!: %d", true));

////////////////////////////////////////////////////////////////////////////


} else {
/////////////////////////////////////////////////
    if (breakinglight) {
	TransmitBoolValue(&m_oBrakeLightOutputPin, false);
	breakinglight = false;
    }
    //LOG_INFO(cString::Format("nicht Bremsen: %d", false));
/////////////////////////////////////////////////
    m_oSpeedOut.Transmit(pMediaSample);
}

//TransmitBreaklight(pMediaSample);

//f.close();

RETURN_NOERROR;
}

////////////////7//////////////////////////
tResult FirstFilter::TransmitBoolValue(cOutputPin* oPin, bool value)
{
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    tBool bValue = value;
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
    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}

/////////////////////////////////////////////////////7

