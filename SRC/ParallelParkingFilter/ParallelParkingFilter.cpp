/*
 * Date 17.03.2015
 */ 
/**
Copyright (c)                                                                                     
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#include <math.h>
#include "stdafx.h"
#include "ParallelParkingFilter.h"

#include <fstream>


ADTF_FILTER_PLUGIN("ParallelParkingFilter", __guid, ParallelParkingFilter);




ParallelParkingFilter::ParallelParkingFilter(const tChar* __info): m_hazzardON(false), m_turnRightON(false), m_reverseON(false), m_finishParking(false), m_isParking(false)
{
    SetPropertyInt("Actuator Update Rate [Hz]",30);
    SetPropertyStr("Actuator Update Rate [Hz]" NSSUBPROP_DESCRIPTION, "Defines how much updates for steering and speed controller are sent in one second (Range: 0 to 100 Hz)"); 
    SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MIN, 0); 
    SetPropertyInt("Actuator Update Rate [Hz]" NSSUBPROP_MAX, 100);

    SetPropertyFloat("Default Speed",0);
    SetPropertyStr("Default Speed" NSSUBPROP_DESCRIPTION, "Defines the default speed which is transmitted when nothing is read from the list"); 
    SetPropertyBool("Default Speed" NSSUBPROP_ISCHANGEABLE, tTrue); 

    SetPropertyFloat("Default Steering",90);
    SetPropertyStr("Default Steering" NSSUBPROP_DESCRIPTION, "Defines the default steering which is transmitted when nothing is read from the list"); 
    SetPropertyFloat("Default Steering" NSSUBPROP_MIN, 60);
    SetPropertyFloat("Default Steering" NSSUBPROP_MAX, 120);
    SetPropertyBool("Default Steering" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat("Time Stretch Factor",1.615);
    SetPropertyStr("Time Stretch Factor" NSSUBPROP_DESCRIPTION, "Time Stretch Factor");
    SetPropertyFloat("Time Stretch Factor" NSSUBPROP_MIN, 0.0);
    SetPropertyBool("Time Stretch Factor" NSSUBPROP_ISCHANGEABLE, tTrue);

    calc();
}

ParallelParkingFilter::~ParallelParkingFilter()
{
}

void ParallelParkingFilter::calc() {

    fX1 = 0.0;
    fY1 = -m_sProperties.fRmin;

    tFloat32 faux = sqrt((m_sProperties.fb+m_sProperties.fRmin)*(m_sProperties.fb+m_sProperties.fRmin)+m_sProperties.fl*m_sProperties.fl);

    fAlpha1=asin(m_sProperties.fl/faux);
    fAlpha1=asin((fX1-m_sProperties.fXmin)/(m_sProperties.fRmin+m_sProperties.fb))-fAlpha1;

    fX2=fX1-2.0*sin(fAlpha1)*m_sProperties.fRmin;
    fY2=fY1+2.0*cos(fAlpha1)*m_sProperties.fRmin;

    fY3=m_sProperties.fYc-m_sProperties.fRmin;
    fX3=fX2+sqrt(4.0*m_sProperties.fRmin*m_sProperties.fRmin-(fY2-fY3)*(fY2-fY3));

    fXi=fX3-(fX3-fX2)/2.0;
    fYi=(fY2-fY3)/2.0+fY3;

    fX2i=fX1-sin(fAlpha1)*m_sProperties.fRmin;
    fY2i=fY1+cos(fAlpha1)*m_sProperties.fRmin;

    // auxiliary memory
    faux=sqrt((fXi-fX2i)*(fXi-fX2i)+(fYi-fY2i)*(fYi-fY2i));
    fAlpha2=acos((2.0*m_sProperties.fRmin*m_sProperties.fRmin-faux*faux)/(2.0*m_sProperties.fRmin*m_sProperties.fRmin));

    fDistance[2]=m_sProperties.fRmin*fAlpha1;
    fDistance[1]=m_sProperties.fRmin*fAlpha2;

    fAlpha3=asin((fY2-fY3)/(2.0*m_sProperties.fRmin));
    fAlpha3=M_PI/2.0-fAlpha3;

    fDistance[0]=m_sProperties.fRmin*fAlpha3;

    faux = m_sProperties.fMinAcc-m_sProperties.fMaxAcc;

    //tf_1
    fSwitchingPointsOrig[0]=-m_sProperties.fv/m_sProperties.fMaxAcc+sqrt(m_sProperties.fv*m_sProperties.fv/(m_sProperties.fMaxAcc*m_sProperties.fMaxAcc)+(2.0*fDistance[0]*m_sProperties.fMinAcc+m_sProperties.fv*m_sProperties.fv)/(m_sProperties.fMinAcc*faux-faux*faux));
		
    //te_1
    fSwitchingPointsOrig[1]=(-m_sProperties.fv+faux*fSwitchingPointsOrig[0])/m_sProperties.fMinAcc;

    //tf_2
    fSwitchingPointsOrig[2]=sqrt(2.0*fDistance[1]*m_sProperties.fMinAcc/(m_sProperties.fMinAcc*faux-faux*faux));

    //te_2
    fSwitchingPointsOrig[3]=(faux*fSwitchingPointsOrig[2])/m_sProperties.fMinAcc;

    //tf_3
    fSwitchingPointsOrig[4]=sqrt(2.0*fDistance[2]*m_sProperties.fMinAcc/(m_sProperties.fMinAcc*faux-faux*faux));

    //te_3
    fSwitchingPointsOrig[5]=(faux*fSwitchingPointsOrig[4])/m_sProperties.fMinAcc;

    fSwitchingPointsOrig[2]+=fSwitchingPointsOrig[1];
    fSwitchingPointsOrig[3]+=fSwitchingPointsOrig[1];
    fSwitchingPointsOrig[4]+=fSwitchingPointsOrig[3];
    fSwitchingPointsOrig[5]+=fSwitchingPointsOrig[3];


}


tResult ParallelParkingFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)    {

		//get the description manager for this filter
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        //get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Get description for bool values
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);


        //get mediatype description for output data type
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutput2));

        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStartParkingInput));


        //create input pin for starting
    	RETURN_IF_FAILED(m_iStartParking.Create("StartParallelParking", pTypeBoolSignalValue, this));
    	RETURN_IF_FAILED(RegisterPin(&m_iStartParking));

        //create output pin for speed output data
        RETURN_IF_FAILED(m_oSpeedOutput.Create("Speed", pTypeSignalValue, 	static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oSpeedOutput));

        //create output pin for steering output data
        RETURN_IF_FAILED(m_oSteeringOutput.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringOutput));	

        //create output pin for hazzard light output data
        RETURN_IF_FAILED(m_oHazzardLightOutput.Create("Hazzard_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oHazzardLightOutput));	

        //create output pin for turn-right light output data
        RETURN_IF_FAILED(m_oTurnRightOutput.Create("turn-right_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnRightOutput));

        //create output pin for reverse light output data
        RETURN_IF_FAILED(m_oReverseLightOutput.Create("reverse_Light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oReverseLightOutput));

		//create output pin for finish parking output data
        RETURN_IF_FAILED(m_oFinishParkingOutput.Create("finish_Parking", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oFinishParkingOutput));
    	
        RETURN_NOERROR;
    }
    else if(eStage == StageNormal)
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Speed"));
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Steering"));
        m_fTSF = static_cast<tFloat32>(GetPropertyFloat("Time Stretch Factor"));
    }
    else if(eStage == StageGraphReady)
    {
        // no ids were set so far
        m_bIDsOutputSet = tFalse;
        m_bIDsOutput2Set = tFalse;
		m_bIDsBoolValueOutput = tFalse;
		m_tInt32StartTimeStamp = 0;
		//m_hazzardON = tFalse;
		//m_turnRightON = tFalse;
		//m_reverseON = tFalse;
		//m_finishParking = tFalse;
		//m_isParking = tFalse;
    }
    RETURN_NOERROR;
}


tResult ParallelParkingFilter::PropertyChanged(const tChar* strName)
{
    //read from property if it was changed
    if (cString::IsEqual("Default Speed", strName))
    {
        m_f32DefaultValue1 = static_cast<tFloat32>(GetPropertyFloat("Default Speed"));
    }
    else if (cString::IsEqual("Default Steering", strName))
    {
        m_f32DefaultValue2 = static_cast<tFloat32>(GetPropertyFloat("Default Steering"));
    }
    else if (cString::IsEqual("Time Stretch Factor", strName))
    {  
		//fstream f;
		//f.open("parallelParkingSwitchingPoints",ios::out);

	 	m_fTSF = static_cast<tFloat32>(GetPropertyFloat("Time Stretch Factor"));
		for (int i=0;i<6;i++){
			fSwitchingPoints[i] = fSwitchingPointsOrig[i] * m_fTSF;
			//f << fSwitchingPoints[i] << "\n";
		}
		//f.close();
    }

    RETURN_NOERROR;
}
tResult ParallelParkingFilter::Start(__exception)
{
    RETURN_IF_FAILED(cFilter::Start(__exception_ptr));

    //create the timer for the transmitting actuator values
    tTimeStamp tmPeriod = tTimeStamp(1/float(GetPropertyInt("Actuator Update Rate [Hz]"))*1000000);
    m_hTimerOutput = _kernel->TimerCreate(tmPeriod, GetPropertyInt("Actuator Startup Time Delay [sec]")*1000000, static_cast<IRunnable*>(this),
        NULL, &m_hTimerOutput, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult ParallelParkingFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
	if (!m_isParking)
	{	
		return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);	
	}
    if (nActivationCode == IRunnable::RUN_TIMER)
    {        
        // actuator timer was called, time to transmit actuator samples
        if (pvUserData==&m_hTimerOutput)
        {

            //create new media sample for steering controller	
            {                
                cObjectPtr<IMediaSample> pMediaSampleValue1;	
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue1));

                //allocate memory with the size given by the descriptor	
                cObjectPtr<IMediaSerializer> pSerializer1;	
                m_pDescriptionOutput->GetMediaSampleSerializer(&pSerializer1);
                tInt nSize = pSerializer1->GetDeserializedSize();	
                RETURN_IF_FAILED(pMediaSampleValue1->AllocBuffer(nSize));	
                tFloat32 value1 = getCurrentValue(_clock->GetStreamTime(),1);
                {   // focus for sample write lock	
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput,pMediaSampleValue1,pCoder);
                    // get the IDs for the items in the media sample 
                    if(!m_bIDsOutputSet)
                    {
                        pCoder->GetID("f32Value", m_szIDOutputF32Value);
                        pCoder->GetID("ui32ArduinoTimestamp", m_szIDOutputArduinoTimestamp);
                        m_bIDsOutputSet = tTrue;
                    }   
                    pCoder->Set(m_szIDOutputF32Value, (tVoid*)&value1);
                }

                //transmit media sample over output pin

                RETURN_IF_FAILED(pMediaSampleValue1->SetTime(_clock->GetStreamTime()));


                RETURN_IF_FAILED(m_oSpeedOutput.Transmit(pMediaSampleValue1));
            }
            //create new media sample for speed controller	
            {                
                cObjectPtr<IMediaSample> pMediaSampleValue2;	
                RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleValue2));

                //allocate memory with the size given by the descriptor	
                cObjectPtr<IMediaSerializer> pSerializerSpeed;	
                m_pDescriptionOutput2->GetMediaSampleSerializer(&pSerializerSpeed);
                tInt nSizeSpeed = pSerializerSpeed->GetDeserializedSize();	
                RETURN_IF_FAILED(pMediaSampleValue2->AllocBuffer(nSizeSpeed));	

                tFloat32 value2 = getCurrentValue(_clock->GetStreamTime(),2);

                {   // focus for sample write lock	
                    //write date to the media sample with the coder of the descriptor
                    __adtf_sample_write_lock_mediadescription(m_pDescriptionOutput2,pMediaSampleValue2,pCoder2);

                    // get the IDs for the items in the media sample 
                    if(!m_bIDsOutput2Set)
                    {
                        pCoder2->GetID("f32Value", m_szIDOutput2F32Value);
                        pCoder2->GetID("ui32ArduinoTimestamp", m_szIDOutput2ArduinoTimestamp);
                        m_bIDsOutput2Set = tTrue;
                    }      

                    pCoder2->Set(m_szIDOutput2F32Value, (tVoid*)&value2);
                }

                //transmit media sample over output pin	
                RETURN_IF_FAILED(pMediaSampleValue2->SetTime(_clock->GetStreamTime()));
                RETURN_IF_FAILED(m_oSteeringOutput.Transmit(pMediaSampleValue2));
            }
        }

    }
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}


tResult ParallelParkingFilter::Stop(__exception)
{       
    if (m_hTimerOutput) 
    {
        _kernel->TimerDestroy(m_hTimerOutput);
        m_hTimerOutput = NULL;
    }

    RETURN_IF_FAILED(cFilter::Stop(__exception_ptr));

    RETURN_NOERROR;
}

tResult ParallelParkingFilter::Shutdown(tInitStage eStage, __exception)
{ 
    return cFilter::Shutdown(eStage, __exception_ptr);
}


//void ParkingFilter::OnSendTuple(int iStateFlag, float fTime, float fValue1, float fValue2)
void ParallelParkingFilter::parallelParking(tBool start)
{

    if (start && !m_isParking) {
		m_finishParking = false;
    	m_tInt32StartTimeStamp = _clock->GetStreamTime();
		LOG_INFO(cString::Format("%f| %f| %f| %f| %f| %f",fSwitchingPoints[0],fSwitchingPoints[1],fSwitchingPoints[2],fSwitchingPoints[3],fSwitchingPoints[4],fSwitchingPoints[5]));
	}
	m_isParking=start;
}

tFloat32 ParallelParkingFilter::getDefaultValue(tInt8 i8ValueID)
{
    switch (i8ValueID)
    {
    case 1:
        return m_f32DefaultValue1;
        break;
    case 2:
        return m_f32DefaultValue2;
        break;
    default:                
        return m_f32DefaultValue1;
        break;
    }
}

tFloat32 ParallelParkingFilter::getCurrentValue(tFloat32 fTime, tInt8 i8ValueID)
{

	// list empty or output disabled, return default
    if (m_isParking == tFalse) {
		return getDefaultValue(i8ValueID);
	}

    // turn on turnRight-light
    if (!m_turnRightON) {
        TransmitBoolValue(&m_oTurnRightOutput, true);
        m_turnRightON = true;
    }

    //get time difference in seconds
    fTime = (_clock->GetStreamTime()-m_tInt32StartTimeStamp)/1000000.0f;

//Parallel Parking
    switch (i8ValueID)
    {
    case 1: {
        if (fTime>=0 && fTime<fSwitchingPoints[0]) {
                // turn on reverse-light
                if (!m_reverseON) {
                        TransmitBoolValue(&m_oReverseLightOutput, true);
                        m_reverseON = true;
                }
                return m_sProperties.fMinAcc*fTime+m_sProperties.fv;
        } else if (fTime<fSwitchingPoints[1]) {
                return m_sProperties.fMaxAcc*fTime-m_sProperties.fMaxAcc*fSwitchingPoints[1];
        } else if (fTime<(fSwitchingPoints[2])) {
                return m_sProperties.fMinAcc*(fTime)-m_sProperties.fMinAcc*fSwitchingPoints[1];
        } else if (fTime<(fSwitchingPoints[3])) {
                return m_sProperties.fMaxAcc*(fTime)-m_sProperties.fMaxAcc*(fSwitchingPoints[3]);
        } else if (fTime<(fSwitchingPoints[4])) {
                // turn off reverse-light
                if (m_reverseON) {
						m_reverseON = false;
                        TransmitBoolValue(&m_oReverseLightOutput, false);
                }
                return m_sProperties.fMaxAcc*(fTime)-m_sProperties.fMaxAcc*(fSwitchingPoints[3]);
        } else if (fTime<(fSwitchingPoints[5])) {
                return m_sProperties.fMinAcc*(fTime)-m_sProperties.fMinAcc*(fSwitchingPoints[5]);
        } else if (fTime<(fSwitchingPoints[5])+3.0) {


                // turn on hazzard-light
                if (!m_hazzardON) {
                	TransmitBoolValue(&m_oHazzardLightOutput, true);
					m_hazzardON = true;
                }

                
                return 0.0;
        } else  {

                // turn off hazzard-light
                if (m_hazzardON) {
						m_hazzardON = false;
                        TransmitBoolValue(&m_oHazzardLightOutput, false);
                }

				// turnRight-light switch off while hazzard light is on
				m_turnRightON = false;


				m_isParking=false;
				if (!m_finishParking) {
					TransmitBoolValue(&m_oFinishParkingOutput, true);
					m_finishParking = true;
				}

                return 0.0;
        }
        break;}
    case 2: {
        if (fTime>=0 && fTime<fSwitchingPoints[1]) {
                return 120;
        } else if (fTime<fSwitchingPoints[3]){
                return 60;
        } else if (fTime<fSwitchingPoints[5]){
                return 120;
        } else {
                return 90;
        }
        break;}
    }

    //should never happen (only if valueID is not 1 or 2:
    return m_f32DefaultValue1;
}

tResult ParallelParkingFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// so we received a media sample, so this pointer better be valid.
    	RETURN_IF_POINTER_NULL(pMediaSample);

        if (&m_iStartParking == pSource) {
	
                tBool bValue = tFalse;				

				{
					__adtf_sample_read_lock_mediadescription(m_pStartParkingInput ,pMediaSample, pCoder);    

					pCoder->GetID("bValue", m_szIDBoolValueStartParkingInput);
					pCoder->GetID("ui32ArduinoTimestamp",  m_szIDTimestampStartParkingInput);    		
					pCoder->Get( m_szIDBoolValueStartParkingInput, (tVoid*)&bValue);         
				}

			if (bValue) LOG_INFO(cString::Format("PPF: Start Parallel Parking"));
			else 		LOG_INFO(cString::Format("PPF: Shutdown Parallel Parking"));

			parallelParking(bValue);
			
		}
	}
    

    RETURN_NOERROR;
}


tResult ParallelParkingFilter::TransmitBoolValue(cOutputPin* oPin, bool value)
{
	
	//__synchronized_obj(m_oTransmitBoolCritSection);
	
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

