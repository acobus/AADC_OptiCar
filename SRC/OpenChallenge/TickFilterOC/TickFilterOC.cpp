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

//17.03.16

// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include <math.h>
#include "stdafx.h"
#include "TickFilterOC.h"
#include <fstream>


#define CW_SLOT_COUNT 60.f
#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f

ADTF_FILTER_PLUGIN("Tick Filter OC", OID_ADTF_TICK_FILTER_OC, TickFilterOC)

    TickFilterOC::TickFilterOC(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Wheel circumference",0.34);
    SetPropertyFloat("Wheel circumference" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Wheel circumference" NSSUBPROP_DESCRIPTION, "Set the wheel circumference in meter here"); 
 
	m_OffsetLeft = 0;
	m_OffsetRight = 0;

}

TickFilterOC::~TickFilterOC()
{
}

tResult TickFilterOC::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
            cObjectPtr<IMediaDescriptionManager> pDescManager;
            RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

            tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
            RETURN_IF_POINTER_NULL(strDescSignalValue);        
            cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    


            //get description for wheel sensors data pins
            tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");	
            RETURN_IF_POINTER_NULL(strDescWheelData);
            //get mediatype for wheeldata sensor data pins
            cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			// Get description for bool values
			tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
			RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
			cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			//create input pin for starting
    		RETURN_IF_FAILED(m_iStart.Create("Start", pTypeBoolSignalValue, this));
    		RETURN_IF_FAILED(RegisterPin(&m_iStart));

            //create pin for wheel left sensor data
            RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeft_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

            //create pin for wheel right data
            RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRight_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));


            //create pin for distance overall data
            RETURN_IF_FAILED(m_oOutputDistanceOverall.Create("distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDistanceOverall));


            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputOverallDistance));


		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

            //get mediatype description for wheel sensor data type
            RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataRight));

            RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataLeft));
        }
        else if (eStage == StageNormal)
        {
            m_f32wheelCircumference = static_cast<tFloat32>(GetPropertyFloat("wheel circumference"));

			InitialState();

        }
        else if(eStage == StageGraphReady)
        {    
            m_bIDsWheelDataLeftSet = tFalse;
            m_bIDsWheelDataRightSet = tFalse;
            m_bIDsOverallDistanceSet = tFalse;
        }

        RETURN_NOERROR;
}

tResult TickFilterOC::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {

		if (pSource == &m_iStart){
			tBool bValue = tFalse;

			{
				__adtf_sample_read_lock_mediadescription(m_pDescriptionBool ,pMediaSample, pCoder);    

				pCoder->GetID("bValue", m_szIDBoolValueOutput);
 		
				pCoder->Get( m_szIDBoolValueOutput, (tVoid*)&bValue);         
			}

			if ((!m_filterActive  && bValue)){ 	// activate filter
				InitialState();					// delete old values
				m_filterActive = tTrue;
			} else if (!bValue){  				// stop filter
				m_filterActive = tFalse;
			}
			
		} else if (pSource == &m_oInputWheelLeft && m_filterActive)
        {
            // save the last struct to the struct beforeLast if it is not the first one
            if (m_bfirstSampleReceivedLeftWheel==tTrue)
            {
                m_tBeforeLastStructLeft = m_tLastStructLeft;
            }            
            
            tUInt32 ui32Tach = 0;
            tInt8 i8Direction = 0;
            tUInt32 ui32Timestamp = 0;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataLeft,pMediaSample,pCoderInput);

                // get IDs
                if(!m_bIDsWheelDataLeftSet)
                {
                    pCoderInput->GetID("i8WheelDir",m_szIDWheelDataLeftI8WheelDir);
                    pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataLeftUi32WheelTach);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataLeftArduinoTimestamp);
                    m_bIDsWheelDataLeftSet = tTrue;
                }  
                //get values from media sample    

                //get values from media sample        
                pCoderInput->Get(m_szIDWheelDataLeftUi32WheelTach, (tVoid*)&ui32Tach);        
                pCoderInput->Get(m_szIDWheelDataLeftI8WheelDir, (tVoid*)&i8Direction);  
                pCoderInput->Get(m_szIDWheelDataLeftArduinoTimestamp, (tVoid*)&ui32Timestamp);            
            }

            // if it is the first sample stop here and set to true
            if (m_bfirstSampleReceivedLeftWheel==tFalse)
            {
                m_bfirstSampleReceivedLeftWheel = tTrue;

				m_OffsetLeft = ui32Tach;
                
                m_tLastStructLeft.i8WheelDir = i8Direction;
                m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructLeft.ui32WheelTach = ui32Tach;
            }
            // doing the calculation and the transmit
            else
            {            
            	

                m_tLastStructLeft.i8WheelDir = i8Direction;
                m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructLeft.ui32WheelTach = ui32Tach;

            }

        }
        else if (pSource == &m_oInputWheelRight && m_filterActive)
        {                                    
            // save the last struct to the struct beforeLast if it is not the first one
            if (m_bfirstSampleReceivedRightWheel==tTrue)
            {
                m_tBeforeLastStructRight = m_tLastStructRight;
            }            

            tUInt32 ui32Tach = 0;
            tInt8 i8Direction = 0;
            tUInt32 ui32Timestamp = 0;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pDescriptionWheelDataRight,pMediaSample,pCoderInput);


                // get IDs
                if(!m_bIDsWheelDataRightSet)
                {
                    pCoderInput->GetID("i8WheelDir",m_szIDWheelDataRightI8WheelDir);
                    pCoderInput->GetID("ui32WheelTach", m_szIDWheelDataRightUi32WheelTach);
                    pCoderInput->GetID("ui32ArduinoTimestamp", m_szIDWheelDataRightArduinoTimestamp);
                    m_bIDsWheelDataRightSet = tTrue;
                }  
                //get values from media sample    
                pCoderInput->Get(m_szIDWheelDataRightUi32WheelTach, (tVoid*)&ui32Tach);        
                pCoderInput->Get(m_szIDWheelDataRightI8WheelDir, (tVoid*)&i8Direction);
                pCoderInput->Get(m_szIDWheelDataRightArduinoTimestamp, (tVoid*)&ui32Timestamp);              
            }

            // if it is the first sample stop here and set to true
            if (m_bfirstSampleReceivedRightWheel==tFalse)
            {
                m_bfirstSampleReceivedRightWheel = tTrue;

				m_OffsetRight = ui32Tach;
                
                m_tLastStructRight.i8WheelDir = i8Direction;
                m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructRight.ui32WheelTach = ui32Tach;
            }
            // doing the calculation and the transmit
            else
            {

               

                m_tLastStructRight.i8WheelDir = i8Direction;
                m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructRight.ui32WheelTach = ui32Tach;

				try{
	                TransmitSamples(); // Fehler?? (Andi)
				}catch(...){
					fstream f;
					f.open("TickFilterErrLog.dat", ios::out| ios::app);
					f << "TransmitSamples() failed";
					f.close();  
				}
            }                
        }
    }
    RETURN_NOERROR;
}

tTimeStamp TickFilterOC::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}


tResult TickFilterOC::TransmitSamples()	
{    
    // enter critical section for transmitting
    __synchronized_obj(m_oCritSectionTransmit);

   
    // distance since last sample
    tFloat32 f32distance = 0;
    tFloat32 f32distanceLeft = 0;
    tFloat32 f32distanceRight = 0;

    // calculate the overall distance
    // if the speed is negative (car is going backward, distance is decreasing)    
    if (m_tLastStructLeft.ui32ArduinoTimestamp!=0 && m_tLastStructRight.ui32ArduinoTimestamp!=0 && 
        m_tBeforeLastStructRight.ui32ArduinoTimestamp!=0 && m_tBeforeLastStructLeft.ui32ArduinoTimestamp !=0)
        {
                //                    (LeftTach + RightTach)
                //   distance =   ------------------------------ * Wheel Circ.
                //                            2 * 60
                f32distance = (m_tLastStructLeft.ui32WheelTach-m_OffsetLeft + m_tLastStructRight.ui32WheelTach-m_OffsetRight)/120.0 * m_f32wheelCircumference;
	        	//LOG_INFO(cString::Format("StructLeft: %i, OffsetLeft: %i", m_tLastStructLeft.ui32WheelTach, m_OffsetLeft));
		        //LOG_INFO(cString::Format("StructRight: %i, OffsetRight: %i", m_tLastStructRight.ui32WheelTach, m_OffsetRight));
	        	//LOG_INFO(cString::Format("Wheel: %f",m_f32wheelCircumference));
                m_f32OverallDistance = f32distance;

                
                f32distanceLeft = (m_tLastStructLeft.ui32WheelTach-m_OffsetLeft)/60.0 * m_f32wheelCircumference;
                f32distanceRight = (m_tLastStructRight.ui32WheelTach-m_OffsetRight)/60.0 * m_f32wheelCircumference;

                m_f32OverallDistanceLeft = f32distanceLeft;
                m_f32OverallDistanceRight = f32distanceRight;
        }



    //calculate the average of the arduino timestamp
    tUInt32 ui32arduinoTimestamp =(m_tLastStructLeft.ui32ArduinoTimestamp + m_tLastStructRight.ui32ArduinoTimestamp)/2;


    //create new media sample for overall distance
    cObjectPtr<IMediaSample> pMediaSampleOverallDistance;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOverallDistance));

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer1;	
    m_pDescriptionOutputOverallDistance->GetMediaSampleSerializer(&pSerializer1);
    tInt nSize = pSerializer1->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSampleOverallDistance->AllocBuffer(nSize));


    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputOverallDistance,pMediaSampleOverallDistance,pCoder1);

        // set the ids if not already done
        if(!m_bIDsOverallDistanceSet)
        {
            pCoder1->GetID("f32Value", m_szIDOverallDistanceF32Value);
            pCoder1->GetID("ui32ArduinoTimestamp", m_szIDOverallDistanceArduinoTimestamp);    
            m_bIDsOverallDistanceSet = tTrue;
        }

        // set the values
        pCoder1->Set(m_szIDOverallDistanceF32Value, (tVoid*)&m_f32OverallDistance);
        pCoder1->Set(m_szIDOverallDistanceArduinoTimestamp, (tVoid*)&ui32arduinoTimestamp);
    }

	/*fstream f7;
	f7.open("TickFilterOverallDistanceOut.dat", ios::out | ios::app);
	f7 << "m_f32OverallDistance = " << m_f32OverallDistance << "\tTimestamp = " << ui32arduinoTimestamp << "\n";
	f7.close(); */

    //transmit media sample over output pin
    pMediaSampleOverallDistance->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputDistanceOverall.Transmit(pMediaSampleOverallDistance));

	fstream f4;
	f4.open("Dist_Right.dat", ios::out | ios::app);
	f4 << m_f32OverallDistanceRight << "\n";
	f4.close();

    fstream f2;
    f2.open("Dist_Left.dat",ios::out | ios::app);
    f2<<m_f32OverallDistanceLeft << "\n" ;
    f2.close();

	
    /*fstream f3;
    f3.open("Dist_Rot.dat",ios::out | ios::app);
    f3<<((m_f32OverallDistanceLeft-m_f32OverallDistanceRight)/0.26)*180/M_PI<< "\n" ;
    f3.close();*/


    RETURN_NOERROR;
}

tResult TickFilterOC::InitialState(){

	fstream f4;
	f4.open("Dist_Right.dat", ios::out );
	f4 << "";
	f4.close();

    fstream f2;
    f2.open("Dist_Left.dat",ios::out );
    f2 << "" ;
    f2.close();

	LOG_INFO("TF: Datei geloescht.");

	m_f32OverallDistance         = 0.0f;
    m_f32OverallDistanceRight    = 0.0f;
    m_f32OverallDistanceLeft     = 0.0f;
    m_bfirstSampleReceivedLeftWheel = tFalse;
    m_bfirstSampleReceivedRightWheel = tFalse;
	m_filterActive = tFalse;

	RETURN_NOERROR;
}



