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
#include "TickFilter.h"
#include <fstream>


#define CW_SLOT_COUNT 60.f
#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f

ADTF_FILTER_PLUGIN("Tick Filter", OID_ADTF_TICK_FILTER, TickFilter)

    TickFilter::TickFilter(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("Wheel circumference",0.34);
    SetPropertyFloat("Wheel circumference" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Wheel circumference" NSSUBPROP_DESCRIPTION, "Set the wheel circumference in meter here"); 
 
	m_OffsetLeft = 0;
	m_OffsetRight = 0;

}

TickFilter::~TickFilter()
{
}

tResult TickFilter::Init(tInitStage eStage, __exception)
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


            //create pin for wheel left sensor data
            RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeft_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

            //create pin for wheel right data
            RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRight_Struct", pTypeWheelData, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));


            //create pin for wheel right data
            RETURN_IF_FAILED(m_oOutputCarSpeed.Create("car_speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oOutputCarSpeed));

            //create pin for distance overall data
            RETURN_IF_FAILED(m_oOutputDistanceOverall.Create("distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));	
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDistanceOverall));

            //create pin for distance overall of wheel right data
            RETURN_IF_FAILED(m_oOutputDistanceOverallRight.Create("distance_overall_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDistanceOverallRight));

            //create pin for distance overall of wheel left data
            RETURN_IF_FAILED(m_oOutputDistanceOverallLeft.Create("distance_overall_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
            RETURN_IF_FAILED(RegisterPin(&m_oOutputDistanceOverallLeft));

            //get mediatype description for output
            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputSpeed));

            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputOverallDistance));

            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputOverallDistanceRight));

            RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionOutputOverallDistanceLeft));

            //get mediatype description for wheel sensor data type
            RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataRight));

            RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelDataLeft));
        }
        else if (eStage == StageNormal)
        {
            m_f32wheelCircumference = static_cast<tFloat32>(GetPropertyFloat("wheel circumference"));

            m_f32OverallDistance         = 0.0f;
            m_f32OverallDistanceRight    = 0.0f;
            m_f32OverallDistanceLeft     = 0.0f;
            m_bfirstSampleReceivedLeftWheel = tFalse;
            m_bfirstSampleReceivedRightWheel = tFalse;
        }
        else if(eStage == StageGraphReady)
        {    
            m_bIDsWheelDataLeftSet = tFalse;
            m_bIDsWheelDataRightSet = tFalse;
            m_bIDsSpeedSet = tFalse;
            m_bIDsSampleDistanceSet = tFalse;
            m_bIDsOverallDistanceSet = tFalse;
            m_bIDsOverallDistanceRightSet = tFalse;
            m_bIDsOverallDistanceLeftSet = tFalse;

            m_f32LastCalculatedSpeedRight = 0;
            m_f32LastCalculatedSpeedLeft = 0;
        }

        RETURN_NOERROR;
}

tResult TickFilter::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {
        if (pSource == &m_oInputWheelLeft)
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
            	m_f32LastCalculatedSpeedLeft = calculateSpeed(ui32Timestamp,m_tLastStructLeft.ui32ArduinoTimestamp,ui32Tach-m_tLastStructLeft.ui32WheelTach);

                m_tLastStructLeft.i8WheelDir = i8Direction;
                m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
                m_tLastStructLeft.ui32WheelTach = ui32Tach;

            }

        }
        else if (pSource == &m_oInputWheelRight)
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

                m_f32LastCalculatedSpeedRight = calculateSpeed(ui32Timestamp,m_tLastStructRight.ui32ArduinoTimestamp,ui32Tach-m_tLastStructRight.ui32WheelTach);

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

tTimeStamp TickFilter::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

tFloat32 TickFilter::calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks)
{
    // return if time difference is 0, if time difference is smaller than 0, if ticks are 0 or smaller 0
    if ((ui32CurrentTimeStamp-ui32LastTimeStamp==0) || (ui32Ticks==0)) return 0;    
    //          circumference      SlotsInTimeDiff
    // speed =  -------------- *  -------------
    //           TotalSlots*          TimeDiff



    return (m_f32wheelCircumference/CW_SLOT_COUNT*static_cast<tFloat32>(ui32Ticks))/
        (static_cast<tFloat32>(ui32CurrentTimeStamp-ui32LastTimeStamp)/static_cast<tFloat32>(1e6));
}


tResult TickFilter::TransmitSamples()	
{    
    // enter critical section for transmitting
    __synchronized_obj(m_oCritSectionTransmit);

    // static variable for warning outputs to console 
    static tInt32 i32WarningCounter = 0;

    // calculate the average of both wheel speeds
    tFloat32 f32speed = (m_f32LastCalculatedSpeedRight+m_f32LastCalculatedSpeedLeft)/2;

    if (fabs((m_f32LastCalculatedSpeedRight-m_f32LastCalculatedSpeedLeft))> fabs(m_f32LastCalculatedSpeedRight)*CW_ERROR_DIFFERENCE_SIDES)
    {
        if (m_f32LastCalculatedSpeedRight<CW_MIN_LIMIT_IGNORE)
        {
            f32speed = m_f32LastCalculatedSpeedLeft;
            if (m_tLastStructLeft.i8WheelDir == 1)
                f32speed = f32speed * -1;
        }
        else if (m_f32LastCalculatedSpeedLeft<CW_MIN_LIMIT_IGNORE)
        {
            f32speed = m_f32LastCalculatedSpeedRight;    
            if (m_tLastStructRight.i8WheelDir == 1)
                f32speed = f32speed * -1;
        }
        i32WarningCounter++;
        if (i32WarningCounter%200==0)
            LOG_WARNING(cString::Format("Wheel speed from left and right side are very different. Please check cables and connections! Right: %f, Left: %f, Result: %f",
            m_f32LastCalculatedSpeedRight,m_f32LastCalculatedSpeedLeft,f32speed)); 
    }
    else
    {
    // if direction is backwards speed should be negative
    if (m_tLastStructLeft.i8WheelDir == 1 && m_tLastStructRight.i8WheelDir == 1)
        f32speed = f32speed * -1;
    }
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

	/*fstream f2;
	f2.open("TickFilterOverallDistanceOut.dat", ios::out);
	f2 << "m_f32OverallDistance = " << m_f32OverallDistance << "\tTimestamp = " << ui32arduinoTimestamp;
	f2.close(); */

    //transmit media sample over output pin
    pMediaSampleOverallDistance->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputDistanceOverall.Transmit(pMediaSampleOverallDistance));


    //create new media sample for speed
    cObjectPtr<IMediaSample> pMediaSampleSpeed;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleSpeed));

    //allocate memory with the size given by the descriptor	
	cObjectPtr<IMediaSerializer> pSerializer2;	
    m_pDescriptionOutputSpeed->GetMediaSampleSerializer(&pSerializer2);	
    nSize = pSerializer2->GetDeserializedSize();	
    RETURN_IF_FAILED(pMediaSampleSpeed->AllocBuffer(nSize));	


    {   // focus for sample write lock	
        //write date to the media sample with the coder of the descriptor    	
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputSpeed,pMediaSampleSpeed,pCoder2);	

        // set the ids if not already done
        if(!m_bIDsSpeedSet)
        {
            pCoder2->GetID("f32Value", m_szIDSpeedF32Value);
            pCoder2->GetID("ui32ArduinoTimestamp", m_szIDSpeedArduinoTimestamp);
            m_bIDsSpeedSet = tTrue;
        }    

        pCoder2->Set(m_szIDSpeedArduinoTimestamp, (tVoid*)&ui32arduinoTimestamp);
        pCoder2->Set(m_szIDSpeedF32Value, (tVoid*)&f32speed);
    }  

	/*fstream f1;
	f1.open("TickFilterSpeedOut.dat", ios::out);
	f1 << "f32speed = " << f32speed << "\tTimestamp = " << ui32arduinoTimestamp;
	f1.close(); */ 	

    //transmit media sample over output pin	
    pMediaSampleSpeed->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputCarSpeed.Transmit(pMediaSampleSpeed));  // Fehler ? (Andi) // Nochmal


    //create new media sample for overall distance
    cObjectPtr<IMediaSerializer> pSerializer3;	
    cObjectPtr<IMediaSample> pMediaSampleOverallDistanceLeft;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOverallDistanceLeft));

    //allocate memory with the size given by the descriptor
    m_pDescriptionOutputOverallDistanceLeft->GetMediaSampleSerializer(&pSerializer3);
    nSize = pSerializer3->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSampleOverallDistanceLeft->AllocBuffer(nSize));


    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputOverallDistanceLeft,pMediaSampleOverallDistanceLeft,pCoder3);

        // set the ids if not already done
        if(!m_bIDsOverallDistanceLeftSet)
        {
            pCoder3->GetID("f32Value", m_szIDOverallDistanceLeftF32Value);
            pCoder3->GetID("ui32ArduinoTimestamp", m_szIDOverallDistanceLeftArduinoTimestamp); //m_szIDSpeedArduinoTimestamp);   Fehler??
            m_bIDsOverallDistanceLeftSet = tTrue;
        }

        // set the values
        pCoder3->Set(m_szIDOverallDistanceLeftF32Value, (tVoid*)&m_f32OverallDistanceLeft);
        pCoder3->Set(m_szIDOverallDistanceLeftArduinoTimestamp, (tVoid*)&m_tLastStructLeft.ui32ArduinoTimestamp);
    }

	/*fstream f3;
	f3.open("TickFilterOverallDistanceLeftOut.dat", ios::out);
	f3 << "m_f32OverallDistanceLeft = " << m_f32OverallDistanceLeft << "\tTimestamp = " << m_tLastStructLeft.ui32ArduinoTimestamp;
	f3.close(); */

    //transmit media sample over output pin
    pMediaSampleOverallDistanceLeft->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputDistanceOverallLeft.Transmit(pMediaSampleOverallDistanceLeft));

    /*fstream f;
    f.open("Dist_Left.dat",ios::out | ios::app);
    f<<m_f32OverallDistanceLeft << "\n" ;
    f<<m_tLastStructLeft.ui32ArduinoTimestamp << "\n" ;
    f.close();*/




    //create new media sample for overall distance
    cObjectPtr<IMediaSerializer> pSerializer4;	
    cObjectPtr<IMediaSample> pMediaSampleOverallDistanceRight;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSampleOverallDistanceRight));

    //allocate memory with the size given by the descriptor
    m_pDescriptionOutputOverallDistanceRight->GetMediaSampleSerializer(&pSerializer4);
    nSize = pSerializer4->GetDeserializedSize();
    RETURN_IF_FAILED(pMediaSampleOverallDistanceRight->AllocBuffer(nSize));


    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescriptionOutputOverallDistanceRight,pMediaSampleOverallDistanceRight,pCoder4);

        // set the ids if not already done
        if(!m_bIDsOverallDistanceRightSet)
        {
            pCoder4->GetID("f32Value", m_szIDOverallDistanceRightF32Value);
            pCoder4->GetID("ui32ArduinoTimestamp", m_szIDOverallDistanceRightArduinoTimestamp);
            m_bIDsOverallDistanceRightSet = tTrue;
        }

        // set the values
        pCoder4->Set(m_szIDOverallDistanceRightF32Value, (tVoid*)&m_f32OverallDistanceRight);
        pCoder4->Set(m_szIDOverallDistanceRightArduinoTimestamp, (tVoid*)&m_tLastStructRight.ui32ArduinoTimestamp);
    }

	/*fstream f4;
	f4.open("Dist_Right.dat", ios::out | ios::app);
	f4 << m_tLastStructRight.ui32ArduinoTimestamp - m_tBeforeLastStructRight.ui32ArduinoTimestamp << " , " << m_f32OverallDistanceRight << "\n";
	f4.close();*/

    //transmit media sample over output pin
    pMediaSampleOverallDistanceRight->SetTime(_clock->GetStreamTime());
    RETURN_IF_FAILED(m_oOutputDistanceOverallRight.Transmit(pMediaSampleOverallDistanceRight));

    /*fstream f2;
    f2.open("Dist_Left.dat",ios::out | ios::app);
    f2<<m_f32OverallDistanceLeft << "\n" ;
    f2.close();*/

	
    /*fstream f3;
    f3.open("Dist_Rot.dat",ios::out | ios::app);
    f3<<((m_f32OverallDistanceLeft-m_f32OverallDistanceRight)/0.26)*180/M_PI<< "\n" ;
    f3.close();*/


    RETURN_NOERROR;
}



