/*
 * Date 17.03.16
 */


#include <math.h>
#include "stdafx.h"
#include "stopFilterCrossParking.h"
#include <fstream>
#include "../Util/Util.h"

#define MIN_SPEED 0.4
#define MIDDLE_SPEED 0.8

// seitliche Distanz zum Auto in der Parkluecke
#define DIST_TO_CAR 0.40

// Mitte der Luecke +- TEST_AREA Bereich für seitlichen Sensortest
#define TEST_AREA 0.06
#define SPOT_TOL 0.15

// if no new spot after driving this distance, shutdown filter
#define DRIVE_AROUND_DISTANCE 3.0
#define DRIVE_AROUND_SPEED 0.07

// unbedingt hoch lassen!!
#define MAX_WAITING_TIME 10.0

#define OBJECT_HEIGHT 0.07

#define GOOD_COUNT_PERCENTAGE 0.7

//Don't change this!_____
#define OFFSET_X 0.09
#define LANE_OFFSET_X 0.08
#define LENGHT_CROSS_SPOT 0.45
//_____________________

#define OFFSET_CROSS (LENGHT_CROSS_SPOT * 0.5 + 0.28 + 0.03)
//#define OFFSET_CROSS (LENGHT_CROSS_SPOT * 0.5 + 0.18)
//dist(Hinderrrad_ Kamera-9cm)

#define SFCP_PROP_SHOW_GCL "Common::Show GCL Output"


ADTF_FILTER_PLUGIN("Stop Filter for Cross Parking", OID_STOP_FILTER_CROSS_PARKING, cStopFilterCrossParking);


cStopFilterCrossParking::cStopFilterCrossParking(const tChar* __info):cFilter(__info)
{

	SetPropertyBool(SFCP_PROP_SHOW_GCL, tFalse);
    SetPropertyStr(SFCP_PROP_SHOW_GCL NSSUBPROP_DESCRIPTION, "If true, the opencv windows will be shown and the gcl output is enabled.");

}

cStopFilterCrossParking::~cStopFilterCrossParking()
{

}

tResult cStopFilterCrossParking::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {	
		cObjectPtr<IMediaDescriptionManager> pDescManager;
 		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             	IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             	(tVoid**)&pDescManager,
                                             	__exception_ptr));



	
        //get description for sensor data pins
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");	
        RETURN_IF_POINTER_NULL(strDescSignalValue);	
        //get mediatype for ultrasonic sensor data pins
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // MediaDescription of StopFilter-Struct
        tChar const* strDescStopStruct = pDescManager->GetMediaDescription("tStopStruct");
        RETURN_IF_POINTER_NULL(strDescStopStruct);
        cObjectPtr<IMediaType> pTypeStopStruct = new cMediaType(0, 0, 0, "tStopStruct", strDescStopStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

        // Get description for bool values
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
        cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
        // Media Description LaneTracking Velocity
        tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
        RETURN_IF_POINTER_NULL(LTVelocityDescValue);
        cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description ParkingID
		tChar const * ParkingDesc = pDescManager->GetMediaDescription("tParkingStruct");
		RETURN_IF_POINTER_NULL(ParkingDesc);
		cObjectPtr<IMediaType> pTypeParkingDesc = new cMediaType(0, 0, 0, "tParkingStruct", ParkingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		

		// DistanceInput
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iDistance));

		// DistanceInput
		RETURN_IF_FAILED(m_iDistanceSpot.Create("Distance_spot", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iDistanceSpot));

		// US right (side)
		RETURN_IF_FAILED(m_iUltraSonicRight.Create("Sensor_right", pTypeSignalValue, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iUltraSonicRight));

		// US front left
		RETURN_IF_FAILED(m_iUltraSonicFrontLeft.Create("Sensor_front_left", pTypeSignalValue, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iUltraSonicFrontLeft));
		
		// StopFilter Structure
		RETURN_IF_FAILED(m_iStopStruct.Create("StopStruct", pTypeStopStruct, static_cast <IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStopStruct));

		// Depthimage Input
		RETURN_IF_FAILED(m_iDepthimagePin.Create("Depthimage_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_iDepthimagePin));
		
		// VelocityOutput
		RETURN_IF_FAILED(m_oVelocity.Create("Velocity_LT", pTypeLTVelocityValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVelocity));
		
        //create output pin for finish parking output data
        RETURN_IF_FAILED(m_oFinishStoppingOutput.Create("finish_Stopping", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oFinishStoppingOutput));

        //create output pin for 
        RETURN_IF_FAILED(m_oShutDownOutput.Create("shut_down", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oShutDownOutput));
		
        // Stop LaneTracking Output
        RETURN_IF_FAILED(m_oStopLF.Create("Stop_LF", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oStopLF));	

        // Stop EmergencySTOP Output
        RETURN_IF_FAILED(m_oStopES.Create("Stop_ES", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oStopES));

        //create output pin for right light
        RETURN_IF_FAILED(m_oTurnRightOutput.Create("right_light", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
        RETURN_IF_FAILED(RegisterPin(&m_oTurnRightOutput));

		// stop search for ParkingSpot (Output)
		RETURN_IF_FAILED(m_oStopSearchParkingSpot.Create("StopSearchParkingSpot", pTypeParkingDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStopSearchParkingSpot));
		RETURN_IF_FAILED(pTypeParkingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopSearchParkingSpotOutput));


        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSRightInput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pUSFrontLeftInput));
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceSpotInput));
        RETURN_IF_FAILED(pTypeLTVelocityValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pVelocityOutput));
        RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStopStructInput));
        RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));

        //GLC Output
        cObjectPtr<IMediaType> pCmdType = NULL;
        RETURN_IF_FAILED(AllocMediaType(&pCmdType, MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL, __exception_ptr));
        RETURN_IF_FAILED(m_oGCLOutput.Create("GLC_Output",pCmdType, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oGCLOutput));

    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
		m_showGCL = GetPropertyBool(SFCP_PROP_SHOW_GCL);

		// ID not set yet
		m_pStopStructInputSet 	= tFalse;
		m_bIDsBoolValueOutput 	= tFalse;
		m_bIDsDistanceSet       = tFalse;
		m_bIDsDistanceSpotSet   = tFalse;
		m_bIDsVelocityOutput	= tFalse;
		m_bFirstFrameDepthimage = tTrue;
		m_bIDsUSRightSet        = tFalse;
		m_bIDsUSFrontLeftSet	= tFalse;
		m_bStopSearchParkingSpotOutputSet 	= tFalse;
		SetToInitialState();	
    }

    RETURN_NOERROR;
}

tResult cStopFilterCrossParking::Shutdown(tInitStage eStage, __exception)
{
   
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cStopFilterCrossParking::PropertyChanged(const char* strProperty)
{

	if (NULL == strProperty || cString::IsEqual(strProperty, SFCP_PROP_SHOW_GCL)) {
		m_showGCL = GetPropertyBool(SFCP_PROP_SHOW_GCL);
	}

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
		

        if (pSource == &m_iStopStruct){
                ProcessStopStruct(pMediaSample);
        } else if (pSource == &m_iDistance && m_stopActive && !m_startParking){
                ProcessSpeed(pMediaSample);
        } else if (pSource == &m_iDistanceSpot && m_stopActive && !m_startParking){
                ProcessSpotDistance(pMediaSample);
        } else if (pSource == &m_iUltraSonicRight && m_checkSpot){ 
                ProcessUltrasonicRightInput(pMediaSample);
        } else if (pSource == &m_iUltraSonicFrontLeft && m_stopActive && (m_distanceZero + m_offsetSpot - m_drivenDistance < 0.05)){
                ProcessUltrasonicFrontLeftInput(pMediaSample);
        } else if (pSource == &m_iDepthimagePin && m_stopActive) {

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
	                //LOG_ERROR("No Bitmap information found on pin \"input\"");
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

				m_i32StartTimeStamp = 0;
        	}		

			if ( m_distanceZero + m_offsetSpot - m_drivenDistance < 0.5) {
				if (m_i32StartTimeStamp==0){
					m_i32StartTimeStamp = _clock->GetStreamTime();
				} 
				tUInt32 fTime = _clock->GetStreamTime();

				if ((fTime-m_i32StartTimeStamp)/1000000.0 > m_waiting_time){
					//LOG_INFO(cString::Format("SFT: waitingTime %f", m_waiting_time));
					ProcessInputDepth(pMediaSample);
					m_i32StartTimeStamp = fTime;
				}
			}
    	}
	}

    RETURN_NOERROR;
}

tResult cStopFilterCrossParking::ProcessStopStruct(IMediaSample* pMediaSample){

	tBool filterActive = m_stopActive;

	{
		__adtf_sample_read_lock_mediadescription(m_pStopStructInput, pMediaSample, pCoder);

		if (!m_pStopStructInputSet){
			pCoder->GetID("fVelocity", m_szIDVelocityStructInput);
			pCoder->GetID("fDistance", m_szIDDistanceStructInput);
			pCoder->GetID("bStart", m_szIDBoolStructInput);
			m_pStopStructInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDVelocityStructInput, (tVoid*)&m_velocityZero);
		pCoder->Get(m_szIDDistanceStructInput, (tVoid*)&m_distanceZero);
		pCoder->Get(m_szIDBoolStructInput, (tVoid*)&m_stopActive);
	}

	/*fstream f2;
	f2.open("SFCP_fromDM.dat",ios::out|ios::app);
	f2 << m_distanceZero << "\n";
	f2.close();*/ 


	if (!m_stopActive) { // shutdown filter
		TransmitBoolValue(&m_oTurnRightOutput, tFalse);
		//TransmitBoolValue(&m_oStopLF, tTrue);
		TransmitBoolValue(&m_oStopES, tTrue);
		TransmitBoolValue(&m_oFinishStoppingOutput, tFalse);
		SetToInitialState();
		LOG_INFO(cString::Format("SFCP: shutdown stop parking filter"));
	} else if (!filterActive){ // start filter
        m_offsetSpot = OFFSET_CROSS;
		spot.push_back(spotStruct(m_distanceZero,1));
		LOG_INFO(cString::Format("SFCP: start stop parking filter"));
	}

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::ProcessSpotDistance(IMediaSample* pMediaSample){
	//tInt32 zeit = _clock->GetStreamTime();
    tFloat32 fSpotDistance = 0.0;

	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceSpotInput,pMediaSample, pCoder);

		if (!m_bIDsDistanceSpotSet) {
			pCoder->GetID("f32Value", m_szIDDistanceSpotInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceSpotInput);
			m_bIDsDistanceSpotSet = tTrue;
		}

		pCoder->Get(m_szIDDistanceSpotInput, (tVoid*)&fSpotDistance);
	}

	fSpotDistance += m_drivenDistance;

	bool flag = false;

	//fstream f2;
	//f2.open("SFCP_ParkingSpotsCrossLuecken.dat",ios::out|ios::app);

	for( size_t i = 0; i < spot.size(); ++i) {
		if (fabs(spot[i].distance - fSpotDistance) < SPOT_TOL) {
			
				//f2 << "Luecke weiter hinten " << spot[i].distance << " | " << fSpotDistance <<"\n";

				spot[i].distance = (spot[i].distance*spot[i].counter + fSpotDistance)/(spot[i].counter+1);
				spot[i].counter++;
			
			flag = true;
		}
	}


	if (!flag) {
		spot.push_back(spotStruct(fSpotDistance,1));
	}
	
	sort(spot.begin(),spot.end());

	if (spot[m_spotCounter-1].distance != m_distanceZero){
		//f2 << "m_distanceZero changed from " << m_distanceZero << " | " << spot[m_spotCounter-1].distance <<"\n";
		m_distanceZero = spot[m_spotCounter-1].distance;
		
	}


	/*f2 << "________________" << "\n";
	for( size_t i = 0; i < spot.size(); ++i) {
		f2 << i << " | " << spot[i].distance << "\n";
	}
	f2 << "________________" << "\n";

	zeit = _clock->GetStreamTime() - zeit;
	f2 << zeit << "\n";
	f2.close(); */

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::ProcessUltrasonicFrontLeftInput(IMediaSample* pMediaSample){

        tFloat32 fUSFrontLeft = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pUSFrontLeftInput,pMediaSample, pCoder);

		if (!m_bIDsUSFrontLeftSet) {
			pCoder->GetID("f32Value", m_szIDUSFrontLeftInput);
			m_bIDsUSFrontLeftSet = tTrue;
		}

		pCoder->Get(m_szIDUSFrontLeftInput, (tVoid*)&fUSFrontLeft);
	}

    if (fUSFrontLeft > 0.4){// no car on left lane
            ++m_goodUSFrontLeftCount;
    } else {
            ++m_badUSFrontLeftCount;
    }

	if (m_badUSFrontLeftCount>=2) {
		m_badUSFrontLeftCount = 0;
		m_goodUSFrontLeftCount = 0;
		m_noTrafficUS = tFalse;
	} else if (m_goodUSFrontLeftCount>=3) {
		m_noTrafficUS = tTrue;
	}

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::ProcessUltrasonicRightInput(IMediaSample* pMediaSample){

	tFloat32 fUSRight = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pUSRightInput,pMediaSample, pCoder);

		if (!m_bIDsUSRightSet) {
			pCoder->GetID("f32Value", m_szIDUSRightInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampUSRightInput);
			m_bIDsUSRightSet = tTrue;
		}

		pCoder->Get(m_szIDUSRightInput, (tVoid*)&fUSRight);
	}

	fstream fs;
	fs.open("SFCP_us_right.dat",ios::out|ios::app);
	fs << fUSRight << "\n";
	fs.close();
	
    if (fUSRight > DIST_TO_CAR) { // no car in spot 
            ++m_goodUSRightCount;
    } else {
            ++m_badUSRightCount;
    }
	
	RETURN_NOERROR;
}


tResult cStopFilterCrossParking::ProcessSpeed(IMediaSample* pMediaSample){
	
    tFloat32 fDistance = 0.0;
	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

		if (!m_bIDsDistanceSet) {
			pCoder->GetID("f32Value", m_szIDDistanceInput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampDistanceInput);
			m_bIDsDistanceSet = tTrue;
		}

		pCoder->Get(m_szIDDistanceInput, (tVoid*)&fDistance);
	}

	if (m_distanceOffset == 0.0) {
		m_distanceOffset = fDistance;
		//LOG_INFO(cString::Format("%f",m_distanceOffset));
	}
	
	fDistance -= m_distanceOffset;
	m_drivenDistance = fDistance;

	/*fstream fff;
	fff.open("SFCP_dist",ios::out|ios::app);
	fff << m_drivenDistance << "\n";
	fff.close();*/

	if (!m_turnRightON) {
		m_turnRightON = tTrue;
		TransmitBoolValue(&m_oTurnRightOutput, tTrue);
	}

    if (m_driveAround && (spot.size() > m_spotCounter)) { //new spot found
            m_driveAround = tFalse;//stop driving around
            m_driveAroundOffset = 0.0;
			m_distanceZero = spot[m_spotCounter].distance; // try next "free" spot
			m_i32StartTimeStampMax = 0;
			LOG_INFO(cString::Format("SFCP: next spot %f", m_distanceZero));
			++m_spotCounter;
    }


	/*fff.open("SFCP_dist1",ios::out|ios::app);
	fff << m_distanceZero + (LENGHT_CROSS_SPOT / 2) - TEST_AREA + 0.19 << "\n";
	fff.close();

	fff.open("SFCP_dist2",ios::out|ios::app);
	fff << m_distanceZero + (LENGHT_CROSS_SPOT / 2) + TEST_AREA + 0.19 << "\n";
	fff.close();*/

    if (m_drivenDistance > (m_distanceZero + (LENGHT_CROSS_SPOT / 2) - TEST_AREA + 0.19)){  // car nearly next to middle of free parking spot or has already passed it

			tFloat32 distAux = m_distanceZero + (LENGHT_CROSS_SPOT / 2) + TEST_AREA + 0.19; // 0.19 dist cam to side sensor
			if (distAux > m_distanceZero + OFFSET_CROSS){
				distAux = m_distanceZero + OFFSET_CROSS;
				
			}

            if ( m_drivenDistance < distAux){
					if (m_checkSpot == tFalse) {
                    	m_checkSpot = tTrue;
					}
            } else if (m_checkSpot){
                    m_checkSpot = tFalse;

                    // decide if spot is really free

                    tUInt32 counter = m_goodUSRightCount + m_badUSRightCount;

					if (counter > 0 ){ // else no sensorvalue received => spot is free
                    	if (m_goodUSRightCount/(tFloat32)counter < GOOD_COUNT_PERCENTAGE) {   //spot not free
                            if (spot.size() <= m_spotCounter) {
								m_spotFree = tFalse;
                            	m_driveAround = tTrue; //no more spots in list, start to drive around
                            	m_driveAroundOffset = fDistance;
								LOG_INFO(cString::Format("SFCP: start to drive around"));
                            } else {
                                m_distanceZero = spot[m_spotCounter].distance; // try next "free" spot
								m_i32StartTimeStampMax = 0;
								LOG_INFO(cString::Format("SFCP: try next spot %f, goodCount %i, badCount %i", m_distanceZero, m_goodUSRightCount, m_badUSRightCount));
                                ++m_spotCounter;

								fstream fs;
								fs.open("SFCP_us_right.dat",ios::out|ios::app);
								fs << ",,,,,,,,,,,,,,,,,,,," << "\n";
								fs.close();


								m_spotFree = tTrue; // assumption: next spot is free
                            }
                    	} else {//loeschen
							LOG_INFO(cString::Format("SFCP: start parking, goodCount %i, badCount %i", m_goodUSRightCount, m_badUSRightCount));
						}
                    	m_goodUSRightCount = 0;
                    	m_badUSRightCount = 0;
					} else { //loeschen
						LOG_INFO(cString::Format("SFCP: start parking, no sensorValuesReceived"));
					}
				
			}
    }

    tFloat32 fSpeed = 0.0;

	/*fff.open("SFCP_distZero",ios::out|ios::app);
	fff << m_distanceZero << "   " << m_offsetSpot << "\n";
	fff.close();*/

	tFloat32 faux = 1.0 - fDistance / (m_distanceZero + m_offsetSpot);

	if (faux < 1e-4 && m_spotFree) {
		fSpeed = 0.0;
		SendVelocity(fSpeed);
		LOG_INFO(cString::Format("SFCP: Timestamp auf 0 und startParking = true"));
		m_i32StartTimeStampMax = 0;
 		m_startParking = tTrue;
		stopSearchForParkingSpot(tFalse, 0);
	} else if ((m_distanceZero + m_offsetSpot - fDistance)<0.45) {
		fSpeed = MIN_SPEED;
	} else {
		fSpeed = sqrt(faux)*m_velocityZero;

		// Don't drive slower than MIN_SPEED
		if (fSpeed < MIDDLE_SPEED) {
			fSpeed = MIDDLE_SPEED;
		}
	}

    if (m_driveAround) {
		if (DRIVE_AROUND_DISTANCE - (fDistance-m_driveAroundOffset) < 1e-4) {
			m_stopActive = tFalse;
			m_shutDown = tTrue;
			//LOG_INFO(cString::Format("SFP: SHUTDOWN");
		} else {
			fSpeed = DRIVE_AROUND_SPEED;
		}
	}


    if (m_stopActive || m_startParking) { //either stopping process or send 0 because of traffic
		SendVelocity(fSpeed);
    } else if (m_shutDown) {
            // shut down filter
			LOG_INFO(cString::Format("initial1"));
			/*fstream fff;
			fff.open("SFCP_initial",ios::out|ios::app);
			fff << "initial1" << "\n";
			fff.close();*/
            SetToInitialState();
            TransmitBoolValue(&m_oShutDownOutput, tTrue);
    }

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::SendVelocity(tFloat32 fSpeed){

	tUInt32 nTimeStamp = 0;

	cObjectPtr<IMediaSample> pNewMediaSample;
    AllocMediaSample((tVoid**)&pNewMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pVelocityOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pNewMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pVelocityOutput, pNewMediaSample, pCoderOutput);

		// set the id if not already done
        if(!m_bIDsVelocityOutput)
        {
			pCoderOutput->GetID("fMinVel", m_szIDMinVelocityOutput);
			pCoderOutput->GetID("fMaxVel", m_szIDMaxVelocityOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDVelocityTimestampOutput);
			m_bIDsVelocityOutput = tTrue;
        }

		pCoderOutput->Set(m_szIDMinVelocityOutput, (tVoid*)&(fSpeed));
    	pCoderOutput->Set(m_szIDMaxVelocityOutput, (tVoid*)&(fSpeed));
    	pCoderOutput->Set(m_szIDVelocityTimestampOutput, (tVoid*)&nTimeStamp);

    	pNewMediaSample->SetTime(pNewMediaSample->GetTime());
	}

    m_oVelocity.Transmit(pNewMediaSample);
	if (fSpeed == 0.0){
		LOG_INFO(cString::Format("SFCP: Send Velocity %f", fSpeed) );
	}

	RETURN_NOERROR;
}

tResult cStopFilterCrossParking::ProcessInputDepth(IMediaSample* pSample){

	// VideoInput
	RETURN_IF_POINTER_NULL(pSample);

	const tVoid* l_pSrcBuffer;

	IplImage* oImg = cvCreateImage(cvSize(m_sInputFormatDepthimage.nWidth, m_sInputFormatDepthimage.nHeight), IPL_DEPTH_16U, 3);
	RETURN_IF_FAILED(pSample->Lock(&l_pSrcBuffer));
	oImg->imageData = (char*)l_pSrcBuffer;
	Mat image(cvarrToMat(oImg));
	cvReleaseImage(&oImg);
	pSample->Unlock(l_pSrcBuffer);

	Mat depthImage = Mat(m_sInputFormatDepthimage.nHeight,m_sInputFormatDepthimage.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sInputFormatDepthimage.nBytesPerLine);

	m_c = m_cAll= 0;

	auxObs.clear();
	auxAll.clear();


	for (int j=50;j<200;j++) {
		for (int i=0; i<200; i++) {
			Point3f coordinates = Util::ComputeWorldCoordinate(2*i,2*j,depthImage.at<ushort>(j, i), 0,0);
			coordinates.x += OFFSET_X;
			countObstacles(coordinates, i,j);
		}
	}
	m_noTraffic = checkObstacles();
	//LOG_INFO(cString::Format("SFCP: m_noTraffic %i, m_noTrafficUS %i, m_startParking %i", m_noTraffic, m_noTrafficUS, m_startParking));

       /*	fstream f;
	f.open("Hinderniszaehler.dat",ios::out|ios::app);
	f << m_c1 << ", " << m_c2 <<  ", " <<m_c4 << ", " <<m_c5 << ", " <<m_c6 << ", " <<m_c7 << ", " <<m_c8 << ", " <<m_c9  <<"\n";
	f << m_c1all << ", " << m_c2all <<  ", " <<m_c4all << ", " <<m_c5all << ", " <<m_c6all << ", " <<m_c7all << ", " <<m_c8all << ", " <<m_c9all  <<"\n";
	f << m_c1/(tFloat32)m_c1all << ", " << m_c2/(tFloat32)m_c2all <<  ", " << m_c4/(tFloat32)m_c4all << ", " << m_c5/(tFloat32)m_c5all << ", " << m_c6/(tFloat32)m_c6all << ", " << m_c7/(tFloat32)m_c7all << ", " << m_c8/(tFloat32)m_c8all << ", " << m_c9/(tFloat32)m_c9all  <<"\n";
	f.close();  */

	
	tUInt32 fTime = _clock->GetStreamTime();
    if (m_startParking){
        if (m_i32StartTimeStampMax == 0) {
                m_i32StartTimeStampMax = fTime;
				LOG_INFO(cString::Format("SFCP: Timestamp auf clock gesetzt"));
        } else if ((fTime-m_i32StartTimeStampMax)/1000000.0 > MAX_WAITING_TIME){
                m_noTraffic = m_noTrafficUS = tTrue; // erzwinge Einparken
				LOG_INFO(cString::Format("SFCP: maximale Wartezeit um, erzwinge Einparken"));
        }
    }

	CreateAndTransmitGCL();

    if (m_noTraffic && m_startParking && m_noTrafficUS /*&& ((fTime-m_i32StartTimeStampMax)/1000000.0 > 0.3)*/) {
		LOG_INFO(cString::Format("initial2"));

		/*fstream fff;
		fff.open("SFCP_initial",ios::out|ios::app);
		fff << "initial2" << "\n";
		fff.close();*/
        SetToInitialState();
        TransmitBoolValue(&m_oStopLF, tFalse);
        TransmitBoolValue(&m_oStopES, tFalse);
        TransmitBoolValue(&m_oFinishStoppingOutput, tTrue);
    }


	
	RETURN_NOERROR;
}

void cStopFilterCrossParking::countObstacles(Point3f &p, int i, int j) {


	if (p.x > -0.65 + LANE_OFFSET_X && p.x < -0.1 /*-0.25*/ - LANE_OFFSET_X && p.z > 0.25 + (m_distanceZero + m_offsetSpot - m_drivenDistance) && p.z < (m_distanceZero + m_offsetSpot - m_drivenDistance) + 1.0 /*0.3*//*1.5*/) {
		if (p.y < 0.22 - OBJECT_HEIGHT) {
			++m_c;
            if(m_showGCL) {auxObs.push_back(i); auxObs.push_back(j);}
		}
		++m_cAll;

        if(m_showGCL) {auxAll.push_back(i); auxAll.push_back(j);}
	}
}

tBool cStopFilterCrossParking::checkObstacles() {

	static const float pObs = 0.02;

	++m_cAll;


/*LOG_INFO(cString::Format("SFT: box 6=%f",m_c6/(tFloat32)m_c6all));
LOG_INFO(cString::Format("SFT: box 7=%f",m_c7/(tFloat32)m_c7all));
LOG_INFO(cString::Format("SFT: goodFrameCount=%i",m_good_frame_count)); */

    if (m_c/(tFloat32)m_cAll > pObs){
        //Hindernis 
        ++m_bad_depth_count;
        m_waiting_time = 0.3;
	} else {
        ++m_good_depth_count;
        m_waiting_time = 0.08;
    }

    if (m_bad_depth_count >= 2) {
        m_bad_depth_count = 0;
        m_good_depth_count = 0;
    } else if (m_good_depth_count >= 4) {
        return tTrue;
    }

    return tFalse;
}

tResult cStopFilterCrossParking::CreateAndTransmitGCL()
{
    // just draw gcl if the pin is connected and debug mode is enabled
    if (!m_oGCLOutput.IsConnected() || !m_showGCL)
    {
        RETURN_NOERROR;
    }

    // create a mediasample
    cObjectPtr<IMediaSample> pSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pSample));

    RETURN_IF_FAILED(pSample->AllocBuffer(1024*2048));

    pSample->SetTime(_clock->GetStreamTime());

    tUInt32* aGCLProc;
    RETURN_IF_FAILED(pSample->WriteLock((tVoid**)&aGCLProc));

    tUInt32* pc = aGCLProc;


	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 185, 15).GetRGBA());
	for (size_t i = 0; i < auxAll.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, auxAll[i]*2-1, auxAll[i+1]*2-1, auxAll[i]*2+1, auxAll[i+1]*2+1);
	}

	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(0, 0, 255).GetRGBA());	
	for (size_t i = 0; i < auxObs.size(); i+=2) {
		cGCLWriter::StoreCommand(pc, GCL_CMD_FILLRECT, auxObs[i]*2-1, auxObs[i+1]*2-1, auxObs[i]*2+1, auxObs[i+1]*2+1); 
	}

	tFloat64 text = (m_c)/(tFloat32)(m_cAll);
	cString strText = cString::FromFloat64(text);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 20, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_good_depth_count);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 40, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_bad_depth_count);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 60, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_goodUSRightCount);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 90, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_badUSRightCount);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 110, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_goodUSFrontLeftCount);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 130, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

	strText = cString::FromUInt32(m_badUSFrontLeftCount);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 150, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr());


	strText = cString::FromBool(m_noTraffic);
	cGCLWriter::StoreCommand(pc, GCL_CMD_FGCOL, cColor(255, 255, 255).GetRGBA());	
	if(m_noTraffic) cGCLWriter::StoreCommand(pc, GCL_CMD_TEXT, 10, 170, strText.GetLength());
	cGCLWriter::StoreData(pc, strText.GetLength(), strText.GetPtr()); 

    cGCLWriter::StoreCommand(pc, GCL_CMD_END);

    pSample->Unlock(aGCLProc);

    RETURN_IF_FAILED(m_oGCLOutput.Transmit(pSample));
    RETURN_NOERROR;

}

/*
 * This Method stops the parkingSpot detection filter
 */
tResult cStopFilterCrossParking::stopSearchForParkingSpot(tBool status, tInt32 ID) {

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pStopSearchParkingSpotOutput->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pStopSearchParkingSpotOutput, pMediaSample, pCoder);    

			if (!m_bStopSearchParkingSpotOutputSet) {
				pCoder->GetID("ID", m_szIDStopSearchParkingSpotOutput);
				pCoder->GetID("bStart", m_szIDBoolValueStopSearchParkingSpotOutput);
				m_bStopSearchParkingSpotOutputSet = tTrue;
			}

			// set value from sample
			pCoder->Set(m_szIDStopSearchParkingSpotOutput, (tVoid*)&ID);
			pCoder->Set(m_szIDBoolValueStopSearchParkingSpotOutput, (tVoid*)&status);
		}

		pMediaSample->SetTime(_clock->GetStreamTime());

		m_oStopSearchParkingSpot.Transmit(pMediaSample);

	RETURN_NOERROR; 
}


tResult cStopFilterCrossParking::TransmitBoolValue(cOutputPin* oPin, bool value)
{
	__synchronized_obj(m_oBoolValueCritSection);
	
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

tResult cStopFilterCrossParking::SetToInitialState() {

	/*fstream f;
	f.open("ParkingSpotsToilelrName.dat",ios::out|ios::app);
	for (size_t i=0; i<spot.size();i++){
		f << spot[i].distance << " | " << spot[i].counter  <<"\n";
	}
	f.close(); 

	f.open("SFCP_spots",ios::out|ios::app);
	for (size_t i=0; i<spot.size();i++){
		f << spot[i].distance <<"\n";
	}
	f.close();

	f.open("SFCP_init_dis",ios::out|ios::app);
	f << m_drivenDistance << "\n"; 
	f.close();*/


    m_goodUSRightCount		= 0;
    m_badUSRightCount		= 0;
    m_bad_depth_count       = 0;
    m_good_depth_count      = 0;
    m_badUSFrontLeftCount	= 0;
    m_goodUSFrontLeftCount	= 0;
	m_c						= 0;
	m_cAll					= 0;
    m_noTraffic             = tFalse;
	m_noTrafficUS			= tTrue;
	m_spotFree				= tTrue;
    m_driveAround           = tFalse;
    m_stopActive            = tFalse;
    m_turnRightON           = tFalse;
    m_checkSpot             = tFalse;
    m_shutDown              = tFalse;
    m_startParking          = tFalse;
    m_driveAroundOffset     = 0.0;
    m_offsetSpot            = 0.0;
    m_drivenDistance        = 0.0;
    m_velocityZero          = 0.0;
    m_distanceZero          = 0.0;
    m_distanceOffset        = 0.0;
    m_i32StartTimeStamp     = 0;
    m_i32StartTimeStampMax  = 0;
    m_waiting_time          = 0.8;
    m_spotCounter           = 1;

    spot.clear();
	auxObs.clear();
	auxAll.clear();

    RETURN_NOERROR;
}

