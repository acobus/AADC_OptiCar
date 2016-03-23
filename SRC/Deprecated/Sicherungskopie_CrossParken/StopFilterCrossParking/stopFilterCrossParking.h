/*
 * Date 16.02.16
 */
#ifndef _STOP_FILTER_CROSS_PARKING_H_
#define _STOP_FILTER_CROSS_PARKING_H_

#define OID_STOP_FILTER_CROSS_PARKING "adtf.stop_filter_cross_parking"


//*************************************************************************************************
class cStopFilterCrossParking : public adtf::cFilter
{
    ADTF_FILTER(OID_STOP_FILTER_CROSS_PARKING, "Stop Filter for Cross Parking", adtf::OBJCAT_DataFilter);

protected:
	cInputPin		m_iDistance;
	cInputPin		m_iDistanceSpot;
	cInputPin 		m_iStopStruct;
	cInputPin		m_iUltraSonicRight;
	cInputPin		m_iUltraSonicFrontLeft;
	cVideoPin		m_iDepthimagePin;
	cOutputPin 		m_oVelocity;
	cOutputPin 		m_oFinishStoppingOutput;
	cOutputPin 		m_oShutDownOutput;
	cOutputPin		m_oStopLF;
	cOutputPin 		m_oStopES;
	cOutputPin      m_oGCLOutput;
	cOutputPin 		m_oStopSearchParkingSpot;

	/*! the output pin to send turn-right-light */
	cOutputPin 		m_oTurnRightOutput;

public:
    cStopFilterCrossParking(const tChar* __info);
    virtual ~cStopFilterCrossParking();

	tResult PropertyChanged(const char* strProperty);

private:

	// OverallDistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBufferID m_szIDTimestampDistanceInput;
	tBool m_bIDsDistanceSet;

	// USRightInput
	cObjectPtr<IMediaTypeDescription> m_pUSFrontLeftInput;
	tBufferID m_szIDUSFrontLeftInput;
	tBufferID m_szIDTimestampUSFrontLeftInput;
	tBool m_bIDsUSFrontLeftSet;

	// USRightInput
	cObjectPtr<IMediaTypeDescription> m_pUSRightInput;
	tBufferID m_szIDUSRightInput;
	tBufferID m_szIDTimestampUSRightInput;
	tBool m_bIDsUSRightSet;

	// DistanceToParkingSpotInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceSpotInput;
	tBufferID m_szIDDistanceSpotInput;
	tBufferID m_szIDTimestampDistanceSpotInput;
	tBool m_bIDsDistanceSpotSet;
	
	// VelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pVelocityOutput;
	tBufferID m_szIDMinVelocityOutput;
	tBufferID m_szIDMaxVelocityOutput;
	tBufferID m_szIDVelocityTimestampOutput;
	tBool m_bIDsVelocityOutput;

	// StopStruct (Input)
	cObjectPtr<IMediaTypeDescription> m_pStopStructInput;
	tBufferID m_szIDVelocityStructInput;
	tBufferID m_szIDDistanceStructInput; 
	tBufferID m_szIDBoolStructInput;
	tBool m_pStopStructInputSet;

	// Bool Output
	cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;
	tBufferID m_szIDBoolValueOutput;
	tBufferID m_szIDArduinoTimestampOutput;
	tBool m_bIDsBoolValueOutput;

	// Stop Search for ParkingSpotOutput
	cObjectPtr<IMediaTypeDescription> m_pStopSearchParkingSpotOutput;
	tBufferID m_szIDBoolValueStopSearchParkingSpotOutput;
	tBufferID m_szIDStopSearchParkingSpotOutput;
	tBool m_bStopSearchParkingSpotOutputSet;

    //depth image
    tBool m_bFirstFrameDepthimage;
    // bitmap format of input pin
    tBitmapFormat m_sInputFormatDepthimage;
    tUInt32 m_i32StartTimeStamp;
    //saves time where first depth image has been received (for maximal waiting time)
    tUInt32 m_i32StartTimeStampMax;
    // waiting time between two frames
	tFloat32 m_waiting_time;
	tUInt32 m_c, m_cAll;
	// counter for frames without obstacle
	tUInt32 m_good_depth_count;
	// counter for frames with obstacle
	tUInt32 m_bad_depth_count;

	//sensor front left
	// counter for frames without obstacle
	tUInt32 m_goodUSFrontLeftCount;
	// counter for frames with obstacle
	tUInt32 m_badUSFrontLeftCount;

	tBool m_showGCL;

	tBool m_turnRightON;
	// true if car is next to free parking spot
	tBool m_checkSpot;
	// stopping mode active
	tBool m_stopActive;
	// true if no free spot has been found yet
	tBool m_driveAround;
	// true if no free spot has been found after a driven distance
	tBool m_shutDown;
	// flag for free lane on the left handside (no traffic)
	tBool m_noTraffic;
	// spot is free (us)
	tBool m_noTrafficUS;

	tBool m_startParking;
	//driven distance since driving around has been started
	tFloat32 m_driveAroundOffset;
	// velocity at t=0
	tFloat32 m_velocityZero;
	// distance to stopping position at t=0
	tFloat32 m_distanceZero;
	// distance offset
	tFloat32 m_distanceOffset;
    // driven distance since starting pin has been received
    tFloat32 m_drivenDistance;
    // offset front corner of spot to stopping position of car
    tFloat32 m_offsetSpot;
    // counter for frames with free spot
    tUInt32 m_goodUSRightCount;
    // counter for frames without free spot
    tUInt32 m_badUSRightCount;
    tUInt32 m_spotCounter;

	tBool m_spotFree;


	struct spotStruct{
		tFloat32 distance;
		tInt32 counter;
		spotStruct() {};
		spotStruct(tFloat32 d, tInt32 c) : distance(d), counter(c) {}
		bool operator<(const spotStruct& other) const {
			return (distance < other.distance);
		}
	};
    //distances to free parking spots
    std::vector<spotStruct> spot;

    std::vector<int> auxObs;
    std::vector<int> auxAll;

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    tResult ProcessSpeed(IMediaSample* pMediaSample);
    tResult ProcessSpotDistance(IMediaSample* pMediaSample);
    tResult ProcessStopStruct(IMediaSample* pMediaSample);
    tResult ProcessInputDepth(IMediaSample* pMediaSample);
    tResult ProcessUltrasonicRightInput(IMediaSample* pMediaSample);
    tResult ProcessUltrasonicFrontLeftInput(IMediaSample* pMediaSample);
    
    tResult TransmitBoolValue(cOutputPin* oPin, bool value);

	tResult stopSearchForParkingSpot(tBool status, tInt32 ID);	

    cCriticalSection    m_oBoolValueCritSection;
    
    tResult SendVelocity(tFloat32 fSpeed);
    tResult SetToInitialState();
    tResult CreateAndTransmitGCL();
    
    void countObstacles(Point3f &coordinates, int i, int j);
    tBool checkObstacles();


    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif 
