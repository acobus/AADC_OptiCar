#ifndef _ADAPTIVE_CRUISE_CONTROL
#define _ADAPTIVE_CRUISE_CONTROL


#define __guid "adtf.aadc.AdaptiveCruiseControl"

#include "stdafx.h"

class cAdaptiveCruiseControl : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(__guid, "AdaptiveCruiseControl", OBJCAT_DataFilter, "cAdaptiveCruiseControl", 1, 0, 0, "BFFT GmbH");    


public: // construction
	cAdaptiveCruiseControl(const tChar *);
	virtual ~cAdaptiveCruiseControl();

	/*! overrides cFilter */
	virtual tResult Init(tInitStage eStage, __exception = NULL);

	// properties
	tResult PropertyChanged(const char* strProperty);
	tResult ReadProperties(const tChar* strPropertyName);


protected:

    cVideoPin           m_iVideoInputPin;
	cVideoPin			m_iDepthimagePin;
	cInputPin			m_iStateACC;
	cInputPin 			m_iSteeringAngle;
	cInputPin 			m_iCarSpeed;
	cInputPin 			m_iCurrentLanetype;
    //cInputPin 			m_iUSSensorLeft;
	cInputPin 			m_iUSSensorFrontLeft;
	cInputPin 			m_iOvertakingAllowed;
    cOutputPin          m_oGCLOutput;
	cOutputPin 			m_oCarSpeed;
	cOutputPin 			m_oStartDriveRound;
	cOutputPin 			m_oBreaklight;


	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource,
			tInt nEventCode,
			tInt nParam1,
			tInt nParam2,
			IMediaSample* pMediaSample);

private:

	tResult 	trackTheCar(IMediaSample* pSample, tTimeStamp);
	tResult 	checkLongStraightLane(IMediaSample* pMediaSample, tTimeStamp);
	tResult 	setACCState(IMediaSample* pSample);
	tResult 	processLanetype(IMediaSample* pMediaSample);
	tResult 	processSteeringAngle(IMediaSample *pMediaSample);
	tResult 	processVelocity(IMediaSample* pMediaSample);
	tResult 	processOvertakingAllowed(IMediaSample* pMediaSample);
    //tResult 	processUSLeft(IMediaSample* pMediaSample);
	tResult 	processUSFrontLeft(IMediaSample* pMediaSample);
	tFloat32 	distanceToEnemy(int height, int width);
	tBool 		checkContraflow(int height, int width);
	tFloat32 	speedControl();
	tResult 	setVelocity(tFloat32 speed);
	tResult 	startOvertaking(tBool state);
	tResult 	TransmitBreaklightValue(tBool bValue);
	tResult 	CreateAndTransmitGCL();

	tResult 	resetValues();

	// StateACCInput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateACCInput;
	tBufferID m_szIDBoolValueStateACCInput; 
	tBool m_bStateACCInputSet;

	// SteeringAngleInput
	cObjectPtr<IMediaTypeDescription> m_pSignalSteeringAngleInput;
	tBufferID m_szIDSignalValueSteeringAngleInput;
	tBool m_bSteeringAngleInputSet;

	// CarSpeedInput
	cObjectPtr<IMediaTypeDescription> m_pSignalCarSpeedInput;
	tBufferID m_szIDSignalValueCarSpeedInput;
	tBool m_bCarSpeedInputSet;

	// LanetypeInput
	cObjectPtr<IMediaTypeDescription> m_pCurrentLanetypeInput;
	tBufferID m_szIDLanetypeInput;
	tBool m_bLanetypeInputSet;
	
	// OvertakingInput
	cObjectPtr<IMediaTypeDescription> m_pOvertakingInput;
	tBufferID m_szIDOvertakingRoadSignInput; 
	tBufferID m_szIDOvertakingManeuvreInput; 
	tBufferID m_szIDOvertakingPedestrainInput;
	tBool m_bOvertakingInputSet; 

	// USLeftSideInput
/*	cObjectPtr<IMediaTypeDescription> m_pUSLeftInput;
	tBufferID m_szIDUSLeftInput;
	tBool m_bUSLeftInputSet;*/

	// USFrontLeftInput
	cObjectPtr<IMediaTypeDescription> m_pUSFrontLeftInput;
	tBufferID m_szIDUSFrontLeftInput;
	tBool m_bUSFrontLeftInputSet;


	// SpeedOutOutput
	cObjectPtr<IMediaTypeDescription> m_pSignalSpeedOutput;
	tBufferID m_szIDSignalValueSpeedOutput;
	tBufferID m_szIDTimestampSpeedOutput;
	tBool m_bSpeedOutputSet; 

	// StateDriveRound
	cObjectPtr<IMediaTypeDescription> m_pStateDROutput;
	tBufferID m_szIDStateDROutput;
	tBool m_bStateDROutputSet;

	// BreaklightOutput
	cObjectPtr<IMediaTypeDescription> m_pBreaklightOutput;
	tBufferID m_szIDBoolValueBreaklightOutput;
	tBufferID m_szIDTimestampBreaklightOutput;
	tBool m_bBreaklightOutputSet;

	// Flags
	tBool m_bFirstFrameDepthimage;
    tBool m_bFirstFrame;
	tUInt8 m_iframeCounter;
	tBool m_ACCActive;

	// Videoformat
	tBitmapFormat 		m_sInputFormat;
	tBitmapFormat 		m_sInputFormatDepthimage;

	// Filter current steering angle
	tFloat m_currentSteering;
	std::deque<tFloat32> m_steeringBuffer;

	Mat 				m_ImageDepth;

	vector<tInt16> 		m_foundSquares;
	vector<tInt16> 		m_foundSquaresContraflow;

	tFloat32 			m_referenceSpeed;
	tFloat32 			m_currDistance;

	tFloat32 			m_USFrontBufferValue;
	tFloat32 			m_USSideBufferValue;

	tInt32 				m_Lanetype;

	tBool 				m_LeftFrontFree;
	//tBool 				m_LeftSideFree;
	
	tBool 				m_LongStraightLane;

	tBool 				m_breaking;

	//std::deque<tFloat32> m_USLeftBuffer;
	std::deque<tFloat32> m_USFrontLeftBuffer;

	std::deque<tFloat32> m_ContraflowBuffer;

	//Mat 				m_BinImage;
	Mat 				m_ImageZ;
	Mat 				m_ImageCF;
	
	// Overtaking-Allowed flags
	tBool m_OvertakingAlowed_RoadSign;
	tBool m_OvertakingAlowed_Maneuvre;
	tBool m_OvertakingAlowed_Pedestrain;

	tBool m_startLookingForContraflow;
	tBool m_noContraflow;

	tFloat32 m_derivation;

	//Hough lines
	vector<Vec2f> 	m_lines;
	vector<Vec2f>   m_lines_corr;
	tInt	m_nThresholdValueHough;
	tInt	m_nThresholdValueCanny;

	tInt 	m_standingCounter;

};

#endif // _ADAPTIVE_CRUISE_CONTROL
