#ifndef _DRIVE_ROUND
#define _DRIVE_ROUND


#define __guid "adtf.aadc.aadc_DriveRound"

#include "stdafx.h"




class DriveRound : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "DriveRound", OBJCAT_DataFilter, "DriveRound", 1, 0, 0, "BFFT GmbH");    


public: // construction
    DriveRound(const tChar *);
    virtual ~DriveRound();

    /*! overrides cFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);

    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);   

protected:
    
    cInputPin 		m_iStart;
    cInputPin 		m_iDistanceOverallLeft;
    cInputPin		m_iDistanceOverallRight;
    cInputPin 		m_iUSSensorRight;
	cInputPin 		m_iUSSensorFrontRight;
	cInputPin		m_iCurrentLanetype;
    cOutputPin 		m_oSpeed;
    cOutputPin 		m_oSteering;
    cOutputPin 		m_oStateLF;
    cOutputPin 		m_oStateES_WSC;
    cOutputPin 		m_oTurnRight;
    cOutputPin 		m_oTurnLeft;
    cOutputPin 		m_oStateToDM;
    
 	// implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);

private:
    
    tResult SetState(IMediaSample* pMediaSample);
    tResult processDistanceRight(IMediaSample* pMediaSample);
    tResult processDistanceLeft(IMediaSample* pMediaSample);
    tResult processUSRight(IMediaSample* pMediaSample);
	tResult processUSFrontRight(IMediaSample* pMediaSample);
	tResult processLanetype(IMediaSample* pMediaSample);
    
    tResult TransmitControls();
    tResult SetState_LF(tBool state);
    tResult SetState_ES_WSC(tBool state);
    tResult SetLeftLight(tBool status);
    tResult SetRightLight(tBool status);
	tResult sendStateToDM(tBool state);
    
	// StateInput
	cObjectPtr<IMediaTypeDescription> m_pStartInput;
	tBufferID m_szIDStateInput; 
	tBool m_bStateInputSet;
	
	// RightWheelDistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceOverallRightInput;
	tBufferID m_szIDDistanceOverallRightInput;
	tBool m_bDistanceOverallRightSet;
	
	// LeftWheelDistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceOverallLeftInput;
	tBufferID m_szIDDistanceOverallLeftInput;
	tBool m_bDistanceOverallLeftSet;
	
	// USRightSideInput
	cObjectPtr<IMediaTypeDescription> m_pUSRightInput;
	tBufferID m_szIDUSRightInput;
	tBool m_bUSRightInputSet;

	// USFrontRightInput
	cObjectPtr<IMediaTypeDescription> m_pUSFrontRightInput;
	tBufferID m_szIDUSFrontRightInput;
	tBool m_bUSFrontRightInputSet;

	// LanetypeInput
	cObjectPtr<IMediaTypeDescription> m_pCurrentLanetypeInput;
	tBufferID m_szIDLanetypeInput;
	tBool m_bLanetypeInputSet;
	
	// SteeringOutput
	cObjectPtr<IMediaTypeDescription> m_pSteeringOutput;
	tBufferID m_szIDSteeringOutput;
	tBufferID m_szIDSteeringTimestampOutput;
	tBool m_bSteeringOutputSet;
	
	// SpeedOutput
	cObjectPtr<IMediaTypeDescription> m_pSpeedOutput;
	tBufferID m_szIDSpeedOutput;
	tBufferID m_szIDSpeedTimestampOutput;
	tBool m_bSpeedOutputSet;

	// TurnRightLightOutput
	cObjectPtr<IMediaTypeDescription> m_pTurnRightLightOutput;
	tBufferID m_szIDTurnRightLightOutput;
	tBufferID m_szIDTurnRightLightTimestampOutput;
	tBool m_bTurnRightLightOutputSet;

	// TurnLeftLightOutput
	cObjectPtr<IMediaTypeDescription> m_pTurnLeftLightOutput;
	tBufferID m_szIDTurnLeftLightOutput;
	tBufferID m_szIDTurnLeftLightTimestampOutput;
	tBool m_bTurnLeftLightOutputSet;
	
	// StateLFOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateLFOutput;
	tBufferID m_szIDBoolValueStateLFOutput;
	tBufferID m_szIDTimestampStateLFOutput;	
	tBool m_bStateLFOutputSet; 
	
	// State_ES_WSC_Output
	cObjectPtr<IMediaTypeDescription> m_pBoolState_ES_WSC_Output;
	tBufferID m_szIDBoolValueState_ES_WSC_Output;
	tBufferID m_szIDTimestampState_ES_WSC_Output;	
	tBool m_bState_ES_WSC_OutputSet; 
	
	// StateToDMOutput
	cObjectPtr<IMediaTypeDescription> m_pStateToDMOutput;
	tBufferID m_szIDStateToDMOutput; 
	tBufferID m_szIDTimestampStateToDMOutput;
	tBool m_bStateToDMOutputSet; 
 
	tInt16 m_Active;

	tInt32 	m_Lanetype;
	
	tFloat32 m_distanceOffset;
	
	tBool 	m_wheelLeftFirst;
	tBool 	m_wheelRightFirst;

	tBool 	m_bufferToEmptyFront;
	tBool 	m_bufferToEmptySide;

	int 	m_sideCounter;
	int 	m_frontCounter;

	int m_USFrontState;
	
	std::deque<tFloat32> m_USRightBuffer;
	std::deque<tFloat32> m_USFrontRightBuffer;

	tFloat32 	m_driveLeft;
	tFloat32 	m_driveBackLeft;
	tFloat32 	m_driveRight;
	tFloat32 	m_driveBackRight;

	tUInt32 	m_activationTime;
	
};

#endif
