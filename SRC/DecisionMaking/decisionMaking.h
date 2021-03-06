/**
 *
 * DecisionMaking
 *
 * $Author: OptiCar $
 * $Date: 2016-03-11
 *
 * @remarks todo
 *
 */
#ifndef _DECISION_MAKING_H_
#define _DECISION_MAKING_H_

#include "../../include/juryEnums.h"
#include "../../include/driving_states_enum.h"
#include "../../include/action_enum.h"
#include "../../include/aadc_roadSign_enums.h"
#include "../../include/cross_sign_type.h"
#include "../../include/cross_type.h"
#include "../../include/pullout_enum.h"

#define OID_ADTF_DECISION_MAKING "adtf.decision_making"


//*************************************************************************************************
class cDecisionMaking : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_DECISION_MAKING, "Decision Making", adtf::OBJCAT_DataFilter);

protected:
    cInputPin     	m_iStateDM;
	cInputPin 		m_iRoadSign;
	cInputPin 		m_iEmergencyBreak;
	cInputPin 		m_iCurrentManeuvre;
	cInputPin 		m_iCarSpeed;
	cInputPin 		m_iDistance;
	cInputPin 		m_iSteeringAngle;
	cInputPin 		m_iFoundParkingSpot;
	cInputPin 		m_iFinishedParking;
	cInputPin 		m_iFinishedPullOut;
	cInputPin 		m_iFoundCrossing;
	cInputPin 		m_iFoundZebraStripes;
	cInputPin 		m_iFinishedTurning;
	cInputPin		m_iFinishedOvertaking;
	cInputPin 		m_iFinishedZebraStripe;
	cInputPin 		m_iCheckedParkLot;
	cOutputPin		m_oStateHold;
    cOutputPin    	m_oStateLF;
	cOutputPin 	  	m_oStateCrossPark;
	cOutputPin 	  	m_oStateParallelPark;
	cOutputPin 		m_oStatePullOut;
	cOutputPin 		m_oStatePedestrain;
	cOutputPin 		m_oIncrementManeuvre;
	cOutputPin 		m_oStateComplete;
	cOutputPin 		m_oSearchParkingSpot;
	cOutputPin 		m_oSearchZebraStripes;
	cOutputPin 		m_oVelocityLF;
	cOutputPin 		m_oSearchCrossing;
	cOutputPin 		m_oStateTurning;
	cOutputPin 		m_oStateACC;
	cOutputPin 		m_oOvertaking;
	cOutputPin		m_oCheckPullOut;
	cOutputPin		m_oLFCircleMode;

public:
    cDecisionMaking(const tChar* __info);
    virtual ~cDecisionMaking();
   
	tResult DMwork(IMediaSample* pMediaSample);
	tResult processEmergencyBreak(IMediaSample* pMediaSample);
	tResult processFoundRoadSign(IMediaSample* pMediaSample);
	tResult processCurrentManeuvre(IMediaSample* pMediaSample);
	tResult processCarSpeed(IMediaSample* pMediaSample);
	tResult processOverallDistance(IMediaSample* pMediaSample);
	tResult processSteeringAngle(IMediaSample *pMediaSample);
	tResult processFoundParkingSpot(IMediaSample* pMediaSample);
	tResult processFoundCrossing(IMediaSample* pMediaSample);
	tResult processFoundZebraStripes(IMediaSample* pMediaSample);
	tResult ProcessParkLotType(IMediaSample* pMediaSample);

	tResult processFinishedManeuvre_Parking(IMediaSample* pMediaSample);
	tResult processFinishedManeuvre_PullOut(IMediaSample* pMediaSample);
	tResult processFinishedManeuvre_Turning(IMediaSample* pMediaSample);
	tResult processFinishedManeuvre_Overtaking(IMediaSample* pMediaSample);
	tResult processFinisehdManeuvre_ZebraStripes(IMediaSample* pMediaSample);

	tResult startModuleByID(int id);
	tResult resetAllFilters();

	tResult sendStatusHold(tBool status);
    tResult sendStatusLF(tBool status);
	tResult sendStatusCrossPark(tBool status, tFloat32 pDistance);
	tResult sendStatusParallelPark(tBool status, tFloat32 pDistance);	
	tResult sendStatusTurning(tBool status, tFloat pDistance, tInt32 pCrossType, tInt16 pCrossingSign, tInt32 pManeuvre, tFloat32 pRotationDegree);
	tResult sendStatusPullOut(tInt32 status, tBool state);
	tResult sendStatusPedestrainCrossing(tFloat32 nearDist, tFloat32 farDist, tBool state);
	tResult sendStatusACC(tBool status);
	tResult sendStateCompleted(tBool state);
	
	tResult incrementManeuvre();
	tResult searchForParkingSpot(tBool status, tInt32 ID);
	tResult searchForCrossing(tBool status, tInt32 ID);
	tResult searchForZebraStripes(tBool status);
	tResult setVelocityLF(tFloat32 fMin, tFloat32 fMax);
	tResult overtakingAllowed(tBool RoadSignAllowed, tBool ManeuvreAllowed, tBool PedestrainAllowed);
	tResult checkPullOutType(tBool state);
	tResult setToInitialState(tBool allValues);
	tResult LFCircleMode();
	tResult finished();
   

private:
	// StateDMInput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateDMInput;
	tBufferID m_szIDBoolValueStateDMInput; 
	tBufferID m_szIDTimestampStateDMInput;
	tBool m_bStateDMInputSet;

	// RoadSignInput
	cObjectPtr<IMediaTypeDescription> m_pRoadSignInput;
	tBufferID m_szIDSignIdentifierInput;
	tBufferID m_szIDSignSizeInput;
	tBool m_bRoadSignInputSet;
	
	// EmergencyBreakInput
	cObjectPtr<IMediaTypeDescription> m_pEmergencyBreakInput;
	tBufferID m_szIDBoolValueEmergencyBreakInput;
	tBufferID m_szIDTimestampEmergencyBreakInput;
	tBool m_bEmergencyBreakInputSet;
	
	// CurrentManeuvreInput
	cObjectPtr<IMediaTypeDescription> m_pCurrentManeuvreInput;
	tBufferID m_szIDInt32ValueCurrentManeuvreInput;
	tBufferID m_szIDTimestampCurrentManeuvreInput;
	tBool m_bCurrentManeuvreInputSet;
	
	// CarSpeedInput
	cObjectPtr<IMediaTypeDescription> m_pSignalCarSpeedInput;
	tBufferID m_szIDSignalValueCarSpeedInput;
	tBool m_bCarSpeedInputSet;

	// DistanceInput
	cObjectPtr<IMediaTypeDescription> m_pDistanceInput;
	tBufferID m_szIDDistanceInput;
	tBool m_bIDsDistanceSet;

	// SteeringAngleInput
	cObjectPtr<IMediaTypeDescription> m_pSignalSteeringAngleInput;
	tBufferID m_szIDSignalValueSteeringAngleInput;
	tBool m_bSteeringAngleInputSet;

	// FoundParkingSpotInput
	cObjectPtr<IMediaTypeDescription> m_pFoundParkingSpotInput;
	tBufferID m_szIDSignalValueFoundParkingSpotInput;
	tBool m_bFoundParkingSpotInputSet;	

	// FoundCrossingInput
	cObjectPtr<IMediaTypeDescription> m_pFoundCrossingInput;
	tBufferID m_szIDFoundCrossingCrosstypeInput;
	tBufferID m_szIDFoundCrossingDistanceInput;
	tBufferID m_szIDFoundCrossingRotationDegreeInput;
	tBool m_bFoundCrossingInputSet;
	
	// FoundZebrasInput
	cObjectPtr<IMediaTypeDescription> m_pFoundZebrasInput;
	tBufferID m_szIDZebraNearInput;
	tBufferID m_szIDZebraFarInput;
	tBool m_bFoundZebrasInputSet;
	
	// CheckedParkLotTypeInput
	cObjectPtr<IMediaTypeDescription> m_pCheckedParkLotInput;
	tBufferID m_szIDBoolValueCheckedParkLotInput;
	tBool m_bCheckedParkLotInputSet;

	// FinishedParkingInput
	cObjectPtr<IMediaTypeDescription> m_pBoolFinishedParkingInput;
	tBufferID m_szIDBoolValueFinishedParkingInput; 
	tBool m_bFinishedParkingInputSet;

	// FinishedPulloutInput
	cObjectPtr<IMediaTypeDescription> m_pBoolFinishedPulloutInput;
	tBufferID m_szIDBoolValueFinishedPulloutInput; 
	tBool m_bFinishedPulloutInputSet;
	
	// FinishedZebraStripeInput
	cObjectPtr<IMediaTypeDescription> m_pBoolFinishedZebraStripeInput;
	tBufferID m_szIDBoolValueFinishedZebraStripeInput; 
	tBool m_bFinishedZebraStripeInputSet;

	// FinishedTurningInput
	cObjectPtr<IMediaTypeDescription> m_pBoolFinishedTurningInput;
	tBufferID m_szIDBoolValueFinishedTurningInput; 
	tBool m_bFinishedTurningInputSet;

	// FinishedOvertakingInput
	cObjectPtr<IMediaTypeDescription> m_pFinishedOvertakingInput;
	tBufferID m_szIDFinishedOvertakingInput; 
	tBool m_bFinishedOvertakingInputSet; 


	// StateHoldOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateHoldOutput;
	tBufferID m_szIDBoolValueStateHoldOutput;
	tBufferID m_szIDTimestampStateHoldOutput;
	tBool m_bStateHoldOutputSet;

	// StateLFOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateLFOutput;
	tBufferID m_szIDBoolValueStateLFOutput;
	tBufferID m_szIDTimestampStateLFOutput;	
	tBool m_bStateLFOutputSet; 

	// StateCrossParkOutput
	cObjectPtr<IMediaTypeDescription> m_pStateCrossParkOutput;
	tBufferID m_szIDBoolValueStateCrossParkOutput;
	tBufferID m_szIDTimestampStateCrossParkOutput;
	tBufferID m_szIDVelocityStateCrossParkOutput;
	tBufferID m_szIDDistanceStateCrossParkOutput;
	tBool m_bCrossParkOutputSet;

	// StateCrossParallelOutput
	cObjectPtr<IMediaTypeDescription> m_pStateParallelParkOutput;
	tBufferID m_szIDBoolValueStateParallelParkOutput;
	tBufferID m_szIDTimestampStateParallelParkOutput;
	tBufferID m_szIDVelocityStateParallelParkOutput;
	tBufferID m_szIDDistanceStateParallelParkOutput;
	tBool m_bParallelParkOutputSet;

	// StatePullOut
	cObjectPtr<IMediaTypeDescription> m_pStatePullOutOutput;
	tBufferID m_szIDStatePullOutOutput;
	tBool m_bStatePullOutOutputSet;

	// StateTurningOutput
	cObjectPtr<IMediaTypeDescription> m_pStateTurningOutput;
	tBufferID m_szIDStateTurningTimestampOutput;
	tBufferID m_szIDStateTurningModeOutput;
	tBufferID m_szIDStateTurningTypeOutput;
	tBufferID m_szIDStateTurningGiveWayOutput;
	tBufferID m_szIDStateTurningStopSignOutput;
	tBufferID m_szIDStateTurningDistanceOutput;
	tBufferID m_szIDStateTurningVelocityOutput;
	tBufferID m_szIDStateTurningStatusOutput;
	tBufferID m_szIDStateTurningPullOutModeOutput;
	tBufferID m_szIDStateTurningRotationDegreeOutput;
	tBufferID m_szIDStateTurningCircleOutput;
	tBufferID m_szIDStateTurningIsInCircleOutput;
	tBool m_bTurningOutputSet;
	
	// StatePedestrainCrossingOutput
	cObjectPtr<IMediaTypeDescription> m_pStatePCOutput;
	tBufferID m_szIDStatePCnearDistOutput;
	tBufferID m_szIDStatePCfarDistOutput;
	tBufferID m_szIDStatePCvelocityOutput;
	tBufferID m_szIDStatePCOutput;
	tBool m_szIDStatePCOutputSet;

	// StateACCOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateACCOutput;
	tBufferID m_szIDBoolValueStateACCOutput;
	tBool m_bStateACCOutputSet;	

	// IncrementManeuvreOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolIncrementManeuvreOutput;
	tBufferID m_szIDBoolValueIncrementManeuvreOutput;
	tBufferID m_szIDTimestampIncrementManeuvreOutput;
	tBool m_bIncrementManeuvreOutputSet;
	
	// StateCompleteOutput
	cObjectPtr<IMediaTypeDescription> m_pBoolStateCompleteOutput;
	tBufferID m_szIDBoolValueStateCompleteOutput;
	tBufferID m_szIDTimestampStateCompleteOutput;
	tBool m_bStateCompleteOutputSet;

	// Search for ParkingSpotOutput
	cObjectPtr<IMediaTypeDescription> m_pSearchParkingSpotOutput;
	tBufferID m_szIDBoolValueSearchParkingSpotOutput;
	tBufferID m_szIDSearchParkingSpotOutput;
	tBool m_bSearchParkingSpotOutputSet;
	
	// Search for Crossing Output
	cObjectPtr<IMediaTypeDescription> m_pSearchCrossingOutput;
	tBufferID m_szIDBoolValueSearchCrossingOutput;
	tBufferID m_szIDBoolValueIsCircleOutput;
	tBufferID m_szIDSearchCrossingOutput;
	tBool m_bSearchCrossingOutputSet;
	
	// Search for ZebraStripes Output
	cObjectPtr<IMediaTypeDescription> m_pSearchZebraOutput;
	tBufferID m_szIDBoolValueSearchZebraOutput; 
	tBool m_bSearchZebraOutputSet;

	// VelocityOutput
	cObjectPtr<IMediaTypeDescription> m_pVelocityOutput;
	tBufferID m_szIDVelocityMinOutput; 
	tBufferID m_szIDVelocityMaxOutput; 
	tBufferID m_szIDTimestampVelocityOutput;
	tBool m_bVelocityOutputSet; 
	
	// OvertakingOutput
	cObjectPtr<IMediaTypeDescription> m_pOvertakingOutput;
	tBufferID m_szIDOvertakingRoadSignOutput; 
	tBufferID m_szIDOvertakingManeuvreOutput; 
	tBufferID m_szIDOvertakingPedestrainOutput;
	tBool m_bOvertakingOutputSet; 
	
	// CheckPullOutOutput
	cObjectPtr<IMediaTypeDescription> m_pCheckPullOutOutput;
	tBufferID m_szIDBoolValueCheckPullOutOutput; 
	tBufferID m_szIDTimestampCheckPullOutOutput;
	tBool m_bCheckPullOutOutputSet;

	// LFCircleModeOutput
	cObjectPtr<IMediaTypeDescription> m_pLFCircleModeOutput;
	tBufferID m_szIDBoolValueLFCircleModeOutput; 
	tBufferID m_szIDTimestampLFCircleModeOutput;
	tBool m_bLFCircleModeOutputSet;
	
	// Flags
	tBool m_DMActive;
	tBool m_LFActive;
	tBool m_CrossParkActive;
	tBool m_ParallelParkActive;
	tBool m_HoldActive;
	tBool m_TurningActive;
	tBool m_ZebraActive;
	tBool m_ACCActive;
	tBool m_EmergencyBreak;

	tBool m_HoldWhileEmergency;
	
	tInt16 m_ParkingMode;
	
	tInt16 m_CurrentCrossingSign;
	tBool  m_CurrentlyCrossing;
	tBool  m_currentlyPullOut;
	
	tInt32 m_CurrentManeuvre;
	
	// Filter current car speed	
	tFloat32 m_currentSpeed;
	std::deque<tFloat32> m_speedBuffer;

	// Filter current steering angle
	tFloat32 m_currentSteering;
	std::deque<tFloat32> m_steeringBuffer;
	
	tBool m_processingRoadSign;
	
	// Overtaking-Allowed flags
	tBool m_OvertakingAlowed_RoadSign;
	tBool m_OvertakingAlowed_Maneuvre;
	tBool m_OvertakingAlowed_Pedestrain;

	tBool m_currentlyOvertaking;

	// Flag to know if we are driving within a cricle or not
	tBool m_currentlyCircle;
	tBool m_withinCircle;

	tFloat32 m_overallDistance;
	tFloat32 m_referenceDistance;
	tBool m_checkDistance;

	tFloat32 m_lastDistanceCheckpoint;
	tFloat32 m_checkLastDistance;

	// Check distance to know when to start searching in circle
	tBool m_checkCircleDistance;
	tFloat32 m_circleDistance;
	tBool m_checkStraightCircleDistance;
	tFloat32 m_StraightCircleDistance;

	int m_Counter;

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
};

//*************************************************************************************************
#endif // _TEMPLATE_PROJECT_FILTER_H_
