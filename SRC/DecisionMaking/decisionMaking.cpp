/**
 * 
 * DecisionMaking 
 *
 * $Author: OptiCar $
 * $Date: 11.03.2016
 *
 */
#include "stdafx.h"
#include "decisionMaking.h"
#include <fstream>

#define CROSS_PARK_DISTANCE_OFFSET 0.0 
//(0.21 + 0.205 + 0.03)
#define PARALLEL_PARK_DISTANCE_OFFSET 0.0
//1.47
#define TURNING_OFFSET 0.50
//0.57
//0.34
//0.81 


#define ACTION_VELOCITY 0.6
#define MIN_VELOCITY 1.3
//1.5
#define MAX_VELOCITY 1.8
//1.5
#define MIN_VELOCITY_AFTER_ACTION 0.6
#define MAX_VELOCITY_AFTER_ACTION 0.6
#define MIN_OVERTAKING_VELOCITY 1.5
#define MAX_OVERTAKING_VELOCITY 2.0

#define DRIVE_SLOW_DIST 0.7

#define STEERING_ERROR 20

#define CIRCLE_DEGREE -2.0
#define CIRCLE_EXIT_DEGREE -1.0
#define CIRCLE_OFFSET 0.10 	// For driving into the circle
#define CIRCLE_DISTANCE 1.2
#define CIRCLE_STRAIGHT_DISTANCE 1.7

#define ROAD_SIGN_MIN_SIZE 300
#define ROAD_SIGN_CROSSING 400

#define LAST_DISTANCE 0.5
//1.0

ADTF_FILTER_PLUGIN("Decision Making", OID_ADTF_DECISION_MAKING, cDecisionMaking);


cDecisionMaking::cDecisionMaking(const tChar* __info):cFilter(__info)
{

}

cDecisionMaking::~cDecisionMaking()
{
}

tResult cDecisionMaking::Init(tInitStage eStage, __exception)
{   

	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
				IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
				(tVoid**)&pDescManager,
				__exception_ptr));

		// Get description for bool values
		tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");	
		RETURN_IF_POINTER_NULL(strDescBoolSignalValue);	
		cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for RoadSign Struct
		tChar const * strDescRoadSign = pDescManager->GetMediaDescription("tRoadSign");   
		RETURN_IF_POINTER_NULL(strDescRoadSign);    
		cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strDescRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for tInt32
		tChar const * strDescInt32SignalValue = pDescManager->GetMediaDescription("tInt32SignalValue");	
		RETURN_IF_POINTER_NULL(strDescInt32SignalValue);	 
		cObjectPtr<IMediaType> pTypeInt32SignalValue = new cMediaType(0, 0, 0, "tInt32SignalValue", strDescInt32SignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description for SignalValue 
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");   
		RETURN_IF_POINTER_NULL(strDescSignalValue);    
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Get description of StopFilter-Struct
		tChar const* strDescStopStruct = pDescManager->GetMediaDescription("tStopStruct");
		RETURN_IF_POINTER_NULL(strDescStopStruct);
		cObjectPtr<IMediaType> pTypeStopStruct = new cMediaType(0, 0, 0, "tStopStruct", strDescStopStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description LaneTracking Velocity
		tChar const * LTVelocityDescValue = pDescManager->GetMediaDescription("tLTVelocityValue");
		RETURN_IF_POINTER_NULL(LTVelocityDescValue);
		cObjectPtr<IMediaType> pTypeLTVelocityValue = new cMediaType(0, 0, 0, "tLTVelocityValue", LTVelocityDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description ParkingID
		tChar const * ParkingDesc = pDescManager->GetMediaDescription("tParkingStruct");
		RETURN_IF_POINTER_NULL(ParkingDesc);
		cObjectPtr<IMediaType> pTypeParkingDesc = new cMediaType(0, 0, 0, "tParkingStruct", ParkingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description TurningState
		tChar const * TurningStateDesc = pDescManager->GetMediaDescription("tTurningStopStruct");
		RETURN_IF_POINTER_NULL(TurningStateDesc);
		cObjectPtr<IMediaType> pTypeTurningStateDesc = new cMediaType(0, 0, 0, "tTurningStopStruct", TurningStateDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description CrossingID
		tChar const * SearchCrossingDesc = pDescManager->GetMediaDescription("tStartCrossSearchingStruct");
		RETURN_IF_POINTER_NULL(SearchCrossingDesc);
		cObjectPtr<IMediaType> pTypeSearchCrossingDesc = new cMediaType(0, 0, 0, "tStartCrossSearchingStruct", SearchCrossingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description FoundParkingLots
		tChar const * FoundParkingLotsDesc = pDescManager->GetMediaDescription("tParkingLotStruct");
		RETURN_IF_POINTER_NULL(FoundParkingLotsDesc);
		cObjectPtr<IMediaType> pTypeFoundParkingLotsDesc = new cMediaType(0, 0, 0, "tParkingLotStruct", FoundParkingLotsDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Media Description Found Crossing
		tChar const * FoundCrossingDesc = pDescManager->GetMediaDescription("tCrossingStruct");
		RETURN_IF_POINTER_NULL(FoundCrossingDesc);
		cObjectPtr<IMediaType> pTypeFoundCrossingDesc = new cMediaType(0, 0, 0, "tCrossingStruct", FoundCrossingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
		// Media Description Overtaking
		tChar const * OvertakingDesc = pDescManager->GetMediaDescription("tOvertakingStruct");
		RETURN_IF_POINTER_NULL(OvertakingDesc);
		cObjectPtr<IMediaType> pTypeOvertakingDesc = new cMediaType(0, 0, 0, "tOvertakingStruct", OvertakingDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
		// Get description for Zebra Stripes Struct
		tChar const * strDescZebraStripes = pDescManager->GetMediaDescription("tZebraStripesStruct");   
		RETURN_IF_POINTER_NULL(strDescZebraStripes);    
		cObjectPtr<IMediaType> pTypeZebraStripes = new cMediaType(0, 0, 0, "tZebraStripesStruct", strDescZebraStripes, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		
		// MediaDescription of StopFilter-Struct
		tChar const* strDescStopZebraStruct = pDescManager->GetMediaDescription("tStopZebraStripesStruct");
		RETURN_IF_POINTER_NULL(strDescStopZebraStruct);
		cObjectPtr<IMediaType> pTypeStopZebraStruct = new cMediaType(0, 0, 0, "tStopZebraStripesStruct", strDescStopZebraStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// State of Decision Making (Input)
		RETURN_IF_FAILED(m_iStateDM.Create("State_Decision_Making", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iStateDM));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateDMInput));

		// Found RoadSigns (Input)
		RETURN_IF_FAILED(m_iRoadSign.Create("RoadSign", pTypeRoadSign, this));
		RETURN_IF_FAILED(RegisterPin(&m_iRoadSign));
		RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pRoadSignInput));

		// State of EmergencyBreak (Input)
		RETURN_IF_FAILED(m_iEmergencyBreak.Create("Emergency_Break", pTypeBoolSignalValue, this));
		RETURN_IF_FAILED(RegisterPin(&m_iEmergencyBreak));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pEmergencyBreakInput));

		// Current Maneuvre Code (Input)
		RETURN_IF_FAILED(m_iCurrentManeuvre.Create("Maneuvre_Code", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iCurrentManeuvre));
		RETURN_IF_FAILED(pTypeInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCurrentManeuvreInput));

		// create Inputpin for CarSpeed (Input)
		RETURN_IF_FAILED(m_iCarSpeed.Create("CarSpeed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iCarSpeed));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalCarSpeedInput));

		// Distance (Input)
		RETURN_IF_FAILED(m_iDistance.Create("Distance_overall", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDistance));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDistanceInput));

		// create Inputpin for Steering Angle (Input)
		RETURN_IF_FAILED(m_iSteeringAngle.Create("SteeringAngle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iSteeringAngle));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSignalSteeringAngleInput));

		// Receive Found ParkingSpot (Input)
		RETURN_IF_FAILED(m_iFoundParkingSpot.Create("FoundParkingSpot", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFoundParkingSpot));
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pFoundParkingSpotInput));

		// Receive Found Crossing (Input)
		RETURN_IF_FAILED(m_iFoundCrossing.Create("FoundCrossing", pTypeFoundCrossingDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFoundCrossing));
		RETURN_IF_FAILED(pTypeFoundCrossingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pFoundCrossingInput));
		
		// Receive Found ZebraStripes (Input)
		RETURN_IF_FAILED(m_iFoundZebraStripes.Create("FoundZebraStripes", pTypeZebraStripes, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFoundZebraStripes));
		RETURN_IF_FAILED(pTypeZebraStripes->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pFoundZebrasInput));
		
		// Receive the park type to start pull out (Input)
		RETURN_IF_FAILED(m_iCheckedParkLot.Create("Checked_Park_Type", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iCheckedParkLot));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCheckedParkLotInput));

		// Finished Parking (Input)
		RETURN_IF_FAILED(m_iFinishedParking.Create("Finished_Parking", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFinishedParking));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolFinishedParkingInput));

		// Finished Pullout (Input)
		RETURN_IF_FAILED(m_iFinishedPullOut.Create("Finished_Pullout", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFinishedPullOut));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolFinishedPulloutInput));

		// Finished Turning (Input)
		RETURN_IF_FAILED(m_iFinishedTurning.Create("Finished_Turning", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_iFinishedTurning));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolFinishedTurningInput));

		// FinishedOvertaking (Input)
		RETURN_IF_FAILED(m_iFinishedOvertaking.Create("FinishedOvertaking", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_iFinishedOvertaking));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pFinishedOvertakingInput));
		
		// FinishedZebraStripes (Input)
		RETURN_IF_FAILED(m_iFinishedZebraStripe.Create("FinishedZebraStripe", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_iFinishedZebraStripe));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolFinishedZebraStripeInput));


		// State of Hold (Output)
		RETURN_IF_FAILED(m_oStateHold.Create("State_Hold", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateHold));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateHoldOutput));

		// State of Lane Following (Output)
		RETURN_IF_FAILED(m_oStateLF.Create("State_Lane_Following", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateLF));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateLFOutput));

		// State of Cross Park (Output)
		RETURN_IF_FAILED(m_oStateCrossPark.Create("State_CrossPark", pTypeStopStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateCrossPark));
		RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateCrossParkOutput));

		// State of Parallel Park (Output)
		RETURN_IF_FAILED(m_oStateParallelPark.Create("State_ParallelPark", pTypeStopStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateParallelPark));
		RETURN_IF_FAILED(pTypeStopStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateParallelParkOutput));

		// State of PullOut (Output)
    	RETURN_IF_FAILED(m_oStatePullOut.Create("State_PullOut", pTypeInt32SignalValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_oStatePullOut));
		RETURN_IF_FAILED(pTypeInt32SignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStatePullOutOutput)); 

		// State of Turning (Output)
		RETURN_IF_FAILED(m_oStateTurning.Create("State_Turning", pTypeTurningStateDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateTurning));
		RETURN_IF_FAILED(pTypeTurningStateDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStateTurningOutput));

		// State of AdaptiveCruiseControl (Output)
		RETURN_IF_FAILED(m_oStateACC.Create("State_ACC", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateACC));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateACCOutput));		
		
		// State of PedestrainCrossing (Output)
		RETURN_IF_FAILED(m_oStatePedestrain.Create("State_StopZebra", pTypeStopZebraStruct, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStatePedestrain));
		RETURN_IF_FAILED(pTypeStopZebraStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pStatePCOutput));

		// Increment Maneuvre (Output)
		RETURN_IF_FAILED(m_oIncrementManeuvre.Create("Increment_Maneuvre", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oIncrementManeuvre));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolIncrementManeuvreOutput));
		
		// Set State complete (Output)
		RETURN_IF_FAILED(m_oStateComplete.Create("Completed", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oStateComplete));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pBoolStateCompleteOutput));

		// Search for ParkingSpot (Output)
		RETURN_IF_FAILED(m_oSearchParkingSpot.Create("SearchParkingSpot", pTypeParkingDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSearchParkingSpot));
		RETURN_IF_FAILED(pTypeParkingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSearchParkingSpotOutput));

		// Search for Crossing (Output)
		RETURN_IF_FAILED(m_oSearchCrossing.Create("SearchCrossing", pTypeSearchCrossingDesc, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSearchCrossing));
		RETURN_IF_FAILED(pTypeSearchCrossingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSearchCrossingOutput));
		
		// Search for ZebraStripes (Output)
		RETURN_IF_FAILED(m_oSearchZebraStripes.Create("Search_Zebra", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oSearchZebraStripes));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pSearchZebraOutput));

		// Velocity LaneTracking (Output)
		RETURN_IF_FAILED(m_oVelocityLF.Create("VelocityLF", pTypeLTVelocityValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oVelocityLF));
		RETURN_IF_FAILED(pTypeLTVelocityValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pVelocityOutput));
		
		// Overtaking Allowed (Output)
		RETURN_IF_FAILED(m_oOvertaking.Create("OvertakingAllowed", pTypeOvertakingDesc, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oOvertaking));
		RETURN_IF_FAILED(pTypeOvertakingDesc->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pOvertakingOutput));
		
		// CheckPullOut (Output)
		RETURN_IF_FAILED(m_oCheckPullOut.Create("CheckPullOut", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oCheckPullOut));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCheckPullOutOutput));

		// LF-CircleMode (Output)
		RETURN_IF_FAILED(m_oLFCircleMode.Create("LF_CircleMode", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));	
		RETURN_IF_FAILED(RegisterPin(&m_oLFCircleMode));
		RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pLFCircleModeOutput));


	}else if (eStage == StageNormal){

		setToInitialState(tTrue);
	}
	else if (eStage == StageGraphReady){

		// ID's not set yet
		m_bStateDMInputSet				= tFalse;
		m_bRoadSignInputSet				= tFalse;
		m_bEmergencyBreakInputSet		= tFalse;
		m_bCurrentManeuvreInputSet		= tFalse;
		m_bCarSpeedInputSet 			= tFalse;
		m_bStateHoldOutputSet			= tFalse;
		m_bStateLFOutputSet				= tFalse;
		m_bCrossParkOutputSet			= tFalse;
		m_bParallelParkOutputSet		= tFalse;
		m_bIncrementManeuvreOutputSet 	= tFalse;
		m_bSearchParkingSpotOutputSet   = tFalse;
		m_bFoundParkingSpotInputSet		= tFalse;
		m_bSteeringAngleInputSet 		= tFalse;
		m_bVelocityOutputSet 			= tFalse;
		m_bFinishedParkingInputSet 		= tFalse;
		m_bFinishedTurningInputSet 		= tFalse;
		m_bSearchCrossingOutputSet		= tFalse;
		m_bFoundCrossingInputSet 		= tFalse;
		m_bTurningOutputSet 			= tFalse;
		m_bStateACCOutputSet			= tFalse;
		m_bOvertakingOutputSet 			= tFalse;
		m_bFinishedOvertakingInputSet 	= tFalse;
		m_bIDsDistanceSet 				= tFalse;
		m_bStatePullOutOutputSet 		= tFalse;
		m_bFinishedPulloutInputSet 		= tFalse;
		m_bSearchZebraOutputSet 		= tFalse;
		m_bFoundZebrasInputSet 			= tFalse;
		m_szIDStatePCOutputSet			= tFalse;
		m_bFinishedZebraStripeInputSet 	= tFalse;
		m_bCheckPullOutOutputSet		= tFalse;
		m_bCheckedParkLotInputSet 		= tFalse;
		m_bStateCompleteOutputSet 		= tFalse;
		m_bLFCircleModeOutputSet 		= tFalse;

	}

	RETURN_NOERROR;
}

tResult cDecisionMaking::Shutdown(tInitStage eStage, __exception)
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

tResult cDecisionMaking::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{

		RETURN_IF_POINTER_NULL(pMediaSample);

		if (pSource == &m_iEmergencyBreak)
		{
			processEmergencyBreak(pMediaSample);
		}
		else if (pSource == &m_iStateDM)
		{
			DMwork(pMediaSample);
		}
		else if(pSource == &m_iFinishedOvertaking)
		{
			processFinishedManeuvre_Overtaking(pMediaSample);
		}
		else if (pSource == &m_iCarSpeed)
		{
			processCarSpeed(pMediaSample);
		}
		else if (pSource == &m_iSteeringAngle)
		{
			processSteeringAngle(pMediaSample);
		}

		else if (pSource == &m_iDistance && !m_EmergencyBreak && m_DMActive && (m_checkDistance || m_checkLastDistance || m_checkCircleDistance || m_checkStraightCircleDistance) )
		{
			processOverallDistance(pMediaSample);
		}

		else if (pSource == &m_iCurrentManeuvre)
		{
			processCurrentManeuvre(pMediaSample);
		}

		else if (m_DMActive){

			if (pSource == &m_iFoundParkingSpot)
			{
				processFoundParkingSpot(pMediaSample);
			}
			else if (pSource == &m_iFinishedParking)
			{
				processFinishedManeuvre_Parking(pMediaSample);
			}
			else if (pSource == &m_iFinishedPullOut)
			{
				processFinishedManeuvre_PullOut(pMediaSample);
			}
			else if (pSource == &m_iFinishedTurning)
			{
				processFinishedManeuvre_Turning(pMediaSample);
			}
			else if (pSource == &m_iFinishedZebraStripe)
			{
				processFinisehdManeuvre_ZebraStripes(pMediaSample);
			}
			else if (pSource == &m_iFoundCrossing)
			{
				processFoundCrossing(pMediaSample);
			}
			else if (pSource == &m_iFoundZebraStripes)
			{
				processFoundZebraStripes(pMediaSample);
			}
			else if (pSource == &m_iCheckedParkLot)
			{
				ProcessParkLotType(pMediaSample);
			}

			if (!m_EmergencyBreak && !m_currentlyOvertaking &&!m_checkLastDistance){

				if (pSource == &m_iRoadSign)
				{
		/*fstream f;
		f.open("FoundRoadSigns", ios::out | ios::app);
		f << "RECEIVED:"  << "\t" << m_processingRoadSign << "\t" << m_currentlyCircle << "\n";
		f.close();*/
					processFoundRoadSign(pMediaSample);
				}
			}
		}
	}

	RETURN_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////

/*
 * This Method sets state of the Decision Making
 */
tResult cDecisionMaking::DMwork(IMediaSample* pMediaSample){

	tBool bValue = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolStateDMInput ,pMediaSample, pCoder);    

		if (!m_bStateDMInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStateDMInput);		
			m_bStateDMInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueStateDMInput, (tVoid*)&bValue);         
	}

	

	//LOG_INFO(cString::Format("DM: CurrentManeuvre = %i", m_CurrentManeuvre) );

	// Standard-Cases
	if (bValue && !m_DMActive){
		//setToInitialState(tTrue);
		LOG_INFO(cString::Format("DM: ACTIVATE SYSTEM") );
		m_DMActive = bValue;
		if (m_CurrentManeuvre == ACTION_PULL_OUT_LEFT){
			m_checkDistance 	= tTrue;
			if (!m_currentlyPullOut) sendStatusPullOut(CROSS_LEFT, tTrue);
		}
		else if (m_CurrentManeuvre == ACTION_PULL_OUT_RIGHT){
			m_checkDistance 	= tTrue;
			if (!m_currentlyPullOut) checkPullOutType(tTrue);
		}
		else {
			startModuleByID(STATE_LANE_FOLLOWING);
			setVelocityLF(MIN_VELOCITY, MAX_VELOCITY);
		}

	}
	else if(!bValue && m_DMActive) {
		LOG_INFO(cString::Format("DM: DeACTIVATE SYSTEM") );
		resetAllFilters(); 
		m_DMActive = bValue;
	}
	
	

	RETURN_NOERROR;	
}

/*
 * This method organizes to reset working filters, so we can restart
 */
tResult cDecisionMaking::resetAllFilters(){

	// Reset the pull out filters if we are working here
	if (m_currentlyPullOut){
		checkPullOutType(tFalse);
		sendStatusPullOut(PULLOUT_UNKNOWN, tFalse);
	}

	startModuleByID(STATE_HOLD);

	overtakingAllowed(tTrue, tTrue, tTrue);

	setToInitialState(tFalse);	

	RETURN_NOERROR;

}

/*
 * Method to work with Emergency Break
 */
tResult cDecisionMaking::processEmergencyBreak(IMediaSample* pMediaSample){

	tBool bValue = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pEmergencyBreakInput ,pMediaSample, pCoder);    

		if (!m_bEmergencyBreakInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueEmergencyBreakInput);
			m_bEmergencyBreakInputSet = tTrue;
		}

		pCoder->Get( m_szIDBoolValueEmergencyBreakInput, (tVoid*)&bValue);         
	}

	LOG_INFO(cString::Format("DM: Received Emergency Value %i", bValue) );

	m_EmergencyBreak = bValue;

	// Deprecated: Emergency stop filter, sends zero velocity signal on is own!
	/*
	// Something has changed
	if (m_EmergencyBreak != bValue){

		m_EmergencyBreak = bValue; 	// Set new value

		if (m_EmergencyBreak){ 
			if (m_HoldActive){
				m_HoldWhileEmergency 	= tTrue;
			}
			else {
				m_HoldWhileEmergency 	= tFalse;
				startModuleByID(STATE_HOLD);
			}
		}
		else{

			if (m_HoldWhileEmergency || !m_DMActive) 	startModuleByID(STATE_HOLD);
			else									startModuleByID(STATE_LANE_FOLLOWING);

			m_HoldWhileEmergency 		= tFalse;	

		}
	}
	*/

	RETURN_NOERROR;
}

/*
 * Method to work with incoming Road signs. It is only allowed to process
 * one sign at time.
 */
tResult cDecisionMaking::processFoundRoadSign(IMediaSample* pMediaSample){

	if (!m_processingRoadSign && !m_currentlyCircle){

		tInt16 		identifier 	= -1;
		tFloat32	size		=  0;

		{
			__adtf_sample_read_lock_mediadescription(m_pRoadSignInput ,pMediaSample, pCoder);    

			if (!m_bRoadSignInputSet){
				pCoder->GetID("i16Identifier", m_szIDSignIdentifierInput);
				pCoder->GetID("f32Imagesize", m_szIDSignSizeInput);
				m_bRoadSignInputSet = tTrue;
			}

			pCoder->Get(m_szIDSignIdentifierInput, (tVoid*)&identifier);  
			pCoder->Get(m_szIDSignSizeInput, (tVoid*)&size);       
		}

		if (identifier != -1){	
			
			// Overtaking not allowed
			if (identifier == MARKER_ID_NOOVERTAKING){
				// Don't do anything, this sign will not appear during competition!
				
				//overtakingAllowed(tFalse, m_OvertakingAlowed_Maneuvre, m_OvertakingAlowed_Pedestrain);
				RETURN_NOERROR;
			}			
						
			if(size > ROAD_SIGN_MIN_SIZE){
				// Block this section
				m_processingRoadSign 	= tTrue;
				
				// Zebra-Stripes
				if (identifier == MARKER_ID_PEDESTRIANCROSSING){
					LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_PEDESTRIANCROSSING") );
					startModuleByID(STATE_ZEBRA);
				}

				// Parking
				else if (identifier == MARKER_ID_PARKINGAREA && m_CurrentManeuvre == ACTION_CROSS_PARKING){
					// Cross Parking			
					LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_PARKINGAREA") );
					startModuleByID(STATE_CROSS_PARKING);
				}
				else if ( identifier == MARKER_ID_PARKINGAREA && m_CurrentManeuvre == ACTION_PARALLEL_PARKING ){
					// Parallel Parking
					LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_PARKINGAREA") );
					startModuleByID(STATE_PARALLEL_PARKING);
				}

				// Turning
				else if ( (m_CurrentManeuvre == ACTION_LEFT || m_CurrentManeuvre == ACTION_RIGHT || m_CurrentManeuvre == ACTION_STRAIGHT) && size > ROAD_SIGN_CROSSING){

					// Crossing-Detection: UnmarkedIntersection
					if ( identifier == MARKER_ID_UNMARKEDINTERSECTION){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_UNMARKEDINTERSECTION") );
						m_CurrentCrossingSign 	= MARKER_ID_UNMARKEDINTERSECTION;
						startModuleByID(STATE_TURNING);					
					}

					// Crossing-Detection: StopAndGiveWay
					else if ( identifier == MARKER_ID_STOPANDGIVEWAY){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_STOPANDGIVEWAY") );
						m_CurrentCrossingSign 	= MARKER_ID_STOPANDGIVEWAY;
						startModuleByID(STATE_TURNING);
					}

					// Crossing-Detection: HaveWay
					else if ( identifier == MARKER_ID_HAVEWAY){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_HAVEWAY") );
						m_CurrentCrossingSign	= MARKER_ID_HAVEWAY;
						startModuleByID(STATE_TURNING);
					}

					// Crossing-Detection: GiveWay
					else if ( identifier == MARKER_ID_GIVEWAY){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_GIVEWAY" ) );
						m_CurrentCrossingSign 	= MARKER_ID_GIVEWAY;
						startModuleByID(STATE_TURNING);
					} 

					else if (identifier == MARKER_ID_AHEADONLY){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_AHEADONLY") );
						// TODO: a.t.m. implement this as a right before left crossing
						m_CurrentCrossingSign 	= MARKER_ID_UNMARKEDINTERSECTION;
						startModuleByID(STATE_TURNING);
					}

					else if ( identifier == MARKER_ID_ROUNDABOUT){
						LOG_INFO(cString::Format("DM: Used Sign MARKER_ID_ROUNDABOUT" ) );					
						m_CurrentCrossingSign 	= MARKER_ID_ROUNDABOUT;
						startModuleByID(STATE_TURNING);
					}
					else{

						// Free this section if no case is true
						m_processingRoadSign 	= tFalse;
					}
				}
				else{

					// Free this section if no case is true
					m_processingRoadSign 	= tFalse;
				}
			}

			// Free this section
			//m_processingRoadSign 	= tFalse;
		}
	}

	RETURN_NOERROR;
}

/*
 * Method to work with incoming Maneuvres
 */
tResult cDecisionMaking::processCurrentManeuvre(IMediaSample* pMediaSample){

	tInt32 intValue 		= ACTION_UNKNOWN;

	{
		__adtf_sample_read_lock_mediadescription(m_pCurrentManeuvreInput, pMediaSample, pCoder);    

		if (!m_bCurrentManeuvreInputSet){
			pCoder->GetID("intValue", m_szIDInt32ValueCurrentManeuvreInput);
			m_bCurrentManeuvreInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDInt32ValueCurrentManeuvreInput, (tVoid*)&intValue);        
	}

	// There was an error while transmitting maneuvre -> don't do anything
	if (intValue == ACTION_UNKNOWN) RETURN_NOERROR;

	// Do we have to start the pullout module?
	if (!m_currentlyPullOut && m_DMActive){
		if (intValue == ACTION_PULL_OUT_LEFT){
			m_checkDistance  	= tTrue;
			sendStatusPullOut(CROSS_LEFT, tTrue);
		}
		else if (intValue == ACTION_PULL_OUT_RIGHT){
			m_checkDistance 	= tTrue;
			if (m_CurrentManeuvre == ACTION_PARALLEL_PARKING){
				sendStatusPullOut(PARALLEL_RIGHT, tTrue);
			}
			else if (m_CurrentManeuvre == ACTION_CROSS_PARKING){
				sendStatusPullOut(CROSS_RIGHT, tTrue);
			}
			else{
				checkPullOutType(tTrue);
			}
		}
	}

	/*if (m_CurrentManeuvre == ACTION_PARALLEL_PARKING){
		if (intValue == ACTION_PULL_OUT_RIGHT){
			sendStatusPullOut(PARALLEL_RIGHT); 
		}
		else LOG_ERROR(cString::Format("There has to be a pullout maneuvre after parallel parking!") );
	}else if (m_CurrentManeuvre == ACTION_CROSS_PARKING){
		if (intValue == ACTION_PULL_OUT_RIGHT){
			sendStatusPullOut(CROSS_RIGHT);
		}else if (intValue == ACTION_PULL_OUT_LEFT){
			sendStatusPullOut(CROSS_LEFT);
		}else LOG_ERROR(cString::Format("There has to be a pullout maneuvre after cross parking!") );		
	}	*/

	tInt32 lastManeuvre  	= m_CurrentManeuvre;
	m_CurrentManeuvre 		= intValue;

	//LOG_INFO(cString::Format("DM: ManeuverCode = %i", intValue) );

	// We have finished driving:
	if(m_CurrentManeuvre == ACTION_FINISHED){
		if (lastManeuvre == ACTION_PARALLEL_PARKING || lastManeuvre == ACTION_CROSS_PARKING){ // If we parked at last then stop here
			finished();
		} else {
			m_checkLastDistance 		= tTrue;             // Drive one meter at the end
			m_referenceDistance			= 0.0; 	  // Deactivate slow drive check 
			m_checkDistance 			= tFalse; // (will not stop slow driving then!)
			LOG_INFO(cString::Format("DM: Drive last meter") );
		}
	}

	RETURN_NOERROR;
}

/*
 * This method filters the current car speed
 */
tResult cDecisionMaking::processCarSpeed(IMediaSample* pMediaSample){

	tFloat32 fSpeed = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pSignalCarSpeedInput, pMediaSample, pCoderOutput);

		if (!m_bCarSpeedInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueCarSpeedInput);
			m_bCarSpeedInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueCarSpeedInput, (tVoid*)&(fSpeed));
	}

	m_speedBuffer.push_back(fSpeed);

	int bufferSize 	= m_speedBuffer.size();

	// Compare 10 last elements
	if (bufferSize > 10){
		m_speedBuffer.pop_front();
		bufferSize--;
	}

	// Set actual speed as mean of last measures
	tFloat32 buffer = 0.0;
	for (int i = 0; i < bufferSize; i++){
		buffer 	+= m_speedBuffer.at(i);
	}

	m_currentSpeed 	= buffer/bufferSize;

	RETURN_NOERROR;
}

/*
 * This method saves the current driven overall distance
 */
tResult cDecisionMaking::processOverallDistance(IMediaSample* pMediaSample){

	tFloat32 fDistance 	= 0.0;	

	{
		__adtf_sample_read_lock_mediadescription(m_pDistanceInput,pMediaSample, pCoder);

		if (!m_bIDsDistanceSet) {
			pCoder->GetID("f32Value", m_szIDDistanceInput);
			m_bIDsDistanceSet = tTrue;
		}  

		pCoder->Get(m_szIDDistanceInput, (tVoid*)&(fDistance));
	
	}
	
	// Set new overall distance
	m_overallDistance 	= fDistance > m_overallDistance ? fDistance : m_overallDistance;

	if (m_checkCircleDistance && m_circleDistance == 0.0){
		LOG_INFO(cString::Format("DM: Check CircleDistance") );
		m_circleDistance 	= m_overallDistance;
	}

	if (m_checkStraightCircleDistance && m_StraightCircleDistance == 0.0){
		LOG_INFO(cString::Format("DM: Check CircleStraightDistance") );
		m_StraightCircleDistance 	= m_overallDistance;
	}
	
	if (m_checkLastDistance && m_lastDistanceCheckpoint == 0){
		LOG_INFO(cString::Format("DM: processing last distance!") );
		m_lastDistanceCheckpoint = m_overallDistance;
	}

	// Check if we have to start searching for circle exit
	if (m_checkCircleDistance && (m_circleDistance + CIRCLE_DISTANCE) <= m_overallDistance){
		LOG_INFO(cString::Format("DM: Start Looking for circle exit") );
		startModuleByID(STATE_TURNING);
		m_checkCircleDistance 	= tFalse;
		m_circleDistance	= 0.0;
	}

	// Check if we passed circle exit
	if (m_checkStraightCircleDistance && (m_StraightCircleDistance + CIRCLE_STRAIGHT_DISTANCE) <= m_overallDistance){
		LOG_INFO(cString::Format("DM: Passed circle exit") );
		incrementManeuvre();
		m_LFActive 				= tFalse;	
		startModuleByID(STATE_LANE_FOLLOWING);

		// Check distance if we are within a circle
		if (m_currentlyCircle){
			m_withinCircle 			= tTrue;
			LFCircleMode();
			m_checkCircleDistance 	= tTrue;
			m_circleDistance 		= 0.0;	
		}
		m_checkStraightCircleDistance 	= tFalse;
		m_StraightCircleDistance	= 0.0;
	}

	// Check if we have to stop the driving state
	if (m_checkLastDistance && m_lastDistanceCheckpoint + LAST_DISTANCE <= m_overallDistance){
		LOG_INFO(cString::Format("m_lastDistanceCheckpoint, m_overallDistance = %f, %f", m_lastDistanceCheckpoint, m_overallDistance) );
		m_checkLastDistance 		= tFalse;
		m_lastDistanceCheckpoint 	= 0.0;
		finished();
	}

	// Check if we reached normal driving state
	if (m_checkDistance && m_referenceDistance > 0.0 && m_overallDistance > (m_referenceDistance + DRIVE_SLOW_DIST)){
		setVelocityLF(MIN_VELOCITY, MAX_VELOCITY);
		m_referenceDistance	= 0.0;
		m_checkDistance 	= tFalse;
	}

	RETURN_NOERROR;
}

/*
 * This method filters the current steering angle
 */
tResult cDecisionMaking::processSteeringAngle(IMediaSample* pMediaSample){

	tFloat32 fSteering = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pSignalSteeringAngleInput, pMediaSample, pCoderOutput);

		if (!m_bSteeringAngleInputSet){
			pCoderOutput->GetID("f32Value", m_szIDSignalValueSteeringAngleInput);
			m_bSteeringAngleInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDSignalValueSteeringAngleInput, (tVoid*)&(fSteering));
	}

	m_steeringBuffer.push_back(fSteering);

	int bufferSize 	= m_steeringBuffer.size();

	// Compare 10 last elements
	if (bufferSize > 10){
		m_steeringBuffer.pop_front();
		bufferSize--;
	}

	// Set actual steering as mean of last measures
	tFloat32 buffer = 0.0;
	for (int i = 0; i < bufferSize; i++){
		buffer 	+= m_steeringBuffer.at(i);
	}

	m_currentSteering 	= buffer/bufferSize;

	// If we found a certain sign: start searching if wheels are approximately parallel! (If circle mode: ignore this!)
	if ((m_currentSteering < 90+STEERING_ERROR && m_currentSteering > 90-STEERING_ERROR) || m_currentlyCircle){
		// Parking
		if (m_ParkingMode > 0){

			searchForParkingSpot(tTrue, m_ParkingMode);
			m_ParkingMode 	= 0;
		}

		// Turning
		if(m_CurrentlyCrossing){

			//if (m_Counter < 2){
			//m_Counter ++;
			m_CurrentlyCrossing 	= tFalse;
			searchForCrossing(tTrue, m_CurrentManeuvre);
			//}else {
				//startModuleByID(STATE_HOLD);
			//	LOG_INFO(cString::Format("HOLD!") );
			//}
			//LOG_INFO(cString::Format("DM: Start Searching for Crossing Spot!") );
			//startModuleByID(STATE_HOLD);
		}

		// ATM: Start AdaptiveCruiseControl only with straight wheels
		// TODO: delete
		/*
		{
			if (m_LFActive && !m_CrossParkActive && !m_ParallelParkActive && !m_TurningActive){
				if (!m_ACCActive) 	sendStatusACC(m_ACCActive = tTrue);
			}
			else{
				if (m_ACCActive) 	sendStatusACC(m_ACCActive = tFalse);
			}
		}*/

	}
	// TODO: delete
	//else if (m_ACCActive) 		sendStatusACC(m_ACCActive = tFalse);

	RETURN_NOERROR;
}


/*
 * Method to process a found parking spot
 */
tResult cDecisionMaking::processFoundParkingSpot(IMediaSample* pMediaSample){

	tFloat32 distance;

	{
		__adtf_sample_read_lock_mediadescription(m_pFoundParkingSpotInput, pMediaSample, pCoder);    

		if (!m_bFoundParkingSpotInputSet){
			pCoder->GetID("f32Value", m_szIDSignalValueFoundParkingSpotInput);
			m_bFoundParkingSpotInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDSignalValueFoundParkingSpotInput, (tVoid*)&distance); 
	}
	/*
fstream f1;
f1.open("output2.dat",ios::out|ios::app);
for(size_t i=0; i<6;i++)
{
	f1 << "\n" << distances[i] ;
}
f1.close(); */

	LOG_INFO(cString::Format("DM: Received Parking Spot, distance = %f",distance));	

	if (distance > 0) {
		// At this moment: send first found point
		if (m_CurrentManeuvre == ACTION_PARALLEL_PARKING) sendStatusParallelPark(tTrue, distance);

		else if (m_CurrentManeuvre == ACTION_CROSS_PARKING) sendStatusCrossPark(tTrue, distance);
	}

	RETURN_NOERROR;
}

/*
 * Method to process a found crossing
 */
tResult cDecisionMaking::processFoundCrossing(IMediaSample* pMediaSample){

	tFloat32 distance 			= -1.0;
	tInt32 crossType 			= UNDEFINED_CROSSING;
	tFloat32 rotationDegree 	= 0.0;

	{
		__adtf_sample_read_lock_mediadescription(m_pFoundCrossingInput, pMediaSample, pCoder);    

		if (!m_bFoundCrossingInputSet){
			pCoder->GetID("crosstype", m_szIDFoundCrossingCrosstypeInput);
			pCoder->GetID("distance", m_szIDFoundCrossingDistanceInput);
			pCoder->GetID("rotationDegree", m_szIDFoundCrossingRotationDegreeInput);
			m_bFoundCrossingInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDFoundCrossingCrosstypeInput, (tVoid*)&crossType); 
		pCoder->Get(m_szIDFoundCrossingDistanceInput, (tVoid*)&distance);  
		pCoder->Get(m_szIDFoundCrossingRotationDegreeInput, (tVoid*)&rotationDegree);  
	}

	//LOG_INFO(cString::Format("DM: CrossType: %i, Distance: %f, Maneuvre: %i", crossType, distance, m_CurrentManeuvre) );

	// Start the Turning-Maneuvre
	if ( distance > 0 && (m_CurrentManeuvre == ACTION_LEFT || m_CurrentManeuvre == ACTION_RIGHT || m_CurrentManeuvre == ACTION_STRAIGHT) ) {

		//LOG_INFO(cString::Format("DM: Found crossing, dist: %f", distance) );
		//startModuleByID(STATE_HOLD);
		sendStatusTurning(tTrue, distance, crossType, m_CurrentCrossingSign, m_CurrentManeuvre, rotationDegree);
	}
	else{ // Couldn't find crossing

		LOG_WARNING(cString::Format("DM: Invalid Crossing Data!") );
 
		incrementManeuvre();
		m_processingRoadSign 	= tFalse;
		setVelocityLF(MIN_VELOCITY, MAX_VELOCITY);
		overtakingAllowed(tTrue, tTrue, tTrue);
		m_LFActive 				= tFalse;
		m_referenceDistance 	= 0.0;
		m_checkDistance 		= tFalse;
		startModuleByID(STATE_LANE_FOLLOWING);
	}

	RETURN_NOERROR;
}

/*
 * Method to evaluate found ZebraStripe distances
 */
tResult cDecisionMaking::processFoundZebraStripes(IMediaSample* pMediaSample){
	
	tFloat32 nearDist 	= -1;
	tFloat32 farDist 	= -1;

	{
		__adtf_sample_read_lock_mediadescription(m_pFoundZebrasInput, pMediaSample, pCoder);    

		if (!m_bFoundZebrasInputSet){
			pCoder->GetID("neardist", m_szIDZebraNearInput);
			pCoder->GetID("fardist", m_szIDZebraFarInput);
			m_bFoundZebrasInputSet = tTrue;
		}

		// set value from sample
		pCoder->Get(m_szIDZebraNearInput, (tVoid*)&nearDist); 
		pCoder->Get(m_szIDZebraFarInput, (tVoid*)&farDist);  
	}

	// Check pedestrain crossing
	if ( nearDist > 0 && farDist > 0) {

		//LOG_INFO(cString::Format("DM: Found Zebra %f | %f", nearDist, farDist) );
		sendStatusPedestrainCrossing(nearDist, farDist, tTrue);
		//startModuleByID(STATE_HOLD);
	}
	else{
		LOG_WARNING(cString::Format("DM: Invalid zebra stripe Data!") );
		m_processingRoadSign 	= tFalse;
		setVelocityLF(MIN_VELOCITY_AFTER_ACTION, MAX_VELOCITY_AFTER_ACTION);
		overtakingAllowed(tTrue, tTrue, tTrue); // TODO: eigentlich der erste pin nicht true, da verbotsschild aber nicht auftaucht, verhindert dies fehler..
		//m_LFActive 				= tFalse;
		m_referenceDistance 	= m_overallDistance;	
		startModuleByID(STATE_LANE_FOLLOWING);
	}

	RETURN_NOERROR;
}

/*
 * Process the current park slot and start correct pull out maneuvre
 */
tResult cDecisionMaking::ProcessParkLotType(IMediaSample* pMediaSample){
	
	tBool isCrossPark 	= tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pCheckedParkLotInput ,pMediaSample, pCoder);    

		if (!m_bCheckedParkLotInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueCheckedParkLotInput);		
			m_bCheckedParkLotInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueCheckedParkLotInput, (tVoid*)&isCrossPark);         
	}

	LOG_INFO(cString::Format("DM: ParkLotType is CrossPark = %i", isCrossPark) );

	// Send Pullout maneuvre!
	if (isCrossPark)	sendStatusPullOut(CROSS_RIGHT, tTrue);
	else 				sendStatusPullOut(PARALLEL_RIGHT, tTrue);
		

	RETURN_NOERROR;
	
}

/*
 * This method receives if we finished the parking maneuver and sets the next state
 */
tResult cDecisionMaking::processFinishedManeuvre_Parking(IMediaSample* pMediaSample){

	tBool finished = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolFinishedParkingInput ,pMediaSample, pCoder);    

		if (!m_bFinishedParkingInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueFinishedParkingInput);		
			m_bFinishedParkingInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueFinishedParkingInput, (tVoid*)&finished);         
	}

	LOG_INFO(cString::Format("DM: Received finished_parking with = %i", finished) );

	// Finished parking succesfully?
	if (finished){
		incrementManeuvre(); // check at the maneuvre receiving function if we have to start the pullout module
	}

	RETURN_NOERROR;
}

/*
 *This method processes the finished pullout maneuvre
 */
tResult cDecisionMaking::processFinishedManeuvre_PullOut(IMediaSample* pMediaSample){

	tBool finished = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolFinishedPulloutInput ,pMediaSample, pCoder);    

		if (!m_bFinishedPulloutInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueFinishedPulloutInput);		
			m_bFinishedPulloutInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueFinishedPulloutInput, (tVoid*)&finished);         
	}

	LOG_INFO(cString::Format("DM: Received finished_pullout with = %i", finished) );

	// Finished pullout?
	if (finished){
		m_currentlyPullOut 		= tFalse;
		incrementManeuvre();
		m_processingRoadSign 	= tFalse;
		setVelocityLF(MIN_VELOCITY_AFTER_ACTION, MAX_VELOCITY_AFTER_ACTION);
		overtakingAllowed(tTrue, tTrue, tTrue);
		m_LFActive 				= tFalse;
		m_referenceDistance 	= m_overallDistance;	
		startModuleByID(STATE_LANE_FOLLOWING);
	}

	RETURN_NOERROR;
}

/*
 * This method receives if we finished the turning maneuver and sets the next state
 */
tResult cDecisionMaking::processFinishedManeuvre_Turning(IMediaSample* pMediaSample){

	tBool finished = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolFinishedTurningInput ,pMediaSample, pCoder);    

		if (!m_bFinishedTurningInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueFinishedTurningInput);		
			m_bFinishedTurningInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueFinishedTurningInput, (tVoid*)&finished);         
	}

	LOG_INFO(cString::Format("DM: Received finished_turning with = %i", finished) );

	// Finished turning succesfully?
	if (finished){
		incrementManeuvre();
		m_processingRoadSign 	= tFalse;
		setVelocityLF(MIN_VELOCITY_AFTER_ACTION, MAX_VELOCITY_AFTER_ACTION);
		overtakingAllowed(tTrue, tTrue, tTrue);
		m_LFActive 				= tFalse;
		m_referenceDistance 	= m_overallDistance;	
		startModuleByID(STATE_LANE_FOLLOWING);

		LFCircleMode(); // TODO: Put this at the end of this method?

		// Check distance if we are within a circle
		if (m_currentlyCircle){
			m_withinCircle 			= tTrue;
			//LFCircleMode();
			m_checkCircleDistance 	= tTrue;
			m_circleDistance 		= 0.0;	
		}
	}

	RETURN_NOERROR;
}

/*
 * Method to set new states if we finished zebra stripe
 */
tResult cDecisionMaking::processFinisehdManeuvre_ZebraStripes(IMediaSample* pMediaSample){
	
	tBool finished = tFalse;

	{
		__adtf_sample_read_lock_mediadescription(m_pBoolFinishedZebraStripeInput ,pMediaSample, pCoder);    

		if (!m_bFinishedZebraStripeInputSet){
			pCoder->GetID("bValue", m_szIDBoolValueFinishedZebraStripeInput);		
			m_bFinishedZebraStripeInputSet = tTrue;
		}

		pCoder->Get(m_szIDBoolValueFinishedZebraStripeInput, (tVoid*)&finished);         
	}

	LOG_INFO(cString::Format("DM: Received finished_zebra with = %i", finished) );

	// Finished Zabra stripes succesfully?
	if (finished){		
		m_processingRoadSign 	= tFalse;
		setVelocityLF(MIN_VELOCITY_AFTER_ACTION, MAX_VELOCITY_AFTER_ACTION);
		overtakingAllowed(tTrue, tTrue, tTrue); // TODO: eigentlich der erste pin nicht true, da verbotsschild aber nicht auftaucht, verhindert dies fehler..
		//m_LFActive 				= tFalse;
		m_referenceDistance 	= m_overallDistance;	
		startModuleByID(STATE_LANE_FOLLOWING);
	}

	RETURN_NOERROR;
}

/*
 * Method receives signal, wheter we are currently overtaking an obstacle
 */
tResult cDecisionMaking::processFinishedManeuvre_Overtaking(IMediaSample* pMediaSample){

	tBool state = 0;

	{
		__adtf_sample_read_lock_mediadescription(m_pFinishedOvertakingInput, pMediaSample, pCoderOutput);

		if (!m_bFinishedOvertakingInputSet){
			pCoderOutput->GetID("bValue", m_szIDFinishedOvertakingInput);
			m_bFinishedOvertakingInputSet = tTrue;
		}

		pCoderOutput->Get(m_szIDFinishedOvertakingInput, (tVoid*)&(state));
	}

	LOG_INFO(cString::Format("DM: Received finished_overtaking with = %i", state) );

	// Set state
	m_currentlyOvertaking 	= state;

	// set correct velocity
	if (m_currentlyOvertaking){
		setVelocityLF(MIN_OVERTAKING_VELOCITY, MAX_OVERTAKING_VELOCITY);
		sendStatusACC(tFalse);
	}else{
		setVelocityLF(MIN_VELOCITY, MAX_VELOCITY);
		sendStatusACC(tTrue);
	}
	
	RETURN_NOERROR;
}

/*
 * Method to make sure that only one module is working.
 * ID's of the modules are:
 * 
 * 0 		- 	Hold
 * 1 		-  	Lane following
 * 2 		- 	Cross Parking
 * 3 		- 	Parallel Parking
 * 4 		- 	Turning
 *
 */
tResult cDecisionMaking::startModuleByID(int state){

	LOG_INFO(cString::Format("DM: Start Module %i", state) );
	
	// if we have mode TURNING or PARKING then we are not allowed to overtake:

	// Module 'Hold'
	if (!m_DMActive) 																sendStatusHold(m_HoldActive = tTrue);
	else if (state != STATE_HOLD && m_HoldActive)									sendStatusHold(m_HoldActive = tFalse);
	else if (state == STATE_HOLD && !m_HoldActive)									sendStatusHold(m_HoldActive = tTrue);

	// Lane following (Stay active when parking, turning or zebra is activated)
	if ( ( (state != STATE_LANE_FOLLOWING && state != STATE_CROSS_PARKING 
			&& state != STATE_PARALLEL_PARKING && state != STATE_TURNING 
			&& state != STATE_ZEBRA) && m_LFActive ) || !m_DMActive) 				sendStatusLF(m_LFActive = tFalse);
	else if (state == STATE_LANE_FOLLOWING && !m_LFActive && !m_ZebraActive) 		sendStatusLF(m_LFActive = tTrue);

	// Cross Parking
	if ( (state != STATE_CROSS_PARKING && m_CrossParkActive) || !m_DMActive)		{m_CrossParkActive =  tFalse; searchForParkingSpot(tFalse, ACTION_UNKNOWN); sendStatusCrossPark(tFalse, 0.0); }
	else if (state == STATE_CROSS_PARKING && !m_CrossParkActive) 					{m_CrossParkActive 	= tTrue; m_checkDistance = tTrue; m_referenceDistance = 0.0; m_ParkingMode = ACTION_CROSS_PARKING; overtakingAllowed(m_OvertakingAlowed_RoadSign, tFalse, m_OvertakingAlowed_Pedestrain);}

	// Parallel Parking
	if ( (state != STATE_PARALLEL_PARKING && m_ParallelParkActive) || !m_DMActive)	{m_ParallelParkActive = tFalse; searchForParkingSpot(tFalse, ACTION_UNKNOWN); sendStatusParallelPark(tFalse, 0.0);}
	else if (state == STATE_PARALLEL_PARKING && !m_ParallelParkActive) 				{m_ParallelParkActive = tTrue; m_checkDistance = tTrue; m_referenceDistance = 0.0; m_ParkingMode = ACTION_PARALLEL_PARKING; overtakingAllowed(m_OvertakingAlowed_RoadSign, tFalse, m_OvertakingAlowed_Pedestrain);}

	// Turning
	if ( (state != STATE_TURNING && m_TurningActive) || !m_DMActive)				{m_TurningActive = tFalse; searchForCrossing(tFalse, ACTION_UNKNOWN); sendStatusTurning(tFalse, 0.0, UNDEFINED_CROSSING, MARKER_ID_NOMATCH, ACTION_UNKNOWN, 0.0);}
	else if (state == STATE_TURNING && !m_TurningActive) 							{m_TurningActive = tTrue; m_checkDistance = tTrue; m_referenceDistance = 0.0; m_CurrentlyCrossing = tTrue; overtakingAllowed(m_OvertakingAlowed_RoadSign, tFalse, m_OvertakingAlowed_Pedestrain);}
	
	// Zebra Stripes
	if ( (state != STATE_ZEBRA && m_ZebraActive) || !m_DMActive) 					{m_ZebraActive = tFalse; searchForZebraStripes(tFalse); sendStatusPedestrainCrossing(0.0, 0.0, tFalse);}
	else if (state == STATE_ZEBRA && !m_ZebraActive) 								{m_ZebraActive = tTrue; m_checkDistance = tTrue; m_referenceDistance = 0.0; searchForZebraStripes(tTrue); overtakingAllowed(m_OvertakingAlowed_RoadSign, m_OvertakingAlowed_Maneuvre, tFalse);}


	RETURN_NOERROR;
}

/*
 * This Method sends a working state to the Hold-Module
 */
tResult cDecisionMaking::sendStatusHold(tBool status){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStateHoldOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = 0;
	tBool bValue = status;

	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStateHoldOutput, pMediaSample, pCoder);    

		if (!m_bStateHoldOutputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStateHoldOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampStateHoldOutput);
			m_bStateHoldOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueStateHoldOutput, (tVoid*)&bValue);     
		pCoder->Set(m_szIDTimestampStateHoldOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStateHold.Transmit(pMediaSample);

	RETURN_NOERROR; 
}


/*
 * This Method sends a working state to the Lane Following-Module
 */
tResult cDecisionMaking::sendStatusLF(tBool status){   

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStateLFOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = _clock->GetStreamTime();
	tBool bValue = status;

	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStateLFOutput, pMediaSample, pCoder);    

		if (!m_bStateLFOutputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStateLFOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampStateLFOutput);
			m_bStateLFOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueStateLFOutput, (tVoid*)&bValue);     
		pCoder->Set(m_szIDTimestampStateLFOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStateLF.Transmit(pMediaSample);

	RETURN_NOERROR;   
}

/*
 * This Method sends a working state to the Cross-Parking-Module
 */
tResult cDecisionMaking::sendStatusCrossPark(tBool status, tFloat32 pDistance){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStateCrossParkOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp 	= _clock->GetStreamTime();
	tBool 		bValue 		= status;
	tFloat32 	velocity 	= m_currentSpeed;
	tFloat32 	distance 	= pDistance + CROSS_PARK_DISTANCE_OFFSET;
	/*
	fstream fss;
	fss.open("output2.dat",ios::out|ios::app);
	fss << "pDistance: " << pDistance << ", distance: " << distance << "\n";
	fss.close(); */

	{
		__adtf_sample_write_lock_mediadescription(m_pStateCrossParkOutput, pMediaSample, pCoder);    

		if (!m_bCrossParkOutputSet){
			pCoder->GetID("bStart", m_szIDBoolValueStateCrossParkOutput);
			pCoder->GetID("fVelocity", m_szIDVelocityStateCrossParkOutput);
			pCoder->GetID("fDistance", m_szIDDistanceStateCrossParkOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampStateCrossParkOutput);
			m_bCrossParkOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueStateCrossParkOutput, (tVoid*)&bValue);     
		pCoder->Set(m_szIDVelocityStateCrossParkOutput, (tVoid*)&velocity);
		pCoder->Set(m_szIDDistanceStateCrossParkOutput, (tVoid*)&distance);
		pCoder->Set(m_szIDTimestampStateCrossParkOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	//LOG_INFO(cString::Format("DM: Start CrossPark %f, %f, %i", velocity, distance, bValue) );
	//RETURN_NOERROR;

	m_oStateCrossPark.Transmit(pMediaSample);

	RETURN_NOERROR; 
}

/*
 * This Method sends a working state to the Parallel-Parking-Module
 */
tResult cDecisionMaking::sendStatusParallelPark(tBool status, tFloat32 pDistance){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStateParallelParkOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp 	= _clock->GetStreamTime();
	tBool	 	bValue 		= status;
	tFloat32 	velocity 	= m_currentSpeed;
	tFloat32 	distance 	= pDistance + PARALLEL_PARK_DISTANCE_OFFSET;

	{
		__adtf_sample_write_lock_mediadescription(m_pStateParallelParkOutput, pMediaSample, pCoder);    

		if (!m_bParallelParkOutputSet){
			pCoder->GetID("bStart", m_szIDBoolValueStateParallelParkOutput);
			pCoder->GetID("fVelocity", m_szIDVelocityStateParallelParkOutput);
			pCoder->GetID("fDistance", m_szIDDistanceStateParallelParkOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampStateParallelParkOutput);
			m_bParallelParkOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueStateParallelParkOutput, (tVoid*)&bValue);     
		pCoder->Set(m_szIDVelocityStateParallelParkOutput, (tVoid*)&velocity);
		pCoder->Set(m_szIDDistanceStateParallelParkOutput, (tVoid*)&distance);
		pCoder->Set(m_szIDTimestampStateParallelParkOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStateParallelPark.Transmit(pMediaSample);

	RETURN_NOERROR; 
}

/*
 * This method starts the pull out module
 */
tResult cDecisionMaking::sendStatusPullOut(tInt32 status, tBool state){

	m_currentlyPullOut 	= state;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStatePullOutOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	//LOG_INFO(cString::Format("DM: Start PullOut with ID = %i",status) );

	{
		__adtf_sample_write_lock_mediadescription(m_pStatePullOutOutput, pMediaSample, pCoder); 
   
		if (!m_bStatePullOutOutputSet){
			pCoder->GetID("intValue", m_szIDStatePullOutOutput);
			m_bStatePullOutOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDStatePullOutOutput, (tVoid*)&status); 
  
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStatePullOut.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * This Method sends a working state to the Turning-Module
 */
tResult cDecisionMaking::sendStatusTurning(tBool status, tFloat pDistance, tInt32 pCrossType, tInt16 pCrossingSign, tInt32 pManeuvre, tFloat32 pRotationDegree){

	if (m_currentlyCircle && m_withinCircle && pManeuvre == ACTION_STRAIGHT){
		m_checkStraightCircleDistance 	= tTrue;
		m_StraightCircleDistance 		= 0.0;
		setVelocityLF(MIN_VELOCITY, MAX_VELOCITY);	

	}else{

		cObjectPtr<IMediaSample> pMediaSample;
		AllocMediaSample((tVoid**)&pMediaSample);

		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pStateTurningOutput->GetMediaSampleSerializer(&pSerializer);
		pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

		tUInt32 	ui32TimeStamp 	= _clock->GetStreamTime();
		tBool	 	bValue 			= status;
		tFloat32 	velocity 		= m_currentSpeed;
		tInt32		mode			= pManeuvre;
		tInt32 		type 			= (pCrossType == UNDEFINED_CROSSING) ? CROSSROAD : pCrossType;
		tBool		pullOutMode		= tFalse;

		// Check circle mode
		if (pCrossingSign == MARKER_ID_ROUNDABOUT) 							m_currentlyCircle 	= tTrue;					// Drive into circle
		tBool bCircle				= m_currentlyCircle;
		tBool bIsInCircle 			= m_withinCircle;

		// Set accurate rotationDegree
		tFloat32 	rotationDegree 	= m_currentlyCircle ? CIRCLE_DEGREE : pRotationDegree;
		rotationDegree = (m_currentlyCircle && m_withinCircle && mode == ACTION_RIGHT) ? CIRCLE_EXIT_DEGREE : rotationDegree;

		// Distance to crossing
		tFloat turningOffset 		= m_currentlyCircle ? - TURNING_OFFSET + CIRCLE_OFFSET : - TURNING_OFFSET;
		tFloat32 	distance 		= pDistance + turningOffset < 0 ? 0.001 : pDistance + turningOffset;

		if (m_currentlyCircle && mode == ACTION_RIGHT && m_withinCircle) 	{m_currentlyCircle 	= m_withinCircle = tFalse;} 	// Drive out of circle

		// Check giveWayType!
		tInt32		giveWay;
		if (pCrossingSign == MARKER_ID_UNMARKEDINTERSECTION){
			giveWay 	= RIGHTBEFORELEFT;
		}
		else if (pCrossingSign == MARKER_ID_HAVEWAY){
			giveWay 	= HAVEWAY;
		}
		else if (pCrossingSign == MARKER_ID_STOPANDGIVEWAY || pCrossingSign == MARKER_ID_GIVEWAY || pCrossingSign == MARKER_ID_ROUNDABOUT){
			giveWay 	= GIVEWAY;
		}
		else{
			giveWay 	= UNDEFINED_SIGN;
		}

		// Check stopSign type
		tBool 		stopSign 		= (pCrossingSign == MARKER_ID_STOPANDGIVEWAY) ? tTrue : tFalse;

		//LOG_INFO(cString::Format("DM: StartTurning stopSign %i | giveWay %i | type %i | mode %i | ", stopSign, giveWay, type, mode) );

		{
			__adtf_sample_write_lock_mediadescription(m_pStateTurningOutput, pMediaSample, pCoder);    

			if (!m_bTurningOutputSet){
				pCoder->GetID("bStart", m_szIDStateTurningStatusOutput);
				pCoder->GetID("fVelocity", m_szIDStateTurningVelocityOutput);
				pCoder->GetID("fDistance", m_szIDStateTurningDistanceOutput);
				pCoder->GetID("mode", m_szIDStateTurningModeOutput);
				pCoder->GetID("type", m_szIDStateTurningTypeOutput);
				pCoder->GetID("giveWay", m_szIDStateTurningGiveWayOutput);
				pCoder->GetID("stopSign", m_szIDStateTurningStopSignOutput);
				pCoder->GetID("bPullOutMode", m_szIDStateTurningPullOutModeOutput);
				pCoder->GetID("ui32ArduinoTimestamp", m_szIDStateTurningTimestampOutput);
				pCoder->GetID("rotationDegree", m_szIDStateTurningRotationDegreeOutput);
				pCoder->GetID("bCircle", m_szIDStateTurningCircleOutput);
				pCoder->GetID("bIsInCircle", m_szIDStateTurningIsInCircleOutput);
				m_bTurningOutputSet = tTrue;
			}

			// set value from sample
			pCoder->Set(m_szIDStateTurningStatusOutput, (tVoid*)&bValue);
			pCoder->Set(m_szIDStateTurningVelocityOutput, (tVoid*)&velocity);
			pCoder->Set(m_szIDStateTurningDistanceOutput, (tVoid*)&distance);
			pCoder->Set(m_szIDStateTurningModeOutput, (tVoid*)&mode);
			pCoder->Set(m_szIDStateTurningTypeOutput, (tVoid*)&type);
			pCoder->Set(m_szIDStateTurningGiveWayOutput, (tVoid*)&giveWay);
			pCoder->Set(m_szIDStateTurningStopSignOutput, (tVoid*)&stopSign);
			pCoder->Set(m_szIDStateTurningPullOutModeOutput, (tVoid*)&pullOutMode);
			pCoder->Set(m_szIDStateTurningTimestampOutput, (tVoid*)&ui32TimeStamp);
			pCoder->Set(m_szIDStateTurningRotationDegreeOutput, (tVoid*)&rotationDegree);
			pCoder->Set(m_szIDStateTurningCircleOutput, (tVoid*)&bCircle);
			pCoder->Set(m_szIDStateTurningIsInCircleOutput, (tVoid*)&bIsInCircle);
		}

		pMediaSample->SetTime(_clock->GetStreamTime());

		m_oStateTurning.Transmit(pMediaSample);
	}

	RETURN_NOERROR; 
}

/*
 * Method that works with found zebra stripes
 */
tResult cDecisionMaking::sendStatusPedestrainCrossing(tFloat32 nearDist, tFloat32 farDist, tBool state){
	
	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pStatePCOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pStatePCOutput, pMediaSample, pCoder);    

		if (!m_szIDStatePCOutputSet){
			pCoder->GetID("neardist", m_szIDStatePCnearDistOutput);
			pCoder->GetID("fardist", m_szIDStatePCfarDistOutput);
			pCoder->GetID("fVelocity", m_szIDStatePCvelocityOutput);
			pCoder->GetID("start", m_szIDStatePCOutput);
			m_szIDStatePCOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDStatePCnearDistOutput, (tVoid*)&nearDist);
		pCoder->Set(m_szIDStatePCfarDistOutput, (tVoid*)&farDist);
		pCoder->Set(m_szIDStatePCvelocityOutput, (tVoid*)&m_currentSpeed);
		pCoder->Set(m_szIDStatePCOutput, (tVoid*)&state);
	}

	LOG_INFO(cString::Format("DM: stop: %f, %f, %f, %i", m_currentSpeed, nearDist, farDist, state ));

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStatePedestrain.Transmit(pMediaSample);

	RETURN_NOERROR;  
}

/*
 * This Method sends a working state to the AdaptiveCruiseControl-Module
 */
tResult cDecisionMaking::sendStatusACC(tBool status){   

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStateACCOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tBool bValue = status;

	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStateACCOutput, pMediaSample, pCoder);    

		if (!m_bStateACCOutputSet){
			pCoder->GetID("bValue", m_szIDBoolValueStateACCOutput);
			m_bStateACCOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueStateACCOutput, (tVoid*)&bValue);       
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oStateACC.Transmit(pMediaSample);

	RETURN_NOERROR;   
}

/*
 * This Method sends an 'increment' signal to the state controller
 */
tResult cDecisionMaking::incrementManeuvre(){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolIncrementManeuvreOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = _clock->GetStreamTime();
	tBool bValue = tTrue;

	{
		__adtf_sample_write_lock_mediadescription(m_pBoolIncrementManeuvreOutput, pMediaSample, pCoder);    

		if (!m_bIncrementManeuvreOutputSet){
			pCoder->GetID("bValue", m_szIDBoolValueIncrementManeuvreOutput);
			pCoder->GetID("ui32ArduinoTimestamp", m_szIDTimestampIncrementManeuvreOutput);
			m_bIncrementManeuvreOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueIncrementManeuvreOutput, (tVoid*)&bValue);     
		pCoder->Set(m_szIDTimestampIncrementManeuvreOutput, (tVoid*)&(ui32TimeStamp));    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oIncrementManeuvre.Transmit(pMediaSample);

	RETURN_NOERROR; 
}

/*
 * This Method starts the parkingSpot detection filter
 */
tResult cDecisionMaking::searchForParkingSpot(tBool status, tInt32 ID) {
	
	// Only drive slow if we want to START the searching algorithm
	if (status) setVelocityLF(ACTION_VELOCITY, ACTION_VELOCITY);			

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pSearchParkingSpotOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pSearchParkingSpotOutput, pMediaSample, pCoder);    

		if (!m_bSearchParkingSpotOutputSet) {
			pCoder->GetID("ID", m_szIDSearchParkingSpotOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchParkingSpotOutput);
			m_bSearchParkingSpotOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDSearchParkingSpotOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchParkingSpotOutput, (tVoid*)&status);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oSearchParkingSpot.Transmit(pMediaSample);

	RETURN_NOERROR; 
}

/*
 * This Method starts the Crossing detection filter
 */
tResult cDecisionMaking::searchForCrossing(tBool status, tInt32 ID) {

	// no circles implemented yet
	tBool isCircle 	= m_currentlyCircle;

	// Only drive slow if we want to START the searching algorithm
	if (status) setVelocityLF(ACTION_VELOCITY, ACTION_VELOCITY);				

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pSearchCrossingOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pSearchCrossingOutput, pMediaSample, pCoder);    

		if (!m_bSearchCrossingOutputSet) {
			pCoder->GetID("ID", m_szIDSearchCrossingOutput);
			pCoder->GetID("bStart", m_szIDBoolValueSearchCrossingOutput);
			pCoder->GetID("bIsCircleMode", m_szIDBoolValueIsCircleOutput);
			m_bSearchCrossingOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDSearchCrossingOutput, (tVoid*)&ID);
		pCoder->Set(m_szIDBoolValueSearchCrossingOutput, (tVoid*)&status);
		pCoder->Set(m_szIDBoolValueIsCircleOutput, (tVoid*)&isCircle);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oSearchCrossing.Transmit(pMediaSample);


	RETURN_NOERROR; 
}

/*
 * Method to start the zebraStripes-detection
 */
tResult cDecisionMaking::searchForZebraStripes(tBool status){
	
	tBool state = status;  

	// Only drive slow if we want to START the searching algorithm
	if (status) setVelocityLF(ACTION_VELOCITY, ACTION_VELOCITY);	
	
	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pSearchZebraOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		__adtf_sample_write_lock_mediadescription(m_pSearchZebraOutput, pMediaSample, pCoder);    

		if (!m_bSearchZebraOutputSet){
			pCoder->GetID("bValue", m_szIDBoolValueSearchZebraOutput);
			m_bSearchZebraOutputSet = tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDBoolValueSearchZebraOutput, (tVoid*)&state);       
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oSearchZebraStripes.Transmit(pMediaSample);

	RETURN_NOERROR; 
}

/*
 * Method to set Velocity of LaneFollowing
 */ 
tResult cDecisionMaking::setVelocityLF(tFloat32 fMin, tFloat32 fMax){

	tUInt32 nTimeStamp = 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pVelocityOutput->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();
	pMediaSample->AllocBuffer(nSize);

	{
		__adtf_sample_write_lock_mediadescription(m_pVelocityOutput, pMediaSample, pCoder); 


		if (!m_bVelocityOutputSet){
			pCoder->GetID("fMinVel", m_szIDVelocityMinOutput);
			pCoder->GetID("fMaxVel", m_szIDVelocityMaxOutput);
			pCoder->GetID("ui32ArduinoTimeStamp", m_szIDTimestampVelocityOutput);
			m_bVelocityOutputSet = tTrue;
		}

		pCoder->Set(m_szIDVelocityMinOutput, (tVoid*)&(fMin));
		pCoder->Set(m_szIDVelocityMaxOutput, (tVoid*)&(fMax));
		pCoder->Set(m_szIDTimestampVelocityOutput, (tVoid*)&nTimeStamp);

	}

	fstream f;
	f.open("DM_LTVelocityOutput", ios::out|ios::app);
	f << fMin << "\t" << fMax << "\n";
	f.close();

	pMediaSample->SetTime(pMediaSample->GetTime());

	m_oVelocityLF.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * Send Overtaking-Alowed-Status to the adaptive cruis control module
 */
tResult cDecisionMaking::overtakingAllowed(tBool RoadSignAllowed, tBool ManeuvreAllowed, tBool PedestrainAllowed)
{
	
	tBool somethingChanged 	= tFalse;
	
	if (m_OvertakingAlowed_RoadSign != RoadSignAllowed){
		m_OvertakingAlowed_RoadSign		= RoadSignAllowed;
		somethingChanged 				= tTrue;
	}
	
	if (m_OvertakingAlowed_Maneuvre != ManeuvreAllowed){
		m_OvertakingAlowed_Maneuvre		= ManeuvreAllowed;
		somethingChanged 				= tTrue;
	}
	
	if (m_OvertakingAlowed_Pedestrain != PedestrainAllowed){
		m_OvertakingAlowed_Pedestrain		= PedestrainAllowed;
		somethingChanged 					= tTrue;
	}
	
	// Only send pin if there are some changes
	if (somethingChanged){		
	
		cObjectPtr<IMediaSample> pNewSample;
		AllocMediaSample((tVoid**)&pNewSample);
	
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pOvertakingOutput->GetMediaSampleSerializer(&pSerializer);
		pNewSample->AllocBuffer(pSerializer->GetDeserializedSize());

		{
			__adtf_sample_write_lock_mediadescription(m_pOvertakingOutput, pNewSample, pCoderOutput); 
			
			if (!m_bOvertakingOutputSet){
				pCoderOutput->GetID("RoadSign", m_szIDOvertakingRoadSignOutput);
				pCoderOutput->GetID("Maneuvre", m_szIDOvertakingManeuvreOutput);
				pCoderOutput->GetID("Pedestrain", m_szIDOvertakingPedestrainOutput);
				m_bOvertakingOutputSet = tTrue;
			}
			
			pCoderOutput->Set(m_szIDOvertakingRoadSignOutput, (tVoid*)&(m_OvertakingAlowed_RoadSign));
			pCoderOutput->Set(m_szIDOvertakingManeuvreOutput, (tVoid*)&(m_OvertakingAlowed_Maneuvre));
			pCoderOutput->Set(m_szIDOvertakingPedestrainOutput, (tVoid*)&(m_OvertakingAlowed_Pedestrain));
		
		}
	
		pNewSample->SetTime(pNewSample->GetTime());
		m_oOvertaking.Transmit(pNewSample);
	
	}
    
    RETURN_NOERROR;

}

/*
 * Check in what kind of parking area the car is standing
 */
tResult cDecisionMaking::checkPullOutType(tBool state){
	
	m_currentlyPullOut 	= state;

	tUInt32 nTimeStamp 	= 0;

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pCheckPullOutOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


	{
		__adtf_sample_write_lock_mediadescription(m_pCheckPullOutOutput, pMediaSample, pCoderOutput); 

		if(!m_bCheckPullOutOutputSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueCheckPullOutOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampCheckPullOutOutput);
			m_bCheckPullOutOutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueCheckPullOutOutput, (tVoid*)&(state));
		pCoderOutput->Set(m_szIDTimestampCheckPullOutOutput, (tVoid*)&nTimeStamp);
	
	}

	pMediaSample->SetTime(pMediaSample->GetTime());
	m_oCheckPullOut.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * This method sends the circle mode to the lane tracking module
 */
tResult cDecisionMaking::LFCircleMode(){
	
	cObjectPtr<IMediaSample> pNewSample;
	AllocMediaSample((tVoid**)&pNewSample);

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pLFCircleModeOutput->GetMediaSampleSerializer(&pSerializer);
	pNewSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 nTimeStamp 	= _clock->GetStreamTime();

	{
		__adtf_sample_write_lock_mediadescription(m_pLFCircleModeOutput, pNewSample, pCoderOutput); 
		
		if (!m_bLFCircleModeOutputSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueLFCircleModeOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampLFCircleModeOutput);
			m_bLFCircleModeOutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueLFCircleModeOutput, (tVoid*)&(m_withinCircle));
		pCoderOutput->Set(m_szIDTimestampLFCircleModeOutput, (tVoid*)&(nTimeStamp));
	
	}

	//LOG_INFO(cString::Format("DM: Send circle mode to LT: %i", m_withinCircle) );

	pNewSample->SetTime(pNewSample->GetTime());
	m_oLFCircleMode.Transmit(pNewSample);

	RETURN_NOERROR;
}

/*
 * Send complete state to jury module
 */
tResult cDecisionMaking::sendStateCompleted(tBool state){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pBoolStateCompleteOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tUInt32 ui32TimeStamp = 0;

	{
		__adtf_sample_write_lock_mediadescription(m_pBoolStateCompleteOutput, pMediaSample, pCoderOutput); 

		if(!m_bStateCompleteOutputSet){
			pCoderOutput->GetID("bValue", m_szIDBoolValueStateCompleteOutput);
			pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIDTimestampStateCompleteOutput);
			m_bStateCompleteOutputSet = tTrue;
		}
		
		pCoderOutput->Set(m_szIDBoolValueStateCompleteOutput, (tVoid*)&(state));
		pCoderOutput->Set(m_szIDTimestampStateCompleteOutput, (tVoid*)&ui32TimeStamp);
	
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	//transmit media sample over output pin
	m_oStateComplete.Transmit(pMediaSample);

	RETURN_NOERROR;
}

/*
 * Reset this filter
 */
tResult cDecisionMaking::finished(){
	
	LOG_INFO(cString::Format("DM: Finished Parkours") );
	startModuleByID(STATE_HOLD);
	sendStateCompleted(tTrue);
	setToInitialState(tFalse);

	RETURN_NOERROR;
}

/*
 * Set all variables to initial state
 */
tResult cDecisionMaking::setToInitialState(tBool allValues){

	if (allValues){
		m_HoldActive 				= tFalse;
		m_HoldWhileEmergency 		= tFalse;
	}
	
	m_DMActive 						= tFalse;
	m_LFActive 						= tFalse;
	m_CrossParkActive				= tFalse;
	m_ParallelParkActive  			= tFalse;	
	m_TurningActive 				= tFalse;
	m_ZebraActive					= tFalse;
	m_ACCActive 					= tFalse;
	m_EmergencyBreak 				= tFalse;		

	m_currentlyOvertaking 			= tFalse;	
	m_currentlyPullOut 				= tFalse;	

	m_checkCircleDistance 			= tFalse;	
	m_circleDistance 				= 0.0;	
	m_checkStraightCircleDistance 	= tFalse;
	m_StraightCircleDistance 		= 0.0;				

	m_CurrentManeuvre 				= ACTION_UNKNOWN;

	m_currentSpeed 					= 0.0;
	m_currentSteering 				= 90.0;

	m_ParkingMode 					= 0;

	m_CurrentCrossingSign			= UNDEFINED_SIGN;
	m_CurrentlyCrossing				= tFalse;

	m_processingRoadSign 			= tFalse;

	m_currentlyCircle 				= tFalse;
	m_withinCircle 					= tFalse;

	// Overtaking-Allowed flags
	m_OvertakingAlowed_RoadSign 	= tTrue;
	m_OvertakingAlowed_Maneuvre		= tTrue;
	m_OvertakingAlowed_Pedestrain 	= tTrue;

	m_overallDistance	 			= 0.0;
	m_referenceDistance				= 0.0;
	m_checkDistance 				= tFalse;

	m_Counter 						= 0;

	m_lastDistanceCheckpoint		= 0.0;
	m_checkLastDistance 			= tFalse;

	RETURN_NOERROR;
}
