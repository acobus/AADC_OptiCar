/*
 * Date 17.03.2016
 */
/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#ifndef _CALC_AREA_FILTER_H_
#define _CALC_AREA_FILTER_H_

#define OID_ADTF_CALC_AREA_FILTER "adtf.aadc.aadc_calcAreaFilter"

#include "stdafx.h"

/*!
This filter calculates the speed of the vehicle and some distance measurements. It takes the wheel structs containing the direction and the interrupt counter of both wheels from the Arduino sensor as input values. The wheels have a small disk mounted at the inner side and a Transmissive  Encoder Sensor which detects the slots in the rotating disk. The detected slots are transmitted to a interrupt routine at a Arduino which counts the ticks and sends them periodically to ADTF.
The speed is calculated by the difference between the increasing tick counter in the incoming media samples and the corresponding time  difference between the media samples. For a correct calculation the wheel circumference has to be set to the correct value in the properties, the default value is 0.34m.
The output samples are triggered to the wheel right struct, so if no samples are received from that sensor no output samples are generated by this filter.
If the property Filtering enabled is set to true a first order filtering is applied to smooth the signals. The first order filter constant can also be set in the properties.

*/
class CalcAreaFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CALC_AREA_FILTER, "CalcAreaFilter", OBJCAT_DataFilter, "CalcAreaFilter", 1, 0, 0, "OptiCar");
           


public:
    CalcAreaFilter(const tChar* __info);
    virtual ~CalcAreaFilter();

protected: 

   	cInputPin m_iStart;
	cInputPin m_iShowResult;
	cInputPin m_iInputDistanceOverall;

    cOutputPin m_oOutputSpeed;
	cOutputPin m_oStartTickFilter;

    tResult Init(tInitStage eStage, __exception = NULL);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:


	cObjectPtr<IMediaTypeDescription> m_pDescriptionStart;
    tBufferID m_szIDBoolValueStart;
	tBufferID m_szIDArduinoTimestampStart;
    tBool m_bIDsSetBoolValueStart;

	cObjectPtr<IMediaTypeDescription> m_pDescriptionShowResult;

	cObjectPtr<IMediaTypeDescription> m_pInputDistanceOverall; 
    tBufferID m_szIDDistanceF32Value; 
    tBufferID m_szIDDistanceArduinoTimestamp;         
    tBool m_bIDsDistanceSet;
      
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSpeed; 
    tBufferID m_szIDMinVelocityOutput; 
    tBufferID m_szIDMaxVelocityOutput;     
    tBool m_bIDsSpeedSet;

	cObjectPtr<IMediaTypeDescription> m_pStartTickFilter;
	tBufferID m_szIDBoolValueStartTickFilter; 
	tBufferID m_szIDTimestampStartTickFilter;
	tBool m_bStartStartTickFilterSet;


	tBool m_active;

	tBool m_showResultActive;

	tResult calcArea();
	tResult showResult(tBool aux);
	tResult ProcessSpeed(IMediaSample* pMediaSample);
	tResult TransmitControl(tFloat32 fSpeed);
	tResult TransmitBoolValue(tBool state);
	tResult InitialState();

	tFloat32 m_area;
	tFloat32 m_distanceOffset;
	tFloat32 m_drivenDistance;

	vector<double> m_dist_Left, m_dist_Right;

};




//*************************************************************************************************

#endif
