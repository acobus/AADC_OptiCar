/*
 * Date 01.02.2016
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


#ifndef _WHEELSPEEDCONTROLLER_FILTER_H_
#define _WHEELSPEEDCONTROLLER_FILTER_H_

#include "stdafx.h"

#define OID_ADTF_WHEELSPEEDCONTROLLER_FILTER "adtf.aadc.wheelSpeedController"

/*!
This filter implements a controller to set the wheelspeed of the vehicle with a P/PI/PID or PT1 algorithm. The input pin measured_wheelSpeed  has to be connected to the output pin of the Converter Wheels Filter and the desired wheel speed has to be set to the input pin set_WheelSpeed. 
The controller parameters have to be adapted to each individual car. The default values for the PT1 controller are good start for the controller but maybe have to be adapted to the individual team car. There are different methods to get the correct parameter, please refer to other literature.
The output pin actuator_output have to be connected to a Calibration XML which maps the speed in m/〖sec〗^2  to servo angle of steering controller. In experiments the following mapping was found:

This values are saved in the Sample XML SpeedController.xml in the folder configuration_files and can be used at the beginning.
A typical configuration with the Wheel Speed Controller should contain at least the following Filters:
 
*/
class WheelSpeedController : public adtf::cFilter, 
    public adtf::ISignalProvider
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_WHEELSPEEDCONTROLLER_FILTER, "Wheel Speed Controller", OBJCAT_DataFilter, "Wheel Speed Controller", 1, 1,0, "BFFT GmbH");    

    cInputPin m_InputMeasWheelSpeed;                // the input pin for the measured value
    cInputPin m_InputSetWheelSpeed;                // the input pin for the set point value
	cInputPin m_InputActivateFilter;                // the input pin for activating the filter ( true=filter on, false=filter off)
    cOutputPin m_OutputActuator;        // the output pin for the manipulated value

public:
    WheelSpeedController(const tChar* __info);
    virtual ~WheelSpeedController();

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);        
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);    

private:
    /*! creates all the input Pins
    @param __exception pointer for exception
    */
    tResult CreateInputPins(__exception = NULL);

    /*! creates all the output Pins
    @param __exception pointer for exception
    */
    tResult CreateOutputPins(__exception = NULL);    

    /* called if one of the properties is changed 
    @param strProperty the changed property
    */
    tResult PropertyChanged(const char* strProperty);

    /* reads the properties and save to member variables 
    @param strPropertyName name of the property changed
    */
    tResult ReadProperties(const tChar* strPropertyName);

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
    @param measuredValue    the measuredValue
    */
    tFloat64 getControllerValue(tFloat64 i_f64MeasuredValue);

    /*! returns the currentstreamtime */
    tTimeStamp GetTime();

    /*! holds the last measuredValue */
    tFloat64 m_f64MeasuredVariable;
    /*! holds the last measured error */
    tFloat64 m_f64LastMeasuredError;
    /*! holds the last setpoint */
    tFloat64 m_f64SetPoint ;
    /*! holds the last output */ 
    tFloat64 m_f64LastOutput;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller */
    tFloat64 m_f64accumulatedVariable;

    /*! media description for the input pin set speed */
    cObjectPtr<IMediaTypeDescription> m_pDescSetSpeed;        
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDSetSpeedArduinoTimestamp;         
    /*! indicates of bufferIDs were set */
    tBool m_bInputSetWheelSpeedGetID;


    /*! media description for the input pin measured speed */
    cObjectPtr<IMediaTypeDescription> m_pDescMeasSpeed;
    /*! the id for the f32value of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin for the measured speed */
    tBufferID m_buIDMeasSpeedArduinoTimestamp; 
    /*! indicates of bufferIDs were set */
    tBool m_bInputMeasWheelSpeedGetID;

    /*! media description for the input pin activate filter */
    cObjectPtr<IMediaTypeDescription> m_pDescActivateFilter;
    /*! the id for the bvalue of the media description for input pin for activating filter */
    tBufferID m_buIDActivateFilterBool; 
    /*! the id for the arduino time stamp of the media description for input pin for activating filter  */
    tBufferID m_buIDActivateFilterArduinoTimestamp; 
    /*! indicates of bufferIDs were set */
    tBool m_bInputActivateFilterGetID;

    /*! the critical section for the on pin events */
    cCriticalSection m_critSecOnPinEvent;

    /*! media description for the output pin with speed */                
    cObjectPtr<IMediaTypeDescription> m_pDescActuator;       
    /*! the id for the f32value of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorF32Value; 
    /*! the id for the arduino time stamp of the media description for input pin for the set speed */
    tBufferID m_buIDActuatorArduinoTimestamp;         
    /*! indicates of bufferIDs were set */
    tBool m_bInputActuatorGetID;

    // PID-Controller values
    // proportional factor for PID Controller
    tFloat64    m_f64PIDKp;
    // integral factor for PID Controller
    tFloat64    m_f64PIDKi;
    // differential factor for PID Controller
    tFloat64    m_f64PIDKd;
    // the sampletime for the pid controller
    tFloat64 m_f64PIDSampleTime;
    // the minimum output value for the controller
    tFloat64 m_f64PIDMinimumOutput;    
    // the maximum output value for the controller
    tFloat64 m_f64PIDMaximumOutput ;

    // PT1-Controller values
    // tau value for PT1 controller
    tFloat64    m_f64PT1Tau;
    // sample time for PT1 Controller
    tFloat64    m_f64PT1Sampletime;
    // gain factor for PT1 controller
    tFloat64    m_f64PT1Gain;
    // time constant for pt1 controller
    tFloat64 m_f64PT1TimeConstant;
    //input factor for PT1
    tFloat64 m_f64PT1OutputFactor;
    //input factor for PT1
    tFloat64 m_f64PT1InputErrorFactor;

    // the set point is multiplied with this factor, otherwise the set point is not reached by the controller.
    tFloat64 m_f64PT1CorrectionFactor;

    // holds the last speed value
    tFloat64 m_f64LastSpeedValue;

    //defines whether PID or PT1 is used
    tInt32 m_i32ControllerMode;

	// active
	tBool m_active;

	//dcllesdfk
	tFloat32 faux;


public: // implements ISignalProvider
    virtual tResult GetSignalValue(tSignalID nSignalID, tSignalValue* pValue);
    virtual tResult ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate = 0);
    virtual tResult DeactivateSignalEvents(tSignalID nSignalID);
    /*! this functions sends the signal data to the registry*/ 
    tResult SendSignalData();



public: // implements IObject
    tResult GetInterface(const tChar* idInterface, tVoid** ppvObject);
    tUInt Ref();
    tUInt Unref();
    tVoid Destroy();

    // members for signal registry
    typedef std::set<tSignalID> tActiveSignals;

    ucom::cObjectPtr<ISignalRegistryExtended> m_pISignalRegistry;
    cKernelMutex                              m_oLock;
    tActiveSignals                            m_oActive;

    // indicates if the internal signals are also plotted to registry
    tBool       m_bShowDebug;
};

#endif // _cWheelSpeedController_H_

