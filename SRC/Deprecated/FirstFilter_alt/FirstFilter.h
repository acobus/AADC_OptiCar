#ifndef _FIRST_FILTER_H_
#define _FIRST_FILTER_H_

#define OID_ADTF_FIRST_FILTER "adtf.example.first_filter"


//*************************************************************************************************
class FirstFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_FIRST_FILTER, "First Filter", adtf::OBJCAT_DataFilter);

public:
    FirstFilter(const tChar* __info);
    virtual ~FirstFilter();

    cInputPin m_oUSFrontInput;
    cInputPin m_oUSRearInput;
    cInputPin m_oSpeedController;
    cInputPin m_oCarSpeedInput;
    cOutputPin m_oSpeedOut;
    cOutputPin m_oBrakeLightOutputPin;  

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
private:
    struct tProperties
    {
        tFloat32 fMinDistance;

        tProperties()
        {
	    	fMinDistance = 0.5f;
		}
    } m_sProperties;

    bool speedZeroFront, speedZeroRear;
    bool driveForward;
    bool breakingModeFront;
    bool breakingModeRear;
    bool breakinglight;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
    cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;

    /*! the id for the bool value output of the media description */
    tBufferID m_szIDBoolValueOutput;

    /*! indicates if bufferIDs were set */
    tBool m_bIDsBoolValueOutput;

    /*! the id for the arduino timestamp output of the media description */
    tBufferID m_szIDArduinoTimestampOutput;

    /*! descriptor for actuator values data */        
    cObjectPtr<IMediaTypeDescription> m_pDescriptionActuatorOutput;
    
    tResult ProcessUSFrontInput(IMediaSample* pMediaSample);
    tResult ProcessUSRearInput(IMediaSample* pMediaSample);
    tResult ProcessCarSpeedInput(IMediaSample* pMediaSample);
    tResult TransmitSpeed(IMediaSample* pMediaSample);


    /*! helper function to toggle a bool signal via the specified pin
    @param the destination pin to toggle
    */
    tResult TransmitBoolValue(cOutputPin* pin, tBool value);
};



//*************************************************************************************************
#endif
