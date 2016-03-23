/**
*
*TicTacTactic
*
* Date 4.02.2016
*
*/
 
 
#include "stdafx.h"
#include "TicTacTactic.h"
#include "/home/aadc/AADC/src/aadcUser/include/intrinsic_data.h"
#include "/home/aadc/Desktop/AADC Source/src/aadcUser/src/Util/Util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <fstream>
#include <cmath>


ADTF_FILTER_PLUGIN("TicTacTactic", OID_ADTF_TicTacTactic, cTicTacTactic)



//TODO 1
//#define LT_ENABLE_CANNY_WINDOWS


cTicTacTactic::cTicTacTactic(const tChar* __info) : cFilter(__info)
{
    


    m_pISignalRegistry = NULL;
}

cTicTacTactic::~cTicTacTactic()
{
}

tResult cTicTacTactic::GetInterface(const tChar* idInterface,
    tVoid** ppvObject)
{
    if (idmatch(idInterface, IID_ADTF_SIGNAL_PROVIDER))
    {
        *ppvObject = static_cast<ISignalProvider*> (this);
    }
    else
    {
        return cFilter::GetInterface(idInterface, ppvObject);
    }

    Ref();

    RETURN_NOERROR;
}

tUInt cTicTacTactic::Ref()
{
    return cFilter::Ref();
}

tUInt cTicTacTactic::Unref()
{
    return cFilter::Unref();
}

tVoid cTicTacTactic::Destroy()
{
    delete this;
}

tResult cTicTacTactic::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cTicTacTactic::Stop(__exception)
{
   

    return cFilter::Stop(__exception_ptr);
}
tResult cTicTacTactic::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		// Get description for bool values
    	tChar const * strDescValue = pDescManager->GetMediaDescription("tSignalValue");	
    	RETURN_IF_POINTER_NULL(strDescValue);	
    	cObjectPtr<IMediaType> pTypeValue = new cMediaType(0, 0, 0, "tSignalValue", strDescValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// 1 (Input)
    	RETURN_IF_FAILED(m_i1.Create("1", pTypeValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_i1));
		RETURN_IF_FAILED(pTypeValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_p1Input)); 

		// 2 (Input)
    	RETURN_IF_FAILED(m_i2.Create("2", pTypeValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_i2));
		RETURN_IF_FAILED(pTypeValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_p2Input)); 

		// 3 (Input)
    	RETURN_IF_FAILED(m_i3.Create("3", pTypeValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_i3));
		RETURN_IF_FAILED(pTypeValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_p3Input)); 

		// 4 (Input)
    	RETURN_IF_FAILED(m_i4.Create("4", pTypeValue, static_cast<IPinEventSink*> (this)));
    	RETURN_IF_FAILED(RegisterPin(&m_i4));
		RETURN_IF_FAILED(pTypeValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_p4Input)); 



              
    }
    else if (eStage == StageNormal)
    {

        

    }
    RETURN_NOERROR;
}

tResult cTicTacTactic::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult cTicTacTactic::ReadProperties(const tChar* strPropertyName)
{
    RETURN_NOERROR;
}

tResult cTicTacTactic::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr)
{

    RETURN_NOERROR;
}

tResult cTicTacTactic::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{

    
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cTicTacTactic::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

        if(pSource == &m_i1)
        {
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_p1Input,pMediaSample,pCoder);

				if (!m_b1set) {
					pCoder->GetID("f32Value", m_sz1);
					m_b1set = tTrue;
				}

                pCoder->Get(m_sz1, (tVoid*)&m_1);	
            }
        }

        else if(pSource == &m_i2)
        {
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_p2Input,pMediaSample,pCoder);

				if (!m_b2set) {
					pCoder->GetID("f32Value", m_sz2);
					m_b2set = tTrue;
				}

                pCoder->Get(m_sz2, (tVoid*)&m_2);	
            }
        }

        else if(pSource == &m_i3)
        {
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_p3Input,pMediaSample,pCoder);

				if (!m_b3set) {
					pCoder->GetID("f32Value", m_sz3);
					m_b3set = tTrue;
				}

                pCoder->Get(m_sz3, (tVoid*)&m_3);	
            }
        }

        else if(pSource == &m_i4)
        {
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_p4Input,pMediaSample,pCoder);

				if (!m_b4set) {
					pCoder->GetID("f32Value", m_sz4);
					m_b4set = tTrue;
				}

                pCoder->Get(m_sz4, (tVoid*)&m_4);	
            }
        }
              
        
        RETURN_NOERROR;
    }
    
    RETURN_NOERROR;
}


tResult cTicTacTactic::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cTicTacTactic::ProcessOutput()
{
    RETURN_NOERROR;
}



tResult cTicTacTactic::GetSignalValue(tSignalID nSignalID, tSignalValue * pValue)
{
  
    RETURN_NOERROR;
}

/**
 *   Activates a signal.
 *   Activated signals send their values to the Signal Registry Service.
 */
tResult cTicTacTactic::ActivateSignalEvents(tSignalID nSignalID, tTimeStamp nUpdateRate)
{     
    RETURN_NOERROR;
}

/**
 *   Deactivates a signal.
 */
tResult cTicTacTactic::DeactivateSignalEvents(tSignalID nSignalID)
{
    RETURN_NOERROR;
}

