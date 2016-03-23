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


#ifndef _testStopFilter
#define _testStopFilter

#define __guid "adtf.aadc.aadc_testStopFilter"

#include "stdafx.h"
#include "displaywidget.h"


/*!
With this small helper simple Media Samples of the tBoolSignalValue can be generated. When started a GUI with two buttons is shown. When clicking on �Send Value FALSE� a Media Sample with �False� in the media description element bValue is transmitted, when clicking on �Send Value TRUE� a Media Sample with �True� is transmitted.
*/
class testStopFilter : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "Test Stop Filter", OBJCAT_Auxiliary, "Test Stop Filter", 1, 0, 0, "BFFT GmbH");    

    Q_OBJECT
    
signals:


public: // construction
    testStopFilter(const tChar *);
    virtual ~testStopFilter();

    /*! overrides cFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Start(__exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);
   

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);


protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();
    

private:

    /*! The displayed widget*/    
    DisplayWidget *m_pWidget;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionStopStruct;
	tBufferID m_szIDVelocityStructOutput; 
	tBufferID m_szIDDistanceStructOutput; 
	tBufferID m_szIDBoolStructOutput;
    /*! indicates if bufferIDs were set */
    tBool m_bIDStopStructOutput;


	// StateTurningOutput
	cObjectPtr<IMediaTypeDescription> m_pStateTurningOutput;
	tBufferID m_szIDTurningStructModeOutput;
	tBufferID m_szIDTurningStructTypeOutput;
	tBufferID m_szIDTurningStructGiveWayOutput;
	tBufferID m_szIDTurningStructStopSignOutput;
	tBufferID m_szIDTurningStructDistanceOutput;
	tBufferID m_szIDTurningStructVelocityOutput;
	tBufferID m_szIDTurningStructStatusOutput;
	tBool m_bTurningOutputSet;

	tInt32 m_intOccasion;
	tInt32 	m_mode;
	tInt32 	m_type;
	tInt32 	m_giveWay;
	tBool 	m_stopSign;
    
    /*! output pin for the run command*/
    cOutputPin      m_oStopStructOutputPin; // for parking

	cOutputPin 		m_oStateTurning; // for crossing

    
public slots:
    /*! transmits a new mediasample with value false */
    void OnTransmitValueFalse();
    
    /*! transmits a new mediasample with value true */
    void OnTransmitValueTrue();
	tResult PropertyChanged(const tChar* strName);



};

#endif
