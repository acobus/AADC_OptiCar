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
#include "stdafx.h"
#include "displaywidget.h"

 

DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent)
{ 
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_btSendValueTrue = new QPushButton(this);
    m_btSendValueTrue->setText("Send next left maneuver");
    m_btSendValueTrue->setFixedSize(200,50);

    m_btSendValueFalse = new QPushButton(this);
    m_btSendValueFalse->setText("Send next right maneuver");
    m_btSendValueFalse->setFixedSize(200,50);

    m_tSendStraight = new QPushButton(this);
    m_tSendStraight->setText("Send next straight maneuver");
    m_tSendStraight->setFixedSize(200,50);

    m_tSendValueCircle = new QPushButton(this);
    m_tSendValueCircle->setText("Send next traffic circle maneuver");
    m_tSendValueCircle->setFixedSize(200,50);

    m_mainLayout = new QVBoxLayout();
	m_mainLayout->addWidget(m_tSendStraight,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueFalse,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueTrue, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_tSendValueCircle, 0,Qt::AlignCenter);
    setLayout(m_mainLayout);

	connect(m_tSendStraight,  SIGNAL(clicked()), this, SLOT(sendStraight()));
    connect(m_btSendValueFalse,  SIGNAL(clicked()), this, SLOT(sendValueFalse()));
    connect(m_btSendValueTrue,  SIGNAL(clicked()), this, SLOT(sendValueTrue()));
    connect(m_tSendValueCircle,  SIGNAL(clicked()), this, SLOT(sendCircle()));
}
