/*
 * Date 17.12.2015
 */

#include "stdafx.h"
#include "RoadSignPreprocessing.h"
#include "../../include/aadc_roadSign_enums.h"

#include <fstream>

// TODO: Find out good value for this!
#define ROAD_SIGN_MIN_SIZE 450


/// Create filter shell
ADTF_FILTER_PLUGIN("Roadsign Preprocessing Filter", OID_ADTF_ROAD_SIGN_PREPROCESSING, cRoadSignPreprocessing);


cRoadSignPreprocessing::cRoadSignPreprocessing(const tChar* __info):cFilter(__info)
{
	// no working state at beginning
	working 	= tFalse;

	last_time 	= 0;
	
	// Initialize sizeList with zero
	for ( int i = 0; i < NUM_ROADSIGNS; i++ ){
		sizeList[i] 	= 0;
	}

}

cRoadSignPreprocessing::~cRoadSignPreprocessing()
{

}

tResult cRoadSignPreprocessing::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
			cObjectPtr<IMediaDescriptionManager> pDescManager;
    							RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
    							IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
    							(tVoid**)&pDescManager,
    							__exception_ptr));

    		// Get description for RoadSign
    		tChar const * strDescRoadSign = pDescManager->GetMediaDescription("tRoadSign");   
    		RETURN_IF_POINTER_NULL(strDescRoadSign);    
    		cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strDescRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    		// Get description for RoadSign_ext
    		tChar const * strDescRoadSignExt = pDescManager->GetMediaDescription("tRoadSignExtended");   
    		RETURN_IF_POINTER_NULL(strDescRoadSignExt);    
    		cObjectPtr<IMediaType> pTypeRoadSignExt = new cMediaType(0, 0, 0, "tRoadSignExtended", strDescRoadSignExt, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

			// Found RoadSigns (Input)
    		RETURN_IF_FAILED(m_iRoadSign.Create("RoadSign", pTypeRoadSignExt, this));
    		RETURN_IF_FAILED(RegisterPin(&m_iRoadSign));
    		RETURN_IF_FAILED(pTypeRoadSignExt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pRoadSignInput));

			// Filtered RoadSigns (Output)
    		RETURN_IF_FAILED(m_oRoadSignFiltered.Create("RoadSignsFiltered", pTypeRoadSign, this));
    		RETURN_IF_FAILED(RegisterPin(&m_oRoadSignFiltered));
    		RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pRoadSignOutput));

    }
    else if (eStage == StageNormal)
    {

    }
    else if (eStage == StageGraphReady)
    {
		m_bSignInputSet 	= tFalse;
		m_bSignOutputSet	= tFalse;
    }

    RETURN_NOERROR;
}

tResult cRoadSignPreprocessing::Shutdown(tInitStage eStage, __exception)
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

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cRoadSignPreprocessing::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

		if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
		{

			if (pSource == &m_iRoadSign)
			{
				addToBuffer(pMediaSample);
				calcRoadSign();
			}
        }
    }

    RETURN_NOERROR;
}

/*
 * Method to save the incoming RoadSigns
 */
tResult cRoadSignPreprocessing::addToBuffer(IMediaSample* pMediaSample){

	// Check last foundings
	/*time_t act_time;
	time(&act_time);

	// Reference time year 2000
	static struct tm y2k;


	y2k.tm_hour = 0;
	y2k.tm_min  = 0;
	y2k.tm_sec  = 0;
	y2k.tm_year = 100;
	y2k.tm_mon  = 0;
	y2k.tm_mday = 1;

	double act_time_d 	= difftime(act_time,mktime(&y2k));*/

	tUInt32 act_time_d = _clock->GetStreamTime();

	// Delete buffer if neccessary
	if ( (act_time_d - last_time)/1000000.0 > 0.5 ){
		m_roadSignBuffer.clear();
	}				

	tInt16 		identifier 	= -1;
	tFloat32	size		=  0;
	float 		Tvec[3];
	float		Rvec[3];

	{
		__adtf_sample_read_lock_mediadescription(m_pRoadSignInput ,pMediaSample, pCoder);    

		if (!m_bSignInputSet){
			pCoder->GetID("i16Identifier", m_szIDSignIdentifierInput);
			pCoder->GetID("f32Imagesize", m_szIDSignSizeInput);
			pCoder->GetID("RVec1", m_szIDSignRotation1Input);
			pCoder->GetID("RVec2", m_szIDSignRotation2Input);
			pCoder->GetID("RVec3", m_szIDSignRotation3Input);
			pCoder->GetID("TVec1", m_szIDSignTranslation1Input);		
			pCoder->GetID("TVec2", m_szIDSignTranslation2Input);		
			pCoder->GetID("TVec3", m_szIDSignTranslation3Input);		
			m_bSignInputSet 	= tTrue;
		}

		pCoder->Get(m_szIDSignIdentifierInput, (tVoid*)&identifier);  
		pCoder->Get(m_szIDSignSizeInput, (tVoid*)&size);   
		pCoder->Get(m_szIDSignTranslation1Input, (tVoid*)&Tvec[0]);
		pCoder->Get(m_szIDSignTranslation2Input, (tVoid*)&Tvec[1]);
		pCoder->Get(m_szIDSignTranslation3Input, (tVoid*)&Tvec[2]);
		pCoder->Get(m_szIDSignRotation1Input, (tVoid*)&Rvec[0]);
		pCoder->Get(m_szIDSignRotation2Input, (tVoid*)&Rvec[1]);
		pCoder->Get(m_szIDSignRotation3Input, (tVoid*)&Rvec[2]);
    
	}

	/*
	fstream fss;
	fss.open("RoadSignVectors.dat",ios::out);
	fss << "size: " << size << "\tidentifier:" << identifier << "\n";
	fss << "Tvec: [" << Tvec[0] << "," << Tvec[1]<< "," << Tvec[2] << "]\n\n";
	fss << "Rvec: [" << Rvec[0] << "," << Rvec[1]<< "," << Rvec[2] << "]\n\n";
	fss.close();*/
 
/*
	LOG_INFO(cString::Format("RPP: Tvec: [%f,%f,%f]",Tvec[0],Tvec[1],Tvec[2]) );
	LOG_INFO(cString::Format("RPP: Rvec: [%f,%f,%f]",Rvec[0],Rvec[1],Rvec[2]) ); 
	LOG_INFO(cString::Format("RPP: Size: %f", size ) );
*/

	/*fstream f;
	f.open("FoundRoadSigns", ios::out | ios::app);
	f << (act_time_d - last_time)/1000000.0 << "\t" << identifier << "\t" << Rvec[0] << "\t" << Rvec[1] << "\t" << size << "\n";
	f.close();*/

	// |Rvec[0]|>2 && |Rvec[1]|>2 -> So roadsign looks straight to us
	// It also has to be close enough
	//if ( (Rvec[0]>1.9 || Rvec[0]<-1.9) && (Rvec[1]>1.9 || Rvec[1]<-1.9) && size > ROAD_SIGN_MIN_SIZE ){
	if ( fabs(Rvec[0]) > 1.88 /*&& fabs(Rvec[0]) < 2.4*/ && fabs(Rvec[1]) > 1.88 /*&& fabs(Rvec[1]) < 2.4*/ && size > ROAD_SIGN_MIN_SIZE ){

		m_roadSignBuffer.push_back(identifier);
		sizeList[getListIndexOfSign(identifier)] = size;
		last_time  	= act_time_d;

	}

	
	RETURN_NOERROR;
}

/*
 * Method to filter the incoming RoadSigns
 */
tResult cRoadSignPreprocessing::calcRoadSign(){
	// No parallel access
	if ( !working && m_roadSignBuffer.size() > RANGE_DATASET ){

		working = tTrue;

		tInt16 data;
    
    	short int num_list[NUM_ROADSIGNS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        for(int i = 0; i < RANGE_DATASET; i++)
        {
            data = m_roadSignBuffer.front();
            //m_roadSignBuffer.pop_front();

            num_list[getListIndexOfSign(data)]++;
            
        }
		m_roadSignBuffer.pop_front();

        tInt16 tmp = 0; 
		tInt16 num_id = 0; 
        //get the max value
        for (tInt16 i = 0; i < NUM_ROADSIGNS; i++)
        {
           	 if(num_list[i] > tmp)
           	 {
             	 tmp = num_list[i];
             	 num_id = i;
            }
        }
		
		// Only use if we found enough signs
		if (tmp >= RANGE_DATASET-1) {
			tInt16 sign_id	= getSignByIndex(num_id);

        	transmitRoadSign(sign_id);
		}

		working = tFalse;	
	}	

	RETURN_NOERROR;
}

/*
 * Read from enum
 */
tInt16 cRoadSignPreprocessing::getListIndexOfSign(tInt16 signId)
{
    switch(signId)
            {
            case MARKER_ID_NOMATCH:
                return 0;
                break;
            case MARKER_ID_GIVEWAY:
                return 1;
                break;
            case MARKER_ID_HAVEWAY:
                return 2;
                break;
            case MARKER_ID_STOPANDGIVEWAY:
                return 3;
                break;
            case MARKER_ID_PARKINGAREA:
                return 4;
                break;
            case MARKER_ID_AHEADONLY:
                return 5;
                break;
            case MARKER_ID_UNMARKEDINTERSECTION:
                return 6;
                break;
            case MARKER_ID_PEDESTRIANCROSSING:
                return 7;
                break;
            case MARKER_ID_ROUNDABOUT:
                return 8;
                break;
            case MARKER_ID_NOOVERTAKING:
                return 9;
                break;
            case MARKER_ID_NOENTRYVEHICULARTRAFFIC:
                return 10;
                break;
            case MARKER_ID_ONEWAYSTREET:
                return 11;
                break;
            default:
                return 0;
                break;
            }
}

/*
 * Get ID of Found RoadSign
 */
tInt16 cRoadSignPreprocessing::getSignByIndex(tInt16 index)
{
    switch(index)
            {
            case 0:
                return MARKER_ID_NOMATCH;
                break;
            case 1:
                return MARKER_ID_GIVEWAY;
                break;
            case 2:
                return MARKER_ID_HAVEWAY;
                break;
            case 3:
                return MARKER_ID_STOPANDGIVEWAY;
                break;
            case 4:
                return MARKER_ID_PARKINGAREA;
                break;
            case 5:
                return MARKER_ID_AHEADONLY;
                break;
            case 6:
                return MARKER_ID_UNMARKEDINTERSECTION;
                break;
            case 7:
                return MARKER_ID_PEDESTRIANCROSSING;
                break;
            case 8:
                return MARKER_ID_ROUNDABOUT;
                break;
            case 9:
                return MARKER_ID_NOOVERTAKING;
                break;
            case 10:
                return MARKER_ID_NOENTRYVEHICULARTRAFFIC;
                break;
            case 11:
                return MARKER_ID_ONEWAYSTREET;
                break;
            default:
                return MARKER_ID_NOMATCH;
                break;
            }
}



/*
 * This Method sends the filtered roadSign to the DM module
 */
tResult cRoadSignPreprocessing::transmitRoadSign(tInt16 sign_id){

	cObjectPtr<IMediaSample> pMediaSample;
	AllocMediaSample((tVoid**)&pMediaSample);

	//allocate memory with the size given by the descriptor
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pRoadSignOutput->GetMediaSampleSerializer(&pSerializer);
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	tInt16 identifier 	= sign_id;
	tFloat32 size 		= sizeList[getListIndexOfSign(identifier)];

	//LOG_INFO(cString::Format("RSPP: Found RoadSign %i", sign_id) );

	/*fstream f;
	f.open("FoundRoadSigns", ios::out | ios::app);
	f << "TRANSMIT:" << "\t" << sign_id << "\t" << sizeList[getListIndexOfSign(identifier)] << "\n";
	f.close();*/

	{
		__adtf_sample_write_lock_mediadescription(m_pRoadSignOutput, pMediaSample, pCoder);    

		// set the id
		if (!m_bSignOutputSet){
			pCoder->GetID("i16Identifier", m_szIDSignIdentifierOutput);
			pCoder->GetID("f32Imagesize", m_szIDSignSizeOutput);  
			m_bSignOutputSet 	= tTrue;
		}

		// set value from sample
		pCoder->Set(m_szIDSignIdentifierOutput, (tVoid*)&identifier);     
		pCoder->Set(m_szIDSignSizeOutput, (tVoid*)&size);    
	}

	pMediaSample->SetTime(_clock->GetStreamTime());

	m_oRoadSignFiltered.Transmit(pMediaSample);

	RETURN_NOERROR;
}
