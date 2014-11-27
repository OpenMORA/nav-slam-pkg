/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


/**  @moos_module Real-time accurate pose tracking by fusing different localization sources (particle filters, odometry, etc...)
  */

#include "CLocalizationFusionApp.h"
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/system/threads.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;

CLocalizationFusionApp::CLocalizationFusionApp() :
	m_last_robot_cov_valid(false)
{
}

CLocalizationFusionApp::~CLocalizationFusionApp()
{
}


bool CLocalizationFusionApp::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{

		m_robot_pose.reset();

		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CLocalizationFusionApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CLocalizationFusionApp::Iterate()
{
	try
	{
		
		// Publish localization:
		PublishCurrentBestLocalization();
		return true;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CLocalizationFusionApp::PublishCurrentBestLocalization()
{
	float 	v,w;
	TPose2D	pose;


	if (!m_robot_pose.getCurrentEstimate(pose,v,w,mrpt::system::now()))
	{
		cout << endl << "It doesn't estimate the pose because data is not updated";
		return false;
	}

	const CPose2D	poseMean = CPose2D(pose);

	//! @moos_publish	LOCALIZATION   The robot estimated pose in format "[x y phi]"
	string sPose;
	poseMean.asString(sPose);
    m_Comms.Notify("LOCALIZATION", sPose );

	//! @moos_publish	LOCALIZATION_COV  The robot estimated pose uncertainty, as a 3x3 covariance matrix for [x y yaw]
	if (m_last_robot_cov_valid)
	{
		string sPoseCov = m_last_robot_cov.inMatlabFormat();
		m_Comms.Notify("LOCALIZATION_COV", sPoseCov );
	}

	// Get Localization as Obs (Serializable Object):
		mrpt::slam::CObservationOdometryPtr locObs = mrpt::slam::CObservationOdometry::Create();
		locObs->sensorLabel = "LOCALIZATION";
		locObs->odometry = poseMean;
		locObs->timestamp = now();
		locObs->hasVelocities = false;
		locObs->velocityLin = 0.0;
		locObs->velocityAng = 0.0;
		locObs->hasEncodersInfo = false;
		locObs->encoderLeftTicks = 0;
		locObs->encoderRightTicks = 0;		

		mrpt::vector_byte vec_loc;
		mrpt::utils::ObjectToOctetVector(locObs.pointer(), vec_loc);
		//! @moos_publish	LOCALIZATION_OBS  The robot estimated pose uncertainty as mrpt::slam::CObservationOdometry
		m_Comms.Notify("LOCALIZATION_OBS", vec_loc);


	cout << endl << "The pose is estimated and the iteration takes " << GetTimeSinceIterate() << " seconds";

    return true;
}



bool CLocalizationFusionApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CLocalizationFusionApp::DoRegistrations()
{
	
	//! @moos_subscribe ODO_REFERENCE
	m_Comms.Register("ODO_REFERENCE", 0 );
	
	//! @moos_subscribe ODOMETRY
	m_Comms.Register("ODOMETRY_OBS", 0 );

	//! @moos_subscribe LOCALIZATION_PF, LOCALIZATION_COV_PF
	m_Comms.Register("LOCALIZATION_PF", 0 );
	m_Comms.Register("LOCALIZATION_COV_PF", 0 );

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    RegisterMOOSVariables();

    return true;
}


bool CLocalizationFusionApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
    UpdateMOOSVariables(NewMail);

	for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if (MOOSStrCmp(i->GetName(),"ODO_REFERENCE"))
		{
			m_odo_reference.fromString( i->GetString() );
		}
	}

    for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
    	try
    	{
			if (MOOSStrCmp(i->GetName(),"LOCALIZATION_PF"))
			{
				TPose2D curPose;
				curPose.fromString( i->GetString() );
				m_robot_pose.processUpdateNewPoseLocalization(curPose, m_last_robot_cov, m_odo_reference, mrpt::system::now());
			}
			else
			if (MOOSStrCmp(i->GetName(),"LOCALIZATION_COV_PF"))
			{
				m_last_robot_cov_valid = true;
				m_last_robot_cov.fromMatlabStringFormat( i->GetString());
			}
			else
			if (MOOSStrCmp(i->GetName(),"ODOMETRY_OBS"))
			{
				/*
				CPose2D  cur_odo;
				cur_odo.fromString(i->GetString());
				//float v = 0, w=0;
				mrpt::system::TTimeStamp time_now = mrpt::system::getCurrentTime();
				//debug jgmonroy
				cerr << "Calling UpdateODOMETRY with Timestamp =" << time_now << "\n";
				m_robot_pose.processUpdateNewOdometry(cur_odo, time_now );
				*/
				CSerializablePtr obj;
				//StringToObject(i->GetString(),obj);
				mrpt::utils::RawStringToObject(i->GetString(),obj);

				if (obj && IS_CLASS(obj,CObservationOdometry))
				{
					CObservationOdometryPtr cur_odo_ptr = CObservationOdometryPtr( obj );
					CPose2D cur_odo = cur_odo_ptr->odometry;
					mrpt::system::TTimeStamp time_now = cur_odo_ptr->timestamp;
					//cerr << "Processing ODOMETRY: " << cur_odo.asString() << endl;
					m_robot_pose.processUpdateNewOdometry(cur_odo,time_now);
				}
				else
				{
					cerr << "ODOMETRY is not CObservationOdometry" << endl;
				}

			}
			else
			if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
			{
				// Disconnect comms:
				MOOSTrace("Closing Module \n");
				this->RequestQuit();
			}
    	}
    	catch (std::exception &e)
    	{
    		cerr << "**ERROR** processing mail: " << i->GetName() << endl << e.what() << endl;
    	}
		//system::sleep(2);
	}

    return true;
}
