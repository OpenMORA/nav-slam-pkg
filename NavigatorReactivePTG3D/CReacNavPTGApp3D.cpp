/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Mariano Jaimez Tarifa  <marianojt@uma.es>                     |
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


#include "CReacNavPTGApp3D.h"
#include <sstream>
#include <iomanip>
#include <iostream>


using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::reactivenav;


//CReacNavPTGApp::CReacNavPTGApp() :
//{
//}

//CReacNavPTGApp::~CReacNavPTGApp()
//{
//}


bool CReacNavPTGApp3D::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{
		DoNavigatorReset();
		InitializeObstacleGrid();
		if (m_visualization)
			InitializeGridScene();

		DoRegistrations();

		m_reactive_clock.Tic();

		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CReacNavPTGApp3D::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pNavigatorReactivePTG3D only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	printf("COMMAND RECEIVED %s\n",sCmd.c_str());

	
	//!  @moos_cmd   CANCEL   Cancel the current navigation.
    if(MOOSStrCmp(sCmd,"CANCEL"))
    {
		m_navigator.m_navstate = CANCELLED;  
		this->SendSystemLogString("Navigation has been cancelled.");
	}
	//!  @moos_cmd   PAUSE  Pause the current navigation, which can be resumed later on.
    else if(MOOSStrCmp(sCmd,"PAUSE"))
    {
		m_navigator.m_navstate = PAUSED;
		this->SendSystemLogString("Navigation has been paused.");
	}
	//!  @moos_cmd   RESUME  Resume a paused navigation.
    else if(MOOSStrCmp(sCmd,"RESUME"))
    {
		m_navigator.m_navstate = ACTIVE;
		this->SendSystemLogString("Paused navigation has been resumed.");
	}

    return true;
}

bool CReacNavPTGApp3D::DoNavigatorReset()
{
	// Load parameters from moos config file:
	try
	{
		// Reload config:
		m_navigator.loadRobotConfiguration(m_ini);
		m_navigator.m_logFile = NULL;
		m_navigator.EnableLogFile(m_navigator.m_logFile);
		m_visualization = m_ini.read_bool("","Opengl_scene", 0, false);
		m_memory_on = m_ini.read_bool("","Memory_on", 1, false);
		vred_a = m_ini.read_float("","Vred_a", 1, false);
		vred_b = m_ini.read_float("","Vred_b", 0.2, false);
		m_minv = m_ini.read_float("","VMIN_MPS", 0.1, false);
		m_minw = DEG2RAD(m_ini.read_float("","WMIN_DEGPS", 10, false));
		m_maxv = m_ini.read_float("","VMAX_MPS", 0.6, false);
		m_maxw = DEG2RAD(m_ini.read_float("","WMAX_DEGPS", 60, false));
		m_log_decimation = m_ini.read_uint64_t("","LOG_DECIMATION", 1, false);
		m_iteration = 0;
		m_ird_warning = 0;

		//Speed initial values
		m_navigator.m_dynfeatures.new_cmd_v = 0;
		m_navigator.m_dynfeatures.new_cmd_w = 0;

		//Set the initial state as PAUSED
		m_navigator.m_navstate = PAUSED;
		
		//Create the collision grid
		m_navigator.PTGGridBuilder(m_ini);

		//Necesary variables to save information in the LogFile
		m_navigator.m_HLFRs.resize( m_navigator.m_ptgmultilevel.size() );
		m_navigator.m_newLogRec.infoPerPTG.resize( m_navigator.m_ptgmultilevel.size() );
		
		
		return true;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}



bool CReacNavPTGApp3D::Iterate()
{
	try
	{
		// Do we have a new navigation command??
		CMOOSVariable *pVarNav = GetMOOSVar( "NAVIGATE_TARGET" );
		if(pVarNav && pVarNav->IsFresh())
		{
			pVarNav->SetFresh(false);
			CMatrixDouble M;
			if (M.fromMatlabStringFormat(pVarNav->GetStringVal()) && size(M,1)==1 && size(M,2)>=2)
			{
				m_event_sent = 0;
				m_navigator.m_reactiveparam.WS_Target.x(M(0,0));
				m_navigator.m_reactiveparam.WS_Target.y(M(0,1));
				m_navigator.m_navstate = ACTIVE;

				//Center the Neck
				m_Comms.Notify("NECKMSG","SERVO=0,ANG=0,SPEED=60,FILTER=1");
				
				//! @moos_publish MORA_GLOBAL_LOG
				this->SendSystemLogString(format("Starting navigation to (%.02f,%.02f).",
								m_navigator.m_reactiveparam.WS_Target[0], m_navigator.m_reactiveparam.WS_Target[1]));
			}
			else MOOSTrace("ERROR: Invalid format of NAVIGATE_TARGET");
		}

		// It runs the the reactive algorithm.
		return DoReactiveNavigation();

	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CReacNavPTGApp3D::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CReacNavPTGApp3D::DoRegistrations()
{
	//! @moos_subscribe	ODOMETRY
    AddMOOSVariable("ODOMETRY", "ODOMETRY", "ODOMETRY", 0 );

	//! @moos_subscribe	LOCALIZATION, LOCALIZATION_COV
	AddMOOSVariable("LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0);
	AddMOOSVariable("LOCALIZATION_COV", "LOCALIZATION_COV", "LOCALIZATION_COV", 0);

	//! @moos_subscribe	LASER1, LASER2
	AddMOOSVariable("LASER1","LASER1","LASER1", 0);
	AddMOOSVariable("LASER2","LASER2","LASER2", 0);

	//! @moos_subscribe	KINECT
	AddMOOSVariable("KINECT1", "KINECT1", "KINECT1", 0);
	
	//! @moos_subscribe	NAVIGATE_TARGET
	AddMOOSVariable("NAVIGATE_TARGET", "NAVIGATE_TARGET", "NAVIGATE_TARGET", 0);

	//! @moos_subscribe	NAVIGATE_CANCEL
	AddMOOSVariable("NAVIGATE_CANCEL", "NAVIGATE_CANCEL", "NAVIGATE_CANCEL", 0);

	//! @moos_subscribe	CANCEL
	AddMOOSVariable("CANCEL", "CANCEL", "CANCEL", 0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe EST_GOODNESS
	AddMOOSVariable( "EST_GOODNESS", "EST_GOODNESS", "EST_GOODNESS", 0 );	

	//! @moos_subscribe IRD_WARNING
	AddMOOSVariable( "IRD_WARNING", "IRD_WARNING", "IRD_WARNING", 0 );	

    RegisterMOOSVariables();
    return true;
}


bool CReacNavPTGApp3D::OnNewMail(MOOSMSG_LIST &NewMail)
{
    std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
		if( (i->GetName()=="PNAVIGATORREACTIVEPTG3D_CMD") && (MOOSStrCmp(i->GetString(),"CANCEL")))
		{
			// Pause navigation manually:
			m_navigator.m_navstate = PAUSED;
			m_navigator.m_dynfeatures.new_cmd_v = 0;
			m_navigator.m_dynfeatures.new_cmd_w = 0;
			changeSpeeds(m_navigator.m_dynfeatures.new_cmd_v,m_navigator.m_dynfeatures.new_cmd_w);
		}
		if (i->GetName()=="IRD_WARNING")
			if (MOOSStrCmp(i->GetString(),"true"))
				m_ird_warning = true;
			else
				m_ird_warning = false;
	}

    UpdateMOOSVariables(NewMail);
    return true;
}


bool CReacNavPTGApp3D::DoReactiveNavigation()
{
	try
	{
		//CTicTac	ctest;
		//ctest.Tic();
		
		float vaux,waux;
		unsigned int too_near_obstacles = 0;
		float pose_estimation_goodness;
		CMOOSVariable *pVarLoc = GetMOOSVar( "EST_GOODNESS" );
		if (pVarLoc)
			pose_estimation_goodness = pVarLoc->GetDoubleVal();


		// Do navigation step:
		// --------------------------------------------------

		switch (m_navigator.m_navstate)
		{
		case (CANCELLED):
			m_Comms.Notify("SHUTDOWN",1);
			break;

		case (PAUSED):

			//Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);
			m_navigator.m_robmov.setRealPose( m_navigator.m_dynfeatures.curpose);			
			
			break;

		case (WAITING):

			//Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);
			m_navigator.m_robmov.setRealPose( m_navigator.m_dynfeatures.curpose);			
			
			//Read sensor measurements and save them in the reactive class variables
			sensorDataToReactive(m_navigator);
			m_navigator.ClassifyPointsInLevels();

			//Update virtual obstacles (if relocalization hasn't occurred)
			if (m_memory_on && (pose_estimation_goodness > m_reloc_threshold))
			{
				UpdateObstacleGrid();
				if (m_visualization)
					UpdateGridScene();
			}

			m_navigator.ObstaclesToTPSpace();
			m_navigator.ApplyHolonomicMethod();  
			m_navigator.EvaluateAllPTGs();
			m_navigator.PTG_Selector();
			m_navigator.GenerateSpeedCommands();

			for (unsigned int i=0; i<m_navigator.m_ptgmultilevel[m_navigator.m_reactiveparam.m_ptgselected].TPObstacles.size(); i++)
			{
				if ( m_navigator.m_ptgmultilevel[m_navigator.m_reactiveparam.m_ptgselected].TPObstacles[i] < 0.05)		//Experimental threshold
					too_near_obstacles++;
			}
			if ((too_near_obstacles < 0.5*m_navigator.m_ptgmultilevel[m_navigator.m_reactiveparam.m_ptgselected].TPObstacles.size())&& m_ird_warning == 0)
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
				m_navigator.m_navstate = ACTIVE;
			}

			//Save log file if pertinent
			if ((m_navigator.m_reactiveparam.m_recordLogFile == 1)&&(m_iteration % m_log_decimation == 0))
			{
				m_navigator.m_newLogRec.estimatedExecutionPeriod = m_log_decimation*m_reactive_period;
				m_navigator.SaveInLogFile( m_navigator.m_logFile );
			}

			//Move backward to find an alternative
			if (m_stepback_clock.Tac() < 3)
				changeSpeeds( -0.1, 0);
			else
				changeSpeeds( 0, 0);

			break;

		case (ACTIVE):

			//Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);
			m_navigator.m_robmov.setRealPose( m_navigator.m_dynfeatures.curpose);

			//Read sensor measurements and save them in the reactive class variables
			sensorDataToReactive(m_navigator);
			m_navigator.ClassifyPointsInLevels();

			//Update virtual obstacles (if relocalization hasn't occurred)
			if (m_memory_on && (pose_estimation_goodness > m_reloc_threshold))
			{
				UpdateObstacleGrid();
				if (m_visualization)
					UpdateGridScene();
			}

			//Reactive steps
			m_navigator.ObstaclesToTPSpace();
			m_navigator.ApplyHolonomicMethod(); 
			m_navigator.EvaluateAllPTGs();
			m_navigator.PTG_Selector();
			m_navigator.GenerateSpeedCommands();

			//Target proximity stop condition
			if (m_navigator.m_reactiveparam.rel_Target.norm() < 0.5)
			{
				if (m_event_sent == 0)
				{
					sendNavigationEndEvent();
					m_event_sent = 1;
				}
				if (m_navigator.m_reactiveparam.rel_Target.norm() < 0.3)
				{
					m_navigator.m_dynfeatures.new_cmd_v = 0;
					m_navigator.m_dynfeatures.new_cmd_w = 0;
				}
			}

			std::vector<float> &TPObstacles_ptgs = m_navigator.m_ptgmultilevel[m_navigator.m_reactiveparam.m_ptgselected].TPObstacles;

			//Too many obstacles stop condition and speed control
			m_obs_average = 0;
			unsigned weight_sum = 0;
			unsigned weight;
			for (unsigned int i=0; i<TPObstacles_ptgs.size(); i++)
			{
				if ( TPObstacles_ptgs[i] < 0.06)		//Experimental threshold
					too_near_obstacles++;
				
				weight = min<unsigned>(i, TPObstacles_ptgs.size()-1-i);
				m_obs_average += weight*TPObstacles_ptgs[i];
				weight_sum += weight;
			}

			m_obs_average = m_obs_average/float(weight_sum);
			//cout << endl << "Obs_average: " << m_obs_average;

			//Respect max and min speed (Although it is not exact due to the filter...)
			m_speed_reducer = vred_a*m_obs_average + vred_b;
			if ((m_navigator.m_dynfeatures.new_cmd_v > 0)&&(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_v < m_minv) && (abs(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_w) < m_minw))
			{
				if (m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_v/m_minv < abs(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_w)/m_minw)
					m_speed_reducer = abs(m_minw/m_navigator.m_dynfeatures.new_cmd_w);

				else
					m_speed_reducer = m_minv/m_navigator.m_dynfeatures.new_cmd_v;
			}

			else if ((m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_v > m_maxv)||(abs(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_w) > m_maxw))
			{
				if (m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_v/m_maxv < abs(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_w)/m_maxw)
					m_speed_reducer = abs(m_maxw/m_navigator.m_dynfeatures.new_cmd_w);

				else
					m_speed_reducer = m_maxv/m_navigator.m_dynfeatures.new_cmd_v;
			}

			if ((too_near_obstacles > 0.5*TPObstacles_ptgs.size()) || m_ird_warning == 1)
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
			}

			//Filter speed commmands
			//float period = 1/m_ini.read_float("","AppTick", 10, true);
			float alfa = m_reactive_period/(m_reactive_period + m_navigator.m_dynfeatures.ROBOTMODEL_TAU);
			m_navigator.m_dynfeatures.new_cmd_v = m_speed_reducer*alfa*m_navigator.m_dynfeatures.new_cmd_v + (1-alfa)*m_navigator.m_dynfeatures.last_cmd_v;
			m_navigator.m_dynfeatures.new_cmd_w = m_speed_reducer*alfa*m_navigator.m_dynfeatures.new_cmd_w + (1-alfa)*m_navigator.m_dynfeatures.last_cmd_w;

			//Change to "PAUSED" state
			if ((m_navigator.m_reactiveparam.rel_Target.norm() < 0.3)&&(m_navigator.m_dynfeatures.new_cmd_v < 0.002))
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
				m_navigator.m_navstate = PAUSED;
			}

			//Change to "WAITING" state
			if (((too_near_obstacles > 0.5*TPObstacles_ptgs.size())|| m_ird_warning == 1)&&(m_navigator.m_dynfeatures.new_cmd_v < 0.002))
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
				m_navigator.m_navstate = WAITING;
				m_stepback_clock.Tic();
			}

			changeSpeeds(m_navigator.m_dynfeatures.new_cmd_v,m_navigator.m_dynfeatures.new_cmd_w);


			//Save log file if pertinent
			if ((m_navigator.m_reactiveparam.m_recordLogFile == 1)&&(m_iteration % m_log_decimation == 0))
			{
				m_navigator.m_newLogRec.estimatedExecutionPeriod = m_log_decimation*m_reactive_period;
				m_navigator.SaveInLogFile( m_navigator.m_logFile );
			}

			break;
		}

		//cout << endl << "Tiempo total: " << ctest.Tac();

		m_reactive_period = m_reactive_clock.Tac();
		m_reactive_clock.Tic();
		cout << endl << "Reactive period: " << m_reactive_period << " seconds";
		m_iteration++;

		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail(e.what() );
	}
}


/** Get the current pose and speeds of the robot.
 *   \param curPose Current robot pose.
 *   \param curV Current linear speed, in meters per second.
 *	 \param curW Current angular speed, in radians per second.
 * \return false on any error.
 */
bool CReacNavPTGApp3D::getCurrentPoseAndSpeeds( mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CMOOSVariable *pVarLoc = GetMOOSVar( "LOCALIZATION" );
	if(!pVarLoc)
		return MOOSFail("[CReacNavPTGApp::getCurrentPoseAndSpeeds] ERROR: No LOCALIZATION data.");

	if(pVarLoc->GetAge(MOOSTime())>10.0)
	{
		MOOSFail("[Age is %f\n",pVarLoc->GetAge(MOOSTime()));
		MOOSFail("[%s]",pVarLoc->GetStringVal().c_str());
		return MOOSFail("[CReacNavPTGApp::getCurrentPoseAndSpeeds] ERROR: LOCALIZATION data too old.");
	}
	else if (pVarLoc->IsFresh())
	{
		pVarLoc->SetFresh(false);
		curPose.fromString( pVarLoc->GetStringVal() );
		curV = 0;
		curW = 0;
		return true;
	}
	else
	{
		curPose.fromString( pVarLoc->GetStringVal() );
		curV = 0;
		curW = 0;
		return false;
	}
}


/** Change the instantaneous speeds of robot.
 *   \param v Linear speed, in meters per second.
 *	 \param w Angular speed, in radians per second.
 * \return false on any error.
 */
bool CReacNavPTGApp3D::changeSpeeds( float v, float w )
{
	//!  @moos_publish   MOTION_CMD_V  The desired robot linear speed (m/s)
	//!  @moos_publish   MOTION_CMD_W  The desired robot angular speed (rad/s)
	m_Comms.Notify("MOTION_CMD_V", v );
	m_Comms.Notify("MOTION_CMD_W", w );

	return true;
}


/** Start the watchdog timer of the robot platform, if any.
 * \param T_ms Period, in ms.
 * \return false on any error.
 */
bool CReacNavPTGApp3D::startWatchdog(float T_ms)
{
	return true;
}


/** Stop the watchdog timer.
 * \return false on any error.
 */
bool CReacNavPTGApp3D::stopWatchdog()
{
	return true;
}


void CReacNavPTGApp3D::sendNavigationStartEvent ()
{
	//!  @moos_publish   NAV_EVENT_START  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_START", "" );
}

void CReacNavPTGApp3D::sendNavigationEndEvent()
{
	//!  @moos_publish   NAV_EVENT_END  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_END", "" );
	this->SendSystemLogString("Navigation about to end.");
}

void CReacNavPTGApp3D::sendNavigationEndDueToErrorEvent()
{
	//!  @moos_publish   NAV_EVENT_ERROR  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_ERROR", "" );
	this->SendSystemLogString("Navigation ended with ERROR.");
}

void CReacNavPTGApp3D::sendWaySeemsBlockedEvent()
{
	//!  @moos_publish   NAV_EVENT_NOWAY  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_NOWAY", "" );
	this->SendSystemLogString("Navigation ended with NO WAY error.");
}

void CReacNavPTGApp3D::notifyHeadingDirection(const double heading_dir_angle)
{
	double head=RAD2DEG( heading_dir_angle );
	if (fabs(head)>45) head=0;

	m_Comms.Notify("NECKMSG", format("ANG=%.1f,FAST=0,FILTER=1", head ) );
}


void CReacNavPTGApp3D::sensorDataToReactive(CReactiveNavigator &nav)
{
	CMOOSVariable *pVarLaser1 = GetMOOSVar( "LASER1" );
	CMOOSVariable *pVarLaser2 = GetMOOSVar( "LASER2" );

	CSerializablePtr obj;

	//Laser1
	if(pVarLaser1 && pVarLaser1->IsFresh())
	{
		pVarLaser1->SetFresh(false);
		//StringToObject(pVarLaser1->GetStringVal(),obj); (deprecated)
		mrpt::utils::RawStringToObject(pVarLaser1->GetStringRef(),obj);
		if (obj && IS_CLASS(obj,CObservation2DRangeScan))
			nav.m_robot.m_lasers[0].m_scan = *CObservation2DRangeScanPtr(obj);
	}

	//Laser2
	if(pVarLaser2 && pVarLaser2->IsFresh())
	{
		pVarLaser2->SetFresh(false);
		//StringToObject(pVarLaser2->GetStringVal(),obj);	(deprecated)
		mrpt::utils::RawStringToObject(pVarLaser2->GetStringRef(),obj);
		if (obj && IS_CLASS(obj,CObservation2DRangeScan))
			nav.m_robot.m_lasers[1].m_scan = *CObservation2DRangeScanPtr(obj);
	}

	//Kinect
	CMOOSVariable *pVarkinect = GetMOOSVar( "KINECT1" );
	if(pVarkinect && pVarkinect->IsFresh())
	{
		pVarkinect->SetFresh(false);
		StringToObject(pVarkinect->GetStringVal(),obj);
		if (obj && IS_CLASS(obj,CSimplePointsMap))
			nav.m_robot.m_kinects[0].m_points = *CSimplePointsMapPtr(obj);
	}
}

void CReacNavPTGApp3D::InitializeObstacleGrid()
{
	float grid_length = m_ini.read_float("","Obs_grid_length", 0.8, 1);
	float grid_resolution = m_ini.read_float("","Obs_grid_resolution", 0.1, 1);
	m_vision_limit = m_ini.read_float("","Vision_limit", 0.6, 1);
	m_pos_likelihood_incr = m_ini.read_float("","Pos_likelihood_incr", 0.55, 1);
	m_neg_likelihood_incr = m_ini.read_float("","Neg_likelihood_incr", 0.45, 1);
	m_occupancy_threshold = m_ini.read_float("","Occupancy_threshold", 0.8, 1);
	m_reloc_threshold = m_ini.read_float("","Likelihood_threshold", -2, 1);

	m_robot_ingrid.x = 0;
	m_robot_ingrid.y = 0;

	m_dyngrid.resize(m_navigator.m_robot.m_levels.size());
	for (unsigned int i=0; i < m_dyngrid.size(); i++)
	{
		m_dyngrid[i].setSize(-grid_length, grid_length, -grid_length, grid_length, grid_resolution, 0.5);
	}
}

void CReacNavPTGApp3D::UpdateObstacleGrid()
{
	//First, move the robot respect to the grid and adjust the likelihood values in the grid according to that movement
	//-----------------------------------------------------------------------------------------------------------------
	float incrx = m_navigator.m_dynfeatures.curpose[0] - m_last_pose[0];
	float incry = m_navigator.m_dynfeatures.curpose[1] - m_last_pose[1];

	if (( abs(m_robot_ingrid.x + incrx) < m_dyngrid[0].getResolution())&&( abs(m_robot_ingrid.y + incry) < m_dyngrid[0].getResolution())) 
	// The grid hasn't to be diplaced
	{
		m_robot_ingrid.x = m_robot_ingrid.x + incrx;
		m_robot_ingrid.y = m_robot_ingrid.y + incry;
	}
	else if (sqrt(square(incrx) + square(incry)) > 2.6*m_dyngrid[0].getXMax()) 
	// The displacement is too big so the grid is reset
	{
		for (unsigned int i=0; i < m_dyngrid.size(); i++)
		{
			m_dyngrid[i].setSize(m_dyngrid[0].getXMin(), m_dyngrid[0].getXMax(), m_dyngrid[0].getYMin(), m_dyngrid[0].getXMax(), m_dyngrid[0].getResolution(), 0.515);
		}	
	}
	else
	// The grid is displaced according to the robot movement
	{
		int despx = m_dyngrid[0].x2idx(m_robot_ingrid.x + incrx) - m_dyngrid[0].x2idx(m_robot_ingrid.x); 
		int despy = m_dyngrid[0].y2idx(m_robot_ingrid.y + incry) - m_dyngrid[0].y2idx(m_robot_ingrid.y);
		int despxpos = abs(despx);
		int despypos = abs(despy);
		float despxmeters = despx*m_dyngrid[0].getResolution(); 
		float despymeters = despy*m_dyngrid[0].getResolution();

		float xcel, ycel;
		vector <float> cells_newval;

		//For each of the "n" grids
		for (unsigned int n=0; n < m_dyngrid.size(); n++)
		{
			cells_newval.clear();
			
			//Cell values are stored
			for (unsigned int i = 0; i < m_dyngrid[n].getSizeX(); i++)
			{
				for (unsigned int j = 0; j < m_dyngrid[n].getSizeY(); j++)
				{
					xcel = m_dyngrid[n].idx2x(i) + despxmeters;
					ycel = m_dyngrid[n].idx2y(j) + despymeters;
					if ((abs(xcel) > m_dyngrid[n].getXMax()) || (abs(ycel) > m_dyngrid[n].getYMax()))
						cells_newval.push_back(-1);
					else
						cells_newval.push_back(m_dyngrid[n].getCell( i + despx, j + despy));
				}
			}

			//Cell values are updated in their new "positions"
			for (unsigned int i = 0; i < m_dyngrid[n].getSizeX(); i++)
			{
				for (unsigned int j = 0; j < m_dyngrid[n].getSizeY(); j++)
				{
					if (cells_newval[j + m_dyngrid[n].getSizeY()*i] == -1)
						m_dyngrid[n].setCell( i, j, 0.5);//0.5
					else
						m_dyngrid[n].setCell( i, j, cells_newval[j + m_dyngrid[n].getSizeY()*i] );
				}
			}
		}

		m_robot_ingrid.x = sign<float>(m_robot_ingrid.x + incrx)*remainder(abs(m_robot_ingrid.x + incrx),m_dyngrid[0].getResolution()); 
		m_robot_ingrid.y = sign<float>(m_robot_ingrid.y + incry)*remainder(abs(m_robot_ingrid.y + incry),m_dyngrid[0].getResolution());
	}


	//Second, update the likelihood values according to kinect scan
	//-------------------------------------------------------------

	float angrot = -m_navigator.m_dynfeatures.curpose.phi();
	float paso;
	float incr_grid_reactive = 0.2/m_dyngrid[0].getResolution();  //Number marks distance in meters (but it's transformed into an index) 
	TPoint3D paux;
	unsigned int index;
	unsigned int lim_visionxn = m_dyngrid[0].x2idx(-m_vision_limit + m_robot_ingrid.x);
	unsigned int lim_visionxp = m_dyngrid[0].x2idx(m_vision_limit + m_robot_ingrid.x);
	unsigned int lim_visionyn = m_dyngrid[0].x2idx(-m_vision_limit + m_robot_ingrid.y);
	unsigned int lim_visionyp = m_dyngrid[0].x2idx(m_vision_limit + m_robot_ingrid.y);
	unsigned int num_col = m_dyngrid[0].getSizeX();
	float xylim = m_dyngrid[0].getXMax();

	float level_height = 0;
	vector <bool> obs_in;
	obs_in.resize(square(num_col),0);
	m_gridpoints.clear();		//Clean the GridPointCloud

	for (unsigned int n=0; n < m_dyngrid.size(); n++)
	{
		obs_in.assign(square(num_col),0);
		
		//Vector obs_in is filled with 0 or 1 depending on the presence of any obstacle at each cell (of the grid)
		for (unsigned int i=0; i<m_navigator.m_robot.m_kinects[0].m_points.size(); i++)
		{
			m_navigator.m_robot.m_kinects[0].m_points.getPoint(i, paux);

			//Points rotation and translation
			paso = paux.x*cos(angrot) + paux.y*sin(angrot) + m_robot_ingrid.x;
			paux.y = -paux.x*sin(angrot) + paux.y*cos(angrot) + m_robot_ingrid.y;
			paux.x = paso;

			//Set binary occupancy of the cells (1 - there is at least one point, 0 - there isn't any point)
			if ((paux.x >= -xylim)&&(paux.x <= xylim)&&(paux.y >= -xylim)&&(paux.y <= xylim)&&(paux.z < level_height + m_navigator.m_robot.m_levels[n].m_height)&&(paux.z > level_height))
			{
				index = m_dyngrid[n].x2idx(paux.x) + num_col*m_dyngrid[n].y2idx(paux.y);
				obs_in[index] = 1;
			}
		}

		//The likelihood values of the grid are updated
		float angle_cell, dif_angle, rango = M_PI/6 - 0.05;

		for (unsigned int i=0; i<num_col; i++)
		{
			for (unsigned int j=0; j<num_col; j++)
			{
				if (obs_in[i + num_col*j] == 1)
					m_dyngrid[n].updateCell(i,j,m_pos_likelihood_incr);
				else if (((i < lim_visionxn)||(i > lim_visionxp))||((j < lim_visionyn)||(j > lim_visionyp)))
				{
					//The angle between the advance direction of the robot and the cell is calculated				
					angle_cell = atan2(m_dyngrid[n].idx2y(j)-m_robot_ingrid.y, m_dyngrid[n].idx2x(i)-m_robot_ingrid.x);
					dif_angle = abs(angle_cell - m_navigator.m_dynfeatures.curpose.phi());
					if (dif_angle > M_PI)
						dif_angle = dif_angle - 2*M_PI;
					if (abs(dif_angle) < rango)
						m_dyngrid[n].updateCell(i,j,m_neg_likelihood_incr);
				}
		
				//Transform the cell with high occupancy likelihood into 3D points
				if (((i >= lim_visionxn-incr_grid_reactive)&&(i <= lim_visionxp+incr_grid_reactive))&&((j >= lim_visionyn-incr_grid_reactive)&&(j <= lim_visionyp+incr_grid_reactive)))
				{
					if (m_dyngrid[n].getCell(i,j) > m_occupancy_threshold)
					{
						paux.x = (m_dyngrid[n].idx2x(i)-m_robot_ingrid.x)*cos(-angrot) + (m_dyngrid[n].idx2y(j)-m_robot_ingrid.y)*sin(-angrot);
						paux.y = -(m_dyngrid[n].idx2x(i)-m_robot_ingrid.x)*sin(-angrot) + (m_dyngrid[n].idx2y(j)-m_robot_ingrid.y)*cos(-angrot);
						paux.z = level_height + 0.02;
						m_gridpoints.insertPoint(paux.x, paux.y, paux.z);
						//Include points in their level for reactive navigation
						m_navigator.m_obstacles_inlevels[n].insertPoint(paux.x, paux.y, paux.z);
					}
				}
			}
		}

		level_height += m_navigator.m_robot.m_levels[n].m_height;
	}

	string sgridpoints = ObjectToString(&m_gridpoints);
	m_Comms.Notify("GRIDPOINTS", sgridpoints );

}

float CReacNavPTGApp3D::remainder(float dividend, float divisor)
{
	while (dividend > divisor)
		dividend -= divisor;
	return dividend;
}

void CReacNavPTGApp3D::InitializeGridScene()
{
	m_navigator.m_window = gui::CDisplayWindow3D::Create();
	
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;  

	m_navigator.m_window->setWindowTitle("Reactive Navigation. Obstacles Grid");
	m_navigator.m_window->resize(500,500);
	m_navigator.m_window->setPos(10,180);
	m_navigator.m_window->setCameraZoom(10);
	m_navigator.m_window->setCameraAzimuthDeg(90);
	m_navigator.m_window->setCameraElevationDeg(90);

	m_navigator.m_scene = m_navigator.m_window->get3DSceneAndLock();

	////The OccupancyGrid is inserted
	//CSetOfObjectsPtr obj1 = CSetOfObjects::Create();
	//m_memory.getAs3DObject(obj1);
	//m_navigator.m_scene->insert(obj1);

	//The "graphicGrid" is inserted 
	CGridPlaneXYPtr obj2 = CGridPlaneXY::Create(m_dyngrid[0].getXMin(), m_dyngrid[0].getXMax(), m_dyngrid[0].getYMin(), m_dyngrid[0].getYMax(), 0, m_dyngrid[0].getResolution());
	obj2->setColor(0.1,0.1,0.1);
	m_navigator.m_scene->insert( obj2 );

	//A representation of the robot is inserted
	//CBoxPtr obj3 = CBox::Create(TPoint3D(0,0,0),TPoint3D(m_dyngrid[0].getResolution()/3, m_dyngrid[0].getResolution()/6, m_dyngrid[0].getResolution()/6), true, 3.0);
	//obj3->setLocation(m_robot_ingrid.x, m_robot_ingrid.y, 0);
	//obj3->setColor(0.8,0,0);
	//m_navigator.m_scene->insert( obj3 );

	//The robot is inserted
	{
		float h;
		for (unsigned int i=0;i<m_navigator.m_robot.m_levels.size();i++)
		{
			if (i == 0) {h = 0;}
			else {h = m_navigator.m_robot.m_levels[i-1].m_height + h;}
			CPolyhedronPtr obj;
			obj = opengl::CPolyhedron::CreateCustomPrism(m_navigator.m_robot.m_levels[i].m_points, m_navigator.m_robot.m_levels[i].m_height);
			obj->setName(format("Level%d",i+1));
			obj->setPose(CPose3D(0,0,h,0,0,0));
			obj->setColor(0,0,1);
			obj->setWireframe(true);
			obj->setLineWidth(2);
			m_navigator.m_scene->insert( obj );
		}
	}

	//Points are inserted
	CPointCloudPtr obj4 = CPointCloud::Create();
	m_navigator.m_scene->insert( obj4 );
	obj4->setPose(CPose3D(0, 0, 0, m_navigator.m_dynfeatures.curpose.phi(), 0, 0));
	obj4->setPointSize(3.0);
	obj4->setColor(1,0,0);
	obj4->enablePointSmooth();
	TPoint3D paux;

	for (unsigned int i=0;i<m_navigator.m_robot.m_kinects[0].m_points.size();i++)
	{
		m_navigator.m_robot.m_kinects[0].m_points.getPoint(i, paux);
		obj4->insertPoint(paux.x, paux.y, paux.z);
	}

	//Frustum
	CFrustumPtr obj5 = CFrustum::Create(0.2, 3, 60, 48, 1.5f, true, false);
	obj5->setLocation( 0, 0, m_navigator.m_robot.m_kinects[0].m_zrel);
	m_navigator.m_scene->insert( obj5 );

	//Points are inserted
	CPointCloudPtr obj6 = CPointCloud::Create();
	m_navigator.m_scene->insert( obj6 );
	obj6->setPose(CPose3D(-m_robot_ingrid.x, -m_robot_ingrid.y, 0, m_navigator.m_dynfeatures.curpose.phi(), 0, 0));
	obj6->setPointSize(5.0);
	obj6->setColor(0,1,0);
	obj6->enablePointSmooth();

	for (unsigned int i=0;i<m_gridpoints.size();i++)
	{
		m_gridpoints.getPoint(i, paux);
		obj6->insertPoint(paux.x, paux.y, 0);
	}

	m_navigator.m_window->unlockAccess3DScene();
	m_navigator.m_window->forceRepaint();
}

void CReacNavPTGApp3D::UpdateGridScene()
{
	m_navigator.m_scene = m_navigator.m_window->get3DSceneAndLock();

	CGridPlaneXYPtr obj0;
	obj0 = m_navigator.m_scene->getByClass<CGridPlaneXY> (0);
	obj0->setLocation(-m_robot_ingrid.x, -m_robot_ingrid.y, 0);

	//CSetOfObjectsPtr obj1;
	//obj1 = m_navigator.m_scene->getByClass<CSetOfObjects> (0);
	//m_memory.getAs3DObject(obj1);
	//obj1->setLocation(-m_robot_ingrid.x, -m_robot_ingrid.y, 0);

	//CBoxPtr obj2;
	//obj2 = m_navigator.m_scene->getByClass<CBox> (0);
	//obj2->setPose(CPose2D(0, 0, m_navigator.m_dynfeatures.curpose[2]));

	//The robot pose is updated
	CRenderizablePtr obj1;
	float h;
	for (unsigned int i=0;i<m_navigator.m_robot.m_levels.size();i++)
	{
		obj1 = m_navigator.m_scene->getByName(format("Level%d",i+1));

		if (i == 0) {h = 0;}
		else { h = m_navigator.m_robot.m_levels[i-1].m_height + h;}
		obj1->setPose(CPose3D( 0, 0, h, m_navigator.m_dynfeatures.curpose[2], 0, 0));
	}

	CPointCloudPtr obj3;
	obj3 = m_navigator.m_scene->getByClass<CPointCloud> (0);
	obj3->clear();
	obj3->setPose(CPose3D(0, 0, 0, m_navigator.m_dynfeatures.curpose.phi(), 0, 0));
	TPoint3D paux;

	for (unsigned int i=0;i<m_navigator.m_robot.m_kinects[0].m_points.size();i++)
	{
		m_navigator.m_robot.m_kinects[0].m_points.getPoint(i, paux);
		obj3->insertPoint(paux.x, paux.y, paux.z);
	}

	CFrustumPtr obj4 ;
	obj4 = m_navigator.m_scene->getByClass<CFrustum> (0);
	obj4->setPose(CPose3D(0,0,m_navigator.m_robot.m_kinects[0].m_zrel,m_navigator.m_dynfeatures.curpose.phi(),0,0));

	CPointCloudPtr obj5;
	obj5 = m_navigator.m_scene->getByClass<CPointCloud> (1);
	obj5->clear();
	obj5->setPose(CPose3D(-m_robot_ingrid.x, -m_robot_ingrid.y, 0, m_navigator.m_dynfeatures.curpose.phi(), 0, 0));
	
	for (unsigned int i=0;i<m_gridpoints.size();i++)
	{
		m_gridpoints.getPoint(i, paux);
		obj5->insertPoint(paux.x, paux.y, paux.z);
	}

	m_navigator.m_window->unlockAccess3DScene();
	m_navigator.m_window->updateWindow();
}


