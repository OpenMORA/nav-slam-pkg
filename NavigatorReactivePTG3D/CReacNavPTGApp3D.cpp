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

/**  @moos_module A generic reactive navigator with 3D obstacles.
  *  This module is a wrapper to the MRPT navigator class mrpt::reactivenav::CReactiveNavigationSystem </a>. <br>
  *  The internal algorithm is a purely reactive method (the Nearness Diagram Navigation) within one or more 
  *  Parameterized Trajectory Generators (PTG) that abstract the robot shape and kinematic restrictions.
  *	  
  */

#include "CReacNavPTGApp3D.h"


using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::maps;



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
    if( Msg.IsSkewed(MOOSTime()) )
        return true;

    if( !Msg.IsString() )
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
		printf("[NavigatorReactive]: PAUSED\n");
		this->SendSystemLogString("Navigation has been paused.");
	}
	//!  @moos_cmd   RESUME  Resume a paused navigation.
    else if(MOOSStrCmp(sCmd,"RESUME"))
    {
		m_navigator.m_navstate = ACTIVE;
		printf("[NavigatorReactive]: ACTIVE\n");
		this->SendSystemLogString("Paused navigation has been resumed.");
	}

    return true;
}

bool CReacNavPTGApp3D::DoNavigatorReset()
{
	// Load parameters from moos config file:
	try
	{
		// Reload config parameters:
		m_navigator.loadRobotConfiguration(m_ini);
		m_navigator.m_logFile = NULL;
		m_navigator.EnableLogFile(m_navigator.m_logFile);

		//! @moos_param  Memory_on
		m_memory_on = m_ini.read_bool("","Memory_on", 1, false);
		//! @moos_param  Memory_visualization A opengl visualization of the local memory (only works if the memory is on)
		m_visualization = (m_memory_on && m_ini.read_bool("","Memory_visualization", 0, false));
		//! @moos_param  Speed_factor_obs Used to scale the robot speed according to the obstacles distribution (in TP-Space)
		speed_factor_obs = m_ini.read_float("","Speed_factor_obs", 1, false);
		//! @moos_param  Speed_factor_cons Used to scale the robot speed according to the obstacles distribution (in TP-Space)
		speed_factor_cons = m_ini.read_float("","Speed_factor_cons", 0.2, false);
		//! @moos_param  v_accel_lim Defines the maximum acceleration allowed in m/s2
		m_av_lim = m_ini.read_float("","v_accel_lim", 0.5f, true);
		//! @moos_param  w_accel_lim Defines the maximum angular acceleration allowed in deg/s2
		m_aw_lim = DEG2RAD(m_ini.read_float("","w_accel_lim", 90.f, true));
		//! @moos_param  VMIN_MPS
		m_minv = m_ini.read_float("","VMIN_MPS", 0.1, false);
		//! @moos_param  WMIN_DEGPS
		m_minw = DEG2RAD(m_ini.read_float("","WMIN_DEGPS", 10, false));
		//! @moos_param  VMAX_MPS
		m_maxv = m_ini.read_float("","VMAX_MPS", 0.6, false);
		//! @moos_param  WMAX_DEGPS
		m_maxw = DEG2RAD(m_ini.read_float("","WMAX_DEGPS", 60, false));
		//! @moos_param  LOG_DECIMATION
		m_log_decimation = m_ini.read_uint64_t("","LOG_DECIMATION", 1, false);
		m_iteration = 0;
		m_ird_warning = 0;

		//Speed initial values
		m_navigator.m_dynfeatures.new_cmd_v = 0.f;
		m_navigator.m_dynfeatures.new_cmd_w = 0.f;

		//Set the initial state as PAUSED
		m_navigator.m_navstate = PAUSED;
		printf("[NavigatorReactive]: Initialized as PAUSED\n");
		
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
		//Run the the reactive algorithm.
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
	//! @moos_subscribe	LOCALIZATION
	AddMOOSVariable("LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0);	

	//! @moos_subscribe	LASER (as many as are specified in the config file)
	for (unsigned int i=1; i<=m_navigator.m_robot.m_lasers.size(); i++)
	{
		string lasername = format("LASER%d",i);
		AddMOOSVariable(lasername, lasername, lasername, 0);
	}

	//! @moos_subscribe	RANGECAM (as many as are specified in the config file)
	for (unsigned int i=1; i<=m_navigator.m_robot.m_rangecams.size(); i++)
	{
		string rangecam_name = format("RANGECAM%d",i);
		AddMOOSVariable(rangecam_name, rangecam_name, rangecam_name, 0);
	}
	
	//! @moos_subscribe	NAVIGATE_TARGET
	AddMOOSVariable("NAVIGATE_TARGET", "NAVIGATE_TARGET", "NAVIGATE_TARGET", 0);
	
	//! @moos_subscribe	CANCEL
	AddMOOSVariable("CANCEL", "CANCEL", "CANCEL", 0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe EST_GOODNESS
	AddMOOSVariable( "EST_GOODNESS", "EST_GOODNESS", "EST_GOODNESS", 0 );	

	//! @moos_subscribe IRD_WARNING
	AddMOOSVariable( "IRD_WARNING", "IRD_WARNING", "IRD_WARNING", 0 );

	//! @moos_subscribe PNAVIGATORREACTIVEPTG3D_CMD
	AddMOOSVariable( "PNAVIGATORREACTIVEPTG3D_CMD", "PNAVIGATORREACTIVEPTG3D_CMD", "PNAVIGATORREACTIVEPTG3D_CMD", 0 );

    RegisterMOOSVariables();
    return true;
}


bool CReacNavPTGApp3D::OnNewMail(MOOSMSG_LIST &NewMail)
{
    std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		const CMOOSMsg &m = *i;

		// NAVIGATE_TARGET x y
		if( MOOSStrCmp(m.GetKey(),"NAVIGATE_TARGET") )
		{		
			CMatrixDouble M;
			if (M.fromMatlabStringFormat(m.GetString()) && size(M,1)==1 && size(M,2)>=2)
			{
				m_event_sent = 0;
				m_navigator.m_reactiveparam.WS_Target.x(M(0,0));
				m_navigator.m_reactiveparam.WS_Target.y(M(0,1));
				m_navigator.m_navstate = ACTIVE;
				printf("[NavigatorReactive]: ACTIVE\n");

				//Center the Neck
				//! @moos_publish NECKMSG
				m_Comms.Notify("NECKMSG","SERVO=0,ANG=0,SPEED=60,FILTER=1");
				
				//! @moos_publish MORA_GLOBAL_LOG
				this->SendSystemLogString(format("Starting navigation to (%.02f,%.02f).",
								m_navigator.m_reactiveparam.WS_Target[0], m_navigator.m_reactiveparam.WS_Target[1]));
			}
			else 
				MOOSTrace("[ReactiveNavigator3D]: ERROR - Invalid format of NAVIGATE_TARGET");
		}


		
		if( (MOOSStrCmp(m.GetKey(),"PNAVIGATORREACTIVEPTG3D_CMD")) && (MOOSStrCmp(m.GetString(),"CANCEL")) )		
		{
			// Canell navigation manually:
			printf("[NavigatorReactivePTG3D]: Canceling current reactive navigation\n");
			m_navigator.m_navstate = PAUSED;
			printf("[NavigatorReactive]: PAUSED\n");
			m_navigator.m_dynfeatures.new_cmd_v = 0.0;
			m_navigator.m_dynfeatures.new_cmd_w = 0.0;
			changeSpeeds(m_navigator.m_dynfeatures.new_cmd_v,m_navigator.m_dynfeatures.new_cmd_w);
		}

		if( MOOSStrCmp(m.GetKey(),"IRD_WARNING") )			
		{
			if( MOOSStrCmp(m.GetString(),"true") )
				m_ird_warning = true;
			else
				m_ird_warning = false;
		}
		
		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{
			MOOSTrace("Closing Module \n");
			this->RequestQuit();			
		}
	}

    UpdateMOOSVariables(NewMail);
    return true;
}



// The main function, executed in the iterate loop
bool CReacNavPTGApp3D::DoReactiveNavigation()
{
	try
	{	
		float vaux,waux;
		unsigned int too_near_obstacles = 0;
		float pose_estimation_goodness;
		CMOOSVariable *pVarLoc = GetMOOSVar( "EST_GOODNESS" );
		if (pVarLoc)
			pose_estimation_goodness = pVarLoc->GetDoubleVal();


		// Do navigation step:
		// -----------------------------------------------------------------
		switch (m_navigator.m_navstate)
		{
		case (CANCELLED):			
			//m_Comms.Notify("SHUTDOWN",1);
			break;

		case (PAUSED):
			//Only Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);		
			break;

		case (WAITING):
			//Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);		
			
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
				printf("[NavigatorReactive]: ACTIVE\n");
			}

			//Save log file if pertinent
			if ((m_navigator.m_reactiveparam.m_recordLogFile == 1)&&(m_iteration % m_log_decimation == 0))
			{
				m_navigator.m_newLogRec.estimatedExecutionPeriod = m_log_decimation*m_reactive_period;
				m_navigator.SaveInLogFile( m_navigator.m_logFile );
			}

			//Move backward to find an alternative
			if (m_stepback_clock.Tac() < 6)
			{
				printf("[NavigatorReactive]: Moving Backward to find alternative\n");
				changeSpeeds( -0.1, 0);
			}
			else
			{
				printf("[NavigatorReactive]: No alternative found. Keep waiting.\n");
				changeSpeeds( 0, 0);
			}

			break;

		case (ACTIVE):
			//Update the robot pose
			m_last_pose = m_navigator.m_dynfeatures.curpose;
			getCurrentPoseAndSpeeds( m_navigator.m_dynfeatures.curpose, vaux, waux);

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


			//Adapt the robot speed to the spatial obstacle distribution
			AdaptSpeedToObstacles();

			//Stop if obstacles are too close
			if ((too_near_obstacles > 0.5*TPObstacles_ptgs.size()) || m_ird_warning == 1)
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
			}

			//Apply acceleration limits (if necessary)
			ApplyAccelerationLimits();


			//Change to "PAUSED" state
			if ((m_navigator.m_reactiveparam.rel_Target.norm() < 0.3)&&(m_navigator.m_dynfeatures.new_cmd_v < 0.002))
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
				m_navigator.m_navstate = PAUSED;
				printf("[NavigatorReactive]: PAUSED\n");
			}

			//Change to "WAITING" state
			if (((too_near_obstacles > 0.5*TPObstacles_ptgs.size())|| m_ird_warning == 1)&&(m_navigator.m_dynfeatures.new_cmd_v < 0.002))
			{
				m_navigator.m_dynfeatures.new_cmd_v = 0;
				m_navigator.m_dynfeatures.new_cmd_w = 0;
				m_navigator.m_navstate = WAITING;
				m_stepback_clock.Tic();
				printf("[NavigatorReactive]: WAITING\n");
			}

			//Publish the new velocity commands
			changeSpeeds(m_navigator.m_dynfeatures.new_cmd_v,m_navigator.m_dynfeatures.new_cmd_w);


			//Save log file if pertinent
			if ((m_navigator.m_reactiveparam.m_recordLogFile == 1)&&(m_iteration % m_log_decimation == 0))
			{
				m_navigator.m_newLogRec.estimatedExecutionPeriod = m_log_decimation*m_reactive_period;
				m_navigator.SaveInLogFile( m_navigator.m_logFile );
			}

			break;
		}

		m_reactive_period = m_reactive_clock.Tac();
		m_reactive_clock.Tic();
		m_iteration++;

		//cout << endl << "[NavigatorReactive3D]: Reactive period: " << m_reactive_period << " seconds";

		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail(e.what() );
	}
}

/** Adapt the robot speed to the obstacle configuration in the TP-Space.
 */
void CReacNavPTGApp3D::AdaptSpeedToObstacles()
{
	m_speed_reducer = speed_factor_obs*m_obs_average + speed_factor_cons;
	if ((m_navigator.m_dynfeatures.new_cmd_v > 0.f)&&(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_v < m_minv) && (abs(m_speed_reducer*m_navigator.m_dynfeatures.new_cmd_w) < m_minw))
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
}

/** Constrain the motion commands (if necessary) according to the acceleration limits.
 */
void CReacNavPTGApp3D::ApplyAccelerationLimits()
{
	//Warning!!! The v-w proportion is modified
	
	//Check av_lim violation:
	const float v_incr = m_navigator.m_dynfeatures.new_cmd_v - m_navigator.m_dynfeatures.last_cmd_v;
	if (abs(v_incr) > m_av_lim*m_reactive_period)
		m_navigator.m_dynfeatures.new_cmd_v =  m_navigator.m_dynfeatures.last_cmd_v + sign(v_incr)*m_av_lim*m_reactive_period;

	//Check aw_lim violation:
	const float w_incr = m_navigator.m_dynfeatures.new_cmd_w - m_navigator.m_dynfeatures.last_cmd_w;
	if (abs(w_incr) > m_aw_lim*m_reactive_period)
		m_navigator.m_dynfeatures.new_cmd_w=  m_navigator.m_dynfeatures.last_cmd_w + sign(w_incr)*m_aw_lim*m_reactive_period;

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
	printf("[NavigatorReactive3D]: Motion_cmd_v/W = %0.3f m/s , %0.3f rad/s \n",v,w);
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

/** Read data published by the sensor modules.
 */
void CReacNavPTGApp3D::sensorDataToReactive(CReactiveNavigator &nav)
{	
	CSerializablePtr obj;

	const unsigned int num_lasers = m_navigator.m_robot.m_lasers.size();
	const unsigned int num_rangecams = m_navigator.m_robot.m_rangecams.size();

	//Read lasers
	for (unsigned int i=0; i<num_lasers; i++)
	{
		string lasername = format("LASER%d", i+1);
		CMOOSVariable *pVarLaser = GetMOOSVar( lasername );

		if(pVarLaser && pVarLaser->IsFresh())
		{
			pVarLaser->SetFresh(false);
			mrpt::utils::RawStringToObject(pVarLaser->GetStringRef(),obj);
			if (obj && IS_CLASS(obj,CObservation2DRangeScan))
				nav.m_robot.m_lasers[i].m_scan = *CObservation2DRangeScanPtr(obj);
		}
	}

	//Read range cameras
	for (unsigned int i=0; i<num_rangecams; i++)
	{
		string rangecam_name = format("RANGECAM%d", i+1);
		CMOOSVariable *pVarRangeCam = GetMOOSVar( rangecam_name );

		if(pVarRangeCam && pVarRangeCam->IsFresh())
		{
			pVarRangeCam->SetFresh(false);
			StringToObject(pVarRangeCam->GetStringVal(),obj);
			if (obj && IS_CLASS(obj,CSimplePointsMap))
				nav.m_robot.m_rangecams[i] = *CSimplePointsMapPtr(obj);
		}
	}
}

/** Initialize the local memory grid.
 */
void CReacNavPTGApp3D::InitializeObstacleGrid()
{
	m_ini.enableSectionNames();
	
	//! @moos_param  Obs_grid_length
	float grid_length = m_ini.read_float("","Obs_grid_length", 0.8, 1);
	//! @moos_param  Obs_grid_resolution
	float grid_resolution = m_ini.read_float("","Obs_grid_resolution", 0.1, 1);
	//! @moos_param  Vision_limit
	m_vision_limit = m_ini.read_float("","Vision_limit", 0.6, 1);
	//! @moos_param  Pos_likelihood_incr
	m_pos_likelihood_incr = m_ini.read_float("","Pos_likelihood_incr", 0.55, 1);
	//! @moos_param  Neg_likelihood_incr
	m_neg_likelihood_incr = m_ini.read_float("","Neg_likelihood_incr", 0.45, 1);
	//! @moos_param  Occupancy_threshold
	m_occupancy_threshold = m_ini.read_float("","Occupancy_threshold", 0.8, 1);
	//! @moos_param  Likelihood_threshold
	m_reloc_threshold = m_ini.read_float("Localization2D_PF","Likelihood_threshold", -2, 1);
	printf("\n Likelihood threshold = %f", m_reloc_threshold);

	m_robot_ingrid.x = 0;
	m_robot_ingrid.y = 0;

	m_dyngrid.resize(m_navigator.m_robot.m_levels.size());
	for (unsigned int i=0; i < m_dyngrid.size(); i++)
		m_dyngrid[i].setSize(-grid_length, grid_length, -grid_length, grid_length, grid_resolution, 0.5);

}

/** Update the local memory grid with the new obstacles
 */
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


	//Second, update the likelihood values according to range images
	//--------------------------------------------------------------

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
		for (unsigned int i=0; i<m_navigator.m_robot.m_rangecams[0].size(); i++)
		{
			m_navigator.m_robot.m_rangecams[0].getPoint(i, paux);

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

/** Initialize the opengl scene to show the local memory
 */
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

	//The "graphicGrid" is inserted 
	CGridPlaneXYPtr obj2 = CGridPlaneXY::Create(m_dyngrid[0].getXMin(), m_dyngrid[0].getXMax(), m_dyngrid[0].getYMin(), m_dyngrid[0].getYMax(), 0, m_dyngrid[0].getResolution());
	obj2->setColor(0.1,0.1,0.1);
	m_navigator.m_scene->insert( obj2 );

	//The robot is inserted
	float h;
	for (unsigned int i=0;i<m_navigator.m_robot.m_levels.size();i++)
	{
		if (i == 0) {h = 0;}
		else {h = m_navigator.m_robot.m_levels[i-1].m_height + h;}
		CPolyhedronPtr robotsec;
		robotsec = opengl::CPolyhedron::CreateCustomPrism(m_navigator.m_robot.m_levels[i].m_points, m_navigator.m_robot.m_levels[i].m_height);
		robotsec->setName(format("Level%d",i+1));
		robotsec->setPose(CPose3D(0,0,h,0,0,0));
		robotsec->setColor(0,0,1);
		robotsec->setWireframe(true);
		robotsec->setLineWidth(2);
		m_navigator.m_scene->insert( robotsec );
	}

	//Points are inserted
	CPointCloudPtr obj4 = CPointCloud::Create();
	m_navigator.m_scene->insert( obj4 );
	obj4->setPose(CPose3D(0, 0, 0, m_navigator.m_dynfeatures.curpose.phi(), 0, 0));
	obj4->setPointSize(3.0);
	obj4->setColor(1,0,0);
	obj4->enablePointSmooth();
	TPoint3D paux;

	for (unsigned int i=0;i<m_navigator.m_robot.m_rangecams[0].size();i++)
	{
		m_navigator.m_robot.m_rangecams[0].getPoint(i, paux);
		obj4->insertPoint(paux.x, paux.y, paux.z);
	}

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

/** Update the local memory visualization
 */
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

	for (unsigned int i=0;i<m_navigator.m_robot.m_rangecams[0].size();i++)
	{
		m_navigator.m_robot.m_rangecams[0].getPoint(i, paux);
		obj3->insertPoint(paux.x, paux.y, paux.z);
	}


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

/** Remainder function with float numbers
 */
float CReacNavPTGApp3D::remainder(float dividend, float divisor)
{
	while (dividend > divisor)
		dividend -= divisor;
	return dividend;
}
