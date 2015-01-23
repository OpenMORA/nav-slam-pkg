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


/**  @moos_module A generic reactive navigator.
  *   <br> This module is a wrapper to the MRPT navigator class <a href="http://babel.isa.uma.es/mrpt/reference/svn/classmrpt_1_1reactivenav_1_1_c_reactive_navigation_system.html" target="_blank"> mrpt::reactivenav::CReactiveNavigationSystem </a>. <br>
  *     The internal algorithm is a purely reactive method (the Nearness Diagram Navigation) within one or more Parameterized Trajectory Generators (PTG) that abstract the robot shape and kinematic restrictions.
  *	   <br>
  *   <code><pre>
  *    Blanco, J.L. and Gonzalez, J. and Fernandez-Madrigal, J.A., Extending obstacle avoidance methods through multiple parameter-space transformations,
  *    Autonomous Robots, vol. 24, pp. 29-48, 2008.
  *   </pre></code>
  */

#include "CReacNavPTGApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>


using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace mrpt::obs;
using namespace mrpt::maps;


CReacNavPTGApp::CReacNavPTGApp() :
	m_navigator(
		*this,  // Implementation of reactive-robot interface
		true,	// enable console output
		false 	// enable log file
		)
{
	m_obstacles.insertionOptions.minDistBetweenLaserPoints = 0.02;
	m_obstacles.insertionOptions.fuseWithExisting = false;
	m_obstacles.insertionOptions.isPlanarMap = false;
}

CReacNavPTGApp::~CReacNavPTGApp()
{
}


bool CReacNavPTGApp::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{
		// Load config from .moos mission file:
		DoNavigatorReset();

		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CReacNavPTGApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pNavigatorReactivePTG only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	printf("COMMAND RECEIVED %s\n",sCmd.c_str());

	//!  @moos_cmd   CANCEL   Cancel the current navigation.
    if(MOOSStrCmp(sCmd,"CANCEL"))
    {
		m_navigator.cancel();
		this->SendSystemLogString("Navigation has been cancelled.");
	}
	//!  @moos_cmd   PAUSE  Pause the current navigation, which can be resumed later on.
    else if(MOOSStrCmp(sCmd,"PAUSE"))
    {
		m_navigator.suspend();
		this->SendSystemLogString("Navigation has been paused.");
	}
	//!  @moos_cmd   RESUME  Resume a paused navigation.
    else if(MOOSStrCmp(sCmd,"RESUME"))
    {
		m_navigator.resume();
		this->SendSystemLogString("Paused navigation has been resumed.");
	}

    return true;
}

bool CReacNavPTGApp::DoNavigatorReset()
{
	// Load parameters from moos config block:
	try
	{
		// Reload config:
		m_navigator.loadConfigFile(m_ini, m_ini);

        //! @moos_param save_log  Save reactive log (default:false)
		bool logging = m_ini.read_bool("","save_log",false);
		m_navigator.enableLogFile(logging);

        //! @moos_param virtual_obstacles  An optional list fixed obstacle points in global coordinates, as a 2xM or 3xM matrix with the X, Y and Z coordinates of the obstacles: "[x1 x2 x3 ...;y1 y2 y3 ...;z1 z2 z3...]"
		//!            Virtual obstacles can be also changed in runtime using the MOOS variable NAVIGATION_VIRTUAL_OBS. <br>
		//!            In fact, this module will also publish in NAVIGATION_VIRTUAL_OBS the obstacles read from the mission file, if any.
		std::string sObsVirt = m_ini.read_string("","virtual_obstacles","");
		if (!sObsVirt.empty())
			m_Comms.Notify("NAVIGATION_VIRTUAL_OBS", mrpt::system::trim(sObsVirt));

        //! @moos_param virtual_obstacles_file  An optional file with a list of fixed obstacle points in global coordinates, as a 2xM or 3xM matrix with the X, Y and Z coordinates of the obstacles: "[x1 x2 x3 ...;y1 y2 y3 ...;z1 z2 z3...]"
		//!            Virtual obstacles can be also changed in runtime using the MOOS variable NAVIGATION_VIRTUAL_OBS. <br>
		//!            In fact, this module will also publish in NAVIGATION_VIRTUAL_OBS the obstacles read from the given file, if any.
		std::string sObsVirtFile = m_ini.read_string("","virtual_obstacles_file","");
		if (!sObsVirtFile.empty() && mrpt::system::fileExists(sObsVirtFile))
		{
			MOOSTrace("Reading virtual obstacle file: %s\n", sObsVirtFile.c_str());
			CStringList ss;
			ss.loadFromFile(sObsVirtFile);
			m_Comms.Notify("NAVIGATION_VIRTUAL_OBS",mrpt::system::trim(ss.getText()));
		}

		// Observations window configuration
		{
			m_obs_win_type	=	m_ini.read_string("","obs_win_type","no_win");

			MOOSTrace( mrpt::format("Observations window type: %s",m_obs_win_type.c_str() ) );

			m_obs_win_temp_size = m_ini.read_double("","obseravations_window_temporal_size",1);

			m_pose_win_diameter			= m_ini.read_int("","pose_win_diameter", 90);
			m_pose_win_size_of_cell		= m_ini.read_int("","pose_win_size_of_cell", 10);
		}

		// Let's roll...
		m_navigator.initialize();

		return true;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}



bool CReacNavPTGApp::Iterate()
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
				CAbstractReactiveNavigationSystem::TNavigationParams   navParams;
				navParams.target.x = M(0,0);
				navParams.target.y = M(0,1) ;
				navParams.targetAllowedDistance = 0.40f;
				navParams.targetIsRelative = false;

				m_navigator.navigate( &navParams );

				//! @moos_publish MORA_GLOBAL_LOG
				this->SendSystemLogString(format("Starting navigation to (%.02f,%.02f).",navParams.target.x,navParams.target.y));
			}
			else MOOSTrace("ERROR: Invalid format of NAVIGATE_TARGET");
		}

		// Do we have a new navigation command??
		CMOOSVariable *pVarVObs = GetMOOSVar( "NAVIGATION_VIRTUAL_OBS" );
		if(pVarVObs && pVarVObs->IsFresh())
		{
			pVarVObs->SetFresh(false);
			CMatrixDouble M;
			if (M.fromMatlabStringFormat(pVarVObs->GetStringVal()) && (size(M,1)==2 || size(M,1)==3) && size(M,2)>=1)
			{
				this->loadVirtualObstaclesFromMatrix(M);
			}
			else MOOSTrace("ERROR: Invalid format of NAVIGATION_VIRTUAL_OBS. Expected 2xN or 3xN matrix.");
		}

		return DoReactiveNavigation();
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CReacNavPTGApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CReacNavPTGApp::DoRegistrations()
{
	//! @moos_subscribe	ODOMETRY, JOYSTICK_MODE
    AddMOOSVariable("ODOMETRY",  "ODOMETRY","ODOMETRY", 0.1 );
	AddMOOSVariable("JOYSTICK_MODE", "JOYSTICK_MODE", "JOYSTICK_MODE", 0);

	//! @moos_subscribe	LOCALIZATION, LOCALIZATION_COV
	AddMOOSVariable("LOCALIZATION_PF", "LOCALIZATION_PF", "LOCALIZATION_PF", 0.1);
	AddMOOSVariable("LOCALIZATION_COV", "LOCALIZATION_COV", "LOCALIZATION_COV", 0.1);

	//! @moos_subscribe	LASER1, LASER2, LASER3
	for (int i=1;i<=3;i++)
	{
		const string s=format("LASER%i",i);
		AddMOOSVariable(s,s,s,0.08);

		m_subscribedObstacleSensors.insert(s);
	}

	//! @moos_subscribe	SONAR1
	AddMOOSVariable("SONAR1","SONAR1","SONAR1",0.08);
	m_subscribedObstacleSensors.insert("SONAR1");

	//! @moos_subscribe	INFRARED1
	AddMOOSVariable("INFRARED1","INFRARED1","INFRARED1",0.08);
	m_subscribedObstacleSensors.insert("INFRARED1");

	//! @moos_subscribe	RANGECAM1
	AddMOOSVariable("RANGECAM1","RANGECAM1","RANGECAM1",0.08);
	m_subscribedObstacleSensors.insert("RANGECAM1");


	//! @moos_subscribe	NAVIGATE_TARGET
	AddMOOSVariable("NAVIGATE_TARGET", "NAVIGATE_TARGET", "NAVIGATE_TARGET", 0);

	//! @moos_subscribe	NAVIGATE_CANCEL
	AddMOOSVariable("NAVIGATE_CANCEL", "NAVIGATE_CANCEL", "NAVIGATE_CANCEL", 0);


	//! @moos_subscribe	NAVIGATION_VIRTUAL_OBS
	//! @moos_var NAVIGATION_VIRTUAL_OBS A list of fixed obstacle points in global coordinates, as a 2xM or 3xM matrix with the X, Y and Z coordinates of the obstacles: "[x1 x2 x3 ...;y1 y2 y3 ...;z1 z2 z3...]"
	//!            Virtual obstacles can be also loaded from the mission file at start up. See parameter virtual_obstacles
	AddMOOSVariable("NAVIGATION_VIRTUAL_OBS", "NAVIGATION_VIRTUAL_OBS", "NAVIGATION_VIRTUAL_OBS", 0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

    RegisterMOOSVariables();
    return true;
}


bool CReacNavPTGApp::OnNewMail(MOOSMSG_LIST &NewMail)
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

	}


    UpdateMOOSVariables(NewMail);
    return true;
}

bool CReacNavPTGApp::DoReactiveNavigation()
{
	try
	{
		// Process new observations:
		// --------------------------------------------------
		if ( m_obs_win_type == "no_win" ) prepareObstaclesNoWin();
		else if ( m_obs_win_type == "temp_win" ) prepareObstaclesTemporalWin();
		else if ( m_obs_win_type == "pose_win" ) prepareObstaclesPoseWin();
		else if ( m_obs_win_type == "memory_win") prepareObstaclesMemory();

		// Joystick mode?
		// --------------------------------------------------
		static bool lastJoyMode = false;
		CMOOSVariable *pVarJoy = GetMOOSVar( "JOYSTICK_MODE" );
		const bool isJoyMode= pVarJoy && ( atoi(pVarJoy->GetAsString().c_str()) !=0);
		if(isJoyMode)
		{
			// Just in the case of the first iteration in Joystick mode, stop the robot
			// since if there is no joystick the robot would continue with the latest velocities!
			if (!lastJoyMode)
			{
				m_Comms.Notify("MOTION_CMD_V", 0.0);
				m_Comms.Notify("MOTION_CMD_W", 0.0);
			}
			lastJoyMode = isJoyMode;

			// Nothing to do, we're in joystick mode.
			return true;
		}
		lastJoyMode = isJoyMode;

		// Do navigation step:
		// --------------------------------------------------
		m_navigator.navigationStep();

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
bool CReacNavPTGApp::getCurrentPoseAndSpeeds( mrpt::poses::CPose2D &curPose, float &curV, float &curW)
{
	CMOOSVariable *pVarLoc = GetMOOSVar( "LOCALIZATION_PF" );
	if(!pVarLoc)
		return MOOSFail("[CReacNavPTGApp::getCurrentPoseAndSpeeds] ERROR: No LOCALIZATION data.");

	if(pVarLoc->GetAge(MOOSTime())>10.0)
	{
		MOOSFail("[Age is %f\n",pVarLoc->GetAge(MOOSTime()));
		MOOSFail("[%s]",pVarLoc->GetStringVal().c_str());
		return MOOSFail("[CReacNavPTGApp::getCurrentPoseAndSpeeds] ERROR: LOCALIZATION data too old.");
	}
	// Parse:
	curPose.fromString( pVarLoc->GetStringVal() );
	curV = 0;
	curW = 0;

	return true;
}


/** Change the instantaneous speeds of robot.
 *   \param v Linear speed, in meters per second.
 *	 \param w Angular speed, in radians per second.
 * \return false on any error.
 */
bool CReacNavPTGApp::changeSpeeds( float v, float w )
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
bool CReacNavPTGApp::startWatchdog(float T_ms)
{
	return true;
}


/** Stop the watchdog timer.
 * \return false on any error.
 */
bool CReacNavPTGApp::stopWatchdog()
{
	return true;
}


/** Return the current set of obstacle points.
  * \return false on any error.
  */
bool CReacNavPTGApp::senseObstacles( mrpt::maps::CSimplePointsMap &obstacles )
{
	if (!m_obstacles_are_fresh)
	{
		this->SendSystemLogString("Error sensing obstacles: not fresh.");
		return false;
	}

	obstacles = m_obstacles;

	// Add virtual obstacles, if any:
	if (m_virtual_obstacles.size()!=0)
	{
		mrpt::poses::CPose2D curPose;
		float curV,curW;
		if (this->getCurrentPoseAndSpeeds(curPose, curV, curW))
		{
			// Insert with the displacement of the current robot pose.
			obstacles.insertAnotherMap(&m_virtual_obstacles,  CPose2D(0,0,0)-curPose );
		}
		else MOOSTrace("Warning: Couldn't not add virtual obstacles due to bad localization.");
	}

	return true;
}

void CReacNavPTGApp::sendNavigationStartEvent ()
{
	//!  @moos_publish   NAV_EVENT_START  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_START", "" );
}

void CReacNavPTGApp::sendNavigationEndEvent()
{
	//!  @moos_publish   NAV_EVENT_END  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_END", "" );
	this->SendSystemLogString("Navigation about to end.");
}

void CReacNavPTGApp::sendNavigationEndDueToErrorEvent()
{
	//!  @moos_publish   NAV_EVENT_ERROR  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_ERROR", "" );
	this->SendSystemLogString("Navigation ended with ERROR.");
}

void CReacNavPTGApp::sendWaySeemsBlockedEvent()
{
	//!  @moos_publish   NAV_EVENT_NOWAY  An event from the navigator.
	m_Comms.Notify("NAV_EVENT_NOWAY", "" );
	this->SendSystemLogString("Navigation ended with NO WAY error.");
}

void CReacNavPTGApp::notifyHeadingDirection(const double heading_dir_angle)
{
	double head=RAD2DEG( heading_dir_angle );
	if (fabs(head)>45) head=0;

	m_Comms.Notify("NECKMSG", format("ANG=%.1f,FAST=0,FILTER=1", head ) );
}

bool CReacNavPTGApp::loadVirtualObstaclesFromMatrix(const CMatrixDouble &M)
{
	try
	{
		if ((size(M,1)==2 || size(M,1)==3) && size(M,2)>=1)
		{
			const bool M_has_Z = size(M,1)==3;

			m_virtual_obstacles.clear();
			const size_t nPts = size(M,2);
			for (size_t i=0;i<nPts;i++)
				m_virtual_obstacles.insertPoint(M(0,i),M(1,i),M_has_Z ? M(2,i) : 0.8 );

			MOOSTrace("Acepted list of %u new virtual obstacles.\n",static_cast<unsigned int>(nPts));
			return true;
		}
		else
		{
			MOOSTrace("ERROR: Invalid format of NAVIGATION_VIRTUAL_OBS. Expected 2xN or 3xN matrix.");
			return false;
		}
	}
	catch( std::exception &e)
	{
		return MOOSFail("[loadVirtualObstaclesFromMatrix] Exception: %s\n",e.what());
	}
}

void CReacNavPTGApp::prepareObstaclesMemory()
{
	CObservation2DRangeScan	short_laser;
	mrpt::poses::CPose2D robotpose;
	float curv,curw;
	bool robotPoseAvailable = false;

	if( getCurrentPoseAndSpeeds(robotpose,curv,curw) )
		robotPoseAvailable = true;

	m_memory.insertionOptions.considerInvalidRangesAsFreeSpace=false;

	// Process new observations:
	// --------------------------------------------------

	for (set<string>::const_iterator itS = m_subscribedObstacleSensors.begin();itS!=m_subscribedObstacleSensors.end();++itS)
	{
		CMOOSVariable *pVarObs = GetMOOSVar( *itS );
		if(pVarObs && pVarObs->IsFresh())
		{
			pVarObs->SetFresh(false);
			CSerializablePtr obj;
			mrpt::utils::RawStringToObject(pVarObs->GetStringVal(),obj);
			if (obj && IS_DERIVED(obj,CObservation))
			{
				CObservationPtr obs = CObservationPtr(obj);
				m_last_observations[*itS] = obs;
			}
		}
	}

	// and update the obstacle point map:
	// --------------------------------------------------
	m_obstacles.clear();
	m_obstacles_are_fresh = false;
	const TTimeStamp  tNow = mrpt::system::now();

	for (map<string,CObservationPtr>::iterator itO = m_last_observations.begin();itO!=m_last_observations.end();++itO)
	{
		const double dT = mrpt::system::timeDifference(itO->second->timestamp, tNow);
		if (dT<1.0)
		{
			m_obstacles_are_fresh = true;
			m_obstacles.insertObservationPtr( itO->second );
			if ( robotPoseAvailable )
			{
				const CPose3D robotpose3D(robotpose);
				m_memory.insertObservationPtr(itO->second,&robotpose3D); // Danger: Only works if the robot pose is known!
			}

		}
	}

	if ( robotPoseAvailable )
	{
		if (winMemoryMap)
		{
			winMemoryMap = mrpt::gui::CDisplayWindow::Create("Reactive navigator: Obstacles memory",400,400);
			winMemoryMap->setPos(10,10);
		}



		short_laser.aperture = M_2PIf;
		short_laser.rightToLeft = true;
		short_laser.maxRange  = 2.0f;
		short_laser.stdError  = 0.00f;

		m_memory.laserScanSimulator(short_laser,robotpose,0.2); // Danger: Only works if the robot pose is known!

		m_obstacles.insertObservation(&short_laser);

		// Prepare data for map and obstacles visualization
		CImage img;
		m_memory.getAsImage(img);

		CSimplePointsMap mapmemory;
		const CPose3D robotpose3D(robotpose);
		mapmemory.insertObservation((CObservation*)&short_laser,&robotpose3D);
		std::vector<float> xs;
		std::vector<float> ys;
		mapmemory.getAllPoints(xs,ys);

		for (size_t i=0; i< xs.size();i++)
		{
			xs[i]=(20+xs[i])/m_memory.getResolution();
			ys[i]=(20-ys[i])/m_memory.getResolution();

		}

		double x=robotpose.x();
		double y=robotpose.y();
		img.drawCircle((x+20)/m_memory.getResolution(),(20-y)/m_memory.getResolution(),2/m_memory.getResolution(),TColor::red,1);
		winMemoryMap->showImageAndPoints(img,xs,ys,TColor::red,false);
	}

}


void CReacNavPTGApp::prepareObstaclesNoWin()
{
		// Process new observations:
		// --------------------------------------------------
		for (set<string>::const_iterator itS = m_subscribedObstacleSensors.begin();itS!=m_subscribedObstacleSensors.end();++itS)
		{
			CMOOSVariable *pVarObs = GetMOOSVar( *itS );
			if(pVarObs && pVarObs->IsFresh())
			{
				pVarObs->SetFresh(false);
				CSerializablePtr obj;
				mrpt::utils::RawStringToObject(pVarObs->GetStringVal(),obj);				
				if (obj && IS_DERIVED(obj,CObservation))
				{
					CObservationPtr obs = CObservationPtr(obj);
					m_last_observations[*itS] = obs;
				}
			}
		}

		// and update the obstacle point map:
		// --------------------------------------------------
		m_obstacles.clear();
		m_obstacles_are_fresh = false;
		const TTimeStamp  tNow = mrpt::system::now();

		for (map<string,CObservationPtr>::iterator itO = m_last_observations.begin();itO!=m_last_observations.end();++itO)
		{
			const double dT = mrpt::system::timeDifference(itO->second->timestamp, tNow);
			if (dT<1.0)
			{
				m_obstacles_are_fresh = true;
				m_obstacles.insertObservationPtr( itO->second );
			}
		}

}

void CReacNavPTGApp::prepareObstaclesTemporalWin()
{
	// 1. Obtain last observations
	for (set<string>::const_iterator itS = m_subscribedObstacleSensors.begin();itS!=m_subscribedObstacleSensors.end();++itS)
	{
		CMOOSVariable *pVarObs = GetMOOSVar( *itS );
		if(pVarObs && pVarObs->IsFresh())
		{
			pVarObs->SetFresh(false);
			CSerializablePtr obj;
			mrpt::utils::RawStringToObject(pVarObs->GetStringVal(),obj);			
			if (obj && IS_DERIVED(obj,CObservation))
			{
				CObservationPtr obs = CObservationPtr(obj);
					if ( pVarObs->GetName() == "RANGECAM1" )  // Observations from range cameras have a special process
					m_obs_win[ mrpt::system::now() ] = obs;
				else
					m_last_observations[*itS] = obs;
			}

		}
	}

	// 2. Delete old obs in m_obs_win, range camera observations
	size_t num_obs_to_delete = 0;

	for ( std::map<mrpt::system::TTimeStamp, mrpt::obs::CObservationPtr>::const_iterator it = m_obs_win.begin();
		it != m_obs_win.end(); it++ )
	{
		//cout << "TIME DIFF: "  << timeDifference(it->first, mrpt::system::now());
		if ( timeDifference(it->first, mrpt::system::now()) > m_obs_win_temp_size )
			num_obs_to_delete++;
	}

	while ( num_obs_to_delete )
	{
		m_obs_win.erase( m_obs_win.begin() );
		num_obs_to_delete--;
	}

	// 3. Update the obstacle points map
	{
		// In the past this was the only update performed
		m_obstacles.clear();
		m_obstacles_are_fresh = false;
		const TTimeStamp  tNow = mrpt::system::now();
			for (map<string,CObservationPtr>::iterator itO = m_last_observations.begin();itO!=m_last_observations.end();++itO)
		{
			const double dT = mrpt::system::timeDifference(itO->second->timestamp, tNow);
			if (dT<1.0)
			{
				m_obstacles_are_fresh = true;
				m_obstacles.insertObservationPtr( itO->second );
			}
		}
	}

	// 4. Update range camera observations thar are in the temporal window in the obstacle point map
	CMOOSVariable *pVarLoc = GetMOOSVar( "LOCALIZATION_PF" );
	if(pVarLoc)
	{
		// Obtain current pose
		CPose2D curPose;

		for ( std::map<mrpt::system::TTimeStamp, mrpt::obs::CObservationPtr>::const_iterator
		it = m_obs_win.begin(); it != m_obs_win.end(); it++ )
		{
			m_obstacles_are_fresh = true;
			//it->second->pose = it->second->pose - curPose;
			m_obstacles.insertObservationPtr( it->second );
		}
	}

	//cout << "Size m_obs_win: " << m_obs_win.size() << endl;
	//cout << "Num of observations: " << m_obstacles.size() << endl;

}

void CReacNavPTGApp::prepareObstaclesPoseWin()
{
	// Needed variables
	std::vector<size_t> index_obs_in_current_pose;

	// 1. Delete obs that now are out of the grid window with diameter m_pose_win_diameter
	CMOOSVariable *pVarLoc = GetMOOSVar( "LOCALIZATION_PF" );

	CPose2D curPose;
	bool curPoseOk = false;

	if(pVarLoc && (pVarLoc->GetStringVal()!="" ))
	{
		//cout << "Localization variable content: " << pVarLoc->GetStringVal() << endl;
		// Obtain current pose
		curPose.fromString( pVarLoc->GetStringVal() );
		curPoseOk = true;

		// Prepare needed values to compute only one time
		//size_t center = ( m_pose_win_diameter / 2 );
		double half_size_of_cell = m_pose_win_size_of_cell / 2;
		size_t half_pose_win_diameter = floor( (float)m_pose_win_diameter / 2 );
		size_t max_manhattan_distance = half_pose_win_diameter*2 - 2;

		// Check if each obs saved in the window is in the current grid.
		size_t N = m_pose_obs_win.size();
		size_t i = 0;
		while ( i < N )
		{
			// Obtain the relative pose of the obs to the current pose
			CPose2D relativeObsPose;
			relativeObsPose = m_pose_obs_win[i].pose - curPose;

			CVectorDouble v_obsPose;
			relativeObsPose.getAsVector( v_obsPose );

			// Obtain correspondent row into the grid
			size_t row = floor( ( v_obsPose[0] - half_size_of_cell ) / m_pose_win_size_of_cell );
			if ( row < 0 ) row = half_pose_win_diameter - row;

			// Obtain correspondent column into the grid
			size_t col = floor( ( v_obsPose[1] - half_size_of_cell ) / m_pose_win_size_of_cell );
			if ( col < 0 ) col = half_pose_win_diameter - col;

			// If the observation is in the current pose grid position store its index
			if ( !row && !col )
				index_obs_in_current_pose.push_back( i );

			// Calculate Manhattan distance
			size_t manhattan_distance;

			manhattan_distance = abs( (int)( half_pose_win_diameter - row )
										 + (int)( half_pose_win_diameter - col ) );

			cout << "Manhattan distace= " << manhattan_distance << endl;
			cout << "Row= " << row << endl;
			cout << "Col= " << col << endl;
			cout << "half pose win diameter: " << half_pose_win_diameter << endl;

			// Finally, check if the manhattan distance is less than the maximun distance
			if ( manhattan_distance > max_manhattan_distance )
			{
				// Now this observation is out, remove it!
				m_pose_obs_win.erase( m_pose_obs_win.begin() + i, m_pose_obs_win.begin() + i + 1 );
				N--; // Decrease the number of observations, but not the index to analyze
			}
			else
			{
				// Ok, the observation is in the grid, check the next one!
				i++;
			}
		}

	}

	// 2. Obtain last observations
	for (set<string>::const_iterator itS = m_subscribedObstacleSensors.begin();itS!=m_subscribedObstacleSensors.end();++itS)
	{
		CMOOSVariable *pVarObs = GetMOOSVar( *itS );
		if(pVarObs && pVarObs->IsFresh())
		{
			pVarObs->SetFresh(false);
			CSerializablePtr obj;
			mrpt::utils::RawStringToObject(pVarObs->GetStringVal(),obj);			
			if (obj && IS_DERIVED(obj,CObservation))
			{
				CObservationPtr obs = CObservationPtr(obj);
				if ( pVarObs->GetName() == "RANGECAM1" )  // Observations from range cameras have a special process
				{
					if ( curPoseOk ) cout << "Current pose OK! "; else cout << "Bad current pose! ";
					if ( curPoseOk && !insertIntoPoseGrid( curPose, obs, index_obs_in_current_pose ) )
						m_last_observations[*itS] = obs; // If it's inserted into the grid, anyway take in account
				}
				else
					m_last_observations[*itS] = obs;
			}

		}
	}

	// 3. Update the obstacle point map
	{
		// In the past this was the only update performed
		m_obstacles.clear();
		m_obstacles_are_fresh = false;
		const TTimeStamp  tNow = mrpt::system::now();
			for (map<string,CObservationPtr>::iterator itO = m_last_observations.begin();itO!=m_last_observations.end();++itO)
		{
			const double dT = mrpt::system::timeDifference(itO->second->timestamp, tNow);
			if (dT<1.0)
			{
				m_obstacles_are_fresh = true;
				m_obstacles.insertObservationPtr( itO->second );
			}
		}
	}
	// 4. Update range camera observations thar are in the pose window in the obstacle point map

	if( curPoseOk )
	{
		for ( size_t i = 0; i < m_pose_obs_win.size(); i++ )
		{
			m_obstacles_are_fresh = true;

			CPose3D sensorPose;
			m_pose_obs_win[i].obs->getSensorPose( sensorPose );
			CPose2D finalPose = m_pose_obs_win[i].pose - curPose;
			m_pose_obs_win[i].obs->setSensorPose( sensorPose + finalPose );

			m_obstacles.insertObservationPtr( m_pose_obs_win[i].obs );
		}
	}

	cout << "Number of poses saved in the window: " << m_pose_obs_win.size() << endl;

	//cout << "Size m_obs_win: " << m_obs_win.size() << endl;
	//cout << "Num of observations: " << m_obstacles.size() << endl;
}

bool CReacNavPTGApp::insertIntoPoseGrid( const CPose2D &curPose, CObservationPtr &obs, std::vector<size_t> &index_obs_in_current_pose )
{
	cout << "Inserting... " ;

	size_t N = index_obs_in_current_pose.size();

	if ( !N ) // If there aren't any obs, insert it!
	{
		cout << "Inserted!" << endl;
		m_pose_obs_win.push_back( pose_obs_win_data( curPose, obs ) );
		return true;
	}
	else // If there are observations, check its orientations
	{
		CVectorDouble v_curPose;
		curPose.getAsVector( v_curPose );

		size_t orientation_cell_angle = 30;

		//size_t orientations = ceil ((float)360 / orientation_cell_angle);
		size_t curOrientation = floor ( (float)v_curPose[2] / orientation_cell_angle );

		// Check if exist an observation with the same orientation and update it
		for ( size_t i = 0; i < N; i++ )
		{
			size_t index = index_obs_in_current_pose[i];

			CVectorDouble v_obsPose;
			m_pose_obs_win.at(index).pose.getAsVector( v_obsPose );

			size_t poseOrientation = floor ( (float)v_obsPose[2] / orientation_cell_angle );

			if ( curOrientation == poseOrientation )
			{
				// The same orientation, update it with the new observation!
				m_pose_obs_win[index].obs = obs;
				m_pose_obs_win[index].pose = curPose;
				return true;
			}
		}

		// If doesn't exist an observation with the same orientation, insert it
		m_pose_obs_win.push_back( pose_obs_win_data( curPose, obs ) );
	}

	return true;
}
