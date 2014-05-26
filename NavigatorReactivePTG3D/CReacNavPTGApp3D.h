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

#ifndef CReacNavPTGApp_H
#define CReacNavPTGApp_H

#include <mrpt/utils.h>
#include <COpenMORAMOOSApp.h>
#include "reactnav3D_OM.h"


class CReacNavPTGApp3D : public COpenMORAApp
{
public:

	//CReacNavPTGApp();
	//virtual ~CReacNavPTGApp();

protected:
	// DATA
	CReactiveNavigator	m_navigator;

	mrpt::slam::CSimplePointsMap		m_gridpoints;
	mrpt::slam::COccupancyGridMap2D		m_memory;
	mrpt::poses::CPose2D				m_last_pose;
	mrpt::math::TPoint2D				m_robot_ingrid;
	float								m_vision_limit;
	float								m_pos_likelihood_incr;
	float								m_neg_likelihood_incr;
	float								m_occupancy_threshold;
	float								m_reloc_threshold;
	float								m_obs_average;
	float								m_speed_reducer;
	float								vred_a;
	float								vred_b;
	float								m_minv;
	float								m_minw;
	float								m_maxv;
	float								m_maxw;
	bool								m_memory_on;
	bool								m_event_sent;
	bool								m_ird_warning;


	vector <mrpt::slam::COccupancyGridMap2D>	m_dyngrid;

	CTicTac		m_reactive_clock;
	CTicTac		m_stepback_clock;
	float		m_reactive_period;
	bool		m_visualization;
	unsigned	m_iteration;
	unsigned	m_log_decimation;


    /** called at startup */
    virtual bool OnStartUp();

    /** called when new mail arrives */
    virtual bool OnNewMail(MOOSMSG_LIST & NewMail);

    /** called when work is to be done */
    virtual bool Iterate();

    /** called when app connects to DB */
    virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

    /** performs the registration for mail */
    bool DoRegistrations();

    /** Do the main processing. Return false on error. */
    bool DoReactiveNavigation();

	/** Must be called before navigating: parses the mission file, create collision tables, etc...*/
    bool DoNavigatorReset();

	void sensorDataToReactive(CReactiveNavigator &nav);

	void InitializeObstacleGrid();

	void UpdateObstacleGrid();

	void InitializeGridScene();

	void UpdateGridScene();

	float remainder(float dividend, float divisor);


protected:
		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool getCurrentPoseAndSpeeds( CPose2D &curPose, float &curV, float &curW);

		/** Change the instantaneous speeds of robot.
		 *   \param v Linear speed, in meters per second.
		 *	 \param w Angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool changeSpeeds( float v, float w );

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error.
		 */
		bool startWatchdog(float T_ms);

		/** Stop the watchdog timer.
		 * \return false on any error.
		 */
		bool stopWatchdog();

		void sendNavigationStartEvent ();
		void sendNavigationEndEvent();
		void sendNavigationEndDueToErrorEvent();
		void sendWaySeemsBlockedEvent();
		void notifyHeadingDirection(const double heading_dir_angle);
};

#endif
