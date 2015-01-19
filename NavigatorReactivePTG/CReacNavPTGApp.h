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

#ifndef CReacNavPTGApp_H
#define CReacNavPTGApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt//nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/aligned_containers.h>

class CReacNavPTGApp : public COpenMORAApp, public mrpt::nav::CReactiveInterfaceImplementation
{
public:
    CReacNavPTGApp();

    virtual ~CReacNavPTGApp();

protected:
	// DATA
	mrpt::nav::CReactiveNavigationSystem	m_navigator;

	mrpt::maps::CSimplePointsMap 						m_obstacles;
	mrpt::maps::CSimplePointsMap						m_virtual_obstacles;
	mrpt::maps::COccupancyGridMap2D						m_memory;

	mrpt::gui::CDisplayWindowPtr   winMemoryMap;

	bool												m_obstacles_are_fresh;
	std::map<std::string, mrpt::obs::CObservationPtr> 	m_last_observations;

	std::set<std::string>	m_subscribedObstacleSensors;

	std::map<mrpt::system::TTimeStamp, mrpt::obs::CObservationPtr>  m_obs_win;

	struct pose_obs_win_data
	{
		pose_obs_win_data( const mrpt::poses::CPose2D &p,const mrpt::obs::CObservationPtr &o )
			: pose(p), obs(o) {};
		mrpt::poses::CPose2D	pose;
		mrpt::obs::CObservationPtr obs;
	};

	mrpt::aligned_containers<pose_obs_win_data>::vector_t m_pose_obs_win;

	std::string		m_obs_win_type;
	double			m_obs_win_temp_size;

	int		m_pose_win_diameter;
	int		m_pose_win_size_of_cell;

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

    /** Do the main processing.
	  *	\return false on error.
	  */
    bool DoReactiveNavigation();

	/** Must be called before navigating: parses the mission file, create collision tables, etc...
	  */
    bool DoNavigatorReset();

	bool loadVirtualObstaclesFromMatrix(const mrpt::math::CMatrixDouble &M);

	/* Functions for prepare obstacles sense by sensors, perhaps maintaining a window of past observations that check some requarements */
	void prepareObstaclesNoWin();
	void prepareObstaclesTemporalWin();
	void prepareObstaclesPoseWin();
	void prepareObstaclesMemory();

	bool insertIntoPoseGrid( const mrpt::poses::CPose2D &curPose, mrpt::obs::CObservationPtr &obs, std::vector<size_t> &index_obs_in_current_pose );



protected:
		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		bool getCurrentPoseAndSpeeds( mrpt::poses::CPose2D &curPose, float &curV, float &curW);

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

		/** Return the current set of obstacle points.
		  * \return false on any error.
		  */
		bool senseObstacles( mrpt::maps::CSimplePointsMap &obstacles );

		void sendNavigationStartEvent ();
		void sendNavigationEndEvent();
		void sendNavigationEndDueToErrorEvent();
		void sendWaySeemsBlockedEvent();
		void notifyHeadingDirection(const double heading_dir_angle);
};

#endif
