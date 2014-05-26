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

#ifndef CPFLOCALIZATION_H
#define CPFLOCALIZATION_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>

class CPFLocalizationApp : public COpenMORAApp
{
public:
    CPFLocalizationApp();

    virtual ~CPFLocalizationApp();

protected:
	// DATA
	/** The robot pose PDF as a set of particles:
	  */
	mrpt::slam::CMonteCarloLocalization2D	m_PDF;
	/** The particle filter algorithm:
	  */
	mrpt::bayes::CParticleFilter		m_PF;

	/** The map */
	mrpt::slam::COccupancyGridMap2D	m_gridmap;

	mrpt::poses::CPose2D	m_lastOdo;
	bool					m_firstOdo; //!< true if the next one will be the first odometry

	/** Motion model options */
	mrpt::slam::CActionRobotMovement2D::TMotionModelOptions	m_motionModel;

	double	m_unif_x_min, m_unif_x_max, m_unif_y_min, m_unif_y_max;
	double	m_reloc_threshold;
	float	m_pose_goodness;

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

    /** Process odomety and/or laser data
	  *	\return false on error.
	  */
    bool ProcessParticleFilter();

    /** Publish  data
	  *	\return false on error.
	  */
    bool PublishPFLocalization();

    /** Restart the particle filter */
    bool OnPFReset();
};

#endif
