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

#ifndef CLocalizationFusion_H
#define CLocalizationFusion_H

#include <COpenMORAMOOSApp.h>
#include "CFusionPoseEstimator.h"

class CLocalizationFusionApp : public COpenMORAApp
{
public:
    CLocalizationFusionApp();

    virtual ~CLocalizationFusionApp();

protected:
	CFusionPoseEstimator				m_robot_pose;		//!< Latest best estimate of the robot pose
	mrpt::math::CMatrixDouble33 		m_last_robot_cov;	//!< As a simple solution, just re-publish the covariance "as-is" from our source.
	bool								m_last_robot_cov_valid;
	mrpt::math::TPose2D				m_odo_reference;

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


    bool PublishCurrentBestLocalization();
};

#endif
