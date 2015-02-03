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

#ifndef CLaserOdometry2D_H
#define CLaserOdometry2D_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/system/threads.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/opengl.h>
#include <mrpt/math/CHistogram.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>

/*
#define M_LOG2E 1.44269504088896340736 //log2(e)
inline float log2(const float x)
{
    return  log(x) * M_LOG2E;
}
*/

class CLaserOdometry2D : public COpenMORAApp
{
public:
	CLaserOdometry2D();
	virtual ~CLaserOdometry2D();

protected:
	// Data
	std::vector<Eigen::MatrixXf> range;
	std::vector<Eigen::MatrixXf> range_old;
	std::vector<Eigen::MatrixXf> range_inter;
	std::vector<Eigen::MatrixXf> range_warped;
	std::vector<Eigen::MatrixXf> xx;
	std::vector<Eigen::MatrixXf> xx_inter;
	std::vector<Eigen::MatrixXf> xx_old;
	std::vector<Eigen::MatrixXf> xx_warped;
	std::vector<Eigen::MatrixXf> yy;
	std::vector<Eigen::MatrixXf> yy_inter;
	std::vector<Eigen::MatrixXf> yy_old;
	std::vector<Eigen::MatrixXf> yy_warped;
	std::vector<Eigen::MatrixXf> transformations;
	
	Eigen::MatrixXf range_wf;
	Eigen::MatrixXf dtita;
	Eigen::MatrixXf dt;
	Eigen::MatrixXf rtita;
	Eigen::MatrixXf normx, normy, norm_ang;
	Eigen::MatrixXf weights;
	Eigen::MatrixXi null;

	Eigen::MatrixXf A;
	Eigen::MatrixXf B;
	Eigen::Matrix<float, 3, 1> Var;	//3 unknowns: vx, vy, w
	Eigen::Matrix<float, 3, 3> cov_odo;

	std::string LaserVarName;				//Name of the OpenMora Variable containing the scan lasers
	float fps;								//In Hz
	float fovh;								//Horizontal FOV
	unsigned int cols;
	unsigned int cols_i;
	unsigned int width;
	unsigned int ctf_levels;
	unsigned int image_level, level;
	unsigned int num_valid_range;
	float g_mask[5];

	mrpt::gui::CDisplayWindowPlots window;
	mrpt::utils::CTicTac		m_clock;
	float		m_runtime;

	mrpt::math::CMatrixFloat31 kai_abs;
	mrpt::math::CMatrixFloat31 kai_loc;
	mrpt::math::CMatrixFloat31 kai_loc_old;
	mrpt::math::CMatrixFloat31 kai_loc_level;

	mrpt::poses::CPose3D laser_pose;
	mrpt::poses::CPose3D laser_oldpose;
	bool test;
	bool modedule_initialized;

	// Methods
	void Init();
	void createImagePyramid();
	void readLaser(mrpt::obs::CObservation2DRangeScan scan);
	bool readLaser();
	void calculateCoord();
	void performWarping();
	void calculaterangeDerivativesSurface();
	void computeNormals();
	void showGraph();
	void computeWeights();
	void findNullPoints();
	void solveSystemOneLevel();
	void filterLevelSolution();
	void PoseUpdate();
	void odometryCalculation();
	void Reset(mrpt::poses::CPose3D ini_pose, mrpt::obs::CObservation2DRangeScan scan);

	//MORA methods
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();
	/** called when new command */
	bool OnCommandMsg(CMOOSMsg Msg);
	/** performs the registration for mail */
	bool DoRegistrations();


	//If you want to save a log_file with info
	//ofstream				m_fres;
	//void OpenResFile();
	//void WriteTrajFile();
};

#endif