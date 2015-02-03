/* +---------------------------------------------------------------------------+
|                 Open MORA (MObile Robot Arquitecture)                     |
|                                                                           |
|                        http://babel.isa.uma.es/mora/                      |
|                                                                           |
|   Copyright (C) 2015  University of Malaga                                |
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

/**  @moos_module Module to estimate the robot odometry from 2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry
*  Allowing the estimation of a precise odometry with low computational cost.
*/

#include "CLaserOdometry2D.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;
using namespace Eigen;
using namespace mrpt::obs;
using namespace mrpt::poses;
//using mrpt::poses::CPose3D;
//using mrpt::obs::CObservation2DRangeScan;

/* Constructor*/
CLaserOdometry2D::CLaserOdometry2D()
{	
}


/* Destructor */
CLaserOdometry2D::~CLaserOdometry2D()
{
}


/* called at startup */
bool CLaserOdometry2D::OnStartUp()
{
	// Read Module Params
	//--------------------
	//! @moos_param  LaserVarName  Name of the OpenMora variable containing the laser scans to be processed
	LaserVarName = m_ini.read_string("", "LaserVarName", "LASER1", true);

	modedule_initialized = false;

	// Register to OpenMORA variables of interest.
	DoRegistrations();

	return true;
}

void CLaserOdometry2D::Init()
{
	//Read obs to get its parameters
	bool laser_found = false;
	CSerializablePtr obj;
	printf("Initializing Module...\n");

	CMOOSVariable *pVar = GetMOOSVar(LaserVarName);
	if (pVar)
	{		
		mrpt::utils::RawStringToObject(pVar->GetStringVal(), obj);
		if (obj && IS_CLASS(obj, CObservation2DRangeScan))
			laser_found = true;
		else
			printf("ERROR: Variable %s does not contain a valid CObservation2DRangeScan\n", LaserVarName.c_str());
	}
	else
		printf("ERROR: cannot read from Variable: %s\n", LaserVarName.c_str());


	if (laser_found)
	{
		//Got scan laser, obtain its parametes		
		mrpt::obs::CObservation2DRangeScanPtr laser_obj = mrpt::obs::CObservation2DRangeScanPtr(obj);
		width = laser_obj->scan.size();		//width = 241;			Num of samples (size) of the scan laser
		cols = width;						// 241					Max reolution. Should be similar to the width parameter
		fovh = laser_obj->aperture;			//fovh = DEG2RAD(240)	Horizontal Laser's FOV
		ctf_levels = 4;
		mrpt::poses::CPose3D LaserPoseOnTheRobot;
		laser_obj->getSensorPose(LaserPoseOnTheRobot);
		//Set the initial pose
		laser_pose = LaserPoseOnTheRobot;
		laser_oldpose = LaserPoseOnTheRobot;


		// Init module
		//-------------
		range_wf.setSize(1, width);

		//Resize vectors according to levels
		transformations.resize(ctf_levels);
		for (unsigned int i = 0; i < ctf_levels; i++)
			transformations[i].resize(3, 3);

		//Resize pyramid
		unsigned int s, cols_i;
		const unsigned int pyr_levels = round(log2(round(float(width) / float(cols)))) + ctf_levels;
		range.resize(pyr_levels);
		range_old.resize(pyr_levels);
		range_inter.resize(pyr_levels);
		xx.resize(pyr_levels);
		xx_inter.resize(pyr_levels);
		xx_old.resize(pyr_levels);
		yy.resize(pyr_levels);
		yy_inter.resize(pyr_levels);
		yy_old.resize(pyr_levels);
		range_warped.resize(pyr_levels);
		xx_warped.resize(pyr_levels);
		yy_warped.resize(pyr_levels);

		for (unsigned int i = 0; i < pyr_levels; i++)
		{
			s = pow(2.f, int(i));
			cols_i = ceil(float(width) / float(s));

			range[i].resize(1, cols_i);
			range_inter[i].resize(1, cols_i);
			range_old[i].resize(1, cols_i);
			range[i].assign(0.0f);
			range_old[i].assign(0.0f);
			xx[i].resize(1, cols_i);
			xx_inter[i].resize(1, cols_i);
			xx_old[i].resize(1, cols_i);
			xx[i].assign(0.0f);
			xx_old[i].assign(0.0f);
			yy[i].resize(1, cols_i);
			yy_inter[i].resize(1, cols_i);
			yy_old[i].resize(1, cols_i);
			yy[i].assign(0.f);
			yy_old[i].assign(0.f);

			if (cols_i <= cols)
			{
				range_warped[i].resize(1, cols_i);
				xx_warped[i].resize(1, cols_i);
				yy_warped[i].resize(1, cols_i);
			}
		}

		dt.resize(1, cols);
		dtita.resize(1, cols);
		normx.resize(1, cols);
		normy.resize(1, cols);
		norm_ang.resize(1, cols);
		weights.setSize(1, cols);
		null.setSize(1, cols);
		null.assign(0);
		cov_odo.assign(0.f);


		fps = 1.f;		//In Hz
		num_valid_range = 0;

		//Compute gaussian mask
		g_mask[0] = 1.f / 16.f; g_mask[1] = 0.25f; g_mask[2] = 6.f / 16.f; g_mask[3] = g_mask[1]; g_mask[4] = g_mask[0];

		kai_abs.assign(0.f);
		kai_loc_old.assign(0.f);
		//OpenResFile();

		modedule_initialized = true;
	}
}

/** called when new mail arrives */
bool CLaserOdometry2D::OnNewMail(MOOSMSG_LIST & NewMail)
{
	std::string cad;
	for (MOOSMSG_LIST::iterator i = NewMail.begin(); i != NewMail.end(); ++i)
	{
		if ((i->GetName() == "SHUTDOWN") && (MOOSStrCmp(i->GetString(), "true")))
		{
			// Disconnect module:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}

	UpdateMOOSVariables(NewMail);
	return true;
}


/** called when work is to be done */
bool CLaserOdometry2D::Iterate()
{
	if (modedule_initialized)
	{
		//1. Read a new scan laser
		if (readLaser())
		{
			//2. Process odometry estimation
			odometryCalculation();
		}
	}
	else
		Init();

	return true;
}


/** called when app connects to DB */
bool CLaserOdometry2D::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


/** called when new command */
bool CLaserOdometry2D::OnCommandMsg(CMOOSMsg Msg)
{
	return true;
}


/** performs the registration for mail */
bool CLaserOdometry2D::DoRegistrations()
{
	printf("\n Registering to Variables: %s, SHUTDOWN\n", LaserVarName.c_str());
	//! @moos_subscribe <LaserVarName>
	//! @moos_var <LaserVarName> Variable containing Laser scans. The name of the variable is set as a parameter.
	AddMOOSVariable(LaserVarName, LaserVarName, LaserVarName, 0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable("SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0);

	RegisterMOOSVariables();
	return true;
}



/* Get a new scan laser */
void CLaserOdometry2D::readLaser(CObservation2DRangeScan scan)
{
	for (unsigned int i=0; i<width; i++)
		range_wf(i) = scan.scan[i];
}


/* Get a new scan laser from OpenMORA variable */
bool CLaserOdometry2D::readLaser()
{
	CMOOSVariable * pVar = GetMOOSVar(LaserVarName);
	if (pVar)
	{
		CSerializablePtr obj;
		mrpt::utils::RawStringToObject(pVar->GetStringVal(), obj);
		if (obj && IS_CLASS(obj, CObservation2DRangeScan))
		{
			mrpt::obs::CObservation2DRangeScanPtr laser_obj = mrpt::obs::CObservation2DRangeScanPtr(obj);			
			//copy laser scan to internal variable
			for (unsigned int i = 0; i<width; i++)
				range_wf(i) = laser_obj->scan[i];
			return true;
		}
		else
		{
			printf("ERROR: Variable %s does not contain a valid CObservation2DRangeScan\n", LaserVarName.c_str());
			return false;
		}
	}
	else
	{
		printf("ERROR: Variable %s does not exists\n", LaserVarName.c_str());
		return false;
	}
}


void CLaserOdometry2D::createImagePyramid()
{
	const float max_range_dif = 0.3f;
	
	//Push the frames back
	range_old.swap(range);
	xx_old.swap(xx);
	yy_old.swap(yy);

    //The number of levels of the pyramid does not match the number of levels used
    //in the odometry computation (because we sometimes want to finish with lower resolutions)

    unsigned int pyr_levels = round(log2(round(float(width)/float(cols)))) + ctf_levels;

    //Generate levels
    for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = ceil(float(width)/float(s));

		const unsigned int cols_i2 = 2*cols_i;
		const unsigned int i_1 = i-1;

		//First level -> Filter (not downsampling);
        if (i == 0)
		{
			for (unsigned int u = 0; u < cols_i; u++)
            {	
				const float dcenter = range_wf(u);
					
				//Inner pixels
                if ((u>1)&&(u<cols_i-2))
                {		
					if (dcenter > 0.f)
					{	
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)
						{
							const float abs_dif = abs(range_wf(u+l)-dcenter);
							if (abs_dif < max_range_dif)
							{
								const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
								weight += aux_w;
								sum += aux_w*range_wf(u+l);
							}
						}
						range[i](u) = sum/weight;
					}
					else
						range[i](u) = 0.f;

                }

                //Boundary
                else
                {
                    if (dcenter > 0.f)
					{						
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)	
						{
							const int indu = u+l;
							if ((indu>=0)&&(indu<cols_i))
							{
								const float abs_dif = abs(range_wf(indu)-dcenter);										
								if (abs_dif < max_range_dif)
								{
									const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
									weight += aux_w;
									sum += aux_w*range_wf(indu);
								}
							}
						}
						range[i](u) = sum/weight;
					}
					else
						range[i](u) = 0.f;

                }
            }
		}

        //                              Downsampling
        //-----------------------------------------------------------------------------
        else
        {            
			for (unsigned int u = 0; u < cols_i; u++)
            {
                const int u2 = 2*u;		
				const float dcenter = range[i_1](u2);
					
				//Inner pixels
                if ((u>0)&&(u<cols_i-1))
                {		
					if (dcenter > 0.f)
					{	
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)
						{
							const float abs_dif = abs(range[i_1](u2+l)-dcenter);
							if (abs_dif < max_range_dif)
							{
								const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
								weight += aux_w;
								sum += aux_w*range[i_1](u2+l);
							}
						}
						range[i](u) = sum/weight;
					}
					else
						range[i](u) = 0.f;

                }

                //Boundary
                else
                {
                    if (dcenter > 0.f)
					{						
						float sum = 0.f;
						float weight = 0.f;

						for (int l=-2; l<3; l++)	
						{
							const int indu = u2+l;
							if ((indu>=0)&&(indu<cols_i2))
							{
								const float abs_dif = abs(range[i_1](indu)-dcenter);										
								if (abs_dif < max_range_dif)
								{
									const float aux_w = g_mask[2+l]*(max_range_dif - abs_dif);
									weight += aux_w;
									sum += aux_w*range[i_1](indu);
								}
							}
						}
						range[i](u) = sum/weight;
					}
					else
						range[i](u) = 0.f;

                }
            }
        }

        //Calculate coordinates "xy" of the points
        for (unsigned int u = 0; u < cols_i; u++) 
		{
            if (range[i](u) > 0.f)
			{
				const float tita = -0.5*fovh + float(u)*fovh/float(cols_i-1);
				xx[i](u) = range[i](u)*cos(tita);
				yy[i](u) = range[i](u)*sin(tita);
			}
			else
			{
				xx[i](u) = 0.f;
				yy[i](u) = 0.f;
			}
		}
    }
}



void CLaserOdometry2D::calculateCoord()
{		
	for (unsigned int u = 0; u < cols_i; u++)
	{
		if ((range_old[image_level](u) == 0.f) || (range_warped[image_level](u) == 0.f))
		{
			range_inter[image_level](u) = 0.f;
			xx_inter[image_level](u) = 0.f;
			yy_inter[image_level](u) = 0.f;
		}
		else
		{
			range_inter[image_level](u) = 0.5f*(range_old[image_level](u) + range_warped[image_level](u));
			xx_inter[image_level](u) = 0.5f*(xx_old[image_level](u) + xx_warped[image_level](u));
			yy_inter[image_level](u) = 0.5f*(yy_old[image_level](u) + yy_warped[image_level](u));
		}
	}
}


void CLaserOdometry2D::calculaterangeDerivativesSurface()
{	
	//The gradient size ir reserved at the maximum size (at the constructor)

    //Compute connectivity
	rtita.resize(1,cols_i); 		//Defined in a different way now, without inversion
    rtita.assign(1.f); 

	for (unsigned int u = 0; u < cols_i-1; u++)
    {
		const float dist = square(xx_inter[image_level](u+1) - xx_inter[image_level](u))
							+ square(yy_inter[image_level](u+1) - yy_inter[image_level](u));
		if (dist  > 0.f)
			rtita(u) = sqrt(dist);
	}

    //Spatial derivatives
    for (unsigned int u = 1; u < cols_i-1; u++)
		dtita(u) = (rtita(u-1)*(range_inter[image_level](u+1)-range_inter[image_level](u)) + rtita(u)*(range_inter[image_level](u) - range_inter[image_level](u-1)))/(rtita(u)+rtita(u-1));

	dtita(0) = dtita(1);
	dtita(cols_i-1) = dtita(cols_i-2);

	//Temporal derivative
	for (unsigned int u = 0; u < cols_i; u++)
		dt(u) = fps*(range_warped[image_level](u) - range_old[image_level](u));


	//Apply median filter to the range derivatives
	//MatrixXf dtitamed = dtita, dtmed = dt;
	//vector<float> svector(3);
	//for (unsigned int u=1; u<cols_i-1; u++)
	//{
	//	svector.at(0) = dtita(u-1); svector.at(1) = dtita(u); svector.at(2) = dtita(u+1);
	//	std::sort(svector.begin(), svector.end());
	//	dtitamed(u) = svector.at(1);

	//	svector.at(0) = dt(u-1); svector.at(1) = dt(u); svector.at(2) = dt(u+1);
	//	std::sort(svector.begin(), svector.end());
	//	dtmed(u) = svector.at(1);
	//}

	//dtitamed(0) = dtitamed(1);
	//dtitamed(cols_i-1) = dtitamed(cols_i-2);
	//dtmed(0) = dtmed(1);
	//dtmed(cols_i-1) = dtmed(cols_i-2);

	//dtitamed.swap(dtita);
	//dtmed.swap(dt);
}


void CLaserOdometry2D::computeNormals()
{
	normx.assign(0.f);
	normy.assign(0.f);
	norm_ang.assign(0.f);

	const float incr_tita = fovh/float(cols_i-1);
	for (unsigned int u=0; u<cols_i; u++)
	{
		if (null(u) == 0.f)
		{
			const float tita = -0.5f*fovh + float(u)*incr_tita;
			const float alfa = -atan2(2.f*dtita(u), 2.f*range[image_level](u)*incr_tita);
			norm_ang(u) = tita + alfa;
			if (norm_ang(u) < -M_PI)
				norm_ang(u) += 2.f*M_PI;
			else if (norm_ang(u) < 0.f)
				norm_ang(u) += M_PI;
			else if (norm_ang(u) > M_PI)
				norm_ang(u) -= M_PI;

			normx(u) = cos(tita + alfa);
			normy(u) = sin(tita + alfa);
		}
	}
}


void CLaserOdometry2D::computeWeights()
{
	//The maximum weight size is reserved at the constructor
	weights.assign(0.f);
	
	//Parameters for error_linearization
	const float kdtita = 1.f;
	const float kdt = kdtita/square(fps);
	const float k2d = 0.2f;
	
	for (unsigned int u = 1; u < cols_i-1; u++)
		if (null(u) == 0)
		{	
			//							Compute derivatives
			//-----------------------------------------------------------------------
			const float ini_dtita = range_old[image_level](u+1) - range_old[image_level](u-1);
			const float final_dtita = range_warped[image_level](u+1) - range_warped[image_level](u-1);

			const float dtitat = ini_dtita - final_dtita;
			const float dtita2 = dtita(u+1) - dtita(u-1);

			const float w_der = kdt*square(dt(u)) + kdtita*square(dtita(u)) + k2d*(abs(dtitat) + abs(dtita2));

			weights(u) = sqrt(1.f/w_der);
		}

	const float inv_max = 1.f/weights.maximum();
	weights = inv_max*weights;
}


void CLaserOdometry2D::findNullPoints()
{
	//Size of null matrix is set to its maximum size (constructor)
	num_valid_range = 0;

	for (unsigned int u = 1; u < cols_i-1; u++)
	{
		if (range_inter[image_level](u) == 0.f)
			null(u) = 1;
		else
		{
			num_valid_range++;
			null(u) = 0;
		}
	}
}


void CLaserOdometry2D::showGraph()
{
	//For example: show dt
	if (level == ctf_levels-1)
	{
		std::vector<float> dtv;
		for (unsigned int u=0; u<cols; u++)
			dtv.push_back(dtita(u));

		window.axis(-2, cols+2, -1,1);
		window.plot(dtv);
	}
}


void CLaserOdometry2D::solveSystemOneLevel()
{
	A.resize(num_valid_range,3);
	B.setSize(num_valid_range,1);
	unsigned int cont = 0;
	const float kdtita = (cols_i-1)/fovh;

	//Fill the matrix A and the vector B
	//The order of the variables will be (vx, vy, wz)

	for (unsigned int u = 1; u < cols_i-1; u++)
		if (null(u) == 0)
		{
			// Precomputed expressions
			const float tw = weights(u);
			const float tita = -0.5*fovh + u/kdtita;

			//Fill the matrix A
			A(cont, 0) = tw*(cos(tita) + dtita(u)*kdtita*sin(tita)/range_inter[image_level](u));
			A(cont, 1) = tw*(sin(tita) - dtita(u)*kdtita*cos(tita)/range_inter[image_level](u));
			A(cont, 2) = tw*(-yy[image_level](u)*cos(tita) + xx[image_level](u)*sin(tita) - dtita(u)*kdtita);
			B(cont,0) = tw*(-dt(u));

			cont++;
		}
	
	//Solve the linear system of equations using a minimum least squares method
	MatrixXf AtA, AtB;
	AtA.multiply_AtA(A);
	AtB.multiply_AtB(A,B);
	Var = AtA.ldlt().solve(AtB);

	//Covariance matrix calculation 	Cov Order -> vx,vy,wz
	MatrixXf res(num_valid_range,1);
	res = A*Var - B;
	cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();

	kai_loc_level = Var;
}


void CLaserOdometry2D::Reset(CPose3D ini_pose, CObservation2DRangeScan scan)
{
	//Set the initial pose
	laser_pose = ini_pose;
	laser_oldpose = ini_pose;

	readLaser(scan);
	createImagePyramid();
	readLaser(scan);
	createImagePyramid();
}


void CLaserOdometry2D::performWarping()
{
	Matrix3f acu_trans; 
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=level; i++)
		acu_trans = transformations[i-1]*acu_trans;

	MatrixXf wacu(1,cols_i);
	wacu.assign(0.f);
	range_warped[image_level].assign(0.f);

	const float cols_lim = float(cols_i-1);
	const float kdtita = cols_lim/fovh;

	for (unsigned int j = 0; j<cols_i; j++)
	{				
		if (range[image_level](j) > 0.f)
		{
			//Transform point to the warped reference frame
			const float x_w = acu_trans(0,0)*xx[image_level](j) + acu_trans(0,1)*yy[image_level](j) + acu_trans(0,2);
			const float y_w = acu_trans(1,0)*xx[image_level](j) + acu_trans(1,1)*yy[image_level](j) + acu_trans(1,2);
			const float tita_w = atan2(y_w, x_w);
			const float range_w = sqrt(x_w*x_w + y_w*y_w);

			//Calculate warping
			const float uwarp = kdtita*(tita_w + 0.5*fovh);

			//The warped pixel (which is not integer in general) contributes to all the surrounding ones
			if (( uwarp >= 0.f)&&( uwarp < cols_lim))
			{
				const int uwarp_l = uwarp;
				const int uwarp_r = uwarp_l + 1;
				const float delta_r = float(uwarp_r) - uwarp;
				const float delta_l = uwarp - float(uwarp_l);

				//Very close pixel
				if (abs(round(uwarp) - uwarp) < 0.05f)
				{
					range_warped[image_level](round(uwarp)) += range_w;
					wacu(round(uwarp)) += 1.f;
				}
				else
				{
					const float w_r = square(delta_l);
					range_warped[image_level](uwarp_r) += w_r*range_w;
					wacu(uwarp_r) += w_r;

					const float w_l = square(delta_r);
					range_warped[image_level](uwarp_l) += w_l*range_w;
					wacu(uwarp_l) += w_l;
				}
			}
		}
	}

	//Scale the averaged range and compute coordinates
	for (unsigned int u = 0; u<cols_i; u++)
	{	
		if (wacu(u) > 0.f)
		{
			const float tita = -0.5f*fovh + float(u)/kdtita;
			range_warped[image_level](u) /= wacu(u);
			xx_warped[image_level](u) = range_warped[image_level](u)*cos(tita);
			yy_warped[image_level](u) = range_warped[image_level](u)*sin(tita);
		}
		else
		{
			range_warped[image_level](u) = 0.f;
			xx_warped[image_level](u) = 0.f;
			yy_warped[image_level](u) = 0.f;
		}
	}
}


void CLaserOdometry2D::odometryCalculation()
{
	//==================================================================================
	//						DIFERENTIAL  ODOMETRY  MULTILEVEL
	//==================================================================================

	m_clock.Tic();
	createImagePyramid();

    //Coarse-to-fine scheme
    for (unsigned int i=0; i<ctf_levels; i++)
    {
		//Previous computations
		transformations[i].setIdentity();

		level = i;
		unsigned int s = pow(2.f,int(ctf_levels-(i+1)));
        cols_i = ceil(float(cols)/float(s));
        image_level = ctf_levels - i + round(log2(round(float(width)/float(cols)))) - 1;

		//1. Perform warping
		if (i == 0)
		{
			range_warped[image_level] = range[image_level];
			xx_warped[image_level] = xx[image_level];
			yy_warped[image_level] = yy[image_level];
		}
		else
			performWarping();

		//2. Calculate inter coords
		calculateCoord();

		//3. Find null points
		findNullPoints();

		//4. Compute derivatives
		calculaterangeDerivativesSurface();

		//5. Compute normals
		//computeNormals();

		//6. Compute weights
		computeWeights();

		//7. Solve odometry
		if (num_valid_range > 3)
			solveSystemOneLevel();

		//8. Filter solution
		filterLevelSolution();

		//X. Show something
		showGraph();
	}

	m_runtime = 1000*m_clock.Tac();
	cout << endl << "Time odometry (ms): " << m_runtime;

	//Update poses
	PoseUpdate();
}


void CLaserOdometry2D::filterLevelSolution()
{
	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	SelfAdjointEigenSolver<MatrixXf> eigensolver(cov_odo);
	if (eigensolver.info() != Success) 
	{ 
		printf("Eigensolver couldn't find a solution. Pose is not updated");
		return;
	}
	
	//First, we have to describe both the new linear and angular speeds in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,3,3> Bii;
	Matrix<float,3,1> kai_b;
	Bii = eigensolver.eigenvectors();

	kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level);

	//Second, we have to describe both the old linear and angular speeds in the "eigenvector" basis too
	//-------------------------------------------------------------------------------------------------
	CMatrixFloat31 kai_loc_sub;

	//Important: we have to substract the solutions from previous levels
	Matrix3f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=0; i<level; i++)
		acu_trans = transformations[i]*acu_trans;

	kai_loc_sub(0) = -fps*acu_trans(0,2);
	kai_loc_sub(1) = -fps*acu_trans(1,2);
	if (acu_trans(0,0) > 1.f)
		kai_loc_sub(2) = 0.f;
	else
		kai_loc_sub(2) = -fps*acos(acu_trans(0,0))*sign(acu_trans(1,0));
	kai_loc_sub += kai_loc_old;

	Matrix<float,3,1> kai_b_old;
	kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

	//Filter speed
	const float cf = 15e3f*expf(-int(level)), df = 0.05f*expf(-int(level));

	Matrix<float,3,1> kai_b_fil;
	for (unsigned int i=0; i<3; i++)
	{
			kai_b_fil(i,0) = (kai_b(i,0) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i,0))/(1.f + cf*eigensolver.eigenvalues()(i,0) + df);
			//kai_b_fil_f(i,0) = (1.f*kai_b(i,0) + 0.f*kai_b_old_f(i,0))/(1.0f + 0.f);
	}

	//Transform filtered speed to local reference frame and compute transformation
	Matrix<float,3,1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

	//transformation
	const float incrx = kai_loc_fil(0)/fps;
	const float incry = kai_loc_fil(1)/fps;
	const float rot = kai_loc_fil(2)/fps;
	transformations[level](0,0) = cos(rot);
	transformations[level](0,1) = -sin(rot);
	transformations[level](1,0) = sin(rot);
	transformations[level](1,1) = cos(rot);
	transformations[level](0,2) = incrx;
	transformations[level](1,2) = incry;
}


void CLaserOdometry2D::PoseUpdate()
{
	//First, compute the overall transformation
	//---------------------------------------------------
	Matrix3f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=ctf_levels; i++)
		acu_trans = transformations[i-1]*acu_trans;


	//				Compute kai_loc and kai_abs
	//--------------------------------------------------------
	kai_loc(0) = fps*acu_trans(0,2);
	kai_loc(1) = fps*acu_trans(1,2);
	if (acu_trans(0,0) > 1.f)
		kai_loc(2) = 0.f;
	else
		kai_loc(2) = fps*acos(acu_trans(0,0))*sign(acu_trans(1,0));

	//cout << endl << "Arc cos (incr tita): " << kai_loc(2);

	float phi = laser_pose.yaw();

	kai_abs(0) = kai_loc(0)*cos(phi) - kai_loc(1)*sin(phi);
	kai_abs(1) = kai_loc(0)*sin(phi) + kai_loc(1)*cos(phi);
	kai_abs(2) = kai_loc(2);


	//						Update poses
	//-------------------------------------------------------
	laser_oldpose = laser_pose;
	math::CMatrixDouble33 aux_acu = acu_trans;
	poses::CPose2D pose_aux_2D(acu_trans(0,2), acu_trans(1,2), kai_loc(2)/fps);
	laser_pose = laser_pose + pose_aux_2D;



	//				Compute kai_loc_old
	//-------------------------------------------------------
	phi = laser_pose.yaw();
	kai_loc_old(0) = kai_abs(0)*cos(phi) + kai_abs(1)*sin(phi);
	kai_loc_old(1) = -kai_abs(0)*sin(phi) + kai_abs(1)*cos(phi);
	kai_loc_old(2) = kai_abs(2);


	// Publish estimated odometry as a mrpt::obs::CObservationOdometry
	//-----------------------------------------------------------------
	mrpt::obs::CObservationOdometryPtr odom = mrpt::obs::CObservationOdometry::Create();
	odom->sensorLabel = "ODOMETRY_LASER2D";
	odom->odometry = mrpt::poses::CPose2D(laser_pose);
	odom->timestamp = mrpt::system::now();
	odom->hasVelocities = false;
	odom->velocityLin = 0.0;
	odom->velocityAng = 0.0;
	odom->hasEncodersInfo = false;
	odom->encoderLeftTicks = 0;
	odom->encoderRightTicks = 0;
	mrpt::vector_byte vec_odom;
	mrpt::utils::ObjectToOctetVector(odom.pointer(), vec_odom);
	//! @moos_publish  ODOMETRY_OBS_LASER2D  The absolute odometry as mrpt::obs::CObservationOdometry estimated from Laser Scans
	m_Comms.Notify("ODOMETRY_OBS_LASER2D", vec_odom);
}