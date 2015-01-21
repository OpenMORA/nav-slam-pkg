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


/**  @moos_module Monte-Carlo localization for a mobile robot within a known map.
  *    This module takes a metric map (e.g. grid map) and perform global localization
  *    and pose tracking of the robot using an adaptive number of particles.
  */

#include "CPFLocalizationApp.h"
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/random.h>
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::obs;


CPFLocalizationApp::CPFLocalizationApp() :
	m_firstOdo(true),
	m_unif_x_min(-4), m_unif_x_max(-4), m_unif_y_min(-4), m_unif_y_max(4)
{
}

CPFLocalizationApp::~CPFLocalizationApp()
{
}


bool CPFLocalizationApp::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{
		// Load config from .moos mission file:
		string  sSimplemapFil;
		string  sGridmapFil;

		//! @moos_param simplemap_file A .simplemap to load the map from.
		if (m_MissionReader.GetConfigurationParam("simplemap_file",sSimplemapFil))
		{
			TSetOfMetricMapInitializers	mapList;
			mapList.loadFromConfigFile(m_ini,"MetricMap");

			CMultiMetricMap	 metricMap;
			metricMap.setListOfMaps( &mapList );

			printf("Loading '.simplemap' file...");
			CSimpleMap simpleMap;
			CFileGZInputStream(sSimplemapFil) >> simpleMap;
			printf("Ok (%u poses)\n",(unsigned)simpleMap.size());

			ASSERT_( simpleMap.size()>0 );

			// Build metric map:
			// ------------------------------
			printf("Building metric map(s) from '.simplemap'...");
			metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
			printf("Ok\n");

			ASSERTMSG_(!metricMap.m_gridMaps.empty(),"The simplemap file has no gridmap!");

			m_gridmap = *metricMap.m_gridMaps[0];
		}
		//! @moos_param gridmap_image_file An image to load a gridmap from.
		else if (m_MissionReader.GetConfigurationParam("gridmap_image_file",sGridmapFil))
		{
			double grid_res = 0.10;
			double grid_cx = -1;
			double grid_cy = -1;

			//! @moos_param gridmap_image_res When using gridmap_image_file, the size in meters of each pixel.
			//! @moos_param gridmap_image_cx When using gridmap_image_file, the X pixel where the (0,0) origin is at.
			//! @moos_param gridmap_image_cy When using gridmap_image_file, the Y pixel where the (0,0) origin is at.
			m_MissionReader.GetConfigurationParam("gridmap_image_res",grid_res);
			m_MissionReader.GetConfigurationParam("gridmap_image_cx",grid_cx);
			m_MissionReader.GetConfigurationParam("gridmap_image_cy",grid_cy);

			m_gridmap.loadFromBitmapFile(sGridmapFil,grid_res,grid_cx,grid_cy);
		}
		else
			return MOOSFail("Neither 'simplemap_file' or 'gridmap_image_file' found in mission file. Quitting.");


		// Load uniform distribution limits:
		string   x_min="-1";
		string   x_max="1";
		string   y_min="-1";
		string   y_max= "1";

		//! @moos_param X_MIN The rectangle where a uniform pdf of particles is initialized for global localization.
		//! @moos_param X_MAX The rectangle where a uniform pdf of particles is initialized for global localization.
		//! @moos_param Y_MIN The rectangle where a uniform pdf of particles is initialized for global localization.
		//! @moos_param Y_MAX The rectangle where a uniform pdf of particles is initialized for global localization.

		m_MissionReader.GetConfigurationParam("X_MIN",x_min);
		m_MissionReader.GetConfigurationParam("X_MAX",x_max);
		m_MissionReader.GetConfigurationParam("Y_MIN",y_min);
		m_MissionReader.GetConfigurationParam("Y_MAX",y_max);

		m_MissionReader.GetConfigurationParam("Likelihood_threshold", m_reloc_threshold);

		m_unif_x_min = atof(x_min.c_str());
		m_unif_x_max = atof(x_max.c_str());
		m_unif_y_min = atof(y_min.c_str());
		m_unif_y_max = atof(y_max.c_str());


		OnPFReset();

		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CPFLocalizationApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pLocalizationPF only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

//    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

	//!  @moos_cmd   RESTART   Reset the particle filter to its initial state.
    if(MOOSStrCmp(sCmd,"RESTART"))
    {
        OnPFReset();
    }
    return true;
}

// Reset the PF according to the initial distribution:
bool CPFLocalizationApp::OnPFReset()
{
	// Load parameters from moos config block:
	try
	{
		m_firstOdo = true;

		// Publish our used map so the GUIs, etc... know about it:
		// -----------------------------------------------------------
		try
		{
			const string sTmpGridmap = mrpt::system::getTempFileName();
			m_lstTempFilesToDeleteAtExit.push_back(sTmpGridmap);

			CFileGZOutputStream(sTmpGridmap) << m_gridmap;

			//! @moos_publish	CURRENT_MAP_GRIDMAP   A temporary file where the current grid map has been serialized to.
			m_Comms.Notify("CURRENT_MAP_GRIDMAP", sTmpGridmap );
		}
		catch (std::exception &e)
		{
			MOOSTrace("ERROR creating CURRENT_MAP_GRIDMAP!: %s\n",e.what());
		}


		// Create the PF object:
		// --------------------------
		bayes::CParticleFilter::TParticleFilterOptions		PF_options;
		PF_options.adaptiveSampleSize = true;
		PF_options.PF_algorithm = bayes::CParticleFilter::pfStandardProposal;

		// Link the particles to its map:
		m_PDF.options.metricMap = & m_gridmap;

		PF_options.loadFromConfigFile( m_ini, "PF_OPTIONS" );

		m_PF.m_options = PF_options;

		// Draw initial particles:
		// -----------------------------------

		mrpt::random::Randomize( static_cast<long>( MOOSTime() ) );

		m_PDF.resetUniformFreeSpace(
			&m_gridmap,
			0.6,		// freeCellsThreshold
			PF_options.sampleSize,
			m_unif_x_min,m_unif_x_max, m_unif_y_min, m_unif_y_max );

		// Prepare other options:
		// ---------------------------------
		m_motionModel.modelSelection = CActionRobotMovement2D::mmGaussian;
		m_motionModel.gausianModel.minStdXY = 0.02;	

		m_PDF.options.KLD_params.KLD_binSize_PHI= MOOSDeg2Rad( 2.5 );
		m_PDF.options.KLD_params.KLD_binSize_XY=0.07;
		m_PDF.options.KLD_params.KLD_delta=0.01;
		m_PDF.options.KLD_params.KLD_epsilon=0.01;
		m_MissionReader.GetConfigurationParam("sampleSize",m_PDF.options.KLD_params.KLD_maxSampleSize);
		m_MissionReader.GetConfigurationParam("min_sampleSize",m_PDF.options.KLD_params.KLD_minSampleSize);


		m_gridmap.likelihoodOptions.likelihoodMethod= COccupancyGridMap2D::lmLikelihoodField_Thrun;
		m_gridmap.likelihoodOptions.LF_decimation=10;
		m_gridmap.likelihoodOptions.LF_stdHit=0.20;
		m_gridmap.likelihoodOptions.LF_maxCorrsDistance=0.30;
		m_gridmap.likelihoodOptions.LF_zHit=0.95;
		m_gridmap.likelihoodOptions.LF_zRandom=0.05;
		m_gridmap.likelihoodOptions.LF_maxRange=80;
		m_gridmap.likelihoodOptions.LF_alternateAverageMethod=false;

	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}

    return true;
}



bool CPFLocalizationApp::Iterate()
{
	try
	{
		if(!ProcessParticleFilter())
			return MOOSFail("failed to process PF");

		if(!PublishPFLocalization())
			return MOOSFail("failed to publish PF data");

		return true;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}

	
}


bool CPFLocalizationApp::PublishPFLocalization()
{
	mrpt::poses::CPose2D  			poseMean;
	mrpt::math::CMatrixDouble33 	poseCov;
	m_PDF.getCovarianceAndMean(poseCov,poseMean);

	//! @moos_publish	EST_GOODNESS   The goodness of the robot pose estimation
    m_Comms.Notify("EST_GOODNESS", m_pose_goodness );

	//! @moos_publish	ODO_REFERENCE   The last odometry used to estimate the pose
	string odoRef;
	m_lastOdo.asString( odoRef );
	m_Comms.Notify("ODO_REFERENCE", odoRef);

	//! @moos_publish	LOCALIZATION_PF   The robot estimated pose in format "[x y phi]"
	string sPose;
	poseMean.asString(sPose);
    m_Comms.Notify("LOCALIZATION_PF", sPose );

	//! @moos_publish	LOCALIZATION_COV_PF  The robot estimated pose uncertainty, as a 3x3 covariance matrix for [x y yaw]
	string sPoseCov = poseCov.inMatlabFormat();
    m_Comms.Notify("LOCALIZATION_COV_PF", sPoseCov );

    // Publish raw particles:
    // ----------------------------
	stringstream	s;
	const size_t N = m_PDF.size();

	size_t decim = 1;
	size_t Nmod = N;
	if (N>300)
	{
		decim = 1;
		Nmod = 300;
	}

	s << "[";
	s.setf(std::ios::fixed);
	s.precision(3);

	for (size_t i=0;i<Nmod;i+=decim)
	{
		if (i>0) s << ";";

		s << m_PDF.m_particles[i].d->x() << " "
		  << m_PDF.m_particles[i].d->y() << " "
		  << m_PDF.m_particles[i].d->phi();
	}
	s << "]";

	//! @moos_publish	LOCALIZATION_PARTICLES   A matrix [x y phi;...] with a few of the particles used in PF localization.
	m_Comms.Notify("LOCALIZATION_PARTICLES", s.str() );

    return true;
}



bool CPFLocalizationApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CPFLocalizationApp::DoRegistrations()
{
	//! @moos_subscribe ODOMETRY, LASER1, LASER2, LASER3

	AddMOOSVariable( "ODOMETRY",  "ODOMETRY","ODOMETRY", 0 );
    AddMOOSVariable( "LASER1",  "LASER1","LASER1", 0 );
    AddMOOSVariable( "LASER2",  "LASER2","LASER2", 0 );

	//! @moos_subscribe RELOCALIZE_IN_AREA
    AddMOOSVariable( "RELOCALIZE_IN_AREA", "RELOCALIZE_IN_AREA", "RELOCALIZE_IN_AREA", 0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

    RegisterMOOSVariables();

    return true;
}


bool CPFLocalizationApp::OnNewMail(MOOSMSG_LIST &NewMail)
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


bool CPFLocalizationApp::ProcessParticleFilter()
{
	try
	{
		mrpt::poses::CPose2D			 odometryIncrement(0,0,0);

		// RELOCALIZE_IN_AREA ?
		CMOOSVariable * pVarReloc = GetMOOSVar( "RELOCALIZE_IN_AREA" );
		if(pVarReloc && pVarReloc->IsFresh())
		{
			pVarReloc->SetFresh(false);
			mrpt::math::CMatrixDouble M;
			if ( M.fromMatlabStringFormat( pVarReloc->GetStringVal()) && size(M,1)>=1 && size(M,2)>=5 )
			{
				const unsigned int nParts = M(0,4);
				this->SendSystemLogString(format("Relocalizing with %u samples in [%.1f,%.1f]-[%.1f,%.1f].", nParts,
													0.5*(M(0,0)+M(0,1)) + m_unif_x_min,
													0.5*(M(0,0)+M(0,1)) + m_unif_x_max,
													0.5*(M(0,2)+M(0,3)) + m_unif_y_min,
													0.5*(M(0,2)+M(0,3))	+ m_unif_y_max  ));

				m_PDF.resetUniformFreeSpace(
					&m_gridmap,
					0.6,		// freeCellsThreshold
					nParts,
					0.5*(M(0,0)+M(0,1)) + m_unif_x_min,
					0.5*(M(0,0)+M(0,1)) + m_unif_x_max,
					0.5*(M(0,2)+M(0,3)) + m_unif_y_min,
					0.5*(M(0,2)+M(0,3))	+ m_unif_y_max  );

			}
		}

		// Odometry:
		CMOOSVariable * pVarOdo = GetMOOSVar( "ODOMETRY" );
		if(pVarOdo && pVarOdo->IsFresh())
		{
			pVarOdo->SetFresh(false);
			mrpt::poses::CPose2D  cur_odo;
			cur_odo.fromString(pVarOdo->GetStringVal());

			if (!m_firstOdo)
			{
				odometryIncrement = cur_odo-m_lastOdo;
			}
			m_lastOdo = cur_odo;
			m_firstOdo=false;

		}

		// If the robot didn't move, do not process all the time (bad for particles!)
		static int countRobotStill = 0;
		if ( fabs(odometryIncrement.x())<1e-3 && fabs(odometryIncrement.y())<1e-3  && fabs(odometryIncrement.phi())<1e-4 )
		{
			if ((countRobotStill++ % 10) != 0 )
				return true;
		}

		// Now the laser:
		CSensoryFrame	observations;

		CMOOSVariable * pVarLaser1 = GetMOOSVar( "LASER1" );
		if(pVarLaser1 && pVarLaser1->IsFresh())
		{
			pVarLaser1->SetFresh(false);
			CSerializablePtr obj;
			//StringToObject(pVarLaser1->GetStringVal(),obj);	(deprecated) CGenericSensor now employs ObjectToOctetVector() to serialize objects to strings
			mrpt::utils::RawStringToObject(pVarLaser1->GetStringRef(), obj);

			if (obj && IS_CLASS(obj,CObservation2DRangeScan))
			{
				CObservation2DRangeScanPtr scan = CObservation2DRangeScanPtr(obj);
				observations.insert(scan);  // It will free the memory
			}
		}

		CMOOSVariable *pVarLaser2 = GetMOOSVar( "LASER2" );
		if(pVarLaser2 && pVarLaser2->IsFresh())
		{
			pVarLaser2->SetFresh(false);
			CSerializablePtr obj;			
			mrpt::utils::RawStringToObject(pVarLaser2->GetStringRef(), obj);

			if (obj && IS_CLASS(obj,CObservation2DRangeScan))
			{
				CObservation2DRangeScanPtr scan = CObservation2DRangeScanPtr(obj);
				observations.insert(scan);  // It will free the memory
			}
		}

		// ---------------------
		//   Execute PF
		// ---------------------
		CActionCollection	action;
		CActionRobotMovement2D	act;
		act.computeFromOdometry( odometryIncrement, m_motionModel );
		action.insert( act );

		double t0 = MOOSTime();
		bayes::CParticleFilter::TParticleFilterStats stats;
		m_PF.executeOn(
			m_PDF,			// The PDF
			&action,		// Action
			&observations,	// Obs
			&stats);
			
		double At = MOOSTime() - t0;

		mrpt::math::CMatrixDouble cov;
		m_PDF.getCovariance(cov);

		cout << endl << "Current odometry: " << m_lastOdo << endl;
		printf("[PF] Takes %.03fms | ESS:%f%% | %u samples \n", At*1000, 100*m_PDF.ESS(), static_cast<unsigned>( m_PDF.size() ) );
		cout << "[PF] mean: " << m_PDF.getMeanVal() << " ESS " << stats.ESS_beforeResample << endl;
		cout << "[PF] covariance norm: " << cov.norm() << endl;

		mrpt::poses::CPose2D last_pose = m_PDF.getMeanVal();
		//The particle considered doesn't seem to affect the value of "likelihood"...
		m_pose_goodness = m_PDF.PF_SLAM_computeObservationLikelihoodForParticle(m_PF.m_options, 0, observations, mrpt::poses::CPose3D(last_pose[0],last_pose[1],0,last_pose[2],0,0));
		cout << "[PF] The goodness of the estimated pose is " << m_pose_goodness << endl;
		static double reloc_time;
		
		if ((m_PDF.ESS()<m_PF.m_options.BETA || m_pose_goodness < m_reloc_threshold) && (MOOSTime()-reloc_time)>3)  //relocalization can only be made after 3 seconds from the last reloc. operation
		{
			cout << endl << endl << "Relocalizing..." << endl << endl;
			double reloc_x_min = last_pose[0] + m_unif_x_min;
			double reloc_x_max = last_pose[0] + m_unif_x_max;
			double reloc_y_min = last_pose[1] + m_unif_y_min; 
			double reloc_y_max = last_pose[1] + m_unif_y_max;

			cout << endl << "It's going to be relocalized in [" << reloc_x_min << ", " << reloc_x_max << ", " << reloc_y_min << ", " << reloc_y_max << "]";
		
			m_PDF.resetUniformFreeSpace(
						&m_gridmap,
						0.6,		// freeCellsThreshold
						20000,
						reloc_x_min,reloc_x_max, reloc_y_min, reloc_y_max );
			this->SendSystemLogString(format("I was almost lost: Relocalizing with %u samples in [%.1f,%.1f]-[%.1f,%.1f].",20000, 
				reloc_x_min,reloc_x_max,reloc_y_min,reloc_y_max ));
			reloc_time=MOOSTime();
		}

		//Localize the best particle
		//size_t index_bestparticle;
		//double max_weight = 0;

		//for (unsigned int i=0; i<m_PDF.particlesCount(); i++)
		//{
		//	if (m_PDF.getW(i) > max_weight)
		//	{
		//		max_weight = m_PDF.getW(i);
		//		index_bestparticle = i;
		//	}
		//}

		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail(e.what() );
	}
}
