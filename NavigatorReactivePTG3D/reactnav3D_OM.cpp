
#include "reactnav3D_OM.h"
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/math/ops_containers.h> // sum()
#include <mrpt/utils/CFileOutputStream.h>

//#include <mrpt/reactivenav/CHolonomicVFF.h>
#include <iostream>
#include <mrpt/utils/metaprogramming.h>

//Function which reads the geometric parameters of the robot, as well as the lasers and kinects configuration
//-----------------------------------------------------------------------------------------------------------


void CReactiveNavigator::loadRobotConfiguration(CMissionReader2ConfigFile_adaptor configRobot)
{
	try
	{
		TPoint2D coord;
		unsigned int num_levels,num_lasers,num_kinects;
		vector <float> xaux,yaux,lasercoord;

		num_levels = configRobot.read_int("pMobileRobot_Simul3D","HEIGHT_LEVELS", 1, true);
		m_robot.m_levels.resize(num_levels);
		for (unsigned int i=1;i<=num_levels;i++)
		{
			m_robot.m_levels[i-1].m_height = configRobot.read_float("",format("LEVEL%d_HEIGHT",i), 1, true);
			configRobot.read_vector("",format("LEVEL%d_VECTORX",i), vector<float> (0), xaux, false);
			configRobot.read_vector("",format("LEVEL%d_VECTORY",i), vector<float> (0), yaux, false);
			for (unsigned int j=0;j<xaux.size();j++)
			{
				coord.x = xaux[j];
				coord.y = yaux[j];
				m_robot.m_levels[i-1].m_points.push_back(coord);
			}
		}

		num_lasers = configRobot.read_int("","N_LASERS", 1, true);
		m_robot.m_lasers.resize(num_lasers);
		for (unsigned int i=1;i<=num_lasers;i++)
		{
			configRobot.read_vector("",format("LASER%d_POSE",i), vector<float> (0), lasercoord , true);
			m_robot.m_lasers[i-1].m_scan.maxRange = configRobot.read_float("",format("LASER%d_MAX_RANGE",i), 50, true);
			m_robot.m_lasers[i-1].m_scan.aperture = configRobot.read_float("",format("LASER%d_APERTURE",i), M_PI, true);
			m_robot.m_lasers[i-1].m_scan.stdError = configRobot.read_float("",format("LASER%d_STD_ERROR",i), 0.05, true);
			m_robot.m_lasers[i-1].m_scan.sensorPose.setFromValues(lasercoord[0],lasercoord[1],lasercoord[2],lasercoord[3],lasercoord[4],lasercoord[5]);
			m_robot.m_lasers[i-1].m_level = configRobot.read_int("",format("LASER%d_LEVEL",i), 1, true);
			m_robot.m_lasers[i-1].m_segments = configRobot.read_int("",format("LASER%d_SEGMENTS",i), 181, true);
		}

		num_kinects = configRobot.read_int("","N_KINECTS", 1, true );
		m_robot.m_kinects.resize(num_kinects);
		for (unsigned int i=1;i<=num_kinects;i++)
		{
			m_robot.m_kinects[i-1].m_level = configRobot.read_int("",format("KINECT%d_LEVEL",i), 1, true);
			m_robot.m_kinects[i-1].m_xrel = configRobot.read_float("",format("KINECT%d_X",i), 0, true);
			m_robot.m_kinects[i-1].m_yrel = configRobot.read_float("",format("KINECT%d_Y",i), 0, true);
			m_robot.m_kinects[i-1].m_zrel = configRobot.read_float("",format("KINECT%d_Z",i), 0, true);
			m_robot.m_kinects[i-1].m_phi = configRobot.read_float("",format("KINECT%d_PHI",i), 0, true);
			m_robot.m_kinects[i-1].m_min_range = configRobot.read_float("",format("KINECT%d_MINRANGE",i), 0, true);
			m_robot.m_kinects[i-1].m_max_range = configRobot.read_float("",format("KINECT%d_MAXRANGE",i), 0, true);
			m_robot.m_kinects[i-1].m_fov_v = DEG2RAD(configRobot.read_float("",format("KINECT%d_FOV_V",i), 60, true));
			m_robot.m_kinects[i-1].m_fov_h = DEG2RAD(configRobot.read_float("",format("KINECT%d_FOV_H",i), 60, true));
			m_robot.m_kinects[i-1].m_pitch_angle = DEG2RAD(configRobot.read_float("",format("KINECT%d_PITCH_ANGLE",i), 60, true));
			m_robot.m_kinects[i-1].m_rows = configRobot.read_int("",format("KINECT%d_ROWS",i), 10, true);
			m_robot.m_kinects[i-1].m_columns = configRobot.read_int("",format("KINECT%d_COLUMNS",i), 10, true);
			m_robot.m_kinects[i-1].m_std_error = configRobot.read_float("",format("KINECT%d_STD_ERROR",i), 0.05, true);
		}

		m_dynfeatures.pos_ini.x(configRobot.read_float("","X0", 0, true));
		m_dynfeatures.pos_ini.y(configRobot.read_float("","Y0", 0, true));
		m_dynfeatures.pos_ini.phi(DEG2RAD(configRobot.read_float("","PHI0", 0, true)));
		m_dynfeatures.robotMax_V_mps = configRobot.read_float("","VMAX_MPS", 1, true);
		m_dynfeatures.robotMax_W_degps = configRobot.read_float("","WMAX_DEGPS", 30, true);
		m_dynfeatures.ROBOTMODEL_DELAY = configRobot.read_float("","ROBOTMODEL_DELAY", 0, true);
		m_dynfeatures.ROBOTMODEL_TAU = configRobot.read_float("","ROBOTMODEL_TAU", 0, true);
		m_dynfeatures.refDistance = configRobot.read_float("","MAX_DISTANCE_PTG", 1, true);
		m_reactiveparam.WS_Target.x(configRobot.read_float("","X_TARGET", 1, true));
		m_reactiveparam.WS_Target.y(configRobot.read_float("","Y_TARGET", 1, true));
		m_reactiveparam.m_reload_ptgfiles = configRobot.read_bool("","RELOAD_PTGFILES", 1, true);
		m_reactiveparam.m_recordLogFile = configRobot.read_bool("","RECORD_LOGFILE", 0, true);

		configRobot.read_vector("","weights", vector<float> (0), m_reactiveparam.weights, 1);

		m_reactiveparam.m_holonomicmethod = configRobot.read_bool("","HOLONOMIC_METHOD", 0, true);
		unsigned num_ptg = configRobot.read_int("","PTG_COUNT", 1, true);
		m_holonomicND.resize(num_ptg);
		CConfigFileBase *configFile;
		configFile = &configRobot;

		if (m_reactiveparam.m_holonomicmethod == 0)
			m_holonomicVFF = new CHolonomicVFF(configFile);
		else
			for (unsigned i=0; i<num_ptg; i++)
				m_holonomicND[i] = new CHolonomicND(configFile);

		cout << endl << "The configuration file has been loaded successfully.";
	}
	catch( std::exception & e)
	{
		cout << e.what() << endl;
	}
}



void CReactiveNavigator::loadMapsfromimages(CMissionReader2ConfigFile_adaptor configRobot)
{
	try
	{
		COccupancyGridMap2D grid;
		CImage myImg;
		bool aux;
		int family = configRobot.read_int("","FAMILY", 1, true);
		float resolution = configRobot.read_float("","MAP_RESOLUTION", 1, true);

		for (int i=1;i<=configRobot.read_int("","NUM_MAPS", 1, true);i++)
		{
			aux = myImg.loadFromFile(format("map%d_%d.png",family,i),-1);
			grid.loadFromBitmap(myImg,resolution);
			m_maps.push_back(grid);
		}
		cout << endl << "Maps have been loaded successfully.";
	}
	catch( std::exception & e)
	{
		cout << e.what() << endl;
	}
}



void CReactiveNavigator::ShowRobotMotion ()
{
	m_scene = m_window->get3DSceneAndLock();
	CPose3D robotpose3d;
	CRenderizablePtr obj;
	robotpose3d.x(m_dynfeatures.curpose[0]);
	robotpose3d.y(m_dynfeatures.curpose[1]);
	robotpose3d.setYawPitchRoll(m_dynfeatures.curpose[2],0,0);

	//The target is updated
	{
		obj = m_scene->getByName("Target");
		obj->setPose( CPoint3D(m_reactiveparam.WS_Target[0], m_reactiveparam.WS_Target[1], 0));
		obj->setColor(0.2,0.3,0.9);
	}

	//The robot pose is updated
	{
		float h;
		for (unsigned int i=0;i<m_robot.m_levels.size();i++)
		{
			obj = m_scene->getByName(format("Level%d",i+1));

			if (i == 0) {h = 0;}
			else { h = m_robot.m_levels[i-1].m_height + h;}

			robotpose3d.z(h);
			obj->setPose(robotpose3d);
		}
	}

	//The laserscan is inserted
	{
		CPlanarLaserScanPtr lasobj;

		for (unsigned int i=0;i<m_robot.m_lasers.size();i++)
		{
			lasobj = m_scene->getByClass<CPlanarLaserScan> (i);
			lasobj->setScan(m_robot.m_lasers[i].m_scan);
			lasobj->setPose(m_dynfeatures.curpose);
		}
	}

	//The Kinectscan is inserted
	{
		robotpose3d.z(0);
		TPoint3D point;
		CPointCloudPtr obj;

		for (unsigned int i=0;i<m_robot.m_kinects.size();i++)
		{
			obj = m_scene->getByClass<CPointCloud> (i);
			obj->clear();
			obj->setPose(robotpose3d);

			for (unsigned int j=0;j<m_robot.m_kinects[i].m_points.size();j++)
			{
				m_robot.m_kinects[i].m_points.getPoint(j,point);
				obj->insertPoint(point.x,point.y,point.z);
			}
		}
	}

	m_window->unlockAccess3DScene();
	m_window->addTextMessage(5,5, format("%.02fFPS", m_window->getRenderingFPS()));
	//mrpt::system::sleep(40);
	m_window->repaint();
}




void CReactiveNavigator::InitializeScene()
{
	CPose3D robotpose3d;
	robotpose3d.x(m_dynfeatures.pos_ini[0]);
	robotpose3d.y(m_dynfeatures.pos_ini[1]);
	robotpose3d.setYawPitchRoll(m_dynfeatures.pos_ini[2],0,0);


	//The display window is created

	m_window = gui::CDisplayWindow3D::Create();
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;  //I don't know how this influences the scene...

	m_window->setWindowTitle("Reactive Navigation. Robot motion simulation");
	m_window->resize(1800,980);
	m_window->setPos(50,0);

	m_scene = m_window->get3DSceneAndLock();
	m_window->setCameraZoom(50);


	//The maps are inserted
	{
		CSetOfObjectsPtr gl_grid = CSetOfObjects::Create();
		for (unsigned int i=0;i<m_maps.size();i++)
		{
			m_maps[i].getAs3DObject(gl_grid);
			m_scene->insert(gl_grid);
		}
	}

	//A CornerXYZ object is inserted as an absolute frame of reference
	{
		CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZ();
		obj->setLocation(0,0,0);
		m_scene->insert( obj );
	}

	//A reference grid is inserted
	{
		CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-16,16,-16,16,0,1);
		obj->setColor(0.4,0.4,0.4);
		obj->setLocation(0,0,0);
		obj->setName("gridref");
		m_scene->insert( obj );

	}

	//The target is inserted
	{
		CDiskPtr obj = opengl::CDisk::Create(0.4, 0.3);
		obj->setLocation(m_reactiveparam.WS_Target[0], m_reactiveparam.WS_Target[1], 0);
		obj->setColor(0.2,0.3,0.9);
		obj->setName("Target");
		m_scene->insert( obj );
	}

	//The robot is inserted
	{
		float h;
		for (unsigned int i=0;i<m_robot.m_levels.size();i++)
		{
			if (i == 0) {h = 0;}
			else {h = m_robot.m_levels[i-1].m_height + h;}

			robotpose3d.z(h);
			CPolyhedronPtr obj;
			obj = opengl::CPolyhedron::CreateCustomPrism(m_robot.m_levels[i].m_points,m_robot.m_levels[i].m_height);
			obj->setName(format("Level%d",i+1));
			obj->setPose(robotpose3d);
			obj->setColor(0.2,0.5,0.2,1);
			obj->setWireframe(true);
			obj->setLineWidth(2);
			m_scene->insert( obj );
		}
	}

	SimulateSensors();

	//The laserscan is inserted
	{
		vector <CPlanarLaserScanPtr> gl_scan;
		CPlanarLaserScanPtr gl_scanind;

		for (unsigned int i=0;i<m_robot.m_lasers.size();i++)
		{
			gl_scan.push_back(gl_scanind);
			gl_scan[i] = CPlanarLaserScan::Create();

			gl_scan[i]->enableLine(true);
			gl_scan[i]->enableSurface(false);
			gl_scan[i]->enablePoints(true);
			gl_scan[i]->setName(format("Laser%d",i+1));
			gl_scan[i]->setScan(m_robot.m_lasers[i].m_scan);
			gl_scan[i]->setPose(m_dynfeatures.pos_ini);
			m_scene->insert(gl_scan[i]);
		}
	}

	//The Kinectscan is inserted

	{
		robotpose3d.z(0);
		TPoint3D point;
		vector <CPointCloudPtr> obj;
		CPointCloudPtr indobj;

		for (unsigned int i=0;i<m_robot.m_kinects.size();i++)
		{
			obj.push_back(indobj);
			obj[i] = opengl::CPointCloud::Create();
			obj[i]->setPose(robotpose3d);
			obj[i]->setName(format("Kinect%d",i+1));
			m_scene->insert(obj[i]);
			obj[i]->setColor(0,0,1);
			obj[i]->setPointSize(4.0);
			obj[i]->enablePointSmooth();
			for (unsigned int j=0;j<m_robot.m_kinects[i].m_points.size();j++)
			{
				m_robot.m_kinects[i].m_points.getPoint(j,point);
				obj[i]->insertPoint(point.x,point.y,point.z);
			}
		}
	}

	m_window->unlockAccess3DScene();
	m_window->addTextMessage(5,5, format("%.02fFPS", m_window->getRenderingFPS()));
	m_window->repaint();
}



void CReactiveNavigator::SimulateSensors()
{
	CPose2D robotpose2d;
	CPose3D robotpose3d;
	CPose3D kinectrelpose(0,0,0,0,0,0);
	m_robmov.getRealPose(robotpose2d);
	robotpose3d.x(robotpose2d[0]);
	robotpose3d.y(robotpose2d[1]);
	robotpose3d.z(0);
	robotpose3d.setYawPitchRoll(robotpose2d[2],0,0);

	//Laser scans

	for (unsigned int i=0;i<m_robot.m_lasers.size();i++)
	{
		m_maps[m_robot.m_lasers[i].m_level-1].laserScanSimulator( m_robot.m_lasers[i].m_scan
			, robotpose2d, 0.5f, m_robot.m_lasers[i].m_segments, m_robot.m_lasers[i].m_scan.stdError, 1, 0);
	}

	//Kinect scans

	for (unsigned int i=0;i<m_robot.m_kinects.size();i++)
	{
		kinectrelpose.x(m_robot.m_kinects[i].m_xrel);
		kinectrelpose.y(m_robot.m_kinects[i].m_yrel);
		kinectrelpose.z(m_robot.m_kinects[i].m_zrel);
		kinectrelpose.setYawPitchRoll(m_robot.m_kinects[i].m_phi, 0, 0);
		m_robot.m_kinects[i].KinectScan(m_maps, m_robot.m_levels, robotpose3d, kinectrelpose);
	}
}



void CReactiveNavigator::InitializeRobotMotion()
{
	m_robmov.resetStatus();
	m_robmov.setOdometryErrors(0);	//Set odometry errors
	m_robmov.setDelayModelParams(m_dynfeatures.ROBOTMODEL_TAU, m_dynfeatures.ROBOTMODEL_DELAY);
	m_robmov.setRealPose(m_dynfeatures.pos_ini);
	m_dynfeatures.curpose = m_dynfeatures.pos_ini;
	m_dynfeatures.curposeodo = m_dynfeatures.pos_ini;
	m_dynfeatures.last_cmd_v = 0;
	m_dynfeatures.last_cmd_w = 0;
	m_dynfeatures.new_cmd_v = 0;
	m_dynfeatures.new_cmd_w = 0;
}



void CReactiveNavigator::SimulateRobotMotion(float dt)
{
	m_robmov.movementCommand(m_dynfeatures.new_cmd_v, m_dynfeatures.new_cmd_w);
	m_robmov.simulateInterval(dt);
	m_robmov.getRealPose( m_dynfeatures.curpose);
}



void CRobotKinects::CorrectFloorPoints(CPose3D kinectrelpose)
{
	TSegment3D ray;
	TPoint3D p1,p2,pint;
	TObject3D pintobj;
	TPlane ground(0,0,1,0);
	vector <float> x, y, z;

	m_points.getAllPoints(x,y,z,1);
	p2.x = kinectrelpose[0];
	p2.y = kinectrelpose[1];
	p2.z = kinectrelpose[2];
	ray.point2 = p2;

	for (unsigned int i=0; i<m_points.size();i++)
	{
		if (z[i] < 0)
		{
			p1.x = x[i];
			p1.y = y[i];
			p1.z = z[i];
			ray.point1 = p1;
			intersect(ray,ground,pintobj);
			pintobj.getPoint(pint);
			x[i] = pint.x;
			y[i] = pint.y;
			z[i] = pint.z;
		}
	}
	m_points.setAllPoints(x,y,z);
}



void CRobotKinects::CorrectCeiling(CPose3D kinectrelpose, float height)
{
	TSegment3D ray;
	TPoint3D p1,p2,pint;
	TObject3D pintobj;
	TPlane ceiling(0,0,1,-height);
	vector <float> x, y, z;

	m_points.getAllPoints(x,y,z,1);

	p2.x = kinectrelpose[0];
	p2.y = kinectrelpose[1];
	p2.z = kinectrelpose[2];
	ray.point2 = p2;

	for (unsigned int i=0; i<m_points.size();i++)
	{
		if (z[i] > height)
		{
			p1.x = x[i];
			p1.y = y[i];
			p1.z = z[i];
			ray.point1 = p1;
			intersect(ray,ceiling,pintobj);
			pintobj.getPoint(pint);
			x[i] = pint.x;
			y[i] = pint.y;
			z[i] = pint.z;
		}
	}
	m_points.setAllPoints(x,y,z);
}


void CRobotKinects::CorrectRanges(CPose3D kinectrelpose)
{
	vector <float> x, y, z;
	vector <bool> deletion;

	m_points.getAllPoints(x,y,z,1);

	for (unsigned int i=0; i<m_points.size();i++)
	{
		if ((kinectrelpose.distance3DTo(x[i],y[i],z[i]) < m_min_range)||(kinectrelpose.distance3DTo(x[i],y[i],z[i]) > m_max_range))
		{
			deletion.push_back(1);
		}
		else
		{
			deletion.push_back(0);
		}
	}
	m_points.applyDeletionMask(deletion);
}


void CRobotKinects::KinectScan(vector <COccupancyGridMap2D> m_maps, vector <CRobotLevel> m_levels, CPose3D robotpose,CPose3D kinectrelpose)
{
	unsigned int acc_factor = max(1,mrpt::utils::round(80.0/m_columns));
	float h = 0, incrz;
	CObservation2DRangeScan m_auxlaser;
	CPose2D scanpose2d;
	TPoint3D point;
	CSimplePointsMap row_points;
	row_points.insertionOptions.minDistBetweenLaserPoints = 0;
	m_points.clear();

	scanpose2d.x(robotpose[0]);
	scanpose2d.y(robotpose[1]);
	scanpose2d.phi(robotpose[3]);
	m_auxlaser.setSensorPose(kinectrelpose);
	m_auxlaser.aperture = m_fov_h;
	//m_auxlaser.beamAperture = 0.01;  //Optional

	for (unsigned int k=0;k<m_maps.size();k++)
	{
		m_maps[k].laserScanSimulator( m_auxlaser, scanpose2d, 0.5f, acc_factor*m_columns, m_std_error, 1, 0);  //"Trick" to increase accuracy
		row_points.insertObservation( &m_auxlaser);

		for (unsigned int i=0;i<m_rows;i++)
		{
			for (unsigned int j=0;j<m_columns;j++)
			{
				if (row_points.size() > acc_factor*j)
				{
					row_points.getPoint(acc_factor*j,point);
					incrz = kinectrelpose.distance3DTo(point.x,point.y,point.z)*tan((float(i)/(m_rows-1)-0.5)*m_fov_v)*cos((float(j)/(m_columns-1)-0.5)*m_fov_h);
					point.z = point.z + incrz;
					if (m_maps.size() == 1)
					{
						m_points.insertPoint(point);
					}
					else
					{
						if (k == 0)
						{
							if (point.z < m_levels[k].m_height) {m_points.insertPoint(point);}
						}
						else if (k == m_maps.size() - 1)
						{
							if (point.z >= h) {m_points.insertPoint(point);}
						}
						else
						{
							if ((point.z >= h)&&(point.z < h + m_levels[k].m_height)) {m_points.insertPoint(point);}
						}
					}
				}
			}
		}
		row_points.clear();
		h = h + m_levels[k].m_height;
	}

	CorrectFloorPoints(kinectrelpose);
	CorrectCeiling(kinectrelpose, 3);  //Default: ceiling height = 3 meters
	CorrectRanges(kinectrelpose);
}


void CReactiveNavigator::ClassifyPointsInLevels()
{
	unsigned int cont;
	bool clasified;
	float h;
	TPoint3D paux;
	m_obstacles_inlevels.clear();
	m_obstacles_inlevels.resize(m_robot.m_levels.size());
	for (unsigned int i=0;i<m_robot.m_lasers.size();i++)
	{
		m_obstacles_inlevels[m_robot.m_lasers[i].m_level-1].insertObservation(&m_robot.m_lasers[i].m_scan);
	}
	for (unsigned int i=0;i<m_robot.m_kinects.size();i++)
	{
		for (unsigned int j=0;j<m_robot.m_kinects[i].m_points.size();j++)
		{
			m_robot.m_kinects[i].m_points.getPoint(j,paux.x,paux.y,paux.z);
			clasified  = 0;
			cont  = 0;
			h = 0;
			while (clasified == 0)
			{
				if (paux.z < 0.01)
					clasified = 1;
				else
				{
					h += m_robot.m_levels[cont].m_height;
					if (paux.z < h)
					{
						m_obstacles_inlevels[cont].insertPoint(paux.x,paux.y,paux.z);
						clasified = 1;
					}
					cont++;
					if ((cont == m_robot.m_levels.size())&&(clasified == 0))
						clasified = 1;
				}
			}
		}
	}
}



void CReactiveNavigator::build_PTG_collision_grids(
	CParameterizedTrajectoryGenerator * PT,
	const mrpt::math::CPolygon        & robotShape,
	const std::string                 & cacheFilename,
	const bool                          verbose
	)
{
	MRPT_START

	if (verbose)
		cout << endl << "[build_PTG_collision_grids] Starting... *** THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! **" << endl;

	utils::CTicTac		tictac;

	if (verbose)
		cout << "Computing collision cells for PTG '" << cacheFilename << "'...";

	ASSERT_(PT)

	tictac.Tic();

	//const size_t nPaths = PT->getAlfaValuesCount();


	// Check for collisions between the robot shape and the grid cells:
	// ----------------------------------------------------------------------------
	const size_t Ki = PT->getAlfaValuesCount();

	// Load the cached version, if possible
	if ( PT->LoadColGridsFromFile( cacheFilename, robotShape ) )
	{
		if (verbose)
			cout << "loaded from file OK" << endl;
	}
	else
	{
		// BUGFIX: In case we start reading the file and in the end detected an error,
		//         we must make sure that there's space enough for the grid:
		PT->m_collisionGrid.setSize( -PT->refDistance,PT->refDistance,-PT->refDistance,PT->refDistance,PT->m_collisionGrid.getResolution());

		const int grid_cx_max = PT->m_collisionGrid.getSizeX()-1;
		const int grid_cy_max = PT->m_collisionGrid.getSizeY()-1;

		const size_t nVerts = robotShape.verticesCount();
		std::vector<mrpt::math::TPoint2D> transf_shape(nVerts); // The robot shape at each location

		// RECOMPUTE THE COLLISION GRIDS:
		// ---------------------------------------
		for (size_t k=0;k<Ki;k++)
		{
			const size_t nPoints = PT->getPointsCountInCPath_k(k);
			ASSERT_(nPoints>1)

			for (size_t n=0;n<(nPoints-1);n++)
			{
				// Translate and rotate the robot shape at this C-Space pose:
				const double x   = PT->GetCPathPoint_x( k,n );
				const double y   = PT->GetCPathPoint_y( k,n );
				const double phi = PT->GetCPathPoint_phi( k,n );

				mrpt::math::TPoint2D bb_min(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
				mrpt::math::TPoint2D bb_max(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

				for (size_t m = 0;m<nVerts;m++)
				{
					transf_shape[m].x = x + cos(phi)*robotShape.GetVertex_x(m)-sin(phi)*robotShape.GetVertex_y(m);
					transf_shape[m].y = y + sin(phi)*robotShape.GetVertex_x(m)+cos(phi)*robotShape.GetVertex_y(m);
					mrpt::utils::keep_max( bb_max.x, transf_shape[m].x); mrpt::utils::keep_max( bb_max.y, transf_shape[m].y);
					mrpt::utils::keep_min( bb_min.x, transf_shape[m].x); mrpt::utils::keep_min( bb_min.y, transf_shape[m].y);
				}

				// Robot shape polygon:
				const mrpt::math::TPolygon2D poly(transf_shape);

				// Get the range of cells that may collide with this shape:
				const int ix_min = std::max(0,PT->m_collisionGrid.x2idx(bb_min.x)-1);
				const int iy_min = std::max(0,PT->m_collisionGrid.y2idx(bb_min.y)-1);
				const int ix_max = std::min(PT->m_collisionGrid.x2idx(bb_max.x)+1,grid_cx_max);
				const int iy_max = std::min(PT->m_collisionGrid.y2idx(bb_max.y)+1,grid_cy_max);

				for (int ix=ix_min;ix<ix_max;ix++)
				{
					const float cx = PT->m_collisionGrid.idx2x(ix);

					for (int iy=iy_min;iy<iy_max;iy++)
					{
						const float cy = PT->m_collisionGrid.idx2y(iy);

						if ( poly.contains( TPoint2D(cx,cy) ) )
						{
							// Colision!! Update cell info:
							const float d = PT->GetCPathPoint_d(k,n);
							PT->m_collisionGrid.updateCellInfo(ix  ,iy  ,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix-1,iy  ,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix  ,iy-1,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix-1,iy-1,  k,d);
						}
					}	// for iy
				}	// for ix

			} // n

			if (verbose)
				cout << k << "/" << Ki << ",";
		} // k

		if (verbose)
			cout << format("Done! [%.03f sec]\n",tictac.Tac() );

		// save it to the cache file for the next run:
		PT->SaveColGridsToFile( cacheFilename, robotShape );

	}	// "else" recompute all PTG

	MRPT_END
}



void CReactiveNavigator::PTGGridBuilder(CMissionReader2ConfigFile_adaptor configRobot)
{
	try
	{
		unsigned int num_ptgs, levels = m_robot.m_levels.size(), num_alfas;
		string prefix;
		vector <CPolygon> shape;
		vector <double> x,y;
		TParameters<double> params;
		CParameterizedTrajectoryGenerator *ptgaux;

		num_ptgs = configRobot.read_int("","PTG_COUNT", 1, true);
		params["ref_distance"] = configRobot.read_float("","MAX_DISTANCE_PTG", 1, true);
		params["resolution"] = configRobot.read_float("","GRID_RESOLUTION", 0.03, true);
		m_ptgmultilevel.resize(num_ptgs);

		//Build the polygons (robot shape)

		shape.resize(m_robot.m_levels.size());

		for (unsigned int i=0;i<m_robot.m_levels.size();i++)
		{
			x.resize(m_robot.m_levels[i].m_points.size());
			y.resize(m_robot.m_levels[i].m_points.size());
			for (unsigned int j=0;j<m_robot.m_levels[i].m_points.size();j++)
			{
				x[j] = m_robot.m_levels[i].m_points[j].x;
				y[j] = m_robot.m_levels[i].m_points[j].y;
			}
			shape[i].setAllVertices(x,y);
		}

		// Read each PTG parameters, and generate N x L collisiongrids
		//	N - Number of PTGs
		//	L - Number of levels

		for (unsigned int j=1;j<=num_ptgs;j++)
		{
			params["PTG_type"]	= configRobot.read_int("",format("PTG%d_TYPE",j),1,true);
			params["v_max"] = configRobot.read_float("",format("PTG%d_VMAX",j),1,true);
			params["w_max"] = DEG2RAD(configRobot.read_float("",format("PTG%d_WMAX",j),1,true));
			params["K"] = configRobot.read_int("",format("PTG%d_K",j),1,true);
			params["cte_a0v"] = DEG2RAD(configRobot.read_float("",format("PTG%d_AV",j),1,true));
			params["cte_a0w"] = DEG2RAD(configRobot.read_float("",format("PTG%d_AW",j),1,true));
			num_alfas = configRobot.read_int("",format("PTG%d_NALFAS",j),30,true);

			for (unsigned int i=1; i<=m_robot.m_levels.size(); i++)
			{
				ptgaux = CParameterizedTrajectoryGenerator::CreatePTG(params);
				m_ptgmultilevel[j-1].PTGs.push_back(ptgaux);
				m_ptgmultilevel[j-1].PTGs[i-1]->simulateTrajectories(num_alfas,75, m_dynfeatures.refDistance, 600, 0.01f, 0.015f);
				//Arguments -> n_alfas, max.tim, max.dist (ref_distance), max.n, diferencial_t, min_dist
				build_PTG_collision_grids(m_ptgmultilevel[j-1].PTGs[i-1], shape[i-1],
											format("ReacNavGrid_PTG%03u_L%02u.dat.gz",i,j), 1);

			}
		}
	}
	catch( std::exception & e)
	{
		cout << e.what() << endl;
	}
}


void CReactiveNavigator::ObstaclesToTPSpace()
{
	try
	{
		size_t Ki,nObs;
		float invoperation;

		for (unsigned int a=0; a<m_ptgmultilevel.size(); a++)
		{
			Ki = m_ptgmultilevel[a].PTGs[0]->getAlfaValuesCount();

			if ( m_ptgmultilevel[a].TPObstacles.size() != Ki )
				m_ptgmultilevel[a].TPObstacles.resize(Ki);


			for (unsigned int k=0;k<Ki;k++)
			{
				// Initialized at max. distance
				m_ptgmultilevel[a].TPObstacles[k] = m_ptgmultilevel[a].PTGs[0]->refDistance;

				// Si acaba girado 180deg, acabar ahi.  (Pruebo a quitarlo)
				//float phi = m_ptgmultilevel[a].PTGs[0]->GetCPathPoint_phi(k,m_ptgmultilevel[a].PTGs[0]->getPointsCountInCPath_k(k)-1);  //Orientación del último punto
				//if (fabs(phi) >= M_PI* 0.95 )
				//	m_ptgmultilevel[a].TPObstacles[k]= m_ptgmultilevel[a].PTGs[0]->GetCPathPoint_d(k,m_ptgmultilevel[a].PTGs[0]->getPointsCountInCPath_k(k)-1);
			}

			for (unsigned int j=0;j<m_robot.m_levels.size();j++)
			{
				nObs = m_obstacles_inlevels[j].size();

				for (unsigned int obs=0;obs<nObs;obs++)
				{
					float ox,oy;
					m_obstacles_inlevels[j].getPoint(obs,ox,oy);

					const CParameterizedTrajectoryGenerator::TCollisionCell &cell = m_ptgmultilevel[a].PTGs[j]->m_collisionGrid.getTPObstacle(ox,oy);

					// Keep the minimum distance:
					for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();i++)
						if ( i->second < m_ptgmultilevel[a].TPObstacles[i->first] )
							m_ptgmultilevel[a].TPObstacles[i->first] = i->second;
				}
			}

			// Distances in TP-Space are normalized to [0,1]

			invoperation = 1/m_ptgmultilevel[a].PTGs[0]->refDistance;
			for (unsigned int k=0;k<Ki;k++)
				m_ptgmultilevel[a].TPObstacles[k] *= invoperation;
		}
	}
	catch (std::exception &e)
	{
		printf("[CReactiveNavigationSystem::STEP3_SpaceTransformer] Exception:");
		printf("%s", e.what());
	}
	catch (...)
	{
		cout << "\n[CReactiveNavigationSystem::STEP3_SpaceTransformer] Unexpected exception!:\n" << endl;
	}
}



void CReactiveNavigator::ApplyHolonomicMethod()
{
	try
	{
		int k;
		float d, alfa;
		m_robmov.getRealPose(m_dynfeatures.curpose);
		m_reactiveparam.rel_Target = m_reactiveparam.WS_Target - m_dynfeatures.curpose;

		for (unsigned int i=0; i<m_ptgmultilevel.size();i++)
		{
			m_ptgmultilevel[i].PTGs[0]->inverseMap_WS2TP(m_reactiveparam.rel_Target[0], m_reactiveparam.rel_Target[1], k, d);
			alfa = m_ptgmultilevel[i].PTGs[0]->index2alpha(k);
			m_ptgmultilevel[i].TP_Target[0] = cos(alfa)*d;
			m_ptgmultilevel[i].TP_Target[1] = sin(alfa)*d;
		}

		switch (m_reactiveparam.m_holonomicmethod)
		{
		case 0:
		{
			for (unsigned int i=0;i<m_ptgmultilevel.size();i++)
				m_holonomicVFF->navigate( m_ptgmultilevel[i].TP_Target, m_ptgmultilevel[i].TPObstacles, m_dynfeatures.robotMax_V_mps,
										m_ptgmultilevel[i].holonomicmov.direction, m_ptgmultilevel[i].holonomicmov.speed, this->m_HLFRs[i]);
			break;
		}

		case 1:
			{
			for (unsigned int i=0;i<m_ptgmultilevel.size();i++)
				m_holonomicND[i]->navigate( m_ptgmultilevel[i].TP_Target, m_ptgmultilevel[i].TPObstacles, m_dynfeatures.robotMax_V_mps,
										  m_ptgmultilevel[i].holonomicmov.direction, m_ptgmultilevel[i].holonomicmov.speed, this->m_HLFRs[i]);
			break;
			}
		};

	}
	catch (std::exception &e)
	{
		printf("[CReactiveNavigationSystem::STEP4_HolonomicMethod] Exception:");
		printf("%s",e.what());
	}
	catch (...)
	{
		printf("[CReactiveNavigationSystem::STEP4_HolonomicMethod] Unexpected exception!\n");
	}
}



void CReactiveNavigator::PTG_Evaluator(
    THolonomicMovement				&in_holonomicMovement,
    std::vector<float>				&in_TPObstacles,
    const mrpt::poses::CPoint2D		&rel_Target,
    const mrpt::poses::CPoint2D		&TP_Target,
    CLogFileRecord::TInfoPerPTG		&log)
{
	try
	{
		float	a;
		float	factor1, factor2,factor3,factor4,factor5,factor6;

		if (TP_Target.x()!=0 || TP_Target.y()!=0)
			a = atan2( TP_Target.y(), TP_Target.x());
		else	a = 0;


		const int		TargetSector = in_holonomicMovement.PTG->alpha2index( a );
		const double	TargetDist = TP_Target.norm();
		const int		kDirection = in_holonomicMovement.PTG->alpha2index( in_holonomicMovement.direction );
		const double	refDist	   = in_holonomicMovement.PTG->refDistance;


		// Coordinates of the trajectory end for the given PTG and "alpha":
		float	x,y,phi,t,d;
		d = std::min( in_TPObstacles[ kDirection ], (float)(0.90f*TargetDist) );
		in_holonomicMovement.PTG->getCPointWhen_d_Is( d, kDirection,x,y,phi,t );


		// Factor 1: Free distance for the chosen PTG and "alpha" in the TP-Space:
		// ----------------------------------------------------------------------
		factor1 = in_TPObstacles[kDirection];


		// Factor 2: Distance in sectors:
		// -------------------------------------------
		float   dif = fabs(((float)( TargetSector - kDirection )));
		float	nSectors = (float)in_TPObstacles.size();
		if ( dif > (0.5f*nSectors)) dif = nSectors - dif;
		factor2= exp(-square( dif / (in_TPObstacles.size()/3.0f))) ;


		// Factor 3: Angle between the robot at the end of the chosen trajectory and the target
		// -------------------------------------------------------------------------------------
		float   t_ang = atan2( rel_Target.y() - y, rel_Target.x() - x );
		t_ang -= phi;

		while (t_ang> M_PI)  t_ang-=(float)M_2PI;
		while (t_ang<-M_PI)  t_ang+=(float)M_2PI;

		factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );


		// Factor4:		Decrease in euclidean distance between (x,y) and the target:
		//  Moving away of the target is negatively valued
		// ---------------------------------------------------------------------------
		float dist_eucl_final = rel_Target.distance2DTo(x,y);
		float dist_eucl_now   = rel_Target.norm();

		factor4 = min(2.0*refDist,max(0.0,((dist_eucl_now - dist_eucl_final)+refDist)))/(2*refDist);

		// ---------
		//	float decrementDistanc = dist_eucl_now - dist_eucl_final;
		//	if (dist_eucl_now>0)
		//			factor4 = min(1.0,min(refDist*2,max(0,decrementDistanc + refDist)) / dist_eucl_now);
		//	else	factor4 = 0;
		// ---------
		//	factor4 = min(2*refDist2,max(0,decrementDistanc + refDist2)) / (2*refDist2);
		//  factor4=  (refDist2 - min( refDist2, dist_eucl ) ) / refDist2;


		// Factor5: Histeresis:
		// -----------------------------------------------------
		float	want_v,want_w;
		in_holonomicMovement.PTG->directionToMotionCommand( kDirection, want_v, want_w);

		float	likely_v = exp( -fabs(want_v-m_dynfeatures.last_cmd_v)/0.10f );
		float	likely_w = exp( -fabs(want_w-m_dynfeatures.last_cmd_w)/0.40f );

		factor5 = min( likely_v,likely_w );



		// Factor6: Fast when free space
		// -----------------------------------------------------
		float aver_obs = 0;
		for (unsigned int i=0; i<in_TPObstacles.size(); i++)
			aver_obs += in_TPObstacles[i];

		aver_obs = aver_obs/in_TPObstacles.size();

		factor6 = aver_obs*want_v;




		// --------------------
		//  SAVE LOG
		// --------------------
		log.evalFactors.resize(6);
		log.evalFactors[0] = factor1;
		log.evalFactors[1] = factor2;
		log.evalFactors[2] = factor3;
		log.evalFactors[3] = factor4;
		log.evalFactors[4] = factor5;
		log.evalFactors[5] = factor6;


		if (in_holonomicMovement.speed == 0)
		{
			// If no movement has been found -> the worst evaluation:
			in_holonomicMovement.evaluation = 0;
		}
		else
		{
			// Sum: two cases:
			if (dif<2	&&											// Heading the target
			        in_TPObstacles[kDirection]*0.95f>TargetDist) 	// and free space towards the target
			{
				//	Direct path to target:
				//	in_holonomicMovement.evaluation = 1.0f + (1 - TargetDist) + factor5 * weight5 + factor6*weight6;
				in_holonomicMovement.evaluation = 1.0f + (1 - t/15.0f) + factor5 * m_reactiveparam.weights[4] + factor6*m_reactiveparam.weights[5];
			}
			else
			{
				// General case:
				in_holonomicMovement.evaluation = (
				                                      factor1 * m_reactiveparam.weights[0] +
				                                      factor2 * m_reactiveparam.weights[1] +
				                                      factor3 * m_reactiveparam.weights[2] +
				                                      factor4 * m_reactiveparam.weights[3] +
				                                      factor5 * m_reactiveparam.weights[4] +
				                                      factor6 * m_reactiveparam.weights[5]
				                                  ) / ( math::sum(m_reactiveparam.weights));
			}
		}

	}
	catch (std::exception &e)
	{
		THROW_STACKED_EXCEPTION(e);
	}
	catch (...)
	{
		std::cout << "[CReactiveNavigationSystem::STEP5_Evaluator] Unexpected exception!:\n";
	}
}


void CReactiveNavigator::EvaluateAllPTGs()
{
	for (unsigned int i=0; i<m_ptgmultilevel.size();i++)
	{
		m_ptgmultilevel[i].holonomicmov.PTG = m_ptgmultilevel[i].PTGs[0];
		PTG_Evaluator(m_ptgmultilevel[i].holonomicmov,m_ptgmultilevel[i].TPObstacles,
			m_reactiveparam.rel_Target, CPoint2D(m_ptgmultilevel[i].TP_Target), this->m_newLogRec.infoPerPTG[i]);
	}
}


void CReactiveNavigator::PTG_Selector()
{
	// If nothing better is found, there isn't any possible movement:
	m_reactiveparam.m_movselected.direction= 0;
	m_reactiveparam.m_movselected.speed = 0;
	m_reactiveparam.m_movselected.PTG = NULL;
	m_reactiveparam.m_movselected.evaluation= 0;
	m_reactiveparam.m_ptgselected = 0;

	// Choose the best evaluated trajectory:
	for (unsigned int i=0;i<m_ptgmultilevel.size();i++)
	{
		float ev = m_ptgmultilevel[i].holonomicmov.evaluation;
		if ( ev > m_reactiveparam.m_movselected.evaluation )
		{
			m_reactiveparam.m_movselected = m_ptgmultilevel[i].holonomicmov;
			m_reactiveparam.m_movselected.evaluation = ev;
			m_reactiveparam.m_ptgselected = i;		//It starts at 0
		}
	}
}


void CReactiveNavigator::GenerateSpeedCommands()
{
	const float r = 0.1;

	m_dynfeatures.last_cmd_v = m_dynfeatures.new_cmd_v;
	m_dynfeatures.last_cmd_w = m_dynfeatures.new_cmd_w;

	if (m_reactiveparam.m_movselected.speed == 0)
	{
		// The robot will stop:
		m_dynfeatures.new_cmd_v = m_dynfeatures.new_cmd_w = 0;
	}
	else
	{
		// Take the normalized movement command:
		m_reactiveparam.m_movselected.PTG->directionToMotionCommand(
			m_reactiveparam.m_movselected.PTG->alpha2index( m_reactiveparam.m_movselected.direction),
			m_dynfeatures.new_cmd_v, m_dynfeatures.new_cmd_w );

		// Scale holonomic speeds to real-world one:
		//cout << endl << m_reactiveparam.m_movselected.PTG->getMax_V_inTPSpace(); ¿Qué pasa con esto?
		const double reduction = min(1.0, m_reactiveparam.m_movselected.speed / sqrt( square(m_dynfeatures.new_cmd_v) + square(r*m_dynfeatures.new_cmd_w)));

		// To scale:
		m_dynfeatures.new_cmd_v*=reduction;
		m_dynfeatures.new_cmd_w*=reduction;

		// Assure maximum speeds:
		if (fabs(m_dynfeatures.new_cmd_v)>m_dynfeatures.robotMax_V_mps)
		{
			// Scale:
			float F = fabs(m_dynfeatures.robotMax_V_mps / m_dynfeatures.new_cmd_v);
			m_dynfeatures.new_cmd_v *= F;
			m_dynfeatures.new_cmd_w *= F;
		}

		if (fabs(m_dynfeatures.new_cmd_w)>DEG2RAD(m_dynfeatures.robotMax_W_degps))
		{
			// Scale:
			float F = fabs((float)DEG2RAD(m_dynfeatures.robotMax_W_degps) / m_dynfeatures.new_cmd_w);
			m_dynfeatures.new_cmd_v *= F;
			m_dynfeatures.new_cmd_w *= F;
		}
	}
}


void CReactiveNavigator::EnableLogFile(CStream *&logFile)
{
	try
	{
		bool enable = m_reactiveparam.m_recordLogFile;

		// Disable:
		// -------------------------------
		if (!enable)
		{
			if (logFile)
			{
				printf("[CReactiveNavigationSystem::enableLogFile] Stopping logging.\n");
				// Close file:
				delete logFile;
				logFile = NULL;
			}
			else return;	// Already disabled.
		}
		else
		{	// Enable
			// -------------------------------
			if (logFile) return; // Already enabled:

			// Open file, find the first free file-name.
			char	aux[100];
			int     nFichero = 0;
			bool    nombre_libre = false;

			system::createDirectory("./reactivenav.logs");

			while (!nombre_libre)
			{
				nFichero++;
				sprintf(aux, "./reactivenav.logs/log_%03u.reactivenavlog", nFichero );
				nombre_libre = !system::fileExists(aux);
			}

			// Open log file:
			logFile = new CFileOutputStream(aux);

			printf("\n[CReactiveNavigationSystem::enableLogFile] Logging to file:");
			printf("%s",aux);
			printf("\n");
		}
	} catch (...) {
		printf("[CReactiveNavigationSystem::enableLogFile] Exception!!\n");
	}
}


void CReactiveNavigator::SaveInLogFile( CStream *logFile )
{
	CSimplePointsMap auxpointsmap;

	for (unsigned int i=0; i < m_obstacles_inlevels.size(); i++)
		auxpointsmap += *m_obstacles_inlevels[i].getAsSimplePointsMap();

	this->m_newLogRec.WS_Obstacles				= auxpointsmap;
	this->m_newLogRec.robotOdometryPose			= m_dynfeatures.curpose;
	this->m_newLogRec.WS_target_relative		= m_reactiveparam.rel_Target;
	this->m_newLogRec.v							= m_dynfeatures.new_cmd_v;
	this->m_newLogRec.w							= m_dynfeatures.new_cmd_w;
	this->m_newLogRec.nSelectedPTG				= m_reactiveparam.m_ptgselected;
	this->m_newLogRec.executionTime				= 0;
	this->m_newLogRec.actual_v					= m_robmov.getV();
	this->m_newLogRec.actual_w					= m_robmov.getW();
	//this->m_newLogRec.estimatedExecutionPeriod = ...;
	this->m_newLogRec.timestamp					= 0;
	this->m_newLogRec.nPTGs						= m_ptgmultilevel.size();
	this->m_newLogRec.navigatorBehavior			= m_reactiveparam.m_holonomicmethod;  // 0 - VFF, 1 - ND


	//Polygons of each height level are drawn (but they are shown connected...)
	if (this->m_newLogRec.robotShape_x.size() == 0)
	{
		size_t nVerts = 0;
		TPoint2D paux;
		size_t cuenta = 0;
		for (unsigned int i=0; i < m_robot.m_levels.size(); i++)
			nVerts += m_robot.m_levels[i].m_points.size()+1;
		if (size_t(this->m_newLogRec.robotShape_x.size()) != nVerts)
		{
			this->m_newLogRec.robotShape_x.resize(nVerts);
			this->m_newLogRec.robotShape_y.resize(nVerts);
		}
		for (unsigned int i=0; i < m_robot.m_levels.size(); i++)
		{
			for (unsigned int j=0; j < m_robot.m_levels[i].m_points.size(); j++)
			{
				paux = m_robot.m_levels[i].m_points[j];
				this->m_newLogRec.robotShape_x[cuenta]= paux.x;
				this->m_newLogRec.robotShape_y[cuenta]= paux.y;
				cuenta++;
			}
			paux = m_robot.m_levels[i].m_points[0];
			this->m_newLogRec.robotShape_x[cuenta]= paux.x;
			this->m_newLogRec.robotShape_y[cuenta]= paux.y;
			cuenta++;
		}
	}

	size_t sizeobs;

	// For each PTG:
	for (size_t i=0;i<m_ptgmultilevel.size();i++)
	{
		sizeobs = m_ptgmultilevel[i].TPObstacles.size();
		mrpt::utils::metaprogramming::copy_container_typecasting(m_ptgmultilevel[i].TPObstacles, this->m_newLogRec.infoPerPTG[i].TP_Obstacles);
		this->m_newLogRec.infoPerPTG[i].PTG_desc					= m_ptgmultilevel[i].PTGs[0]->getDescription();
		this->m_newLogRec.infoPerPTG[i].TP_Target					= m_ptgmultilevel[i].TP_Target;
		this->m_newLogRec.infoPerPTG[i].timeForTPObsTransformation	= 0;  //times_TP_transformations[indexPTG];
		this->m_newLogRec.infoPerPTG[i].timeForHolonomicMethod		= 0;  //times_HoloNav[indexPTG];
		this->m_newLogRec.infoPerPTG[i].HLFR						= this->m_HLFRs[i];
		this->m_newLogRec.infoPerPTG[i].desiredDirection			= m_ptgmultilevel[i].holonomicmov.direction;
		this->m_newLogRec.infoPerPTG[i].desiredSpeed				= m_ptgmultilevel[i].holonomicmov.speed;
		this->m_newLogRec.infoPerPTG[i].evaluation					= m_ptgmultilevel[i].holonomicmov.evaluation;

		this->m_HLFRs[i].clear_unique();
	}

	if (logFile)
	{
		(*logFile) << this->m_newLogRec;
	}
}


//CReactiveNavigator::CReactiveNavigator ()
//{
//	configRobot = NULL;
//}


//CRobotShape::CRobotShape()
//{
//	loadRobotConfiguration();
//}
