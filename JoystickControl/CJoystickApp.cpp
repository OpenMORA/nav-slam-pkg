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


/**  @moos_module Allow the robot to be commanded with a Joystick/Keyboard.
  */

#include "CJoystickApp.h"
#include <mrpt/system/os.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;


CJoystickApp::CJoystickApp() :
	m_maxV (0.5 ),
	m_maxW ( DEG2RAD(60) )
{
}

CJoystickApp::~CJoystickApp()
{
}

bool CJoystickApp::OnStartUp()
{
	//! @moos_param max_v  Maximum robot linear speed (m/s)
	//! @moos_param max_w  Maximum robot angularspeed (deg/s)
	m_MissionReader.GetConfigurationParam("max_v",m_maxV);
	if (m_MissionReader.GetConfigurationParam("max_w",m_maxW))
		m_maxW = DEG2RAD(m_maxW);

	// Actual speed (m/s)
	v_now = 0.0;
	Jenable = false;		//Start as inactive (for security)

	return DoRegistrations();
}

bool CJoystickApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pJoystickControl only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

short getchX()	{
	unsigned short r=static_cast<short>(mrpt::system::os::getch());
	return (r==0x00||r==0xE0)?(0x100+mrpt::system::os::getch()):r;
}

bool getKeyboardPosition(int &kx, int &ky)
{
	if ( mrpt::system::os::kbhit() )
	{
		short c = getchX();
		
		switch (c){			
			case 0x148:	//Up
				ky = 1;
				break;
			case 0x150:	//Down
				ky = -1;
				break;
			case 0x14B:	//Left
				kx = 1;				
				break;
			case 0x14D:	//Right
				kx = -1;
				break;			
			default:
				return false;
		}
		return true;

	}else
	{
		return false;
	}
	
}


bool CJoystickApp::Iterate()
{
	// Is MANUAL_MODE enabled??
	if( !Jenable )
		return true;

	//printf("[Joystick] Into Iterate\n");
	double cmd_v=0, cmd_w=0;
	// Params to read the joystick:
	float 			jx,jy,jz;
	vector_bool 	btns;
	// new
	unsigned b=0x0;
	unsigned mask = 0x1;
	vector_bool::iterator bIter;
	// /new

	//Params to read the keyboard
	int kx=0 ,ky=0;	

	//Try reading the keyboard, else the joystick
	if (getKeyboardPosition(kx,ky))
	{
		//printf("[Joystick] Got Keyboard\n");
		cmd_v = ky*m_maxV;
		cmd_w = kx*m_maxW;
	}
	else if (m_joy.getJoystickPosition(0,jx,jy,jz,btns))
	{
		//printf("[Joystick] Got Joystick\n");
		cmd_v = -jy*m_maxV;
		cmd_w = -jx*m_maxW;
		// button reading
		for(bIter=btns.begin();bIter!=btns.end();bIter++){
			if(*bIter){
				b = b|mask;
			}
			mask = mask<<1;
		}				
	}
	else
		printf("[Joystick] No Keyboard/Joystick command present. Waiting...\n");

	//Create aceleration profile
	if (abs(v_now-cmd_v)<0.001 )
		v_now = cmd_v;
	else if (v_now < cmd_v)
		v_now = v_now + m_maxV/10;
	else if (v_now > cmd_v)
		v_now = v_now - m_maxV/10;

	//cout << "V_now = " << v_now << endl;

	//!  @moos_publish   MOTION_CMD_V  The desired robot linear speed (m/s)
	//!  @moos_publish   MOTION_CMD_W  The desired robot angular speed (rad/s)
	//!  @moos_publish   BUTTONS_CMD  Byte containing the pressed buttons
	m_Comms.Notify("MOTION_CMD_V", v_now );
	m_Comms.Notify("MOTION_CMD_W", cmd_w );
	m_Comms.Notify("BUTTONS_CMD",b);

	printf("[Joystick]: Comamnd send to Base is: %.03f m/s, %03f deg/s\n", v_now, cmd_w);
    return true;
}

bool CJoystickApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CJoystickApp::DoRegistrations()
{
	//! @moos_subscribe	JOYSTICK_MODE
	AddMOOSVariable("JOYSTICK_MODE", "JOYSTICK_MODE", "JOYSTICK_MODE", 0);
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    
	RegisterMOOSVariables();
    return true;
}


bool CJoystickApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{	
		//JOYSTICK_MODE
		if( i->GetName()=="JOYSTICK_MODE" )
		{
			if( i->GetString() == "0" )
			{
				Jenable = false;
				printf("[Joystick]: Module Disabled.\n");
				//Stop the robot and set to mode manual
				//! @moos_publish CANCEL_NAVIGATION
				m_Comms.Notify("CANCEL_NAVIGATION", "Joystick - Disabled");
			}
			else if( i->GetString() == "1" )
			{
				Jenable = true;
				printf("[Joystick]: Module Enabled.\n");
			}
			else
				printf("[Joystick]: ERROR: incorrect command.\n");
		}

		//SHUTDOWN
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