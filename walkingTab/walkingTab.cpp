/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "walkingTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <iostream>

#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Robot.h>
#include "PathPlanner.h"
#include "PathShortener.h"
#include "Trajectory.h"
#include "Controller.h"


using namespace std;

// Define IDs for buttons
enum DynamicSimulationTabEvents {
  id_button_SetStart = 8345,
  id_button_SetGoal,
  id_button_SetPredefStart,
  id_button_SetPredefGoal,
  id_button_RelocateObjects,
  id_button_ShowStart,
  id_button_ShowGoal,
  id_button_PrevStep,
  id_button_NextStep,
  id_button_Plan
};

// Handler for events
BEGIN_EVENT_TABLE(walkingTab, wxPanel)
EVT_COMMAND (id_button_SetStart, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonSetStart)
EVT_COMMAND (id_button_SetGoal, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonSetGoal)
EVT_COMMAND (id_button_SetPredefStart, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonSetPredefStart)
EVT_COMMAND (id_button_SetPredefGoal, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonSetPredefGoal)
EVT_COMMAND (id_button_RelocateObjects, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonRelocateObjects)
EVT_COMMAND (id_button_ShowStart, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonShowStart)
EVT_COMMAND (id_button_ShowGoal, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonShowGoal)
EVT_COMMAND (id_button_PrevStep, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonPrevStep)
EVT_COMMAND (id_button_NextStep, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonNextStep)
EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, walkingTab::onButtonPlan)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(walkingTab, GRIPTab)

walkingTab::walkingTab(wxWindow *parent,
					   const wxWindowID id,
					   const wxPoint& pos,
					   const wxSize& size,
					   long style) : GRIPTab(parent,
							   	   	   	   	 id,
							   	   	   	   	 pos,
							   	   	   	   	 size,
							   	   	   	   	 style)
{
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
  
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup planning problem"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Check"));
	wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Execute"));
  
	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
	wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

	ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Start Conf")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_SetGoal, wxT("Set Goal Conf")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_SetPredefGoal, wxT("Set Predef Goal")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_RelocateObjects, wxT("Relocate objects")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ShowGoal, wxT("Show Goal")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_PrevStep, wxT("Prev Step")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_NextStep, wxT("Next Step")), 0, wxALL, 1);
	ss3BoxS->Add(new wxButton(this, id_button_Plan, wxT("Do Planning")), 0, wxALL, 1);

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// Initialize variables
	mRobotIndex = 0; // We only simulate one robot in this demo so we know its index is 0

/* for test arms
	mPredefStartConf.resize(2);
	mPredefGoalConf.resize(2);

	mPredefStartConf[0] = -60.0*M_PI/180;
	mPredefStartConf[1] = -30.0*M_PI/180;

	mPredefStartConfList.push_back(mPredefStartConf);

	mPredefGoalConf[0] = -70.0*M_PI/180;
	mPredefGoalConf[1] = 35.0*M_PI/180;

	mPredefGoalConfList.push_back(mPredefGoalConf);
*/

	// 6 Dofs Right Arm control

	mPredefStartConf.resize(6);
	mPredefGoalConf.resize(6);

	mPredefStartConf[0] = -40.0*M_PI/180;//-66.24*M_PI/180;
	mPredefStartConf[1] = -55.0*M_PI/180;//-37.44*M_PI/180;
	mPredefStartConf[2] =  45.0*M_PI/180;//61.92*M_PI/180;
	mPredefStartConf[3] = -45.0*M_PI/180;//-46.97*M_PI/180;
	mPredefStartConf[4] = -90.0*M_PI/180;//25.92*M_PI/180;
	mPredefStartConf[5] = -15.0*M_PI/180;//-14.35*M_PI/180;

	mPredefStartConfList.push_back(mPredefStartConf);

	mPredefGoalConf[0] = -45.0*M_PI/180;//-72*M_PI/180;
	mPredefGoalConf[1] =   0.0*M_PI/180;//33.12*M_PI/180;
	mPredefGoalConf[2] =  75.0*M_PI/180;//-90.72*M_PI/180;
	mPredefGoalConf[3] = -60.0*M_PI/180;//-24.22*M_PI/180;
	mPredefGoalConf[4] = -25.0*M_PI/180;//-180*M_PI/180;
	mPredefGoalConf[5] = -15.0*M_PI/180;//40.37*M_PI/180;

	mPredefGoalConfList.push_back(mPredefGoalConf);


	// Set predefined start and goal configuration
	/* for quad walking
	mPredefStartConf.resize(10);
	mPredefGoalConf.resize(10);

	mPredefStartConf[0] = 90*M_PI/180;
	mPredefStartConf[1] = 0;
	mPredefStartConf[2] = 90*M_PI/180;
	mPredefStartConf[3] = 0;

	mPredefStartConf[4] = 0;
	mPredefStartConf[5] = 90*M_PI/180;
	mPredefStartConf[6] = 0;

	mPredefStartConf[7] = 0;
	mPredefStartConf[8] = 90*M_PI/180;
	mPredefStartConf[9] = 0;
	mPredefStartConfList.push_back(mPredefStartConf);

	mPredefGoalConf[0] = 0;
	mPredefGoalConf[1] = -90*M_PI/180;
	mPredefGoalConf[2] = 90*M_PI/180;
	mPredefGoalConf[3] = 0;

	mPredefGoalConf[4] = 0;
	mPredefGoalConf[5] = 90*M_PI/180;
	mPredefGoalConf[6] = 0;

	mPredefGoalConf[7] = 0;
	mPredefGoalConf[8] = 90*M_PI/180;
	mPredefGoalConf[9] = 0;
	mPredefGoalConfList.push_back(mPredefGoalConf);
	 */


	/*
	mPredefStartConf.resize(12);
	mPredefGoalConf.resize(12);

	//mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
	//mPredefGoalConf << -0.69115, 0.121475, 0.284977, -1.02486, 0.0, 0.0;

	//mPredefGoalConf[0] = 0;
	//mPredefGoalConf[1] = 0.1;
	//mPredefGoalConf[2] = 0.95;
	//mPredefGoalConf[3] = -8/M_PI;
	//mPredefGoalConf[4] = 8/M_PI;
	//mPredefGoalConf[5] = -8/M_PI;
	//mPredefGoalConf[6] = 8/M_PI;

	mPredefStartConf[0] = 0*M_PI/180; 	//LHY
	mPredefStartConf[1] = 0*M_PI/180; 	//LHR
	mPredefStartConf[2] = -10*M_PI/180; 	//LHP
	mPredefStartConf[3] = 20*M_PI/180;	//LKP
	mPredefStartConf[4] = -10*M_PI/180;	//LAP
	mPredefStartConf[5] = 0*M_PI/180;	//LAR

	mPredefStartConf[6] = 0*M_PI/180;
	mPredefStartConf[7] = 0*M_PI/180;
	mPredefStartConf[8] = -10*M_PI/180;
	mPredefStartConf[9] = 20*M_PI/180;
	mPredefStartConf[10] = -10*M_PI/180;
	mPredefStartConf[11] = 0*M_PI/180;
	mPredefStartConfList.push_back(mPredefStartConf);

	mPredefGoalConf[0] = 0*M_PI/180; 	//LHY
	mPredefGoalConf[1] = 2*M_PI/180; 	//LHR
	mPredefGoalConf[2] = -30*M_PI/180; 	//LHP
	mPredefGoalConf[3] = 50*M_PI/180;	//LKP
	mPredefGoalConf[4] = -15*M_PI/180;	//LAP
	mPredefGoalConf[5] = -3*M_PI/180;	//LAR

	mPredefGoalConf[6] = 0*M_PI/180;
	mPredefGoalConf[7] = -2*M_PI/180;
	mPredefGoalConf[8] = -30*M_PI/180;
	mPredefGoalConf[9] = 50*M_PI/180;
	mPredefGoalConf[10] = -15*M_PI/180;
	mPredefGoalConf[11] = -3*M_PI/180;


	mPredefGoalConfList.push_back(mPredefGoalConf);
	mPredefStartConfList.push_back(mPredefGoalConf);


	mPredefGoalConf[0] = -0.0174533 ; 	//LHY
	mPredefGoalConf[1] = 0.0523599  ; 	//LHR
	mPredefGoalConf[2] = -0.872665; 	//LHP
	mPredefGoalConf[3] = 1.0472 ;	//LKP
	mPredefGoalConf[4] = -0.157080;	//LAP
	mPredefGoalConf[5] = -0.139626;	//LAR

	mPredefGoalConf[6] =  -0.0174533;
	mPredefGoalConf[7] = -0.053826;
	mPredefGoalConf[8] = -0.469354;
	mPredefGoalConf[9] = 1.06061;
	mPredefGoalConf[10] = -0.680678;
	mPredefGoalConf[11] = 0.0145124;


	mPredefGoalConfList.push_back(mPredefGoalConf);
*/


	counter = 0;
	mStartConf = mPredefStartConf;
	mGoalConf = mPredefGoalConf;

}


/// Gets triggered after a world is loaded
void
walkingTab::GRIPEventSceneLoaded()
{
	mWorld->getRobot(mRobotIndex)->getDof(19)->setValue(-10.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(20)->setValue(-10.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(23)->setValue(20.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(24)->setValue(20.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(27)->setValue(-10.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(28)->setValue(-10.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->update();

	// Set initial configuration for the legs
	/*
	mWorld->getRobot(mRobotIndex)->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getDof(1)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getDof(2)->setValue(0.39);
	mWorld->getRobot(mRobotIndex)->getDof(3)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getDof(4)->setValue(-90.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getDof(5)->setValue(0);

	mWorld->getRobot(mRobotIndex)->getNode("Body_LHY")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LHR")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LHP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LKP")->getDof(0)->setValue(90.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LAP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LAR")->getDof(0)->setValue(0);

	mWorld->getRobot(mRobotIndex)->getNode("Body_RHY")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RHR")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RHP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RKP")->getDof(0)->setValue(90.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RAP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RAR")->getDof(0)->setValue(0);

	mWorld->getRobot(mRobotIndex)->getNode("Body_LSP")->getDof(0)->setValue(90.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LSR")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LSY")->getDof(0)->setValue(180.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LEP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LWY")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_LWP")->getDof(0)->setValue(-90.0 * M_PI/180.0);

	mWorld->getRobot(mRobotIndex)->getNode("Body_RSP")->getDof(0)->setValue(90.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RSR")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RSY")->getDof(0)->setValue(180.0 * M_PI/180.0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_REP")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RWY")->getDof(0)->setValue(0);
	mWorld->getRobot(mRobotIndex)->getNode("Body_RWP")->getDof(0)->setValue(-90.0 * M_PI/180.0);


	mWorld->getRobot(mRobotIndex)->update();
	*/

	// Define right arm nodes
	const string testArmNodes[] = {"Body_RSP", "Body_RSR"};
	//const string armNodes[] = {"Body_RSP", "Body_RSR"};
	mTestArmDofs.resize(2);

	for(int i = 0; i < mTestArmDofs.size(); i++)
		mTestArmDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(testArmNodes[i].c_str())->getDof(0)->getSkelIndex();


	// Define right arm nodes
	const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
	//const string armNodes[] = {"Body_RSP", "Body_RSR"};
	mArmDofs.resize(6);

	for(int i = 0; i < mArmDofs.size(); i++)
		mArmDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(armNodes[i].c_str())->getDof(0)->getSkelIndex();

	// Define left leg nodes
	const string leftLegNodes[] = {"Body_LHY", "Body_LHR", "Body_LHP", "Body_LKP", "Body_LAP", "Body_LAR"};
	mLeftLegDofs.resize(6);

	for(int i = 0; i < mLeftLegDofs.size(); i++)
		mLeftLegDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(leftLegNodes[i].c_str())->getDof(0)->getSkelIndex();

	// Define right leg nodes
	const string rightLegNodes[] = {"Body_RHY", "Body_RHR", "Body_RHP", "Body_RKP", "Body_RAP", "Body_RAR"};
	mRightLegDofs.resize(6);

	for(int i = 0; i < mRightLegDofs.size(); i++)
		mRightLegDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(rightLegNodes[i].c_str())->getDof(0)->getSkelIndex();

	// Define lower body nodes
	const string bothLegsNodes[] = {"Body_LHY", "Body_LHR", "Body_LHP", "Body_LKP", "Body_LAP", "Body_LAR",
			                        "Body_RHY", "Body_RHR", "Body_RHP", "Body_RKP", "Body_RAP", "Body_RAR"};
	mBothLegsDofs.resize(12);

	for(int i = 0; i < mBothLegsDofs.size(); i++)
		mBothLegsDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(bothLegsNodes[i].c_str())->getDof(0)->getSkelIndex();

	// Define quad body nodes
	const string quadNodes[] = {"Body_LSP", "Body_LEP", "Body_RSP", "Body_REP",
			                    "Body_LHP", "Body_LKP", "Body_LAP", "Body_RHP", "Body_RKP", "Body_RAP"};
	mQuadDofs.resize(10);

	for(int i = 0; i < mQuadDofs.size(); i++)
		mQuadDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(quadNodes[i].c_str())->getDof(0)->getSkelIndex();

	const string leanNodes[] = {"Body_LHR", "Body_LAR", "Body_RHR", "Body_RAR"};
 	//mLeanDofs.resize(7);
 	//mLeanDofs[0]= 0;
 	//mLeanDofs[1]= 1;
 	//mLeanDofs[2]= 2;

	//for(int i = 0; i < mLeanDofs.size()-3; i++)
	//	mLeanDofs[i+3] = mWorld->getRobot(mRobotIndex)->getNode(leanNodes[i].c_str())->getDof(0)->getSkelIndex();

	mLeanDofs.resize(4);
	for(int i = 0; i < mLeanDofs.size(); i++)
			mLeanDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(leanNodes[i].c_str())->getDof(0)->getSkelIndex();


	mPredefGoalCom = mWorld->getRobot(mRobotIndex)->getWorldCOM();
	std::cout << "current com:" << mPredefGoalCom.transpose() << " - ";

	mPredefGoalCom[1] = mPredefGoalCom[1] + 0.1;
	std::cout << "goal com:" << mPredefGoalCom.transpose() << "\n";
}


/// Before each simulation step we set the torques the controller applies to the joints
void
walkingTab::GRIPEventSimulationBeforeTimestep()
{
	Eigen::VectorXd torques = mController->getTorques(mWorld->getRobot(mRobotIndex)->getPose(),
													  mWorld->getRobot(mRobotIndex)->getQDotVector(),
													  mWorld->mTime);
	mWorld->getRobot(mRobotIndex)->setInternalForces(torques);

	//std::cout << "cur_com : [ " << mWorld->getRobot(mRobotIndex)->getWorldCOM().transpose() << " ]\n";
}


/// Set start configuration to the configuration the arm is currently in
void
walkingTab::onButtonSetStart(wxCommandEvent & _evt)
{
	if(!mWorld || mWorld->getNumRobots() < 1)
	{
		cout << "No world loaded or world does not contain a robot." << endl;
		return;
	}

	//mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mQuadDofs);
	mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mArmDofs);
	//mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mTestArmDofs);
	//mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mBothLegsDofs);
	//mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mLeanDofs);

	cout << "Start Configuration: " << mStartConf.transpose() << endl;
}


/// Set goal configuration to the configuration the arm is currently in
void
walkingTab::onButtonSetGoal(wxCommandEvent & _evt)
{
	if(!mWorld || mWorld->getNumRobots() < 1)
	{
		cout << "No world loaded or world does not contain a robot." << endl;
		return;
	}

	//mGoalConf = mWorld->getRobot(mRobotIndex)->getConfig(mQuadDofs);
	mGoalConf = mWorld->getRobot(mRobotIndex)->getConfig(mArmDofs);
	//mGoalConf = mWorld->getRobot(mRobotIndex)->getConfig(mTestArmDofs);
	//mGoalConf = mWorld->getRobot(mRobotIndex)->getConfig(mBothLegsDofs);
	//mGoalConf = mWorld->getRobot(mRobotIndex)->getConfig(mLeanDofs);
	cout << "Goal Configuration: " << mGoalConf.transpose() << endl;
}


/// Reset start configuration to the predefined one
void
walkingTab::onButtonSetPredefStart(wxCommandEvent & _evt)
{
	mStartConf = mPredefStartConfList[counter];
	cout << "Start Configuration: " << mStartConf.transpose() << endl;
	//++counter;
	//mStartConf = mPredefStartConf;
}


/// Reset goal configuration to the predefined one
void
walkingTab::onButtonSetPredefGoal(wxCommandEvent & _evt)
{
	mGoalConf = mPredefGoalConfList[counter];
	cout << "Goal Configuration: " << mGoalConf.transpose() << endl;
	++counter;
	//mGoalCom = mPredefGoalCom;
	//std::cout << "Goal com " << mGoalCom.transpose() << std::endl;
}


/// Move objects to obstruct the direct path between the predefined start and goal configurations
void
walkingTab::onButtonRelocateObjects(wxCommandEvent & _evt)
{
	robotics::Robot* orangeCube = (robotics::Robot*)mWorld->getSkeleton("orangeCube");
	robotics::Robot* yellowCube = (robotics::Robot*)mWorld->getSkeleton("yellowCube");
  
	if(!orangeCube || !yellowCube)
	{
		cout << "Did not find orange or yellow object. Exiting and no moving anything" << endl;
		return;
	}
  
	Eigen::Matrix<double, 6, 1> pose;
	pose << 0.30, -0.30, 0.83, 0.0, 0.0, 0.0;
	orangeCube->setRootTransform(pose);
	pose << 0.30, -0.30, 0.935, 0.0, 0.0, 0.0;
	yellowCube->setRootTransform(pose);

	viewer->DrawGLScene();
}


/// Show the currently set start configuration
void
walkingTab::onButtonShowStart(wxCommandEvent & _evt)
{
	//cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mTestArmDofs, mStartConf);

	cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
	mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mStartConf);

	//cout << "Showing start conf for both legs: " << mStartConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mBothLegsDofs, mStartConf);

	//cout << "Showing start conf lean joints: " << mStartConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mLeanDofs, mStartConf);

	//cout << "Showing start conf for quad dofs: " << mStartConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mQuadDofs, mStartConf);

	viewer->DrawGLScene();
}


/// Show the currently set goal configuration
void
walkingTab::onButtonShowGoal(wxCommandEvent & _evt)
{
	//cout << "Showing goal conf for right arm: " << mGoalConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mTestArmDofs, mGoalConf);

	cout << "Showing goal conf for right arm: " << mGoalConf.transpose() << endl;
	mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mGoalConf);

	//cout << "Showing goal conf for both legs: " << mGoalConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mBothLegsDofs, mGoalConf);

	//cout << "Showing goal conf for leaning joints: " << mGoalConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mLeanDofs, mGoalConf);

	//cout << "Showing goal conf for quad dofs: " << mGoalConf.transpose() << endl;
	//mWorld->getRobot(mRobotIndex)->setConfig(mQuadDofs, mGoalConf);

	viewer->DrawGLScene();
}

void
walkingTab::onButtonPrevStep(wxCommandEvent & _evt)
{
	if (path_index < 0)
		path_index = 0;
	else if (path_index >= computed_path.size())
		path_index = computed_path.size()-1;

	//mWorld->getRobot(mRobotIndex)->setConfig(mTestArmDofs, computed_path[path_index]);

	mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, computed_path[path_index]);

	//mWorld->getRobot(mRobotIndex)->setConfig(mBothLegsDofs, computed_path[path_index]);

	//mWorld->getRobot(mRobotIndex)->setConfig(mQuadDofs, computed_path[path_index]);

	viewer->DrawGLScene();
	path_index = path_index-1;
}

void
walkingTab::onButtonNextStep(wxCommandEvent & _evt)
{
	if (path_index < 0)
		path_index = 0;
	else if (path_index >= computed_path.size())
		path_index = computed_path.size()-1;

	//mWorld->getRobot(mRobotIndex)->setConfig(mTestArmDofs, computed_path[path_index]);
	mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, computed_path[path_index]);
	//mWorld->getRobot(mRobotIndex)->setConfig(mBothLegsDofs, computed_path[path_index]);

	//mWorld->getRobot(mRobotIndex)->setConfig(mQuadDofs, computed_path[path_index]);

	viewer->DrawGLScene();

	path_index = path_index+1;
}

/// Set initial dynamic parameters and call planner and controller
void
walkingTab::onButtonPlan(wxCommandEvent & _evt)
{
	std::cout << "Planning started\n";

	// Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
	std::vector<int> actuatedDofs(mWorld->getRobot(mRobotIndex)->getNumDofs() - 6);

	for(unsigned int i = 0; i < actuatedDofs.size(); i++)
		actuatedDofs[i] = i + 6;


	// Deactivate collision checking between the feet and the ground during planning
	dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");
	//mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_LAR"), ground->getNode(1));
	//mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_RAR"), ground->getNode(1));
  
	// Define PD controller gains
	Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
	Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
	Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());

	// Define gains for the ankle PD
	std::vector<int> ankleDofs(2);
	ankleDofs[0] = 27;
	ankleDofs[1] = 28;
	const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
	const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);

	// Set robot to start configuration
	//mWorld->getRobot(mRobotIndex)->setConfig(mTestArmDofs, mStartConf);
	mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mStartConf);
	//mWorld->getRobot(mRobotIndex)->setConfig(mBothLegsDofs, mStartConf);
	//mWorld->getRobot(mRobotIndex)->setConfig(mLeanDofs, mStartConf);

	//mWorld->getRobot(mRobotIndex)->setConfig(mQuadDofs, mStartConf);

	// Create controller
	mController = new walking::Controller(mWorld->getRobot(mRobotIndex),
										   actuatedDofs,
										   kP,
										   kD,
										   ankleDofs,
										   anklePGains,
										   ankleDGains);

	// Call path planner
	walking::PathPlanner<> pathPlanner(*mWorld);
	std::list<Eigen::VectorXd> path;

	pathPlanner.connect = false;
	pathPlanner.bidirectional = false;
	pathPlanner.maxNodes = 1e6;
	pathPlanner.stepSize = 0.15;
	pathPlanner.goalBias = 0.07;


	//if(!pathPlanner.planPath(mRobotIndex, mTestArmDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath(mRobotIndex, mQuadDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath(mRobotIndex, mArmDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath(mRobotIndex, mLeanDofs, mStartConf, mGoalCom, path))

	//if(!pathPlanner.planPath(mRobotIndex, mLeanDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath(mRobotIndex, mBothLegsDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath_star(mRobotIndex, mBothLegsDofs, mStartConf, mGoalConf, path))
	if(!pathPlanner.planPath_sharp(mRobotIndex, mArmDofs, mStartConf, mGoalConf, path))
	//if(!pathPlanner.planPath_star(mRobotIndex, mTestArmDofs, mStartConf, mGoalConf, path))
    //if(!pathPlanner.planPath_sharp(mRobotIndex, mTestArmDofs, mStartConf, mGoalConf, path))
	{
		std::cout << "Path planner could not find a path." << std::endl;
	}
	else
	{
		path_index = 0;
		// Call path shortener
		//walking::PathShortener pathShortener(mWorld, mRobotIndex, mArmDofs);
		//walking::PathShortener pathShortener(mWorld, mRobotIndex, mBothLegsDofs);
		//walking::PathShortener pathShortener(mWorld, mRobotIndex, mLeanDofs);
		//walking::PathShortener pathShortener(mWorld, mRobotIndex, mQuadDofs);

		//pathShortener.shortenPath(path);
		//pathShortener.shortenPath(path);

		std::list<Eigen::VectorXd>::iterator it;
		//computed_path.resize(path.size());
		int k = 0;
		for (it = path.begin(); k < path.size(); ++it )
		{
			computed_path.push_back(*it);
			//computed_path[k] = *it;
			//std::cout << (computed_path[k]).transpose() << std::endl;
			k = k + 1;
		}

		mWorld->getRobot(mRobotIndex)->update();

		// Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
		//const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mQuadDofs.size());
		//const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mQuadDofs.size());

		//const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTestArmDofs.size());
		//const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTestArmDofs.size());

		const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
		const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());

		//const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mBothLegsDofs.size());
		//const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mBothLegsDofs.size());

		//const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mLeanDofs.size());
		//const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mLeanDofs.size());

		walking::Trajectory* trajectory = new walking::Trajectory(path, maxVelocity, maxAcceleration);
		std::cout << "-- Trajectory duration: " << trajectory->getDuration() << std::endl;
		//mController->setTrajectory(trajectory, 0.0, mTestArmDofs);
		mController->setTrajectory(trajectory, 0.0, mArmDofs);
		//mController->setTrajectory(trajectory, 0.0, mQuadDofs);
		//mController->setTrajectory(trajectory, 0.0, mBothLegsDofs);
		//mController->setTrajectory(trajectory, 0.0, mLeanDofs);
	}
  
	// Reactivate collision of feet with floor
	mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_LAR"), ground->getNode(1));
	mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_RAR"), ground->getNode(1));
}

// Local Variables:
// c-basic-offset: 2
// End:
