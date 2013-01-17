/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifndef PLANNING_TAB
#define PLANNING_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>

namespace planning { class Controller; }

/**
 * @class pushDemoTab
 */
class pushDemoTab : public GRIPTab
{
public:
  pushDemoTab(){};
  pushDemoTab(wxWindow * parent, wxWindowID id = -1,
	      const wxPoint & pos = wxDefaultPosition,
	      const wxSize & size = wxDefaultSize,
	      long style = wxTAB_TRAVERSAL);
  virtual ~pushDemoTab(){};
	
  wxSizer* sizerFull;
  
  void OnSlider(wxCommandEvent &evt);
  void OnButton(wxCommandEvent &evt);
  void GRIPStateChange();

  // *************************************  
  // Dynamic Simulation Variables
  enum playstate_enum {
    SIMULATE,
    RECORD,
    PLAYBACK,
    PAUSED
  };

  playstate_enum mPlayState;
  playstate_enum mPlayStateLast;
  int mSimFrame;
  int mPlayFrame;
  int mMovieFrame;
  bool mScreenshotScheduled;
  bool mShowMarker;
  double mDisplayTimeout;

  std::vector<Eigen::VectorXd> mBakedStates;
  planning::Controller* mController;
  static const string mRA_TrajNodes[];
  int static const mRA_NumNodes = 7;
  Eigen::VectorXd mStartConf;
  Eigen::VectorXd mGoalConf;
  

  void addFloor();
  void settings();
  void simulate();
  void SetTimeline();
  void bake();
  void retrieveBakedState( int _frame );
  // *************************************

  
  // Thread specific
  // GRIPThread* thread;
  
  // Your Thread routine
  // call GRIPThread::CheckPoint() regularly
  // void Thread();
  // void onCompleteThread();
  
  DECLARE_DYNAMIC_CLASS(pushDemoTab)
    DECLARE_EVENT_TABLE()
    };

#endif
