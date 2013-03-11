/**
 * @file walkingTabApp.h
 * @brief Creates application for walkingTab
 * @author O. Arslan
 */

#include "GRIPApp.h"
#include "walkingTab.h"

extern wxNotebook* tabView;

class walkingTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new walkingTab(tabView), wxT("walking Tab"));
	}
};

IMPLEMENT_APP(walkingTabApp)
