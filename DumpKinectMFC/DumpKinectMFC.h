
// DumpKinectMFC.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CDumpKinectMFCApp:
// See DumpKinectMFC.cpp for the implementation of this class
//

class CDumpKinectMFCApp : public CWinApp
{
public:
	CDumpKinectMFCApp();

// Overrides
public:
	virtual BOOL InitInstance();

	virtual BOOL OnIdle(LONG lCount);

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CDumpKinectMFCApp theApp;