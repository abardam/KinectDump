
// DumpKinectMFCDlg.h : header file
//

#pragma once
#include "afxwin.h"


// CDumpKinectMFCDlg dialog
class CDumpKinectMFCDlg : public CDHtmlDialog
{
// Construction
public:
	CDumpKinectMFCDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_DUMPKINECTMFC_DIALOG, IDH = IDR_HTML_DUMPKINECTMFC_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

	HRESULT OnButtonOK(IHTMLElement *pElement);
	HRESULT OnButtonCancel(IHTMLElement *pElement);

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	DECLARE_DHTML_EVENT_MAP()
public:
	afx_msg void OnBnClickedButton1();
	void ShowKinect();

	CStatic m_cStatic;
};
