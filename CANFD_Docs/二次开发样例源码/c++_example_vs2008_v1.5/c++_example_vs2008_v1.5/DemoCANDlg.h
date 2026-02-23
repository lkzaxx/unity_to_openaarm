// DemoCANDlg.h : header file
//

#if !defined(AFX_DEMOCANDLG_H__1C22D061_FD97_4A72_AE0D_046165FE8B1D__INCLUDED_)
#define AFX_DEMOCANDLG_H__1C22D061_FD97_4A72_AE0D_046165FE8B1D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CDemoCANDlg dialog
#include "ColoredListCtrl.h"

class CDemoCANDlg : public CDialog
{
// Construction
public:
	CDemoCANDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CDemoCANDlg)
	enum { IDD = IDD_DEMOCAN_DIALOG };
	CColoredListCtrl	m_list;
	int		m_nSendFrameFormat;
	int		m_nSendFrameType;
	CString	m_strSendData;
	CString	m_strSendID;
	int		m_radioIDFormat;
	BOOL	m_bCanRxEn;
	int		m_nCanIndex;
	int		m_nDevType;
	int		m_canFD;
	int     m_RecvNum;
	int     m_RecvFDNum;
	int     m_RecvCan1Num;
	int     m_RecvCanFD1Num;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDemoCANDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL
public:
	int DeviceIndex;
	int m_DevType;
	int m_DevIndex;
	static UINT ReceiveThread(void *param);
// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CDemoCANDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnButtonOpenDevice();
	afx_msg void OnButtonSend();
	afx_msg void OnCheckCanrxEn();
	afx_msg void OnButtonClear();
	afx_msg void OnButtonCloseDevice();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnEnChangeEdit1();
	afx_msg void OnEnChangeRecvNum();
	afx_msg void OnEnChangeEditSendId();
	afx_msg void OnStnClickedStaticSelect();
	afx_msg void OnLvnItemchangedList1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButtonUpdate();
	//afx_msg void OnCbnSelchangeComboDevtype();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DEMOCANDLG_H__1C22D061_FD97_4A72_AE0D_046165FE8B1D__INCLUDED_)
