// DemoCANDlg.cpp : implementation file
//
#include "stdafx.h"
#include "DemoCAN.h"
#include "DemoCANDlg.h"
#include "ControlCANFD.h"

// v1.0
// v1.1-- 
// v1.2 -- add external apps for labView
// v1.3 -- test filter seting

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//int     m_RecvNum=0;
CString sFile;
unsigned long nextrow;
int StopFlag=0;
///////////////////////////////////////////////////////////////////////////// 
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDemoCANDlg dialog

CDemoCANDlg::CDemoCANDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CDemoCANDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDemoCANDlg)
	m_nSendFrameFormat = 0;
	m_nSendFrameType = 0;
	m_strSendData = _T("11 22 33 44 55 66 77 88");
	m_strSendID = _T("00 00 00 88");
	m_radioIDFormat = 1;
	m_bCanRxEn = FALSE;
	m_nCanIndex = 0;
	//m_nDevType = 1;
	m_canFD = 0;
	m_RecvNum = 0;
	m_RecvFDNum = 0;
	m_RecvCan1Num = 0;
	m_RecvCanFD1Num = 0;
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CDemoCANDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDemoCANDlg)
	DDX_Control(pDX, IDC_LIST1, m_list);
	DDX_CBIndex(pDX, IDC_COMBO_SENDFRAMEFORMAT, m_nSendFrameFormat);
	DDX_CBIndex(pDX, IDC_COMBO_SENDFRAMETYPE, m_nSendFrameType);
	DDX_Text(pDX, IDC_EDIT_SEND_DATA, m_strSendData);
	DDX_Text(pDX, IDC_EDIT_SEND_ID, m_strSendID);
	DDX_Radio(pDX, IDC_RADIO_ID_FORMAT, m_radioIDFormat);
	DDX_Check(pDX, IDC_CHECK_CANRX_EN, m_bCanRxEn);
	DDX_CBIndex(pDX, IDC_COMBO_CAN_INDEX, m_nCanIndex);
	DDX_CBIndex(pDX, IDC_COMBO_DEVTYPE, /*m_nDevType*/m_canFD);
/*
	DDX_Text(pDX, IDC_RECV_NUM, m_RecvNum);
	DDX_Text(pDX, IDC_RECVFD_NUM, m_RecvFDNum);
	DDX_Text(pDX, IDC_RECV_CAN1_NUM, m_RecvCan1Num);
	DDX_Text(pDX, IDC_RECV_CANFD1_NUM, m_RecvCanFD1Num);
	*/
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDemoCANDlg, CDialog)
	//{{AFX_MSG_MAP(CDemoCANDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_OPEN_DEVICE, OnButtonOpenDevice)
	ON_BN_CLICKED(IDC_BUTTON_SEND, OnButtonSend)
	ON_BN_CLICKED(IDC_CHECK_CANRX_EN, OnCheckCanrxEn)
	ON_BN_CLICKED(IDC_BUTTON_CLEAR, OnButtonClear)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE_DEVICE, OnButtonCloseDevice)
	//}}AFX_MSG_MAP
	//ON_EN_CHANGE(IDC_EDIT1, &CDemoCANDlg::OnEnChangeEdit1)
//	ON_EN_CHANGE(IDC_RECV_NUM, &CDemoCANDlg::OnEnChangeRecvNum)
//	ON_EN_CHANGE(IDC_EDIT_SEND_ID, &CDemoCANDlg::OnEnChangeEditSendId)
//	ON_STN_CLICKED(IDC_STATIC_SELECT, &CDemoCANDlg::OnStnClickedStaticSelect)
//	ON_NOTIFY(LVN_ITEMCHANGED, IDC_LIST1, &CDemoCANDlg::OnLvnItemchangedList1)
//ON_BN_CLICKED(IDC_BUTTON_UPDATE, &CDemoCANDlg::OnBnClickedButtonUpdate)
//ON_CBN_SELCHANGE(IDC_COMBO_DEVTYPE, &CDemoCANDlg::OnCbnSelchangeComboDevtype)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDemoCANDlg message handlers

BOOL CDemoCANDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
    //信息显示列表初始化
	m_list.InsertColumn(0,"Seq");
	m_list.SetColumnWidth(0,40);
	m_list.InsertColumn(1,"Time");
	m_list.SetColumnWidth(1,120);
	//m_list.InsertColumn(2,"CANIndex");
	m_list.InsertColumn(2,"Channel");
	m_list.SetColumnWidth(2,70);
	m_list.InsertColumn(3,"Tx-Rx");
	m_list.SetColumnWidth(3,60);
	m_list.InsertColumn(4," ID ");
	m_list.SetColumnWidth(4,60);
	m_list.InsertColumn(5,"Frame");
	m_list.SetColumnWidth(5,60);
	m_list.InsertColumn(6,"Type");
	m_list.SetColumnWidth(6,80);
	m_list.InsertColumn(7,"DLC");
	m_list.SetColumnWidth(7,40);
	m_list.InsertColumn(8,"CAN-FD");
	m_list.SetColumnWidth(8,80);
	m_list.InsertColumn(9,"Data");	
	m_list.SetColumnWidth(9,800); 
	//m_list.SetColumnWidth(8,160); 
	//信息显示列表初始化完毕
	m_strSendID = "12 34 56 78";
	// TODO: Add extra initialization here
	UpdateData(FALSE);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CDemoCANDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CDemoCANDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CDemoCANDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

//一位十六进制转换为十进制
int HexChar(char c)
{
	if((c>='0') && (c<='9'))
		return c-0x30;
	else if((c>='A') && (c<='F'))
		return c-'A'+10;
	else if((c>='a') && (c<='f'))
		return c-'a'+10;
	else
		return 0x10;
}
//两位十六进制数转换为十进制
int Str2Hex(CString str)
{	
	int len = str.GetLength();
	if(len == 2)
	{
		int a= HexChar(str[0]);
		int b =HexChar(str[1]);
		if(a==16 || b==16 )
		{
			AfxMessageBox("Format error");
			return 256;
		}
		else
		{
			return a*16+b;
			
		}
		
	}
	else
	{
		AfxMessageBox("input length must be 2");
		return 256;
	}
}

DEVICE_HANDLE  m_dev=INVALID_DEVICE_HANDLE;
CHANNEL_HANDLE dev_ch1;
CHANNEL_HANDLE dev_ch2;
void CDemoCANDlg::OnButtonOpenDevice() 
{ 
	UpdateData(TRUE);
	
	m_DevType = ZCAN_USBCANFD_200U;
	m_DevIndex=0;
	DWORD Reserved=0;

	//打开设备
	m_dev = ZCAN_OpenDevice(m_DevType,m_DevIndex,Reserved);
	if(INVALID_DEVICE_HANDLE == m_dev)
	{
		MessageBox("open failed");
		return;	
	}
	
	IProperty * _pPro = GetIProperty(m_dev);
	const char * str;
	if(_pPro == NULL)
	{
		MessageBox("Property's NULL!");
		return;
	}
#if 1
	//属性方式波特率设置
	//设置通道1 仲裁域波特率 500kbps
	if( STATUS_OK != _pPro->SetValue("0/canfd_abit_baud_rate","500000") )
	{
		MessageBox("Set ch0 rateA failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 数据域波特率 1Mbps
	if( STATUS_OK != _pPro->SetValue("0/canfd_dbit_baud_rate","1000000") )
	{
		MessageBox("Set ch0 rateD failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道2 仲裁域波特率 500kbps
	if( STATUS_OK != _pPro->SetValue("1/canfd_abit_baud_rate","500000") )
	{
		MessageBox("Set ch1 rateA failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道2 数据域波特率 1Mbps
	if( STATUS_OK != _pPro->SetValue("1/canfd_dbit_baud_rate","10000000") )
	{
		MessageBox("Set ch1 rateD failed!");
		ReleaseIProperty(_pPro);
		return;
	}
#else
#if 1
	//函数方式设置特定波特率
	//设置通道1 仲裁域波特率 500kbps
	if( STATUS_OK != ZCAN_SetAbitBaud(m_dev,0,500000) )
	{
		MessageBox("Set ch0 rateA failed!");		
		return;
	}
	//设置通道1 数据域波特率 1Mbps
	if( STATUS_OK != ZCAN_SetDbitBaud(m_dev,0,1000000) )
	{
		MessageBox("Set ch0 rateD failed!");		
		return;
	}
	//设置通道2 仲裁域波特率 500kbps
	if( STATUS_OK != ZCAN_SetAbitBaud(m_dev,1,500000) )
	{
		MessageBox("Set ch1 rateA failed!");		
		return;
	}
	//设置通道2 数据域波特率 1Mbps
	if( STATUS_OK != ZCAN_SetDbitBaud(m_dev,1,1000000) )
	{
		MessageBox("Set ch1 rateD failed!");		
		return;
	}
#else
	//函数方式自定义波特率设置	
	//设置通道1 数据域波特率 1Mbps
	if( STATUS_OK != ZCAN_SetBaudRateCustom(m_dev,0,"1.0Mbps(80%),5.0Mbps(83%),(60,02C00002,00400003)") )
	{
		MessageBox("Set ch1 rateD failed!");
		ReleaseIProperty(_pPro);
		return;
	}

//设置通道2 数据域波特率 1Mbps
	if( STATUS_OK != ZCAN_SetBaudRateCustom(m_dev,1,"1.0Mbps(80%),5.0Mbps(83%),(60,02C00002,00400003)") )
	{
		MessageBox("Set ch1 rateD failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	
#endif

//设置通道1 FDMode ISO
	if( STATUS_OK != ZCAN_SetCANFDStandard(m_dev,0,0) )
	{
		MessageBox("Set ch1 FDMode ISO Failed!");
		ReleaseIProperty(_pPro);
		return;
	}
//设置通道2 FDMode BOSCH
	if( STATUS_OK != ZCAN_SetCANFDStandard(m_dev,1,1) )
	{
		MessageBox("Set ch2 FDMode BOSCH Failed!");
		ReleaseIProperty(_pPro);
		return;
	}

//设置通道1 中断电阻 disable
	if( STATUS_OK != ZCAN_SetResistanceEnable(m_dev,0,0) )
	{
		MessageBox("Set ch1 resistance disable Failed!");
		ReleaseIProperty(_pPro);
		return;
	}
//设置通道2 中断电阻 enable
	if( STATUS_OK != ZCAN_SetResistanceEnable(m_dev,1,1) )
	{
		MessageBox("Set ch2 resistance enable Failed!");
		ReleaseIProperty(_pPro);
		return;
	}
#endif
	
	ZCAN_CHANNEL_INIT_CONFIG cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.can_type = TYPE_CANFD; 	// FD设备
	cfg.canfd.mode = 0;  		//正常模式 
	cfg.canfd.filter = 0;
	cfg.canfd.pad = 0;	
	cfg.canfd.brp = 0;
	//cfg.canfd.abit_timing = 0;
	//cfg.canfd.dbit_timing = 0;
	cfg.canfd.acc_code = 0;
	cfg.canfd.acc_mask = 0xffffffff;
	cfg.canfd.reserved = 0;

	//初始化通道1
	dev_ch1 = ZCAN_InitCAN(m_dev,0,&cfg);
	if(INVALID_CHANNEL_HANDLE == dev_ch1)
	{
		MessageBox("Init-CAN0 failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	#if 1
	//////////////////////////////////////////////////////////////////////////////////
	//////// v1.3 -- add filter set after init!!!
	#if 0
	//属性方式滤波设置
	//清除通道1 filter
	if( STATUS_OK != _pPro->SetValue("0/filter_clear","0") )
	{
		MessageBox("clear ch0 filter failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 模式：标准帧滤波
	if( STATUS_OK != _pPro->SetValue("0/filter_mode","0") )
	{
		MessageBox("set ch0 filter  mode failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 起始ID：0x100
	if( STATUS_OK != _pPro->SetValue("0/filter_start","0x000100") )
	{
		MessageBox("set ch0 filter  start failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 结束ID：0x200
	if( STATUS_OK != _pPro->SetValue("0/filter_end","0x000200") )
	{
		MessageBox("set ch0 filter  end failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//生效通道1 filter
	if( STATUS_OK != _pPro->SetValue("0/filter_ack","0") )
	{
		MessageBox("set ch0 filter  ack failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	
	#else
	//函数方式滤波设置	
	//清除通道1 filter
	if( STATUS_OK != ZCAN_ClearFilter(dev_ch1) )
	{
		MessageBox("clear ch0 filter failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 模式：标准帧滤波
	if( STATUS_OK != ZCAN_SetFilterMode(dev_ch1,0) )
	{
		MessageBox("set ch0 filter  mode failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 起始ID：0x100
	if( STATUS_OK != ZCAN_SetFilterStartID(dev_ch1,0x100) )
	{
		MessageBox("set ch0 filter  start failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//设置通道1 filter 结束ID：0x200
	if( STATUS_OK != ZCAN_SetFilterEndID(dev_ch1,0x200) )
	{
		MessageBox("set ch0 filter  end failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	//生效通道1 filter
	if( STATUS_OK != ZCAN_AckFilter(dev_ch1) )
	{
		MessageBox("set ch0 filter  ack failed!");
		ReleaseIProperty(_pPro);
		return;
	}
	
	#endif
	
	#endif
	
	//启动通道1
	if(STATUS_ERR == ZCAN_StartCAN(dev_ch1))
	{
		MessageBox("Start-CAN0 failed!");
		ReleaseIProperty(_pPro);
		return;
	}

    //初始化通道2
	dev_ch2 = ZCAN_InitCAN(m_dev, 1, &cfg);
	if(INVALID_CHANNEL_HANDLE == dev_ch2)
	{
		MessageBox("Init-CAN1 failed!");
		ReleaseIProperty(_pPro);
		return;
	}
    //启动通道2
	if(STATUS_ERR == ZCAN_StartCAN(dev_ch2))
	{
		MessageBox("Start-CAN1 failed!");
		ReleaseIProperty(_pPro);
		return;
	}


	ZCAN_DEVICE_INFO devInfo;
	if(STATUS_ERR == ZCAN_GetDeviceInf(m_dev, &devInfo))
	{
		MessageBox("Get Dev info failed!");
		ReleaseIProperty(_pPro);
		return;
	}

	MessageBox("Open successfule!\n Start CAN OK!");	

}

void CDemoCANDlg::OnButtonCloseDevice() 
{
	//关闭设备
	if(STATUS_OK != ZCAN_CloseDevice(m_dev))
	{
		MessageBox("Close failed！");
		return;
		
	}
	MessageBox("Close successful!");
}

int testFlag=0;
//发送信息
void CDemoCANDlg::OnButtonSend() 
{
	UpdateData(TRUE);

//从界面获取发送信息
	//VCI_CAN_OBJ sendbuf[1];

	CString str5[332];
	BYTE buf[350];
	BYTE SendID[10];
	CString strtemp,strtemp1;
	CString str1;
	int len,datanum=0,IDnum=0,newflag=1,i;

	/// get id from edit
	memset(SendID, 0 ,10);
	len=m_strSendID.GetLength();	//from dialog
	for(i=0; i<len;i++)
	{
		strtemp=m_strSendID.GetAt(i);
		if(strtemp==" ")
			newflag=1;
		else if(newflag==1)
		{
			newflag=0;
			strtemp=m_strSendID.GetAt(i);
			if(i==(len-1))
			{
				str5[IDnum]="0"+strtemp;
			}
			else
			{
				strtemp1=m_strSendID.GetAt(i+1);

				if(strtemp1 == " ")
					str5[IDnum]="0"+strtemp;
				else
					str5[IDnum]=strtemp+strtemp1;
			}
			SendID[IDnum]=Str2Hex(str5[IDnum]);
			IDnum++;
			if(IDnum>=4)
				break;
		}

	}
	UINT GetId = SendID[0];
	for(i=1; i<IDnum; i++)
		GetId = (GetId<<8) + SendID[i];
	GetId = (m_nSendFrameType==1) ? GetId&0x1FFFFFFF : GetId&0x7FF;
	//GetId = (m_nSendFrameType==1) ? (((SendID[0]<<24)|(SendID[1]<<16)|(SendID[2]<<8)|SendID[3])&0x1FFFFFFF) : (((SendID[2]<<8)|SendID[3])&0x7FF);

	
	/// get data from edit
	newflag=1;
	len=m_strSendData.GetLength();	
	if(m_nSendFrameFormat==0)//if remote frame, data area is invalid
	{
	
		for(i=0; i<len;i++)
		{
			strtemp=m_strSendData.GetAt(i);
			if(strtemp==" ")
				newflag=1;
			else if(newflag==1)
			{	
				newflag=0;
				strtemp=m_strSendData.GetAt(i);		
				if(i==(len-1))
				{
					str5[datanum]="0"+strtemp;
				}
				else
				{
					strtemp1=m_strSendData.GetAt(i+1);

					if(strtemp1 == " ")
					{
						str5[datanum]="0"+strtemp;
					
					}
					else
						str5[datanum]=strtemp+strtemp1;
				
				}
				buf[datanum]=Str2Hex(str5[datanum]);
				datanum++;
				//if(datanum>=8)
				if(datanum>=64)
					break;
			}
		}
	}
	else
	{
		for(i=0;i<datanum;i++)
			buf[i]=0;
	}
	
/****************************************************************************/	 
/******************************从界面获取发送信息完毕***********************/
/****************************************************************************/ 	
	
	testFlag = m_canFD;
	if(testFlag == 0)
	{
		//testFlag = 1;
		
		//向通道1发送CAN帧		
		ZCAN_Transmit_Data can_data;
		can_data.frame.can_id = MAKE_CAN_ID(GetId, m_nSendFrameType, m_nSendFrameFormat, 0);
		can_data.frame.can_dlc = datanum;
		for(int i=0;i<can_data.frame.can_dlc;i++)
			can_data.frame.data[i]=buf[i];
		can_data.transmit_type = 0; //正常发送
		
		if( 1 != ZCAN_Transmit((m_nCanIndex == 0) ? dev_ch1 : dev_ch2, &can_data, 1) )
		{
			MessageBox("send failed\n");
			return;		
		}
	}
	else
	{
		//testFlag = 0;
		//向通道1发送CANFD帧
		ZCAN_TransmitFD_Data canfd_data;
		canfd_data.frame.can_id = MAKE_CAN_ID(GetId, m_nSendFrameType , m_nSendFrameFormat, 0);
		canfd_data.frame.len = datanum;
		for(int i=0;i<canfd_data.frame.len;i++)
			canfd_data.frame.data[i]=buf[i];
		canfd_data.transmit_type = 0; //正常发送
		
		if( 1 != ZCAN_TransmitFD((m_nCanIndex == 0) ? dev_ch1 : dev_ch2, &canfd_data, 1) )
		{
			MessageBox("sendFD failed\n");
			return;
		}
	}

	
///*--test
	CSize size;
	unsigned int JustnowItem;
	BYTE data;
    
    //发送信息列表显示
	CString strTime;
	SYSTEMTIME   systime;   
	GetLocalTime(&systime);   
	strTime.Format("%02d:%02d:%02d:%03d", systime.wHour,systime.wMinute,systime.wSecond,systime.wMilliseconds);   
	
	size.cx=0;
	size.cy=50;
	CString str;
	str.Format("%d",nextrow);
	m_list.ItemColorFlag[nextrow]=1;
	JustnowItem=m_list.InsertItem(nextrow,str);	

	nextrow++;	
	m_list.SetItemText(JustnowItem,1,strTime);		
	str.Format("%d",m_nCanIndex);	
	m_list.SetItemText(JustnowItem,2,str);		
	m_list.SetItemText(JustnowItem,3,"Send");		
	str="";

	if(m_nSendFrameFormat==1)
	{
		m_list.SetItemText(JustnowItem,5,"Remote");	
	}
	else
	{
		m_list.SetItemText(JustnowItem,5,"Data");	
	}
	if(m_nSendFrameType==1)			
	{
			/*
			for(i=0;i<4;i++)
			{
				data=SendID[i];
				str1.Format("%02X",data);
				str+=str1;
			}
			*/
			//UINT id = (SendID[0]<<24) + (SendID[1]<<16) + (SendID[2]<<8) + SendID[3];
			str.Format("0x%08X",GetId&0x1fffffff);
			m_list.SetItemText(JustnowItem,4,str);			
			m_list.SetItemText(JustnowItem,6,"Extended");
	}
	else									
	{
			/*
			for(i=0;i<2;i++)
			{			

					data=SendID[i+2];

				str1.Format("%02X",data);
				str+=str1;
			}
			*/
			//UINT id = (SendID[2]<<8) + SendID[3];
			str.Format("0x%03X",GetId&0x7ff);
			m_list.SetItemText(JustnowItem,4,str);
			m_list.SetItemText(JustnowItem,6,"Standard");
	}
	
	str.Format("%d",datanum);	
	m_list.SetItemText(JustnowItem,7,str);	
	
	m_list.SetItemText(JustnowItem,8,(testFlag==0)? "CAN" : "CANFD");
	
	str="";
	for(i=0;i<datanum;i++)		 
	{
		data=buf[i];
		str1.Format("%02X",data);
		str+=str1+" ";
	}	
	m_list.SetItemText(JustnowItem,9,str);	
	m_list.Scroll(size); 
	//发送信息列表显示完毕

}

int RV_CAN0_NUMS=0;
int RV_CANFD0_NUMS=0;
int RV_CAN1_NUMS=0;
int RV_CANFD1_NUMS=0;
#if 0
#else

ZCAN_Receive_Data	pCanObj0[2500];
ZCAN_Receive_Data	pCanObj1[2500];
ZCAN_ReceiveFD_Data	pCanObjFD0[2500];
ZCAN_ReceiveFD_Data	pCanObjFD1[2500];


//接收信息列表显示
static void disp_recv_can_frame(BYTE ch, ZCAN_Receive_Data* pReceive, UINT NumValue)
{
	CDemoCANDlg *dlg=(CDemoCANDlg*) AfxGetApp()->GetMainWnd();
	//ZCAN_Receive_Data * pCan = 
	
	CSize size;
	unsigned int JustnowItem;
	DWORD ReceivedID;
	size.cx=0;
	size.cy=50;
	CString str;
	CString str1;
	
	CString strTime; 
	SYSTEMTIME   systime;   
	GetLocalTime(&systime);   
	strTime.Format("%02d:%02d:%02d:%03d", systime.wHour,systime.wMinute,systime.wSecond,systime.wMilliseconds);   
				
	for(int num=0;num<NumValue;num++)
	{
		if(nextrow==59999)
		{
			dlg->m_list.DeleteAllItems();
			nextrow=0;
		}

		//序号
		str.Format("%d",nextrow);
		dlg->m_list.ItemColorFlag[nextrow]=0;				
		JustnowItem=dlg->m_list.InsertItem(nextrow,str);	
		nextrow++;	
		
		//时间-
		dlg->m_list.SetItemText(JustnowItem,1,strTime);	
		
		//CAN通道
		str.Format("%d",ch);
		dlg->m_list.SetItemText(JustnowItem,2,str);	
		
		//方向-Rx
		dlg->m_list.SetItemText(JustnowItem,3,"Recv");	
		str="";

		//远程帧 
		if(IS_RTR(pReceive[num].frame.can_id)==1)
		{
			dlg->m_list.SetItemText(JustnowItem,5,"Remote");	
		}
		else
		{
			dlg->m_list.SetItemText(JustnowItem,5,"Data");	
		
		}
		// 扩展帧
		if(IS_EFF(pReceive[num].frame.can_id)==1)		
		{
			ReceivedID=GET_ID(pReceive[num].frame.can_id);
			str1.Format("0x%08X",ReceivedID&0x1FFFFFFF);
			dlg->m_list.SetItemText(JustnowItem,4,str1);	//ID信息	
					
			dlg->m_list.SetItemText(JustnowItem,6,"Extended");				
		}
		else//标准帧
		{
			ReceivedID=GET_ID(pReceive[num].frame.can_id);
			str1.Format("0x%03X",ReceivedID&0x7FF);
			dlg->m_list.SetItemText(JustnowItem,4,str1);	//ID信息	
			dlg->m_list.SetItemText(JustnowItem,6,"Standard");
		}
		
		//长度信息
		str.Format("%d",pReceive[num].frame.can_dlc);	
		dlg->m_list.SetItemText(JustnowItem,7,str);	
		
		//协议
		dlg->m_list.SetItemText(JustnowItem,8,"CAN");
				
		//数据信息
		str="";
		for(int i=0;i<(pReceive[num].frame.can_dlc);i++)	
		{
			str1.Format("%02X ",pReceive[num].frame.data[i]);
			str+=str1;
		}				

		dlg->m_list.SetItemText(JustnowItem,9,str);	
		dlg->m_list.Scroll(size); 	
		 //接收信息列表显示完毕
	}	
}

//接收信息列表显示
static void disp_recv_canfd_frame(BYTE ch, ZCAN_ReceiveFD_Data* pReceive, UINT NumValue)
{
	CDemoCANDlg *dlg=(CDemoCANDlg*) AfxGetApp()->GetMainWnd();	
	
	CSize size;
	unsigned int JustnowItem;
	DWORD ReceivedID;
	size.cx=0;
	size.cy=50;
	CString str;
	CString str1;
	
	CString strTime; 
	SYSTEMTIME   systime;   
	GetLocalTime(&systime);   
	strTime.Format("%02d:%02d:%02d:%03d", systime.wHour,systime.wMinute,systime.wSecond,systime.wMilliseconds);   
				
	for(int num=0;num<NumValue;num++)
	{
		if(nextrow==59999)
		{
			dlg->m_list.DeleteAllItems();
			nextrow=0;
		}

		//序号
		str.Format("%d",nextrow);
		dlg->m_list.ItemColorFlag[nextrow]=0;				
		JustnowItem=dlg->m_list.InsertItem(nextrow,str);	
		nextrow++;	
		
		//时间-
		dlg->m_list.SetItemText(JustnowItem,1,strTime);	
		
		//CAN通道
		str.Format("%d",ch);
		dlg->m_list.SetItemText(JustnowItem,2,str);	
		
		//方向-Rx
		dlg->m_list.SetItemText(JustnowItem,3,"Recv");	
		str="";

		//远程帧 
		if(IS_RTR(pReceive[num].frame.can_id)==1)
		{
			dlg->m_list.SetItemText(JustnowItem,5,"Remote");	
		}
		else
		{
			dlg->m_list.SetItemText(JustnowItem,5,"Data");	
		
		}
		// 扩展帧
		if(IS_EFF(pReceive[num].frame.can_id)==1)		
		{
			ReceivedID=GET_ID(pReceive[num].frame.can_id);
			str1.Format("0x%08X",ReceivedID&0x1FFFFFFF);
			dlg->m_list.SetItemText(JustnowItem,4,str1);	//ID信息	
					
			dlg->m_list.SetItemText(JustnowItem,6,"Extended");				
		}
		else//标准帧
		{
			ReceivedID=GET_ID(pReceive[num].frame.can_id);
			str1.Format("0x%03X",ReceivedID&0x7FF);
			dlg->m_list.SetItemText(JustnowItem,4,str1);	//ID信息	
			dlg->m_list.SetItemText(JustnowItem,6,"Standard");
		}
		
		//长度信息
		str.Format("%d",pReceive[num].frame.len);	
		dlg->m_list.SetItemText(JustnowItem,7,str);	
		
		//协议
		dlg->m_list.SetItemText(JustnowItem,8,"CANFD");
				
		//数据信息
		str="";
		for(int i=0;i<(pReceive[num].frame.len);i++)	
		{
			str1.Format("%02X ",pReceive[num].frame.data[i]);
			str+=str1;
		}				

		dlg->m_list.SetItemText(JustnowItem,9,str);	
		dlg->m_list.Scroll(size); 	
		 //接收信息列表显示完毕
	}	
}

UINT CDemoCANDlg::ReceiveThread(LPVOID v)
{
	CDemoCANDlg *dlg=(CDemoCANDlg*) AfxGetApp()->GetMainWnd();	
	int k=0;
	int i;

	DWORD can0_num=0, can0fd_num=0, can1_num=0, can1fd_num=0;
	
	dlg->SetDlgItemInt(IDC_RECV_NUM, RV_CAN0_NUMS, TRUE);
	dlg->SetDlgItemInt(IDC_RECVFD_NUM, RV_CANFD0_NUMS, TRUE);
	dlg->SetDlgItemInt(IDC_RECV_CAN1_NUM, RV_CAN1_NUMS, TRUE);
	dlg->SetDlgItemInt(IDC_RECV_CANFD1_NUM, RV_CANFD1_NUMS, TRUE);

	while(1)
	{							
		//获取通道1缓冲区CAN报文数目
		can0_num=ZCAN_GetReceiveNum(dev_ch1,0);
		if(can0_num)			
		{
			UINT ReadLen=0; 
			//如果缓冲区有数据就读取
			ReadLen = ZCAN_Receive(dev_ch1, pCanObj0, can0_num, 50);				
			RV_CAN0_NUMS += ReadLen;
			can0_num = 0;
			dlg->SetDlgItemInt(IDC_RECV_NUM, RV_CAN0_NUMS, TRUE);
			disp_recv_can_frame(0, pCanObj0, ReadLen);
		}
		
		//获取通道1缓冲区CANFD报文数目
		can0fd_num=ZCAN_GetReceiveNum(dev_ch1,1);
		if(can0fd_num)			
		{
			UINT ReadLen=0; 
			//如果缓冲区有数据就读取
			ReadLen = ZCAN_ReceiveFD(dev_ch1, pCanObjFD0, can0fd_num, 50);
			RV_CANFD0_NUMS += ReadLen;
			can0fd_num = 0;
			dlg->SetDlgItemInt(IDC_RECVFD_NUM, RV_CANFD0_NUMS, TRUE);
			disp_recv_canfd_frame(0, pCanObjFD0, ReadLen);
		}

		can1_num=ZCAN_GetReceiveNum(dev_ch2,0);
		if(can1_num)
		{
			UINT ReadLen=0;
			ReadLen = ZCAN_Receive(dev_ch2, pCanObj1, can1_num, 50);
			RV_CAN1_NUMS += ReadLen;
			can1_num = 0;
			dlg->SetDlgItemInt(IDC_RECV_CAN1_NUM, RV_CAN1_NUMS, TRUE);
			disp_recv_can_frame(1, pCanObj1, ReadLen);
		}

		can1fd_num=ZCAN_GetReceiveNum(dev_ch2,1);
		if(can1fd_num)			
		{
			UINT ReadLen = 0;
			ReadLen = ZCAN_ReceiveFD(dev_ch2, pCanObjFD1, can1fd_num, 50);
			RV_CANFD1_NUMS += ReadLen;
			can1fd_num = 0;
			dlg->SetDlgItemInt(IDC_RECV_CANFD1_NUM, RV_CANFD1_NUMS, TRUE);
			disp_recv_canfd_frame(1, pCanObjFD1, ReadLen);
		}					
		/*
		if(StopFlag==1)
			return 0;	
		*/
	}

	return 1;
}
#endif

void CDemoCANDlg::OnCheckCanrxEn() 
{
	if(m_dev == INVALID_DEVICE_HANDLE)
		return ;

	UpdateData(TRUE);
	if(m_bCanRxEn)
	{
		StopFlag=0;
		//开启接收线程
		AfxBeginThread(ReceiveThread,0);	
	}
	else
		StopFlag=1;
}
//int danT=0;
//清空信息显示列表
void CDemoCANDlg::OnButtonClear() 
{
//	UpdateData(TRUE);
	m_list.DeleteAllItems();	
	//danT++;
	RV_CAN0_NUMS = 0;
	RV_CANFD0_NUMS=0;
 	RV_CAN1_NUMS=0;
 	RV_CANFD1_NUMS=0;
	
	SetDlgItemInt(IDC_RECV_NUM, RV_CAN0_NUMS, TRUE);
	SetDlgItemInt(IDC_RECVFD_NUM, RV_CANFD0_NUMS, TRUE);
	SetDlgItemInt(IDC_RECV_CAN1_NUM, RV_CAN1_NUMS, TRUE);
	SetDlgItemInt(IDC_RECV_CANFD1_NUM, RV_CANFD1_NUMS, TRUE);
	
}


/*
void CDemoCANDlg::OnCbnSelchangeComboDevtype()
{
	// TODO: 在此添加控件通知处理程序代码
	int i;
	i=i;
}
*/