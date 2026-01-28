//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop
#include <string>
#include "Unit1.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TForm1 *Form1;


/*
下面是具体的调用动态库的做法，采用的是动态加载的方法，主要就是在程序初始化的
时候从动态库中取得各个函数的地址并保存起来，然后在需要的时候调用就可以了，最后在
程序退出的时候释放掉打开的动态库句柄就行。
*/

/*---------------------------兼容ZLG的及数据类型------------------------------*/
//首先定义需要用到的数据结构
//USB-CAN总线适配器板卡信息的数据类型。
#define ZCAN_USBCANFD_200U			41
#define VCI_USBCAN_E_U 				20
#define VCI_USBCAN_2E_U 			21

//函数调用返回状态值
#define STATUS_ERR					0
#define	STATUS_OK					1
#define STATUS_ONLINE               2
#define	STATUS_OFFLINE              3
#define STATUS_UNSUPPORTED          4

#define INVALID_CHANNEL_HANDLE 		0
#define INVALID_DEVICE_HANDLE 		0

/* special address description flags for the MAKE_CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */ 
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */
#define CAN_ID_FLAG  0x1FFFFFFFU /* id */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */
// make id
#define MAKE_CAN_ID(id, eff, rtr, err) (id | (!!(eff) << 31) | (!!(rtr) << 30) | (!!(err) << 29))
#define IS_EFF(id) (!!(id & CAN_EFF_FLAG)) //1:extend frame 0:standard frame
#define IS_RTR(id) (!!(id & CAN_RTR_FLAG)) //1:remote frame 0:data frame
#define IS_ERR(id) (!!(id & CAN_ERR_FLAG)) //1:error frame 0:normal frame
#define GET_ID(id) (id & CAN_ID_FLAG)

#define TYPE_CAN   0
#define TYPE_CANFD 1
#define CAN_MAX_DLEN		8
#define CANFD_MAX_DLEN		64
#define FUNC_CALL __stdcall
struct _Meta;
struct _Pair;
struct _Options;
struct _ConfigNode;

typedef struct _Meta Meta;
typedef struct _Pair Pair;
typedef struct _Options Options;
typedef struct _ConfigNode ConfigNode;


struct _Options
{
	const char * type;

	const char * value;

	const char * desc;
};


struct _Meta
{
	const char * type;

	const char * desc;

	int read_only;

	const char * format;

	double min_value;

	double max_value;

	const char * unit;

	double delta;

        const char* visible;

        const char* enable;

	int editable;

	Options** options;
};

struct _Pair
{
	const char * key;
	const char * value;
};

struct _ConfigNode
{
	const char * name;
	const char * value;
        const char* binding_value;
	const char * path;
	Meta* meta_info;
	ConfigNode** children;
	Pair** attributes;
};
typedef struct _can_frame{
	UINT	can_id;  /* 32 bit MAKE_CAN_ID + EFF/RTR/ERR flags */
	BYTE    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	BYTE    __pad;   /* padding */
	BYTE    __res0;  /* reserved / padding */
	BYTE    __res1;  /* reserved / padding */
	BYTE    data[CAN_MAX_DLEN]/* __attribute__((aligned(8)))*/;
}can_frame,*Pcan_frame;

typedef struct _canfd_frame{
	UINT    can_id;  /* 32 bit MAKE_CAN_ID + EFF/RTR/ERR flags */
	BYTE    len;     /* frame payload length in byte */
	BYTE    flags;   /* additional flags for CAN FD,i.e error code */
	BYTE    __res0;  /* reserved / padding */
	BYTE    __res1;  /* reserved / padding */
	BYTE    data[CANFD_MAX_DLEN]/* __attribute__((aligned(8)))*/;
}canfd_frame,*Pcanfd_frame;
typedef struct _ZCAN_Transmit_Data
{
    can_frame frame;
    UINT transmit_type;
}ZCAN_Transmit_Data,*PZCAN_Transmit_Data;

typedef struct _ZCAN_Receive_Data
{
    can_frame frame;
    UINT64    timestamp;//us
}ZCAN_Receive_Data,*PZCAN_Receive_Data;

typedef struct _ZCAN_TransmitFD_Data
{
    canfd_frame frame;
    UINT transmit_type;
}ZCAN_TransmitFD_Data,*PZCAN_TransmitFD_Data;

typedef struct _ZCAN_ReceiveFD_Data
{
    canfd_frame frame;
    UINT64      timestamp;//us
}ZCAN_ReceiveFD_Data,*PZCAN_ReceiveFD_Data;

typedef const ConfigNode* (*GetPropertysFunc)();

typedef int (*SetValueFunc)(const char* path, const char* value);

typedef const char* (*GetValueFunc)(const char* path);

typedef struct  _IProperty
{
	SetValueFunc     SetValue;
	GetValueFunc     GetValue;
	GetPropertysFunc GetPropertys;
}IProperty,*PIProperty;

typedef unsigned long DEVICE_HANDLE;
typedef void * CHANNEL_HANDLE;

typedef  struct  _ZCAN_DEVICE_INFO{
		USHORT	hw_Version;
		USHORT	fw_Version;
		USHORT	dr_Version;
		USHORT	in_Version;
		USHORT	irq_Num;
		BYTE	can_Num;
		CHAR	str_Serial_Num[20];
		CHAR	str_hw_Type[40];
		USHORT	Reserved[4];
}ZCAN_DEVICE_INFO,*PZCAN_DEVICE_INFO;



typedef struct _ZCAN_CHANNEL_INIT_CONFIG {
        UINT can_type;
        union
        {
                struct
                {
                        UINT  acc_code;
                        UINT  acc_mask;
                        BYTE  filter;
                        BYTE  timing0;
                        BYTE  timing1;
                        BYTE  mode;
                }can;
                struct
                {
                        UINT  acc_code;
                        UINT  acc_mask;
                        UINT  abit_timing;
                        UINT  dbit_timing;
                        UINT  brp;
                        BYTE  filter;
                        BYTE  mode;
                        USHORT  pad;
                        UINT  reserved;
                }canfd;
        };
}ZCAN_CHANNEL_INIT_CONFIG,*PZCAN_CHANNEL_INIT_CONFIG;


typedef DWORD (CALLBACK*  LPZCAN_OpenDevice)(DWORD,DWORD,DWORD);
typedef DWORD (CALLBACK*  LPZCAN_CloseDevice)(DEVICE_HANDLE);

typedef CHANNEL_HANDLE (CALLBACK*  LPZCAN_InitCan)(DWORD,DWORD,PZCAN_CHANNEL_INIT_CONFIG);


typedef DWORD (CALLBACK*  LPZCAN_StartCAN)(CHANNEL_HANDLE);
typedef DWORD (CALLBACK*  LPZCAN_ResetCAN)(CHANNEL_HANDLE);

typedef ULONG (CALLBACK*  LPZCAN_Transmit)(CHANNEL_HANDLE,PZCAN_Transmit_Data,ULONG);
typedef ULONG (CALLBACK*  LPZCAN_TransmitFD)(CHANNEL_HANDLE,PZCAN_TransmitFD_Data,ULONG);
typedef DWORD (CALLBACK*  LPZCAN_ReceiveFD)(CHANNEL_HANDLE,PZCAN_ReceiveFD_Data,ULONG,INT);
typedef DWORD (CALLBACK*  LPZCAN_Receive)(CHANNEL_HANDLE,PZCAN_Receive_Data,ULONG,INT);
typedef PIProperty (CALLBACK*  LPGetIProperty)(ULONG);
typedef DWORD (CALLBACK*  LPReleaseIProperty)(PIProperty);



//其他函数
//////////////////////////////////////////////////////////////////////////

HANDLE m_hDLL;//用来保存打开的动态库句柄

//接口函数指针
LPZCAN_OpenDevice ZCAN_OpenDevice;
LPZCAN_CloseDevice ZCAN_CloseDevice;
LPZCAN_InitCan ZCAN_InitCAN;
LPZCAN_StartCAN ZCAN_StartCAN;
LPZCAN_ResetCAN ZCAN_ResetCAN;
LPZCAN_ReceiveFD ZCAN_ReceiveFD;
LPZCAN_Receive ZCAN_Receive;
LPZCAN_TransmitFD ZCAN_TransmitFD;
LPZCAN_Transmit ZCAN_Transmit;
LPGetIProperty GetIProperty;
LPReleaseIProperty ReleaseIProperty;
////////////////////////////////////////////////////////////////////////////
DWORD m_devind=0;
DWORD m_cannum=0;
int m_connect=0;
DEVICE_HANDLE a;
PZCAN_CHANNEL_INIT_CONFIG CanFDinit = new ZCAN_CHANNEL_INIT_CONFIG;
CHANNEL_HANDLE dev_1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
        : TForm(Owner)
{
        m_hDLL = LoadLibrary("ControlCANFD.dll");//打开动态库

        //取得函数地址
        ZCAN_OpenDevice=(LPZCAN_OpenDevice)GetProcAddress(m_hDLL,"ZCAN_OpenDevice");
        ZCAN_CloseDevice=(LPZCAN_CloseDevice)GetProcAddress(m_hDLL,"ZCAN_CloseDevice");
        ZCAN_InitCAN=(LPZCAN_InitCan)GetProcAddress(m_hDLL,"ZCAN_InitCAN");
        ZCAN_StartCAN=(LPZCAN_StartCAN)GetProcAddress(m_hDLL,"ZCAN_StartCAN");
        ZCAN_ResetCAN=(LPZCAN_ResetCAN)GetProcAddress(m_hDLL,"ZCAN_ResetCAN");
        ZCAN_Transmit=(LPZCAN_Transmit)GetProcAddress(m_hDLL,"ZCAN_Transmit");
        ZCAN_TransmitFD=(LPZCAN_TransmitFD)GetProcAddress(m_hDLL,"ZCAN_TransmitFD");
        ZCAN_ReceiveFD=(LPZCAN_ReceiveFD)GetProcAddress(m_hDLL,"ZCAN_ReceiveFD");
        ZCAN_Receive=(LPZCAN_Receive)GetProcAddress(m_hDLL,"ZCAN_Receive");
        GetIProperty=(LPGetIProperty)GetProcAddress(m_hDLL,"GetIProperty");
        ReleaseIProperty=(LPReleaseIProperty)GetProcAddress(m_hDLL,"ReleaseIProperty");
}
void __fastcall TForm1::FormClose(TObject *Sender, TCloseAction &Action)
{
        if(m_connect==1)
        {
                m_connect=0;
                WaitForSingleObject(m_readhandle,2000);
                m_readhandle=NULL;
                ZCAN_CloseDevice(a);
        }

        FreeLibrary(m_hDLL);//释放动态库句柄
}
void __fastcall TForm1::EnableUI(BOOL bEnable)
{
        Label8->Enabled = bEnable;
        ComboBox1->Enabled = bEnable;
        Label9->Enabled = bEnable;
        ComboBox2->Enabled = bEnable;
        Label1->Enabled = bEnable;
        Edit2->Enabled = bEnable;
        Label2->Enabled = bEnable;
        Edit3->Enabled = bEnable;
        Label12->Enabled = bEnable;
        ComboBox3->Enabled = bEnable;
        Label13->Enabled = bEnable;
        ComboBox4->Enabled = bEnable;
        Label10->Enabled = bEnable;
        Edit5->Enabled = bEnable;
        Label11->Enabled = bEnable;
        Edit6->Enabled = bEnable;
}

//---------------------------------------------------------------------------
void ReceiveThread(void *param)
{
   TListBox *box=(TListBox*)param;
   ZCAN_ReceiveFD_Data receivedataFD[2500];
   int len,i;
   AnsiString str,tmpstr;
   while(1)
   {

        if(m_connect==0)
                break;
        Sleep(1);

        len=ZCAN_ReceiveFD(dev_1,receivedataFD,2500,20);
        if(len<=0)
        {

        }
        else
        {
	        for(i=0;i<len;i++)
		{
			str="接收到数据帧:  ";
			if(receivedataFD[i].timestamp==0)
				tmpstr="时间标识:无  ";
			else
				tmpstr="时间标识:0x"+IntToHex((int)receivedataFD[i].timestamp,1)+" ";
			str+=tmpstr;
			tmpstr="帧ID:0x"+IntToHex((int)receivedataFD[i].frame.can_id,1)+" ";
			str+=tmpstr;
			str+="帧格式:";
			if(IS_RTR(receivedataFD[i].frame.can_id)==0)
				tmpstr="数据帧 ";
			else
				tmpstr="远程帧 ";
			str+=tmpstr;
			str+="帧类型:";
			if(IS_EFF(receivedataFD[i].frame.can_id)==0)
				tmpstr="标准帧 ";
			else
				tmpstr="扩展帧 ";
			str+=tmpstr;
			box->Items->Add(str);
			if(IS_RTR(receivedataFD[i].frame.can_id)==0)
			{
				str="数据FD:";
                                if(receivedataFD[i].frame.len>64)
                                        receivedataFD[i].frame.len=64;
				for(int j=0;j<receivedataFD[i].frame.len;j++)
				{
					tmpstr=IntToHex((int)receivedataFD[i].frame.data[j],2)+" ";
					str+=tmpstr;
				}
				box->Items->Add(str);
			}
		}
		box->ItemIndex=box->Items->Count-1;
        }
   }
   _endthread();
}
void ReceiveThread1(void *param)
{
   TListBox *box=(TListBox*)param;
   ZCAN_Receive_Data receivedata[2500];
   int len,i;
   AnsiString str,tmpstr;
   while(1)
   {

        if(m_connect==0)
                break;
        Sleep(1);

        len=ZCAN_Receive(dev_1,receivedata,2500,20);
        if(len<=0)
        {

        }
        else
        {
	        for(i=0;i<len;i++)
		{
			str="接收到数据帧:  ";
			if(receivedata[i].timestamp==0)
				tmpstr="时间标识:无  ";
			else
				tmpstr="时间标识:0x"+IntToHex((int)receivedata[i].timestamp,1)+" ";
			str+=tmpstr;
			tmpstr="帧ID:0x"+IntToHex((int)receivedata[i].frame.can_id,1)+" ";
			str+=tmpstr;
			str+="帧格式:";
			if(IS_RTR(receivedata[i].frame.can_id)==0)
				tmpstr="数据帧 ";
			else
				tmpstr="远程帧 ";
			str+=tmpstr;
			str+="帧类型:";
			if(IS_EFF(receivedata[i].frame.can_id)==0)
				tmpstr="标准帧 ";
			else
				tmpstr="扩展帧 ";
			str+=tmpstr;
			box->Items->Add(str);
			if(IS_RTR(receivedata[i].frame.can_id)==0)
			{
				str="数据:";
                                if(receivedata[i].frame.can_dlc>8)
                                        receivedata[i].frame.can_dlc=8;
				for(int j=0;j<receivedata[i].frame.can_dlc;j++)
				{
					tmpstr=IntToHex((int)receivedata[i].frame.data[j],2)+" ";
					str+=tmpstr;
				}
				box->Items->Add(str);
			}
		}
		box->ItemIndex=box->Items->Count-1;
        }
   }
   _endthread();
}

void __fastcall TForm1::Button1Click(TObject *Sender)
{
        if(m_connect==1)
        {
                Button1->Caption ="连接";
                m_connect=0;
                WaitForSingleObject(m_readhandle,2000);
                //m_readhandle=NULL;
                ZCAN_CloseDevice(a);
                EnableUI(TRUE);
                return;
        }
        int index = 0;
        int cannum = ComboBox2->ItemIndex;
        CanFDinit->can_type=TYPE_CANFD;
        CanFDinit->canfd.acc_code=StrToInt("0x"+Edit2->Text);
        CanFDinit->canfd.acc_mask=StrToInt("0x"+Edit3->Text);
        CanFDinit->canfd.filter=ComboBox3->ItemIndex;
        CanFDinit->canfd.mode=ComboBox4->ItemIndex;
        CanFDinit->canfd.brp=0;


        if(index>=0&&cannum>=0)
        {
                a = ZCAN_OpenDevice(ZCAN_USBCANFD_200U,index,0);
                if(a != INVALID_DEVICE_HANDLE)
                {
                        IProperty * _pPro = GetIProperty(a);
                        const char * str;
                        if(_pPro == NULL)
                        {
                                ShowMessage("获取信息错误");
                                return;
                        }
                        if(STATUS_OK != _pPro->SetValue("0/canfd_abit_baud_rate",Edit5->Text.c_str()))
                        {
		                ShowMessage("Set ch0 rateA failed!");
		                ReleaseIProperty(_pPro);
		                return;
                        }
                        if(STATUS_OK != _pPro->SetValue("0/canfd_dbit_baud_rate",Edit6->Text.c_str()))
                        {
		                ShowMessage("Set ch0 rateB failed!");
		                ReleaseIProperty(_pPro);
		                return;
                        }
                        dev_1 = ZCAN_InitCAN(a,m_cannum,CanFDinit);
                        if(dev_1!=0)
                        {
                                Button1->Caption ="断开";
                                m_connect=1;
                                m_devind=0;
                                m_cannum=cannum;
                        }
                        else
                        {
                                ShowMessage("初始化CAN错误");
                        }
                }
                else
                {
                        ShowMessage("打开端口错误");
                }

        }
        EnableUI(FALSE);
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button2Click(TObject *Sender)
{
        if(m_connect==0)
        {
                ShowMessage("请先打开端口");
                return;
        }
        if(ZCAN_ResetCAN(dev_1)==1)
        {
                ListBox1->Items->Add("复位CAN成功");
                Button6->Enabled = FALSE;                
                Button4->Enabled = FALSE;
        }
        else
        {
                ListBox1->Items->Add("复位CAN失败");
        }
        ListBox1->ItemIndex=ListBox1->Items->Count-1;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button3Click(TObject *Sender)
{
        if(m_connect==0)
        {
                ShowMessage("请先打开端口");
                return;
        }
        if(ZCAN_StartCAN(dev_1)==1)
        {
                ListBox1->Items->Add("启动CAN成功");
                m_readhandle=(HANDLE)_beginthread(ReceiveThread,0,(void*)ListBox1);
                m_readhandle=(HANDLE)_beginthread(ReceiveThread1,0,(void*)ListBox1);
                Button6->Enabled = TRUE;
                Button4->Enabled = TRUE;
        }
        else
        {
                ListBox1->Items->Add("启动CAN失败");
        }
        ListBox1->ItemIndex=ListBox1->Items->Count-1;
}
//---------------------------------------------------------------------------





void __fastcall TForm1::FormCreate(TObject *Sender)
{
      ComboBox1->ItemIndex = 1;
      ComboBox2->ItemIndex = 0;
      ComboBox3->ItemIndex = 0;
      ComboBox4->ItemIndex = 0;
      ComboBox6->ItemIndex =0;
      ComboBox7->ItemIndex =0;
      EnableUI(TRUE);
      Button4->Enabled = FALSE;
      Button6->Enabled = FALSE;
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------

//---------------------------------------------------------------------------


void __fastcall TForm1::Button4Click(TObject *Sender)
{
        if(m_connect==0)
        {
                ShowMessage("请先打开端口");
                return;
        }

        BYTE sendtype,frametype,frameformat;
        DWORD id;
        BYTE data[8];

        frametype=ComboBox6->ItemIndex;
        frameformat=ComboBox7->ItemIndex;
        id=StrToInt("0x"+Edit1->Text);

        AnsiString str=Edit4->Text;
        AnsiString strdata;
        int i,kkk;
        for(i=0;i<8;i++)
        {
                strdata=str.SubString(3*i+1,2);
                strdata=strdata.Trim();
                kkk=strdata.Length();
                if(kkk==0)
                {
                        goto exit;
                }
                data[i]=StrToInt(strdata);
                //sscanf(strdata.c_str(),"%x",data+i);
        }

exit:
        ZCAN_TransmitFD_Data senddata;
        memset(senddata.frame.data, 0, sizeof(senddata.frame.data));
        senddata.frame.can_id = MAKE_CAN_ID(id,frametype,frameformat,0);
        senddata.frame.len = i;
        senddata.frame.flags = 0;
        memcpy(senddata.frame.data,data,i);

        if(ZCAN_TransmitFD(dev_1,&senddata,1)==1)
        {
                ListBox1->Items->Add("发送成功");
        }
        else
        {
                ListBox1->Items->Add("发送失败");
        }
        ListBox1->ItemIndex=ListBox1->Items->Count-1;
}
void __fastcall TForm1::Button7Click(TObject *Sender)
{
        if(m_connect==0)
        {
                ShowMessage("请先打开端口");
                return;
        }

        BYTE sendtype,frametype,frameformat;
        DWORD id;
        BYTE data[8];

        frametype=ComboBox6->ItemIndex;
        frameformat=ComboBox7->ItemIndex;
        id=StrToInt("0x"+Edit1->Text);

        AnsiString str=Edit4->Text;
        AnsiString strdata;
        int i,kkk;
        for(i=0;i<8;i++)
        {
                strdata=str.SubString(3*i+1,2);
                strdata=strdata.Trim();
                kkk=strdata.Length();
                if(kkk==0)
                {
                        goto exit;
                }
                data[i]=StrToInt(strdata);
                //sscanf(strdata.c_str(),"%x",data+i);
        }

exit:
        ZCAN_Transmit_Data senddata;
        memset(senddata.frame.data, 0, sizeof(senddata.frame.data));
        senddata.frame.can_id = MAKE_CAN_ID(id,frametype,frameformat,0);
        senddata.frame.can_dlc = i;
        memcpy(senddata.frame.data,data,i);

        if(ZCAN_Transmit(dev_1,&senddata,1)==1)
        {
                ListBox1->Items->Add("发送成功");
        }
        else
        {
                ListBox1->Items->Add("发送失败");
        }
        ListBox1->ItemIndex=ListBox1->Items->Count-1;
}
//---------------------------------------------------------------------------













void __fastcall TForm1::Button5Click(TObject *Sender)
{
        ListBox1->Clear();
}
//---------------------------------------------------------------------------

