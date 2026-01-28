{#  ------------------------------------------------------------------
#  Author : zhaodong
#  Last change: 11.06.2023
#
#  Language: delphi 2007
#  ------------------------------------------------------------------}
unit Unit2;

interface

uses
  Windows,
  Messages,
  SysUtils,
  Variants,
  Classes,
  Graphics,
  Controls,
  Forms,
  Dialogs,
  StdCtrls,
  ControlCANFD;
  

type
  TForm2 = class(TForm)
    start_Button: TButton;
    can_index_box: TComboBox;
    device_index_box: TComboBox;
    reset_Button: TButton;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label5: TLabel;
    mode_box: TComboBox;
    device_setting: TGroupBox;
    can_type_box: TComboBox;
    Label4: TLabel;
    canfd_standard_box: TComboBox;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    data_display: TGroupBox;
    device_type_box: TComboBox;
    ListBox1: TListBox;
    data_sent_setting: TGroupBox;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    id_edit: TEdit;
    data_edit: TEdit;
    eff_box: TComboBox;
    send_type_box: TComboBox;
    abit_baud_box: TEdit;
    dbit_baud_box: TEdit;
    Label14: TLabel;
    rtr_box: TComboBox;
    send_Button: TButton;
    filter_check: TCheckBox;
    Label15: TLabel;
    filtermode_box: TComboBox;
    Label16: TLabel;
    Label17: TLabel;
    filter_start_edit: TEdit;
    filter_end_edit: TEdit;
    filer_setting: TGroupBox;
    canfd_brs: TCheckBox;
    getdevinf_Button: TButton;
    data_clear_Button: TButton;
    procedure FormCreate(Sender: TObject);
    procedure start_ButtonClick(Sender: TObject);
    procedure reset_ButtonClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var Action: TCloseAction);
    procedure send_ButtonClick(Sender: TObject);
    procedure getdevinf_ButtonClick(Sender: TObject);
    procedure data_clear_ButtonClick(Sender: TObject);
    procedure mode_boxChange(Sender: TObject);


  private
    { Private declarations }
  public
    { Public declarations }
  end;

   PTListBox=^TListBox;

   
  //global val
var
  Form2: TForm2;
  device_handle,init_handle:THandle;
  device_type,device_index,can_index,reserved:Dword;
  m_arrdevtype:array[0..50] of integer;  {设备选项}
  m_connect : DWORD;
  m_threadhandle : integer;
  property_: IProperty;


implementation

{$R *.dfm}




{   ID = CAN_ID + EFF/RTR/ERR flags,Dword ,Total 32 bit

The high 3 bits belong to the flag bit, and the meaning of the flag bit is as follows:
--The 31st bit (highest bit) represents the extended frame flag,=0 represents the standard frame,=1 represents the extended frame, and the function IS_ EFF can obtain this flag;
--The 30th bit represents the remote frame flag,=0 represents the data frame,=1 represents the remote frame, and the function IS_ RTR can obtain this flag;
--The 29th digit represents the error frame standard,=0 represents the CAN frame, and=1 represents the error frame. Currently, it can only be set to 0;
--The remaining bits represent the actual frame ID value, using the macro MAKE_ CAN_ Construct ID using the function GET_ ID acquisition ID
}

function MAKE_CAN_ID(id:Dword; eff, rtr, err:Integer):DWord;
begin
  result :=  id or (eff shl 31) or (rtr shl 30) or (err shl 29);
end;

function IS_EFF(id: Dword):DWord;
begin
  result := id and CAN_EFF_FLAG;
end;

function IS_RTR(id: Dword):DWord;
begin
  result := id and CAN_RTR_FLAG;
end;

function GET_ID(id: Dword):DWord;
begin
  result := id and CAN_ID_FLAG;
end;


//Recv Func
function ReceiveThread(param : Pointer): integer;
var
receivedata : array[0..199] of ZCAN_Receive_Data;
receivedata_fd : array[0..199] of ZCAN_ReceiveFD_Data;
j,i,len,len_can,len_canfd: integer;
dev_type:Dword;
str : AnsiString;
tmpstr :AnsiString;
box : PTListBox;
begin
  box:=param;
   while TRUE do
    begin
        if m_connect=0 then
          break;
        Sleep(1);
         len_can := ZCAN_GetReceiveNum(init_handle,0);
         len_canfd := ZCAN_GetReceiveNum(init_handle,1);
        if ((len_can <=0) and (len_canfd <=0))  then
        begin
            continue;
        end;
              //接收CAN帧
             if len_can>0 then begin
              {Get CAN data，parse to display}
              len:=ZCAN_Receive(init_handle,@receivedata[0],len_can,-1);
              for i:=0 to len-1 do
                begin
                  str:='Recved CAN:  ';
                  tmpstr:='ID:0x'+IntToHex(GET_ID(receivedata[i].can_id),8)+' ';
                  str:=str+tmpstr;
                  str:=str+'Format:';
                  if IS_EFF(receivedata[i].can_id)=0 then
                    tmpstr:='Standard '
                  else
                    tmpstr:='Extended ';
                  str:=str+tmpstr;
                  str:=str+'Type:';
                  if IS_RTR(receivedata[i].can_id)=0 then
                    tmpstr:='Data '
                  else
                    tmpstr:='Remote ';
                  str:=str+tmpstr;
                  box.Items.Add(str);
                  if IS_RTR(receivedata[i].can_id)=0 then {if data frame, then display, remote frame has no data}
                  begin
                    str:='Data:';
                    if receivedata[i].can_dlc>8 then
                      receivedata[i].can_dlc:=8;
                    for j:=0 to receivedata[i].can_dlc-1 do
                      begin
                        tmpstr:=IntToHex(receivedata[i].data[j],2)+' ';
                        str:=str+tmpstr;
                      end;
                     box.Items.Add(str);
                  end;
                end;
              box.ItemIndex:=box.Items.Count-1;

            end else begin
              {Get CANFD data，parse to display}
              len:=ZCAN_ReceiveFD(init_handle,@receivedata_fd[0],len_canfd,-1);
              for i:=0 to len-1 do
                begin
                  str:='Recved CANFD:  ';
                  tmpstr:='ID:0x'+IntToHex(GET_ID(receivedata_fd[i].can_id),8)+' ';
                  str:=str+tmpstr;
                  str:=str+'Format:';
                  if IS_EFF(receivedata_fd[i].can_id)=0 then
                    tmpstr:='Standard '
                  else
                    tmpstr:='Extended ';
                  str:=str+tmpstr;
                  str:=str+'Type:';

                  if IS_RTR(receivedata_fd[i].can_id)=0 then
                    tmpstr:='Data '
                  else
                    tmpstr:='Remote ';
                  str:=str+tmpstr;

                  if receivedata_fd[i].flags =1 then
                    tmpstr:='CANFD BRS '
                  else
                    tmpstr:=' ';
                  str:=str+tmpstr;
                  box.Items.Add(str);

                  if IS_RTR(receivedata_fd[i].can_id)=0 then {if data frame , display; remote frame has no data}
                  begin
                    str:='Data:';

                    for j:=0 to receivedata_fd[i].len-1 do
                      begin
                        tmpstr:=IntToHex(receivedata_fd[i].data[j],2)+' ';
                        str:=str+tmpstr;
                      end;
                     box.Items.Add(str);
                  end;
                end;
              box.ItemIndex:=box.Items.Count-1;
              
            end;
    end;

  EndThread(0);
  ReceiveThread:=0;
end;




 //initial thread -- ui display
procedure TForm2.FormCreate(Sender: TObject);

var
index: integer;
device_handle,init_handle:THandle;
begin
  {init device type}
  index:=0;
  device_type_box.Items.Clear;
  index:=device_type_box.Items.Add( 'USBCANFD');
  m_arrdevtype[index] :=  USBCANFD_TYPE;
{
  index:=device_type_box.Items.Add( 'USBCANFD_100U');
  m_arrdevtype[index] :=  ZCAN_USBCANFD_100U;

  index:=device_type_box.Items.Add( 'USBCANFD_MINI');
  m_arrdevtype[index] :=  ZCAN_USBCANFD_MINI;

  index:=device_type_box.Items.Add( 'USBCAN_2E_U');
  m_arrdevtype[index] :=  ZCAN_USBCAN_2E_U;

  index:=device_type_box.Items.Add( 'USBCAN2');
  m_arrdevtype[index] :=  ZCAN_USBCAN2 ;
}
  {init disp configuration parameters}
  device_type_box.ItemIndex:=0;
  device_index_box.ItemIndex:=0;

  can_index_box.Items.Clear;
  can_index_box.Items.Add('0');
  can_index_box.Items.Add('1');
  can_index_box.ItemIndex:=0;

  mode_box.Items.Clear;
  mode_box.Items.Add('Normal');
  mode_box.Items.Add('ReadOnly');
  mode_box.ItemIndex:=0;

  can_type_box.ItemIndex:=0;
  canfd_standard_box.ItemIndex:=0;

  eff_box.Items.Clear;
  eff_box.Items.Add('Standard');
  eff_box.Items.Add('Extended');
  eff_box.ItemIndex:=0;

  rtr_box.Items.Clear;
  rtr_box.Items.Add('data');
  rtr_box.Items.Add('remote');
  rtr_box.ItemIndex:=0;

  send_type_box.Items.Clear;
  send_type_box.Items.Add('Normal send');
  send_type_box.Items.Add('Single Send');
  send_type_box.Items.Add('Self Send Self Recv');
  send_type_box.Items.Add('Single Self Send Self Recv');
  send_type_box.ItemIndex:=0;

  filtermode_box.Items.Clear;
  filtermode_box.Items.Add('Standard');
  filtermode_box.Items.Add('Extended');
  filtermode_box.ItemIndex:=0;
  {handle init to 0}
  device_handle:=0;
  init_handle:=0;
end;


// get dev info
procedure TForm2.getdevinf_ButtonClick(Sender: TObject);
var
pInfo:ZCAN_DEVICE_INFO;
str : AnsiString;
tmpstr :AnsiString;
Vsion :integer;
begin
 if device_handle<1 then begin
 ListBox1.Items.Add('Please open the device first before obtaining device information.');
 end else begin
 ZCAN_GetDeviceInf (device_handle,@pInfo);

       str:= ' ';
       ListBox1.Items.Add(str);
       str:= 'Dev info as follows:(Hex,100 means V1.00)';
       ListBox1.Items.Add(str);
       str:= '------------------------------------------------------------ ';
       ListBox1.Items.Add(str);
       str:='Hardware Ver: '+ IntToHex(pInfo.hw_Version,3);
       ListBox1.Items.Add(str);
       str:='Firmware Ver: '+ IntToHex(pInfo.fw_Version,3);
       ListBox1.Items.Add(str);
       str:='Driver Ver: '+ IntToHex(pInfo.dr_Version,3);
       ListBox1.Items.Add(str);
       str:='Dll Ver: '+ IntToHex(pInfo.in_Version,3);
       ListBox1.Items.Add(str);
       str:='CAN num: '+ IntToHex(pInfo.can_Num,2);
       ListBox1.Items.Add(str);
       str:= '------------------------------------------------------------ ';
       ListBox1.Items.Add(str);
    end
end;



procedure TForm2.mode_boxChange(Sender: TObject);
begin

end;

//reset thread
procedure TForm2.reset_ButtonClick(Sender: TObject);
begin
 m_connect:=0;
 ZCAN_ClearBuffer(init_handle);
  if ZCAN_ResetCAN (init_handle)>0 then
    ListBox1.Items.Add('reset ok')
    else
    ListBox1.Items.Add('reset failed');
 ZCAN_CloseDevice(device_handle);
 device_handle:=0;
 init_handle:=0;

end;



//start thread
procedure TForm2.start_ButtonClick(Sender: TObject);

var
 threadid: LongWord;
 initconfig : ZCAN_INIT_CONFIG_CAN;
 initconfig_fd : ZCAN_INIT_CONFIG_CANFD;

begin

   device_type:= m_arrdevtype[device_type_box.ItemIndex];
   device_index:=device_index_box.ItemIndex;
   can_index:=can_index_box.ItemIndex;
   reserved:=0;

   {USBCANFD initial}
   if ((device_type=41) or (device_type= 42) or (device_type=43)) then begin
        device_handle:=ZCAN_OpenDevice(device_type,device_index,reserved);
        if device_handle<1 then
           ListBox1.Items.Add('OPEN failed, check if the device is working properly or if it has already been turned on.')
        else
           begin
              property_ := GetIProperty(device_handle); {Return Property Configuration Interface}

              property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/canfd_standard'),PAnsiChar(inttostr(canfd_standard_box.ItemIndex))); {set CANFD standard}


              //init struct
              initconfig_fd.can_type:=1;             {0:CAN , 1:CANFD}
              initconfig_fd.acc_code:=$0;           {USBCANFD Don't care}
              initconfig_fd.acc_mask:=$FFFFFFFF;    {USBCANFD Don't care}
              initconfig_fd.abit_timing:=0;        {USBCANFD Don't care}
              initconfig_fd.dbit_timing:=0;        {USBCANFD Don't care}
              initconfig_fd.brp:=0;                {USBCANFD Don't care}
              initconfig_fd.filter:=0;             {USBCANFD Don't care}
              initconfig_fd.mode:= mode_box.ItemIndex;   {WorkMode，=0Normal,=1Readonly}
              initconfig_fd.pad:=0;
              initconfig_fd.reserved:=0;



              //if custom_baud_check.Checked then begin     {设置波特率}
              //   property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/baud_rate_custom'), PAnsiChar(custom_baud_edit.Text));   {自定义波特率}
              //end else begin
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/canfd_abit_baud_rate'), PAnsiChar(abit_baud_box.Text+'000'));  {仲裁域波特率}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/canfd_dbit_baud_rate'), PAnsiChar(dbit_baud_box.Text+'000'));  {数据域波特率}
              //end;

               {初始化}
              init_handle:=ZCAN_InitCAN(device_handle,can_index,@initconfig_fd);
                 if init_handle<1 then
                 begin
                  ListBox1.Items.Add('Init CAN Failed');
                  Exit;
                 end;

              {终端电阻}
              {
              if resistance.Checked then begin
                property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/initenal_resistance'), '1');
              end;
              }
              


              {filter set ，between ZCAN_InitCAN and ZCAN_StartCAN}
              if filter_check.Checked then begin
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_clear'), '0');  {Clear filter}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_mode'), PAnsiChar(inttostr(filtermode_box.ItemIndex)));  {filter mode}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_start'), PAnsiChar('0x'+filter_start_edit.Text));   {filter start ID}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_end'), PAnsiChar('0x'+filter_end_edit.Text)); {filter end ID, start-end is a group, max 100 groups}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_ack'), '0');   {filter active}
              end;

             if ZCAN_StartCAN(init_handle)>0 then
                ListBox1.Items.Add('Start CAN Success')
             else
                ListBox1.Items.Add('Start CAN Failed');

              ReleaseIProperty(property_); {set over , release , otherwise mem leak}
              m_connect:=1;
              threadid:=111;
              m_threadhandle:=BeginThread(0,0,ReceiveThread,@ListBox1,0,threadid);
             end;

    end else begin
        {USBCANFD Init}
        device_handle:=ZCAN_OpenDevice(device_type,device_index,reserved);
        if device_handle<1 then
           ListBox1.Items.Add('OPEN failed, check if the device is working properly or if it has already been turned on.')
        else
           begin

              initconfig.can_type:=0;             //CAN
              initconfig.acc_code:=$0;           //SJA1000 filter code
              initconfig.acc_mask:=$FFFFFFFF;   //SJA1000 filter mask
              initconfig.reserved:=0;
              initconfig.filter:=0;            //filter mode
              initconfig.timing0:=0;       //rate
              initconfig.timing1:=0;      //rate
              initconfig.mode:= mode_box.ItemIndex;   //WorkMode,=0(normal ), =1 (read only)

              {Config baudrate before ZCAN_InitCAN}
              //if custom_baud_check.Checked then begin
              //  property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/baud_rate_custom'), PAnsiChar(custom_baud_edit.Text));
              //end else begin
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/baud_rate'), PAnsiChar(abit_baud_box.Text+'000'));
              //end;
    
               {Init}
              init_handle:=ZCAN_InitCAN(device_handle,can_index,@initconfig);
                 if init_handle<1 then
                 begin
                  ListBox1.Items.Add('Init CANFD Failed');
                  Exit;
                 end;
                 
              {Filter config，between ZCAN_InitCAN and ZCAN_StartCAN}
              if filter_check.Checked then begin
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_clear'), '0');  {Clear Filter}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_mode'), PAnsiChar(inttostr(filtermode_box.ItemIndex)));  {Filter Mode}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_start'), PAnsiChar('0x'+filter_start_edit.Text));   {Filter Start ID}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_end'), PAnsiChar('0x'+filter_end_edit.Text)); {Filer End ID, start-end is a group，max 100 groups}
                 property_.SetValue(PAnsiChar(inttostr(can_index_box.ItemIndex)+'/filter_ack'), '0');   {Filter active}
              end;

             if ZCAN_StartCAN(init_handle)>0 then
                ListBox1.Items.Add('Start CANFD Success')
             else
                ListBox1.Items.Add('Start CANFD Failed');

              ReleaseIProperty(property_); {Set Over,Release Interface, otherwise cause mem leak}
              m_connect:=1;
              threadid:=111;
              m_threadhandle:=BeginThread(0,0,ReceiveThread,@ListBox1,0,threadid);
             end;
    end;

end;


//send thread
procedure TForm2.send_ButtonClick(Sender: TObject);
var
sendtype:dword;
frametype,frameformat : BYTE;
id: DWORD;
data : array[0..7] of BYTE;
data_fd : array[0..63] of BYTE;
str : AnsiString;
strdata : AnsiString;
senddata : ZCAN_Transmit_Data;
senddata_fd :ZCAN_TransmitFD_Data;
i : integer;
begin

  if m_connect=0 then
    Exit;

   if can_type_box.ItemIndex =0 then begin
        sendtype:=send_type_box.ItemIndex;
        frametype:=eff_box.ItemIndex;
        frameformat:=rtr_box.ItemIndex;

        id:=MAKE_CAN_ID(StrToInt('0x'+id_edit.Text),frametype,frameformat,0);

        str:=data_edit.Text;
        for i:=0 to 7 do
          begin
            strdata:=Copy(str,3*i+1,2);
            strdata:=Trim(strdata);
            if Length(strdata)=0 then
              break;
            data[i]:=StrToInt('0x'+strdata);
          end;
        senddata.can_id:=id;
        senddata.can_dlc:=i;
        senddata.pad:=0;
        senddata.res0:=0;
        senddata.res1:=0;
        senddata.transmit_type:=sendtype;
        Move(data,senddata.data,i);

   if ZCAN_Transmit(init_handle,@senddata,1)>0 then
        ListBox1.Items.Add('Send Success')
   else
        ListBox1.Items.Add('Send Failed');

   end else begin
         {Send CANFD Frame}
        sendtype:=send_type_box.ItemIndex;
        frametype:=eff_box.ItemIndex;
        frameformat:=rtr_box.ItemIndex;

        id:=MAKE_CAN_ID(StrToInt('0x'+id_edit.Text),frametype,frameformat,0);

        str:=data_edit.Text;
        for i:=0 to 63 do
          begin
            strdata:=Copy(str,3*i+1,2);
            strdata:=Trim(strdata);
            if Length(strdata)=0 then
              break;
            data_fd[i]:=StrToInt('0x'+strdata);
          end;
        senddata_fd.can_id:=id;
        senddata_fd.len:=i;
           if canfd_brs.Checked then
               senddata_fd.flags:=$1
           else
              senddata_fd.flags:=0;
        senddata_fd.res0:=0;
        senddata_fd.res1:=0;
        senddata_fd.transmit_type:=sendtype;
        Move(data_fd,senddata_fd.data,i);

        if ZCAN_TransmitFD(init_handle,@senddata_fd,1)>0 then
          ListBox1.Items.Add('CANFD Send Success')
        else
          ListBox1.Items.Add('CANFD Send Failed');
   end;
end;


//Clear Display
procedure TForm2.data_clear_ButtonClick(Sender: TObject);
begin
ListBox1.Items.Clear;
end;


//Close program，Clode CAN device
procedure TForm2.FormClose(Sender: TObject; var Action: TCloseAction);
begin
  if m_connect=1 then
  begin
    m_connect:=0;
    WaitForSingleObject(m_threadhandle,1000);
    m_threadhandle:=0;
    ZCAN_ResetCAN (init_handle);
    ZCAN_CloseDevice(device_handle);
  end
end;


end.

