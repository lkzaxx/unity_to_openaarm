using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using System.Threading;
using System.IO;
using ZLGCAN;

namespace ZLGCANDemo
{
    //窗口类文件
    public partial class CANForm : Form
    {
        const int NULL = 0;
        const int CANFD_BRS = 0x01; /* bit rate switch (second bitrate for payload data) */
        const int CANFD_ESI = 0x02; /* error state indicator of the transmitting node */

        /* CAN payload length and DLC definitions according to ISO 11898-1 */
        const int CAN_MAX_DLC = 8;
        const int CAN_MAX_DLEN = 8;

        /* CAN FD payload length and DLC definitions according to ISO 11898-7 */
        const int CANFD_MAX_DLC = 15;
        const int CANFD_MAX_DLEN = 64;

        const uint CAN_EFF_FLAG = 0x80000000U; /* EFF/SFF is set in the MSB */
        const uint CAN_RTR_FLAG = 0x40000000U; /* remote transmission request */
        const uint CAN_ERR_FLAG = 0x20000000U; /* error message frame */
        const uint CAN_ID_FLAG = 0x1FFFFFFFU; /* id */

        DeviceInfo[] kDeviceType = 
        {
	        new DeviceInfo(Define.ZCAN_USBCANFD_200U, 2),
	        new DeviceInfo(Define.ZCAN_USBCANFD_100U, 1),
	        new DeviceInfo(Define.ZCAN_USBCANFD_MINI, 1)
        };

        uint[] kAbitTiming = 
        {
	        1000000,//1Mbps
	        800000,//800kbps
	        500000,//500kbps
	        250000,//250kbps
	        125000,//125kbps
	        100000,//100kbps
	        50000 //50kbps
        };

        uint[] kDbitTiming = 
        {
	        5000000,//5Mbps
	        4000000,//4Mbps
	        2000000,//2Mbps
	        1000000 //1Mbps
        };

        
        int channel_index_;
        IntPtr device_handle_;
        IntPtr channel_handle_;
        IntPtr channel_handle2_; //CAN2
        IProperty property_;
        recvdatathread recv_data_thread_;
        string list_box_data_;

        bool m_bOpen = false;
        bool m_bStart = false;
        bool m_bCloud = false;

        public CANForm()
        {
            InitializeComponent();
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
        }

        //初始化界面控件
        private void CANForm_Load(object sender, EventArgs e)
        {
            comboBox_device.SelectedIndex = 0;
            comboBox_standard.SelectedIndex = 0;
            comboBox_mode.SelectedIndex = 0;
            comboBox_ABIT.SelectedIndex = 0;
            comboBox_ABIT2.SelectedIndex = 0;
            comboBox_standard2.SelectedIndex = 2;
            comboBox_frametype.SelectedIndex = 0;
            comboBox_protocol.SelectedIndex = 1;
            comboBox_canfd_exp.SelectedIndex = 0;
            comboBox_sendtype.SelectedIndex = 0;
            //comboBox_baud.SelectedIndex = 0;

            //checkBox_ABIT.Checked = false;
            //checkBox_resistance.Checked = false;

            textBox_startid.Text = "00000000";
            textBox_endid.Text = "FFFFFFFF";
            textBox_ID.Text = "00000001";
            textBox_senddata.Text = "00 11 22 33 44 55 66 77";
            
            setComboboxIndex(0, 32, 0);          
        }

        public uint MakeCanId(uint id, int eff, int rtr, int err)//1:extend frame 0:standard frame
        {            
            uint ueff = (uint)(!!(Convert.ToBoolean(eff)) ? 1 : 0);
            uint urtr = (uint)(!!(Convert.ToBoolean(rtr)) ? 1 : 0);
            uint uerr = (uint)(!!(Convert.ToBoolean(err)) ? 1 : 0);
            return id | ueff << 31 | urtr << 30 | uerr << 29;            
        }

        public bool IsEFF(uint id)//1:extend frame 0:standard frame
        {
            return !!Convert.ToBoolean((id & CAN_EFF_FLAG));
        }

        public bool IsRTR(uint id)//1:remote frame 0:data frame
        {
            return !!Convert.ToBoolean((id & CAN_RTR_FLAG));
        }

        public bool IsERR(uint id)//1:error frame 0:normal frame
        {
            return !!Convert.ToBoolean((id & CAN_ERR_FLAG));
        }

        public uint GetId(uint id)
        {
            return id & CAN_ID_FLAG;
        }

        private void CANForm_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (m_bOpen)
            {
                Method.ZCAN_CloseDevice(device_handle_);
            }            
        }

        private void button_open_Click(object sender, EventArgs e)
        {
            uint device_type_index_ = (uint)comboBox_device.SelectedIndex;
            uint device_index_;
         
            device_index_ = (uint)comboBox_index.SelectedIndex;
            
            device_handle_ = Method.ZCAN_OpenDevice(kDeviceType[device_type_index_].device_type, device_index_, 0);
            if (NULL == (int)device_handle_)
            {
                MessageBox.Show("Failed to open device. Please check if the device type and device index are correct", "Note",
                        MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }
            m_bOpen = true;
            EnableCtrl(true);
            button_open.Enabled = false;
            button_init.Enabled = true;
            button_start.Enabled = true;
            button_reset.Enabled = true;
            button_close.Enabled = true;
        }

        private void button_init_Click(object sender, EventArgs e)
        {
            if (!m_bOpen)
            {
                MessageBox.Show("Device Not Opened", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }

            uint type = kDeviceType[comboBox_device.SelectedIndex].device_type;
            
            bool usbCanfd = type == Define.ZCAN_USBCANFD_100U ||
                type == Define.ZCAN_USBCANFD_200U ||
                type == Define.ZCAN_USBCANFD_MINI;
            bool canfdDevice = usbCanfd;
            if (!m_bCloud)
            {
                IntPtr ptr = Method.GetIProperty(device_handle_);
                if (NULL == (int)ptr)
                {
                    MessageBox.Show("Failed to get the specified path property", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                    return;
                }

                property_ = (IProperty)Marshal.PtrToStructure((IntPtr)((UInt32)ptr), typeof(IProperty));

                {
                    if (usbCanfd)
                    {
                        if (!setCANFDStandard(comboBox_standard.SelectedIndex)) //设置CANFD标准
                        {
                            MessageBox.Show("Set CANFD standard failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                            return;
                        }
                    }
                    /*
                    if (checkBox_ABIT.Checked)//设置波特率
                    {
                        if (!setCustomBaudrate())
                        {
                            MessageBox.Show("设置自定义波特率失败", "提示", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                            return;
                        }
                    }        
                    */
                    else
                    {
                        // set rates fd
                        if (!setBaudrateFD('A', kAbitTiming[comboBox_ABIT.SelectedIndex]))
                        {
                            MessageBox.Show("Failed to set arbitration baud rate", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                            return;
                        }
                        if (!setBaudrateFD('D', kDbitTiming[comboBox_ABIT2.SelectedIndex]))
                        {
                            MessageBox.Show("Failed to set data baud rate", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                            return;
                        }
                    }
                }
            }

            ZCAN_CHANNEL_INIT_CONFIG config_ = new ZCAN_CHANNEL_INIT_CONFIG();
            
            {
                config_.canfd.mode = (byte)comboBox_mode.SelectedIndex;
                if (canfdDevice)
                {
                    config_.can_type = Define.TYPE_CANFD;                  
                }
                else
                {
                    config_.can_type = Define.TYPE_CAN;
                }
            }
            IntPtr pConfig = Marshal.AllocHGlobal(Marshal.SizeOf(config_));
            Marshal.StructureToPtr(config_, pConfig, true);

            channel_handle_ = Method.ZCAN_InitCAN(device_handle_, (uint)0, pConfig);
            //CAN2
            channel_handle2_ = Method.ZCAN_InitCAN(device_handle_, (uint)1, pConfig);
            Marshal.FreeHGlobal(pConfig);
                        

            if (NULL == (int)channel_handle_)
            {
                MessageBox.Show("Init CAN1 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }
            if (NULL == (int)channel_handle2_)
            {
                MessageBox.Show("Init CAN2 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }

            
            {
                /*
                if (usbCanfd && checkBox_resistance.Checked)
                {
                    if (!setResistanceEnable())
                    {
                        MessageBox.Show("使能终端电阻失败", "提示", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                        return;
                    }
                }                
                */
                
                if (canfdDevice && !setFilter())
                {
                    MessageBox.Show("Set Filter Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                    return;
                }                
            }

            button_init.Enabled = false;
        }

        private void button_start_Click(object sender, EventArgs e)
        {
            if (Method.ZCAN_StartCAN(channel_handle_) != Define.STATUS_OK)
            {
                MessageBox.Show("Start CAN1 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }
            //CAN2
            if (Method.ZCAN_StartCAN(channel_handle2_) != Define.STATUS_OK)
            {
                MessageBox.Show("Start CAN2 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }

            button_start.Enabled = false;
            button_reset.Enabled = true;
            m_bStart = true;
            if (null == recv_data_thread_)
            {
                recv_data_thread_ = new recvdatathread();
                recv_data_thread_.setChannelHandle(channel_handle_, channel_handle2_);
                recv_data_thread_.setStart(m_bStart);
                recv_data_thread_.RecvCANData += this.AddData;
                recv_data_thread_.RecvFDData += this.AddData;
            }
            else
            {
                recv_data_thread_.setChannelHandle(channel_handle_, channel_handle2_);
            }
        }

        private void button_reset_Click(object sender, EventArgs e)
        {
            if (Method.ZCAN_ResetCAN(channel_handle_) != Define.STATUS_OK)
            {
                MessageBox.Show("Reset CAN1 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }
            //CAN2
            if (Method.ZCAN_ResetCAN(channel_handle2_) != Define.STATUS_OK)
            {
                MessageBox.Show("Reset CAN2 Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }

            button_init.Enabled = true;
            button_start.Enabled = true;
            button_reset.Enabled = false;
            //m_bOpen = false;
        }

        private void button_close_Click(object sender, EventArgs e)
        {
            Method.ZCAN_CloseDevice(device_handle_);
            m_bOpen = false;
            m_bCloud = false;
            EnableCtrl(false);
            button_open.Enabled = true;
            button_start.Enabled = true;
            button_init.Enabled = true;
            button_reset.Enabled = true;
            m_bOpen = false;
        }

        private void button_send_Click(object sender, EventArgs e)
        {
            if (textBox_senddata.Text.Length == 0)
            {
                MessageBox.Show("Data NULL", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                return;
            }

            uint id = (uint)System.Convert.ToInt32(textBox_ID.Text, 16);
            string data = textBox_senddata.Text;
            int frame_type_index = comboBox_frametype.SelectedIndex;
            int protocol_index = comboBox_protocol.SelectedIndex;
            int send_type_index = comboBox_sendtype.SelectedIndex;
            int canfd_exp_index = comboBox_canfd_exp.SelectedIndex;
            int chan = comboBox_channel.SelectedIndex;

            uint result; //发送的帧数

            if (0 == protocol_index) //can
            {
                ZCAN_Transmit_Data can_data = new ZCAN_Transmit_Data();
                can_data.frame.can_id = MakeCanId(id, frame_type_index, 0, 0);
                //can_data.frame.data = new byte[8];
                can_data.frame.data = new byte[64]; // 用最大的数组，避免输入错误导致出错。
                can_data.frame.can_dlc = (byte)SplitData(data, ref can_data.frame.data, CAN_MAX_DLEN);
                //can_data.frame.can_dlc = (can_data.frame.can_dlc > 8) ? 8 : can_data.frame.can_dlc;
                can_data.transmit_type = (uint)send_type_index;
                IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(can_data));
                Marshal.StructureToPtr(can_data, ptr, true);
                result = Method.ZCAN_Transmit((chan == 0) ? channel_handle_ : channel_handle2_, ptr, 1);
                Marshal.FreeHGlobal(ptr);
            }
            else //canfd
            {
                ZCAN_TransmitFD_Data canfd_data = new ZCAN_TransmitFD_Data();
                canfd_data.frame.can_id = MakeCanId(id, frame_type_index, 0, 0);
                canfd_data.frame.data = new byte[64];
                canfd_data.frame.len = (byte)SplitData(data, ref canfd_data.frame.data, CANFD_MAX_DLEN);
                canfd_data.transmit_type = (uint)send_type_index;
                canfd_data.frame.flags = (byte)((canfd_exp_index != 0) ? CANFD_BRS : 0);
                IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(canfd_data));
                Marshal.StructureToPtr(canfd_data, ptr, true);
                result = Method.ZCAN_TransmitFD((chan == 0) ? channel_handle_ : channel_handle2_, ptr, 1);
                Marshal.FreeHGlobal(ptr);
            }

            if (result != 1)
            {
                MessageBox.Show("Transmit Failed", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
                AddErr();
            }
        }

        private void button_clear_Click(object sender, EventArgs e)
        {
            listBox.Items.Clear();
            /*
            // test if dev is online -- ok
            if (Method.ZCAN_IsDeviceOnLine(device_handle_) == Define.STATUS_OFFLINE)
            {
                MessageBox.Show("掉线检测测试：\r\n掉线", "提示", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }
            else
            {
                MessageBox.Show("掉线检测测试：\r\n连接", "提示", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }                
            */
        }

        private void comboBox_device_SelectedIndexChanged(object sender, EventArgs e)
        {
            comboBox_channel.Items.Clear();
            setChannelCombobox(0, (int)kDeviceType[comboBox_device.SelectedIndex].channel_count, 0);
            EnableSet();
        }

        private void comboBox_channel_SelectedIndexChanged(object sender, EventArgs e)
        {
            channel_index_ = comboBox_channel.SelectedIndex;
        }

        private void setComboboxIndex(int start, int end, int current)
        {
            for (int i = start; i < end; ++i)
            {
                comboBox_index.Items.Add(i);
            }

            comboBox_index.SelectedIndex = current;
        }

        private void setChannelCombobox(int start, int end, int current)
        {
            for (int i = start; i < end; ++i)
            {
                comboBox_channel.Items.Add(i);
            }

            comboBox_channel.SelectedIndex = current;
            channel_index_ = comboBox_channel.SelectedIndex;
        }
                
        //设置波特率
        private bool setBaudrateFD(char ad, UInt32 baud)
        {
            /*
                    string path="";
                    if(ad=='A')
                    {
                        path = channel_index_ + "/canfd_abit_baud_rate";
                    }
                    else if (ad == 'D')
                    {
                        path = channel_index_ + "/canfd_dbit_baud_rate";
                    }

                    string value = baud.ToString();

                    return 1 == property_.SetValue(path, value);
            */
            //修改为2个通道同时设置相同参数
            string value = baud.ToString();

            if (ad == 'A')
            {
                if (1 != property_.SetValue("0/canfd_abit_baud_rate", value))
                    return false;
                return 1 == property_.SetValue("1/canfd_abit_baud_rate", value);
            }
            else if (ad == 'D')
            {
                if (1 != property_.SetValue("0/canfd_dbit_baud_rate", value))
                    return false;
                return 1 == property_.SetValue("1/canfd_dbit_baud_rate", value);
            }
            else
                return false;
        }

        //设置CANFD标准
        private bool setCANFDStandard(int canfd_standard)
        {
            string path = channel_index_ + "/canfd_standard";
            string value = canfd_standard.ToString();
            return 1 == property_.SetValue(path, value);
        }

        //设置自定义波特率, 需要从CANMaster目录下的baudcal生成字符串
        //private bool setCustomBaudrate()
        //{
            /*
            string path = channel_index_ + "/baud_rate_custom";
            string baudrate = textBox_ABIT.Text;            
            return 1 == property_.SetValue(path, baudrate);
            */

            /*
            //修改为2个通道同时设置相同参数
            string baudrate = textBox_ABIT.Text;

            if (1 != property_.SetValue("0/baud_rate_custom", baudrate))
                return false;

            return 1 == property_.SetValue("1/baud_rate_custom", baudrate);
            */

       // }
        /*
        //设置终端电阻使能
        private bool setResistanceEnable()
        {
            string path = channel_index_ + "/initenal_resistance";
            string value = (checkBox_resistance.Checked ? "1" : "0");            
            return 1 == property_.SetValue(path, value);
        }
        */

        //设置滤波
        private bool setFilter()
        {
            if (comboBox_standard2.SelectedIndex == 2)
                return true;

            //2通道同时设置相同参数
            string path = channel_index_ + "/filter_clear";//清除滤波
            string value = "0";
            if (0 == property_.SetValue("0/filter_clear", value))
            {
                return false;
            }
            if (0 == property_.SetValue("1/filter_clear", value))
            {
                return false;
            }

            path = channel_index_ + "/filter_mode";
            value = comboBox_standard2.SelectedIndex.ToString();
            if (0 == property_.SetValue("0/filter_mode", value))
            {
                return false;
            }
            if (0 == property_.SetValue("1/filter_mode", value))
            {
                return false;
            }

            path = channel_index_ + "/filter_start";
            value = "0x"+textBox_startid.Text;
            if (0 == property_.SetValue("0/filter_start", value))
            {
                return false;
            }
            if (0 == property_.SetValue("1/filter_start", value))
            {
                return false;
            }

            path = channel_index_ + "/filter_end";
            value = "0x"+textBox_endid.Text;
            if (0 == property_.SetValue("0/filter_end", value))
            {
                return false;
            }
            if (0 == property_.SetValue("1/filter_end", value))
            {
                return false;
            }
            /*
            //add more  1
            path = channel_index_ + "/filter_start";
            value = "0x11";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            path = channel_index_ + "/filter_end";
            value = "0x20";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            //add more  2
            path = channel_index_ + "/filter_start";
            value = "0x101";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            path = channel_index_ + "/filter_end";
            value = "0x200";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            //add more  3
            path = channel_index_ + "/filter_start";
            value = "0x150";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            path = channel_index_ + "/filter_end";
            value = "0x300";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            //add more  4
            path = channel_index_ + "/filter_start";
            value = "0x401";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            path = channel_index_ + "/filter_end";
            value = "0x500";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            //add more  5
            path = channel_index_ + "/filter_start";
            value = "0x601";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            path = channel_index_ + "/filter_end";
            value = "0x602";
            if (0 == property_.SetValue(path, value))
            {
                return false;
            }
            */

            //path = channel_index_ + "/filter_ack";//滤波生效
            value = "0";

            //2个通道 同时生效
            if (0 == property_.SetValue("0/filter_ack", value))
            {
                return false;
            }
            if (0 == property_.SetValue("1/filter_ack", value))
            {
                return false;
            }

            //如果要设置多条滤波，在清除滤波和滤波生效之间设置多条滤波即可
            return true;
        }

        //private bool setFilterCode()
        //{
        //    string path = channel_index_ + "/filter";
        //    string value = "0";
        //    if (0 == property_.SetValue(path, value))
        //    {
        //        return false;
        //    }
        //    path = channel_index_ + "/acc_code";
        //    value = "0x0000";
        //    if (0 == property_.SetValue(path, value))
        //    {
        //        return false;
        //    }
        //    path = channel_index_ + "/acc_mask";
        //    value = "0xFFFFFFFF";
        //    if (0 == property_.SetValue(path, value))
        //    {
        //        return false;
        //    }
        //    return true;
        //}
        
        private void AddData(ZCAN_Receive_Data[] data, uint len, uint ch)
        {
            list_box_data_ = "";
            list_box_data_ = "";
            for (uint i = 0; i < len; ++i)
            {
                ZCAN_Receive_Data can = data[i];
                uint id = data[i].frame.can_id;
                string eff = IsEFF(id) ? "Extended Frame" : "Standard Frame";
                string rtr = IsRTR(id) ? "Remote Frame" : "Data Frame";
                if (ch == 0)
                {
                    list_box_data_ = String.Format("[Chan0] Received CAN ID:0x{0:X8} {1:G} {2:G} LEN:{3:D1} Data:", GetId(id), eff, rtr, can.frame.can_dlc);
                }
                else
                {
                    list_box_data_ = String.Format("[Chan1] Received CAN ID:0x{0:X8} {1:G} {2:G} LEN:{3:D1} Data:", GetId(id), eff, rtr, can.frame.can_dlc);
                }

                for (uint j = 0; j < can.frame.can_dlc; ++j)
                {
                    list_box_data_ = String.Format("{0:G}{1:X2} ", list_box_data_, can.frame.data[j]);
                }
            }

            Object[] list = { this, System.EventArgs.Empty };
            this.listBox.BeginInvoke(new EventHandler(SetListBox), list);
        }

        private void AddData(ZCAN_ReceiveFD_Data[] data, uint len, uint ch)
        {
            list_box_data_ = "";
            for (uint i = 0; i < len; ++i)
            {
                ZCAN_ReceiveFD_Data canfd = data[i];
                uint id = data[i].frame.can_id;
                string eff = IsEFF(id) ? "Extended Frame" : "Standard Frame";
                string rtr = IsRTR(id) ? "Remote Frame" : "Data Frame";

                if (ch == 0)
                {
                    list_box_data_ = String.Format("[Chan0] Received CANFD ID:0x{0:X8} {1:G} {2:G} LEN:{3:D1} Data:", GetId(id), eff, rtr, canfd.frame.len);
                }
                else
                {
                    list_box_data_ = String.Format("[Chan1] Received CANFD ID:0x{0:X8} {1:G} {2:G} LEN:{3:D1} Data:", GetId(id), eff, rtr, canfd.frame.len);
                }

                for (uint j = 0; j < canfd.frame.len; ++j)
                {
                    list_box_data_ = String.Format("{0:G}{1:X2} ", list_box_data_, canfd.frame.data[j]);
                }
            }

            Object[] list = { this, System.EventArgs.Empty };
            this.listBox.BeginInvoke(new EventHandler(SetListBox), list);
        }

        private void AddErr()
        {
            ZCAN_CHANNEL_ERROR_INFO pErrInfo = new ZCAN_CHANNEL_ERROR_INFO();
            IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(pErrInfo));
            Marshal.StructureToPtr(pErrInfo, ptr, true);
            if (Method.ZCAN_ReadChannelErrInfo(channel_handle_, ptr) != Define.STATUS_OK)
            {
                MessageBox.Show("Failed to get err info", "Note", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
            }
            Marshal.FreeHGlobal(ptr);

            string errorInfo = String.Format("ERRCode：{0:D1}", pErrInfo.error_code);
            int index = listBox.Items.Add(errorInfo);
            listBox.SelectedIndex = index;
        }

        private void SetListBox(object sender, EventArgs e)
        {
            int index = listBox.Items.Add(list_box_data_);
            listBox.SelectedIndex = index;
        }

        private void EnableCtrl(bool opened)
        {
            comboBox_device.Enabled = !opened;
            comboBox_index.Enabled = !opened;
            //comboBox_channel.Enabled = !opened;
            button_open.Enabled = !opened;
        }

        private void EnableSet()
        {
            uint type = kDeviceType[comboBox_device.SelectedIndex].device_type;            
            bool usbCanfd = type == Define.ZCAN_USBCANFD_100U ||
                type == Define.ZCAN_USBCANFD_200U ||
                type == Define.ZCAN_USBCANFD_MINI;
            bool canfdDevice = usbCanfd;

            comboBox_mode.Enabled = true;
            comboBox_standard.Enabled = canfdDevice;
            comboBox_ABIT.Enabled = canfdDevice;
            comboBox_ABIT2.Enabled = canfdDevice;
           // checkBox_ABIT.Enabled = true;
           // textBox_ABIT.Enabled = true;
           // checkBox_resistance.Enabled = usbCanfd;
            comboBox_standard2.Enabled = true;
            textBox_startid.Enabled = true;
            textBox_endid.Enabled = true;
            comboBox_index.Enabled = true;
           // label_baudprompt.Enabled = true;
            label_standard.Enabled = canfdDevice;
            label_mode.Enabled = canfdDevice;
            label_ABIT.Enabled = canfdDevice;
            label_ABIT2.Enabled = canfdDevice;
            label_ABIT.Enabled = canfdDevice;
            label_ABIT.Enabled = canfdDevice;
            label_standard2.Enabled = true;
            label_startid.Enabled = true;
            label_endid.Enabled = true;
            label_index.Enabled = true;
        }

        //拆分text到发送data数组
        private int SplitData(string data, ref byte[] transData, int maxLen)
        {
            int retLen = 0;
            string[] dataArray = data.Split(' ');
            for (int i = 0; (i < maxLen) && (i < dataArray.Length); i++)
            {
                transData[i] = Convert.ToByte(dataArray[i].Substring(0, 2), 16);
                retLen++;
            }

            //return dataArray.Length;
            return retLen;
        }

        private void comboBox_netmode_SelectedIndexChanged(object sender, EventArgs e)
        {
            EnableSet();
        }

        private void comboBox_ABIT_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

    }
}
