object Form2: TForm2
  Left = 0
  Top = 0
  Caption = 'CANFD'
  ClientHeight = 638
  ClientWidth = 962
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -14
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  OnClose = FormClose
  OnCreate = FormCreate
  PixelsPerInch = 120
  TextHeight = 17
  object device_setting: TGroupBox
    Left = 10
    Top = 10
    Width = 577
    Height = 167
    Caption = 'Device Parameters'
    TabOrder = 2
    object Label1: TLabel
      Left = 18
      Top = 35
      Width = 36
      Height = 17
      Caption = 'Type:'
    end
    object Label2: TLabel
      Left = 243
      Top = 35
      Width = 40
      Height = 17
      Caption = 'Index:'
    end
    object Label3: TLabel
      Left = 388
      Top = 35
      Width = 84
      Height = 17
      Caption = 'CAN Channel:'
    end
    object Label5: TLabel
      Left = 18
      Top = 82
      Width = 72
      Height = 17
      Caption = 'WorkMode:'
    end
    object Label6: TLabel
      Left = 388
      Top = 82
      Width = 108
      Height = 17
      Caption = 'CANFD Standard:'
    end
    object Label7: TLabel
      Left = 4
      Top = 131
      Width = 116
      Height = 17
      Caption = 'Arbitrate Baudrate:'
    end
    object Label8: TLabel
      Left = 243
      Top = 132
      Width = 93
      Height = 17
      Caption = 'Data Baudrate:'
    end
    object device_index_box: TComboBox
      Left = 288
      Top = 31
      Width = 76
      Height = 25
      ItemHeight = 17
      TabOrder = 0
      Items.Strings = (
        '0'
        '1'
        '2'
        '3')
    end
    object can_index_box: TComboBox
      Left = 492
      Top = 31
      Width = 76
      Height = 25
      ItemHeight = 17
      TabOrder = 1
      Items.Strings = (
        '0'
        '1'
        '2'
        '3')
    end
    object mode_box: TComboBox
      Left = 94
      Top = 78
      Width = 114
      Height = 25
      ItemHeight = 17
      TabOrder = 2
      OnChange = mode_boxChange
      Items.Strings = (
        #27491#24120#27169#24335
        #21482#21548#27169#24335)
    end
    object canfd_standard_box: TComboBox
      Left = 492
      Top = 78
      Width = 76
      Height = 25
      ItemHeight = 17
      TabOrder = 3
      Items.Strings = (
        'ISO'
        'BOSCH')
    end
    object device_type_box: TComboBox
      Left = 63
      Top = 31
      Width = 145
      Height = 25
      ItemHeight = 17
      TabOrder = 4
      Items.Strings = (
        'ZCAN_USBCANFD_200U'
        'ZCAN_USBCANFD_100U'
        'ZCAN_USBCANFD_MINI')
    end
    object abit_baud_box: TEdit
      Left = 126
      Top = 128
      Width = 82
      Height = 25
      TabOrder = 5
      Text = '1000'
    end
    object dbit_baud_box: TEdit
      Left = 350
      Top = 128
      Width = 83
      Height = 25
      TabOrder = 6
      Text = '5000'
    end
  end
  object start_Button: TButton
    Left = 855
    Top = 56
    Width = 98
    Height = 33
    Caption = 'Start CAN'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -15
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 0
    OnClick = start_ButtonClick
  end
  object reset_Button: TButton
    Left = 854
    Top = 97
    Width = 98
    Height = 32
    Caption = 'Reset CAN'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -15
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 1
    OnClick = reset_ButtonClick
  end
  object data_display: TGroupBox
    Left = 8
    Top = 329
    Width = 837
    Height = 305
    Caption = 'Display'
    TabOrder = 3
    object ListBox1: TListBox
      Left = 4
      Top = 31
      Width = 821
      Height = 263
      ItemHeight = 17
      TabOrder = 0
    end
  end
  object data_sent_setting: TGroupBox
    Left = 8
    Top = 206
    Width = 946
    Height = 117
    Caption = 'Transmit'
    TabOrder = 4
    object Label10: TLabel
      Left = 21
      Top = 77
      Width = 45
      Height = 17
      Caption = 'ID(0x):'
    end
    object Label11: TLabel
      Left = 12
      Top = 37
      Width = 78
      Height = 17
      Caption = 'Frame Type:'
    end
    object Label12: TLabel
      Left = 243
      Top = 78
      Width = 60
      Height = 17
      Caption = 'Data(0x):'
    end
    object Label13: TLabel
      Left = 372
      Top = 37
      Width = 86
      Height = 17
      Caption = 'Send Method:'
    end
    object Label14: TLabel
      Left = 191
      Top = 37
      Width = 91
      Height = 17
      Caption = 'Frame Format:'
    end
    object Label4: TLabel
      Left = 695
      Top = 37
      Width = 56
      Height = 17
      Caption = 'Protocol:'
    end
    object id_edit: TEdit
      Left = 72
      Top = 73
      Width = 136
      Height = 25
      TabOrder = 0
      Text = '100'
    end
    object data_edit: TEdit
      Left = 314
      Top = 73
      Width = 490
      Height = 25
      TabOrder = 1
      Text = '11 22 33 44 55 66 77 88 '
    end
    object eff_box: TComboBox
      Left = 96
      Top = 31
      Width = 89
      Height = 25
      ItemHeight = 17
      TabOrder = 2
      Items.Strings = (
        #26631#20934#24103
        #25193#23637#24103)
    end
    object send_type_box: TComboBox
      Left = 464
      Top = 31
      Width = 216
      Height = 25
      ItemHeight = 17
      TabOrder = 3
      Items.Strings = (
        #27491#24120#21457#36865
        #21333#27425#21457#36865
        #33258#21457#33258#25910
        #21333#27425#33258#21457#33258#25910)
    end
    object rtr_box: TComboBox
      Left = 288
      Top = 31
      Width = 78
      Height = 25
      ItemHeight = 17
      TabOrder = 4
      Items.Strings = (
        #25968#25454#24103
        #36828#31243#24103)
    end
    object can_type_box: TComboBox
      Left = 757
      Top = 31
      Width = 76
      Height = 25
      ItemHeight = 17
      TabOrder = 5
      Items.Strings = (
        'CAN'
        'CANFD')
    end
    object canfd_brs: TCheckBox
      Left = 839
      Top = 34
      Width = 104
      Height = 22
      Caption = 'CANFD BRS'
      TabOrder = 6
    end
  end
  object send_Button: TButton
    Left = 855
    Top = 326
    Width = 98
    Height = 32
    Caption = 'Send'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -15
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 5
    OnClick = send_ButtonClick
  end
  object filer_setting: TGroupBox
    Left = 595
    Top = 22
    Width = 252
    Height = 155
    Caption = 'Filter Configuration'
    TabOrder = 6
    object Label15: TLabel
      Left = 17
      Top = 49
      Width = 70
      Height = 17
      Caption = 'Filter Mode:'
    end
    object Label16: TLabel
      Left = 14
      Top = 90
      Width = 79
      Height = 17
      Caption = 'Start ID(0x):'
    end
    object Label17: TLabel
      Left = 17
      Top = 120
      Width = 73
      Height = 17
      Caption = 'End ID(0x):'
    end
    object filter_check: TCheckBox
      Left = -33
      Top = 21
      Width = 108
      Height = 22
      BiDiMode = bdRightToLeft
      Caption = 'Enable'
      Color = clBtnFace
      ParentBiDiMode = False
      ParentColor = False
      TabOrder = 0
    end
    object filtermode_box: TComboBox
      Left = 93
      Top = 46
      Width = 95
      Height = 25
      ItemHeight = 17
      TabOrder = 1
      Items.Strings = (
        #26631#20934#24103
        #25193#23637#24103)
    end
    object filter_end_edit: TEdit
      Left = 111
      Top = 118
      Width = 95
      Height = 25
      TabOrder = 2
      Text = '200'
    end
    object filter_start_edit: TEdit
      Left = 108
      Top = 87
      Width = 96
      Height = 25
      TabOrder = 3
      Text = '100'
    end
  end
  object getdevinf_Button: TButton
    Left = 854
    Top = 137
    Width = 98
    Height = 33
    Caption = 'Device Info'
    TabOrder = 7
    OnClick = getdevinf_ButtonClick
  end
  object data_clear_Button: TButton
    Left = 854
    Top = 418
    Width = 98
    Height = 33
    Caption = 'Clear'
    TabOrder = 8
    OnClick = data_clear_ButtonClick
  end
end
