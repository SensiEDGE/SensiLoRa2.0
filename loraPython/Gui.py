# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import wx
import serial
import serial.tools.list_ports
import time, threading
import wx

EVT_RESULT_ID = wx.Window.NewControlId()


def EVT_RESULT(win, func):
    """Определяем событие завершения."""
    win.Connect(-1, -1, EVT_RESULT_ID, func)

class ResultEvent(wx.PyEvent):
    """Простое события для разных данных."""
    def __init__(self, data):
        """Инициируем событие завершения."""
        wx.PyEvent.__init__(self)
        self.SetEventType(EVT_RESULT_ID)
        self.data = data


class MyPanel(wx.Panel):

    def __init__(self, parent):
        """Constructor"""
        wx.Panel.__init__(self, parent)

        bSizer4 = wx.BoxSizer(wx.VERTICAL)

        bSizer4.SetMinSize(wx.Size(50, 50))
        bSizer5 = wx.BoxSizer(wx.VERTICAL)

        bSizer5.SetMinSize(wx.Size(50, 50))
        self.m_staticText24 = wx.StaticText(self, wx.ID_ANY, u"Connect to PC SensiGFOX | LoRa 2.0", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText24.Wrap(-1)

        self.m_staticText24.SetFont(
            wx.Font(24, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, wx.EmptyString))

        bSizer5.Add(self.m_staticText24, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALL, 5)

        self.m_staticText25 = wx.StaticText(self, wx.ID_ANY, u"Use Type-C connector", wx.DefaultPosition,
                                            wx.DefaultSize, 0)
        self.m_staticText25.Wrap(-1)

        bSizer5.Add(self.m_staticText25, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALL, 5)

        self.m_bitmap5 = wx.StaticBitmap(self, wx.ID_ANY,
                                         wx.Bitmap(u"Безымянный.png", wx.BITMAP_TYPE_ANY),
                                         wx.DefaultPosition, wx.DefaultSize, 0)
        bSizer5.Add(self.m_bitmap5, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALL, 5)

        self.m_bitmap4 = wx.StaticBitmap(self, wx.ID_ANY,
                                         wx.Bitmap(u"SensiEdge.bmp", wx.BITMAP_TYPE_ANY),
                                         wx.DefaultPosition, wx.DefaultSize, 0)
        bSizer5.Add(self.m_bitmap4, 0, wx.ALIGN_RIGHT | wx.ALL, 5)

        bSizer4.Add(bSizer5, 1, wx.EXPAND, 5)

        self.SetSizer(bSizer4)
        self.Centre(wx.BOTH)
        # self.Centre(wx.BOTH)

    def __del__(self):
        pass

class Window(wx.Frame):

    def __init__(self, parent, title, usbManager, memoryManager):
        self.m_parent = parent
        self.m_title = title
        self.m_usb = usbManager
        self.m_mm = memoryManager

        wx.Frame.__init__(self, self.m_parent, id=wx.ID_ANY, title=self.m_title, pos=wx.DefaultPosition,
                          size=wx.Size(700, 400), style=wx.DEFAULT_FRAME_STYLE | wx.TAB_TRAVERSAL)

        self.SetSizeHints(wx.DefaultSize, wx.DefaultSize)
        self.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))

        self.panelConnect = MyPanel(self)
        self.panelChanges = MyPanel1(self)
        self.panelSettings = MyPanel2(self)
        self.panelConnect.Show()
        self.panelChanges.Hide()
        self.panelSettings.Hide()

        self.Bind(wx.EVT_BUTTON, self.click)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.panelConnect, 1, wx.EXPAND)
        self.sizer.Add(self.panelChanges, 1, wx.EXPAND)
        self.sizer.Add(self.panelSettings, 1, wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Layout()
        self.showStatusBar()
        self.Show(True)

    def showConfigWindow(self):
        self.panelConnect.Hide()
        self.panelChanges.Show()
        threading.Timer(10, self.showStatusBar).start()
        # self.showStatusBar()
        self.Layout()

    def showStatusBar(self):
        self.m_statusBar2 = self.CreateStatusBar(4, wx.STB_SIZEGRIP, wx.ID_ANY)
        self.m_statusBar2.SetStatusText('Status: Search device')
        self.m_statusBar2.SetStatusText('Device:None', 1)
        self.m_statusBar2.SetStatusText('Firmware:None', 2)
        self.m_statusBar2.SetStatusText('Battery:None', 3)

    def showOk(self):
        l_name = self.m_mm.getNameDevice()
        l_ver = self.m_mm.getVersionDevice()
        l_batt = self.m_mm.getBatteryPercent()
        l_appKey = self.m_mm.getAppKey()
        l_devEui = self.m_mm.getDevEui()
        l_joinEui = self.m_mm.getJoinEui()
        l_text = "Found device: " + l_name + "\n" + "Firmware version: " + l_ver + "Battery: " + str(l_batt) + '%'+ "\n" + "AppKey: " +str(l_appKey) + "\n" + "DevEui: " + str(l_devEui) + "\n" + "JoinEui: " + str(l_joinEui)
        dlg = wx.MessageDialog(self, l_text,"Found Device", wx.OK)
        dlg.ShowModal()

    def click(self, event):
        self.m_menubar1 = wx.MenuBar( 0 )
        self.Menu = wx.Menu()
        self.m_menubar1.Append( self.Menu, u"Menu" )

        self.m_menu2 = wx.Menu()
        self.m_menubar1.Append( self.m_menu2, u"Info" )

        self.m_menu3 = wx.Menu()
        self.m_menubar1.Append( self.m_menu3, u"Exit" )

        self.SetMenuBar( self.m_menubar1 )
        self.SetSize(1100,500)
        self.panelConnect.Hide()
        self.panelChanges.Hide()
        self.panelSettings.Show()
        self.SetSize(1100, 630)
        self.Layout()





class MyPanel1(wx.Panel):

    def __init__(self, parent):
        """Constructor"""
        wx.Panel.__init__(self, parent)

        bSizer4 = wx.BoxSizer(wx.VERTICAL)

        bSizer4.SetMinSize(wx.Size(50, 50))
        bSizer6 = wx.BoxSizer(wx.HORIZONTAL)

        bSizer16 = wx.BoxSizer(wx.VERTICAL)

        self.m_bitmap9 = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap(u"Test.png", wx.BITMAP_TYPE_ANY), wx.Point(40, 40),
                                         wx.DefaultSize, 0)
        bSizer16.Add(self.m_bitmap9, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        self.m_button4 = wx.Button(self, wx.ID_ANY, u"Test Sensors", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_button4.SetFont(
            wx.Font(15, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, wx.EmptyString))

        bSizer16.Add(self.m_button4, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        bSizer6.Add(bSizer16, 1, wx.EXPAND, 5)

        bSizer17 = wx.BoxSizer(wx.VERTICAL)

        self.m_bitmap10 = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap(u"Settings.png", wx.BITMAP_TYPE_ANY),
                                          wx.Point(100, 10), wx.DefaultSize, 0)
        self.m_bitmap10.SetFont(
            wx.Font(15, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, wx.EmptyString))

        bSizer17.Add(self.m_bitmap10, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        self.m_button5 = wx.Button(self, wx.ID_ANY, u"Config Sensors", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_button5.SetFont(
            wx.Font(15, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, wx.EmptyString))

        bSizer17.Add(self.m_button5, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        bSizer6.Add(bSizer17, 1, wx.EXPAND, 5)

        bSizer18 = wx.BoxSizer(wx.VERTICAL)

        self.m_bitmap11 = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap(u"Info.png", wx.BITMAP_TYPE_ANY),
                                          wx.DefaultPosition, wx.DefaultSize, 0)
        bSizer18.Add(self.m_bitmap11, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        self.m_button6 = wx.Button(self, wx.ID_ANY, u"Information", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_button6.SetFont(
            wx.Font(15, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, wx.EmptyString))

        bSizer18.Add(self.m_button6, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        bSizer6.Add(bSizer18, 1, wx.EXPAND, 5)

        bSizer4.Add(bSizer6, 1, wx.EXPAND, 5)

        self.SetSizer(bSizer4)
        self.Centre(wx.BOTH)

        # Connect Events
        self.m_button5.Bind(wx.EVT_BUTTON, self.click)
    def click(self, event):
        event.Skip()


class MyPanel2(wx.Panel):

    def __init__(self, parent):
        """Constructor"""
        wx.Panel.__init__(self, parent)

        m_accG = ['2g','4g','8g','16g']
        m_accOdr = ['10Hz','50Hz','119Hz','238Hz','476Hz','952Hz']
        m_gyDps = ['245dps', '500dps', '1000dps', '2000dps']
        m_gyOdr = ['15Hz', '60Hz', '119Hz', '238Hz', '476Hz', '952Hz']
        m_magOdr = ['10Hz','20Hz','50Hz','100Hz']
        m_magPm = ['High Resolution','Low Power']
        m_pressOdr = ['1Hz','10Hz', '25Hz', '50Hz', '75Hz','100','200']
        m_humOdr = ['1Hz','7Hz','12.5Hz']
        m_humAvg = ['4','8','16','32','64','128','256','512']
        m_tempAvg = ['2','4', '8', '16', '32', '64', '128', '256']
        m_lightOdr = ['10Hz','20Hz','50Hz','100Hz']
        m_lightPm = ['High Resolution','Low Power']



        bSizer4 = wx.BoxSizer( wx.VERTICAL )

        bSizer4.SetMinSize( wx.Size( 50,50 ) )
        self.m_staticline82 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL, u"j" )
        bSizer4.Add( self.m_staticline82, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer10 = wx.BoxSizer( wx.HORIZONTAL )

        bSizer41 = wx.BoxSizer( wx.VERTICAL )

        sbSizer1 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Accelerometer" ), wx.VERTICAL )

        self.m_enableAcc = wx.CheckBox( sbSizer1.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer1.Add( self.m_enableAcc, 0, wx.ALL, 5 )

        bSizer20 = wx.BoxSizer( wx.HORIZONTAL )

        m_accFullScaleChoices = [ u"Radio Button" ]
        self.m_accFullScale = wx.RadioBox( sbSizer1.GetStaticBox(), wx.ID_ANY, u"Full Scale (g)", wx.DefaultPosition, wx.DefaultSize, m_accG, 1, wx.RA_SPECIFY_COLS )
        self.m_accFullScale.SetSelection( 0 )
        bSizer20.Add( self.m_accFullScale, 0, wx.ALL, 5 )

        m_AccODRChoices = [ u"Radio Button" ]
        self.m_AccODR = wx.RadioBox( sbSizer1.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_accOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_AccODR.SetSelection( 0 )
        bSizer20.Add( self.m_AccODR, 0, wx.ALL, 5 )


        sbSizer1.Add( bSizer20, 1, wx.EXPAND, 5 )


        bSizer41.Add( sbSizer1, 1, wx.EXPAND, 5 )


        bSizer10.Add( bSizer41, 1, wx.EXPAND, 5 )

        self.m_staticline11 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL )
        bSizer10.Add( self.m_staticline11, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer5 = wx.BoxSizer( wx.VERTICAL )

        sbSizer2 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Gyroscope" ), wx.VERTICAL )

        self.m_enableGy = wx.CheckBox( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer2.Add( self.m_enableGy, 0, wx.ALL, 5 )

        bSizer21 = wx.BoxSizer( wx.HORIZONTAL )

        m_GyFSChoices = [ u"Radio Button" ]
        self.m_GyFS = wx.RadioBox( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Full Scale (dps)", wx.DefaultPosition, wx.DefaultSize, m_gyDps, 1, wx.RA_SPECIFY_COLS )
        self.m_GyFS.SetSelection( 0 )
        bSizer21.Add( self.m_GyFS, 0, wx.ALL, 5 )

        m_GyOdrChoices = [ u"Radio Button" ]
        self.m_GyOdr = wx.RadioBox( sbSizer2.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_gyOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_GyOdr.SetSelection( 0 )
        bSizer21.Add( self.m_GyOdr, 0, wx.ALL, 5 )


        sbSizer2.Add( bSizer21, 1, wx.EXPAND, 5 )


        bSizer5.Add( sbSizer2, 1, wx.EXPAND, 5 )


        bSizer10.Add( bSizer5, 1, wx.EXPAND, 5 )

        self.m_staticline121 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL )
        bSizer10.Add( self.m_staticline121, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer51 = wx.BoxSizer( wx.VERTICAL )

        sbSizer3 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Magnetometer" ), wx.VERTICAL )

        self.m_enableMag = wx.CheckBox( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer3.Add( self.m_enableMag, 0, wx.ALL, 5 )

        bSizer22 = wx.BoxSizer( wx.HORIZONTAL )

        m_MagOdrChoices = [ u"Radio Button" ]
        self.m_MagOdr = wx.RadioBox( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_magOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_MagOdr.SetSelection( 0 )
        bSizer22.Add( self.m_MagOdr, 0, wx.ALL, 5 )

        m_PowerModeChoices = [ u"Radio Button" ]
        self.m_PowerMode = wx.RadioBox( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Power Mode", wx.DefaultPosition, wx.DefaultSize, m_magPm, 1, wx.RA_SPECIFY_COLS )
        self.m_PowerMode.SetSelection( 0 )
        bSizer22.Add( self.m_PowerMode, 0, wx.ALL, 5 )


        sbSizer3.Add( bSizer22, 1, wx.EXPAND, 5 )


        bSizer51.Add( sbSizer3, 1, wx.EXPAND, 5 )


        bSizer10.Add( bSizer51, 1, wx.EXPAND, 5 )


        bSizer4.Add( bSizer10, 1, wx.EXPAND, 5 )

        self.m_staticline12 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        bSizer4.Add( self.m_staticline12, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer11 = wx.BoxSizer( wx.HORIZONTAL )

        bSizer411 = wx.BoxSizer( wx.VERTICAL )

        sbSizer4 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Pressure" ), wx.VERTICAL )

        self.m_enablePress = wx.CheckBox( sbSizer4.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer4.Add( self.m_enablePress, 0, wx.ALL, 5 )

        bSizer23 = wx.BoxSizer( wx.HORIZONTAL )

        m_PressODRChoices = [ u"Radio Button" ]
        self.m_PressODR = wx.RadioBox( sbSizer4.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_pressOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_PressODR.SetSelection( 0 )
        bSizer23.Add( self.m_PressODR, 0, wx.ALL, 5 )


        sbSizer4.Add( bSizer23, 1, wx.EXPAND, 5 )


        bSizer411.Add( sbSizer4, 1, wx.EXPAND, 5 )


        bSizer11.Add( bSizer411, 1, wx.EXPAND, 5 )

        self.m_staticline13 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL )
        bSizer11.Add( self.m_staticline13, 0, wx.ALL|wx.EXPAND, 5 )

        bSizer4111 = wx.BoxSizer( wx.VERTICAL )

        sbSizer5 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Humidity" ), wx.VERTICAL )

        self.m_enableHum = wx.CheckBox( sbSizer5.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer5.Add( self.m_enableHum, 0, wx.ALL, 5 )

        bSizer241 = wx.BoxSizer( wx.HORIZONTAL )

        m_humOdrChoices = [ u"Radio Button" ]
        self.m_humOdr = wx.RadioBox( sbSizer5.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_humOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_humOdr.SetSelection( 0 )
        bSizer241.Add( self.m_humOdr, 0, wx.ALL, 5 )

        m_humIASChoices = [ u"Radio Button" ]
        self.m_humIAS = wx.RadioBox( sbSizer5.GetStaticBox(), wx.ID_ANY, u"Internal average samples", wx.DefaultPosition, wx.DefaultSize, m_humAvg, 1, wx.RA_SPECIFY_COLS )
        self.m_humIAS.SetSelection( 0 )
        bSizer241.Add( self.m_humIAS, 0, wx.ALL, 5 )


        sbSizer5.Add( bSizer241, 1, wx.EXPAND, 5 )


        bSizer4111.Add( sbSizer5, 1, wx.EXPAND, 5 )


        bSizer11.Add( bSizer4111, 1, wx.EXPAND, 5 )

        self.m_staticline14 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL )
        bSizer11.Add( self.m_staticline14, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer41111 = wx.BoxSizer( wx.VERTICAL )

        sbSizer6 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Temperature" ), wx.VERTICAL )

        self.m_enableTemp = wx.CheckBox( sbSizer6.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer6.Add( self.m_enableTemp, 0, wx.ALL, 5 )

        bSizer25 = wx.BoxSizer( wx.HORIZONTAL )

        m_TempIASChoices = [ u"Radio Button" ]
        self.m_TempIAS = wx.RadioBox( sbSizer6.GetStaticBox(), wx.ID_ANY, u"Internal average samples", wx.DefaultPosition, wx.DefaultSize, m_tempAvg, 1, wx.RA_SPECIFY_COLS )
        self.m_TempIAS.SetSelection( 0 )
        bSizer25.Add( self.m_TempIAS, 0, wx.ALL, 5 )


        sbSizer6.Add( bSizer25, 1, wx.EXPAND, 5 )


        bSizer41111.Add( sbSizer6, 1, wx.EXPAND, 5 )


        bSizer11.Add( bSizer41111, 1, wx.EXPAND, 5 )

        self.m_staticline7 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL )
        bSizer11.Add( self.m_staticline7, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer511 = wx.BoxSizer( wx.VERTICAL )

        sbSizer7 = wx.StaticBoxSizer( wx.StaticBox( self, wx.ID_ANY, u"Light" ), wx.VERTICAL )

        self.m_enableLight = wx.CheckBox( sbSizer7.GetStaticBox(), wx.ID_ANY, u"Enable", wx.DefaultPosition, wx.DefaultSize, 0 )
        sbSizer7.Add( self.m_enableLight, 0, wx.ALL, 5 )

        bSizer26 = wx.BoxSizer( wx.HORIZONTAL )

        m_LightODRChoices = [ u"Radio Button" ]
        self.m_LightODR = wx.RadioBox( sbSizer7.GetStaticBox(), wx.ID_ANY, u"Output Data Rate (Hz)", wx.DefaultPosition, wx.DefaultSize, m_lightOdr, 1, wx.RA_SPECIFY_COLS )
        self.m_LightODR.SetSelection( 0 )
        bSizer26.Add( self.m_LightODR, 0, wx.ALL, 5 )

        m_LightPMChoices = [ u"Radio Button" ]
        self.m_LightPM = wx.RadioBox( sbSizer7.GetStaticBox(), wx.ID_ANY, u"Power Mode", wx.DefaultPosition, wx.DefaultSize, m_lightPm, 1, wx.RA_SPECIFY_COLS )
        self.m_LightPM.SetSelection( 0 )
        bSizer26.Add( self.m_LightPM, 0, wx.ALL, 5 )


        sbSizer7.Add( bSizer26, 1, wx.EXPAND, 5 )


        bSizer511.Add( sbSizer7, 1, wx.EXPAND, 5 )


        bSizer11.Add( bSizer511, 1, wx.EXPAND, 5 )


        bSizer4.Add( bSizer11, 1, wx.EXPAND, 5 )

        self.m_staticline6 = wx.StaticLine( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        bSizer4.Add( self.m_staticline6, 0, wx.EXPAND |wx.ALL, 5 )

        bSizer12 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_staticText24 = wx.StaticText( self, wx.ID_ANY, u"Period transmit data:", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText24.Wrap( -1 )

        bSizer12.Add( self.m_staticText24, 0, wx.ALL, 5 )

        self.m_slider1 = wx.Slider( self, wx.ID_ANY, 50, 0, 100, wx.DefaultPosition, wx.DefaultSize, wx.SL_HORIZONTAL )
        bSizer12.Add( self.m_slider1, 0, wx.ALL, 5 )

        self.m_staticText25 = wx.StaticText( self, wx.ID_ANY, u"0 second", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText25.Wrap( -1 )

        bSizer12.Add( self.m_staticText25, 0, wx.ALL, 5 )

        self.m_buttonReset = wx.Button( self, wx.ID_ANY, u"Reset", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer12.Add( self.m_buttonReset, 0, wx.ALL, 5 )

        self.m_buttonSave = wx.Button( self, wx.ID_ANY, u"Save", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer12.Add( self.m_buttonSave, 0, wx.ALL, 5 )

        self.m_buttonBack = wx.Button( self, wx.ID_ANY, u"Back", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer12.Add( self.m_buttonBack, 0, wx.ALL, 5 )


        bSizer4.Add( bSizer12, 1, wx.EXPAND, 5 )


        self.SetSizer( bSizer4 )



        self.Centre( wx.BOTH )

        # Connect Events
        self.m_buttonReset.Bind( wx.EVT_BUTTON, self.resetConfig )
        self.m_buttonSave.Bind( wx.EVT_BUTTON, self.saveConfig )
        self.m_buttonBack.Bind( wx.EVT_BUTTON, self.backConfig )

    def __del__( self ):
        pass


    # Virtual event handlers, override them in your derived class
    def resetConfig( self, event ):
        event.Skip()

    def saveConfig( self, event ):
        event.Skip()

    def backConfig( self, event ):
        event.Skip()

