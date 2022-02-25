# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import wx
import serial
import serial.tools.list_ports
import time
import wx

from threading import Thread
from usbManager import UsbManager
from Gui import Window
from memoryManager import MemoryManager

usbManager = None
ex = None







class TestThread(Thread):
    """Test Worker Thread Class."""

    # ----------------------------------------------------------------------
    def __init__(self, wxObject):
        """Init Worker Thread Class."""
        Thread.__init__(self)
        self.m_wxObject = wxObject
        self.start()  # запускаем поток
        self.m = self

    # ----------------------------------------------------------------------
    def run(self):
        """Run Worker Thread."""

        # while(1):
        l_list = usbManager.connecting()
        if (usbManager.readInfo(l_list)):
             ex.showConfigWindow()
             ex.showOk()
            # self.Hide()

            # wx.PostEvent(self.wxObject,ResultEvent(1))
        time.sleep(1)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # drawConnectView()
    m_mm = MemoryManager(None)
    usbManager = UsbManager(m_mm)
    app = wx.App()
    ex = Window(None, "Test", usbManager, m_mm)
    # ex.connect()
    # ex.Two()

    TestThread(ex)
    app.MainLoop()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
# class PanelOne(wx.Panel):
#     """"""
#
#     def __init__(self, parent):
#         """Constructor"""
#         wx.Panel.__init__(self, parent=parent)
#         txt = wx.TextCtrl(self)
#
# import wx.grid as gridlib
# class PanelTwo(wx.Panel):
#     """"""
#
#     def __init__(self, parent):
#         """Constructor"""
#         wx.Panel.__init__(self, parent=parent)
#
#         grid = gridlib.Grid(self)
#         grid.CreateGrid(25, 12)
#
#         sizer = wx.BoxSizer(wx.VERTICAL)
#         sizer.Add(grid, 0, wx.EXPAND)
#         self.SetSizer(sizer)
#
#
# class MyForm(wx.Frame):
#
#     def __init__(self):
#         wx.Frame.__init__(self, None, wx.ID_ANY,
#                           "Panel Switcher Tutorial")
#
#         self.panel_one = MyPanel(self)
#         self.panel_two = MyPanel1(self)
#         self.panel_two.Hide()
#
#         self.sizer = wx.BoxSizer(wx.VERTICAL)
#         self.sizer.Add(self.panel_one, 1, wx.EXPAND)
#         self.sizer.Add(self.panel_two, 1, wx.EXPAND)
#         self.SetSizer(self.sizer)
#
#         menubar = wx.MenuBar()
#         fileMenu = wx.Menu()
#         switch_panels_menu_item = fileMenu.Append(
#             wx.ID_ANY,
#             "Switch Panels",
#             "Some text")
#         self.Bind(wx.EVT_MENU, self.onSwitchPanels,
#                   switch_panels_menu_item)
#         menubar.Append(fileMenu, '&File')
#         self.SetMenuBar(menubar)
#
#     def onSwitchPanels(self, event):
#         """
#         Хэндлер события, который запускается при переключении панелей
#         """
#         if self.panel_one.IsShown():
#             self.SetTitle("Panel Two Showing")
#             self.panel_one.Hide()
#             self.panel_two.Show()
#         else:
#             self.SetTitle("Panel One Showing")
#             self.panel_one.Show()
#             self.panel_two.Hide()
#         self.Layout()
#
# # Запускает программу
# if __name__ == "__main__":
#     app = wx.App(False)
#     frame = MyForm()
#     frame.Show()
#     app.MainLoop()