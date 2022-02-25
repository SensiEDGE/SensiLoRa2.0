# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import serial
import serial.tools.list_ports
import time
import wx

STM32_USB_PID_FS = 22336

class MemoryManager:

    def __init__(self, parent):
        self.m_nameDevice = None
        self.m_versDevice = None
        self.m_percent = None
        self.m_devEui = None
        self.m_joinEui = None
        self.m_appKey = None

    def setNameDevice(self, name):
        self.m_nameDevice = name

    def getNameDevice(self):
        return self.m_nameDevice

    def setVersionDevice(self, vers):
        self.m_versDevice = vers

    def getVersionDevice(self):
        return self.m_versDevice

    def setBatteryPercent(self, percent):
        self.m_percent = percent

    def getBatteryPercent(self):
        return self.m_percent

    def setDevEui(self, devEui):
        self.m_devEui = devEui

    def getDevEui(self):
        return self.m_devEui

    def setJoinEui(self, joinEui):
        self.m_joinEui = joinEui

    def getJoinEui(self):
        return self.m_joinEui

    def setAppKey(self, appKey):
        self.m_appKey = appKey

    def getAppKey(self):
        return self.m_appKey