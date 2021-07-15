# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'button_test.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!
import sys
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets

import subprocess
from subprocess import PIPE, time, signal, sys, os

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(505, 384)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(150, 140, 168, 28))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(self.widget)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.widget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 505, 23))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.pushButton.clicked.connect(self.clickCallback_lane1)
        self.pushButton_2.clicked.connect(self.clickCallback_lane2)

    def clickCallback_lane1(self):
        subprocess.run(["rosnode", "kill", "waypoint_loader"])
        subprocess.run(["rosnode", "kill", "waypoint_marker_publisher"])
        subprocess.run(["rosnode", "kill", "waypoint_replanner"])
        self.proc1 = subprocess.Popen(["roslaunch", "waypoint_maker", "waypoint_loader.launch", "load_csv:=True", "multi_lane_csv:=/home/itolab-chotaro/Senior_car/210714_shintoshin_lane_1.csv"])
        print("lane is changed")
    def clickCallback_lane2(self):
        subprocess.run(["rosnode", "kill", "waypoint_loader"])
        subprocess.run(["rosnode", "kill", "waypoint_marker_publisher"])
        subprocess.run(["rosnode", "kill", "waypoint_replanner"])
        self.proc2 = subprocess.Popen(["roslaunch", "waypoint_maker", "waypoint_loader.launch", "load_csv:=True", "multi_lane_csv:=/home/itolab-chotaro/Senior_car/210714_shintoshin_lane_2.csv"])
        print("lane is changed")

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "lane1"))
        self.pushButton_2.setText(_translate("MainWindow", "lane2"))


