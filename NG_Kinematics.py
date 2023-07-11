# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'NG_Kinematics.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_NG_Kinematics(object):
    def setupUi(self, NG_Kinematics):
        NG_Kinematics.setObjectName("NG_Kinematics")
        NG_Kinematics.resize(736, 460)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(NG_Kinematics.sizePolicy().hasHeightForWidth())
        NG_Kinematics.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(NG_Kinematics)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.Start = QtWidgets.QPushButton(self.centralwidget)
        self.Start.setObjectName("Start")
        self.verticalLayout.addWidget(self.Start)
        self.Calibration = QtWidgets.QPushButton(self.centralwidget)
        self.Calibration.setObjectName("Calibration")
        self.verticalLayout.addWidget(self.Calibration)
        self.Stop = QtWidgets.QPushButton(self.centralwidget)
        self.Stop.setObjectName("Stop")
        self.verticalLayout.addWidget(self.Stop)
        self.Construction = QtWidgets.QPushButton(self.centralwidget)
        self.Construction.setObjectName("Construction")
        self.verticalLayout.addWidget(self.Construction)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.graphicsView = GraphicsLayoutWidget(self.centralwidget)
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout.addWidget(self.graphicsView)
        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 5)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        NG_Kinematics.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(NG_Kinematics)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 736, 26))
        self.menubar.setObjectName("menubar")
        NG_Kinematics.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(NG_Kinematics)
        self.statusbar.setObjectName("statusbar")
        NG_Kinematics.setStatusBar(self.statusbar)

        self.retranslateUi(NG_Kinematics)
        QtCore.QMetaObject.connectSlotsByName(NG_Kinematics)

    def retranslateUi(self, NG_Kinematics):
        _translate = QtCore.QCoreApplication.translate
        NG_Kinematics.setWindowTitle(_translate("NG_Kinematics", "NG_Kinematics"))
        self.Start.setText(_translate("NG_Kinematics", "Start"))
        self.Calibration.setText(_translate("NG_Kinematics", "Calibration"))
        self.Stop.setText(_translate("NG_Kinematics", "Stop"))
        self.Construction.setText(_translate("NG_Kinematics", "Contruction"))

from pyqtgraph import GraphicsLayoutWidget
