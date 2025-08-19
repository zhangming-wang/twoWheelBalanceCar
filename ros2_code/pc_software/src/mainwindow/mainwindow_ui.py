# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QDockWidget, QDoubleSpinBox,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QRadioButton, QSizePolicy, QSpacerItem,
    QSpinBox, QSplitter, QStatusBar, QTabWidget,
    QTextEdit, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1092, 1073)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_38 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_38.setObjectName(u"verticalLayout_38")
        self.splitter = QSplitter(self.centralwidget)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Horizontal)
        self.widget_main = QWidget(self.splitter)
        self.widget_main.setObjectName(u"widget_main")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_main.sizePolicy().hasHeightForWidth())
        self.widget_main.setSizePolicy(sizePolicy)
        self.splitter.addWidget(self.widget_main)
        self.layoutWidget = QWidget(self.splitter)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.verticalLayout_22 = QVBoxLayout(self.layoutWidget)
        self.verticalLayout_22.setObjectName(u"verticalLayout_22")
        self.verticalLayout_22.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_38 = QHBoxLayout()
        self.horizontalLayout_38.setObjectName(u"horizontalLayout_38")
        self.pushButton_reset = QPushButton(self.layoutWidget)
        self.pushButton_reset.setObjectName(u"pushButton_reset")
        self.pushButton_reset.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_38.addWidget(self.pushButton_reset)

        self.pushButton_show_dockWidget = QPushButton(self.layoutWidget)
        self.pushButton_show_dockWidget.setObjectName(u"pushButton_show_dockWidget")
        self.pushButton_show_dockWidget.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_38.addWidget(self.pushButton_show_dockWidget)


        self.verticalLayout_22.addLayout(self.horizontalLayout_38)

        self.verticalSpacer_8 = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_22.addItem(self.verticalSpacer_8)

        self.tabWidget_motion = QTabWidget(self.layoutWidget)
        self.tabWidget_motion.setObjectName(u"tabWidget_motion")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.tabWidget_motion.sizePolicy().hasHeightForWidth())
        self.tabWidget_motion.setSizePolicy(sizePolicy1)
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.horizontalLayout = QHBoxLayout(self.tab_3)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_3.addItem(self.verticalSpacer)

        self.pushButton_move_left = QPushButton(self.tab_3)
        self.pushButton_move_left.setObjectName(u"pushButton_move_left")
        sizePolicy1.setHeightForWidth(self.pushButton_move_left.sizePolicy().hasHeightForWidth())
        self.pushButton_move_left.setSizePolicy(sizePolicy1)
        self.pushButton_move_left.setMinimumSize(QSize(0, 0))

        self.verticalLayout_3.addWidget(self.pushButton_move_left)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_3.addItem(self.verticalSpacer_2)

        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 2)
        self.verticalLayout_3.setStretch(2, 1)

        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.pushButton_move_front = QPushButton(self.tab_3)
        self.pushButton_move_front.setObjectName(u"pushButton_move_front")
        sizePolicy1.setHeightForWidth(self.pushButton_move_front.sizePolicy().hasHeightForWidth())
        self.pushButton_move_front.setSizePolicy(sizePolicy1)
        self.pushButton_move_front.setMinimumSize(QSize(0, 0))

        self.verticalLayout.addWidget(self.pushButton_move_front)

        self.pushButton_move_back = QPushButton(self.tab_3)
        self.pushButton_move_back.setObjectName(u"pushButton_move_back")
        sizePolicy1.setHeightForWidth(self.pushButton_move_back.sizePolicy().hasHeightForWidth())
        self.pushButton_move_back.setSizePolicy(sizePolicy1)
        self.pushButton_move_back.setMinimumSize(QSize(0, 0))

        self.verticalLayout.addWidget(self.pushButton_move_back)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_2.addItem(self.verticalSpacer_3)

        self.pushButton_move_right = QPushButton(self.tab_3)
        self.pushButton_move_right.setObjectName(u"pushButton_move_right")
        sizePolicy1.setHeightForWidth(self.pushButton_move_right.sizePolicy().hasHeightForWidth())
        self.pushButton_move_right.setSizePolicy(sizePolicy1)
        self.pushButton_move_right.setMinimumSize(QSize(0, 0))

        self.verticalLayout_2.addWidget(self.pushButton_move_right)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_2.addItem(self.verticalSpacer_4)

        self.verticalLayout_2.setStretch(0, 1)
        self.verticalLayout_2.setStretch(1, 2)
        self.verticalLayout_2.setStretch(2, 1)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.tabWidget_motion.addTab(self.tab_3, "")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.verticalLayout_7 = QVBoxLayout(self.tab)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label = QLabel(self.tab)
        self.label.setObjectName(u"label")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy2)

        self.horizontalLayout_2.addWidget(self.label)

        self.doubleSpinBox_lineX_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_lineX_spd.setObjectName(u"doubleSpinBox_lineX_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_lineX_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_lineX_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_lineX_spd.setDecimals(3)
        self.doubleSpinBox_lineX_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_lineX_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_2.addWidget(self.doubleSpinBox_lineX_spd)

        self.label_7 = QLabel(self.tab)
        self.label_7.setObjectName(u"label_7")
        sizePolicy2.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy2)

        self.horizontalLayout_2.addWidget(self.label_7)


        self.verticalLayout_4.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_2 = QLabel(self.tab)
        self.label_2.setObjectName(u"label_2")
        sizePolicy2.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy2)

        self.horizontalLayout_3.addWidget(self.label_2)

        self.doubleSpinBox_lineY_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_lineY_spd.setObjectName(u"doubleSpinBox_lineY_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_lineY_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_lineY_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_lineY_spd.setDecimals(3)
        self.doubleSpinBox_lineY_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_lineY_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_3.addWidget(self.doubleSpinBox_lineY_spd)

        self.label_8 = QLabel(self.tab)
        self.label_8.setObjectName(u"label_8")
        sizePolicy2.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy2)

        self.horizontalLayout_3.addWidget(self.label_8)


        self.verticalLayout_4.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_3 = QLabel(self.tab)
        self.label_3.setObjectName(u"label_3")
        sizePolicy2.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy2)

        self.horizontalLayout_4.addWidget(self.label_3)

        self.doubleSpinBox_lineZ_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_lineZ_spd.setObjectName(u"doubleSpinBox_lineZ_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_lineZ_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_lineZ_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_lineZ_spd.setDecimals(3)
        self.doubleSpinBox_lineZ_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_lineZ_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_4.addWidget(self.doubleSpinBox_lineZ_spd)

        self.label_9 = QLabel(self.tab)
        self.label_9.setObjectName(u"label_9")
        sizePolicy2.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy2)

        self.horizontalLayout_4.addWidget(self.label_9)


        self.verticalLayout_4.addLayout(self.horizontalLayout_4)


        self.horizontalLayout_9.addLayout(self.verticalLayout_4)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_4 = QLabel(self.tab)
        self.label_4.setObjectName(u"label_4")
        sizePolicy2.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy2)

        self.horizontalLayout_5.addWidget(self.label_4)

        self.doubleSpinBox_angleX_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_angleX_spd.setObjectName(u"doubleSpinBox_angleX_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_angleX_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_angleX_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_angleX_spd.setDecimals(3)
        self.doubleSpinBox_angleX_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_angleX_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_5.addWidget(self.doubleSpinBox_angleX_spd)

        self.label_10 = QLabel(self.tab)
        self.label_10.setObjectName(u"label_10")
        sizePolicy2.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy2)

        self.horizontalLayout_5.addWidget(self.label_10)


        self.verticalLayout_6.addLayout(self.horizontalLayout_5)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setSpacing(0)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_5 = QLabel(self.tab)
        self.label_5.setObjectName(u"label_5")
        sizePolicy2.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy2)

        self.horizontalLayout_6.addWidget(self.label_5)

        self.doubleSpinBox_angleY_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_angleY_spd.setObjectName(u"doubleSpinBox_angleY_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_angleY_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_angleY_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_angleY_spd.setDecimals(3)
        self.doubleSpinBox_angleY_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_angleY_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_6.addWidget(self.doubleSpinBox_angleY_spd)

        self.label_11 = QLabel(self.tab)
        self.label_11.setObjectName(u"label_11")
        sizePolicy2.setHeightForWidth(self.label_11.sizePolicy().hasHeightForWidth())
        self.label_11.setSizePolicy(sizePolicy2)

        self.horizontalLayout_6.addWidget(self.label_11)


        self.verticalLayout_6.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setSpacing(0)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_6 = QLabel(self.tab)
        self.label_6.setObjectName(u"label_6")
        sizePolicy2.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy2)

        self.horizontalLayout_7.addWidget(self.label_6)

        self.doubleSpinBox_angleZ_spd = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_angleZ_spd.setObjectName(u"doubleSpinBox_angleZ_spd")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_angleZ_spd.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_angleZ_spd.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_angleZ_spd.setDecimals(3)
        self.doubleSpinBox_angleZ_spd.setMinimum(-9999.000000000000000)
        self.doubleSpinBox_angleZ_spd.setMaximum(9999.000000000000000)

        self.horizontalLayout_7.addWidget(self.doubleSpinBox_angleZ_spd)

        self.label_12 = QLabel(self.tab)
        self.label_12.setObjectName(u"label_12")
        sizePolicy2.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy2)

        self.horizontalLayout_7.addWidget(self.label_12)


        self.verticalLayout_6.addLayout(self.horizontalLayout_7)


        self.horizontalLayout_9.addLayout(self.verticalLayout_6)

        self.horizontalLayout_9.setStretch(0, 1)
        self.horizontalLayout_9.setStretch(1, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_9)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.pushButton_set_speed = QPushButton(self.tab)
        self.pushButton_set_speed.setObjectName(u"pushButton_set_speed")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.pushButton_set_speed.sizePolicy().hasHeightForWidth())
        self.pushButton_set_speed.setSizePolicy(sizePolicy3)
        self.pushButton_set_speed.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_8.addWidget(self.pushButton_set_speed)


        self.verticalLayout_7.addLayout(self.horizontalLayout_8)

        self.tabWidget_motion.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.verticalLayout_11 = QVBoxLayout(self.tab_2)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.horizontalLayout_21 = QHBoxLayout()
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.radioButton_cartesian_coordinates = QRadioButton(self.tab_2)
        self.radioButton_cartesian_coordinates.setObjectName(u"radioButton_cartesian_coordinates")

        self.horizontalLayout_21.addWidget(self.radioButton_cartesian_coordinates)

        self.radioButton_polar_coordinates = QRadioButton(self.tab_2)
        self.radioButton_polar_coordinates.setObjectName(u"radioButton_polar_coordinates")

        self.horizontalLayout_21.addWidget(self.radioButton_polar_coordinates)


        self.verticalLayout_11.addLayout(self.horizontalLayout_21)

        self.widget_cartesian_coordinates = QWidget(self.tab_2)
        self.widget_cartesian_coordinates.setObjectName(u"widget_cartesian_coordinates")
        self.horizontalLayout_24 = QHBoxLayout(self.widget_cartesian_coordinates)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.horizontalLayout_24.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_22 = QHBoxLayout()
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.label_25 = QLabel(self.widget_cartesian_coordinates)
        self.label_25.setObjectName(u"label_25")
        sizePolicy2.setHeightForWidth(self.label_25.sizePolicy().hasHeightForWidth())
        self.label_25.setSizePolicy(sizePolicy2)

        self.horizontalLayout_22.addWidget(self.label_25)

        self.doubleSpinBox_path_x = QDoubleSpinBox(self.widget_cartesian_coordinates)
        self.doubleSpinBox_path_x.setObjectName(u"doubleSpinBox_path_x")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_path_x.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_path_x.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_path_x.setDecimals(3)

        self.horizontalLayout_22.addWidget(self.doubleSpinBox_path_x)

        self.label_26 = QLabel(self.widget_cartesian_coordinates)
        self.label_26.setObjectName(u"label_26")
        sizePolicy2.setHeightForWidth(self.label_26.sizePolicy().hasHeightForWidth())
        self.label_26.setSizePolicy(sizePolicy2)

        self.horizontalLayout_22.addWidget(self.label_26)


        self.horizontalLayout_24.addLayout(self.horizontalLayout_22)

        self.horizontalLayout_23 = QHBoxLayout()
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.label_27 = QLabel(self.widget_cartesian_coordinates)
        self.label_27.setObjectName(u"label_27")
        sizePolicy2.setHeightForWidth(self.label_27.sizePolicy().hasHeightForWidth())
        self.label_27.setSizePolicy(sizePolicy2)

        self.horizontalLayout_23.addWidget(self.label_27)

        self.doubleSpinBox_path_y = QDoubleSpinBox(self.widget_cartesian_coordinates)
        self.doubleSpinBox_path_y.setObjectName(u"doubleSpinBox_path_y")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_path_y.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_path_y.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_path_y.setDecimals(3)

        self.horizontalLayout_23.addWidget(self.doubleSpinBox_path_y)

        self.label_28 = QLabel(self.widget_cartesian_coordinates)
        self.label_28.setObjectName(u"label_28")
        sizePolicy2.setHeightForWidth(self.label_28.sizePolicy().hasHeightForWidth())
        self.label_28.setSizePolicy(sizePolicy2)

        self.horizontalLayout_23.addWidget(self.label_28)


        self.horizontalLayout_24.addLayout(self.horizontalLayout_23)


        self.verticalLayout_11.addWidget(self.widget_cartesian_coordinates)

        self.widget_polar_coordinates = QWidget(self.tab_2)
        self.widget_polar_coordinates.setObjectName(u"widget_polar_coordinates")
        self.horizontalLayout_17 = QHBoxLayout(self.widget_polar_coordinates)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalLayout_17.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.label_21 = QLabel(self.widget_polar_coordinates)
        self.label_21.setObjectName(u"label_21")
        sizePolicy2.setHeightForWidth(self.label_21.sizePolicy().hasHeightForWidth())
        self.label_21.setSizePolicy(sizePolicy2)

        self.horizontalLayout_11.addWidget(self.label_21)

        self.doubleSpinBox_path_dis = QDoubleSpinBox(self.widget_polar_coordinates)
        self.doubleSpinBox_path_dis.setObjectName(u"doubleSpinBox_path_dis")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_path_dis.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_path_dis.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_path_dis.setDecimals(3)

        self.horizontalLayout_11.addWidget(self.doubleSpinBox_path_dis)

        self.label_23 = QLabel(self.widget_polar_coordinates)
        self.label_23.setObjectName(u"label_23")
        sizePolicy2.setHeightForWidth(self.label_23.sizePolicy().hasHeightForWidth())
        self.label_23.setSizePolicy(sizePolicy2)

        self.horizontalLayout_11.addWidget(self.label_23)


        self.horizontalLayout_17.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.label_22 = QLabel(self.widget_polar_coordinates)
        self.label_22.setObjectName(u"label_22")
        sizePolicy2.setHeightForWidth(self.label_22.sizePolicy().hasHeightForWidth())
        self.label_22.setSizePolicy(sizePolicy2)

        self.horizontalLayout_14.addWidget(self.label_22)

        self.doubleSpinBox_path_angle = QDoubleSpinBox(self.widget_polar_coordinates)
        self.doubleSpinBox_path_angle.setObjectName(u"doubleSpinBox_path_angle")
        sizePolicy1.setHeightForWidth(self.doubleSpinBox_path_angle.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_path_angle.setSizePolicy(sizePolicy1)
        self.doubleSpinBox_path_angle.setDecimals(3)

        self.horizontalLayout_14.addWidget(self.doubleSpinBox_path_angle)

        self.label_24 = QLabel(self.widget_polar_coordinates)
        self.label_24.setObjectName(u"label_24")
        sizePolicy2.setHeightForWidth(self.label_24.sizePolicy().hasHeightForWidth())
        self.label_24.setSizePolicy(sizePolicy2)

        self.horizontalLayout_14.addWidget(self.label_24)


        self.horizontalLayout_17.addLayout(self.horizontalLayout_14)


        self.verticalLayout_11.addWidget(self.widget_polar_coordinates)

        self.horizontalLayout_18 = QHBoxLayout()
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.pushButton_path_move = QPushButton(self.tab_2)
        self.pushButton_path_move.setObjectName(u"pushButton_path_move")
        sizePolicy1.setHeightForWidth(self.pushButton_path_move.sizePolicy().hasHeightForWidth())
        self.pushButton_path_move.setSizePolicy(sizePolicy1)
        self.pushButton_path_move.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_18.addWidget(self.pushButton_path_move)

        self.pushButton_path_stop = QPushButton(self.tab_2)
        self.pushButton_path_stop.setObjectName(u"pushButton_path_stop")
        sizePolicy1.setHeightForWidth(self.pushButton_path_stop.sizePolicy().hasHeightForWidth())
        self.pushButton_path_stop.setSizePolicy(sizePolicy1)
        self.pushButton_path_stop.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_18.addWidget(self.pushButton_path_stop)


        self.verticalLayout_11.addLayout(self.horizontalLayout_18)

        self.tabWidget_motion.addTab(self.tab_2, "")

        self.verticalLayout_22.addWidget(self.tabWidget_motion)

        self.horizontalLayout_20 = QHBoxLayout()
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.pushButton_stop_slow = QPushButton(self.layoutWidget)
        self.pushButton_stop_slow.setObjectName(u"pushButton_stop_slow")
        sizePolicy1.setHeightForWidth(self.pushButton_stop_slow.sizePolicy().hasHeightForWidth())
        self.pushButton_stop_slow.setSizePolicy(sizePolicy1)
        self.pushButton_stop_slow.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_20.addWidget(self.pushButton_stop_slow)

        self.pushButton_stop_now = QPushButton(self.layoutWidget)
        self.pushButton_stop_now.setObjectName(u"pushButton_stop_now")
        sizePolicy1.setHeightForWidth(self.pushButton_stop_now.sizePolicy().hasHeightForWidth())
        self.pushButton_stop_now.setSizePolicy(sizePolicy1)
        self.pushButton_stop_now.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_20.addWidget(self.pushButton_stop_now)


        self.verticalLayout_22.addLayout(self.horizontalLayout_20)

        self.groupBox_3 = QGroupBox(self.layoutWidget)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.verticalLayout_14 = QVBoxLayout(self.groupBox_3)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.horizontalLayout_19 = QHBoxLayout()
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.checkBox_dynami_follow = QCheckBox(self.groupBox_3)
        self.checkBox_dynami_follow.setObjectName(u"checkBox_dynami_follow")
        self.checkBox_dynami_follow.setChecked(True)

        self.horizontalLayout_19.addWidget(self.checkBox_dynami_follow)

        self.checkBox_dynamic_refresh = QCheckBox(self.groupBox_3)
        self.checkBox_dynamic_refresh.setObjectName(u"checkBox_dynamic_refresh")
        self.checkBox_dynamic_refresh.setChecked(True)

        self.horizontalLayout_19.addWidget(self.checkBox_dynamic_refresh)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_19.addItem(self.horizontalSpacer_2)

        self.checkBox_graph_0 = QCheckBox(self.groupBox_3)
        self.checkBox_graph_0.setObjectName(u"checkBox_graph_0")

        self.horizontalLayout_19.addWidget(self.checkBox_graph_0)

        self.checkBox_graph_1 = QCheckBox(self.groupBox_3)
        self.checkBox_graph_1.setObjectName(u"checkBox_graph_1")

        self.horizontalLayout_19.addWidget(self.checkBox_graph_1)

        self.checkBox_graph_2 = QCheckBox(self.groupBox_3)
        self.checkBox_graph_2.setObjectName(u"checkBox_graph_2")

        self.horizontalLayout_19.addWidget(self.checkBox_graph_2)

        self.checkBox_graph_3 = QCheckBox(self.groupBox_3)
        self.checkBox_graph_3.setObjectName(u"checkBox_graph_3")

        self.horizontalLayout_19.addWidget(self.checkBox_graph_3)


        self.verticalLayout_14.addLayout(self.horizontalLayout_19)

        self.pushButton_clear_plot = QPushButton(self.groupBox_3)
        self.pushButton_clear_plot.setObjectName(u"pushButton_clear_plot")
        self.pushButton_clear_plot.setMinimumSize(QSize(0, 40))

        self.verticalLayout_14.addWidget(self.pushButton_clear_plot)


        self.verticalLayout_22.addWidget(self.groupBox_3)

        self.horizontalLayout_28 = QHBoxLayout()
        self.horizontalLayout_28.setObjectName(u"horizontalLayout_28")
        self.groupBox_5 = QGroupBox(self.layoutWidget)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.groupBox_5.setAlignment(Qt.AlignCenter)
        self.verticalLayout_20 = QVBoxLayout(self.groupBox_5)
        self.verticalLayout_20.setObjectName(u"verticalLayout_20")
        self.label_imu_status = QLabel(self.groupBox_5)
        self.label_imu_status.setObjectName(u"label_imu_status")

        self.verticalLayout_20.addWidget(self.label_imu_status)


        self.horizontalLayout_28.addWidget(self.groupBox_5)

        self.groupBox_6 = QGroupBox(self.layoutWidget)
        self.groupBox_6.setObjectName(u"groupBox_6")
        self.groupBox_6.setAlignment(Qt.AlignCenter)
        self.verticalLayout_21 = QVBoxLayout(self.groupBox_6)
        self.verticalLayout_21.setObjectName(u"verticalLayout_21")
        self.label_odom_msg = QLabel(self.groupBox_6)
        self.label_odom_msg.setObjectName(u"label_odom_msg")

        self.verticalLayout_21.addWidget(self.label_odom_msg)


        self.horizontalLayout_28.addWidget(self.groupBox_6)


        self.verticalLayout_22.addLayout(self.horizontalLayout_28)

        self.groupBox_2 = QGroupBox(self.layoutWidget)
        self.groupBox_2.setObjectName(u"groupBox_2")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy4)
        self.verticalLayout_23 = QVBoxLayout(self.groupBox_2)
        self.verticalLayout_23.setObjectName(u"verticalLayout_23")
        self.textEdit_cmd = QTextEdit(self.groupBox_2)
        self.textEdit_cmd.setObjectName(u"textEdit_cmd")
        self.textEdit_cmd.setReadOnly(True)

        self.verticalLayout_23.addWidget(self.textEdit_cmd)

        self.pushButton_clear_cmd_info = QPushButton(self.groupBox_2)
        self.pushButton_clear_cmd_info.setObjectName(u"pushButton_clear_cmd_info")
        self.pushButton_clear_cmd_info.setMinimumSize(QSize(0, 40))

        self.verticalLayout_23.addWidget(self.pushButton_clear_cmd_info)


        self.verticalLayout_22.addWidget(self.groupBox_2)

        self.splitter.addWidget(self.layoutWidget)

        self.verticalLayout_38.addWidget(self.splitter)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusBar = QStatusBar(MainWindow)
        self.statusBar.setObjectName(u"statusBar")
        MainWindow.setStatusBar(self.statusBar)
        self.dockWidget = QDockWidget(MainWindow)
        self.dockWidget.setObjectName(u"dockWidget")
        self.dockWidget.setFloating(True)
        self.dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea|Qt.RightDockWidgetArea)
        self.dockWidgetContents = QWidget()
        self.dockWidgetContents.setObjectName(u"dockWidgetContents")
        self.verticalLayout_16 = QVBoxLayout(self.dockWidgetContents)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.tabWidget_settings = QTabWidget(self.dockWidgetContents)
        self.tabWidget_settings.setObjectName(u"tabWidget_settings")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.tabWidget_settings.sizePolicy().hasHeightForWidth())
        self.tabWidget_settings.setSizePolicy(sizePolicy5)
        self.tab_9 = QWidget()
        self.tab_9.setObjectName(u"tab_9")
        self.verticalLayout_9 = QVBoxLayout(self.tab_9)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.horizontalLayout_40 = QHBoxLayout()
        self.horizontalLayout_40.setObjectName(u"horizontalLayout_40")
        self.radioButton_dynamic_balance = QRadioButton(self.tab_9)
        self.radioButton_dynamic_balance.setObjectName(u"radioButton_dynamic_balance")

        self.horizontalLayout_40.addWidget(self.radioButton_dynamic_balance)

        self.radioButton_static_balance = QRadioButton(self.tab_9)
        self.radioButton_static_balance.setObjectName(u"radioButton_static_balance")

        self.horizontalLayout_40.addWidget(self.radioButton_static_balance)

        self.radioButton_no_balance = QRadioButton(self.tab_9)
        self.radioButton_no_balance.setObjectName(u"radioButton_no_balance")

        self.horizontalLayout_40.addWidget(self.radioButton_no_balance)


        self.verticalLayout_9.addLayout(self.horizontalLayout_40)

        self.horizontalLayout_43 = QHBoxLayout()
        self.horizontalLayout_43.setObjectName(u"horizontalLayout_43")
        self.groupBox_7 = QGroupBox(self.tab_9)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.horizontalLayout_13 = QHBoxLayout(self.groupBox_7)
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.verticalLayout_42 = QVBoxLayout()
        self.verticalLayout_42.setObjectName(u"verticalLayout_42")
        self.label_36 = QLabel(self.groupBox_7)
        self.label_36.setObjectName(u"label_36")
        sizePolicy1.setHeightForWidth(self.label_36.sizePolicy().hasHeightForWidth())
        self.label_36.setSizePolicy(sizePolicy1)
        self.label_36.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_42.addWidget(self.label_36)

        self.label_37 = QLabel(self.groupBox_7)
        self.label_37.setObjectName(u"label_37")
        sizePolicy1.setHeightForWidth(self.label_37.sizePolicy().hasHeightForWidth())
        self.label_37.setSizePolicy(sizePolicy1)
        self.label_37.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_42.addWidget(self.label_37)

        self.label_38 = QLabel(self.groupBox_7)
        self.label_38.setObjectName(u"label_38")
        sizePolicy1.setHeightForWidth(self.label_38.sizePolicy().hasHeightForWidth())
        self.label_38.setSizePolicy(sizePolicy1)
        self.label_38.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_42.addWidget(self.label_38)

        self.label_39 = QLabel(self.groupBox_7)
        self.label_39.setObjectName(u"label_39")
        sizePolicy1.setHeightForWidth(self.label_39.sizePolicy().hasHeightForWidth())
        self.label_39.setSizePolicy(sizePolicy1)
        self.label_39.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_42.addWidget(self.label_39)


        self.horizontalLayout_13.addLayout(self.verticalLayout_42)

        self.verticalLayout_43 = QVBoxLayout()
        self.verticalLayout_43.setObjectName(u"verticalLayout_43")
        self.doubleSpinBox_position_p = QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_position_p.setObjectName(u"doubleSpinBox_position_p")
        sizePolicy6 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy6.setHorizontalStretch(0)
        sizePolicy6.setVerticalStretch(0)
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_position_p.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_position_p.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_position_p.setDecimals(6)
        self.doubleSpinBox_position_p.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_position_p.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_position_p.setValue(0.625000000000000)

        self.verticalLayout_43.addWidget(self.doubleSpinBox_position_p)

        self.doubleSpinBox_position_i = QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_position_i.setObjectName(u"doubleSpinBox_position_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_position_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_position_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_position_i.setDecimals(6)
        self.doubleSpinBox_position_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_position_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_position_i.setValue(0.125000000000000)

        self.verticalLayout_43.addWidget(self.doubleSpinBox_position_i)

        self.doubleSpinBox_position_d = QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_position_d.setObjectName(u"doubleSpinBox_position_d")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_position_d.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_position_d.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_position_d.setDecimals(6)
        self.doubleSpinBox_position_d.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_position_d.setMaximum(99999999.000000000000000)

        self.verticalLayout_43.addWidget(self.doubleSpinBox_position_d)

        self.doubleSpinBox_position_max_i = QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_position_max_i.setObjectName(u"doubleSpinBox_position_max_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_position_max_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_position_max_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_position_max_i.setDecimals(3)
        self.doubleSpinBox_position_max_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_position_max_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_position_max_i.setValue(5000.000000000000000)

        self.verticalLayout_43.addWidget(self.doubleSpinBox_position_max_i)


        self.horizontalLayout_13.addLayout(self.verticalLayout_43)


        self.horizontalLayout_43.addWidget(self.groupBox_7)

        self.groupBox_8 = QGroupBox(self.tab_9)
        self.groupBox_8.setObjectName(u"groupBox_8")
        self.horizontalLayout_39 = QHBoxLayout(self.groupBox_8)
        self.horizontalLayout_39.setObjectName(u"horizontalLayout_39")
        self.verticalLayout_44 = QVBoxLayout()
        self.verticalLayout_44.setObjectName(u"verticalLayout_44")
        self.label_44 = QLabel(self.groupBox_8)
        self.label_44.setObjectName(u"label_44")
        sizePolicy1.setHeightForWidth(self.label_44.sizePolicy().hasHeightForWidth())
        self.label_44.setSizePolicy(sizePolicy1)
        self.label_44.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_44.addWidget(self.label_44)

        self.label_56 = QLabel(self.groupBox_8)
        self.label_56.setObjectName(u"label_56")
        sizePolicy1.setHeightForWidth(self.label_56.sizePolicy().hasHeightForWidth())
        self.label_56.setSizePolicy(sizePolicy1)
        self.label_56.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_44.addWidget(self.label_56)

        self.label_57 = QLabel(self.groupBox_8)
        self.label_57.setObjectName(u"label_57")
        sizePolicy1.setHeightForWidth(self.label_57.sizePolicy().hasHeightForWidth())
        self.label_57.setSizePolicy(sizePolicy1)
        self.label_57.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_44.addWidget(self.label_57)

        self.label_58 = QLabel(self.groupBox_8)
        self.label_58.setObjectName(u"label_58")
        sizePolicy1.setHeightForWidth(self.label_58.sizePolicy().hasHeightForWidth())
        self.label_58.setSizePolicy(sizePolicy1)
        self.label_58.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_44.addWidget(self.label_58)


        self.horizontalLayout_39.addLayout(self.verticalLayout_44)

        self.verticalLayout_45 = QVBoxLayout()
        self.verticalLayout_45.setObjectName(u"verticalLayout_45")
        self.doubleSpinBox_speed_p = QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_p.setObjectName(u"doubleSpinBox_speed_p")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_speed_p.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_speed_p.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_speed_p.setDecimals(6)
        self.doubleSpinBox_speed_p.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_speed_p.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_speed_p.setValue(0.625000000000000)

        self.verticalLayout_45.addWidget(self.doubleSpinBox_speed_p)

        self.doubleSpinBox_speed_i = QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_i.setObjectName(u"doubleSpinBox_speed_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_speed_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_speed_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_speed_i.setDecimals(6)
        self.doubleSpinBox_speed_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_speed_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_speed_i.setValue(0.125000000000000)

        self.verticalLayout_45.addWidget(self.doubleSpinBox_speed_i)

        self.doubleSpinBox_speed_d = QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_d.setObjectName(u"doubleSpinBox_speed_d")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_speed_d.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_speed_d.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_speed_d.setDecimals(6)
        self.doubleSpinBox_speed_d.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_speed_d.setMaximum(99999999.000000000000000)

        self.verticalLayout_45.addWidget(self.doubleSpinBox_speed_d)

        self.doubleSpinBox_speed_max_i = QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_max_i.setObjectName(u"doubleSpinBox_speed_max_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_speed_max_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_speed_max_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_speed_max_i.setDecimals(3)
        self.doubleSpinBox_speed_max_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_speed_max_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_speed_max_i.setValue(5000.000000000000000)

        self.verticalLayout_45.addWidget(self.doubleSpinBox_speed_max_i)


        self.horizontalLayout_39.addLayout(self.verticalLayout_45)


        self.horizontalLayout_43.addWidget(self.groupBox_8)

        self.horizontalLayout_43.setStretch(0, 1)
        self.horizontalLayout_43.setStretch(1, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_43)

        self.horizontalLayout_45 = QHBoxLayout()
        self.horizontalLayout_45.setObjectName(u"horizontalLayout_45")
        self.groupBox_16 = QGroupBox(self.tab_9)
        self.groupBox_16.setObjectName(u"groupBox_16")
        self.horizontalLayout_41 = QHBoxLayout(self.groupBox_16)
        self.horizontalLayout_41.setObjectName(u"horizontalLayout_41")
        self.verticalLayout_46 = QVBoxLayout()
        self.verticalLayout_46.setObjectName(u"verticalLayout_46")
        self.label_59 = QLabel(self.groupBox_16)
        self.label_59.setObjectName(u"label_59")
        sizePolicy1.setHeightForWidth(self.label_59.sizePolicy().hasHeightForWidth())
        self.label_59.setSizePolicy(sizePolicy1)
        self.label_59.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_46.addWidget(self.label_59)

        self.label_60 = QLabel(self.groupBox_16)
        self.label_60.setObjectName(u"label_60")
        sizePolicy1.setHeightForWidth(self.label_60.sizePolicy().hasHeightForWidth())
        self.label_60.setSizePolicy(sizePolicy1)
        self.label_60.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_46.addWidget(self.label_60)

        self.label_61 = QLabel(self.groupBox_16)
        self.label_61.setObjectName(u"label_61")
        sizePolicy1.setHeightForWidth(self.label_61.sizePolicy().hasHeightForWidth())
        self.label_61.setSizePolicy(sizePolicy1)
        self.label_61.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_46.addWidget(self.label_61)

        self.label_62 = QLabel(self.groupBox_16)
        self.label_62.setObjectName(u"label_62")
        sizePolicy1.setHeightForWidth(self.label_62.sizePolicy().hasHeightForWidth())
        self.label_62.setSizePolicy(sizePolicy1)
        self.label_62.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_46.addWidget(self.label_62)


        self.horizontalLayout_41.addLayout(self.verticalLayout_46)

        self.verticalLayout_47 = QVBoxLayout()
        self.verticalLayout_47.setObjectName(u"verticalLayout_47")
        self.doubleSpinBox_attitude_ahead_p = QDoubleSpinBox(self.groupBox_16)
        self.doubleSpinBox_attitude_ahead_p.setObjectName(u"doubleSpinBox_attitude_ahead_p")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_ahead_p.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_ahead_p.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_ahead_p.setDecimals(6)
        self.doubleSpinBox_attitude_ahead_p.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_p.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_p.setValue(0.625000000000000)

        self.verticalLayout_47.addWidget(self.doubleSpinBox_attitude_ahead_p)

        self.doubleSpinBox_attitude_ahead_i = QDoubleSpinBox(self.groupBox_16)
        self.doubleSpinBox_attitude_ahead_i.setObjectName(u"doubleSpinBox_attitude_ahead_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_ahead_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_ahead_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_ahead_i.setDecimals(6)
        self.doubleSpinBox_attitude_ahead_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_i.setValue(0.125000000000000)

        self.verticalLayout_47.addWidget(self.doubleSpinBox_attitude_ahead_i)

        self.doubleSpinBox_attitude_ahead_d = QDoubleSpinBox(self.groupBox_16)
        self.doubleSpinBox_attitude_ahead_d.setObjectName(u"doubleSpinBox_attitude_ahead_d")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_ahead_d.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_ahead_d.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_ahead_d.setDecimals(6)
        self.doubleSpinBox_attitude_ahead_d.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_d.setMaximum(99999999.000000000000000)

        self.verticalLayout_47.addWidget(self.doubleSpinBox_attitude_ahead_d)

        self.doubleSpinBox_attitude_ahead_max_i = QDoubleSpinBox(self.groupBox_16)
        self.doubleSpinBox_attitude_ahead_max_i.setObjectName(u"doubleSpinBox_attitude_ahead_max_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_ahead_max_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_ahead_max_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_ahead_max_i.setDecimals(3)
        self.doubleSpinBox_attitude_ahead_max_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_max_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_ahead_max_i.setValue(5000.000000000000000)

        self.verticalLayout_47.addWidget(self.doubleSpinBox_attitude_ahead_max_i)


        self.horizontalLayout_41.addLayout(self.verticalLayout_47)


        self.horizontalLayout_45.addWidget(self.groupBox_16)

        self.groupBox_19 = QGroupBox(self.tab_9)
        self.groupBox_19.setObjectName(u"groupBox_19")
        self.horizontalLayout_44 = QHBoxLayout(self.groupBox_19)
        self.horizontalLayout_44.setObjectName(u"horizontalLayout_44")
        self.verticalLayout_49 = QVBoxLayout()
        self.verticalLayout_49.setObjectName(u"verticalLayout_49")
        self.label_63 = QLabel(self.groupBox_19)
        self.label_63.setObjectName(u"label_63")
        sizePolicy1.setHeightForWidth(self.label_63.sizePolicy().hasHeightForWidth())
        self.label_63.setSizePolicy(sizePolicy1)
        self.label_63.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_49.addWidget(self.label_63)

        self.label_64 = QLabel(self.groupBox_19)
        self.label_64.setObjectName(u"label_64")
        sizePolicy1.setHeightForWidth(self.label_64.sizePolicy().hasHeightForWidth())
        self.label_64.setSizePolicy(sizePolicy1)
        self.label_64.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_49.addWidget(self.label_64)

        self.label_65 = QLabel(self.groupBox_19)
        self.label_65.setObjectName(u"label_65")
        sizePolicy1.setHeightForWidth(self.label_65.sizePolicy().hasHeightForWidth())
        self.label_65.setSizePolicy(sizePolicy1)
        self.label_65.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_49.addWidget(self.label_65)

        self.label_66 = QLabel(self.groupBox_19)
        self.label_66.setObjectName(u"label_66")
        sizePolicy1.setHeightForWidth(self.label_66.sizePolicy().hasHeightForWidth())
        self.label_66.setSizePolicy(sizePolicy1)
        self.label_66.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_49.addWidget(self.label_66)


        self.horizontalLayout_44.addLayout(self.verticalLayout_49)

        self.verticalLayout_50 = QVBoxLayout()
        self.verticalLayout_50.setObjectName(u"verticalLayout_50")
        self.doubleSpinBox_attitude_back_p = QDoubleSpinBox(self.groupBox_19)
        self.doubleSpinBox_attitude_back_p.setObjectName(u"doubleSpinBox_attitude_back_p")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_back_p.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_back_p.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_back_p.setDecimals(6)
        self.doubleSpinBox_attitude_back_p.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_back_p.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_back_p.setValue(0.625000000000000)

        self.verticalLayout_50.addWidget(self.doubleSpinBox_attitude_back_p)

        self.doubleSpinBox_attitude_back_i = QDoubleSpinBox(self.groupBox_19)
        self.doubleSpinBox_attitude_back_i.setObjectName(u"doubleSpinBox_attitude_back_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_back_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_back_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_back_i.setDecimals(6)
        self.doubleSpinBox_attitude_back_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_back_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_back_i.setValue(0.125000000000000)

        self.verticalLayout_50.addWidget(self.doubleSpinBox_attitude_back_i)

        self.doubleSpinBox_attitude_back_d = QDoubleSpinBox(self.groupBox_19)
        self.doubleSpinBox_attitude_back_d.setObjectName(u"doubleSpinBox_attitude_back_d")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_back_d.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_back_d.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_back_d.setDecimals(6)
        self.doubleSpinBox_attitude_back_d.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_back_d.setMaximum(99999999.000000000000000)

        self.verticalLayout_50.addWidget(self.doubleSpinBox_attitude_back_d)

        self.doubleSpinBox_attitude_back_max_i = QDoubleSpinBox(self.groupBox_19)
        self.doubleSpinBox_attitude_back_max_i.setObjectName(u"doubleSpinBox_attitude_back_max_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_attitude_back_max_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_attitude_back_max_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_attitude_back_max_i.setDecimals(3)
        self.doubleSpinBox_attitude_back_max_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_attitude_back_max_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_attitude_back_max_i.setValue(5000.000000000000000)

        self.verticalLayout_50.addWidget(self.doubleSpinBox_attitude_back_max_i)


        self.horizontalLayout_44.addLayout(self.verticalLayout_50)


        self.horizontalLayout_45.addWidget(self.groupBox_19)

        self.horizontalLayout_45.setStretch(0, 1)
        self.horizontalLayout_45.setStretch(1, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_45)

        self.horizontalLayout_46 = QHBoxLayout()
        self.horizontalLayout_46.setObjectName(u"horizontalLayout_46")
        self.groupBox_17 = QGroupBox(self.tab_9)
        self.groupBox_17.setObjectName(u"groupBox_17")
        self.horizontalLayout_12 = QHBoxLayout(self.groupBox_17)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label_13 = QLabel(self.groupBox_17)
        self.label_13.setObjectName(u"label_13")
        sizePolicy1.setHeightForWidth(self.label_13.sizePolicy().hasHeightForWidth())
        self.label_13.setSizePolicy(sizePolicy1)
        self.label_13.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_5.addWidget(self.label_13)

        self.label_15 = QLabel(self.groupBox_17)
        self.label_15.setObjectName(u"label_15")
        sizePolicy1.setHeightForWidth(self.label_15.sizePolicy().hasHeightForWidth())
        self.label_15.setSizePolicy(sizePolicy1)
        self.label_15.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_5.addWidget(self.label_15)

        self.label_14 = QLabel(self.groupBox_17)
        self.label_14.setObjectName(u"label_14")
        sizePolicy1.setHeightForWidth(self.label_14.sizePolicy().hasHeightForWidth())
        self.label_14.setSizePolicy(sizePolicy1)
        self.label_14.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_5.addWidget(self.label_14)

        self.label_19 = QLabel(self.groupBox_17)
        self.label_19.setObjectName(u"label_19")
        sizePolicy1.setHeightForWidth(self.label_19.sizePolicy().hasHeightForWidth())
        self.label_19.setSizePolicy(sizePolicy1)
        self.label_19.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_5.addWidget(self.label_19)


        self.horizontalLayout_12.addLayout(self.verticalLayout_5)

        self.verticalLayout_10 = QVBoxLayout()
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.doubleSpinBox_motor_p = QDoubleSpinBox(self.groupBox_17)
        self.doubleSpinBox_motor_p.setObjectName(u"doubleSpinBox_motor_p")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_motor_p.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_motor_p.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_motor_p.setDecimals(6)
        self.doubleSpinBox_motor_p.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_motor_p.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_motor_p.setValue(0.625000000000000)

        self.verticalLayout_10.addWidget(self.doubleSpinBox_motor_p)

        self.doubleSpinBox_motor_i = QDoubleSpinBox(self.groupBox_17)
        self.doubleSpinBox_motor_i.setObjectName(u"doubleSpinBox_motor_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_motor_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_motor_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_motor_i.setDecimals(6)
        self.doubleSpinBox_motor_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_motor_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_motor_i.setValue(0.125000000000000)

        self.verticalLayout_10.addWidget(self.doubleSpinBox_motor_i)

        self.doubleSpinBox_motor_d = QDoubleSpinBox(self.groupBox_17)
        self.doubleSpinBox_motor_d.setObjectName(u"doubleSpinBox_motor_d")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_motor_d.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_motor_d.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_motor_d.setDecimals(6)
        self.doubleSpinBox_motor_d.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_motor_d.setMaximum(99999999.000000000000000)

        self.verticalLayout_10.addWidget(self.doubleSpinBox_motor_d)

        self.doubleSpinBox_motor_max_i = QDoubleSpinBox(self.groupBox_17)
        self.doubleSpinBox_motor_max_i.setObjectName(u"doubleSpinBox_motor_max_i")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_motor_max_i.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_motor_max_i.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_motor_max_i.setDecimals(3)
        self.doubleSpinBox_motor_max_i.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_motor_max_i.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_motor_max_i.setValue(5000.000000000000000)

        self.verticalLayout_10.addWidget(self.doubleSpinBox_motor_max_i)


        self.horizontalLayout_12.addLayout(self.verticalLayout_10)


        self.horizontalLayout_46.addWidget(self.groupBox_17)

        self.groupBox_18 = QGroupBox(self.tab_9)
        self.groupBox_18.setObjectName(u"groupBox_18")
        sizePolicy1.setHeightForWidth(self.groupBox_18.sizePolicy().hasHeightForWidth())
        self.groupBox_18.setSizePolicy(sizePolicy1)
        self.horizontalLayout_15 = QHBoxLayout(self.groupBox_18)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_18 = QLabel(self.groupBox_18)
        self.label_18.setObjectName(u"label_18")
        sizePolicy1.setHeightForWidth(self.label_18.sizePolicy().hasHeightForWidth())
        self.label_18.setSizePolicy(sizePolicy1)
        self.label_18.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_8.addWidget(self.label_18)

        self.label_20 = QLabel(self.groupBox_18)
        self.label_20.setObjectName(u"label_20")
        self.label_20.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_8.addWidget(self.label_20)

        self.label_16 = QLabel(self.groupBox_18)
        self.label_16.setObjectName(u"label_16")
        sizePolicy1.setHeightForWidth(self.label_16.sizePolicy().hasHeightForWidth())
        self.label_16.setSizePolicy(sizePolicy1)
        self.label_16.setLayoutDirection(Qt.LeftToRight)
        self.label_16.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_8.addWidget(self.label_16)

        self.label_17 = QLabel(self.groupBox_18)
        self.label_17.setObjectName(u"label_17")
        sizePolicy1.setHeightForWidth(self.label_17.sizePolicy().hasHeightForWidth())
        self.label_17.setSizePolicy(sizePolicy1)
        self.label_17.setLayoutDirection(Qt.LeftToRight)
        self.label_17.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_8.addWidget(self.label_17)


        self.horizontalLayout_15.addLayout(self.verticalLayout_8)

        self.verticalLayout_48 = QVBoxLayout()
        self.verticalLayout_48.setObjectName(u"verticalLayout_48")
        self.spinBox_millseconds = QSpinBox(self.groupBox_18)
        self.spinBox_millseconds.setObjectName(u"spinBox_millseconds")
        sizePolicy6.setHeightForWidth(self.spinBox_millseconds.sizePolicy().hasHeightForWidth())
        self.spinBox_millseconds.setSizePolicy(sizePolicy6)
        self.spinBox_millseconds.setMinimum(-99999999)
        self.spinBox_millseconds.setMaximum(99999999)
        self.spinBox_millseconds.setValue(20)

        self.verticalLayout_48.addWidget(self.spinBox_millseconds)

        self.doubleSpinBox_max_v = QDoubleSpinBox(self.groupBox_18)
        self.doubleSpinBox_max_v.setObjectName(u"doubleSpinBox_max_v")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_max_v.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_max_v.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_max_v.setDecimals(3)
        self.doubleSpinBox_max_v.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_max_v.setMaximum(99999999.000000000000000)

        self.verticalLayout_48.addWidget(self.doubleSpinBox_max_v)

        self.doubleSpinBox_max_acc = QDoubleSpinBox(self.groupBox_18)
        self.doubleSpinBox_max_acc.setObjectName(u"doubleSpinBox_max_acc")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_max_acc.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_max_acc.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_max_acc.setDecimals(3)
        self.doubleSpinBox_max_acc.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_max_acc.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_max_acc.setValue(1.500000000000000)

        self.verticalLayout_48.addWidget(self.doubleSpinBox_max_acc)

        self.doubleSpinBox_jerk = QDoubleSpinBox(self.groupBox_18)
        self.doubleSpinBox_jerk.setObjectName(u"doubleSpinBox_jerk")
        sizePolicy6.setHeightForWidth(self.doubleSpinBox_jerk.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_jerk.setSizePolicy(sizePolicy6)
        self.doubleSpinBox_jerk.setDecimals(3)
        self.doubleSpinBox_jerk.setMinimum(-99999999.000000000000000)
        self.doubleSpinBox_jerk.setMaximum(99999999.000000000000000)
        self.doubleSpinBox_jerk.setValue(0.500000000000000)

        self.verticalLayout_48.addWidget(self.doubleSpinBox_jerk)


        self.horizontalLayout_15.addLayout(self.verticalLayout_48)


        self.horizontalLayout_46.addWidget(self.groupBox_18)

        self.horizontalLayout_46.setStretch(0, 1)
        self.horizontalLayout_46.setStretch(1, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_46)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.pushButton_read_params = QPushButton(self.tab_9)
        self.pushButton_read_params.setObjectName(u"pushButton_read_params")
        sizePolicy7 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy7.setHorizontalStretch(0)
        sizePolicy7.setVerticalStretch(0)
        sizePolicy7.setHeightForWidth(self.pushButton_read_params.sizePolicy().hasHeightForWidth())
        self.pushButton_read_params.setSizePolicy(sizePolicy7)
        self.pushButton_read_params.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_16.addWidget(self.pushButton_read_params)

        self.pushButton_write_params = QPushButton(self.tab_9)
        self.pushButton_write_params.setObjectName(u"pushButton_write_params")
        sizePolicy7.setHeightForWidth(self.pushButton_write_params.sizePolicy().hasHeightForWidth())
        self.pushButton_write_params.setSizePolicy(sizePolicy7)
        self.pushButton_write_params.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_16.addWidget(self.pushButton_write_params)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_16.addItem(self.horizontalSpacer_4)

        self.pushButton_save_params = QPushButton(self.tab_9)
        self.pushButton_save_params.setObjectName(u"pushButton_save_params")
        sizePolicy7.setHeightForWidth(self.pushButton_save_params.sizePolicy().hasHeightForWidth())
        self.pushButton_save_params.setSizePolicy(sizePolicy7)
        self.pushButton_save_params.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_16.addWidget(self.pushButton_save_params)


        self.verticalLayout_9.addLayout(self.horizontalLayout_16)

        self.tabWidget_settings.addTab(self.tab_9, "")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        sizePolicy7.setHeightForWidth(self.tab_5.sizePolicy().hasHeightForWidth())
        self.tab_5.setSizePolicy(sizePolicy7)
        self.verticalLayout_41 = QVBoxLayout(self.tab_5)
        self.verticalLayout_41.setObjectName(u"verticalLayout_41")
        self.groupBox_15 = QGroupBox(self.tab_5)
        self.groupBox_15.setObjectName(u"groupBox_15")
        self.horizontalLayout_30 = QHBoxLayout(self.groupBox_15)
        self.horizontalLayout_30.setObjectName(u"horizontalLayout_30")
        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.verticalLayout_26 = QVBoxLayout()
        self.verticalLayout_26.setObjectName(u"verticalLayout_26")
        self.label_40 = QLabel(self.groupBox_15)
        self.label_40.setObjectName(u"label_40")
        sizePolicy1.setHeightForWidth(self.label_40.sizePolicy().hasHeightForWidth())
        self.label_40.setSizePolicy(sizePolicy1)
        self.label_40.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_26.addWidget(self.label_40)

        self.label_41 = QLabel(self.groupBox_15)
        self.label_41.setObjectName(u"label_41")
        sizePolicy1.setHeightForWidth(self.label_41.sizePolicy().hasHeightForWidth())
        self.label_41.setSizePolicy(sizePolicy1)
        self.label_41.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_26.addWidget(self.label_41)


        self.horizontalLayout_10.addLayout(self.verticalLayout_26)

        self.verticalLayout_19 = QVBoxLayout()
        self.verticalLayout_19.setObjectName(u"verticalLayout_19")
        self.doubleSpinBox_wheel_diameter = QDoubleSpinBox(self.groupBox_15)
        self.doubleSpinBox_wheel_diameter.setObjectName(u"doubleSpinBox_wheel_diameter")
        sizePolicy.setHeightForWidth(self.doubleSpinBox_wheel_diameter.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_wheel_diameter.setSizePolicy(sizePolicy)
        self.doubleSpinBox_wheel_diameter.setDecimals(3)
        self.doubleSpinBox_wheel_diameter.setMinimum(-99.000000000000000)
        self.doubleSpinBox_wheel_diameter.setMaximum(99.000000000000000)

        self.verticalLayout_19.addWidget(self.doubleSpinBox_wheel_diameter)

        self.doubleSpinBox_track_width = QDoubleSpinBox(self.groupBox_15)
        self.doubleSpinBox_track_width.setObjectName(u"doubleSpinBox_track_width")
        sizePolicy.setHeightForWidth(self.doubleSpinBox_track_width.sizePolicy().hasHeightForWidth())
        self.doubleSpinBox_track_width.setSizePolicy(sizePolicy)
        self.doubleSpinBox_track_width.setDecimals(3)
        self.doubleSpinBox_track_width.setMinimum(-99.000000000000000)
        self.doubleSpinBox_track_width.setMaximum(99.000000000000000)

        self.verticalLayout_19.addWidget(self.doubleSpinBox_track_width)


        self.horizontalLayout_10.addLayout(self.verticalLayout_19)


        self.horizontalLayout_30.addLayout(self.horizontalLayout_10)

        self.horizontalLayout_29 = QHBoxLayout()
        self.horizontalLayout_29.setObjectName(u"horizontalLayout_29")
        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.label_42 = QLabel(self.groupBox_15)
        self.label_42.setObjectName(u"label_42")
        sizePolicy1.setHeightForWidth(self.label_42.sizePolicy().hasHeightForWidth())
        self.label_42.setSizePolicy(sizePolicy1)
        self.label_42.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_12.addWidget(self.label_42)

        self.label_43 = QLabel(self.groupBox_15)
        self.label_43.setObjectName(u"label_43")
        sizePolicy1.setHeightForWidth(self.label_43.sizePolicy().hasHeightForWidth())
        self.label_43.setSizePolicy(sizePolicy1)
        self.label_43.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_12.addWidget(self.label_43)


        self.horizontalLayout_29.addLayout(self.verticalLayout_12)

        self.verticalLayout_27 = QVBoxLayout()
        self.verticalLayout_27.setObjectName(u"verticalLayout_27")
        self.spinBox_pluses_per_revolution = QSpinBox(self.groupBox_15)
        self.spinBox_pluses_per_revolution.setObjectName(u"spinBox_pluses_per_revolution")
        sizePolicy.setHeightForWidth(self.spinBox_pluses_per_revolution.sizePolicy().hasHeightForWidth())
        self.spinBox_pluses_per_revolution.setSizePolicy(sizePolicy)
        self.spinBox_pluses_per_revolution.setMinimum(-999999)
        self.spinBox_pluses_per_revolution.setMaximum(999999)

        self.verticalLayout_27.addWidget(self.spinBox_pluses_per_revolution)

        self.spinBox_revolutions_per_minute = QSpinBox(self.groupBox_15)
        self.spinBox_revolutions_per_minute.setObjectName(u"spinBox_revolutions_per_minute")
        sizePolicy.setHeightForWidth(self.spinBox_revolutions_per_minute.sizePolicy().hasHeightForWidth())
        self.spinBox_revolutions_per_minute.setSizePolicy(sizePolicy)
        self.spinBox_revolutions_per_minute.setMinimum(-999999)
        self.spinBox_revolutions_per_minute.setMaximum(999999)

        self.verticalLayout_27.addWidget(self.spinBox_revolutions_per_minute)


        self.horizontalLayout_29.addLayout(self.verticalLayout_27)


        self.horizontalLayout_30.addLayout(self.horizontalLayout_29)


        self.verticalLayout_41.addWidget(self.groupBox_15)

        self.groupBox_9 = QGroupBox(self.tab_5)
        self.groupBox_9.setObjectName(u"groupBox_9")
        self.verticalLayout_39 = QVBoxLayout(self.groupBox_9)
        self.verticalLayout_39.setObjectName(u"verticalLayout_39")
        self.horizontalLayout_36 = QHBoxLayout()
        self.horizontalLayout_36.setObjectName(u"horizontalLayout_36")
        self.groupBox_10 = QGroupBox(self.groupBox_9)
        self.groupBox_10.setObjectName(u"groupBox_10")
        self.groupBox_10.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_31 = QHBoxLayout(self.groupBox_10)
        self.horizontalLayout_31.setObjectName(u"horizontalLayout_31")
        self.verticalLayout_28 = QVBoxLayout()
        self.verticalLayout_28.setObjectName(u"verticalLayout_28")
        self.label_35 = QLabel(self.groupBox_10)
        self.label_35.setObjectName(u"label_35")
        self.label_35.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_28.addWidget(self.label_35)

        self.label_45 = QLabel(self.groupBox_10)
        self.label_45.setObjectName(u"label_45")
        self.label_45.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_28.addWidget(self.label_45)

        self.label_46 = QLabel(self.groupBox_10)
        self.label_46.setObjectName(u"label_46")
        self.label_46.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_28.addWidget(self.label_46)


        self.horizontalLayout_31.addLayout(self.verticalLayout_28)

        self.verticalLayout_29 = QVBoxLayout()
        self.verticalLayout_29.setObjectName(u"verticalLayout_29")
        self.spinBox_motor0_pin0 = QSpinBox(self.groupBox_10)
        self.spinBox_motor0_pin0.setObjectName(u"spinBox_motor0_pin0")
        self.spinBox_motor0_pin0.setMinimum(-1)
        self.spinBox_motor0_pin0.setValue(-1)

        self.verticalLayout_29.addWidget(self.spinBox_motor0_pin0)

        self.spinBox_motor0_pin1 = QSpinBox(self.groupBox_10)
        self.spinBox_motor0_pin1.setObjectName(u"spinBox_motor0_pin1")
        self.spinBox_motor0_pin1.setMinimum(-1)
        self.spinBox_motor0_pin1.setValue(-1)

        self.verticalLayout_29.addWidget(self.spinBox_motor0_pin1)

        self.spinBox_motor0_PWMPin = QSpinBox(self.groupBox_10)
        self.spinBox_motor0_PWMPin.setObjectName(u"spinBox_motor0_PWMPin")
        self.spinBox_motor0_PWMPin.setMinimum(-1)
        self.spinBox_motor0_PWMPin.setValue(-1)

        self.verticalLayout_29.addWidget(self.spinBox_motor0_PWMPin)


        self.horizontalLayout_31.addLayout(self.verticalLayout_29)


        self.horizontalLayout_36.addWidget(self.groupBox_10)

        self.groupBox_11 = QGroupBox(self.groupBox_9)
        self.groupBox_11.setObjectName(u"groupBox_11")
        self.groupBox_11.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_32 = QHBoxLayout(self.groupBox_11)
        self.horizontalLayout_32.setObjectName(u"horizontalLayout_32")
        self.verticalLayout_30 = QVBoxLayout()
        self.verticalLayout_30.setObjectName(u"verticalLayout_30")
        self.label_47 = QLabel(self.groupBox_11)
        self.label_47.setObjectName(u"label_47")
        self.label_47.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_30.addWidget(self.label_47)

        self.label_48 = QLabel(self.groupBox_11)
        self.label_48.setObjectName(u"label_48")
        self.label_48.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_30.addWidget(self.label_48)

        self.label_49 = QLabel(self.groupBox_11)
        self.label_49.setObjectName(u"label_49")
        self.label_49.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_30.addWidget(self.label_49)


        self.horizontalLayout_32.addLayout(self.verticalLayout_30)

        self.verticalLayout_31 = QVBoxLayout()
        self.verticalLayout_31.setObjectName(u"verticalLayout_31")
        self.spinBox_motor1_pin0 = QSpinBox(self.groupBox_11)
        self.spinBox_motor1_pin0.setObjectName(u"spinBox_motor1_pin0")
        self.spinBox_motor1_pin0.setMinimum(-1)
        self.spinBox_motor1_pin0.setValue(-1)

        self.verticalLayout_31.addWidget(self.spinBox_motor1_pin0)

        self.spinBox_motor1_pin1 = QSpinBox(self.groupBox_11)
        self.spinBox_motor1_pin1.setObjectName(u"spinBox_motor1_pin1")
        self.spinBox_motor1_pin1.setMinimum(-1)
        self.spinBox_motor1_pin1.setValue(-1)

        self.verticalLayout_31.addWidget(self.spinBox_motor1_pin1)

        self.spinBox_motor1_PWMpin = QSpinBox(self.groupBox_11)
        self.spinBox_motor1_PWMpin.setObjectName(u"spinBox_motor1_PWMpin")
        self.spinBox_motor1_PWMpin.setMinimum(-1)
        self.spinBox_motor1_PWMpin.setValue(-1)

        self.verticalLayout_31.addWidget(self.spinBox_motor1_PWMpin)


        self.horizontalLayout_32.addLayout(self.verticalLayout_31)


        self.horizontalLayout_36.addWidget(self.groupBox_11)


        self.verticalLayout_39.addLayout(self.horizontalLayout_36)

        self.horizontalLayout_37 = QHBoxLayout()
        self.horizontalLayout_37.setObjectName(u"horizontalLayout_37")
        self.groupBox_12 = QGroupBox(self.groupBox_9)
        self.groupBox_12.setObjectName(u"groupBox_12")
        self.groupBox_12.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_33 = QHBoxLayout(self.groupBox_12)
        self.horizontalLayout_33.setObjectName(u"horizontalLayout_33")
        self.verticalLayout_32 = QVBoxLayout()
        self.verticalLayout_32.setObjectName(u"verticalLayout_32")
        self.label_50 = QLabel(self.groupBox_12)
        self.label_50.setObjectName(u"label_50")
        self.label_50.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_32.addWidget(self.label_50)

        self.label_51 = QLabel(self.groupBox_12)
        self.label_51.setObjectName(u"label_51")
        self.label_51.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_32.addWidget(self.label_51)


        self.horizontalLayout_33.addLayout(self.verticalLayout_32)

        self.verticalLayout_33 = QVBoxLayout()
        self.verticalLayout_33.setObjectName(u"verticalLayout_33")
        self.spinBox_encoder0_pin0 = QSpinBox(self.groupBox_12)
        self.spinBox_encoder0_pin0.setObjectName(u"spinBox_encoder0_pin0")
        self.spinBox_encoder0_pin0.setMinimum(-1)
        self.spinBox_encoder0_pin0.setValue(-1)

        self.verticalLayout_33.addWidget(self.spinBox_encoder0_pin0)

        self.spinBox_encoder0_pin1 = QSpinBox(self.groupBox_12)
        self.spinBox_encoder0_pin1.setObjectName(u"spinBox_encoder0_pin1")
        self.spinBox_encoder0_pin1.setMinimum(-1)
        self.spinBox_encoder0_pin1.setValue(-1)

        self.verticalLayout_33.addWidget(self.spinBox_encoder0_pin1)


        self.horizontalLayout_33.addLayout(self.verticalLayout_33)


        self.horizontalLayout_37.addWidget(self.groupBox_12)

        self.groupBox_13 = QGroupBox(self.groupBox_9)
        self.groupBox_13.setObjectName(u"groupBox_13")
        self.groupBox_13.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_34 = QHBoxLayout(self.groupBox_13)
        self.horizontalLayout_34.setObjectName(u"horizontalLayout_34")
        self.verticalLayout_34 = QVBoxLayout()
        self.verticalLayout_34.setObjectName(u"verticalLayout_34")
        self.label_52 = QLabel(self.groupBox_13)
        self.label_52.setObjectName(u"label_52")
        self.label_52.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_34.addWidget(self.label_52)

        self.label_53 = QLabel(self.groupBox_13)
        self.label_53.setObjectName(u"label_53")
        self.label_53.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_34.addWidget(self.label_53)


        self.horizontalLayout_34.addLayout(self.verticalLayout_34)

        self.verticalLayout_35 = QVBoxLayout()
        self.verticalLayout_35.setObjectName(u"verticalLayout_35")
        self.spinBox_encoder1_pin0 = QSpinBox(self.groupBox_13)
        self.spinBox_encoder1_pin0.setObjectName(u"spinBox_encoder1_pin0")
        self.spinBox_encoder1_pin0.setMinimum(-1)
        self.spinBox_encoder1_pin0.setValue(-1)

        self.verticalLayout_35.addWidget(self.spinBox_encoder1_pin0)

        self.spinBox_encoder1_pin1 = QSpinBox(self.groupBox_13)
        self.spinBox_encoder1_pin1.setObjectName(u"spinBox_encoder1_pin1")
        self.spinBox_encoder1_pin1.setMinimum(-1)
        self.spinBox_encoder1_pin1.setValue(-1)

        self.verticalLayout_35.addWidget(self.spinBox_encoder1_pin1)


        self.horizontalLayout_34.addLayout(self.verticalLayout_35)


        self.horizontalLayout_37.addWidget(self.groupBox_13)


        self.verticalLayout_39.addLayout(self.horizontalLayout_37)


        self.verticalLayout_41.addWidget(self.groupBox_9)

        self.groupBox_14 = QGroupBox(self.tab_5)
        self.groupBox_14.setObjectName(u"groupBox_14")
        self.groupBox_14.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_51 = QHBoxLayout(self.groupBox_14)
        self.horizontalLayout_51.setObjectName(u"horizontalLayout_51")
        self.horizontalLayout_35 = QHBoxLayout()
        self.horizontalLayout_35.setObjectName(u"horizontalLayout_35")
        self.label_54 = QLabel(self.groupBox_14)
        self.label_54.setObjectName(u"label_54")
        self.label_54.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_35.addWidget(self.label_54)

        self.spinBox_MPU6050_pin0 = QSpinBox(self.groupBox_14)
        self.spinBox_MPU6050_pin0.setObjectName(u"spinBox_MPU6050_pin0")
        self.spinBox_MPU6050_pin0.setMinimum(-1)
        self.spinBox_MPU6050_pin0.setValue(-1)

        self.horizontalLayout_35.addWidget(self.spinBox_MPU6050_pin0)


        self.horizontalLayout_51.addLayout(self.horizontalLayout_35)

        self.horizontalLayout_50 = QHBoxLayout()
        self.horizontalLayout_50.setObjectName(u"horizontalLayout_50")
        self.label_55 = QLabel(self.groupBox_14)
        self.label_55.setObjectName(u"label_55")
        self.label_55.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_50.addWidget(self.label_55)

        self.spinBox_MPU6050_pin1 = QSpinBox(self.groupBox_14)
        self.spinBox_MPU6050_pin1.setObjectName(u"spinBox_MPU6050_pin1")
        self.spinBox_MPU6050_pin1.setMinimum(-1)
        self.spinBox_MPU6050_pin1.setValue(-1)

        self.horizontalLayout_50.addWidget(self.spinBox_MPU6050_pin1)


        self.horizontalLayout_51.addLayout(self.horizontalLayout_50)


        self.verticalLayout_41.addWidget(self.groupBox_14)

        self.verticalSpacer_6 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_41.addItem(self.verticalSpacer_6)

        self.horizontalLayout_42 = QHBoxLayout()
        self.horizontalLayout_42.setObjectName(u"horizontalLayout_42")
        self.pushButton_read_settings = QPushButton(self.tab_5)
        self.pushButton_read_settings.setObjectName(u"pushButton_read_settings")
        sizePolicy7.setHeightForWidth(self.pushButton_read_settings.sizePolicy().hasHeightForWidth())
        self.pushButton_read_settings.setSizePolicy(sizePolicy7)
        self.pushButton_read_settings.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_42.addWidget(self.pushButton_read_settings)

        self.pushButton_write_settings = QPushButton(self.tab_5)
        self.pushButton_write_settings.setObjectName(u"pushButton_write_settings")
        sizePolicy8 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        sizePolicy8.setHorizontalStretch(0)
        sizePolicy8.setVerticalStretch(0)
        sizePolicy8.setHeightForWidth(self.pushButton_write_settings.sizePolicy().hasHeightForWidth())
        self.pushButton_write_settings.setSizePolicy(sizePolicy8)
        self.pushButton_write_settings.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_42.addWidget(self.pushButton_write_settings)

        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_42.addItem(self.horizontalSpacer_5)

        self.pushButton_save_settings = QPushButton(self.tab_5)
        self.pushButton_save_settings.setObjectName(u"pushButton_save_settings")
        sizePolicy7.setHeightForWidth(self.pushButton_save_settings.sizePolicy().hasHeightForWidth())
        self.pushButton_save_settings.setSizePolicy(sizePolicy7)
        self.pushButton_save_settings.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_42.addWidget(self.pushButton_save_settings)


        self.verticalLayout_41.addLayout(self.horizontalLayout_42)

        self.tabWidget_settings.addTab(self.tab_5, "")
        self.tab_8 = QWidget()
        self.tab_8.setObjectName(u"tab_8")
        self.verticalLayout_24 = QVBoxLayout(self.tab_8)
        self.verticalLayout_24.setObjectName(u"verticalLayout_24")
        self.horizontalLayout_27 = QHBoxLayout()
        self.horizontalLayout_27.setObjectName(u"horizontalLayout_27")
        self.groupBox = QGroupBox(self.tab_8)
        self.groupBox.setObjectName(u"groupBox")
        sizePolicy7.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy7)
        self.groupBox.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_25 = QHBoxLayout(self.groupBox)
        self.horizontalLayout_25.setObjectName(u"horizontalLayout_25")
        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.label_29 = QLabel(self.groupBox)
        self.label_29.setObjectName(u"label_29")
        sizePolicy2.setHeightForWidth(self.label_29.sizePolicy().hasHeightForWidth())
        self.label_29.setSizePolicy(sizePolicy2)
        self.label_29.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_13.addWidget(self.label_29)

        self.label_31 = QLabel(self.groupBox)
        self.label_31.setObjectName(u"label_31")
        sizePolicy2.setHeightForWidth(self.label_31.sizePolicy().hasHeightForWidth())
        self.label_31.setSizePolicy(sizePolicy2)
        self.label_31.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_13.addWidget(self.label_31)

        self.label_32 = QLabel(self.groupBox)
        self.label_32.setObjectName(u"label_32")
        sizePolicy2.setHeightForWidth(self.label_32.sizePolicy().hasHeightForWidth())
        self.label_32.setSizePolicy(sizePolicy2)
        self.label_32.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_13.addWidget(self.label_32)


        self.horizontalLayout_25.addLayout(self.verticalLayout_13)

        self.verticalLayout_15 = QVBoxLayout()
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.spinBox_accel_offset_x = QSpinBox(self.groupBox)
        self.spinBox_accel_offset_x.setObjectName(u"spinBox_accel_offset_x")
        self.spinBox_accel_offset_x.setReadOnly(True)
        self.spinBox_accel_offset_x.setMinimum(-99999999)
        self.spinBox_accel_offset_x.setMaximum(99999999)

        self.verticalLayout_15.addWidget(self.spinBox_accel_offset_x)

        self.spinBox_accel_offset_y = QSpinBox(self.groupBox)
        self.spinBox_accel_offset_y.setObjectName(u"spinBox_accel_offset_y")
        self.spinBox_accel_offset_y.setReadOnly(True)
        self.spinBox_accel_offset_y.setMinimum(-99999999)
        self.spinBox_accel_offset_y.setMaximum(99999999)

        self.verticalLayout_15.addWidget(self.spinBox_accel_offset_y)

        self.spinBox_accel_offset_z = QSpinBox(self.groupBox)
        self.spinBox_accel_offset_z.setObjectName(u"spinBox_accel_offset_z")
        self.spinBox_accel_offset_z.setReadOnly(True)
        self.spinBox_accel_offset_z.setMinimum(-99999999)
        self.spinBox_accel_offset_z.setMaximum(99999999)

        self.verticalLayout_15.addWidget(self.spinBox_accel_offset_z)


        self.horizontalLayout_25.addLayout(self.verticalLayout_15)

        self.horizontalLayout_25.setStretch(1, 1)

        self.horizontalLayout_27.addWidget(self.groupBox)

        self.groupBox_4 = QGroupBox(self.tab_8)
        self.groupBox_4.setObjectName(u"groupBox_4")
        sizePolicy7.setHeightForWidth(self.groupBox_4.sizePolicy().hasHeightForWidth())
        self.groupBox_4.setSizePolicy(sizePolicy7)
        self.groupBox_4.setAlignment(Qt.AlignCenter)
        self.horizontalLayout_26 = QHBoxLayout(self.groupBox_4)
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.verticalLayout_17 = QVBoxLayout()
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.label_30 = QLabel(self.groupBox_4)
        self.label_30.setObjectName(u"label_30")
        sizePolicy2.setHeightForWidth(self.label_30.sizePolicy().hasHeightForWidth())
        self.label_30.setSizePolicy(sizePolicy2)
        self.label_30.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_17.addWidget(self.label_30)

        self.label_33 = QLabel(self.groupBox_4)
        self.label_33.setObjectName(u"label_33")
        sizePolicy2.setHeightForWidth(self.label_33.sizePolicy().hasHeightForWidth())
        self.label_33.setSizePolicy(sizePolicy2)
        self.label_33.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_17.addWidget(self.label_33)

        self.label_34 = QLabel(self.groupBox_4)
        self.label_34.setObjectName(u"label_34")
        sizePolicy2.setHeightForWidth(self.label_34.sizePolicy().hasHeightForWidth())
        self.label_34.setSizePolicy(sizePolicy2)
        self.label_34.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.verticalLayout_17.addWidget(self.label_34)


        self.horizontalLayout_26.addLayout(self.verticalLayout_17)

        self.verticalLayout_18 = QVBoxLayout()
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.spinBox_gyro_offset_x = QSpinBox(self.groupBox_4)
        self.spinBox_gyro_offset_x.setObjectName(u"spinBox_gyro_offset_x")
        self.spinBox_gyro_offset_x.setReadOnly(True)
        self.spinBox_gyro_offset_x.setMinimum(-99999999)
        self.spinBox_gyro_offset_x.setMaximum(99999999)

        self.verticalLayout_18.addWidget(self.spinBox_gyro_offset_x)

        self.spinBox_gyro_offset_y = QSpinBox(self.groupBox_4)
        self.spinBox_gyro_offset_y.setObjectName(u"spinBox_gyro_offset_y")
        self.spinBox_gyro_offset_y.setReadOnly(True)
        self.spinBox_gyro_offset_y.setMinimum(-99999999)
        self.spinBox_gyro_offset_y.setMaximum(99999999)
        self.spinBox_gyro_offset_y.setValue(0)

        self.verticalLayout_18.addWidget(self.spinBox_gyro_offset_y)

        self.spinBox_gyro_offset_z = QSpinBox(self.groupBox_4)
        self.spinBox_gyro_offset_z.setObjectName(u"spinBox_gyro_offset_z")
        self.spinBox_gyro_offset_z.setReadOnly(True)
        self.spinBox_gyro_offset_z.setMinimum(-99999999)
        self.spinBox_gyro_offset_z.setMaximum(99999999)

        self.verticalLayout_18.addWidget(self.spinBox_gyro_offset_z)


        self.horizontalLayout_26.addLayout(self.verticalLayout_18)

        self.horizontalLayout_26.setStretch(1, 1)

        self.horizontalLayout_27.addWidget(self.groupBox_4)


        self.verticalLayout_24.addLayout(self.horizontalLayout_27)

        self.verticalSpacer_7 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_24.addItem(self.verticalSpacer_7)

        self.horizontalLayout_47 = QHBoxLayout()
        self.horizontalLayout_47.setObjectName(u"horizontalLayout_47")
        self.pushButton_setOffset = QPushButton(self.tab_8)
        self.pushButton_setOffset.setObjectName(u"pushButton_setOffset")
        sizePolicy1.setHeightForWidth(self.pushButton_setOffset.sizePolicy().hasHeightForWidth())
        self.pushButton_setOffset.setSizePolicy(sizePolicy1)

        self.horizontalLayout_47.addWidget(self.pushButton_setOffset)

        self.pushButton_calibrate_imu = QPushButton(self.tab_8)
        self.pushButton_calibrate_imu.setObjectName(u"pushButton_calibrate_imu")
        sizePolicy7.setHeightForWidth(self.pushButton_calibrate_imu.sizePolicy().hasHeightForWidth())
        self.pushButton_calibrate_imu.setSizePolicy(sizePolicy7)
        self.pushButton_calibrate_imu.setMinimumSize(QSize(0, 50))

        self.horizontalLayout_47.addWidget(self.pushButton_calibrate_imu)


        self.verticalLayout_24.addLayout(self.horizontalLayout_47)

        self.tabWidget_settings.addTab(self.tab_8, "")

        self.verticalLayout_16.addWidget(self.tabWidget_settings)

        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_16.addItem(self.verticalSpacer_5)

        self.dockWidget.setWidget(self.dockWidgetContents)
        MainWindow.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dockWidget)

        self.retranslateUi(MainWindow)

        self.tabWidget_motion.setCurrentIndex(0)
        self.tabWidget_settings.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.pushButton_reset.setText(QCoreApplication.translate("MainWindow", u"\u590d\u4f4d", None))
        self.pushButton_show_dockWidget.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e", None))
        self.pushButton_move_left.setText(QCoreApplication.translate("MainWindow", u"\u5411\u5de6", None))
        self.pushButton_move_front.setText(QCoreApplication.translate("MainWindow", u"\u5411\u524d", None))
        self.pushButton_move_back.setText(QCoreApplication.translate("MainWindow", u"\u5411\u540e", None))
        self.pushButton_move_right.setText(QCoreApplication.translate("MainWindow", u"\u5411\u53f3", None))
        self.tabWidget_motion.setTabText(self.tabWidget_motion.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Jog\u63a7\u5236", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Line_X:", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Line_Y:", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Line_Z:", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"m/s", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Angle_X:", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"rad/s", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Angle_Y:", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"rad/s", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Angle_Z:", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"rad/s", None))
        self.pushButton_set_speed.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u52a8", None))
        self.tabWidget_motion.setTabText(self.tabWidget_motion.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u63a7\u5236", None))
        self.radioButton_cartesian_coordinates.setText(QCoreApplication.translate("MainWindow", u"\u7b1b\u5361\u5c14\u5750\u6807\u7cfb", None))
        self.radioButton_polar_coordinates.setText(QCoreApplication.translate("MainWindow", u"\u6781\u5750\u6807\u7cfb", None))
        self.label_25.setText(QCoreApplication.translate("MainWindow", u"X:", None))
        self.label_26.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.label_27.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
        self.label_28.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.label_21.setText(QCoreApplication.translate("MainWindow", u"\u4f4d\u79fb:", None))
        self.label_23.setText(QCoreApplication.translate("MainWindow", u"m", None))
        self.label_22.setText(QCoreApplication.translate("MainWindow", u"\u89d2\u5ea6:", None))
        self.label_24.setText(QCoreApplication.translate("MainWindow", u"rad", None))
        self.pushButton_path_move.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u52a8", None))
        self.pushButton_path_stop.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62", None))
        self.tabWidget_motion.setTabText(self.tabWidget_motion.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"\u4f4d\u59ff\u63a7\u5236", None))
        self.pushButton_stop_slow.setText(QCoreApplication.translate("MainWindow", u"\u7f13\u505c", None))
        self.pushButton_stop_now.setText(QCoreApplication.translate("MainWindow", u"\u7d27\u505c", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"\u7ed8\u56fe\u8bbe\u7f6e", None))
        self.checkBox_dynami_follow.setText(QCoreApplication.translate("MainWindow", u"\u8ddf\u968f", None))
        self.checkBox_dynamic_refresh.setText(QCoreApplication.translate("MainWindow", u"\u5237\u65b0", None))
        self.checkBox_graph_0.setText(QCoreApplication.translate("MainWindow", u"\u66f2\u7ebf1", None))
        self.checkBox_graph_1.setText(QCoreApplication.translate("MainWindow", u"\u66f2\u7ebf2", None))
        self.checkBox_graph_2.setText(QCoreApplication.translate("MainWindow", u"\u66f2\u7ebf3", None))
        self.checkBox_graph_3.setText(QCoreApplication.translate("MainWindow", u"\u66f2\u7ebf4", None))
        self.pushButton_clear_plot.setText(QCoreApplication.translate("MainWindow", u"\u6e05\u7a7a", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("MainWindow", u"IMU\u4fe1\u606f", None))
        self.label_imu_status.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.groupBox_6.setTitle(QCoreApplication.translate("MainWindow", u"odom\u4fe1\u606f", None))
        self.label_odom_msg.setText(QCoreApplication.translate("MainWindow", u"\u72b6\u6001", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"\u6307\u4ee4\u4fe1\u606f", None))
        self.pushButton_clear_cmd_info.setText(QCoreApplication.translate("MainWindow", u"\u6e05\u9664", None))
        self.radioButton_dynamic_balance.setText(QCoreApplication.translate("MainWindow", u"\u52a8\u6001\u5e73\u8861", None))
        self.radioButton_static_balance.setText(QCoreApplication.translate("MainWindow", u"\u9759\u6001\u5e73\u8861", None))
        self.radioButton_no_balance.setText(QCoreApplication.translate("MainWindow", u"\u53d6\u6d88\u5e73\u8861", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("MainWindow", u"\u4f4d\u7f6e\u73af", None))
        self.label_36.setText(QCoreApplication.translate("MainWindow", u"\u6bd4\u4f8b:", None))
        self.label_37.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206:", None))
        self.label_38.setText(QCoreApplication.translate("MainWindow", u"\u5fae\u5206:", None))
        self.label_39.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206\u4e0a\u9650:", None))
        self.groupBox_8.setTitle(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u73af", None))
        self.label_44.setText(QCoreApplication.translate("MainWindow", u"\u6bd4\u4f8b:", None))
        self.label_56.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206:", None))
        self.label_57.setText(QCoreApplication.translate("MainWindow", u"\u5fae\u5206:", None))
        self.label_58.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206\u4e0a\u9650:", None))
        self.groupBox_16.setTitle(QCoreApplication.translate("MainWindow", u"\u4f4d\u59ff\u73af-\u5411\u524d", None))
        self.label_59.setText(QCoreApplication.translate("MainWindow", u"\u6bd4\u4f8b:", None))
        self.label_60.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206:", None))
        self.label_61.setText(QCoreApplication.translate("MainWindow", u"\u5fae\u5206:", None))
        self.label_62.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206\u4e0a\u9650:", None))
        self.groupBox_19.setTitle(QCoreApplication.translate("MainWindow", u"\u4f4d\u59ff\u73af-\u5411\u540e", None))
        self.label_63.setText(QCoreApplication.translate("MainWindow", u"\u6bd4\u4f8b:", None))
        self.label_64.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206:", None))
        self.label_65.setText(QCoreApplication.translate("MainWindow", u"\u5fae\u5206:", None))
        self.label_66.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206\u4e0a\u9650:", None))
        self.groupBox_17.setTitle(QCoreApplication.translate("MainWindow", u"\u7535\u673a\u73af", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"\u6bd4\u4f8b:", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206:", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"\u5fae\u5206:", None))
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"\u79ef\u5206\u4e0a\u9650:", None))
        self.groupBox_18.setTitle(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u89c4\u5212", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"\u65f6\u95f4\u5468\u671f\u9891\u7387(ms):", None))
        self.label_20.setText(QCoreApplication.translate("MainWindow", u"\u6700\u5927\u901f\u5ea6(m/s):", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"\u6700\u5927\u52a0\u901f\u5ea6(m/s^2):", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"\u52a0\u52a0\u901f\u5ea6(m/s^3):", None))
        self.pushButton_read_params.setText(QCoreApplication.translate("MainWindow", u"\u8bfb\u53d6", None))
        self.pushButton_write_params.setText(QCoreApplication.translate("MainWindow", u"\u5199\u5165", None))
        self.pushButton_save_params.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58", None))
        self.tabWidget_settings.setTabText(self.tabWidget_settings.indexOf(self.tab_9), QCoreApplication.translate("MainWindow", u"\u53c2\u6570\u8bbe\u7f6e", None))
        self.groupBox_15.setTitle(QCoreApplication.translate("MainWindow", u"\u6a21\u578b\u914d\u7f6e", None))
        self.label_40.setText(QCoreApplication.translate("MainWindow", u"\u8f6e\u5b50\u76f4\u5f84(m):", None))
        self.label_41.setText(QCoreApplication.translate("MainWindow", u"\u4e24\u8f6e\u8ddd\u79bb(m):", None))
        self.label_42.setText(QCoreApplication.translate("MainWindow", u"\u5355\u5708\u8109\u51b2\u6570:", None))
        self.label_43.setText(QCoreApplication.translate("MainWindow", u"\u6bcf\u5206\u949f\u5708\u6570:", None))
        self.groupBox_9.setTitle(QCoreApplication.translate("MainWindow", u"\u5f15\u811a\u914d\u7f6e", None))
        self.groupBox_10.setTitle(QCoreApplication.translate("MainWindow", u"\u7535\u673a1", None))
        self.label_35.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a1:", None))
        self.label_45.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a2:", None))
        self.label_46.setText(QCoreApplication.translate("MainWindow", u"PWM:", None))
        self.groupBox_11.setTitle(QCoreApplication.translate("MainWindow", u"\u7535\u673a2", None))
        self.label_47.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a1:", None))
        self.label_48.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a2:", None))
        self.label_49.setText(QCoreApplication.translate("MainWindow", u"PWM:", None))
        self.groupBox_12.setTitle(QCoreApplication.translate("MainWindow", u"\u7f16\u7801\u56681", None))
        self.label_50.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a1:", None))
        self.label_51.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a2:", None))
        self.groupBox_13.setTitle(QCoreApplication.translate("MainWindow", u"\u7f16\u7801\u56682", None))
        self.label_52.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a1:", None))
        self.label_53.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a2:", None))
        self.groupBox_14.setTitle(QCoreApplication.translate("MainWindow", u"MPU6050", None))
        self.label_54.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a1:", None))
        self.label_55.setText(QCoreApplication.translate("MainWindow", u"\u5f15\u811a2:", None))
        self.pushButton_read_settings.setText(QCoreApplication.translate("MainWindow", u"\u8bfb\u53d6", None))
        self.pushButton_write_settings.setText(QCoreApplication.translate("MainWindow", u"\u5199\u5165", None))
        self.pushButton_save_settings.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58", None))
        self.tabWidget_settings.setTabText(self.tabWidget_settings.indexOf(self.tab_5), QCoreApplication.translate("MainWindow", u"\u4e0b\u4f4d\u673a\u914d\u7f6e", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"\u52a0\u901f\u5ea6\u504f\u79fb\u91cf", None))
        self.label_29.setText(QCoreApplication.translate("MainWindow", u"X\u8f74:", None))
        self.label_31.setText(QCoreApplication.translate("MainWindow", u"Y\u8f74:", None))
        self.label_32.setText(QCoreApplication.translate("MainWindow", u"Z\u8f74:", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"\u9640\u87ba\u4eea\u504f\u79fb\u91cf", None))
        self.label_30.setText(QCoreApplication.translate("MainWindow", u"X\u8f74:", None))
        self.label_33.setText(QCoreApplication.translate("MainWindow", u"Y\u8f74:", None))
        self.label_34.setText(QCoreApplication.translate("MainWindow", u"Z\u8f74:", None))
        self.pushButton_setOffset.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u504f\u5dee", None))
        self.pushButton_calibrate_imu.setText(QCoreApplication.translate("MainWindow", u"\u6821\u51c6IMU", None))
        self.tabWidget_settings.setTabText(self.tabWidget_settings.indexOf(self.tab_8), QCoreApplication.translate("MainWindow", u"IMU\u6821\u51c6", None))
    # retranslateUi

