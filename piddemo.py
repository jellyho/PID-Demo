import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
import numpy as np

########################################
#             PID Pendulum             #
#             by jellyho               #
########################################

class PIDDemo(QMainWindow):
    def __init__(self):
        super(PIDDemo, self).__init__()
        self.setupUi(self)
        canvas = QtGui.QPixmap(400, 400)
        canvas.fill(Qt.white)
        self.plabel.setPixmap(canvas)
        self.dt = 20 / 1000
        self.init_PID()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(self.dt * 1000))

    def setupUi(self, MainWindow):
        self.P = 0
        self.I = 0
        self.D = 0
        self.state = np.array([-np.pi/2, 0])
        self.Target = np.pi
        self.max_speed = 10.0
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(720, 480)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.PSlider = QtWidgets.QSlider(self.centralwidget)
        self.PSlider.setGeometry(QtCore.QRect(429, 70, 221, 22))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.PSlider.sizePolicy().hasHeightForWidth())
        self.PSlider.setSizePolicy(sizePolicy)
        self.PSlider.setMinimum(0)
        self.PSlider.setMaximum(1000)
        self.PSlider.setOrientation(QtCore.Qt.Horizontal)
        self.PSlider.setObjectName("PSlider")
        self.PSlider.valueChanged.connect(self.pslider_cb)
        self.ISlider = QtWidgets.QSlider(self.centralwidget)
        self.ISlider.setGeometry(QtCore.QRect(429, 160, 221, 22))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ISlider.sizePolicy().hasHeightForWidth())
        self.ISlider.setSizePolicy(sizePolicy)
        self.ISlider.setMinimum(0)
        self.ISlider.setMaximum(1000)
        self.ISlider.setOrientation(QtCore.Qt.Horizontal)
        self.ISlider.setObjectName("ISlider")
        self.ISlider.valueChanged.connect(self.islider_cb)
        self.DSlider = QtWidgets.QSlider(self.centralwidget)
        self.DSlider.setGeometry(QtCore.QRect(429, 240, 221, 22))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.DSlider.sizePolicy().hasHeightForWidth())
        self.DSlider.setSizePolicy(sizePolicy)
        self.DSlider.setMinimum(0)
        self.DSlider.setMaximum(1000)
        self.DSlider.setOrientation(QtCore.Qt.Horizontal)
        self.DSlider.setObjectName("DSlider")
        self.DSlider.valueChanged.connect(self.dslider_cb)
        self.TargetDial = QtWidgets.QDial(self.centralwidget)
        self.TargetDial.setGeometry(QtCore.QRect(470, 290, 141, 121))
        self.TargetDial.setMinimum(-180)
        self.TargetDial.setMaximum(180)
        self.TargetDial.setProperty("value", 180)
        self.TargetDial.setSliderPosition(180)
        self.TargetDial.setWrapping(True)
        self.TargetDial.setNotchTarget(0.0)
        self.TargetDial.setObjectName("TargetDial")
        self.Targetlabel = QtWidgets.QLabel(self.centralwidget)
        self.Targetlabel.setGeometry(QtCore.QRect(430, 410, 151, 31))
        self.Targetlabel.setObjectName("Target")
        self.TargetDial.valueChanged.connect(self.spin_cb)
        self.Ilabel = QtWidgets.QLabel(self.centralwidget)
        self.Ilabel.setGeometry(QtCore.QRect(430, 120, 161, 20))
        self.Ilabel.setObjectName("label_2")
        self.Plabel = QtWidgets.QLabel(self.centralwidget)
        self.Plabel.setGeometry(QtCore.QRect(430, 30, 161, 21))
        self.Plabel.setObjectName("label_3")
        self.Dlabel = QtWidgets.QLabel(self.centralwidget)
        self.Dlabel.setGeometry(QtCore.QRect(430, 210, 161, 20))
        self.Dlabel.setObjectName("label_4")
        self.plabel = QtWidgets.QLabel(self.centralwidget)
        self.plabel.setGeometry(QtCore.QRect(10, 30, 400, 400))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.plabel.sizePolicy().hasHeightForWidth())
        self.plabel.setSizePolicy(sizePolicy)
        self.plabel.setText("")
        self.plabel.setObjectName("plabel")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PID Pendulum by jellyho"))
        self.Targetlabel.setText(_translate("MainWindow", "Target : 180 degrees"))
        self.Ilabel.setText(_translate("MainWindow", "I : 0"))
        self.Plabel.setText(_translate("MainWindow", "P : 0"))
        self.Dlabel.setText(_translate("MainWindow", "D : 0"))

    def draw(self):
        painter = QtGui.QPainter(self.plabel.pixmap())
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        # background
        self.plabel.pixmap().fill(Qt.white)

        # target line render
        length = 200
        x = np.sin(self.Target)
        y = np.cos(self.Target)

        pen = QPen(QColor(0, 0, 255), 2)
        painter.setPen(pen)

        painter.drawLine(200, 200, int(200 + length * x), int(200 - length * y))
        
        #========================================================================

        #pendulum render
        painter.setPen(QPen(Qt.red, 2, Qt.SolidLine))
        painter.setBrush(QColor(255, 182, 193))  # 분홍색으로 채우기
        pwidth = 10
        plength = 120
        polygon_points = [QPoint(int(200 + pwidth * np.sin(self.state[0] + np.pi / 2)), int(200 - pwidth * np.cos(self.state[0] + np.pi / 2))),
                          QPoint(int(200 + pwidth * np.sin(self.state[0] - np.pi / 2)), int(200 - pwidth * np.cos(self.state[0] - np.pi / 2))),
                          QPoint(int(200 + plength * np.sin(self.state[0]) + pwidth * np.sin(self.state[0] - np.pi / 2)), int(200 - plength * np.cos(self.state[0]) - pwidth * np.cos(self.state[0] - np.pi / 2))),
                          QPoint(int(200 + (plength+30) * np.sin(self.state[0])), int(200 - (plength+30) * np.cos(self.state[0]))),
                          QPoint(int(200 + plength * np.sin(self.state[0]) + pwidth * np.sin(self.state[0] + np.pi / 2)), int(200 - plength * np.cos(self.state[0]) - pwidth * np.cos(self.state[0] + np.pi / 2)))]
        painter.drawPolygon(polygon_points)
        #=====================================================================

        painter.end()
        self.plabel.setPixmap(self.plabel.pixmap())

    def pslider_cb(self, value):
        self.P = value / 100.0
        self.Plabel.setText(f'P : {self.P:.2f}')
    
    def islider_cb(self, value):
        self.I = value / 100.0
        self.Ilabel.setText(f'I : {self.I:.3f}')

    def dslider_cb(self, value):
        self.D = value / 100.0
        self.Dlabel.setText(f'D : {self.D:.3f}')
    
    def spin_cb(self, value):
        self.Target = value / 360 * np.pi * 2
        self.Targetlabel.setText(f'Target : {value} degrees')
        self.sum_error = 0

    def update(self):
        error = self.get_error(self.state[0], self.Target)
        input = self.pid(self.P, self.I, self.D, error)
        self.pendulum(input)
        self.draw()

    def angle_normalize(self, x):
        return ((x + np.pi) % (2 * np.pi)) - np.pi

    def pendulum(self, input):
        th, thdot = self.state  # th := theta

        g = 1
        m = 1
        l = 1
        b = 1
        dt = self.dt

        u = input
        self.last_u = u  # for rendering
        costs = self.angle_normalize(th) ** 2 + 0.1 * thdot**2 + 0.001 * (u**2)

        newthdot = thdot + (3 * g / (2 * l) * np.sin(th) + 3.0 / (m * l**2) * u - b / (m * l**2) * thdot) * dt
        newthdot = np.clip(newthdot, -self.max_speed, self.max_speed)
        newth = th + newthdot * dt

        self.state = np.array([newth, newthdot])

    def get_error(self, curr, target):
        curr = curr % (np.pi * 2)
        target = target % (np.pi * 2)
        
        target -= curr
        if (target < 0):
            target += np.pi * 2
        if (target > np.pi):
            target =  target - np.pi * 2        
        return target

    def init_PID(self):
        self.prev_error = 0
        self.sum_error = 0

    def pid(self, P, I, D, error):
        derror = (error - self.prev_error)
        if I != 0:
            self.sum_error += error * self.dt
        else:
            self.sum_error = 0
        self.prev_error = error
        
        return P * error + D * derror / self.dt + I * self.sum_error
    


app = QApplication([])
window = PIDDemo()
window.show()
app.exec_()
