
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QSlider, QPushButton, QHBoxLayout, QGroupBox, QGridLayout

import pyqtgraph as pg
import sys
from random import randint


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setMaximumSize(800, 570)
        # self.gridMount()
        self.view = QGroupBox("")
        self.slider = QSlider(QtCore.Qt.Vertical)
        self.slider.setMinimum(2)
        self.slider.setMaximum(160)
        self.slider.setMaximum
        # self.slider.valueChanged.connect(lambda : self.limite_line.setData(self.tempo, [self.slider.value() for _ in range(len(self.tempo))]))
        self.chartMount()
        H = QHBoxLayout()
        H.addStretch(1)
        H.addWidget(self.chart)
        H.addWidget(self.slider)
        # H.addWidget(self.grid)
        self.view.setLayout(H)
        self.setCentralWidget(self.view)
        
        self.show()
    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen)
    def chartMount(self):
        styles = {"color": "#f0f0f5", "font-size": "18px"}
        self.chart:pg.PlotWidget = pg.PlotWidget()
        self.chart.setYRange( 2, 160) #, padding=0)
        self.chart.setLabel("left", "Distância (cm)", **styles)
        self.chart.setLabel("bottom", "Tempo (s)", **styles)
        self.chart.addLegend()
        self.tempo = [] 
        self.distanciaLida = [] 
        self.distanciaDesejada = []
        self.data_line =  self.chart.plot(self.tempo, self.distanciaLida, name="Sensor" ,pen=pg.mkPen(color=(255, 0, 0)))
        self.objetive_line =  self.chart.plot(self.tempo, self.distanciaDesejada, name="Objetivo")
        self.timer = QtCore.QTimer()
        self.timer.setInterval(150)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def gridMount(self):
        self.grid = QGroupBox("Grid")
        layout = QGridLayout()
        layout.setColumnStretch(1, 4)
        layout.setColumnStretch(2, 4)
        layout.addWidget(QPushButton('1'),0,0)
        layout.addWidget(QPushButton('2'),0,1)
        layout.addWidget(QPushButton('3'),0,2)
        layout.addWidget(QPushButton('4'),1,0)
        layout.addWidget(QPushButton('5'),1,1)
        layout.addWidget(QPushButton('6'),1,2)
        layout.addWidget(QPushButton('7'),2,0)
        layout.addWidget(QPushButton('8'),2,1)
        layout.addWidget(QPushButton('9'),2,2)
        
        self.grid.setLayout(layout)
    
    def update_plot_data(self):
        if len(self.tempo) >= 200:
            self.distanciaLida = self.distanciaLida[1:]  # Remove the first 
            self.distanciaDesejada = self.distanciaDesejada[1:]  # Remove the first 
            self.tempo = self.tempo[1:]  # Remove the first y element.
            self.tempo.append(self.tempo[-1] + 0.15) # Add a new value 1 higher than the last.
            # dif = (self.slider.value() - self.distanciaLida[-1])
            # self.distanciaLida.append( randint( self.slider.value() - dif, self.slider.value() + dif))  # Add a new random value.
        else:
            self.tempo.append(len(self.tempo) * 0.15) # Add a new value 1 higher than the last.
        
        self.distanciaLida.append( randint( self.slider.value() - 3, self.slider.value() + 3))  # Add a new random value.
        self.distanciaDesejada.append(self.slider.value())

        self.objetive_line.setData(self.tempo, self.distanciaDesejada)
        self.data_line.setData(self.tempo, self.distanciaLida)


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())


