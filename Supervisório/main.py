
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QSlider, QPushButton, QHBoxLayout, QVBoxLayout, QGroupBox, QGridLayout, QComboBox, QWidget
from numpy.core.records import array
from serial.tools.list_ports import comports
from threading import Thread
import serial
import pyqtgraph as pg
import sys

# sginal bytes 
READY=   '#' # 0x0f 
STOP=    '$' # 0x24
HEADER=  '<' # 0x3c /
TAIL=    '>' # 0x3e 
RECIVE=  '%' # 0x13
SEND=    '@' # 0x40

# RECIVE/SEND bytes
SET_DISTANCE=  '!' # 0x21 declara a distância "objetiva" 
REQUEST_DATA=  '_' # 0x5f  byte de requisição # 0x5f byte de confirmação
SPLITER=       ';' # 0x3b byte de separação de valores

DELAY = 0.005
DEFAUL_INTERVAL = 10
DEFAUL_REQUIRED_DISTANCE = 8000.0

class USB:

    def __init__(self, port:str='COM5', frequence=9600, timeout=1, *args, **kwargs):
        super().__init__()
        self.Serial = serial.Serial(port, frequence, timeout=timeout)

    @staticmethod
    def removeKeys(string: str, remove=[
        READY,
        HEADER,
        TAIL,
        RECIVE,
        SEND,
        SET_DISTANCE,
        REQUEST_DATA
    ]):
        for item in remove:
            string = string.replace(item, '')
        return string

    @classmethod
    def getPortList():
        return serial.tools.list_ports.comports()
    
    def send(self, data):
        dataStream = bytes(SEND + HEADER + data + TAIL, 'ascii')
        print(f'sended[bytes: {self.Serial.write(dataStream)} bytes]= {SEND + HEADER + data + TAIL}')
               
        dataRecived = self.Serial.read_until(bytes(RECIVE + HEADER, 'ascii')).decode('ascii')
        dataRecived += self.Serial.read_until(TAIL).decode('ascii')       
        print(f'response[bytes: {len(dataRecived)} bytes]= {dataRecived}') 
        
        return self.removeKeys(dataRecived)
    
    def stop(self):
        dataStream = bytes(SEND + HEADER + STOP + TAIL, 'ascii')
        print(f'STOP sginal! [{self.Serial.write(dataStream)} bytes]')
               
        dataRecived = self.Serial.read_until(bytes(RECIVE + HEADER, 'ascii')).decode('ascii')
        dataRecived += self.Serial.read_until(TAIL).decode('ascii')
        print(f'response[bytes: {len(dataRecived)} bytes]= {dataRecived}') 
        
        return self.removeKeys(dataRecived)

    def start(self) -> bool:
        dataStream = bytes(SEND + HEADER + READY + TAIL, 'ascii')
        print(f'Send READY sginal [{self.Serial.write(dataStream)} bytes]...', end='\t')

        self.Serial.read_until(bytes(RECIVE, 'ascii'))
        dataRecived = self.Serial.read_until(bytes(HEADER, 'ascii')).decode('ascii')
        dataRecived += self.Serial.read_until(bytes(TAIL, 'ascii')).decode('ascii')
        if READY in dataRecived:
            print('Sucess!')
        else:
            print('Fail!')
            self.Serial.flushInput()
            self.Serial.flushOutput()
        
        return READY in dataRecived
    
    def read(self):
        
        self.Serial.read_until(bytes(RECIVE, 'ascii'))
        dataRecived = self.Serial.read_until(bytes(HEADER, 'ascii')).decode('ascii')
        dataRecived += self.Serial.read_until(bytes(TAIL, 'ascii')).decode('ascii')       
        print(f'recived[{len(dataRecived)} bytes]= {dataRecived}') 
        
        return self.removeKeys(dataRecived)

    def flush(self):
        self.Serial.flushInput()
        self.Serial.flushOutput()

        
class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.setMinimumSize(1240, 820)
        self.setMaximumSize(1240, 820)
        
        self.usb:USB = None
        self.is_connected:bool = False
        self.required_distance:float = DEFAUL_REQUIRED_DISTANCE
        
        self.main_container = QGroupBox("")
        
        self.chartsAreaMount()
        self.controlAreaMount()
        
        main_cont = QVBoxLayout()
        main_cont.addStretch(1)
        main_cont.addWidget(self.charts_container)
        main_cont.addWidget(self.controls_container)

        self.main_container.setLayout(main_cont)
        self.setCentralWidget(self.main_container)
        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(DELAY)
        self.timer.timeout.connect(self.update_plot_data)
        

        self.show()
    
    def stop(self):
        self.usb.stop()
        self.timer.stop()
        self.usb.flush()
        
    
    def start(self):
        for _ in range(5):
            if self.usb.start():
                self.usb.flush()
                self.timer.start()
                break

    def chartsAreaMount(self):
    
        styles = {"color": "#f0f0f5", "font-size": "18px"}
        self.sensort_chart:pg.PlotWidget = pg.PlotWidget()
        self.motors_powerChart:pg.PlotWidget = pg.PlotWidget()
        
        self.motors_powerChart.setYRange( -225, 225)
        self.sensort_chart.setYRange(2, 160)
        self.sensort_chart.setLabel("left", "Distância (cm)", **styles)
        self.motors_powerChart.setLabel("left", "PWM", **styles)
        
        self.sensort_chart.setLabel("bottom", "Tempo (s)", **styles)
        self.motors_powerChart.setLabel("bottom", "Tempo (s)", **styles)
        
        self.sensort_chart.addLegend()
        self.motors_powerChart.addLegend()
        
        self.tempo = [] 
        self.distance_data = [] 
        self.pwmLida = [] 
        self.distancia_desejada = []

        self.data_line =  self.sensort_chart.plot(self.tempo, self.distance_data, name="Sensor" ,pen=pg.mkPen(color=(255, 0, 0)))
        self.objetive_line =  self.sensort_chart.plot(self.tempo, self.distancia_desejada, name="Objetivo")
        self.pwm_line = self.motors_powerChart.plot(self.tempo, self.pwmLida, name="PWM") 
        
        
        container = QHBoxLayout()
        container.addStretch(2)
        container.addWidget(self.sensort_chart)
        container.addWidget(self.motors_powerChart)
        self.charts_container = QGroupBox("Charts")
        self.charts_container.setLayout(container)

    def controlAreaMount(self):

        def createField(legend: str, elements: list(QWidget)):
            grupe = QGroupBox(legend)
            layout= QVBoxLayout()
            for element in elements:
                element.setParent(grupe)
                layout.addWidget(element)
            grupe.setLayout(layout)
            return grupe
        
        def update_time_interval():
            self.interval_slider.parent.setTitle(f'Intervalo de analize ({self.interval_slider.value() / 100} s)')
            maxlen = (self.interval_slider.value() / 100) * DELAY
            if len(self.tempo) > maxlen:
                self.tempo = self.tempo[maxlen:-1]
                self.tempo = self.pwmLida[maxlen:-1]
                self.tempo = self.distancia_desejada[maxlen:-1]
                self.tempo = self.distance_data[maxlen:-1]

                self.pwm_line.setData(self.tempo, self.pwmLida)
                self.objetive_line.setData(self.tempo, self.distancia_desejada)
                self.data_line.setData(self.tempo, self.distance_data)  
                

        gridLayout = QGridLayout()
        gridLayout.setColumnStretch(1, 1)
        gridLayout.setColumnMinimumWidth(0, 150)
        
        # instaceado elementos do secundarios
        self.combobox = QComboBox()
        self.startButton = QPushButton("Start")
        self.connectButton = QPushButton("Connect")
        self.refreshButton = QPushButton("Refresh")
        self.distance_slider = QSlider(QtCore.Qt.Horizontal)
        self.interval_slider = QSlider(QtCore.Qt.Horizontal)
        self.controls_container = QGroupBox("Controles")
        
        # definindo os tamanhos minimos dos botões e comboBox
        self.combobox.setMinimumSize(120, 36)
        self.startButton.setMinimumSize(120, 36)
        self.connectButton.setMinimumSize(120, 36)
        self.refreshButton.setMinimumSize(120, 36)
        self.controls_container.setMinimumHeight(300)
        
        # definindo os valores dos sliders (valores maximos, minimos e iniciais)
        self.distance_slider.setMinimum(2000)
        self.interval_slider.setMaximum(50000)
        self.distance_slider.setMaximum(160000)
        self.interval_slider.setMinimum(1000)
        self.interval_slider.setValue(3000)
        self.distance_slider.setValue(self.required_distance)
        
        # definindo as funções de reação dos elementos
        self.startButton.clicked.connect(self.start_stop)
        self.connectButton.clicked.connect(self.connect)
        self.refreshButton.clicked.connect(self.refresh_default)
        self.interval_slider.valueChanged.connect(update_time_interval)
        self.distance_slider.valueChanged.connect(lambda: self.distance_slider.parent.setTitle(f'Distancia ({self.required_distance} cm)'))
        self.combobox.addItems(map(lambda x: x.name, comports()))
        
        # adicionando os campos e elementos ao container de controle
        gridLayout.addWidget(createField('Start/Stop',[self.startButton]), 1, 0)
        gridLayout.addWidget(createField("Conexão", [self.combobox, self.refreshButton, self.connectButton]), 0, 0)
        gridLayout.addWidget(createField("Distancia (0 cm)", [self.distance_slider]), 0, 1)
        gridLayout.addWidget(createField("Intervalo de analize (10.0 s)", [self.interval_slider]), 1, 1)
        self.controls_container.setLayout(gridLayout)
    
    def connect(self):
        self.is_connected = False
        if self.usb is not None:
            self.usb.Serial.close()
            self.usb = None
        
        port = self.combobox.currentText()
        self.usb = USB(port)
        print('Conectado!')
        self.is_connected = True

    def start_stop(self):
        self.startButton.setText("Start" if self.is_pause else "Stop")
        
        if self.is_pause:
            self.start()
        else:
            self.stop()
            

    def refresh_default(self):
        self.combobox.clear()
        self.combobox.addItems(map(lambda x: x.name, comports()))
        
        self.required_distance = DEFAUL_REQUIRED_DISTANCE
        self.distance_slider.setValue(DEFAUL_REQUIRED_DISTANCE)
        self.interval_slider.setValue(DEFAUL_INTERVAL)
        
        self.distance_data = []
        self.pwmLida = []
        self.distancia_desejada = []
        self.tempo = []
        
        self.pwm_line.setData(self.tempo, self.pwmLida)
        self.objetive_line.setData(self.tempo, self.distancia_desejada)
        self.data_line.setData(self.tempo, self.distance_data)
    
    def update_plot_data(self):
        if(self.is_connected and self.is_pause == False):
            
            value= self.usb.send(REQUEST_DATA)
            if(' ' in value  or len(value) <= 2):
                return
            elif len(self.tempo) >= (self.interval_slider.value() / DELAY * 100):
                self.distance_data = self.distance_data[1:]  
                self.distancia_desejada = self.distancia_desejada[1:]  
                self.tempo = self.tempo[1:]  
                self.tempo.append(self.tempo[-1] + DELAY)
            else:
                self.tempo.append(len(self.tempo) * DELAY)
                
                
            self.pwmLida.append(float(value.split(';')[1]))  
            self.distance_data.append(float(value.split(';')[0]))  
            self.distancia_desejada.append(float(self.required_distance))
            
            self.pwm_line.setData(self.tempo, self.pwmLida)
            self.objetive_line.setData(self.tempo, self.distancia_desejada)
            self.data_line.setData(self.tempo, self.distance_data)

        

app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())




