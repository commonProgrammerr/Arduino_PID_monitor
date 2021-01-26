# try:
#     from PyQt5 import QtWidgets, QtCore
#     from PyQt5.QtWidgets import QSlider, QPushButton, QHBoxLayout, QVBoxLayout, QGroupBox, QGridLayout, QComboBox, QWidget
#     from serial.tools.list_ports import comports
#     import serial
#     import pyqtgraph as pg
# except ModuleNotFoundError:
#     import pip
#     if hasattr(pip, 'main'):
#         pip.main(['install', 'pyqt5', 'pyserial', 'pyqtgraph'])
#     else:
#         pip._internal.main(['install', 'pyqt5', 'pyserial',   'pyqtgraph'])
    
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QSlider, QPushButton, QHBoxLayout, QVBoxLayout, QGroupBox, QGridLayout, QComboBox, QWidget
from serial.tools.list_ports import comports
import serial
import pyqtgraph as pg
    
import random
from threading import Thread
import sys

from serial.tools.list_ports_common import ListPortInfo

# sginal bytes 
HEADER=  '<' # 0x3c /
TAIL=    '>' # 0x3e 
RECIVE=  '%' # 0x13
SEND=    '@' # 0x40

# RECIVE/SEND bytes
SET_DISTANCE=  '!' # 0x21 declara a distância 'objetiva' 
REQUEST_DATA=  '#' # 0x5f  byte de requisição # 0x5f byte de confirmação
SPLITER=       ';' # 0x3b byte de separação de valores

# constantes
DELAY = 0.035 # intervalo de atualização do gráfico (en segundos)
# os valores estão multiplicados por 100, devido a um dos componentes só aceitar valores inteiros
DEFAUL_INTERVAL = 3000 # intervalo padrão de analize (tamanho máximo do eixo X [30s]) 
DEFAUL_REQUIRED_DISTANCE = 800 # distância padrão que o robo tentará ajustar (8.0 cm)

class USB():

    def __init__(self, port:str, bound=9600, timeout=0.3, *args, **kwargs):
        super().__init__()
        self.__serial__ = serial.Serial(port, bound, timeout=timeout, *args, **kwargs)

    @classmethod
    def getPortList(cls) -> ListPortInfo:
        return comports()
    
    def send(self, data):
        dataStream = bytes(SEND + HEADER + data + TAIL, 'ascii')
        print(f'sended[bytes: {self.__serial__.write(dataStream)} bytes]= {SEND + HEADER + data + TAIL}')
                      
        return self.read()
    
    
    def read(self):
        
        self.__serial__.read_until(bytes(RECIVE, 'ascii'))
        dataRecived = self.__serial__.read_until(bytes(HEADER, 'ascii')).decode('ascii')
        dataRecived += self.__serial__.read_until(bytes(TAIL, 'ascii')).decode('ascii')       
        print(f'recived[{len(dataRecived)} bytes]= {dataRecived}') 

        for item in [ HEADER, TAIL, RECIVE, SEND, SET_DISTANCE, REQUEST_DATA ]:
            dataRecived = dataRecived.replace(item, '')

        return dataRecived

    def flush(self):
        self.__serial__.flushInput()
        self.__serial__.flushOutput()

        
class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.setMinimumSize(1240, 820)
        self.setMaximumSize(1240, 820)
        
        self.usb:USB = None
        self.is_connected:bool = False   
        self.last_require_value = 0.0
        self.main_container = QGroupBox('')
        
        self.chartsAreaMount()
        self.controlAreaMount()
        
        main_cont = QVBoxLayout()
        main_cont.addStretch(1)
        main_cont.addWidget(self.charts_container)
        main_cont.addWidget(self.controls_container)

        self.main_container.setLayout(main_cont)
        self.setCentralWidget(self.main_container)
        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(int(DELAY * 1000))
        self.timer.timeout.connect(self.update_data)
        
        self.show()

    def chartsAreaMount(self):
        
        self.sensor_chart:pg.PlotWidget = pg.PlotWidget()
        self.power_chart:pg.PlotWidget = pg.PlotWidget()
        

        _STYLES = {'color': '#f0f0f5', 'font-size': '18px'}
        
        
        self.power_chart.setLabel('left', 'PWM', **_STYLES)
        self.sensor_chart.setLabel('left', 'Distância (cm)', **_STYLES)

        # Defini a legenda do eixo X (tempo) em ambos os gráficos
        self.sensor_chart.setLabel('bottom', 'Tempo (s)', **_STYLES)
        self.power_chart.setLabel('bottom', 'Tempo (s)', **_STYLES)
        
        self.power_chart.setYRange( -200, 200)
        self.sensor_chart.setYRange(2, 160)

        self.sensor_chart.addLegend()
        self.power_chart.addLegend()
        
        self.tempo = [] 
        self.distance_data = [] 
        self.pwmLida = [] 
        self.distancia_desejada = []

        self.data_line =  self.sensor_chart.plot(self.tempo, self.distance_data, name='Sensor' ,pen=pg.mkPen(color=(255, 0, 0)))
        self.objetive_line =  self.sensor_chart.plot(self.tempo, self.distancia_desejada, name='Objetivo')
        self.pwm_line = self.power_chart.plot(self.tempo, self.pwmLida, name='PWM') 
        
        
        container = QHBoxLayout()
        container.addStretch(2)
        container.addWidget(self.sensor_chart)
        container.addWidget(self.power_chart)
        self.charts_container = QGroupBox('Charts')
        self.charts_container.setLayout(container)

    def controlAreaMount(self):

        def createField(legend: str, elements: list):
            grupe = QGroupBox(legend)
            layout= QVBoxLayout()
            for element in elements:
                element.setParent(grupe)
                layout.addWidget(element)
            grupe.setLayout(layout)
            return grupe

        # instacia e define o layoute em grid
        gridLayout = QGridLayout()
        gridLayout.setColumnStretch(1, 1)
        gridLayout.setColumnMinimumWidth(0, 150)
        
        # instaciado elementos do secundarios
        self.controls_container = QGroupBox('Controles') # container onde irão ficar os controles
        self.combobox = QComboBox() # comboBox para as portas disponiveis
        self.startButton = QPushButton('Start') # botão de start para iniciar/parar a leitura
        self.connectButton = QPushButton('Connect') # botão de connect para criar conexão com o arduino
        self.refreshButton = QPushButton('Refresh') # botão de refresh para atualizar lista de portas
        
        # sliders de controle para: distancia desejada e intervalo de analize, respectivamente
        self.distance_slider = QSlider(QtCore.Qt.Horizontal) 
        self.interval_slider = QSlider(QtCore.Qt.Horizontal)
        
        # definindo os tamanhos minimos dos botões e comboBox
        self.combobox.setMinimumSize(120, 36)
        self.startButton.setMinimumSize(120, 36)
        self.connectButton.setMinimumSize(120, 36)
        self.refreshButton.setMinimumSize(120, 36)
        self.controls_container.setMinimumHeight(300)
        
        # definindo os valores dos sliders (valores maximos, minimos e iniciais);
        # ambos os sliders estão com valores maximos e minimos, multiplicados por 100 para compensar o valor ser de tipo inteiro
        self.distance_slider.setMinimum(200)
        self.interval_slider.setMinimum(1000) 
        self.interval_slider.setMaximum(50000) 
        self.distance_slider.setMaximum(16000)
        self.interval_slider.setValue(DEFAUL_INTERVAL)
        self.distance_slider.setValue(DEFAUL_REQUIRED_DISTANCE)
        
        # definindo as funções de reação dos elementos
        self.startButton.clicked.connect(self.start_stop)
        self.connectButton.clicked.connect(self.connect_disconnect)
        self.refreshButton.clicked.connect(self.refresh_port_list)
        self.interval_slider.valueChanged.connect(lambda: self.interval_slider.parent().setTitle(f'Intervalo de analize ({self.interval_slider.value() / 100} s)'))
        self.distance_slider.valueChanged.connect(lambda: self.distance_slider.parent().setTitle(f'Distancia ({self.distance_slider.value() / 100} cm)'))
        self.refresh_port_list()
        
        # adicionando os campos e elementos ao container de controle
        gridLayout.addWidget(createField('Start/Stop',[self.startButton]), 1, 0)
        gridLayout.addWidget(createField('Conexão', [self.combobox, self.refreshButton, self.connectButton]), 0, 0)
        gridLayout.addWidget(createField(f'Distancia ({self.distance_slider.value() / 100} cm)', [self.distance_slider]), 0, 1)
        gridLayout.addWidget(createField(f'Intervalo de analize ({self.interval_slider.value() / 100} s)', [self.interval_slider]), 1, 1)
        self.controls_container.setLayout(gridLayout)
    
    def connect(self):
        
        port = self.combobox.currentText()
        try:
            if self.usb is not None:
                self.usb.Serial.close()
            self.usb = USB(port)
            self.usb.flush()
            print('Conectado!')
        except:
            self.usb = None
            print('Falha ao conectar!')
            return
        
        self.is_connected = True
        self.connectButton.setText('Disconnect')
    
    def disconnect(self):
        try:
            if self.usb is not None:
                self.usb.__serial__.close()
                self.usb = None
                self.clear()
                print('Desonectado!')
        except:
            print('Falha ao desconectar!')
            return
        
        self.is_connected = False
        self.connectButton.setText('Connect')

    def connect_disconnect(self):
        self.disconnect() if self.is_connected else self.connect()
    
    def stop(self):
        if self.timer.isActive():
            self.startButton.setText('Start')
            self.timer.stop()
            print('Analize parada!')
        
    def start(self):
        if self.timer.isActive() is False:
            self.startButton.setText('Stop')
            self.timer.start()
            print('Analize iniciada!')
        

    def start_stop(self):
        if self.timer.isActive():
            self.stop()
        else:
            self.start()
            
    def refresh_port_list(self):
        self.combobox.clear()
        self.combobox.addItems(map(lambda x: x.name, USB.getPortList()))
    
    def clear(self):
        if self.timer.isActive():
            self.stop()
        
        self.distance_slider.setValue(DEFAUL_REQUIRED_DISTANCE)
        self.interval_slider.setValue(DEFAUL_INTERVAL)
        
        self.tempo = []
        self.pwmLida = []
        self.distancia_desejada = []
        self.distance_data = []

        self.objetive_line.setData(self.tempo, self.distancia_desejada)
        self.pwm_line.setData(self.tempo, self.pwmLida)
        self.data_line.setData(self.tempo, self.distance_data)
        
    
    def update_data(self):
        print('Atualizando dados... ')
        if self.last_require_value != self.distance_slider.value() / 100:
            newValue = self.distance_slider.value() / 100
            recived_data = self.usb.send(str(newValue)+REQUEST_DATA + SET_DISTANCE)
            if(' ' in recived_data  or len(recived_data) <= 2):
                return
            self.last_require_value = float(recived_data.split(';')[2])
        else:    
            recived_data= self.usb.send(REQUEST_DATA)
        
        if(' ' in recived_data  or len(recived_data) <= 2):
            return

        # tempo máximo de analize / intervalo de leitua  == Nº máximo de leituras
        _MAX = int(self.interval_slider.value() / (DELAY * 100)) 
        
        # caso o tempo do gráfico seja maior que o limite definido,redimensiona dos dados armazenados descartando os primeiros indices
        if len(self.tempo) > _MAX: 

            # array = array - indices antes de: diferença entre o tamanho da array e o tamanho máximo calculado
            self.tempo = self.tempo[abs(len(self.tempo) - _MAX):]               
            self.pwmLida = self.pwmLida[abs(len(self.pwmLida) - _MAX):]                
            self.distancia_desejada = self.distancia_desejada[abs(len(self.distancia_desejada) - _MAX):]                
            self.distance_data = self.distance_data[abs(len(self.distance_data) - _MAX):]                

        # atualiza a lista do tempo (responsavel pelo eixo X de ambos os gráficos)
        self.tempo.append(DELAY if len(self.tempo) == 0 else self.tempo[-1] + DELAY)
        
        # atualiza as listas de valores do gráfico 
        self.distancia_desejada.append(self.last_require_value)
        self.pwmLida.append(float(recived_data.split(';')[1]))  
        self.distance_data.append(float(recived_data.split(';')[0]))  
        
        # insere os dados atualizados no gráfico
        self.objetive_line.setData(self.tempo, self.distancia_desejada)
        self.pwm_line.setData(self.tempo, self.pwmLida)
        self.data_line.setData(self.tempo, self.distance_data)
        print('Atualização completa!')
        
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
