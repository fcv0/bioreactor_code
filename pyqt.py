import sys, time, numpy, os, serial, threading, struct, pandas, math, ctypes, glob
import matplotlib.pyplot as plt
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QDialog
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from serial.tools import list_ports

class _Main:
    ## Private class
    CmotorSpeed, FmotorSpeed, WmotorSpeed = 230, 0, 0
    CmotorDir, FmotorDir, WmotorDir = 0, 0, 0
    OD1, OD2, OD3 = [], [], [] # probs will be a multi dimensional array housing both the raw values and calculated OD values.
    Raw1, Raw2, Raw3 = [], [], []
    initRaw1, initRaw2, initRaw3 = 0, 0, 0
    if len(Raw1) > 0:
        initRaw1, initRaw2, initRaw3 = Raw1[0], Raw2[0], Raw3[0]
    OTemp, ATemp, TempTarg = 37.24, 32.81, 37.2
    StirSpeed, ODTarg = 40, 1.9
    message = numpy.zeros((20, 1), dtype=numpy.uint8) # This change is dynamical from config file 'Number of variables'
    Device, START, DeviceStatus = None, False, 1
    MMMotor, MMMotorDelay = 0, 0
    DeviceType, START = None, False
    Comm_freq, Capture_freq, Save_freq = 10, 4, 100 # In Hz, in seconds and in seconds.
    RunTime = None
    graph_vals = -80
    CommsTurn = True
    HardwareTime = []
    def __init__(self) -> None:
        pass

class Tools:
    not_FOUND = True
    retry = 3
    def __init__(self) -> None:
        self.start_time = time.time()
        self.select_cwd(), self.read_cfg(), self.create_logs
        while self.not_FOUND:
            os.system('cls')
            try: 
                _Main.Device = self.find_port()
            except: print('Could not find micro-controller. ')
            if _Main.Device != None:
                self.not_FOUND = False
                break
            else: time.sleep(self.retry)

    @staticmethod
    def find_port():
        # Automatically find arduino com port, however break if mutliple are connected.
        ports = list_ports.comports(include_links=False)
        for port in ports:
            if 'arduino' in port.description.lower(): # We could use vendor ID also but this should suffice.
                print(f'Found port: {port.device}')
                PORT = serial.Serial(port.device)
                break
            else:   continue
        print('Device information: ', PORT)
        if PORT.isOpen():
            PORT.close()
        PORT = serial.Serial(port=port.device, baudrate=9600, timeout=.1)
        PORT.flushInput(), PORT.flushOutput()
        print(f'Connecting to {PORT.name}')
        return PORT
    
    @staticmethod
    def reset_data() -> None:
        _Main.OD1, _Main.OD2, _Main.OD3 = [], [], []
        _Main.Raw1, _Main.Raw2, _Main.Raw3 = [], [], []
        _Main.initRaw1, _Main.initRaw2, _Main.initRaw3 = 0, 0, 0
        _Main.HardwareTime = []
    
    @staticmethod
    def select_cwd() -> None:
        os.chdir(os.path.abspath(os.path.dirname(__file__)))

    @staticmethod
    def CALC_OD() -> None:
        '''Reduction in computation can be done via doing this in bulk at the end of a run quickly, raw transmission should be just as readable, if not more accurate.'''
        ## Need a fully working diode, if diode is somewhat broken just use the main diode.
        od1 = round(-9.2*math.log10(_Main.Raw1[0]/_Main.Raw1[-1]), 4)
        od2 = round(-9.2*math.log10(_Main.Raw2[0]/_Main.Raw2[-1]), 4)
        od3 = round(-9.2*math.log10(_Main.Raw3[0]/_Main.Raw3[-1]), 4)
        _Main.OD1.append(od1), _Main.OD2.append(od2), _Main.OD3.append(od3)
    
    @staticmethod
    def point_logs() -> None:
        if os.path.isdir('logs/') == False:
            os.mkdir('logs/')
        os.chdir(f'{os.getcwd()}/logs/')
        Total_runs = glob.glob('Run_*') # Sometimes works, sometimes doesnt :) 
        print(Total_runs)
        if len(Total_runs) <= 0:
            os.mkdir('Run_1/')
        else:
            #next_run = int(Total_runs[-1][4:]) + 1
            next_run = len(Total_runs) + 1
            print(next_run)
            os.mkdir(f'Run_{next_run}')
        
    @property
    def create_logs(self, ) -> None:
        self.select_cwd(), self.point_logs(), self.PointCurrRun()
        WRT = 'w'
        if os.path.isfile('main_log.csv') == False:
            WRT = 'x'
        with open('main_log.csv', mode=WRT, newline='') as log:
            log.close()
    
    @staticmethod
    def create_cfg() -> None:
        os.chdir(os.path.abspath(os.path.dirname(__file__)))
        if os.path.isfile('config.ini') == False:
            with open('config.ini', 'x', newline='') as cfg:
                cfg.write('# Update frequency is in Hz e.g 10 = 0.1 seconds. (10 is recommended).\n')
                cfg.write(f'Update frequency = {_Main.Comm_freq}\n')
                cfg.write('# Data frequency is the time between a data point in seconds. (4 is recommended).\n')
                cfg.write(f'Data frequency = {_Main.Capture_freq}\n')
                cfg.write("# Run time is the length of the experiment wanted to be run as the 'start' button is pressed in seconds.\n")
                cfg.write(f'Run Time = 86400\n')
                cfg.write('# Live updating length is the number of recent data points displayed. (< 100 is recommended).\n')
                cfg.write(f'Live Updating length = {_Main.graph_vals}\n')
                cfg.write('# Save frequency is the time between log saves, in seconds.\n')
                cfg.write(f'Save frequency = {_Main.Save_freq}\n')
                cfg.write('# Device mode suggests if the device is turbido or morbidostat.\n')
                cfg.write(f'Device mode = morbidostat\n')
            crtcfg = ctypes.windll.user32.MessageBoxA
            crtcfg(0, ctypes.c_char_p(b"Config file did not exist.\nConfig file has been created.\nFound within the same directory as the main file.\nRe-run the script to proceed with experiments."), ctypes.c_char_p(b"Config creation"), 0x0 | 0x40)
            exit()

    @staticmethod
    def save_data(*arrays) -> None:
        '''
        Ensure the array inputted is in the correct format. I.E in a numpy array and transposed horizontally.
        '''
        df = pandas.DataFrame(arrays)
        df = df.T
        df.to_csv('main_log.csv', header=False, index=False)
    
    @staticmethod
    def PointCurrRun() -> None:
        ### broken
        ### need to fix
        RUN = glob.glob('*')
        cp = os.getcwd()
        cp = cp.split('pyqt.py')
        try:
            os.chdir(f'{cp[-1]}\\{RUN[-1]}')
        except:
            os.chdir(f'{cp[-1]}\\logs\\{RUN[-1]}')
    
    @staticmethod
    def read_cfg() -> None:
        print('Reading config file. ')
        with open('config.ini', mode='r', newline='') as cfg:
            for line in cfg:
                line = line.strip('\n')
                line = line.strip('\r')
                line = line.strip('\t')
                if '#' in line:
                    continue
                if len(line) <= 4:
                    continue
                if 'Update frequency' in line:
                    line = line.split('=')
                    _Main.Comm_freq = float(line[-1])
                if 'Run Time' in line:
                    line = line.split('=')
                    _Main.RunTime = int(line[-1])
                if 'Data frequency' in line:
                    line = line.split('=')
                    _Main.Capture_freq = int(line[-1])
                if 'Live Updating length' in line:
                    line = line.split('=')
                    _Main.graph_vals = -int(line[-1])
                if 'Save frequency' in line:
                    line = line.split('=')
                    _Main.Save_freq = int(line[-1])
                if 'Device mode' in line:
                    line = line.split('=')
                    _Main.DeviceType = str(line[-1])
                # Maybe I should add default/recommend values?? Then give an option to instantly set the message to default or config values in the gui for the message.
                # Need to add number of variables here also.

    @staticmethod
    def float2bytes(data) -> bytes:
        return list(struct.pack('!f', data))

    @staticmethod
    def data_handler() -> None:
        def bytes2float(data) -> float:
            return struct.unpack('f', data[::-1])[0]
        def bytes2int(data) -> int:
            # this isn't really needed to be honest.
            return int.from_bytes(data, 'little')
        StartTime = time.time()
        c, cL, sF = 1, 1, 1
        try:
            while _Main.START:
                currentTime = time.time() - StartTime
                if _Main.CommsTurn == False:
                    if _Main.Device.in_waiting >= 24:
                        os.system('cls')
                        read = _Main.Device.read(24)
                        print(f'Ambient temperature: {bytes2float(read[12:16])} | Object temperature: {bytes2float(read[16:20])} | System time: {(bytes2float(read[20:24])/1000)}')
                        print('Time | ', currentTime)
                        # Turn the LED on before the measurement, standard is (_Main.Capture_freq//2) -> rounded to int() 
                        if (currentTime // 1) == cL: # probs wont work in the future I imagine...
                            _Main.message[18] = 1
                            cL+=1
                        # Captures a reading of the relevant measurements every _Main.Capture_freq seconds -> in cfg known as Data frequency.
                        if (currentTime // _Main.Capture_freq) == c:
                            _Main.Raw1.append(round(bytes2float(read[0:4]), 2)), _Main.Raw2.append(round(bytes2float(read[4:8]), 2)), _Main.Raw3.append(round(bytes2float(read[8:12]), 2)), _Main.HardwareTime.append(round((bytes2float(read[20:24])/1000),3))
                            c+=1
                            Tools.CALC_OD()
                            _Main.message[18] = 0
                        # Save every _Main.Save_freq time point, standard is 100 seconds.
                        if (currentTime // _Main.Save_freq) == sF:
                            Tools.select_cwd(), Tools.PointCurrRun()
                            Tools.save_data(_Main.HardwareTime, _Main.Raw1, _Main.Raw2, _Main.Raw3)
                            print(f'Data last saved at {sF*_Main.Save_freq} in seconds. ')
                            sF+=1
                        # Final save and kills all electronics when run time is complete.
                        if (currentTime // _Main.RunTime) == 1:
                            Tools.save_data(_Main.HardwareTime, _Main.Raw1, _Main.Raw2, _Main.Raw3)
                            _Main.Device.write(bytes(numpy.zeros((20, 1), dtype=numpy.uint8)))
                            exit()
                        _Main.CommsTurn = True
                # Send data at a frequenct of 1/_Main.Comm_freq, e.g standard is 0.1 seconds.
                if _Main.CommsTurn:
                    time.sleep((1/_Main.Comm_freq))
                    _Main.Device.write(bytes(_Main.message))
                    _Main.CommsTurn = False
            _Main.Device.write(bytes(numpy.zeros((20, 1), dtype=numpy.uint8)))
        except KeyboardInterrupt:
            msgba = ctypes.windll.user32.MessageBoxA
            msgba(0, ctypes.c_char_p(b"Software crashed"), ctypes.c_char_p(b"Crash"), 0x0 | 0x30)
            Tools.save_data(_Main.HardwareTime, _Main.Raw1, _Main.Raw2, _Main.Raw3)

class UpdateVariableApp(QWidget, Tools):
    def __init__(self, ) -> None:
        super().__init__()
        self.initUI()
        # Stop fucking working on start up :(

    def initUI(self):
        # Simple system for adding tabs and placing tabs where-ever you like, 
        def create_labels() -> None:
            '''Plain text labels, can have updating values -> check out UpdateLabels(); variables in _Main is required for this typically.'''
            self.cmotorlabel = QLabel('Cycle Motor', self)
            self.freshMotorLabel = QLabel('Fresh Motor', self)
            self.wasteMotorLabel = QLabel('Waste Motor', self)
            self.ControlOptions = QLabel('Control options: ', self)
            self.GRAPHLabel = QLabel('Graphs: ', self)
            self.ODInf = QLabel('OD Info: ', self)
            self.CRaw = QLabel('Current raw values: ', self)
            self.TempInf = QLabel(f'Temp Info: ', self)
            self.AeraLabel = QLabel('Stir Speed', self)
            # OD updating displays
            self.OD1 = QLabel(f'OD1: {0}', self)
            self.OD2 = QLabel(f'OD2: {0}', self)
            self.OD3 = QLabel(f'OD3: {0}', self)
            self.ODTarg = QLabel(f'OD Target: ', self)
            # Raw value updating displays
            self.PDR = QLabel(f'Initial raw values: ', self)
            self.initRaw1 = QLabel(f'Photodiode 1: {0}', self)
            self.initRaw2 = QLabel(f'Photodiode 2: {0}', self)
            self.initRaw3 = QLabel(f'Photodiode 3: {0}', self)
            ## Current raw values
            self.Raw1 = QLabel(f'Photodiode 1: {0}', self)
            self.Raw2 = QLabel(f'Photodiode 2: {0}', self)
            self.Raw3 = QLabel(f'Photodiode 3: {0}', self)
            ## Temperature info
            self.OTemp = QLabel(f'Object Temp: {0}', self)
            self.ATemp = QLabel(f'Ambient Temp: {0}', self)
            self.TargTemp = QLabel(f'Target Temp: ', self)
            ## Time info
            self.disp_time = QLabel(f'Run Time: {0}', self)
            ## Mother machine specific stuff
            self.MMMLabel = QLabel('MM Motor', self)
            self.MMDLabel = QLabel('MM Delay', self)

        def create_button() -> None:
            '''Creates buttons ... each button needs -> self.xname.clicked.connect(self.xfunction) in main UI function.'''
            self.show_gbs = QPushButton("Show OD Graphs", self)
            self.show_gbr = QPushButton('Show Raw Graphs', self)
            #self.EditMsg = QPushButton('Upload Custom Message', self)
            self.START = QPushButton('Start', self)
            self.RunGraph = QPushButton('Run Graph', self)
        
        def create_entry_fields() -> None:
            '''Can enter a set of values into this field, updated values in this entry field can be tracked -> self.xname.textChanged.connect(self.xfunction) in main UI function.
                Existing values can also be set (function below this) set_existing_values(). 
            '''
            self.CycleMotor = QLineEdit(self)
            self.FreshMotor = QLineEdit(self)
            self.WasteMotor = QLineEdit(self)
            self.AerationMotor = QLineEdit(self)
            self.MMMotor = QLineEdit(self)
            self.MMDelay = QLineEdit(self)
            self.setODTarg = QLineEdit(self)
            self.setTempTarg = QLineEdit(self)
        
        def set_existing_values() -> None:
            '''Sets pre-determined standard for initial run for a given entry field.'''
            self.CycleMotor.setText(f'{_Main.CmotorSpeed}')
            self.FreshMotor.setText(f'{_Main.FmotorSpeed}')
            self.WasteMotor.setText(f'{_Main.WmotorSpeed}')
            self.AerationMotor.setText(f'{_Main.StirSpeed}')
            self.MMMotor.setText(f'{_Main.StirSpeed}')
            self.MMDelay.setText(f'{_Main.MMMotorDelay}')
            self.setODTarg.setText(f'{_Main.ODTarg}')
            self.setTempTarg.setText(f'{_Main.TempTarg}')

        def set_layout() -> None:
            '''You can change the layout here or use the standard layouts provided by PyQt5, the standards kinda suck though...'''
            self.ControlOptions.setGeometry(10, 0, 100, 10)
            self.cmotorlabel.setGeometry(10, 10, 75, 20)
            self.CycleMotor.setGeometry(10, 30, 75, 20)
            # Buttons
            self.show_gbs.setGeometry(5, 270, 100, 20)
            self.show_gbr.setGeometry(110, 270, 100, 20)
            #self.EditMsg.setGeometry(5, 220, 130, 20)
            self.START.setGeometry(220, 270, 70, 20)
            # OD info
            self.ODInf.setGeometry(230, 130, 100, 10)
            self.OD1.setGeometry(230, 145, 100, 15)
            self.OD2.setGeometry(230, 160, 100, 15)
            self.OD3.setGeometry(230, 175, 100, 15)
            self.ODTarg.setGeometry(230, 190, 100, 15)
            self.setODTarg.setGeometry(230, 205, 75, 15)
            # Turbidostat motors. 
            self.freshMotorLabel.setGeometry(10, 50, 100, 20)
            self.FreshMotor.setGeometry(10, 70, 75, 20)
            self.wasteMotorLabel.setGeometry(10, 90, 75, 20)
            self.WasteMotor.setGeometry(10, 110, 75, 20)
            # Aeration motor.
            self.AeraLabel.setGeometry(10, 130, 75, 20)
            self.AerationMotor.setGeometry(10, 150, 75, 20)
            # Mother machine mother specifics.
            self.MMMLabel.setGeometry(10, 170, 75, 20)
            self.MMMotor.setGeometry(10, 190, 75, 20)
            self.MMDLabel.setGeometry(10, 210, 75, 20)
            self.MMDelay.setGeometry(10, 230, 75, 20)

            #Photodiode init and current.
            self.PDR.setGeometry(100, 0, 200, 10)
            self.CRaw.setGeometry(100, 60, 200, 10)
            self.Raw1.setGeometry(100, 75, 200, 10)
            self.Raw2.setGeometry(100, 90, 200, 10)
            self.Raw3.setGeometry(100, 105, 200, 10)
            self.initRaw1.setGeometry(100, 15, 200, 10)
            self.initRaw2.setGeometry(100, 30, 200, 10)
            self.initRaw3.setGeometry(100, 45, 200, 10)
            # Temperature info
            self.TempInf.setGeometry(100, 130, 100, 15)
            self.OTemp.setGeometry(100, 145, 100, 15)
            self.ATemp.setGeometry(100, 160, 100, 15)
            self.TargTemp.setGeometry(100, 175, 100, 15)
            self.setTempTarg.setGeometry(100, 190, 75, 15)

            # Extra
            self.GRAPHLabel.setGeometry(10, 245, 100, 30)
            self.disp_time.setGeometry(225, 250, 100, 20)
            self.RunGraph.setGeometry(120, 250, 75, 20)
            
        
        # Create my own custom layout and text fills.
        create_labels(), create_entry_fields(), create_button(), set_existing_values()
        
        # Do something when something new is typed into a given field.
        self.CycleMotor.textChanged.connect(self.UpdateCycleMotor)
        self.FreshMotor.textChanged.connect(self.UpdateFreshMotor)
        self.WasteMotor.textChanged.connect(self.UpdateWasteMotor)
        self.AerationMotor.textChanged.connect(self.UpdateAerationMotor)
        self.setODTarg.textChanged.connect(self.UpdateODTarg)
        self.setTempTarg.textChanged.connect(self.UpdateTempTarg)

        # Refresh the UI every 1000 ms 
        self.test = QTimer(self)
        self.test.timeout.connect(self.UpdateLabels)
        self.test.start(1000) # Need to fix somethings tbh
        set_layout()

        # Do X when X button is clicked.
        self.show_gbs.clicked.connect(self.showGraphOD)
        #self.EditMsg.clicked.connect(self.update_msg)
        self.show_gbr.clicked.connect(self.showGraphRaw)
        self.START.clicked.connect(self.start_run)
        self.RunGraph.clicked.connect(self.quick_plot)

        self.setWindowTitle("Bioreactor Control. ")
        self.setGeometry(100, 100, 300, 300)
        self.set_default()
    
    def quick_plot(self, ) -> None:
        '''A none updating plot of total current points found when pressing. '''
        plt.plot(_Main.HardwareTime, _Main.Raw1, 'r', alpha=0.5), plt.plot(_Main.HardwareTime, _Main.Raw2, 'b', alpha=0.5), plt.plot(_Main.HardwareTime, _Main.Raw3, 'g', alpha=0.5)
        plt.title('Raw vs Time'), plt.xlabel('Time (s)'), plt.ylabel('Raw transmission (nW)')
        plt.pause(10)
    
    def UpdateLabels(self, ) -> None:
        '''Periodic updating of the text labels. self.test corrosponds to the update frequency (self.start(x milliseconds)).'''
        if _Main.START:
            try:
        # Add temperature and run time until completion, similar save points.
                self.OD1.setText(f'OD1: {_Main.OD1[-1]}')
                self.OD2.setText(f'OD2: {_Main.OD2[-1]}')
                self.OD3.setText(f'OD3: {_Main.OD3[-1]}')
                self.initRaw1.setText(f'Photodiode 1: {_Main.Raw1[0]}')
                self.initRaw2.setText(f'Photodiode 2: {_Main.Raw2[0]}')
                self.initRaw3.setText(f'Photodiode 3: {_Main.Raw3[0]}')
                self.Raw1.setText(f'Photodiode 1: {_Main.Raw1[-1]}')
                self.Raw2.setText(f'Photodiode 2: {_Main.Raw2[-1]}')
                self.Raw3.setText(f'Photodiode 3: {_Main.Raw3[-1]}')
                self.OTemp.setText(f'Object Temp: {_Main.OTemp}')
                self.ATemp.setText(f'Ambient Temp: {_Main.ATemp}')
                self.disp_time.setText(f'Run Time (s): {0}')
            except:
                pass
    
    def start_run(self, ) -> None:
        if _Main.START == False:
            _Main.START = True
            s1 = threading.Thread(target=Tools.data_handler)
            s1.start()
            self.START.setStyleSheet('background-color: red')
            self.START.setText('Stop')
        else:
            Tools.reset_data()
            _Main.START = False
            self.START.setText('Start')
            self.START.setStyleSheet('')
            #self.stop_run()
    
    def check_values(self, Updated_Variable):
        '''This should be stable for all motors. '''
        Dir = 0
        if len(Updated_Variable) == 0:
            Updated_Variable = 0
        elif '-' in Updated_Variable and len(Updated_Variable) == 1:
            Updated_Variable = '-0'
        else:
            if '-' in Updated_Variable:
                Updated_Variable = Updated_Variable[1:]
                Dir = 1
                if int(Updated_Variable) is None:
                    Updated_Variable = 0
            elif int(Updated_Variable) >= 255:
                Updated_Variable = 255
        return Dir, Updated_Variable

    ### Change the byte locations and variable names etc to however suited.

    def UpdateCycleMotor(self, ) -> None:
        try:
            CMotorText = self.CycleMotor.text() 
            _Main.CmotorDir, _Main.CmotorSpeed =  self.check_values(CMotorText)
            _Main.message[1], _Main.message[2] = _Main.CmotorDir, _Main.CmotorSpeed
        except:
            _Main.message[1], _Main.message[2] = _Main.CmotorDir, _Main.CmotorSpeed 
    
    def UpdateFreshMotor(self, ) -> None:
        try:
            FMotorText = self.FreshMotor.text()
            _Main.FmotorDir, _Main.FmotorSpeed = self.check_values(FMotorText)
            _Main.message[3], _Main.message[4] = _Main.FmotorDir, _Main.FmotorSpeed
        except:
            _Main.message[3], _Main.message[4] = _Main.FmotorDir, _Main.FmotorSpeed

    
    def UpdateWasteMotor(self, ) -> None:
        try:
            WMotorText = self.WasteMotor.text()
            _Main.WmotorDir, _Main.WmotorSpeed = self.check_values(WMotorText)
            _Main.message[5], _Main.message[6] = _Main.WmotorDir, _Main.WmotorSpeed
        except:
            _Main.message[5], _Main.message[6] = _Main.WmotorDir, _Main.WmotorSpeed
    
    def UpdateAerationMotor(self, ) -> None:
        try:
            AMotorText = self.AerationMotor.text()
            _, _Main.StirSpeed = self.check_values(AMotorText)
            _Main.message[7] = _Main.StirSpeed
        except:
            _Main.message[7] = _Main.StirSpeed

    def UpdateODTarg(self, ) -> None:
        try:
            _Main.ODTarg = float(self.setODTarg.text())
            ODTarg = Tools.float2bytes(_Main.ODTarg)
            _Main.message[10] = ODTarg[0]
            _Main.message[11] = ODTarg[1]
            _Main.message[12] = ODTarg[2]
            _Main.message[13] = ODTarg[3]
        except:
            ODTarg = Tools.float2bytes(_Main.ODTarg)
            _Main.message[10] = ODTarg[0]
            _Main.message[11] = ODTarg[1]
            _Main.message[12] = ODTarg[2]
            _Main.message[13] = ODTarg[3]
    
    def UpdateTempTarg(self, ) -> None:
        try:
            _Main.TempTarg = float(self.setTempTarg.text())
            TempTarg = Tools.float2bytes(_Main.TempTarg)
            _Main.message[14] = TempTarg[0]
            _Main.message[15] = TempTarg[1]
            _Main.message[16] = TempTarg[2]
            _Main.message[17] = TempTarg[3]
        except:
            TempTarg = Tools.float2bytes(_Main.TempTarg)
            _Main.message[14] = TempTarg[0]
            _Main.message[15] = TempTarg[1]
            _Main.message[16] = TempTarg[2]
            _Main.message[17] = TempTarg[3]

    # For the updating graph buttons.

    def showGraphOD(self):
        graph_window = GraphWindow('OD Graphs')
        graph_window.exec_()
    
    def showGraphRaw(self, ) -> None:
        graph_window = GraphWindow('Raw Transmittance')
        graph_window.exec_()

    @staticmethod
    def update_msg():
        '''This is a test function with self.editmsg to see what message would be sent. '''
        # This change is dynamical from config file 'Number of variables'
        _Main.message[0] = _Main.DeviceStatus
        _Main.message[1] = _Main.CmotorDir
        _Main.message[2] = _Main.CmotorSpeed
        _Main.message[3] = _Main.FmotorDir
        _Main.message[4] = _Main.FmotorSpeed
        _Main.message[5] = _Main.WmotorDir
        _Main.message[6] = _Main.WmotorSpeed
        _Main.message[7] = _Main.StirSpeed
        _Main.message[8] = _Main.MMMotor
        _Main.message[9] = _Main.MMMotorDelay
        print(_Main.message)

    def set_default(self, ) -> None:
        '''Default values, these can be set prior to running via _Main'''
        ODTarg, TempTarg = Tools.float2bytes(_Main.ODTarg), Tools.float2bytes(_Main.TempTarg)
        # This change is dynamical from config file 'Number of variables'
        # For standard order check the function update_msg() in the UpdateVariable class (static method).
        _Main.message[0] = _Main.DeviceStatus
        _Main.message[1] = 0 # _Main.CmotorDir
        _Main.message[2] = 230 # _Main.CmotorSpeed
        _Main.message[3] = 0 # _Main.FmotorDir
        _Main.message[4] = 0 # _Main.FmotorSpeed
        _Main.message[5] = 0 # _Main.WmotorDir
        _Main.message[6] = 0 # _Main.WmotorSpeed
        _Main.message[7] = 40 # _Main.StirSpeed
        _Main.message[8] = 0 # _Main.MMMotor
        _Main.message[9] = 0 # _Main.MMMotorDelay

        ## float values encoding for ODTarget and TempTargets.
        _Main.message[10] = ODTarg[0] # byte 1 of OD target
        _Main.message[11] = ODTarg[1] # byte 2 of OD target
        _Main.message[12] = ODTarg[2] # byte 3 of OD target
        _Main.message[13] = ODTarg[3] # byte 4 of OD target

        _Main.message[14] = TempTarg[0] # byte 1 of Temp target
        _Main.message[15] = TempTarg[1] # byte 2 of Temp target
        _Main.message[16] = TempTarg[2] # byte 3 of Temp target
        _Main.message[17] = TempTarg[3] # byte 4 of Temp target
    
    def stop_run(self, ) -> None:
        # This change is dynamical from config file 'Number of variables'
        _Main.message = numpy.zeros((20, 1), dtype=numpy.uint8)

class GraphWindow(QDialog):
    def __init__(self, graphType) -> None:
        super().__init__()
        self.graphType = graphType
        self.setWindowTitle("Graph Window")
        self.setGeometry(200, 200, 960, 540)
        # Draw a figure on a fresh widget
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        # Apply a standard layout
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.ax = self.figure.add_subplot()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateGraph)
        self.timer.start((_Main.Capture_freq*1000))  # Update every 4 second
    
    def updateGraph(self):
        try:
            if self.graphType == 'Raw Transmittance':
                self.ax.clear()
                self.ax.plot(_Main.HardwareTime, _Main.Raw1, 'r', alpha=0.5), self.ax.plot(_Main.HardwareTime, _Main.Raw2, 'b', alpha=0.5), self.ax.plot(_Main.HardwareTime, _Main.Raw3, 'g', alpha=0.5), self.ax.legend(['Red = diode 1', 'Blue = diode 2', 'Green = diode 3']), self.ax.set_xlabel('Time (s)'), self.ax.set_ylabel('Raw Transmission (nW)'), self.ax.set_title('Live updating raw transmission graph')
                self.canvas.draw()
            elif self.graphType == 'OD Graphs':
                self.ax.clear()
                self.ax.plot(_Main.HardwareTime, _Main.OD1, 'r', alpha=0.5), self.ax.plot(_Main.HardwareTime, _Main.OD2, 'b', alpha=0.5), self.ax.plot(_Main.HardwareTime, _Main.OD3, 'g', alpha=0.5), self.ax.legend(['Red = diode 1', 'Blue = diode 2', 'Green = diode 3']), self.ax.set_xlabel('Time (s)'), self.ax.set_ylabel('Optical density'), self.ax.set_title('Live updating OD graph')
                self.canvas.draw()
        except:
            pass

def main():
    app = QApplication(sys.argv)
    window = UpdateVariableApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    Tools.create_cfg()
    main()
