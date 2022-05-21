from PyQt5 import QtCore, QtGui, QtWidgets
import os
import sys

class MainWindow(QtWidgets.QMainWindow):
    """
    A class to create the main window for the Rig Configurator interface
    """
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # set the window title
        self.setWindowTitle('Rig Configurator')

        # create a the main layout
        layout = QtWidgets.QGridLayout()

        # add a text box for getting the experiment directory with a browse button and a show directory button
        layout.addWidget(QtWidgets.QLabel('Experiment Directory'), 0, 0)
        self.experiment_directory = QtWidgets.QLineEdit()
        if len(args) > 0:
            self.experiment_directory.setText(args[0])
        else:
            self.experiment_directory.setText('')
        self.experiment_directory.setReadOnly(True)
        self.browse_button = QtWidgets.QPushButton('Browse')
        self.show_directory_button = QtWidgets.QPushButton('Show Directory in Explorer')
        layout.addWidget(self.experiment_directory, 0, 1)
        layout.addWidget(self.browse_button, 0, 2)
        layout.addWidget(self.show_directory_button, 0, 3)


        # add a text box for getting the mask file with a browse button and a show button
        layout.addWidget(QtWidgets.QLabel('Mask File'), 1, 0)
        self.mask_file = QtWidgets.QLineEdit()
        self.mask_file.setText('')
        self.mask_file.setReadOnly(True)
        self.browse_mask_button = QtWidgets.QPushButton('Browse')
        self.show_mask_button = QtWidgets.QPushButton('Show Mask')
        layout.addWidget(self.mask_file, 1, 1)
        layout.addWidget(self.browse_mask_button, 1, 2)
        layout.addWidget(self.show_mask_button, 1, 3)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel('LED Array Configuration:'), 2, 0, 1, 4, QtCore.Qt.AlignCenter)

        # create a sub-layout for the LED array configuration
        led_array_layout = QtWidgets.QGridLayout()

        # add a dropbox for selecting the COM port for the 4 LED Modules in a 1x4 grid
        self.com_ports_dropboxes = []
        default_indices = [2,5,4,3]
        led_array_layout.addWidget(QtWidgets.QLabel('COM Port:'), 0, 0)
        for i in range(4):
            led_array_layout.addWidget(QtWidgets.QLabel('Module {}'.format(i+1)), 0, 2*i+1, QtCore.Qt.AlignRight)
            self.com_ports_dropboxes.append(QtWidgets.QComboBox())
            self.com_ports_dropboxes[i].addItems(['COM{}'.format(i+1) for i in range(10)])
            self.com_ports_dropboxes[i].setCurrentIndex(default_indices[i])
            led_array_layout.addWidget(self.com_ports_dropboxes[i], 0, 2*i+2)
        
        # add a dropbox for selecting the quadrant id in a 1x4 grid
        self.quadrant_ids_dropboxes = []
        options = ['1000','0100','0010','0001']
        default_indices = [1,2,3,0]
        led_array_layout.addWidget(QtWidgets.QLabel('Panel ID:'), 1, 0)
        for i in range(4):
            led_array_layout.addWidget(QtWidgets.QLabel('Quadrant {}'.format(i+1)), 1, 2*i+1, QtCore.Qt.AlignRight)
            self.quadrant_ids_dropboxes.append(QtWidgets.QComboBox())
            self.quadrant_ids_dropboxes[i].addItems(options)
            self.quadrant_ids_dropboxes[i].setCurrentIndex(default_indices[i])
            led_array_layout.addWidget(self.quadrant_ids_dropboxes[i], 1, 2*i+2)

        # add sub-layout to main layout
        layout.addLayout(led_array_layout, 3, 0, 1, 4)
        
        # add a text box to set IR intensity 
        layout.addWidget(QtWidgets.QLabel('IR Intensity'), 4, 0)
        self.ir_intensity = QtWidgets.QLineEdit()
        self.ir_intensity.setText('25')
        self.ir_intensity.setValidator(QtGui.QIntValidator(10, 100))
        layout.addWidget(self.ir_intensity, 4, 1)

        # add a dropbox to set the baud rate for the LED Modules
        layout.addWidget(QtWidgets.QLabel('Baud Rate'), 4, 2)
        self.baud_rates_dropbox = QtWidgets.QComboBox()
        self.baud_rates_dropbox.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600'])
        self.baud_rates_dropbox.setCurrentIndex(4)
        layout.addWidget(self.baud_rates_dropbox, 4, 3)

        # add a button to test the IR intensity and RGB LEDs
        self.test_ir_intensity_button = QtWidgets.QPushButton('Test IR Intensity')
        self.test_rgb_leds_button = QtWidgets.QPushButton('Test RGB LEDs')
        layout.addWidget(self.test_ir_intensity_button, 5, 0, 1, 2)
        layout.addWidget(self.test_rgb_leds_button, 5, 2, 1, 2)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel('Camera Configuration:'), 6, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a textbox to set FFMPEG path with a browse button
        layout.addWidget(QtWidgets.QLabel('FFMPEG Path'), 7, 0)
        self.ffmpeg_path = QtWidgets.QLineEdit()
        self.ffmpeg_path.setText('C:\\ffmpeg\\bin\\')
        self.ffmpeg_path.setReadOnly(True)
        self.browse_ffmpeg_button = QtWidgets.QPushButton('Browse')
        layout.addWidget(self.ffmpeg_path, 7, 1)
        layout.addWidget(self.browse_ffmpeg_button, 7, 2, 1, 2)

        # add a dropbox to set the camera pixel format
        layout.addWidget(QtWidgets.QLabel('Pixel Format'), 8, 0)
        self.pixel_formats_dropbox = QtWidgets.QComboBox()
        self.pixel_formats_dropbox.addItems(['Mono8', 'Mono16']) 
        self.pixel_formats_dropbox.setCurrentIndex(0)
        layout.addWidget(self.pixel_formats_dropbox, 8, 1)

        # add a textbox to set the camera maximum frame rate
        layout.addWidget(QtWidgets.QLabel('Max Frame Rate'), 8, 2)
        self.max_frame_rate = QtWidgets.QLineEdit()
        self.max_frame_rate.setText('50')
        self.max_frame_rate.setValidator(QtGui.QIntValidator(1,200))
        layout.addWidget(self.max_frame_rate, 8, 3)



        # # add a text box to get camera index
        # layout.addWidget(QtWidgets.QLabel('Camera Index'), 7, 0)
        # self.camera_index = QtWidgets.QLineEdit()
        # self.camera_index.setText('0')
        # self.camera_index.setValidator(QtGui.QIntValidator(0, 100))
        # layout.addWidget(self.camera_index, 7, 1, 1, 2)

        # # add a text box to get camera exposure time in ms
        # layout.addWidget(QtWidgets.QLabel('Camera Exposure Time (ms)'), 8, 0)
        # self.camera_exposure_time = QtWidgets.QLineEdit()
        # self.camera_exposure_time.setText('6000')
        # self.camera_exposure_time.setValidator(QtGui.QIntValidator(0, 100000))
        # layout.addWidget(self.camera_exposure_time, 8, 1, 1, 2)

        # # add a text box to get camera gain in dB
        # layout.addWidget(QtWidgets.QLabel('Camera Gain (dB)'), 9, 0)
        # self.camera_gain = QtWidgets.QLineEdit()
        # self.camera_gain.setText('0')
        # self.camera_gain.setValidator(QtGui.QIntValidator(0, 100))
        # layout.addWidget(self.camera_gain, 9, 1, 1, 2)



        # create the main widget
        self.main_widget = QtWidgets.QWidget()
        self.main_widget.setLayout(layout)
        self.setCentralWidget(self.main_widget)
        
        # show the window
        self.show()

        
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())