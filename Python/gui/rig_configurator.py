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

        # add a textbox to set the camera gain in dB
        layout.addWidget(QtWidgets.QLabel('Gain (dB)'), 9, 0)
        self.gain = QtWidgets.QLineEdit()
        self.gain.setText('0')
        self.gain.setValidator(QtGui.QIntValidator(0,20))
        layout.addWidget(self.gain, 9, 1)

        # add a textbox to set the camera exposure time in us
        layout.addWidget(QtWidgets.QLabel('Exposure Time (us)'), 9, 2)
        max_exposure_time = int(1/int(self.max_frame_rate.text())*1e6)
        self.exposure_time = QtWidgets.QLineEdit()
        self.exposure_time.setText('{}'.format(max_exposure_time))
        self.exposure_time.setValidator(QtGui.QIntValidator(1,max_exposure_time))
        layout.addWidget(self.exposure_time, 9, 3)

        # add a textbox to set the camera index
        layout.addWidget(QtWidgets.QLabel('Camera Index'), 10, 0)
        self.camera_index = QtWidgets.QLineEdit()
        self.camera_index.setText('0')
        self.camera_index.setValidator(QtGui.QIntValidator(0,100))
        layout.addWidget(self.camera_index, 10, 1)

        # add a text box to set time for background calculation
        layout.addWidget(QtWidgets.QLabel('Background Calculation Time (s)'), 10, 2)
        self.background_calculation_time = QtWidgets.QLineEdit()
        self.background_calculation_time.setText('30')
        self.background_calculation_time.setValidator(QtGui.QIntValidator(1,120))
        layout.addWidget(self.background_calculation_time, 10, 3)

        # add a text box to set the opening radius
        layout.addWidget(QtWidgets.QLabel('Opening Radius (pixels)'), 11, 0)
        self.opening_radius = QtWidgets.QLineEdit()
        self.opening_radius.setText('2')
        self.opening_radius.setValidator(QtGui.QIntValidator(1,15))
        layout.addWidget(self.opening_radius, 11, 1)

        # add a text box to set the binarisation threshold
        layout.addWidget(QtWidgets.QLabel('Binarisation Threshold'), 11, 2)
        self.binarisation_threshold = QtWidgets.QLineEdit()
        self.binarisation_threshold.setText('15')
        self.binarisation_threshold.setValidator(QtGui.QIntValidator(1,255))
        layout.addWidget(self.binarisation_threshold, 11, 3)

        # add a checkbox to record video and a text box to set the video folder and a browse button
        self.record_video_checkbox = QtWidgets.QCheckBox('Record Video')
        layout.addWidget(self.record_video_checkbox, 12, 0)
        self.video_folder = QtWidgets.QLineEdit()
        self.video_folder.setText(self.experiment_directory.text()+'\\video\\')
        self.video_folder.setReadOnly(True)
        self.video_folder.setEnabled(False)
        self.browse_video_folder_button = QtWidgets.QPushButton('Browse')
        self.browse_video_folder_button.setEnabled(False)
        layout.addWidget(self.video_folder, 12, 1, 1, 2)
        layout.addWidget(self.browse_video_folder_button, 12, 3)

        # add a checkbox to live stream video and a button to test the video stream
        self.live_stream_checkbox = QtWidgets.QCheckBox('Live Stream')
        layout.addWidget(self.live_stream_checkbox, 13, 0)
        self.test_video_stream_button = QtWidgets.QPushButton('Test Video Stream')
        layout.addWidget(self.test_video_stream_button, 13, 1, 1, 3)
        
        # Add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel('MFC Configuration:'), 14, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a sub-layout to hold the MFC configuration
        self.mfc_configuration_layout = QtWidgets.QGridLayout()

        # add a label to set the MFC Device ID
        self.mfc_device_id_label = QtWidgets.QLabel('Device IDs:')
        self.mfc_configuration_layout.addWidget(self.mfc_device_id_label, 0, 0)

        # add a series of dropboxes to set the MFC Device IDs
        self.mfc_device_id_droboxes = []
        for i in range(16):
            self.mfc_device_id_droboxes.append(QtWidgets.QComboBox())
            self.mfc_device_id_droboxes[i].addItems([chr(i) for i in range(ord('A'), ord('Z')+1)])
            self.mfc_device_id_droboxes[i].setCurrentIndex(i)
            self.mfc_device_id_droboxes[i].setFixedWidth(50)
            self.mfc_configuration_layout.addWidget(self.mfc_device_id_droboxes[i], 0, i+1)
        
        # add the MFC configuration layout to the main layout
        layout.addLayout(self.mfc_configuration_layout, 15, 0, 1, 4)

        # add a text box to set the MFC flow rate
        layout.addWidget(QtWidgets.QLabel('Flow Rate (ml/min)'), 16, 0)
        self.mfc_flow_rate = QtWidgets.QLineEdit()
        self.mfc_flow_rate.setText('300')
        self.mfc_flow_rate.setValidator(QtGui.QIntValidator(1,1000))
        layout.addWidget(self.mfc_flow_rate, 16, 1)

        # add a button to test the MFC
        self.test_mfc_button = QtWidgets.QPushButton('Test MFC')
        layout.addWidget(self.test_mfc_button, 16, 2, 1, 2)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel('Valve Control Configuration:'), 17, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a text box to get ROS environment variable and a browse button
        layout.addWidget(QtWidgets.QLabel('ROS Environment Batch File'), 18, 0)
        self.ros_environment_batch_file = QtWidgets.QLineEdit()
        self.ros_environment_batch_file.setText('C:\\yarena_ws\\install\\setup.bat')
        self.ros_environment_batch_file.setReadOnly(True)
        self.browse_ros_environment_batch_file_button = QtWidgets.QPushButton('Browse')
        layout.addWidget(self.ros_environment_batch_file, 18, 1, 1, 2)
        layout.addWidget(self.browse_ros_environment_batch_file_button, 18, 3)

        # add a button to set minimum message delay
        layout.addWidget(QtWidgets.QLabel('Minimum Message Delay (ms)'), 19, 0)
        self.minimum_message_delay = QtWidgets.QLineEdit()
        self.minimum_message_delay.setText('1')
        self.minimum_message_delay.setValidator(QtGui.QIntValidator(1,10000))
        layout.addWidget(self.minimum_message_delay, 19, 1)

        # add a button to test the odor valves
        self.test_valves_button = QtWidgets.QPushButton('Test Odor Valves')
        layout.addWidget(self.test_valves_button, 19, 2, 1, 2)

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