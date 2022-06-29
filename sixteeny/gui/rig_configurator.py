from PyQt5 import QtCore, QtGui, QtWidgets
import json
import sys
import numpy as np
import matplotlib.pyplot as plt


class MainWindow(QtWidgets.QMainWindow):
    """
    A class to create the main window for the Rig Configurator interface
    """

    def __init__(self, directory="", *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # set the window title
        self.setWindowTitle("Rig Configurator")

        # create a the main layout
        layout = QtWidgets.QGridLayout()

        # add a text box for getting the experiment directory with a browse button and a show directory button
        layout.addWidget(QtWidgets.QLabel("Experiment Directory"), 0, 0)
        self.experiment_directory = QtWidgets.QLineEdit()
        self.experiment_directory.setText(directory)
        self.experiment_directory.setReadOnly(True)
        self.browse_button = QtWidgets.QPushButton("Browse")
        layout.addWidget(self.experiment_directory, 0, 1)
        layout.addWidget(self.browse_button, 0, 2, 1, 2)

        self.browse_button.clicked.connect(self.browse_experiment_directory)

        # add a text box for getting the mask file with a browse button and a show button
        layout.addWidget(QtWidgets.QLabel("Mask File"), 1, 0)
        self.mask_file = QtWidgets.QLineEdit()
        self.mask_file.setText("")
        self.mask_file.setReadOnly(True)
        self.browse_mask_button = QtWidgets.QPushButton("Browse")
        self.show_mask_button = QtWidgets.QPushButton("Show Mask")
        layout.addWidget(self.mask_file, 1, 1)
        layout.addWidget(self.browse_mask_button, 1, 2)
        layout.addWidget(self.show_mask_button, 1, 3)

        self.browse_mask_button.clicked.connect(self.browse_mask_file)
        self.show_mask_button.clicked.connect(self.show_mask_file)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel("LED Array Configuration:"), 2, 0, 1, 4, QtCore.Qt.AlignCenter)

        # create a sub-layout for the LED array configuration
        led_array_layout = QtWidgets.QGridLayout()

        # add a dropbox for selecting the COM port for the 4 LED Modules in a 1x4 grid
        self.com_ports_dropboxes = []
        default_indices = [12,4,11,3]
        led_array_layout.addWidget(QtWidgets.QLabel("COM Port:"), 0, 0)
        for i in range(4):
            led_array_layout.addWidget(QtWidgets.QLabel("Module {}".format(i + 1)), 0, 2 * i + 1, QtCore.Qt.AlignRight)
            self.com_ports_dropboxes.append(QtWidgets.QComboBox())
            self.com_ports_dropboxes[i].addItems(["COM{}".format(i + 1) for i in range(15)])
            self.com_ports_dropboxes[i].setCurrentIndex(default_indices[i])
            led_array_layout.addWidget(self.com_ports_dropboxes[i], 0, 2 * i + 2)

        # add a dropbox for selecting the quadrant id in a 1x4 grid
        self.quadrant_ids_dropboxes = []
        options = ["0001", "0010", "1000", "0100"]
        default_indices = [0, 1, 2, 3]
        led_array_layout.addWidget(QtWidgets.QLabel("Panel ID:"), 1, 0)
        for i in range(4):
            led_array_layout.addWidget(
                QtWidgets.QLabel("Quadrant {}".format(i + 1)), 1, 2 * i + 1, QtCore.Qt.AlignRight
            )
            self.quadrant_ids_dropboxes.append(QtWidgets.QComboBox())
            self.quadrant_ids_dropboxes[i].addItems(options)
            self.quadrant_ids_dropboxes[i].setCurrentIndex(default_indices[i])
            led_array_layout.addWidget(self.quadrant_ids_dropboxes[i], 1, 2 * i + 2)

        # add sub-layout to main layout
        layout.addLayout(led_array_layout, 3, 0, 1, 4)

        # add a text box to set IR intensity
        layout.addWidget(QtWidgets.QLabel("IR Intensity"), 4, 0)
        self.ir_intensity = QtWidgets.QLineEdit()
        self.ir_intensity.setText("25")
        self.ir_intensity.setValidator(QtGui.QIntValidator(10, 100))
        layout.addWidget(self.ir_intensity, 4, 1)

        # add a dropbox to set the baud rate for the LED Modules
        layout.addWidget(QtWidgets.QLabel("Baud Rate"), 4, 2)
        self.baud_rates_dropbox = QtWidgets.QComboBox()
        self.baud_rates_dropbox.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_rates_dropbox.setCurrentIndex(4)
        layout.addWidget(self.baud_rates_dropbox, 4, 3)

        # add a button to test the IR intensity and RGB LEDs
        self.test_ir_intensity_button = QtWidgets.QPushButton("Test IR Intensity")
        self.test_rgb_leds_button = QtWidgets.QPushButton("Test RGB LEDs")
        layout.addWidget(self.test_ir_intensity_button, 5, 0, 1, 2)
        layout.addWidget(self.test_rgb_leds_button, 5, 2, 1, 2)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel("Camera Configuration:"), 6, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a dropbox to set the camera pixel format
        layout.addWidget(QtWidgets.QLabel("Pixel Format"), 8, 0)
        self.pixel_formats_dropbox = QtWidgets.QComboBox()
        self.pixel_formats_dropbox.addItems(["Mono8", "Mono16"])
        self.pixel_formats_dropbox.setCurrentIndex(0)
        layout.addWidget(self.pixel_formats_dropbox, 8, 1)

        # add a textbox to set the camera maximum frame rate
        layout.addWidget(QtWidgets.QLabel("Gamma"), 8, 2)
        self.gamma = QtWidgets.QLineEdit()
        self.gamma.setText("1")
        self.gamma.setValidator(QtGui.QDoubleValidator(0.1, 10, 2))
        layout.addWidget(self.gamma, 8, 3)

        # add a textbox to set the camera gain in dB
        layout.addWidget(QtWidgets.QLabel("Gain (dB)"), 9, 0)
        self.gain = QtWidgets.QLineEdit()
        self.gain.setText("0")
        self.gain.setValidator(QtGui.QIntValidator(0, 20))
        layout.addWidget(self.gain, 9, 1)

        # add a textbox to set the camera exposure time in us
        layout.addWidget(QtWidgets.QLabel("Exposure Time (us)"), 9, 2)
        self.exposure_time = QtWidgets.QLineEdit()
        self.exposure_time.setText("{}".format(20000))
        self.exposure_time.setValidator(QtGui.QIntValidator(1, 100000))
        layout.addWidget(self.exposure_time, 9, 3)

        # add a textbox to set the camera index
        layout.addWidget(QtWidgets.QLabel("Camera Index"), 10, 0)
        self.camera_index = QtWidgets.QLineEdit()
        self.camera_index.setText("0")
        self.camera_index.setValidator(QtGui.QIntValidator(0, 100))
        layout.addWidget(self.camera_index, 10, 1)

        # add a text box to set time for background calculation
        layout.addWidget(QtWidgets.QLabel("Background Calculation Time (s)"), 10, 2)
        self.background_calculation_time = QtWidgets.QLineEdit()
        self.background_calculation_time.setText("30")
        self.background_calculation_time.setValidator(QtGui.QIntValidator(1, 120))
        layout.addWidget(self.background_calculation_time, 10, 3)

        # add a text box to set the closing radius
        layout.addWidget(QtWidgets.QLabel("Closing Radius (pixels)"), 11, 0)
        self.closing_radius = QtWidgets.QLineEdit()
        self.closing_radius.setText("5")
        self.closing_radius.setValidator(QtGui.QIntValidator(1, 20))
        layout.addWidget(self.closing_radius, 11, 1)

        # add a text box to set the erosion radius
        layout.addWidget(QtWidgets.QLabel("Binarization Threshold (Relative; %)"), 11, 2)
        self.binarization_threshold_relative = QtWidgets.QLineEdit()
        self.binarization_threshold_relative.setText("0")
        self.binarization_threshold_relative.setValidator(QtGui.QIntValidator(0, 100))
        layout.addWidget(self.binarization_threshold_relative, 11, 3)

        # add a checkbox to record video
        self.record_video_checkbox = QtWidgets.QCheckBox("Record Video")
        layout.addWidget(self.record_video_checkbox, 12, 0)

        # add a checkbox to live stream video
        self.live_stream_checkbox = QtWidgets.QCheckBox("Live Stream (Reduces FPS)")
        layout.addWidget(self.live_stream_checkbox, 12, 1)

        # add a text box to set the binarization threshold
        layout.addWidget(QtWidgets.QLabel("Binarization Threshold (Absolute)"), 12, 2)
        self.binarization_threshold_absolute = QtWidgets.QLineEdit()
        self.binarization_threshold_absolute.setText("15")
        self.binarization_threshold_absolute.setValidator(QtGui.QIntValidator(0, 65535))
        layout.addWidget(self.binarization_threshold_absolute, 12, 3)

        # add a button to test the video stream
        self.test_video_stream_button = QtWidgets.QPushButton("Test Video Stream")
        layout.addWidget(self.test_video_stream_button, 13, 0, 1, 4)

        # Add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel("MFC Configuration:"), 14, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a drop box to set the MFC COM port
        layout.addWidget(QtWidgets.QLabel("COM Port"), 15, 0)
        self.mfc_com_port = QtWidgets.QComboBox()
        self.mfc_com_port.addItems(["COM{}".format(i + 1) for i in range(20)])
        self.mfc_com_port.setCurrentIndex(7)
        layout.addWidget(self.mfc_com_port, 15, 1, 1, 3)

        # add a sub-layout to hold the MFC configuration
        self.mfc_configuration_layout = QtWidgets.QGridLayout()

        # add a label to set the MFC Device ID
        self.mfc_device_id_label = QtWidgets.QLabel("Device IDs:")
        self.mfc_configuration_layout.addWidget(self.mfc_device_id_label, 0, 0)

        # add a series of dropboxes to set the MFC Device IDs
        self.mfc_device_id_droboxes = []
        for i in range(16):
            self.mfc_device_id_droboxes.append(QtWidgets.QComboBox())
            self.mfc_device_id_droboxes[i].addItems([chr(i) for i in range(ord("A"), ord("Z") + 1)])
            self.mfc_device_id_droboxes[i].setCurrentIndex(i)
            self.mfc_device_id_droboxes[i].setFixedWidth(50)
            self.mfc_configuration_layout.addWidget(self.mfc_device_id_droboxes[i], 0, i + 1)

        # add the MFC configuration layout to the main layout
        layout.addLayout(self.mfc_configuration_layout, 16, 0, 1, 4)

        # add a text box to set the MFC flow rate
        layout.addWidget(QtWidgets.QLabel("Flow Rate (ml/min)"), 17, 0)
        self.mfc_flow_rate = QtWidgets.QLineEdit()
        self.mfc_flow_rate.setText("300")
        self.mfc_flow_rate.setValidator(QtGui.QIntValidator(1, 1000))
        layout.addWidget(self.mfc_flow_rate, 17, 1)

        # add a button to test the MFC
        self.test_mfc_button = QtWidgets.QPushButton("Test MFC")
        layout.addWidget(self.test_mfc_button, 17, 2, 1, 2)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel("Valve Control Configuration:"), 18, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a button to set minimum message delay
        layout.addWidget(QtWidgets.QLabel("Minimum Message Delay (ms)"), 20, 0)
        self.minimum_message_delay = QtWidgets.QLineEdit()
        self.minimum_message_delay.setText("1")
        self.minimum_message_delay.setValidator(QtGui.QIntValidator(1, 10000))
        layout.addWidget(self.minimum_message_delay, 20, 1)

        # add a button to test the odor valves
        self.test_valves_button = QtWidgets.QPushButton("Test Odor Valves")
        layout.addWidget(self.test_valves_button, 20, 2, 1, 2)

        # add a centre-aligned label at the top of the next row
        layout.addWidget(QtWidgets.QLabel("Odor Configuration:"), 21, 0, 1, 4, QtCore.Qt.AlignCenter)

        # add a text box to set the odor 1 name
        layout.addWidget(QtWidgets.QLabel("Odor 1"), 22, 0)
        self.odor_1_name = QtWidgets.QLineEdit()
        self.odor_1_name.setText("PA")
        layout.addWidget(self.odor_1_name, 22, 1)

        # add a text box to set the odor 2 name
        layout.addWidget(QtWidgets.QLabel("Odor 2"), 22, 2)
        self.odor_2_name = QtWidgets.QLineEdit()
        self.odor_2_name.setText("EL")
        layout.addWidget(self.odor_2_name, 22, 3)

        # Add a button to enable the GPU processing
        self.enable_gpu_processing_checkbox = QtWidgets.QCheckBox("Enable GPU Processing")
        layout.addWidget(self.enable_gpu_processing_checkbox, 23, 0)

        # Add a button to load and save the configuration
        self.load_configuration_button = QtWidgets.QPushButton("Load Configuration")
        layout.addWidget(self.load_configuration_button, 23, 1)
        self.load_configuration_button.clicked.connect(self.load_configuration)

        self.save_configuration_button = QtWidgets.QPushButton("Save Configuration")
        layout.addWidget(self.save_configuration_button, 23, 2, 1, 2)
        self.save_configuration_button.clicked.connect(self.save_configuration)

        # create the main widget
        self.main_widget = QtWidgets.QWidget()
        self.main_widget.setLayout(layout)
        self.setCentralWidget(self.main_widget)

        # show the window
        self.show()

    def load_configuration(self):
        """
        A function to load the configuration from a file
        """
        # get the file name
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Open Configuration File", "", "*.yarena")[0]

        if file_name == "":
            return

        # load the configuration as a json object
        with open(file_name, "r") as f:
            configuration = json.load(f)

        # set the values of the widgets
        # general
        self.experiment_directory.setText(configuration["experiment_directory"])
        self.mask_file.setText(configuration["mask_file"])

        # led
        for i in range(4):
            self.com_ports_dropboxes[i].setCurrentIndex(
                self.com_ports_dropboxes[i].findText(configuration["com_ports"][i])
            )
            self.quadrant_ids_dropboxes[i].setCurrentIndex(
                self.quadrant_ids_dropboxes[i].findText(configuration["quadrant_ids"][i])
            )
        self.ir_intensity.setText(str(configuration["ir_intensity"]))
        self.baud_rates_dropbox.setCurrentIndex(self.baud_rates_dropbox.findText(configuration["baud_rate"]))

        # camera
        self.pixel_formats_dropbox.setCurrentIndex(self.pixel_formats_dropbox.findText(configuration["pixel_format"]))
        self.gain.setText(str(configuration["gain"]))
        self.gamma.setText(str(configuration["gamma"]))
        self.exposure_time.setText(str(configuration["exposure_time"]))
        self.camera_index.setText(str(configuration["camera_index"]))
        self.background_calculation_time.setText(str(configuration["background_calculation_time"]))
        self.closing_radius.setText(str(configuration["closing_radius"]))
        self.binarization_threshold_relative.setText(str(configuration["binarization_threshold_relative"]))
        self.binarization_threshold_absolute.setText(str(configuration["binarization_threshold_absolute"]))
        self.record_video_checkbox.setChecked(configuration["record_video"])
        self.live_stream_checkbox.setChecked(configuration["live_stream"])

        # mfc
        self.mfc_com_port.setCurrentIndex(self.mfc_com_port.findText(configuration["mfc_com_port"]))
        for i in range(16):
            self.mfc_device_id_droboxes[i].setCurrentIndex(
                self.mfc_device_id_droboxes[i].findText(configuration["mfc_device_ids"][i])
            )
        self.mfc_flow_rate.setText(str(configuration["mfc_flow_rate"]))

        # valve control
        self.minimum_message_delay.setText(str(configuration["minimum_message_delay"]))

        # odor
        self.odor_1_name.setText(configuration["odor_1"])
        self.odor_2_name.setText(configuration["odor_2"])

        # gpu processing
        self.enable_gpu_processing_checkbox.setChecked(configuration["enable_gpu_processing"])

    def save_configuration(self):
        """
        A function to save the configuration to a file
        """
        # set experiment directory as the file name
        file_name = self.experiment_directory.text() + "/config.yarena"

        # verify either the relative or absolute binarization threshold is 0
        if (
            self.binarization_threshold_relative.text() == "0" and self.binarization_threshold_absolute.text() == "0"
        ) or (
            self.binarization_threshold_relative.text() != "0" and self.binarization_threshold_absolute.text() != "0"
        ):
            QtWidgets.QMessageBox.warning(
                self, "Error", "Either the relative or absolute binarization threshold must be non-zero",
            )
            return
        else:
            threshold_type = "relative" if self.binarization_threshold_relative.text() != "0" else "absolute"

        # create the json object
        configuration = {}
        # general
        configuration["experiment_directory"] = self.experiment_directory.text()
        configuration["mask_file"] = self.mask_file.text()
        # led
        configuration["com_ports"] = []
        configuration["quadrant_ids"] = []
        for i in range(4):
            configuration["com_ports"].append(self.com_ports_dropboxes[i].currentText())
            configuration["quadrant_ids"].append(self.quadrant_ids_dropboxes[i].currentText())
        configuration["ir_intensity"] = int(self.ir_intensity.text())
        configuration["baud_rate"] = self.baud_rates_dropbox.currentText()
        # camera
        configuration["pixel_format"] = self.pixel_formats_dropbox.currentText()
        configuration["gain"] = int(self.gain.text())
        configuration["gamma"] = int(self.gamma.text())
        configuration["exposure_time"] = int(self.exposure_time.text())
        configuration["camera_index"] = int(self.camera_index.text())
        configuration["background_calculation_time"] = int(self.background_calculation_time.text())
        configuration["closing_radius"] = int(self.closing_radius.text())
        configuration["binarization_threshold_relative"] = int(self.binarization_threshold_relative.text())
        configuration["binarization_threshold_absolute"] = int(self.binarization_threshold_absolute.text())
        configuration["threshold_type"] = threshold_type
        configuration["record_video"] = self.record_video_checkbox.isChecked()
        configuration["live_stream"] = self.live_stream_checkbox.isChecked()
        # mfc
        configuration["mfc_com_port"] = self.mfc_com_port.currentText()
        configuration["mfc_device_ids"] = []
        for i in range(16):
            configuration["mfc_device_ids"].append(self.mfc_device_id_droboxes[i].currentText())
        configuration["mfc_flow_rate"] = int(self.mfc_flow_rate.text())
        # valve control
        configuration["minimum_message_delay"] = int(self.minimum_message_delay.text())
        # gpu processing
        configuration["enable_gpu_processing"] = self.enable_gpu_processing_checkbox.isChecked()
        # odor
        configuration["odor_1"] = self.odor_1_name.text()
        configuration["odor_2"] = self.odor_2_name.text()

        # save the configuration as a json object
        with open(file_name, "w") as f:
            json.dump(configuration, f)

        # show a message box saying that the configuration has been saved
        QtWidgets.QMessageBox.information(
            self, "Configuration Saved", "The configuration has been saved to " + file_name
        )

        # ask the user if they want to exit
        reply = QtWidgets.QMessageBox.question(
            self,
            "Save Configuration",
            "Do you want to exit?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close()

    def browse_experiment_directory(self):
        """
        A function to browse for the experiment directory
        """
        # get the directory
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Experiment Directory")

        if directory == "":
            # show a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "No directory selected")
            return

        # set the directory
        self.experiment_directory.setText(directory)

    def browse_mask_file(self):
        """
        A function to browse for the mask file
        """
        # get the file name
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Select Mask File", "", "*.npz")[0]

        if file_name == "":
            # show an warning
            QtWidgets.QMessageBox.warning(self, "Warning", "No file selected")
            return

        # set the file name
        self.mask_file.setText(file_name)

    def show_mask_file(self):
        """
        A function to show the mask file
        """
        # get the file name
        file_name = self.mask_file.text()

        if file_name == "":
            # show a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "No mask file selected")
            return

        # load the mask file
        arm_mask, reward_mask, _ = np.load(file_name, allow_pickle=True)

        # show the mask
        plt.imshow(arm_mask.sum(axis=0) + reward_mask.sum(axis=0), cmap="gray")
        plt.show()

    def browse_ffmpeg_path(self):
        """
        A function to browse for the ffmpeg path
        """
        # get the folder name
        folder_name = QtWidgets.QFileDialog.getExistingDirectory(self, "Select FFMPEG Folder")

        if folder_name == "":
            # show a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "No folder selected")
            return

        # set the file name
        self.ffmpeg_path.setText(folder_name)

    def browse_ros_environment_batch_file(self):
        """
        A function to browse for the ROS environment batch file
        """
        # get the file name
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Select ROS Environment Batch File", "", "*.bat")[0]

        if file_name == "":
            # show an warning
            QtWidgets.QMessageBox.warning(self, "Warning", "No file selected")
            return

        # set the file name
        self.ros_environment_batch_file.setText(file_name)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    if len(sys.argv) > 1:
        main = MainWindow(sys.argv[1])
    else:
        main = MainWindow()
    main.show()
    sys.exit(app.exec_())
