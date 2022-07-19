import sys
import os

import json
import numpy as np
import matplotlib

matplotlib.use("Qt5Agg")

from PyQt5 import QtCore, QtGui, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MplCanvas(FigureCanvas):
    """
    Class for plotting the the stimulus waveform using matplotlib.
    
    variables:
        self.fig: matplotlib figure object
        self.axes: matplotlib axes object
    """

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        """
        Initialize the figure and axes.
        """
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)

        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


class MainWindow(QtWidgets.QMainWindow):
    """
    Class for the main window of the stimulus designer.

    variables:
        self.graph: matplotlib figure object
        self.pulse_width: QLineEdit object for the pulse width
        self.pulse_period: QLineEdit object for the pulse period
        self.pulse_count: QLineEdit object for the pulse count
        self.pulse_delay: QLineEdit object for the pulse delay
        self.pulse_repeat: QLineEdit object for the pulse repeat
        self.pulse_deadtime: QLineEdit object for the pulse deadtime
        self.color: QComboBox object for the color
        self.intensity: QLineEdit object for the intensity
    """

    def __init__(self, start_folder=None, load_file=None, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # Create the grapher
        self.graph = MplCanvas(self, width=5, height=2, dpi=100)
        self.graph.fig.tight_layout()

        # Create the main layout
        layout = QtWidgets.QGridLayout()

        # add the matplotlib canvas to the layout
        layout.addWidget(self.graph, 0, 0, 9, 1)

        # store the start folder and load file
        self.start_folder = start_folder
        self.load_file = load_file

        # add a pulse width edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Pulse Width (ms)"), 0, 1)
        self.pulse_width = QtWidgets.QLineEdit("500")
        self.pulse_width.setValidator(QtGui.QIntValidator(1, 30000))
        layout.addWidget(self.pulse_width, 0, 2)

        # add a pulse period edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Pulse Period (ms)"), 1, 1)
        self.pulse_period = QtWidgets.QLineEdit("1000")
        self.pulse_period.setValidator(QtGui.QIntValidator(1, 30000))
        layout.addWidget(self.pulse_period, 1, 2)

        # add a pulse count edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Pulse Count"), 2, 1)
        self.pulse_count = QtWidgets.QLineEdit("1")
        self.pulse_count.setValidator(QtGui.QIntValidator(1, 1000))
        layout.addWidget(self.pulse_count, 2, 2)

        # add a pulse delay edit box with Float validator
        layout.addWidget(QtWidgets.QLabel("Pulse Delay (s)"), 3, 1)
        self.pulse_delay = QtWidgets.QLineEdit("0")
        self.pulse_delay.setValidator(QtGui.QDoubleValidator(0, 120, 5))
        layout.addWidget(self.pulse_delay, 3, 2)

        # add a pulse repeat edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Pulse Repeat"), 4, 1)
        self.pulse_repeat = QtWidgets.QLineEdit("1")
        self.pulse_repeat.setValidator(QtGui.QIntValidator(0, 30000))
        layout.addWidget(self.pulse_repeat, 4, 2)

        # add a pulse dead-time edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Pulse Deadtime"), 5, 1)
        self.pulse_deadtime = QtWidgets.QLineEdit("0")
        self.pulse_deadtime.setValidator(QtGui.QIntValidator(0, 30000))
        layout.addWidget(self.pulse_deadtime, 5, 2)

        # add a stimulus color dropdown menu
        layout.addWidget(QtWidgets.QLabel("Color"), 6, 1)
        self.color = QtWidgets.QComboBox()
        self.color.addItems(["RED", "GREEN", "BLUE", "CUSTOM"])
        self.color.currentIndexChanged.connect(self.toggle_custom_color)
        layout.addWidget(self.color, 6, 2)

        # add a custom color dialog
        layout.addWidget(QtWidgets.QLabel("Custom Color"), 7, 1)
        self.custom_color = QtWidgets.QPushButton("#ffffff")
        self.custom_color.clicked.connect(self.custom_color_dialog)
        layout.addWidget(self.custom_color, 7, 2)

        # add a stimulus intensity edit box with Integer validator
        layout.addWidget(QtWidgets.QLabel("Intensity"), 8, 1)
        self.intensity = QtWidgets.QLineEdit("100")
        self.intensity.setValidator(QtGui.QIntValidator())
        layout.addWidget(self.intensity, 8, 2)

        # initialize the custom color button to disabled
        self.toggle_custom_color()

        # plot the stimulus waveform
        self.update_stimulus()

        # add the update, load and save buttons
        self.update_button = QtWidgets.QPushButton("Update Stimulus")
        self.update_button.clicked.connect(self.update_stimulus)
        layout.addWidget(self.update_button, 9, 0)

        self.load_button = QtWidgets.QPushButton("Load Stimulus")
        self.load_button.clicked.connect(self.load_stimulus)
        layout.addWidget(self.load_button, 9, 1)

        self.save_button = QtWidgets.QPushButton("Save Stimulus")
        self.save_button.clicked.connect(self.save_stimulus)
        layout.addWidget(self.save_button, 9, 2)

        # create a widget to display the layout
        w = QtWidgets.QWidget()
        w.setLayout(layout)
        self.setCentralWidget(w)

        self.setWindowTitle("16Y Stimulus Designer")
        self.show()

        # if load_file is not None, load the file
        if self.load_file is not None:
            self.load_stimulus(self.load_file)

    def generate_waveform(self):
        """
        Generate the stimulus waveform from the defined parameters.

        pulse_width: pulse width in milliseconds
        pulse_period: pulse period in milliseconds
        pulse_count: number of pulses in a single sequence
        pulse_repeat: number of times to repeat the pulse sequence
        pulse_deadtime: deadtime between pulse sequences in a pulse train in milliseconds
        pulse_delay: delay before the first pulse in the full pulse train in seconds
        """
        pulse_width = int(self.pulse_width.text())
        pulse_period = int(self.pulse_period.text())
        pulse_count = int(self.pulse_count.text())
        pulse_delay = int(float(self.pulse_delay.text()) * 1000)
        pulse_repeat = int(self.pulse_repeat.text())
        pulse_deadtime = int(self.pulse_deadtime.text())
        intensity = int(self.intensity.text())
        wave_unit = np.concatenate((intensity * np.ones(pulse_width), np.zeros(pulse_period - pulse_width)))
        wave_cycle = np.concatenate((np.tile(wave_unit, pulse_count), np.zeros(pulse_deadtime)))
        wave_repeat = np.tile(wave_cycle, pulse_repeat)
        wave_train = np.concatenate((np.zeros(pulse_delay), wave_repeat))
        return wave_train

    def update_stimulus(self):
        """
        Update the stimulus waveform plot.
        """
        wave_train = self.generate_waveform()
        color = self.color.currentText()
        self.graph.axes.clear()
        if color == "RED":
            self.graph.axes.plot(wave_train, "r")
            self.stim_color = "R"
        elif color == "GREEN":
            self.graph.axes.plot(wave_train, "g")
            self.stim_color = "G"
        elif color == "BLUE":
            self.graph.axes.plot(wave_train, "b")
            self.stim_color = "B"
        elif color == "CUSTOM":
            self.graph.axes.plot(wave_train, self.custom_color.text())
            self.stim_color = self.custom_color.text()
        self.graph.axes.set_xlabel("Time (ms)")
        self.graph.axes.set_ylabel("Intensity (%)")
        self.graph.axes.set_ylim([-5, 105])
        self.graph.fig.tight_layout()
        self.graph.draw()

    def load_stimulus(self, load_file=None):
        """
        Load the stimulus waveform from a .stim file.
        """
        if load_file is None:
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Stimulus", ".", "*.stim")
        else:
            filename = load_file

        if filename:
            with open(filename, "r") as f:
                data = json.load(f)
            self.pulse_width.setText(str(data["pulse_width"]))
            self.pulse_period.setText(str(data["pulse_period"]))
            self.pulse_count.setText(str(data["pulse_count"]))
            self.pulse_delay.setText(str(data["pulse_delay"]))
            self.pulse_repeat.setText(str(data["pulse_repeat"]))
            self.pulse_deadtime.setText(str(data["pulse_deadtime"]))
            if data["color"] == "R":
                self.color.setCurrentIndex(0)
            elif data["color"] == "G":
                self.color.setCurrentIndex(1)
            elif data["color"] == "B":
                self.color.setCurrentIndex(2)
            else:
                self.color.setCurrentIndex(3)
                self.custom_color.setEnabled(True)
                self.update_color_button(data["color"])
            self.intensity.setText(str(data["intensity"]))
            self.update_stimulus()

    def save_stimulus(self):
        """
        Save the stimulus waveform to a .stim file. 
        """
        self.update_stimulus()
        default_filename = (
            self.stim_color
            + "-"
            + self.intensity.text()
            + "_"
            + self.pulse_delay.text()
            + "+["
            + self.pulse_count.text()
            + "x("
            + self.pulse_width.text()
            + "v"
            + self.pulse_period.text()
            + ")+"
            + self.pulse_deadtime.text()
            + "]X"
            + self.pulse_repeat.text()
            + ".stim"
        )
        if self.start_folder is not None:
            default_filename = self.start_folder + "/" + default_filename
        else:
            default_filename = "./" + default_filename

        if self.load_file is not None:
            filename = self.load_file
        else:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Stimulus", default_filename, "*.stim")
        if filename:
            data = {
                "pulse_width": int(self.pulse_width.text()),
                "pulse_period": int(self.pulse_period.text()),
                "pulse_count": int(self.pulse_count.text()),
                "pulse_delay": int(self.pulse_delay.text()),
                "pulse_repeat": int(self.pulse_repeat.text()),
                "pulse_deadtime": int(self.pulse_deadtime.text()),
                "color": self.stim_color,
                "intensity": int(self.intensity.text()),
            }

            with open(filename, "w") as f:
                json.dump(data, f)

        # ask user if they want to close the window
        reply = QtWidgets.QMessageBox.question(
            self,
            "Save Stimulus",
            "Do you want to close the stimulus designer?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close()

    def toggle_custom_color(self):
        """
        Toggle the custom color button.
        """
        if self.color.currentText() == "CUSTOM":
            self.custom_color.setEnabled(True)
            self.intensity.setEnabled(False)
        else:
            self.custom_color.setEnabled(False)
            self.custom_color.setText("#ffffff")
            self.update_color_button(self.custom_color.text())
            self.intensity.setEnabled(True)

    def custom_color_dialog(self):
        """
        Open a dialog to choose a custom color.
        """
        color = QtWidgets.QColorDialog.getColor()
        if color.isValid():
            self.update_color_button(color.name())

    def update_color_button(self, color):
        """
        Update the color button to the selected color.
        
        Variables:
            color: color in hex string format
        """
        rgb = color.lstrip("#")
        rgb = tuple(int(rgb[i : i + 2], 16) for i in (0, 2, 4))
        lightness = rgb[0] * 0.3 + rgb[1] * 0.59 + rgb[2] * 0.11
        optimal_text_color = "black" if lightness > 186 else "white"
        self.custom_color.setText(color)
        self.custom_color.setStyleSheet("background-color: %s; color: %s" % (color, optimal_text_color))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    if len(sys.argv) > 1:
        # check if a file is passed as an argument
        if os.path.isfile(sys.argv[1]):
            # if so, pass as load file else pass as start folder
            window = MainWindow(start_folder=None, load_file=sys.argv[1])
        else:
            window = MainWindow(start_folder=sys.argv[1], load_file=None)
    else:
        window = MainWindow()
    sys.exit(app.exec_())
