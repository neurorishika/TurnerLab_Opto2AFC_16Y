from PyQt5 import QtCore, QtGui, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np
import os

class MplCanvas(FigureCanvas):
    """
    Class for plotting the the 1
    
    variables:
        self.fig: matplotlib figure object
        self.axes: matplotlib axes object
    """
    def __init__(self, parent=None, width=5, height=4, dpi=256,interval=10, temp_folder='temp/'):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.temp_folder = temp_folder

        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

        # add a timer to update the plot
        self.timer = self.new_timer(interval,[(self.update_image,(),{})])
        self.timer.start()
    
    def update_image(self):
        """
        Update the plot with new data
        """
        # get the latest image from the temp folder
        if os.path.exists(self.temp_folder+'image.npy'):
            image_path = self.temp_folder + 'image.npy'
            image = np.load(image_path)
            self.axes.imshow(image)
            self.draw()

class CameraViewer(QtWidgets.QMainWindow):
    """
    A class to show the camera image in a matplotlib window
    """
    def __init__(self, *args, **kwargs):
        super(CameraViewer, self).__init__(*args, **kwargs)

        # set the window title
        self.setWindowTitle('Camera Viewer')

        # create a the main layout
        layout = QtWidgets.QGridLayout()

        # add the matplotlib canvas to the main layout
        self.canvas = MplCanvas(self, width=5, height=4, dpi=10)
        layout.addWidget(self.canvas, 0, 0)

        # create the main widget
        self.main_widget = QtWidgets.QWidget()
        self.main_widget.setLayout(layout)
        self.setCentralWidget(self.main_widget)

        # show the window
        self.show()

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = CameraViewer()
    window.show()
    app.exec_()