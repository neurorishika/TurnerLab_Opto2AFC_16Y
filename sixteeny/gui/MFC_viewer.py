from re import A
import sys
import os
from turtle import color

from matplotlib.backends.qt_compat import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvas
import matplotlib.pyplot as plt

from sixteeny.controller.mfc import MFCController

import datetime
import numpy as np

class MainWindow(QtWidgets.QMainWindow):
    '''
    The main PyQt5 application window.
    '''
    def __init__(self, MFCController=None, save_directory=''):
        super().__init__()
        self.MFCController = MFCController
        self.save_directory = save_directory
        self.setGeometry(600, 600, 1600, 800)
        self.setWindowTitle("MFC flow rate viewer")
        self.frm = QtWidgets.QFrame(self)
        self.frm.setStyleSheet("QWidget { background-color: #eeeeec; }")
        self.lyt = QtWidgets.QVBoxLayout()
        self.frm.setLayout(self.lyt)
        self.setCentralWidget(self.frm)

        self.graph = FlowCanvas(x_len=200, y_range=[0, 5000], interval=1000, controller=self.MFCController, save_directory=self.save_directory)
        self.lyt.addWidget(self.graph)

        self.show()
        return

class FlowCanvas(FigureCanvas):
    '''
    This is the FigureCanvas in which the live plot is drawn.

    '''
    def __init__(self, x_len, y_range, interval, controller, save_directory) -> None:
        '''
        Initialize the figure and axes.
        '''
        super().__init__()

        # self.n = np.linspace(0, 499, 500)
        # self.d = np.array([50+i + 25 * (np.sin(self.n / 8.3)) + 10 * (np.sin(self.n / 7.5)) - 5 * (np.sin(self.n / 1.5)) for i in range(n_controllers)]).T
        # self.i = 0

        self.controller = controller
        self.n_controllers = len(controller.mfcs)
        self.save_directory = save_directory

        # Range settings
        self._x_len_ = x_len
        self._y_range_ = y_range

        # Store two lists _x_ and _y_
        self._x_ = np.arange(0, x_len)
        self._y_ = np.zeros((x_len,self.n_controllers))

        # Store a figure ax
        self._ax_ = self.figure.subplots()
        self._ax_.set_ylim(ymin=self._y_range_[0], ymax=self._y_range_[1])
        self._line_ = []
        for i in range(self.n_controllers):
            _line_, = self._ax_.plot(self._x_, self._y_[:,i], label=chr(65+i), color=plt.cm.rainbow(i/self.n_controllers))
            self._line_.append(_line_)
        self._ax_.legend()
        self.draw()

        # Initiate the timer
        self._timer_ = self.new_timer(interval, [(self._update_canvas_, (), {})])
        self._timer_.start()
        return

    def _update_canvas_(self) -> None:
        '''
        This function gets called regularly by the timer.

        '''
        self._y_ = np.concatenate((self._y_[1:], np.array([self.get_next_datapoint()])), axis=0)
        self._ax_.draw_artist(self._ax_.patch)
        for i in range(len(self._line_)):
            self._line_[i].set_ydata(self._y_[:,i])
            self._ax_.draw_artist(self._line_[i])
        # draw the legend
        self._ax_.draw_artist(self._ax_.legend_.legendPatch)
        for i in range(len(self._line_)):
            self._ax_.draw_artist(self._ax_.legend_.texts[i])
            self._ax_.draw_artist(self._ax_.legend_.legendHandles[i])
        self.update()
        self.flush_events()
        return
    
    def get_next_datapoint(self) -> float:
        '''
        Get the next datapoint.

        '''
        # self.i += 1
        # if self.i > 499:
        #     self.i = 0
        # return self.d[self.i]
        next_datapoint = np.array([self.controller.get_flow_rate(i) for i in range(self.n_controllers)])
        timepoint = datetime.datetime.now()

        if self.save_directory != '':
            # check if mfc_data.csv exists in save_directory
            if os.path.isfile(self.save_directory + '/mfc_data.csv'):
                # append to existing file
                with open(self.save_directory + '/mfc_data.csv', 'a') as f:
                    f.write(str(timepoint) + ',' + ','.join([str(i) for i in next_datapoint]) + '\n')
            else:
                # create new file
                with open(self.save_directory + '/mfc_data.csv', 'w') as f:
                    f.write('timepoint' + ',' + ','.join([str(i) for i in range(len(next_datapoint))]) + '\n')
                    f.write(str(timepoint) + ',' + ','.join([str(i) for i in next_datapoint]) + '\n')
        
        return next_datapoint

def start_gui(MFCController=None,save_directory=''):
    '''
    Start the GUI.

    '''
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow(MFCController, save_directory)
    sys.exit(app.exec_())

with MFCController(
        com_port='COM8', 
        device_ids=[chr(i) for i in range(ord('A'), ord('A') + 16)], 
        default_flow_rate=300
    ) as mfc:
    start_gui(mfc, save_directory='Z:/Rishika/4Y-Maze/TurnerLab_Opto2AFC_16Y/sixteeny')