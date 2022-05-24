from PyQt5 import QtCore, QtGui, QtWidgets
import os
import sys

class MainWindow(QtWidgets.QMainWindow):
    """
    A class for the main window of the 16-Y experimenter interface.
    """
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # set the window title
        self.setWindowTitle('16-Y Experimenter')

        # create a the main layout
        layout = QtWidgets.QGridLayout()

        # add a text box for getting the experiment directory with a browse button
        layout.addWidget(QtWidgets.QLabel('Experiment Directory'), 0, 0)
        self.experiment_directory = QtWidgets.QLineEdit()
        self.experiment_directory.setText('')
        self.browse_button = QtWidgets.QPushButton('Browse')
        layout.addWidget(self.experiment_directory, 0, 1)
        layout.addWidget(self.browse_button, 0, 2)

        # add a text box for entering the experiment name
        layout.addWidget(QtWidgets.QLabel('Experiment Name'), 1, 0)
        self.experiment_name = QtWidgets.QLineEdit()
        self.experiment_name.setText('')
        layout.addWidget(self.experiment_name, 1, 1, 1, 2)

        # add a text box for entering the fly genotype
        layout.addWidget(QtWidgets.QLabel('Fly Genotype'), 2, 0)
        self.fly_genotype = QtWidgets.QLineEdit()
        self.fly_genotype.setText('+/+')
        layout.addWidget(self.fly_genotype, 2, 1)

        # add a button to copy the fly genotype to all the arenas
        self.copy_genotype_button = QtWidgets.QPushButton('Copy to all Arenas')
        layout.addWidget(self.copy_genotype_button, 2, 2)
        self.copy_genotype_button.clicked.connect(self.copy_genotype)

        # add a centre-aligned label at the top of the grid
        layout.addWidget(QtWidgets.QLabel('Assign Experiment for each Y-Arena below:'), 3, 0, 1, 3, QtCore.Qt.AlignCenter)
        # set fixed size for the label
        layout.setRowMinimumHeight(3, 30)

        # Get the experiments from the directory
        self.experiments = []

        # add a sub-layout for the dropboxes
        dropbox_layout = QtWidgets.QGridLayout()

        # Fly Numbers
        fly_numbers = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

        # define toggle state for all arenas
        self.toggle_state = False   

        # add a checkbox and a dropbox side by side in each cell of the grid and a genotype box
        self.checkboxes = []
        self.dropboxes = []
        self.textboxes = []
        for i in range(4):
            for j in range(4):
                # add a label for the fly identifier
                label = QtWidgets.QLabel('Fly {}'.format(fly_numbers[i*4+j]))
                
                # add a checkbox with a toggle function that takes in the index of the checkbox
                checkbox = QtWidgets.QCheckBox()
                checkbox.stateChanged.connect(lambda state, index=i*4+j: self.toggle_checkbox(index))
                
                # add a dropbox with a list of experiments
                dropbox = QtWidgets.QComboBox()
                dropbox.addItems(self.experiments)
                dropbox.setCurrentIndex(0)
                dropbox.setEnabled(False)
                
                # add a textbox for the genotype
                textbox = QtWidgets.QLineEdit()
                textbox.setText('+/+')
                textbox.setEnabled(False)
                
                self.checkboxes.append(checkbox)
                self.dropboxes.append(dropbox)
                self.textboxes.append(textbox)

                dropbox_layout.addWidget(label, 2*i, 3*j)
                dropbox_layout.addWidget(checkbox, 2*i, 3*j+1)
                dropbox_layout.addWidget(dropbox, 2*i, 3*j+2)
                dropbox_layout.addWidget(textbox, 2*i+1, 3*j, 1, 3)

        # add the dropbox layout to the grid
        layout.addLayout(dropbox_layout, 4, 0, 1, 3)

        self.browse_button.clicked.connect(self.browse_directory)

        # add a text box for inputting any comments
        layout.addWidget(QtWidgets.QLabel('Comments'), 5, 0)
        self.comments = QtWidgets.QTextEdit()
        self.comments.setFixedHeight(50)
        layout.addWidget(self.comments, 5, 1, 1, 2)

        # add buttons to toggle all arenas and start experiment
        self.toggle_all_arenas_button = QtWidgets.QPushButton('Toggle All Arenas')
        self.toggle_all_arenas_button.clicked.connect(self.toggle_all_arenas)
        layout.addWidget(self.toggle_all_arenas_button, 6, 0)

        button = QtWidgets.QPushButton('Start Experiment')
        button.clicked.connect(self.start_experiment)
        layout.addWidget(button, 6, 1, 1, 2)

        # create the main widget
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)

        # set the central widget
        self.setCentralWidget(widget)

        # show the window
        self.show()

    def browse_directory(self):
        """
        A method to browse the directory.
        """
        # get the directory
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, 'Select Directory')
        self.experiment_directory.setText(directory)
        self.experiments = self.get_experiments_from_directory(directory)
        # update the dropboxes with the experiments
        self.update_dropboxes()

    def update_dropboxes(self):
        """
        A method to update the dropboxes.
        """
        for i in range(16):
            self.dropboxes[i].clear()
            self.dropboxes[i].addItems(self.experiments)
            self.dropboxes[i].setCurrentIndex(0)
    
    def toggle_checkbox(self, index):
        """
        A method to toggle the checkbox.

        Variables:
            index (int): The index of the checkbox.
        """
        if self.checkboxes[index].isChecked():
            self.dropboxes[index].setEnabled(True)
            self.textboxes[index].setEnabled(True)
        else:
            self.dropboxes[index].setEnabled(False)
            self.textboxes[index].setEnabled(False)
    
    def toggle_all_arenas(self):
        """
        A method to toggle all the arenas.
        """
        for i in range(16):
            if self.toggle_state:
                self.checkboxes[i].setChecked(False)
                self.dropboxes[i].setEnabled(False)
                self.textboxes[i].setEnabled(False)
            else:
                self.checkboxes[i].setChecked(True)
                self.dropboxes[i].setEnabled(True)
                self.textboxes[i].setEnabled(True)
        self.toggle_state = not self.toggle_state

    def copy_genotype(self):
        """
        A method to copy the genotype to all the arenas.
        """
        for i in range(16):
            self.textboxes[i].setText(self.fly_genotype.text())

    def verify_start_experiment(self):
        """
        A method to verify the start of the experiment.
        """
        # check if any experiments are loaded
        if len(self.experiments) == 0:
            # show a warning message that no experiments are loaded
            QtWidgets.QMessageBox.warning(self, 'No Experiments Loaded', 'Please load an experiment directory.')
            return False
        
        # check if at least one arena is checked
        self.active_arenas = []
        for i in range(16):
            if self.checkboxes[i].isChecked():
                # add the experiment to the active arenas
                self.active_arenas.append(i)
        
        if len(self.active_arenas) == 0:
            # show an warning message box to inform the user to activate at least one arena
            QtWidgets.QMessageBox.warning(self, 'Error', 'Please activate at least one arena.')
            return False
        else:
            return True
    
    def get_experiments_from_directory(self, directory):
        """
        A method to get the experiments from the directory.

        Variables:
            directory (str): The directory to get the experiments from.
        """
        # check if the directory exists
        if not os.path.exists(directory+'/experiments'):
            experiments = []
        else:
            experiments = os.listdir(directory+'/experiments')
            # filter out the non-experiment files
            experiments = [experiment for experiment in experiments if experiment.endswith('.csv')]
        
        # if there are no experiments, send an alert dialog
        if len(experiments) == 0:
            QtWidgets.QMessageBox.about(self, 'No Experiments', 'There are no experiments in the directory.')
        
        return experiments

    def start_experiment(self):
        """
        A method to start the experiment.
        """
        # verify the start of the experiment
        if not self.verify_start_experiment():
            return
        
        print('Start experiment')




if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())