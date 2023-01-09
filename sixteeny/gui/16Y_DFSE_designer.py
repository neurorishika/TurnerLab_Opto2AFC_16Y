from email.policy import default
from PyQt5 import QtWidgets, QtCore, QtGui
import pandas as pd
import numpy as np
from scipy import stats
import json
import sys
import os


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, start_folder=None, load_file=None, *args, **kwargs):
        super().__init__()
        self.setWindowTitle("Deterministic FSE Designer")
        self.setGeometry(100, 100, 960, 320)

        # create a main layout
        self.main_layout = QtWidgets.QGridLayout()

        # store the start folder and load file
        self.start_folder = start_folder
        self.load_file = load_file

        # create a checkbox for L/R Randomization
        self.randomize_lr = QtWidgets.QCheckBox("Randomize L/R?")
        self.randomize_lr.setChecked(True)
        self.main_layout.addWidget(self.randomize_lr, 0, 0, 1, 2)

        # create a checkbox for Reciprocal Task
        self.reciprocal_task = QtWidgets.QCheckBox("Reciprocal Task?")
        self.reciprocal_task.setChecked(True)
        self.main_layout.addWidget(self.reciprocal_task, 0, 2, 1, 2)

        default_database = "C:\\16YArena\\TurnerLab_Opto2AFC_16Y\\sixteeny\\utils\\experimenter\\tasks.pkl"
        try:
            self.tasks = pd.read_pickle(default_database)
        except:
            self.tasks = None

        # create a text label, text box, and button to load task database
        self.task_database_label = QtWidgets.QLabel("Task Database")
        self.main_layout.addWidget(self.task_database_label, 1, 0, 1, 1)
        self.task_database_textbox = QtWidgets.QLineEdit()
        self.task_database_textbox.setText(default_database)
        self.task_database_button = QtWidgets.QPushButton("Reload")
        self.task_database_button.clicked.connect(self.reload_task_database)
        self.main_layout.addWidget(self.task_database_textbox, 1, 1, 1, 2)
        self.main_layout.addWidget(self.task_database_button, 1, 3, 1, 1)

        # create a text label, text box, and button to load task
        self.task_label = QtWidgets.QLabel("Task ID")
        self.main_layout.addWidget(self.task_label, 2, 0, 1, 1)
        self.task_textbox = QtWidgets.QLineEdit()
        self.task_textbox.setValidator(QtGui.QIntValidator())
        self.task_button = QtWidgets.QPushButton("Load")
        self.task_button.clicked.connect(self.load_task)
        self.main_layout.addWidget(self.task_textbox, 2, 1, 1, 2)
        self.main_layout.addWidget(self.task_button, 2, 3, 1, 1)

        # create a text label and uneditable text box for number of states
        self.num_states_label = QtWidgets.QLabel("Number of States")
        self.num_states_textbox = QtWidgets.QLineEdit()
        self.num_states_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.num_states_label, 3, 0)
        self.main_layout.addWidget(self.num_states_textbox, 3, 1)

        # create a text label and uneditable text box for state labels
        self.state_labels_label = QtWidgets.QLabel("State Labels")
        self.state_labels_textbox = QtWidgets.QLineEdit()
        self.state_labels_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.state_labels_label, 3, 2)
        self.main_layout.addWidget(self.state_labels_textbox, 3, 3)

        # create a text label and uneditable text box for state transitions
        self.state_transitions_label = QtWidgets.QLabel("State Transitions")
        self.state_transitions_textbox = QtWidgets.QLineEdit()
        self.state_transitions_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.state_transitions_label, 4, 0, 1,1)
        self.main_layout.addWidget(self.state_transitions_textbox, 4, 1, 1, 3)
        

        # create a text label, text box, and button to load stimulus for odor 1
        self.rewarded_label = QtWidgets.QLabel("Rewarded Stimulus")
        self.main_layout.addWidget(self.rewarded_label, 5, 0, 1, 1)
        self.rewarded_textbox = QtWidgets.QLineEdit()
        self.rewarded_button = QtWidgets.QPushButton("Load")
        self.rewarded_button.clicked.connect(self.load_rewarded)
        self.rewarded_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.rewarded_textbox, 5, 1, 1, 2)
        self.main_layout.addWidget(self.rewarded_button, 5, 3, 1, 1)

        # create a text label, text box, and button to load stimulus for odor 2
        self.unrewarded_label = QtWidgets.QLabel("Unrewarded Stimulus")
        self.main_layout.addWidget(self.unrewarded_label, 6, 0, 1, 1)
        self.unrewarded_textbox = QtWidgets.QLineEdit()
        self.unrewarded_button = QtWidgets.QPushButton("Load")
        self.unrewarded_button.clicked.connect(self.load_unrewarded)
        self.unrewarded_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.unrewarded_textbox, 6, 1, 1, 2)
        self.main_layout.addWidget(self.unrewarded_button, 6, 3, 1, 1)

        # create a text label, text box to enter number of N trials
        self.naive_trials_label = QtWidgets.QLabel("No. of Naive Trials")
        self.naive_trials_textbox = QtWidgets.QLineEdit()
        self.naive_trials_textbox.setText("20")
        self.naive_trials_textbox.setValidator(QtGui.QIntValidator())
        self.main_layout.addWidget(self.naive_trials_label, 7, 0, 1, 1)
        self.main_layout.addWidget(self.naive_trials_textbox, 7, 1, 1, 1)

        # create a text label, text box to enter number of experiment trials
        self.num_trials_label = QtWidgets.QLabel("No. of Experiment Trials")
        self.num_trials_textbox = QtWidgets.QLineEdit()
        self.num_trials_textbox.setText("180")
        self.num_trials_textbox.setValidator(QtGui.QIntValidator())
        self.main_layout.addWidget(self.num_trials_label, 7, 2, 1, 1)
        self.main_layout.addWidget(self.num_trials_textbox, 7, 3, 1, 1)

        # create a button to save the experiment
        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.clicked.connect(self.save)
        self.main_layout.addWidget(self.save_button, 8, 0, 1, 2)

        # create a button to load the experiment
        self.load_button = QtWidgets.QPushButton("Load")
        self.load_button.clicked.connect(self.load)
        self.main_layout.addWidget(self.load_button, 8, 2, 1, 2)

        # create the central widget
        self.central_widget = QtWidgets.QWidget()
        self.central_widget.setLayout(self.main_layout)

        # set the central widget
        self.setCentralWidget(self.central_widget)

        # show the window
        self.show()

        # if a load file was specified, load it
        if self.load_file is not None:
            self.load(self.load_file)

    def load_rewarded(self):
        """
        Load the stimulus for odor 1.
        """
        # open a file dialog
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", self.start_folder, "*.stim")
        # if a file was selected
        if filename:
            # set the textbox to the file name (without the path)
            self.rewarded_textbox.setText(filename.split("/")[-1])

    def load_unrewarded(self):
        """
        Load the stimulus for odor 2.
        """
        # open a file dialog
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", self.start_folder, "*.stim")
        # if a file was selected
        if filename:
            # set the textbox to the file name (without the path)
            self.unrewarded_textbox.setText(filename.split("/")[-1])

    def reload_task_database(self):
        """
        Reload the task database.
        """
        # get the file name from the textbox
        filename = self.task_database_textbox.text()
        # if the file exists
        if os.path.exists(filename):
            # load the task database
            self.tasks = pd.read_pickle(filename)
        else:
            # show an error message
            QtWidgets.QMessageBox.critical(self, "Error", "The task database file does not exist.")

    def load_task(self):
        """
        Load a task from the database.
        """
        # make sure the task database has been loaded
        if self.tasks is None:
            # show an error message
            QtWidgets.QMessageBox.critical(self, "Error", "The task database has not been loaded.")
            return
        
        try:
            self.task_id = int(self.task_textbox.text())
        except ValueError:
            # show an error message
            QtWidgets.QMessageBox.critical(self, "Error", "The task id must be an integer.")
            return
        
        # if the task id is valid
        if self.task_id in self.tasks.index:
            # get the task
            self.task = self.tasks.loc[self.task_id]['program']
            # set the text boxes
            self.num_states = len(self.task[0])
            self.num_states_textbox.setText(str(self.num_states))
            self.state_labels = self.task[0]
            self.state_labels_textbox.setText("["+",".join([str(i) for i in self.state_labels])+"]")
            self.state_transitions = self.task[1]
            print(self.state_transitions.shape)
            # convert 2D array to a string
            state_transitions_str = "["
            for i in range(self.num_states):
                state_transitions_str += "["+",".join([str(j) for j in self.state_transitions[i]])+"],"
            state_transitions_str = state_transitions_str[:-1]+"]"
            self.state_transitions_textbox.setText(state_transitions_str)
        
    def save(self):
        """
        Convert data into a dictionary and save it to a JSON file.
        """
        
        # save the dataframe to a csv file
        if self.start_folder != "":
            default_folder = self.start_folder
        else:
            default_folder = "."

        if self.load_file is not None:
            filename = self.load_file
        else:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Experiment", default_folder, "*.yfse")

        if filename:
            # create a dictionary of the experiment parameters
            experiment = {
                "num_trials": int(self.num_trials_textbox.text()),
                "num_naive_trials": int(self.naive_trials_textbox.text()),
                "num_states": int(self.num_states_textbox.text()),
                "state_labels": self.state_labels,
                "state_transitions": self.state_transitions.tolist(),
                "rewarded_stimulus": self.rewarded_textbox.text(),
                "unrewarded_stimulus": self.unrewarded_textbox.text(),
                "task_id": self.task_id,
                "task_database": self.task_database_textbox.text(),
                "reciprocal_task": self.reciprocal_task.isChecked(),
                "lr_randomization": self.randomize_lr.isChecked(),
            }
            # save the experiment dictionary to a json file
            with open(filename, "w") as f:
                json.dump(experiment, f)
            
            if self.reciprocal_task.isChecked():
                print()
                experiment["state_transitions"] = (np.array(experiment["state_transitions"])[:,::-1]).tolist()
                with open(filename[:-5]+"_reciprocal.yfse", "w") as f:
                    json.dump(experiment, f)
                
            # show a message box indicating that the experiment was saved
            QtWidgets.QMessageBox.information(
                self, "Success", "The experiment was saved to {}".format(filename),
            )

        # create a dictionary of the used stimuli
        stimuli = {"used_stimuli": [self.rewarded_textbox.text(), self.unrewarded_textbox.text(),]}
        # save the stimuli dictionary to a json file with .meta extension with the same name as the experiment file
        filename = filename.split(".")[0] + ".meta"
        with open(filename, "w") as f:
            json.dump(stimuli, f)
        if self.reciprocal_task.isChecked():
            with open(filename.replace(".meta", "_reciprocal.meta"), "w") as f:
                json.dump(stimuli, f)

        # ask the user if they want to close the window
        reply = QtWidgets.QMessageBox.question(
            self,
            "Close",
            "Do you want to close the Discrete Finite State Experiment Designer?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close()

    def load(self, load_file=None):
        """
        Load an experiment from a yfse file.
        """
        # if a file was passed as an argument, load it
        if load_file != False and load_file is not None:
            filename = load_file
            print(load_file)
        # otherwise, open a file dialog
        else:
            if self.start_folder != "":
                default_folder = self.start_folder
            else:
                default_folder = "."

            filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Experiment", default_folder, "*.yfse")
        
        if filename:
            # load the experiment dictionary from the json file
            with open(filename, "r") as f:
                experiment = json.load(f)
            
            # set the text boxes
            self.num_trials_textbox.setText(str(experiment["num_trials"]))
            self.naive_trials_textbox.setText(str(experiment["num_naive_trials"]))
            self.num_states_textbox.setText(str(experiment["num_states"]))
            self.state_labels = experiment["state_labels"]
            self.state_labels_textbox.setText("["+",".join([str(i) for i in self.state_labels])+"]")
            self.state_transitions = np.array(experiment["state_transitions"])
            # convert 2D array to a string
            state_transitions_str = "["
            for i in range(self.state_transitions.shape[0]):
                state_transitions_str += "["+",".join([str(j) for j in self.state_transitions[i]])+"],"
            state_transitions_str = state_transitions_str[:-1]+"]"
            self.state_transitions_textbox.setText(state_transitions_str)
            self.rewarded_textbox.setText(experiment["rewarded_stimulus"])
            self.unrewarded_textbox.setText(experiment["unrewarded_stimulus"])
            self.task_id = experiment["task_id"]
            self.task_id_textbox.setText(experiment["task_id"])
            self.task_database_textbox.setText(experiment["task_database"])
            self.reciprocal_task.setChecked(experiment["reciprocal_task"])
            self.randomize_lr.setChecked(experiment["lr_randomization"])

            # reload the database
            self.reload_task_database()
        

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    if len(sys.argv) > 1:
        # if a file was passed as an argument, create a new window with the load file
        if os.path.isfile(sys.argv[1]):
            window = MainWindow(start_folder=None, load_file=sys.argv[1])
        else:
            window = MainWindow(start_folder=sys.argv[1], load_file=None)
    else:
        window = MainWindow()
    app.exec_()
