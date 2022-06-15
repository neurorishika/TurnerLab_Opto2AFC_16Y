import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
import os
import sys
import shutil
import json
import subprocess


class MainWindow(QtWidgets.QMainWindow):
    """
    A class for the main window of the 16-Y experimenter interface.
    """

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # set the window title
        self.setWindowTitle("16-Y Experimenter")

        # create a the main layout
        layout = QtWidgets.QGridLayout()

        # add a text box for getting the project directory with a browse button
        layout.addWidget(QtWidgets.QLabel("Project Directory"), 0, 0)
        self.project_directory = QtWidgets.QLineEdit()
        self.project_directory.setText("")
        self.project_directory.setReadOnly(True)
        self.browse_button = QtWidgets.QPushButton("Browse")
        layout.addWidget(self.project_directory, 0, 1)
        layout.addWidget(self.browse_button, 0, 2)

        # add a text box for entering the experiment name
        layout.addWidget(QtWidgets.QLabel("Experiment Name"), 1, 0)
        self.experiment_name = QtWidgets.QLineEdit()
        self.experiment_name.setText("")
        layout.addWidget(self.experiment_name, 1, 1, 1, 2)

        # add a text box for entering the fly genotype
        layout.addWidget(QtWidgets.QLabel("Fly Genotype"), 2, 0)
        self.fly_genotype = QtWidgets.QLineEdit()
        self.fly_genotype.setText("+/+")
        layout.addWidget(self.fly_genotype, 2, 1)

        # add a button to copy the fly genotype to all the arenas
        self.copy_genotype_button = QtWidgets.QPushButton("Copy to all Arenas")
        layout.addWidget(self.copy_genotype_button, 2, 2)
        self.copy_genotype_button.clicked.connect(self.copy_genotype)

        # add a centre-aligned label at the top of the grid
        layout.addWidget(
            QtWidgets.QLabel("Assign Experiment for each Y-Arena below:"), 3, 0, 1, 3, QtCore.Qt.AlignCenter
        )
        # set fixed size for the label
        layout.setRowMinimumHeight(3, 30)

        # Get the experiments from the directory
        self.experiments = []

        # add a sub-layout for the dropboxes
        dropbox_layout = QtWidgets.QGridLayout()

        # Fly Numbers
        self.fly_numbers = [4, 5, 14, 12, 6, 7, 15, 13, 1, 3, 11, 10, 0, 2, 9, 8]

        # define toggle state for all arenas
        self.toggle_state = False

        # add a checkbox and a dropbox side by side in each cell of the grid and a genotype box
        self.checkboxes = []
        self.dropboxes = []
        self.textboxes = []
        for i in range(4):
            for j in range(4):
                # add a label for the fly identifier
                label = QtWidgets.QLabel("Fly {}".format(self.fly_numbers[i * 4 + j]))

                # add a checkbox with a toggle function that takes in the index of the checkbox
                checkbox = QtWidgets.QCheckBox()
                checkbox.stateChanged.connect(lambda state, index=i * 4 + j: self.toggle_checkbox(index))

                # add a dropbox with a list of experiments
                dropbox = QtWidgets.QComboBox()
                dropbox.addItems(self.experiments)
                dropbox.setCurrentIndex(0)
                dropbox.setEnabled(False)

                # add a textbox for the genotype
                textbox = QtWidgets.QLineEdit()
                textbox.setText("+/+")
                textbox.setEnabled(False)

                self.checkboxes.append(checkbox)
                self.dropboxes.append(dropbox)
                self.textboxes.append(textbox)

                dropbox_layout.addWidget(label, 2 * i, 3 * j)
                dropbox_layout.addWidget(checkbox, 2 * i, 3 * j + 1)
                dropbox_layout.addWidget(dropbox, 2 * i, 3 * j + 2)
                dropbox_layout.addWidget(textbox, 2 * i + 1, 3 * j, 1, 3)

        # add the dropbox layout to the grid
        layout.addLayout(dropbox_layout, 4, 0, 1, 3)

        self.browse_button.clicked.connect(self.browse_directory)

        # add a text box for inputting any comments
        layout.addWidget(QtWidgets.QLabel("Comments"), 5, 0)
        self.comments = QtWidgets.QTextEdit()
        self.comments.setFixedHeight(50)
        layout.addWidget(self.comments, 5, 1, 1, 2)

        # add buttons to toggle all arenas and start experiment
        self.toggle_all_arenas_button = QtWidgets.QPushButton("Toggle All Arenas")
        self.toggle_all_arenas_button.clicked.connect(self.toggle_all_arenas)
        layout.addWidget(self.toggle_all_arenas_button, 6, 0)

        button = QtWidgets.QPushButton("Start Experiment")
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
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Directory")
        self.project_directory.setText(directory)
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

    def get_experiments_from_directory(self, directory):
        """
        A method to get the experiments from the directory.

        Variables:
            directory (str): The directory to get the experiments from.
        """
        # check if the directory exists
        if not os.path.exists(directory + "/experiment_zoo"):
            experiments = []
        else:
            experiments = os.listdir(directory + "/experiment_zoo")
            # filter out the non-experiment files
            experiments = [
                experiment for experiment in experiments if experiment.endswith(".csv") or experiment.endswith(".py")
            ]

        # if there are no experiments, send an alert dialog
        if len(experiments) == 0:
            QtWidgets.QMessageBox.about(self, "No Experiments", "There are no experiments in the directory.")

        return experiments

    def verify_start_experiment(self):
        """
        A method to verify the start of the experiment.
        """
        # check if any experiments are loaded
        if len(self.experiments) == 0:
            # show a warning message that no experiments are loaded
            QtWidgets.QMessageBox.warning(self, "No Experiments Loaded", "Please load an project directory.")
            return False

        # check if at least one arena is checked
        self.active_arenas = []
        for i in range(16):
            if self.checkboxes[i].isChecked():
                # add the experiment to the active arenas
                self.active_arenas.append(i)

        if len(self.active_arenas) == 0:
            # show an warning message box to inform the user to activate at least one arena
            QtWidgets.QMessageBox.warning(self, "Error", "Please activate at least one arena.")
            return False

        # check if the project directory already has a folder for the experiment name
        experiment_name = "data/" + self.experiment_name.text()
        project_directory = os.path.join(self.project_directory.text(), experiment_name).replace("\\", "/")

        if os.path.isdir(os.path.join(self.project_directory.text(), experiment_name)):
            # show a warning message and ask the user if they want to overwrite the experiment
            reply = QtWidgets.QMessageBox.question(
                self,
                "Overwrite Experiment",
                "The project directory already has a folder for the experiment name. Do you want to clear the data and reuse configuration?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if reply == QtWidgets.QMessageBox.Yes:
                # remove files in all the directories and subdirectories
                for root, dirs, files in os.walk(project_directory):
                    for file in files:
                        if file == "config.yarena":
                            # skip the file
                            continue
                        os.remove(os.path.join(root, file))
            else:
                # ask the user if they want to continue
                reply = QtWidgets.QMessageBox.question(
                    self,
                    "Continue",
                    "Proceeding will overwrite the experiment. Do you want to continue?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No,
                )
                if reply == QtWidgets.QMessageBox.No:
                    # show a warning message that the experiment was not started
                    QtWidgets.QMessageBox.warning(self, "Error", "The experiment was not started.")
                    # return if the user does not want to continue
                    return False
                else:
                    # remove all the files in the experiment directory
                    for root, dirs, files in os.walk(project_directory):
                        for file in files:
                            os.remove(os.path.join(root, file))
        else:
            # create the experiment folder
            os.mkdir(project_directory)

        # check if there is a config.yarena file in the experiment name directory
        config_present = False
        while not config_present:
            files = os.listdir(os.path.join(self.project_directory.text(), experiment_name))
            if "config.yarena" not in files:
                # show a warning message and ask the user if they want to create a rig configuration file
                reply = QtWidgets.QMessageBox.question(
                    self,
                    "Create Rig Configuration File",
                    "The project directory does not have a rig configuration file. Do you want to create one?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.Yes,
                )
                if reply == QtWidgets.QMessageBox.No:
                    return False
                else:
                    # run the rig configurator script
                    os.system(
                        "python sixteeny/gui/rig_configurator.py "
                        + os.path.join(self.project_directory.text(), experiment_name)
                    )
            else:
                # show a message that the rig configuration file is present
                QtWidgets.QMessageBox.information(
                    self, "Rig Configuration File Present", "The rig configuration file is present."
                )
                # set the config_present flag
                config_present = True
                # set the config file path
                self.config_file = os.path.join(
                    self.project_directory.text(),
                    experiment_name,
                    [file for file in files if file.endswith(".yarena")][0],
                )

        return True

    def save_experiment(self):
        """
        A method to save the experiment details for each fly in an individual *.yexperiment file as a json
        """

        # get active arenas
        self.active_arenas = []
        for i in range(16):
            if self.checkboxes[i].isChecked():
                self.active_arenas.append(self.fly_numbers[i])

                # create a dictionary to store the experiment details
                experiment_details = {}
                experiment_details["fly_experiment_name"] = "data/" + self.experiment_name.text()
                experiment_details["fly_project_directory"] = self.project_directory.text()
                experiment_details["fly_config_file"] = self.config_file
                experiment_details["fly_experiment_start_time"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                experiment_details["fly_genotype"] = self.textboxes[i].text()
                experiment_details["fly_experiment"] = self.dropboxes[i].currentText()
                experiment_details["fly_arena"] = self.fly_numbers[i]
                # save the experiment details for each fly
                with open(
                    os.path.join(
                        self.project_directory.text(),
                        "data/" + self.experiment_name.text(),
                        "fly_" + str(self.fly_numbers[i]) + ".yexperiment",
                    ),
                    "w",
                ) as f:
                    json.dump(experiment_details, f)

    def prepare_directory(self):
        """
        A method to prepare the experiment directory.
        """
        # get used experiment names
        used_experiments = [self.dropboxes[i].currentText() for i in range(16) if self.checkboxes[i].isChecked()]

        # copy the used experiments from experiment_zoo to a new experiments folder in the experiment directory
        for experiment in used_experiments:
            # check if the experiments folder already exists
            if os.path.isdir(
                os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments")
            ):
                # remove the experiments folder
                shutil.rmtree(
                    os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments")
                )
            # create the experiments folder
            os.mkdir(os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments"))
            # copy the experiment file from the experiment_zoo to the experiments folder overwriting the old experiment
            shutil.copy(
                os.path.join(self.project_directory.text(), "experiment_zoo", experiment),
                os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments"),
            )

        # look for the metadata file for the used experiments
        for experiment in used_experiments:
            # check if the metadata file exists
            if os.path.isfile(
                os.path.join(self.project_directory.text(), "experiment_zoo", experiment.split(".")[0] + ".meta")
            ):
                # copy the metadata file to the experiment directory
                shutil.copy(
                    os.path.join(self.project_directory.text(), "experiment_zoo", experiment.split(".")[0] + ".meta"),
                    os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments"),
                )
                # open the metadata file as a json
                with open(
                    os.path.join(
                        self.project_directory.text(),
                        "data/" + self.experiment_name.text(),
                        "experiments",
                        experiment.split(".")[0] + ".meta",
                    ),
                    "r",
                ) as f:
                    metadata = json.load(f)

                # check if the metadata file has the 'used_stimuli' key
                if "used_stimuli" not in metadata:
                    # send a warning message saying that the metadata file is invalid
                    QtWidgets.QMessageBox.warning(
                        self,
                        "Invalid Metadata File",
                        'The metadata file for the experiment: "' + experiment + '" is invalid.',
                    )
                    return False
                else:
                    # check if the stimuli folder already exists
                    if os.path.isdir(
                        os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli")
                    ):
                        # remove the stimuli folder
                        shutil.rmtree(
                            os.path.join(
                                self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli"
                            )
                        )
                    # create the stimuli folder
                    os.mkdir(
                        os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli")
                    )
                    # copy the used stimuli from the stimulus_zoo to a new stimuli folder in the experiment directory
                    for stimulus in metadata["used_stimuli"]:
                        # check if the stimulus_zoo has the stimulus file
                        if os.path.isfile(os.path.join(self.project_directory.text(), "stimulus_zoo", stimulus)):
                            # copy the stimulus file from the stimulus_zoo to the stimuli folder
                            shutil.copy(
                                os.path.join(self.project_directory.text(), "stimulus_zoo", stimulus),
                                os.path.join(
                                    self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli"
                                ),
                            )
                        else:
                            # send a warning message saying that the stimulus is missing
                            QtWidgets.QMessageBox.warning(
                                self, "Missing Stimulus", 'The stimulus: "' + stimulus + '" is missing.'
                            )
                            return False
        return True

    def start_experiment(self):
        """
        A method to start the experiment.
        """
        # verify the start of the experiment
        if not self.verify_start_experiment():
            return

        # save the experiment
        self.save_experiment()

        # prepare the experiment directory
        if not self.prepare_directory():
            return

        print("Start experiment")

        # run main.py
        subprocess.Popen(
            ["python", "sixteeny/main.py", self.project_directory.text(), "data/" + self.experiment_name.text()]
        )


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
