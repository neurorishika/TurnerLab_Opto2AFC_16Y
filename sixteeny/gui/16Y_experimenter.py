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

        # set the window size
        self.setFixedSize(1800, 1200)

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
        layout.addWidget(self.experiment_name, 1, 1)

        # add a button to append timestamp to the experiment name
        self.append_timestamp_button = QtWidgets.QPushButton("Add Timestamp")
        layout.addWidget(self.append_timestamp_button, 1, 2)
        self.append_timestamp_button.clicked.connect(self.append_timestamp)

        # add a sub-layout for the fly genotypes
        fly_genotype_layout = QtWidgets.QGridLayout()

        # add a dropbox for entering the fly genotype
        self.genotypes = ['custom']
        layout.addWidget(QtWidgets.QLabel("Fly Genotype"), 2, 0)
        self.fly_genotype = QtWidgets.QComboBox()
        self.fly_genotype.setEditable(True)
        self.fly_genotype.addItems(self.genotypes)
        self.fly_genotype.setCurrentIndex(0)
        self.fly_genotype.setMinimumWidth(400)
        fly_genotype_layout.addWidget(self.fly_genotype, 0, 0)

        # call function on change of fly genotype
        self.fly_genotype.currentIndexChanged.connect(self.fly_genotype_changed)

        # add a text box for entering custom fly genotype
        self.custom_fly_genotype = QtWidgets.QLineEdit()
        self.custom_fly_genotype.setText("")
        fly_genotype_layout.addWidget(self.custom_fly_genotype, 0, 1)

        layout.addLayout(fly_genotype_layout, 2, 1)

        # add a button to copy the fly genotype to all the arenas
        self.copy_genotype_button = QtWidgets.QPushButton("Copy Genotypes")
        layout.addWidget(self.copy_genotype_button, 2, 2)
        self.copy_genotype_button.clicked.connect(self.copy_genotype)

        # add a widget for the starvation start date and time
        layout.addWidget(QtWidgets.QLabel("Starvation Start"), 3, 0)

        # create a sub-layout for the starvation start date and time, quick buttons, and copy button
        starvation_start_layout = QtWidgets.QGridLayout()

        # add a button for 24 hours, 36 hours, 48 hours, 60 hours, 72 hours
        self.starvation_24_button = QtWidgets.QPushButton("24 hours")
        starvation_start_layout.addWidget(self.starvation_24_button, 0, 0)
        self.starvation_24_button.clicked.connect(self.starvation_24)

        self.starvation_36_button = QtWidgets.QPushButton("36 hours")
        starvation_start_layout.addWidget(self.starvation_36_button, 0, 1)
        self.starvation_36_button.clicked.connect(self.starvation_36)

        self.starvation_48_button = QtWidgets.QPushButton("48 hours")
        starvation_start_layout.addWidget(self.starvation_48_button, 0, 2)
        self.starvation_48_button.clicked.connect(self.starvation_48)

        self.starvation_60_button = QtWidgets.QPushButton("60 hours")
        starvation_start_layout.addWidget(self.starvation_60_button, 0, 3)
        self.starvation_60_button.clicked.connect(self.starvation_60)

        self.starvation_72_button = QtWidgets.QPushButton("72 hours")
        starvation_start_layout.addWidget(self.starvation_72_button, 0, 4)
        self.starvation_72_button.clicked.connect(self.starvation_72)

        self.starvation_period = QtWidgets.QLineEdit()
        self.starvation_period.setValidator(QtGui.QDoubleValidator())
        self.starvation_period.setText("24.0")
        starvation_start_layout.addWidget(self.starvation_period, 0, 5)

        # add a button to update the starvation start to the current time - starvation period
        self.starvation_start_now_button = QtWidgets.QPushButton("Update")
        starvation_start_layout.addWidget(self.starvation_start_now_button, 0, 6)
        self.starvation_start_now_button.clicked.connect(self.starvation_start_now)

        self.starvation_start = QtWidgets.QDateTimeEdit()
        self.starvation_start.setCalendarPopup(True)
        self.starvation_start.setDateTime(self.round_to_30mins(QtCore.QDateTime.currentDateTime()))
        starvation_start_layout.addWidget(self.starvation_start, 0, 7)

        
        # add a button to copy the starvation start to all the arenas
        self.copy_starvation_start_button = QtWidgets.QPushButton("Copy Starvation Starts")
        starvation_start_layout.addWidget(self.copy_starvation_start_button, 0, 8)
        self.copy_starvation_start_button.clicked.connect(self.copy_starvation_start)

        # add the sub-layout to the main layout
        layout.addLayout(starvation_start_layout, 3, 1, 1, 2)

        # add a text box for entering comments
        layout.addWidget(QtWidgets.QLabel("Comments"), 4, 0)
        self.comments = QtWidgets.QLineEdit()
        self.comments.setText("")
        layout.addWidget(self.comments, 4, 1)

        # add a button to copy the comments to all the arenas
        self.copy_comments_button = QtWidgets.QPushButton("Copy Comments")
        layout.addWidget(self.copy_comments_button, 4, 2)
        self.copy_comments_button.clicked.connect(self.copy_comments)

        # Get the experiments from the directory
        self.experiments = []

        # add a dropdown for selecting the experiment
        layout.addWidget(QtWidgets.QLabel("Experiment"), 5, 0)
        self.experiment_dropdown = QtWidgets.QComboBox()
        self.experiment_dropdown.addItems(self.experiments)
        self.experiment_dropdown.setCurrentIndex(0)
        layout.addWidget(self.experiment_dropdown, 5, 1)

        # add a button to copy the experiment to all the arenas
        self.copy_experiment_button = QtWidgets.QPushButton("Copy Experiments")
        layout.addWidget(self.copy_experiment_button, 5, 2)
        self.copy_experiment_button.clicked.connect(self.copy_experiment)

        # add a text box for entering number of repeats
        layout.addWidget(QtWidgets.QLabel("Repeats"), 6, 0)
        self.repeats = QtWidgets.QLineEdit()
        self.repeats.setText("3")
        self.repeats.setValidator(QtGui.QIntValidator())
        layout.addWidget(self.repeats, 6, 1)

        # add a button to auto-populate the repeats
        self.auto_populate_repeats_button = QtWidgets.QPushButton("Auto-Populate Experiments")
        layout.addWidget(self.auto_populate_repeats_button, 6, 2)
        self.auto_populate_repeats_button.clicked.connect(self.auto_populate_repeats)

        # add a button to auto-populate the repeats
        self.auto_populate_exps_button = QtWidgets.QPushButton("Auto-Populate Experiments from File")
        layout.addWidget(self.auto_populate_exps_button, 7, 0, 1, 3)
        self.auto_populate_exps_button.clicked.connect(self.auto_populate_exps)

        # add a centre-aligned label at the top of the grid
        layout.addWidget(
            QtWidgets.QLabel("Assign Experiment for each Y-Arena below:"), 8, 0, 1, 3, QtCore.Qt.AlignCenter
        )
        # set fixed size for the label
        layout.setRowMinimumHeight(3, 30)

        # add a sub-layout for the dropboxes
        dropbox_layout = QtWidgets.QGridLayout()

        # Fly Numbers
        self.fly_numbers = [4, 5, 14, 12, 6, 7, 15, 13, 1, 3, 11, 10, 0, 2, 9, 8]

        # define toggle state for all arenas
        self.toggle_state = False

        # add a checkbox and a dropbox side by side in each cell of the grid and a genotype box
        self.checkboxes = []
        self.dropboxes = []
        self.textboxes_1 = []
        self.textboxes_2 = []
        self.datetimes = []
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
                # increase the size of the dropbox
                dropbox.setMinimumSize(300, 0)

                # add a textbox for the genotype
                textbox1 = QtWidgets.QLineEdit()
                textbox1.setText("+/+")
                textbox1.setEnabled(False)

                # add a textbox for the comments
                textbox2 = QtWidgets.QLineEdit()
                textbox2.setText("")
                textbox2.setEnabled(False)

                # add a datetime for the starvation start
                datetime = QtWidgets.QDateTimeEdit()
                datetime.setDateTime(self.round_to_30mins(QtCore.QDateTime.currentDateTime().addSecs(-3600)))
                datetime.setEnabled(False)

                self.checkboxes.append(checkbox)
                self.dropboxes.append(dropbox)
                self.textboxes_1.append(textbox1)
                self.textboxes_2.append(textbox2)
                self.datetimes.append(datetime)

                dropbox_layout.addWidget(label, 4 * i, 3 * j)
                dropbox_layout.addWidget(checkbox, 4 * i, 3 * j + 1)
                dropbox_layout.addWidget(dropbox, 4 * i, 3 * j + 2)
                dropbox_layout.addWidget(textbox1, 4 * i + 1, 3 * j, 1, 3)
                dropbox_layout.addWidget(textbox2, 4 * i + 2, 3 * j, 1, 3)
                dropbox_layout.addWidget(datetime, 4 * i + 3, 3 * j, 1, 3)

        # add the dropbox layout to the grid
        layout.addLayout(dropbox_layout, 9, 0, 1, 3)

        self.browse_button.clicked.connect(self.browse_directory)

        # add buttons to toggle all arenas and start experiment
        self.toggle_all_arenas_button = QtWidgets.QPushButton("Toggle All Arenas")
        self.toggle_all_arenas_button.clicked.connect(self.toggle_all_arenas)
        layout.addWidget(self.toggle_all_arenas_button, 10, 0)

        button = QtWidgets.QPushButton("Start Experiment")
        button.clicked.connect(self.start_experiment)
        layout.addWidget(button, 10, 1, 1, 2)

        # create the main widget
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)

        # set the central widget
        self.setCentralWidget(widget)

        # show the window
        self.show()

    def append_timestamp(self):
        """
        Append a timestamp to the experiment name
        """
        self.experiment_name.setText(
            self.experiment_name.text() + "_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        )

    def browse_directory(self):
        """
        A method to browse the directory.
        """
        # get the directory
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Directory")
        self.project_directory.setText(directory)
        self.experiments = self.get_experiments_from_directory(directory)
        self.genotypes = self.get_genotypes_from_directory(directory)
        # update the dropboxes with the experiments
        self.update_dropboxes()

    def auto_populate_exps(self):
        """
        A method to open the suggested file dialog and populate arenas with suggested experiments.
        """
        # get the suggested file
        file = QtWidgets.QFileDialog.getOpenFileName(self, "Select Suggested File")
        # make sure the file is not empty and its a log file
        if file[0] and file[0].endswith(".log"):
            # set the suggested file
            pass
        
        # load the suggested file
        with open(file[0], "r") as f:
            # get the lines
            lines = f.readlines()
        
        # remove the empty lines
        lines = [line for line in lines if line.strip()]

        # ensure at least one arena is active
        if not any([self.checkboxes[i].isChecked() for i in range(16)]):
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select at least one arena.")
            return
        
        # find the number of active arenas
        active_arenas = [i for i in range(16) if self.checkboxes[i].isChecked()]
        num_active_arenas = len(active_arenas)
        
        # ensure at least one experiment is available
        if len(self.experiments) == 0:
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a directory with experiments.")
            return

        # loop through the lines
        for i, line in enumerate(lines):
            # make sure there are enough arenas
            if i >= num_active_arenas:
                print("Not enough arenas for all experiments.")
                break
            # check if the experiment is in the list of experiments
            if line.strip() in self.experiments:
                self.dropboxes[active_arenas[i]].setCurrentIndex(self.experiments.index(line.strip()))
            else:
                print("Experiment not found in list of experiments.")
                break


    def update_dropboxes(self):
        """
        A method to update the dropboxes.
        """
        self.experiment_dropdown.clear()
        self.experiment_dropdown.addItems(self.experiments)
        self.experiment_dropdown.setCurrentIndex(0)
        for i in range(16):
            self.dropboxes[i].clear()
            self.dropboxes[i].addItems(self.experiments)
            self.dropboxes[i].setCurrentIndex(0)
        self.fly_genotype.clear()
        self.fly_genotype.addItems(self.genotypes)
        self.fly_genotype.setCurrentIndex(0)
    
    def auto_populate_repeats(self):
        # ensure at least one arena is active
        if not any([self.checkboxes[i].isChecked() for i in range(16)]):
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select at least one arena.")
            return
        # ensure at least one experiment is available
        if len(self.experiments) == 0:
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a directory with experiments.")
            return
        # get the number of repeats
        repeats = int(self.repeats.text())
        # get the number of active arenas
        active_arenas = [i for i in range(16) if self.checkboxes[i].isChecked()]
        # shuffle the active arenas
        import random
        random.shuffle(active_arenas)
        n_arenas = len(active_arenas)
        if repeats > n_arenas:
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select fewer repeats. Not enough arenas.")
            return
        if len(self.experiments)*repeats < n_arenas:
            # give a warning
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select more repeats. Not enough experiments.")
            return
        # iterate through the arenas
        current_experiment = 0
        current_repeat = 0
        for arena in active_arenas:
            if current_repeat == repeats:
                current_repeat = 0
                current_experiment += 1
            self.dropboxes[arena].setCurrentIndex(current_experiment)
            current_repeat += 1
        pass

    def toggle_checkbox(self, index):
        """
        A method to toggle the checkbox.

        Variables:
            index (int): The index of the checkbox.
        """
        if self.checkboxes[index].isChecked():
            self.dropboxes[index].setEnabled(True)
            self.textboxes_1[index].setEnabled(True)
            self.textboxes_2[index].setEnabled(True)
            self.datetimes[index].setEnabled(True)
        else:
            self.dropboxes[index].setEnabled(False)
            self.textboxes_1[index].setEnabled(False)
            self.textboxes_2[index].setEnabled(False)
            self.datetimes[index].setEnabled(False)

    def toggle_all_arenas(self):
        """
        A method to toggle all the arenas.
        """
        for i in range(16):
            if self.toggle_state:
                self.checkboxes[i].setChecked(False)
                self.dropboxes[i].setEnabled(False)
                self.textboxes_1[i].setEnabled(False)
                self.textboxes_2[i].setEnabled(False)
                self.datetimes[i].setEnabled(False)
            else:
                self.checkboxes[i].setChecked(True)
                self.dropboxes[i].setEnabled(True)
                self.textboxes_1[i].setEnabled(True)
                self.textboxes_2[i].setEnabled(True)
                self.datetimes[i].setEnabled(True)
        self.toggle_state = not self.toggle_state

    def copy_genotype(self):
        """
        A method to copy the genotype to all the arenas.
        """
        for i in range(16):
            self.textboxes_1[i].setText(self.custom_fly_genotype.text())
    
    def copy_comments(self):
        """
        A method to copy the comments to all the arenas.
        """
        for i in range(16):
            self.textboxes_2[i].setText(self.comments.text())
    
    def copy_starvation_start(self):
        """
        A method to copy the starvation start to all the arenas.
        """
        for i in range(16):
            self.datetimes[i].setDateTime(self.starvation_start.dateTime())

    def copy_experiment(self):
        """
        A method to copy the experiment to all the arenas.
        """
        for i in range(16):
            self.dropboxes[i].setCurrentText(self.experiment_dropdown.currentText())
    
    def round_to_30mins(self, qdatetime):
        """
        A method to round a QDateTime to the nearest 30 minutes.
        """
        if qdatetime.time().minute() < 15:
            qdatetime.setTime(QtCore.QTime(qdatetime.time().hour(), 0, 0))
        elif qdatetime.time().minute() < 45:
            qdatetime.setTime(QtCore.QTime(qdatetime.time().hour(), 30, 0))
        else:
            qdatetime.setTime(QtCore.QTime(qdatetime.time().hour() + 1, 0, 0))
        return qdatetime
    
    def starvation_start_now(self):
        """
        A method to update the starvation start to now - time set in the text box.
        """
        current_time = QtCore.QDateTime.currentDateTime()
        current_time = self.round_to_30mins(current_time)
        starvation_period = float(self.starvation_period.text())*60*60
        starvation_time = current_time.addSecs(-int(starvation_period))
        self.starvation_start.setDateTime(starvation_time)
    
    def starvation_24(self):
        """
        A method to update the starvation start to 24 hours before now.
        """
        self.starvation_period.setText("24.0")
    
    def starvation_36(self):
        """
        A method to update the starvation start to 36 hours before now.
        """
        self.starvation_period.setText("36.0")

    def starvation_48(self):
        """
        A method to update the starvation start to 48 hours before now.
        """
        self.starvation_period.setText("48.0")
    
    def starvation_60(self):
        """
        A method to update the starvation start to 60 hours before now.
        """
        self.starvation_period.setText("60.0")
    
    def starvation_72(self):
        """
        A method to update the starvation start to 72 hours before now.
        """
        self.starvation_period.setText("72.0")

    def fly_genotype_changed(self):
        """
        A method to update the fly genotype.
        """
        # check if its custom
        if self.fly_genotype.currentText() == "custom":
            self.custom_fly_genotype.setEnabled(True)
            self.custom_fly_genotype.setText("")
        else:
            self.custom_fly_genotype.setEnabled(False)
            self.custom_fly_genotype.setText(self.fly_genotype.currentText())

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
                experiment for experiment in experiments if experiment.endswith(".csv") or experiment.endswith(".yfse") or experiment.endswith(".ymle")
            ]

        # if there are no experiments, send an alert dialog
        if len(experiments) == 0:
            QtWidgets.QMessageBox.about(self, "No Experiments", "There are no experiments in the directory.")

        return experiments
    
    def get_genotypes_from_directory(self, directory):
        """
        A method to get the genotypes from the directory.

        Variables:
            directory (str): The directory to get the genotypes from.
        """
        # check if the directory exists
        if not os.path.exists(directory + "/genotypes.log"):
            genotypes = ["custom"]
        else:
            # read the genotypes from the file
            with open(directory + "/genotypes.log", "r") as file:
                genotypes = file.read().splitlines()
            # add custom to the list
            genotypes.append("custom")
        # remove empty genotypes
        genotypes = [genotype for genotype in genotypes if genotype != ""]
        # remove duplicates
        genotypes = list(dict.fromkeys(genotypes))
        return genotypes
    
    def save_genotypes_to_directory(self, directory):
        """
        A method to save the genotypes to the directory.

        Variables:
            directory (str): The directory to save the genotypes to.
        """
        # write the genotypes to the file
        with open(directory + "/genotypes.log", "w") as file:
            for genotype in self.genotypes:
                if genotype != "custom":
                    file.write(genotype + "\n")
            if self.fly_genotype.currentText() == "custom":
                file.write(self.custom_fly_genotype.text() + "\n")

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

        # check that experiment name is not empty
        if self.experiment_name.text() == "":
            # show a warning message that the experiment name is empty
            QtWidgets.QMessageBox.warning(self, "Error", "Please enter an experiment name.")
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
                        "python sixteeny/gui/16Y_rig_configurator.py "
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
                experiment_details["fly_genotype"] = self.textboxes_1[i].text()
                experiment_details["fly_comments"] = self.textboxes_2[i].text()
                experiment_details["fly_starvation_time"] = self.datetimes[i].dateTime().toString("yyyy-MM-dd hh:mm:ss")
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
        self.save_genotypes_to_directory(self.project_directory.text())

    def prepare_directory(self):
        """
        A method to prepare the experiment directory.
        """
        # get used experiment names
        used_experiments = [self.dropboxes[i].currentText() for i in range(16) if self.checkboxes[i].isChecked()]

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

        # check if the stimuli folder already exists
        if os.path.isdir(os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli")):
            # remove the stimuli folder
            shutil.rmtree(os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli"))
        # create the stimuli folder
        os.mkdir(os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "stimuli"))

        # copy the used experiments from experiment_zoo to a new experiments folder in the experiment directory
        for experiment in used_experiments:
            # copy the experiment file from the experiment_zoo to the experiments folder overwriting the old experiment
            shutil.copy(
                os.path.join(self.project_directory.text(), "experiment_zoo", experiment),
                os.path.join(self.project_directory.text(), "data/" + self.experiment_name.text(), "experiments"),
            )
            # print("Copied " + experiment + " to data/" + self.experiment_name.text() + "/experiments")

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
                # print("Copied " + experiment.split(".")[0] + ".meta to data/" + self.experiment_name.text() + "/experiments")
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

                    # copy the used stimuli from the stimulus_zoo to a new stimuli folder in the experiment directory
                    for stimulus in metadata["used_stimuli"]:
                        # check if the stimulus_zoo has the stimulus file
                        if os.path.isfile(os.path.join(self.project_directory.text(), "stimulus_zoo", stimulus)):
                            # check if the stimuli folder doesn't already has the stimulus file
                            if not os.path.isfile(
                                os.path.join(
                                    self.project_directory.text(),
                                    "data/" + self.experiment_name.text(),
                                    "stimuli",
                                    stimulus,
                                )
                            ):
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
