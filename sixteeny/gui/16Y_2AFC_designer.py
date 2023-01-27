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
        self.setWindowTitle("2AFC Designer")
        self.setGeometry(100, 100, 960, 800)

        # create a main layout
        self.main_layout = QtWidgets.QGridLayout()

        # store the start folder and load file
        self.start_folder = start_folder
        self.load_file = load_file

        # create a table widget
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(3)
        self.table.setRowCount(0)
        self.table.setHorizontalHeaderLabels(
            ["Number of Trials", "Reward Probability (Odor 1)", "Reward Probability (Odor 2)"]
        )
        self.table.setSizeAdjustPolicy(QtWidgets.QTableWidget.AdjustToContents)
        self.table.setShowGrid(False)
        self.table.setCornerButtonEnabled(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_layout.addWidget(self.table, 0, 0, 1, 4)

        # create a checkbox for L/R Randomization
        self.randomize_lr = QtWidgets.QCheckBox("Randomize L/R?")
        self.randomize_lr.setChecked(True)
        self.main_layout.addWidget(self.randomize_lr, 1, 0)

        # create a checkbox for presampled L/R
        self.presampled_lr = QtWidgets.QCheckBox("Presampled L/R?")
        self.presampled_lr.setChecked(False)
        self.main_layout.addWidget(self.presampled_lr, 1, 1)

        # create a checkbox for Reward Hold/Baiting
        self.reward_hold_bait = QtWidgets.QCheckBox("Hold/Bait Rewards?")
        self.reward_hold_bait.setChecked(False)
        self.main_layout.addWidget(self.reward_hold_bait, 1, 2)

        # create a checkbox for Reciprocal Task
        self.reciprocal_task = QtWidgets.QCheckBox("Reciprocal Task?")
        self.reciprocal_task.setChecked(True)
        self.main_layout.addWidget(self.reciprocal_task, 1, 3)

        # create a button to add a row
        self.add_row_button = QtWidgets.QPushButton("Add new block")
        self.add_row_button.clicked.connect(self.add_row)
        self.main_layout.addWidget(self.add_row_button, 2, 0, 1, 2)

        # create a button to remove a row
        self.remove_row_button = QtWidgets.QPushButton("Remove last block")
        self.remove_row_button.clicked.connect(self.remove_row)
        self.main_layout.addWidget(self.remove_row_button, 2, 2, 1, 2)

        # create a text label, text box, and button to load stimulus for odor 1
        self.odor1_label = QtWidgets.QLabel("Odor 1 Stimulus")
        self.main_layout.addWidget(self.odor1_label, 3, 0, 1, 1)
        self.odor1_textbox = QtWidgets.QLineEdit()
        self.odor1_button = QtWidgets.QPushButton("Load")
        self.odor1_button.clicked.connect(self.load_odor1)
        self.odor1_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.odor1_textbox, 3, 1, 1, 2)
        self.main_layout.addWidget(self.odor1_button, 3, 3, 1, 1)

        # create a text label, text box, and button to load stimulus for odor 2
        self.odor2_label = QtWidgets.QLabel("Odor 2 Stimulus")
        self.main_layout.addWidget(self.odor2_label, 4, 0, 1, 1)
        self.odor2_textbox = QtWidgets.QLineEdit()
        self.odor2_button = QtWidgets.QPushButton("Load")
        self.odor2_button.clicked.connect(self.load_odor2)
        self.odor2_textbox.setReadOnly(True)
        self.main_layout.addWidget(self.odor2_textbox, 4, 1, 1, 2)
        self.main_layout.addWidget(self.odor2_button, 4, 3, 1, 1)

        # add a box with random generator settings
        self.random_generator_box = QtWidgets.QGroupBox("Random 2AFC Task Generator (Separate by spaces)")

        regex = r"^(\s*(-|\+)?\d+(?:\.\d+)?\s*,\s*)+(-|\+)?\d+(?:\.\d+)?\s*$"
        validator = QtGui.QRegExpValidator(QtCore.QRegExp(regex), self)

        # create a text label, text box to enter Reward Gain values
        self.reward_gain_label = QtWidgets.QLabel("Reward Gain [0, 1]")
        self.reward_gain_textbox = QtWidgets.QLineEdit()
        self.reward_gain_textbox.setText("0.125 0.25 0.5")
        # self.reward_gain_textbox.setValidator(validator)

        # create a text label, text box to enter Reward Contrast values
        self.reward_contrast_label = QtWidgets.QLabel("Reward Contrast [0.5, 1]")
        self.reward_contrast_textbox = QtWidgets.QLineEdit()
        self.reward_contrast_textbox.setText("0.5 0.65 0.8")
        # self.reward_contrast_textbox.setValidator(validator)

        # create a text label, text box to enter Hazard Rate values
        self.hazard_rate_label = QtWidgets.QLabel("Hazard Rate [0, 1]")
        self.hazard_rate_textbox = QtWidgets.QLineEdit()
        self.hazard_rate_textbox.setText("0.035 0.02 0.01")
        # self.hazard_rate_textbox.setValidator(validator)

        # create a text label, text box to enter number of naive trials
        self.naive_trials_label = QtWidgets.QLabel("Naive Trials")
        self.naive_trials_textbox = QtWidgets.QLineEdit()
        self.naive_trials_textbox.setText("30")
        self.naive_trials_textbox.setValidator(QtGui.QIntValidator())

        # create a text label, text box to enter number of maximum testing trials
        self.max_trials_label = QtWidgets.QLabel("Max Experiment Trials")
        self.max_trials_textbox = QtWidgets.QLineEdit()
        self.max_trials_textbox.setText("170")
        self.max_trials_textbox.setValidator(QtGui.QIntValidator())

        # create a button to generate the random task
        self.generate_button = QtWidgets.QPushButton("Generate")
        self.generate_button.clicked.connect(self.generate_task)

        # create a layout for the random generator box
        self.random_generator_layout = QtWidgets.QGridLayout()
        self.random_generator_layout.addWidget(self.reward_gain_label, 0, 0)
        self.random_generator_layout.addWidget(self.reward_gain_textbox, 0, 1)
        self.random_generator_layout.addWidget(self.reward_contrast_label, 1, 0)
        self.random_generator_layout.addWidget(self.reward_contrast_textbox, 1, 1)
        self.random_generator_layout.addWidget(self.hazard_rate_label, 2, 0)
        self.random_generator_layout.addWidget(self.hazard_rate_textbox, 2, 1)
        self.random_generator_layout.addWidget(self.naive_trials_label, 3, 0)
        self.random_generator_layout.addWidget(self.naive_trials_textbox, 3, 1)
        self.random_generator_layout.addWidget(self.max_trials_label, 4, 0)
        self.random_generator_layout.addWidget(self.max_trials_textbox, 4, 1)
        self.random_generator_layout.addWidget(self.generate_button, 5, 0, 1, 2)
        self.random_generator_box.setLayout(self.random_generator_layout)

        # add the random generator box to the main layout
        self.main_layout.addWidget(self.random_generator_box, 5, 0, 1, 4)

        # create a button to save the table
        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.clicked.connect(self.save)
        self.main_layout.addWidget(self.save_button, 6, 0, 1, 2)

        # create a button to load the table
        self.load_button = QtWidgets.QPushButton("Load")
        self.load_button.clicked.connect(self.load)
        self.main_layout.addWidget(self.load_button, 6, 2, 1, 2)

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

    def load_odor1(self):
        """
        Load the stimulus for odor 1.
        """
        folder = self.start_folder
        # move back one folder
        if folder[-1] == "/":
            folder = folder[:-1]
        folder = folder[:folder.rfind("/")]
        # go to the stimulus_zoo folder
        folder = folder + "/stimulus_zoo/"
        # open a file dialog
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", folder, "*.stim")
        # if a file was selected
        if filename:
            # set the textbox to the file name (without the path)
            self.odor1_textbox.setText(filename.split("/")[-1])

    def load_odor2(self):
        """
        Load the stimulus for odor 2.
        """
        folder = self.start_folder
        # move back one folder
        if folder[-1] == "/":
            folder = folder[:-1]
        folder = folder[:folder.rfind("/")]
        # go to the stimulus_zoo folder
        folder = folder + "/stimulus_zoo/"
        # open a file dialog
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", folder, "*.stim")
        # if a file was selected
        if filename:
            # set the textbox to the file name (without the path)
            self.odor2_textbox.setText(filename.split("/")[-1])

    def add_row(self):
        """
        Add a new row to the table.
        """
        # insert a new row at the end of the table
        self.table.insertRow(self.table.rowCount())
        self.table.setItem(self.table.rowCount() - 1, 0, QtWidgets.QTableWidgetItem("1"))
        self.table.setItem(self.table.rowCount() - 1, 1, QtWidgets.QTableWidgetItem("0.5"))
        self.table.setItem(self.table.rowCount() - 1, 2, QtWidgets.QTableWidgetItem("0.5"))
        # resize the table to fit the new row
        self.table.resizeRowsToContents()
        self.table.resizeColumnsToContents()
        # set the current cell to the last row
        self.table.setCurrentCell(self.table.rowCount() - 1, 0)
        self.table.editItem(self.table.currentItem())

    def remove_row(self):
        """
        Remove the last row from the table.
        """
        # check if there is at least one row in the table
        if self.table.rowCount() > 0:
            # remove the last row
            self.table.removeRow(self.table.rowCount() - 1)
            # resize the table to fit the existing rows
            self.table.resizeRowsToContents()
            self.table.resizeColumnsToContents()
            # set the current cell to the last row
            self.table.setCurrentCell(self.table.rowCount() - 1, 0)
            self.table.editItem(self.table.currentItem())

    def save(self):
        """
        Convert table to a pandas dataframe and save to a csv experiment file.
        """
        # ensure that the table has at least one row and stimulus files have been loaded
        if not (self.table.rowCount() > 0 and self.odor1_textbox.text() != "" and self.odor2_textbox.text() != ""):
            # show an error message
            QtWidgets.QMessageBox.warning(
                self,
                "Error",
                "Please ensure that the table has at least one row and that both odor stimuli have been loaded.",
            )
            return

        # create a pandas dataframe from the table
        df = pd.DataFrame(
            columns=[
                "Trial#",
                "P(R|Air)",
                "P(R|O1)",
                "P(R|O2)",
                "Odor(Start)",
                "Odor(Left)",
                "Odor(Right)",
                "CStim(Air)",
                "CStim(O1)",
                "CStim(O2)",
                "StayTime(Air)",
                "StayTime(O1)",
                "StayTime(O2)",
                "Baited",
                "Timed",
                "OdorDelay",
                "UStim",
            ]
        )
        for row in range(self.table.rowCount()):
            # get the number of trials
            num_trials = int(self.table.item(row, 0).text())
            # get the reward probabilities for odor 1 and 2
            p_r_o1 = float(self.table.item(row, 1).text())
            p_r_o2 = float(self.table.item(row, 2).text())
            # get the odor stimulus filename
            odor1_stim = self.odor1_textbox.text().split("/")[-1]
            odor2_stim = self.odor2_textbox.text().split("/")[-1]
            # get the baiting status
            baited = 1 if self.reward_hold_bait.isChecked() else 0

            # add a row to the dataframe for each trial
            for trial in range(num_trials):
                # if the randomize L/R checkbox is checked, randomly assign L/R
                left = (
                    (np.random.choice([1, 2]) if self.presampled_lr.isChecked() else 1.5)
                    if self.randomize_lr.isChecked()
                    else 1
                )
                # add a row to the dataframe
                df.loc[len(df)] = [
                    1,
                    0,
                    p_r_o1,
                    p_r_o2,
                    0,
                    left,
                    3 - left,
                    "empty.stim",
                    odor1_stim,
                    odor2_stim,
                    "inf",
                    0,
                    0,
                    baited,
                    0,
                    0,
                    "empty.stim",
                ]

        # generate reciprocal by switching the values in P(R|O1) and P(R|O2) and Odor(Left) and Odor(Right)
        reciprocal = df.copy()
        reciprocal["P(R|O1)"] = df["P(R|O2)"]
        reciprocal["P(R|O2)"] = df["P(R|O1)"]
        reciprocal["Odor(Left)"] = df["Odor(Right)"]
        reciprocal["Odor(Right)"] = df["Odor(Left)"]

        # save the dataframe to a csv file
        if self.start_folder != "":
            default_folder = self.start_folder
        else:
            default_folder = "."

        if self.load_file is not None:
            filename = self.load_file
        else:
            filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Experiment", default_folder, "*.csv")

        if filename:
            df.to_csv(filename, index=False)
            if self.reciprocal_task.isChecked():
                reciprocal.to_csv(filename.replace(".csv", "_reciprocal.csv"), index=False)
            # show a message box indicating that the experiment was saved
            QtWidgets.QMessageBox.information(
                self, "Success", "The experiment was saved to {}".format(filename),
            )

        # create a dictionary of the used stimuli
        stimuli = {"used_stimuli": [self.odor1_textbox.text(), self.odor2_textbox.text(),]}
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
            "Do you want to close the 2AFC Designer?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.close()

    def load(self, load_file=None):
        """
        Load an experiment from a csv file.
        """
        if load_file is not None and load_file != False:
            filename = load_file
        else:
            # open a file dialog
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Experiment", ".", "*.csv")
            # if a file was selected

        if filename:
            # load the dataframe from the csv file
            df = pd.read_csv(filename)
            # check if the dataframe has the correct columns
            if not df.columns.tolist() == [
                "Trial#",
                "P(R|Air)",
                "P(R|O1)",
                "P(R|O2)",
                "Odor(Start)",
                "Odor(Left)",
                "Odor(Right)",
                "CStim(Air)",
                "CStim(O1)",
                "CStim(O2)",
                "StayTime(Air)",
                "StayTime(O1)",
                "StayTime(O2)",
                "Baited",
                "Timed",
                "OdorDelay",
                "UStim",
            ]:
                # show an error message
                QtWidgets.QMessageBox.warning(
                    self,
                    "Error",
                    "The experiment file is invalid. Please ensure that the file contains data in the correct format.",
                )
                return
            # delete all rows from the table
            self.table.setRowCount(0)
        else:
            return

        # create an empty dictionary to store all blocks of trials
        all_blocks = {}
        block_no = 1

        # find blocks of consecutive rows with the same values for the P(R|O1) columns
        blocks = df.groupby([(df["P(R|O1)"] != df["P(R|O1)"].shift()).cumsum()]).groups
        # divide the blocks further into groups of consecutive rows with the same values for the P(R|O2) columns
        for block in blocks:
            # get the dataframe for the block
            block_df = df.loc[blocks[block]]
            # find the blocks of consecutive rows with the same values for the P(R|O2) columns
            block_blocks = block_df.groupby([(block_df["P(R|O2)"] != block_df["P(R|O2)"].shift()).cumsum()]).groups
            for block_block in block_blocks:
                # add the block to the dictionary
                all_blocks[block_no] = block_df.loc[block_blocks[block_block]]
                # increment the block number
                block_no += 1

        # blocks = df.groupby(["P(R|O1)", "P(R|O2)"], sort=False).groups
        # add a row to the table for each block
        p_values = []
        for block in all_blocks:
            # get the number of trials in the block
            num_trials = all_blocks[block].shape[0]
            # get the reward probabilities for odor 1 and 2
            p_r_o1 = all_blocks[block].iloc[0]["P(R|O1)"]
            p_r_o2 = all_blocks[block].iloc[0]["P(R|O2)"]
            # add a row to the table
            self.table.insertRow(self.table.rowCount())
            self.table.setItem(self.table.rowCount() - 1, 0, QtWidgets.QTableWidgetItem(str(num_trials)))
            self.table.setItem(self.table.rowCount() - 1, 1, QtWidgets.QTableWidgetItem(str(p_r_o1)))
            self.table.setItem(self.table.rowCount() - 1, 2, QtWidgets.QTableWidgetItem(str(p_r_o2)))
            # check if the block is randomized by looking at the value of the "Odor(Left)" column
            left_values = df.loc[blocks[block], "Odor(Left)"].values - 1
            # find mean of the left values and calculate the p-value
            p_value = stats.binom_test(left_values.sum(), n=num_trials, p=0.5)
            p_values.append(p_value)

        # check if the p-value is more than 0.05 for all blocks
        if np.all(np.array(p_values) > 0.05):
            self.randomize_lr.setChecked(True)
        else:
            self.randomize_lr.setChecked(False)

        # ensure all CStim(O1/O2) and baiting status are the same
        if len(df["CStim(O1)"].unique()) > 1 or len(df["CStim(O2)"].unique()) > 1 or len(df["Baited"].unique()) > 1:
            # show an error message
            QtWidgets.QMessageBox.warning(
                self,
                "Error",
                "The experiment file is invalid. Please ensure that the file contains data in the correct format.",
            )
            return
        # get the odor stimulus filenames
        odor1_stim = df["CStim(O1)"].values[0]
        odor2_stim = df["CStim(O2)"].values[0]
        # get the baiting status
        baited = True if df.loc[blocks[block], "Baited"].values[0] else False
        # set the odor stimulus filenames
        self.odor1_textbox.setText(odor1_stim)
        self.odor2_textbox.setText(odor2_stim)
        # set the baiting status
        self.reward_hold_bait.setChecked(baited)

        # resize the table to fit the new data
        self.table.resizeColumnsToContents()
        self.table.resizeRowsToContents()

    def generate_task(self):
        # convert textbox values to a list of floats
        reward_contrasts = [float(x) for x in self.reward_contrast_textbox.text().split(" ")]
        reward_gains = [float(x) for x in self.reward_gain_textbox.text().split(" ")]
        hazard_rates = [float(x) for x in self.hazard_rate_textbox.text().split(" ")]

        # get naive trial number
        naive_trials = int(self.naive_trials_textbox.text())
        # get maximum trial number
        max_trials = int(self.max_trials_textbox.text())

        # choose random hazard rate
        random_hazard_rate = np.random.choice(hazard_rates)
        experiments = []
        if naive_trials > 0:
            experiments.append([naive_trials, 0, 0])

        n_trials = 0
        max_state = 0

        last_reward_gain = 0
        last_reward_contrast = 0

        while n_trials < max_trials:

            while True:
                # sample block size as geometric distribution using hazard rate
                block_size = np.random.geometric(random_hazard_rate)
                # round block size to nearest five
                block_size = int(np.round(block_size / 5) * 5)
                # check if block size is more than 20 and less than 150
                if block_size >= 20 and block_size <= 150:
                    break

            # sample reward gain and contrast
            while True:
                reward_gain = np.random.choice(reward_gains)
                reward_contrast = np.random.choice(reward_contrasts)
                if reward_gain != last_reward_gain and reward_contrast != last_reward_contrast:
                    last_reward_contrast = reward_contrast
                    last_reward_gain = reward_gain
                    break

            # calculate reward probability
            pr1 = round(reward_gain * 2 * reward_contrast, 2)
            pr2 = round(reward_gain * 2 * (1 - reward_contrast), 2)

            if n_trials + block_size > max_trials:
                block_size = max_trials - n_trials
            if max_state == 0:
                experiments.append([block_size, pr1, pr2])
                max_state = 1
            else:
                experiments.append([block_size, pr2, pr1])
                max_state = 0

            n_trials += block_size

        # add experiments to table
        self.table.setRowCount(0)
        for experiment in experiments:
            self.table.insertRow(self.table.rowCount())
            self.table.setItem(self.table.rowCount() - 1, 0, QtWidgets.QTableWidgetItem(str(experiment[0])))
            self.table.setItem(self.table.rowCount() - 1, 1, QtWidgets.QTableWidgetItem(str(experiment[1])))
            self.table.setItem(self.table.rowCount() - 1, 2, QtWidgets.QTableWidgetItem(str(experiment[2])))

        # resize the table to fit the new data
        self.table.resizeColumnsToContents()
        self.table.resizeRowsToContents()


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
