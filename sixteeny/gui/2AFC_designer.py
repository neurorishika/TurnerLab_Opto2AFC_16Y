from PyQt5 import QtWidgets, QtCore, QtGui
import pandas as pd
import numpy as np
from scipy import stats
import json


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2AFC Designer")
        self.setGeometry(100, 100, 960, 400)

        # create a main layout
        self.main_layout = QtWidgets.QGridLayout()

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
        self.main_layout.addWidget(self.randomize_lr, 1, 0, 1, 2)

        # create a checkbox for Reward Hold/Baiting
        self.reward_hold_bait = QtWidgets.QCheckBox("Hold/Bait Rewards?")
        self.reward_hold_bait.setChecked(False)
        self.main_layout.addWidget(self.reward_hold_bait, 1, 2, 1, 2)

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

        # create a button to save the table
        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.clicked.connect(self.save)
        self.main_layout.addWidget(self.save_button, 5, 0, 1, 2)

        # create a button to load the table
        self.load_button = QtWidgets.QPushButton("Load")
        self.load_button.clicked.connect(self.load)
        self.main_layout.addWidget(self.load_button, 5, 2, 1, 2)

        # create the central widget
        self.central_widget = QtWidgets.QWidget()
        self.central_widget.setLayout(self.main_layout)

        # set the central widget
        self.setCentralWidget(self.central_widget)

        # show the window
        self.show()

    def load_odor1(self):
        """
        Load the stimulus for odor 1.
        """
        # open a file dialog
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", ".", "*.stim")
        # if a file was selected
        if file_name[0] != "":
            # set the textbox to the file name (without the path)
            self.odor1_textbox.setText(file_name[0].split("/")[-1])

    def load_odor2(self):
        """
        Load the stimulus for odor 2.
        """
        # open a file dialog
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Open Stimulus", ".", "*.stim")
        # if a file was selected
        if file_name[0] != "":
            # set the textbox to the file name (without the path)
            self.odor2_textbox.setText(file_name[0].split("/")[-1])

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
                left = np.random.choice([1, 2]) if self.randomize_lr.isChecked() else 1
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
        # save the dataframe to a csv file
        file_name = QtWidgets.QFileDialog.getSaveFileName(self, "Save Experiment", ".", "*.csv")
        if file_name[0] != "":
            df.to_csv(file_name[0], index=False)
            # show a message box indicating that the experiment was saved
            QtWidgets.QMessageBox.information(
                self, "Success", "The experiment was saved to {}".format(file_name[0]),
            )
        # create a dictionary of the used stimuli
        stimuli = {"used_stimuli": [self.odor1_textbox.text(), self.odor2_textbox.text(),]}
        # save the stimuli dictionary to a json file with .meta extension with the same name as the experiment file
        file_name = file_name[0].split(".")[0] + ".meta"
        with open(file_name, "w") as f:
            json.dump(stimuli, f)

    def load(self):
        """
        Load an experiment from a csv file.
        """
        # open a file dialog
        file_name = QtWidgets.QFileDialog.getOpenFileName(self, "Open Experiment", ".", "*.csv")
        # if a file was selected
        if file_name[0] != "":
            # load the dataframe from the csv file
            df = pd.read_csv(file_name[0])
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


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    app.exec_()
