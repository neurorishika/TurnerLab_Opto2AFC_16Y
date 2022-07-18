from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import os
import shutil
from subprocess import call
import pandas as pd


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the window title
        self.setWindowTitle("16Y Project Manager")

        # Set up the window geometry
        self.setGeometry(100, 100, 800, 600)

        # create the main layout
        self.main_layout = QtWidgets.QGridLayout()

        # add a text box for the project directory with a label and buttons to browse or to create a new directory
        self.project_directory_label = QtWidgets.QLabel("Project Directory")
        self.project_directory_textbox = QtWidgets.QLineEdit()
        self.project_directory_textbox.setReadOnly(True)
        self.project_directory_browse_button = QtWidgets.QPushButton("Browse")
        self.project_directory_create_button = QtWidgets.QPushButton("Create New")

        # add the text box and buttons to the main layout
        self.main_layout.addWidget(self.project_directory_label, 0, 0)
        self.main_layout.addWidget(self.project_directory_textbox, 0, 1, 1, 3)
        self.main_layout.addWidget(self.project_directory_browse_button, 0, 4)
        self.main_layout.addWidget(self.project_directory_create_button, 0, 5)

        # connect the buttons to their respective functions
        self.project_directory_browse_button.clicked.connect(self.browse_project_directory)
        self.project_directory_create_button.clicked.connect(self.create_project_directory)

        # add three labels for the experimental data, stimulus zoo, and experiment zoo
        self.experimental_data_label = QtWidgets.QLabel("Experimental Data")
        self.stimulus_zoo_label = QtWidgets.QLabel("Stimulus Zoo")
        self.experiment_zoo_label = QtWidgets.QLabel("Experiment Zoo")

        # add the labels to the main layout
        self.main_layout.addWidget(self.experimental_data_label, 1, 0, 1, 2)
        self.main_layout.addWidget(self.stimulus_zoo_label, 1, 2, 1, 2)
        self.main_layout.addWidget(self.experiment_zoo_label, 1, 4, 1, 2)

        # add a table for the experimental data, stimulus zoo, and experiment zoo
        self.experimental_data_table = QtWidgets.QTableWidget()
        self.stimulus_zoo_table = QtWidgets.QTableWidget()
        self.experiment_zoo_table = QtWidgets.QTableWidget()

        # add the tables to the main layout
        self.main_layout.addWidget(self.experimental_data_table, 2, 0, 1, 2)
        self.main_layout.addWidget(self.stimulus_zoo_table, 2, 2, 1, 2)
        self.main_layout.addWidget(self.experiment_zoo_table, 2, 4, 1, 2)

        # add a button to start processing the selected experimental data and generate a video
        self.process_experimental_data_button = QtWidgets.QPushButton("Process Experimental Data")
        self.generate_video_button = QtWidgets.QPushButton("Generate Video")

        # connect the buttons to their respective functions
        self.process_experimental_data_button.clicked.connect(self.process_experimental_data)
        self.generate_video_button.clicked.connect(self.generate_video)

        # add a button to create a new stimulus and to delete a stimulus
        self.create_new_stimulus_button = QtWidgets.QPushButton("Create New Stimulus")
        self.delete_stimulus_button = QtWidgets.QPushButton("Delete Stimulus")

        # connect the buttons to their respective functions
        self.create_new_stimulus_button.clicked.connect(self.create_new_stimulus)
        self.delete_stimulus_button.clicked.connect(self.delete_stimulus)

        # add a button to create a new experiment and to delete an experiment
        self.create_new_experiment_button = QtWidgets.QPushButton("Create New Experiment")
        self.delete_experiment_button = QtWidgets.QPushButton("Delete Experiment")

        # connect the buttons to their respective functions
        self.create_new_experiment_button.clicked.connect(self.create_new_experiment)
        self.delete_experiment_button.clicked.connect(self.delete_experiment)

        # add the buttons to the main layout
        self.main_layout.addWidget(self.process_experimental_data_button, 3, 0)
        self.main_layout.addWidget(self.generate_video_button, 3, 1)
        self.main_layout.addWidget(self.create_new_stimulus_button, 3, 2)
        self.main_layout.addWidget(self.delete_stimulus_button, 3, 3)
        self.main_layout.addWidget(self.create_new_experiment_button, 3, 4)
        self.main_layout.addWidget(self.delete_experiment_button, 3, 5)

        # create the central widget
        self.central_widget = QtWidgets.QWidget()
        self.central_widget.setLayout(self.main_layout)

        # set the central widget
        self.setCentralWidget(self.central_widget)

        # show the window
        self.show()

    def browse_project_directory(self):
        # get the directory
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Project Directory")

        # set the text box to the directory
        self.project_directory_textbox.setText(directory)

        # refresh the details from the directory
        self.refresh_project_details()

    def create_project_directory(self):
        # get the directory to save the project
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "Select directory to save project")

        # get the name of the project
        name = QtWidgets.QInputDialog.getText(self, "Project Name", "Enter the name of the project")[0]

        # create the project directory
        project_directory = directory + "/" + name
        if not os.path.exists(project_directory):
            os.makedirs(project_directory)
        else:
            # ask the user if they want to overwrite the project directory
            reply = QtWidgets.QMessageBox.question(
                self,
                "Overwrite Project Directory",
                "The project directory already exists. Do you want to overwrite it?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if reply == QtWidgets.QMessageBox.Yes:
                # remove all the files in the directory recursively
                shutil.rmtree(project_directory)
                # create the project directory
                os.makedirs(project_directory)
            else:
                # tell the user that the project directory was not created
                QtWidgets.QMessageBox.information(
                    self, "Project Directory Not Created", "The project directory was not created."
                )
                return

        # create the data folder
        data_directory = project_directory + "/data"
        os.makedirs(data_directory)

        # create the stimulus_zoo folder
        stimulus_zoo_directory = project_directory + "/stimulus_zoo"
        os.makedirs(stimulus_zoo_directory)

        # copy empty stimulus file to the stimulus_zoo folder
        shutil.copy("sixteeny/gui/resources/empty.stim", stimulus_zoo_directory)

        # create the experiment_zoo folder
        experiment_zoo_directory = project_directory + "/experiment_zoo"
        os.makedirs(experiment_zoo_directory)

        # tell the user that the project directory was created
        QtWidgets.QMessageBox.information(self, "Project Directory Created", "The project directory was created.")

        # set the text box to the directory
        self.project_directory_textbox.setText(project_directory)

        # refresh the details from the directory
        self.refresh_project_details()

    def refresh_project_details(self):
        # verify the project structure
        if (
            not os.path.exists(self.project_directory_textbox.text())
            or not os.path.exists(self.project_directory_textbox.text() + "/data")
            or not os.path.exists(self.project_directory_textbox.text() + "/stimulus_zoo")
            or not os.path.exists(self.project_directory_textbox.text() + "/experiment_zoo")
        ):
            # tell the user that the project directory is not valid
            QtWidgets.QMessageBox.information(
                self, "Project Directory Not Valid", "The project directory is not valid."
            )
            # reset the project directory text box
            self.project_directory_textbox.setText("")
            return

        # get the list of folders in the data directory
        data_folders = os.listdir(self.project_directory_textbox.text() + "/data")
        data_folders = [
            folder
            for folder in data_folders
            if os.path.isdir(self.project_directory_textbox.text() + "/data/" + folder)
        ]

        # for each data folder see if there is a processed_data directory with the same number of files as the data folder and if a processed_video is generated
        processed = []
        generated_videos = []
        for folder in data_folders:
            if os.path.exists(self.project_directory_textbox.text() + "/data/" + folder + "/processed_data"):
                if len(
                    os.listdir(self.project_directory_textbox.text() + "/data/" + folder + "/processed_data")
                ) == len(os.listdir(self.project_directory_textbox.text() + "/data/" + folder + "/data")):
                    processed.append(True)
            else:
                processed.append(False)
            if os.path.exists(self.project_directory_textbox.text() + "/data/" + folder + "/processed_video.mp4"):
                generated_videos.append(True)
            else:
                generated_videos.append(False)

        # get the list of *.stim files in the stimulus_zoo directory
        stimulus_zoo_files = os.listdir(self.project_directory_textbox.text() + "/stimulus_zoo")
        stimulus_zoo_files = [file for file in stimulus_zoo_files if file.endswith(".stim")]

        allowed_experiment_types = {"csv": "open-loop+baiting", "yfse": "finite-world-dynamics"}

        # get the list of allowed experiment types in the experiment_zoo directory
        experiment_zoo_files = os.listdir(self.project_directory_textbox.text() + "/experiment_zoo")
        experiment_zoo_files = [
            file for file in experiment_zoo_files if file.split(".")[-1] in allowed_experiment_types.keys()
        ]

        if len(data_folders) == 0:
            # tell the user that there are no data folders
            QtWidgets.QMessageBox.information(
                self, "No Data Folders", "There are no data folders in the project directory."
            )
            # reset the experimental data table
            self.experimental_data_table.setRowCount(0)
            self.experimental_data_table.setColumnCount(3)
            self.experimental_data_table.setHorizontalHeaderLabels(["Data Folder", "Processed", "Generated Video"])
        else:
            # fill the experimental data table with the data from the project directory
            self.experimental_data_table.setRowCount(len(data_folders))
            self.experimental_data_table.setColumnCount(3)
            self.experimental_data_table.setHorizontalHeaderLabels(["Data Folder", "Processed", "Generated Video"])
            for i in range(len(data_folders)):
                self.experimental_data_table.setItem(i, 0, QtWidgets.QTableWidgetItem(data_folders[i]))
                self.experimental_data_table.setItem(i, 1, QtWidgets.QTableWidgetItem(str(processed[i])))
                self.experimental_data_table.setItem(i, 2, QtWidgets.QTableWidgetItem(str(generated_videos[i])))
            self.experimental_data_table.resizeColumnsToContents()
            self.experimental_data_table.resizeRowsToContents()

        if len(stimulus_zoo_files) == 0:
            # tell the user that there are no stimulus zoo files
            QtWidgets.QMessageBox.information(
                self, "No Stimulus Zoo Files", "There are no stimulus zoo files in the project directory."
            )
            # reset the stimulus zoo table
            self.stimulus_zoo_table.setRowCount(0)
            self.stimulus_zoo_table.setColumnCount(1)
            self.stimulus_zoo_table.setHorizontalHeaderLabels(["Stimulus File"])
        else:
            # fill the stimulus zoo table with the data from the project directory
            self.stimulus_zoo_table.setRowCount(len(stimulus_zoo_files))
            self.stimulus_zoo_table.setColumnCount(1)
            self.stimulus_zoo_table.setHorizontalHeaderLabels(["Stimulus File"])
            for i in range(len(stimulus_zoo_files)):
                self.stimulus_zoo_table.setItem(i, 0, QtWidgets.QTableWidgetItem(stimulus_zoo_files[i]))
            self.stimulus_zoo_table.resizeColumnsToContents()
            self.stimulus_zoo_table.resizeRowsToContents()

        if len(experiment_zoo_files) == 0:
            # tell the user that there are no experiment zoo files
            QtWidgets.QMessageBox.information(
                self, "No Experiment Zoo Files", "There are no experiment zoo files in the project directory."
            )
            # reset the experiment zoo table
            self.experiment_zoo_table.setRowCount(0)
            self.experiment_zoo_table.setColumnCount(1)
            self.experiment_zoo_table.setHorizontalHeaderLabels(["Experiment File"])
        else:
            # fill the experiment zoo table with the data from the project directory
            self.experiment_zoo_table.setRowCount(len(experiment_zoo_files))
            self.experiment_zoo_table.setColumnCount(2)
            self.experiment_zoo_table.setHorizontalHeaderLabels(["Experiment File", "Type"])
            for i in range(len(experiment_zoo_files)):
                self.experiment_zoo_table.setItem(i, 0, QtWidgets.QTableWidgetItem(experiment_zoo_files[i]))
                self.experiment_zoo_table.setItem(
                    i, 1, QtWidgets.QTableWidgetItem(allowed_experiment_types[experiment_zoo_files[i].split(".")[-1]])
                )
            self.experiment_zoo_table.resizeColumnsToContents()
            self.experiment_zoo_table.resizeRowsToContents()

    def process_experimental_data(self):
        pass

    def generate_video(self):
        pass

    def create_new_stimulus(self):
        # check is the project directory is not empty
        if self.project_directory_textbox.text() == "":
            return
        # call the 16Y_stimulus_designer script and wait for it to finish
        call(
            ["python", "sixteeny/gui/16Y_stimulus_designer.py", self.project_directory_textbox.text() + "/stimulus_zoo"]
        )
        # refresh the project details
        self.refresh_project_details()

    def delete_stimulus(self):
        # check is the project directory is not empty
        if self.project_directory_textbox.text() == "":
            return
        # get the selected stimulus file
        selected_stimulus_file = self.stimulus_zoo_table.selectedItems()[0].text()
        # delete the stimulus file
        os.remove(self.project_directory_textbox.text() + "/stimulus_zoo/" + selected_stimulus_file)
        # refresh the project details
        self.refresh_project_details()

    def create_new_experiment(self):
        # check is the project directory is not empty
        if self.project_directory_textbox.text() == "":
            return
        # ask the user to select an experiment type between 2AFC and Finite World Dynamics
        types = ["2AFC", "Finite World Dynamics", "Custom CSV"]
        type_dialog = QtWidgets.QInputDialog()
        type_dialog.setInputMode(QtWidgets.QInputDialog.TextInput)
        type_dialog.setComboBoxItems(types)
        type_dialog.setLabelText("Select an experiment type:")
        type_dialog.setWindowTitle("Select Experiment Type")
        type_dialog.setOkButtonText("Select")
        type_dialog.setCancelButtonText("Cancel")
        type_dialog.exec_()
        if type_dialog.result() == QtWidgets.QDialog.Accepted:
            type = type_dialog.textValue()
        else:
            # tell the user that no experiment was created
            QtWidgets.QMessageBox.information(self, "No Experiment Created", "No experiment was created.")
            return
        if type == "2AFC":
            # call the 2AFC_experiment_designer script and wait for it to finish
            call(
                [
                    "python",
                    "sixteeny/gui/16Y_2AFC_designer.py",
                    self.project_directory_textbox.text() + "/experiment_zoo",
                ]
            )
        elif type == "Finite World Dynamics":
            # tell the user that the Finite World Dynamics experiment type is not yet implemented
            QtWidgets.QMessageBox.information(
                self,
                "Finite World Dynamics Not Implemented",
                "The Finite World Dynamics experiment type is not yet implemented.",
            )
        elif type == "Custom CSV":
            # ask the user to enter the name of the custom CSV file
            csv_dialog = QtWidgets.QInputDialog()
            csv_dialog.setInputMode(QtWidgets.QInputDialog.TextInput)
            csv_dialog.setLabelText("Enter the name of the custom CSV file:")
            csv_dialog.setWindowTitle("Enter Custom CSV File Name")
            csv_dialog.setOkButtonText("Enter")
            csv_dialog.setCancelButtonText("Cancel")
            csv_dialog.exec_()
            if csv_dialog.result() == QtWidgets.QDialog.Accepted:
                csv_file = csv_dialog.textValue()
            else:
                # tell the user that no experiment was created
                QtWidgets.QMessageBox.information(self, "No Experiment Created", "No experiment was created.")
                return
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
            df.to_csv(self.project_directory_textbox.text() + "/experiment_zoo/" + csv_file + ".csv")
            # open the custom CSV file in the default csv editor and wait for it to finish
            call(["open", self.project_directory_textbox.text() + "/experiment_zoo/" + csv_file + ".csv"])
        # remind the user to ensure correct stimulus files are used
        QtWidgets.QMessageBox.information(
            self,
            "Ensure Correct Stimulus Files Are Used",
            "Ensure that the correct stimulus files are used in the experiment.",
        )
        # refresh the project details
        self.refresh_project_details()

    def delete_experiment(self):
        # check is the project directory is not empty
        if self.project_directory_textbox.text() == "":
            return
        # get the selected experiment file
        selected_experiment_file = self.experiment_zoo_table.selectedItems()[0].text()
        # delete the experiment file
        os.remove(self.project_directory_textbox.text() + "/experiment_zoo/" + selected_experiment_file)
        # delete the experiment file's associated meta file
        os.remove(
            self.project_directory_textbox.text()
            + "/experiment_zoo/"
            + selected_experiment_file.split(".")[0]
            + ".meta"
        )
        # refresh the project details
        self.refresh_project_details()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())
