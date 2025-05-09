from utils.sharedValues import sharedValues
from utils.trainGazebo import trainGazebo
from utils.continueTrainingGazebo import continueTrainingGazebo
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
import unittest
from unittest.mock import patch, MagicMock, mock_open
import os
import csv
os.environ['TZ'] = 'Europe/Budapest'
import time
time.tzset()

class processConfigFileTest(unittest.TestCase):
    
    @classmethod
    def cleanUpFolders(cls):
        learnFunctionLogFolder = "testSources/gazeboProgramTests/learnFunction/logFolder"
        learnFunctionmodelFolder = "testSources/gazeboProgramTests/learnFunction/modelFolder"
        learnFunctionCsvFolder = "testSources/gazeboProgramTests/learnFunction/csvFolder"
        saveDataFunctionCsvFolder = "testSources/gazeboProgramTests/saveDataFunction/csvFolder"

        for directory in [learnFunctionLogFolder, learnFunctionmodelFolder, learnFunctionCsvFolder, saveDataFunctionCsvFolder]:
            if not os.path.exists(directory):
                os.makedirs(directory)
            else:
                for file in os.listdir(directory):
                    os.remove(os.path.join(directory, file))

        csv_file_path = os.path.join("testSources/gazeboProgramTests/learnFunction/csvFolder", "data.csv")
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)

        csv_file_path = os.path.join("testSources/gazeboProgramTests/saveDataFunction/csvFolder", "data.csv")
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)

    def setUp(self):
        self.cleanUpFolders()

    def tearDown(self):
        self.cleanUpFolders()

    @patch('utils.trainGazebo.createTrainingDirectories')
    def test_learnFunctionWithSave(self, mock_start):
        print("test_learnFunctionWithSave started!")

        logFolder = "testSources/gazeboProgramTests/learnFunction/logFolder"
        modelFolder = "testSources/gazeboProgramTests/learnFunction/modelFolder"
        csvFilePath = "testSources/gazeboProgramTests/learnFunction/csvFolder/data.csv"
        
        sharedValues.setXGoal(5.0)
        sharedValues.setYGoal(5.0)
        sharedValues.setLearningModel("PPO")
        sharedValues.setLength(200)
        sharedValues.setSaveDataAfterFinished(True)
        sharedValues.setCSVFilePath(csvFilePath)
        sharedValues.setLogFolder(logFolder)
        sharedValues.setModelFolder(modelFolder)

        trainGazebo()

        self.assertTrue(os.path.isdir(logFolder), "Log folder not found")
        self.assertTrue(os.path.isdir(modelFolder), "Model folder not found")
        self.assertGreater(len(os.listdir(logFolder)), 0, "Log folder is empty")
        self.assertGreater(len(os.listdir(modelFolder)), 0, "Model folder is empty")

        self.assertTrue(os.path.exists(csvFilePath), "CSV file does not exist")
        with open(csvFilePath, newline='') as csvfile:
            reader = csv.reader(csvfile)
            rows = list(reader)
            self.assertGreater(len(rows), 0, "CSV file is empty")

    def test_saveDataFunction(self):
        print("test_saveDataFunction started!")

        logFolder = "testSources/gazeboProgramTests/saveDataFunction/logFolder"
        csvFilePath = "testSources/gazeboProgramTests/saveDataFunction/csvFolder/data.csv"

        sharedValues.setCSVFilePath(csvFilePath)
        sharedValues.setLogFolder(logFolder)

        saveDataFromTensorboardFiles()

        self.assertTrue(os.path.isdir(logFolder), "Log folder not found")
        self.assertGreater(len(os.listdir(logFolder)), 0, "Log folder is empty")

        self.assertTrue(os.path.exists(csvFilePath), "CSV file does not exist")
        with open(csvFilePath, newline='') as csvfile:
            reader = csv.reader(csvfile)
            rows = list(reader)
            self.assertGreater(len(rows), 0, "CSV file is empty")

if __name__ == '__main__':
    unittest.main()