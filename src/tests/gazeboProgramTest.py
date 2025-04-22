from utils.sharedValues import sharedValues
from utils.trainGazebo import trainGazebo
from utils.continueTrainingGazebo import continueTrainingGazebo
from utils.saveDataFromTensorboardFiles import saveDataFromTensorboardFiles
import unittest
from unittest.mock import patch, MagicMock, mock_open
import os
import csv

class processConfigFileTest(unittest.TestCase):
    
    @classmethod
    def cleanUpFolders(cls):
        for folder in [
            "testSources/gazeboProgramTests/learnFunction/logFolder",
            "testSources/gazeboProgramTests/learnFunction/modelFolder",
            "testSources/gazeboProgramTests/learnFunction/csvFolder",
            "testSources/gazeboProgramTests/continueFunction/logFolder",
            "testSources/gazeboProgramTests/continueFunction/modelFolder",
            "testSources/gazeboProgramTests/continueFunction/csvFolder",
            "testSources/gazeboProgramTests/saveDataFunction/csvFolder"
        ]:
            if os.path.exists(folder):
                for file in os.listdir(folder):
                    os.remove(os.path.join(folder, file))

        csv_file_path = os.path.join("testSources/gazeboProgramTests/learnFunction/csvFolder", "data.csv")
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)

        csv_file_path = os.path.join("testSources/gazeboProgramTests/continueFunction/csvFolder", "data.csv")
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)

        csv_file_path = os.path.join("testSources/gazeboProgramTests/saveDataFunction/csvFolder", "data.csv")
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)

    def setUp(self):
        self.cleanUpFolders()

    def tearDown(self):
        self.cleanUpFolders()

    @patch('utils.trainGazebo.createDirectories')
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

    @patch('utils.continueTrainingGazebo.createDirectories')
    def continueFunctionWithSave(self, mock_start):
        print("test_continueFunctionWithSave started!")

        logFolder = "testSources/gazeboProgramTests/continueFunction/logFolder"
        modelFolder = "testSources/gazeboProgramTests/continueFunction/modelFolder"
        modelPath = "testSources/gazeboProgramTests/continueFunction/1000.zip"
        csvFilePath = "testSources/gazeboProgramTests/continueFunction/csvFolder/data.csv"
        
        sharedValues.setXGoal(5.0)
        sharedValues.setYGoal(5.0)
        sharedValues.setLearningModel("PPO")
        sharedValues.setLength(200)
        sharedValues.setSaveDataAfterFinished(True)
        sharedValues.setCSVFilePath(csvFilePath)
        sharedValues.setModelPath(modelPath)
        sharedValues.setLogFolder(logFolder)
        sharedValues.setModelFolder(modelFolder)

        continueTrainingGazebo()

        self.assertTrue(os.path.isdir(logFolder), "Log folder not found")
        self.assertTrue(os.path.isdir(modelFolder), "Model folder not found")
        self.assertGreater(len(os.listdir(logFolder)), 0, "Log folder is empty")
        self.assertGreater(len(os.listdir(modelFolder)), 0, "Model folder is empty")

        self.assertTrue(os.path.exists(csvFilePath), "CSV file does not exist")
        with open(csvFilePath, newline='') as csvfile:
            reader = csv.reader(csvfile)
            rows = list(reader)
            self.assertGreater(len(rows), 0, "CSV file is empty")

    @patch('utils.trainGazebo.createDirectories')
    @patch('utils.trainGazebo.customGazeboEnv')
    def learnFunctionWithoutSave(self, directory, mock_env_class):
        print("test_goodContinueWithoutSaveConfig started!")
        mock_env = MagicMock()
        mock_env.reset.return_value = ["dummy_obs"]
        mock_env_class.return_value = mock_env

        logFolder = "testSources/gazeboProgramTests/learnFunction/logFolder"
        modelFolder = "testSources/gazeboProgramTests/learnFunction/modelFolder"
        
        sharedValues.setXGoal(5.0)
        sharedValues.setYGoal(5.0)
        sharedValues.setLearningModel("PPO")
        sharedValues.setLength(200)
        sharedValues.setSaveDataAfterFinished(False)
        sharedValues.setLogFolder(logFolder)
        sharedValues.setModelFolder(modelFolder)

        trainGazebo()

        mock_env.reset.assert_called_once()

        self.assertTrue(os.path.isdir(logFolder), "Log folder not found")
        self.assertTrue(os.path.isdir(modelFolder), "Model folder not found")
        self.assertGreater(len(os.listdir(logFolder)), 0, "Log folder is empty")
        self.assertGreater(len(os.listdir(modelFolder)), 0, "Model folder is empty")

if __name__ == '__main__':
    unittest.main()