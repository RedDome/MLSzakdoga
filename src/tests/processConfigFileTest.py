import unittest
from unittest.mock import patch, MagicMock, mock_open
import yaml
from config.processConfigFile import processConfigFile, DEFAULTS
from utils.sharedValues import sharedValues

class processConfigFileTest(unittest.TestCase):
    
    @classmethod
    def cleanUpValues(cls):
        sharedValues.setXGoal(0.0)
        sharedValues.setYGoal(0.0)
        sharedValues.setLearningModel("")
        sharedValues.setLength(0)
        sharedValues.setSaveDataAfterFinished(False)
        sharedValues.setCSVFilePath("")

    def tearDown(self):
        self.cleanUpValues()
    
    @patch('config.processConfigFile.startFunction')
    def test_emptyYamlConfigTest(self, mock_start):
        print("test_emptyYamlConfigTest started!")
        path = "testSources/configProcessingTests/emptyConfigFile.yaml"

        with self.assertRaises(ValueError) as context:
            processConfigFile(path)

        self.assertIn("FunctionName is not defined", str(context.exception))

    @patch('config.processConfigFile.startFunction')
    def test_emptyFunctionNameConfig(self, mock_start):
        print("test_emptyFunctionNameConfig started!")
        path = "testSources/configProcessingTests/missingFunctionNameConfigFile.yaml"

        with self.assertRaises(ValueError) as context:
            processConfigFile(path)

        self.assertIn("FunctionName is not defined", str(context.exception))

    @patch('config.processConfigFile.startFunction')
    def test_emptyLogFolderConfig(self, mock_start):
        print("test_emptyLogFolderConfig started!")
        path = "testSources/configProcessingTests/missingLogFolderConfigFile.yaml"

        with self.assertRaises(ValueError) as context:
            processConfigFile(path)

        self.assertIn("LogFolder is not defined", str(context.exception))

    @patch('config.processConfigFile.startFunction')
    def test_emptyModelPathConfig(self, mock_start):
        print("test_emptyModelPathConfig started!")
        path = "testSources/configProcessingTests/missingModelPathConfigFile.yaml"

        with self.assertRaises(ValueError) as context:
            processConfigFile(path)

        self.assertIn("ModelPath is not defined", str(context.exception))

    @patch('config.processConfigFile.startFunction')
    def test_usingDefaultValuesConfig(self, mock_start):
        print("test_usingDefaultValuesConfig started!")
        path = "testSources/configProcessingTests/missingDefaultValuesConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.xGoal, 2.0)
        self.assertEqual(sharedValues.yGoal, 2.0)
        self.assertEqual(sharedValues.learningModel, "PPO")
        self.assertEqual(sharedValues.length, 8000)
        self.assertEqual(sharedValues.saveDataAfterFinished, False)

    @patch('config.processConfigFile.startFunction')
    def test_usingCsvFilePathDefaultValueConfig(self, mock_start):
        print("test_usingCsvFilePathDefaultValueConfig started!")
        path = "testSources/configProcessingTests/missingCsvFilePathConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.csvFilePath, "/workspaces/MLSzakdoga/resources/processedData/tensorboard_data.csv")

    @patch('config.processConfigFile.startFunction')
    def test_goodLearnWithSaveConfig(self, mock_start):
        print("test_goodLearnWithSaveConfig started!")
        path = "testSources/configProcessingTests/goodLearnWithSaveConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.functionName, "Learn")
        self.assertEqual(sharedValues.xGoal, 44.0)
        self.assertEqual(sharedValues.yGoal, 41.0)
        self.assertEqual(sharedValues.learningModel, "A2C")
        self.assertEqual(sharedValues.length, 17000)
        self.assertEqual(sharedValues.saveDataAfterFinished, True)
        self.assertEqual(sharedValues.csvFilePath, "/workspaces/MLSzakdoga/resources/processedData/mydata.csv")

    @patch('config.processConfigFile.startFunction')
    def test_goodLearnWithoutSaveConfig(self, mock_start):
        print("test_goodLearnWithoutSaveConfig started!")
        path = "testSources/configProcessingTests/goodLearnWithoutSaveConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.functionName, "Learn")
        self.assertEqual(sharedValues.xGoal, 44.0)
        self.assertEqual(sharedValues.yGoal, 41.0)
        self.assertEqual(sharedValues.learningModel, "A2C")
        self.assertEqual(sharedValues.length, 17000)
        self.assertEqual(sharedValues.saveDataAfterFinished, False)
        self.assertEqual(sharedValues.csvFilePath, "")

    @patch('config.processConfigFile.startFunction')
    def test_goodContinueWithSaveConfig(self, mock_start):
        print("test_goodContinueWithSaveConfig started!")
        path = "testSources/configProcessingTests/goodContinueWithSaveConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.functionName, "Continue")
        self.assertEqual(sharedValues.modelPath, "/workspaces/MLSzakdoga/resources/models/mycoolmodel.zip")
        self.assertEqual(sharedValues.xGoal, 1.0)
        self.assertEqual(sharedValues.yGoal, 1.0)
        self.assertEqual(sharedValues.learningModel, "TD3")
        self.assertEqual(sharedValues.length, 77000)
        self.assertEqual(sharedValues.saveDataAfterFinished, True)
        self.assertEqual(sharedValues.csvFilePath, "/workspaces/MLSzakdoga/resources/processedData/mycooltable.csv")

    @patch('config.processConfigFile.startFunction')
    def test_goodContinueWithoutSaveConfig(self, mock_start):
        print("test_goodContinueWithoutSaveConfig started!")
        path = "testSources/configProcessingTests/goodContinueWithoutSaveConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.functionName, "Continue")
        self.assertEqual(sharedValues.modelPath, "/workspaces/MLSzakdoga/resources/models/mycoolmodel.zip")
        self.assertEqual(sharedValues.xGoal, 1.0)
        self.assertEqual(sharedValues.yGoal, 1.0)
        self.assertEqual(sharedValues.learningModel, "TD3")
        self.assertEqual(sharedValues.length, 77000)
        self.assertEqual(sharedValues.saveDataAfterFinished, False)
        self.assertEqual(sharedValues.csvFilePath, "")

    @patch('config.processConfigFile.startFunction')
    def test_goodSaveDataConfig(self, mock_start):
        print("test_goodSaveDataConfig started!")
        path = "testSources/configProcessingTests/goodSaveDataConfigFile.yaml"

        processConfigFile(path)

        self.assertEqual(sharedValues.functionName, "SaveData")
        self.assertEqual(sharedValues.logFolder, "/workspaces/MLSzakdoga/resources/logs/mycoolfolder")
        self.assertEqual(sharedValues.csvFilePath, "/workspaces/MLSzakdoga/resources/processedData/datatable.csv")


if __name__ == '__main__':
    unittest.main()
