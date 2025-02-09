import tensorflow as tf
import os
import csv
import utils.commonvalues

def saveDataFromTensorboardFiles():
    csvFilename = utils.commonvalues.csvFilePath
    tensorboardDataPath = utils.commonvalues.tensorboardDataPath

    with open(csvFilename, 'w', newline='') as csvfile:
        fieldnames = ['step', 'value', 'tag']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        
        for event_file in os.listdir(tensorboardDataPath):
            if event_file.startswith('events.out.tfevents'):
                event_file_path = os.path.join(tensorboardDataPath, event_file)
                for e in tf.compat.v1.train.summary_iterator(event_file_path):
                    for v in e.summary.value:
                        if v.HasField('simple_value'):
                            writer.writerow({
                                'step': e.step,
                                'value': v.simple_value,
                                'tag': v.tag
                            })

    print(f"Data extracted to {csvFilename}")