import tensorflow as tf
import os
import csv
import utils.commonvalues as cm
import pandas as pd
from loguru import logger

def saveDataFromTensorboardFiles():
    logger.info("csvFilePath: " + cm.csvFilePath)
    logger.info("logFolder: " + cm.logFolder)

    with open(cm.csvFilePath, 'w', newline='') as csvfile:
        fieldnames = ['step', 'value', 'tag']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        
        for event_file in os.listdir(cm.logFolder):
            if event_file.startswith('events.out.tfevents'):
                event_file_path = os.path.join(cm.logFolder, event_file)
                for e in tf.compat.v1.train.summary_iterator(event_file_path):
                    for v in e.summary.value:
                        if v.HasField('simple_value'):
                            writer.writerow({
                                'step': e.step,
                                'value': v.simple_value,
                                'tag': v.tag
                            })

    logger.info("Data extracted to : " + cm.csvFilePath)

    df = pd.read_csv(cm.csvFilePath)
    pivot_df = df.pivot_table(index='tag', columns='step', values='value', aggfunc='first')
    pivot_df.reset_index(inplace=True)
    pivot_df.to_csv(cm.csvFilePath, index=False)

    logger.info("Data transformed!")