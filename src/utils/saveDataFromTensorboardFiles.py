import tensorflow as tf
import os
import csv
from utils.sharedValues import sharedValues
import pandas as pd
from loguru import logger

def saveDataFromTensorboardFiles():
    sv = sharedValues()
    logger.info("csvFilePath: " + sv.csvFilePath)
    logger.info("logFolder: " + sv.logFolder)

    with open(sv.csvFilePath, 'w', newline='') as csvfile:
        fieldnames = ['step', 'value', 'tag']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        
        for event_file in os.listdir(sv.logFolder):
            if event_file.startswith('events.out.tfevents'):
                event_file_path = os.path.join(sv.logFolder, event_file)
                for e in tf.compat.v1.train.summary_iterator(event_file_path):
                    for v in e.summary.value:
                        if v.HasField('simple_value'):
                            writer.writerow({
                                'step': e.step,
                                'value': v.simple_value,
                                'tag': v.tag
                            })

    logger.info("Data extracted to : " + sv.csvFilePath)

    df = pd.read_csv(sv.csvFilePath)
    pivot_df = df.pivot_table(index='tag', columns='step', values='value', aggfunc='first')
    pivot_df.reset_index(inplace=True)
    pivot_df.to_csv(sv.csvFilePath, index=False)

    logger.info("Data transformed!")