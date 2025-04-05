import os
import opendatasets as od
import pandas as pd


# Assign the Kaggle data set URL into variable
dataset = 'https://www.kaggle.com/datasets/ashwinsangareddypeta/bulls-eye-target-images-scraped-from-google/data'
# Using opendatasets let's download the data sets
od.download(dataset)