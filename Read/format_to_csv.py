import numpy as np 
import pandas as pd 


# HVAC dataset
datatest = pd.read_csv("../Dataset/HVAC_datatest.txt")
data_val = pd.read_csv("../Dataset/HVAC_data_val.txt")
datatraining = pd.read_csv("../Dataset/HVAC_datatraining.txt")

tera = pd.read_csv("../Dataset/teraterm.log")
df = pd.concat([datatest, data_val, datatraining])
print(tera)