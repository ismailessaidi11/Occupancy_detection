import pandas as pd
import numpy as np
from datetime import datetime
import statistics

def parse_datetime(ts):
    
    return datetime.strptime(ts, "%d-%m-%y %H:%M")

def average_feature(data_frame, window = 4):
    
# Calculate the rolling mean for each feature excluding the current index
    data_frame['average_co2_last_hour'] = data_frame['co2_concentration'].shift(1).rolling(window=window).mean()
    data_frame['average_temperature_last_hour'] = data_frame['temperature'].shift(1).rolling(window=window).mean()
    data_frame['average_humidity_last_hour'] = data_frame['humidity'].shift(1).rolling(window=window).mean()
    # Fill NaN values with the corresponding original values
    data_frame['average_co2_last_hour'] = data_frame['average_co2_last_hour'].fillna(data_frame['co2_concentration'])
    data_frame['average_temperature_last_hour'] = data_frame['average_temperature_last_hour'].fillna(data_frame['temperature'])
    data_frame['average_humidity_last_hour'] = data_frame['average_humidity_last_hour'].fillna(data_frame['humidity'])
    
    return data_frame

def current_minus_avg_last_hour(data_frame):
    data_frame['current_value_minus_average_last_hour_co2'] = data_frame['co2_concentration'] - data_frame['average_co2_last_hour']
    data_frame['current_value_minus_average_last_hour_temperature'] = data_frame['temperature'] - data_frame['average_temperature_last_hour']
    data_frame['current_value_minus_average_last_hour_humidity'] = data_frame['humidity'] -  data_frame['average_humidity_last_hour']
    return data_frame

def current_minus_last_measure(data_frame):
    data_frame["current_value_minus_last_15_min_co2"] = data_frame["co2_concentration"] - data_frame["co2_concentration"].shift(1)
    data_frame["current_value_minus_last_15_min_temperature"] = data_frame["temperature"] - data_frame["temperature"].shift(1)
    data_frame["current_value_minus_last_15_min_humidity"] = data_frame["humidity"] - data_frame["humidity"].shift(1)
    #fill NaN with 0 
    data_frame["current_value_minus_last_15_min_co2"] = data_frame["current_value_minus_last_15_min_co2"].fillna(0)
    data_frame["current_value_minus_last_15_min_temperature"] = data_frame["current_value_minus_last_15_min_temperature"].fillna(0)
    data_frame["current_value_minus_last_15_min_humidity"] = data_frame["current_value_minus_last_15_min_humidity"].fillna(0)
    return data_frame

def time_features(data_frame):
    data_frame['hour_of_the_day'] = data_frame['datetime'].dt.hour
    #data_frame['day_number_of_the_week'] = data_frame['datetime'].dt.dayofweek
    data_frame['day_of_year'] = data_frame['datetime'].dt.dayofyear
    return data_frame

file_path = "../Dataset/teraterm.log"

df = pd.DataFrame({"datetime":[],
                    "temperature":[],
                    "co2_concentration":[],
                    "humidity":[],
                    "room_number":[]})
chars_to_remove = ["#","\n"," "]
with open(file_path) as f:
    while True:
        line = f.readline()
        if not line:
            break
        
        line = str(line).split()
        ts = parse_datetime(line[0]+" "+line[1])
        
        new_row ={"datetime":ts,
                "temperature":float(line[2]),
                "co2_concentration":float(line[3]),
                "humidity":float(line[4]),
                "room_number":int(line[5]),
                "occupancy": int(line[6])}
        df.loc[len(df)] = new_row

#df['datetime'] = df['datetime'].dt.strftime('%d-%m-%y %H:%M')
window = 4
df = average_feature(df, window)
df = current_minus_avg_last_hour(df)
df = current_minus_last_measure(df)
df = time_features(df)
print(df)
#df.iloc[-8:].to_csv("output.csv", index=False)
