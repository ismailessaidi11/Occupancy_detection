import pandas as pd

def average_feature(data_frame, window = 4):
    
# Calculate the rolling mean for each feature excluding the current index
    data_frame['avg_CO2'] = data_frame['indoor_co2_concentration'].shift(1).rolling(window=window).mean()
    data_frame['avg_T'] = data_frame['indoor_operative_temperature'].shift(1).rolling(window=window).mean()
    data_frame['avg_hum'] = data_frame['indoor_relative_humidity'].shift(1).rolling(window=window).mean()
    # Fill NaN values with the corresponding original values
    data_frame['avg_CO2'].fillna(data_frame['indoor_co2_concentration'], inplace=True)
    data_frame['avg_T'].fillna(data_frame['indoor_operative_temperature'], inplace=True)
    data_frame['avg_hum'].fillna(data_frame['indoor_relative_humidity'], inplace=True)
    
    return data_frame

window = 4
# Example usage
data = {
    'indoor_co2_concentration': [400, 420, 430, 450, 440, 410, 420],
    'indoor_operative_temperature': [22.0, 22.1, 22.2, 22.3, 22.1, 22.0, 22.1],
    'indoor_relative_humidity': [45, 46, 47, 48, 47, 46, 45]
}

data_frame = pd.DataFrame(data)
result = average_feature(data_frame, window)
print(result)