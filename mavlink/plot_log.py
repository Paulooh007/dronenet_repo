import matplotlib.pyplot as plt
from datetime import datetime
import csv
import os




def plot_altitude_data(filename):
    altitudes = []
    relative_altitudes = []
    times = []

    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            altitudes.append(float(row['Altitude']))
            relative_altitudes.append(float(row['Relative Altitude']))
            time_str = row['Time']
            time = datetime.strptime(time_str, '%Y%m%d%H%M%S')
            times.append(time)

    plt.plot(times, altitudes, label='Altitude')
    plt.plot(times, relative_altitudes, label='Relative Altitude')
    plt.xlabel('Time')
    plt.ylabel('Altitude/Relative Altitude')
    plt.title('Altitude Data')
    plt.legend()

    # Save the plot as a PNG file with the same filename
    base_path = os.path.splitext(filename)[0]
    save_path = f"{base_path}.png"
    plt.savefig(save_path)
    plt.close() 


plot_altitude_data("altitude_data_20230614113522.csv")