#!/usr/bin/env python
import numpy as np
import csv
import matplotlib.pyplot as plt


def readCsv(file_path):
    x_error = np.array([])
    y_error = np.array([])
    yaw_error = np.array([])

    data_file = open(file_path, "r")
    csv_reader = csv.reader(data_file, delimiter=',')
    i = 0
    for row in csv_reader:
        if i == 0:
            i += 1
        else:
            x_error = np.append(x_error, float(row[0]))
            y_error = np.append(y_error, float(row[1]))
            yaw_error = np.append(yaw_error, float(row[2]))
            i += i

    data_file.close()
    return x_error, y_error, yaw_error


x_error, y_error, yaw_error = readCsv("../data/error.csv")

plt.plot(x_error, 'b', label='x_error')
plt.plot(y_error, 'r', label='y_error')
plt.plot(yaw_error, 'g', label='yaw_error')
plt.legend(loc="upper left")
plt.show()
