import numpy as np
import csv
import matplotlib.pyplot as plt
import math

with open('polar_diagram_combined.txt', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    reader = list(reader)
    reader = [list(x) for x in zip(*reader)]

    r = [float(i) for i in reader[0]]
    print r
    theta = [float(i) for i in reader[2]]
    theta = [math.radians(i) for i in theta]
    print theta

    ax = plt.subplot(111, projection='polar')
    ax.plot(theta, r)
    ax.set_rmax(1)
    ax.set_rticks([1])  # less radial ticks
    ax.set_rlabel_position(-90)  # get radial labels away from plotted line
    ax.set_theta_zero_location("N")
    ax.grid(True)

    ax.set_title("A line plot on a polar axis", va='bottom')
    plt.show()

