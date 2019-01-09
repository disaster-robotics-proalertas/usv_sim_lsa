import numpy as np
import csv
import matplotlib.pyplot as plt
import math

with open('polar_diagram_2.txt', 'rb') as file1, open('polar_diagram_2-5.txt', 'rb') as file2, open('polar_diagram_3_2.txt', 'rb') as file3, open('polar_diagram_3-5_2.txt', 'rb') as file4:
    reader1 = csv.reader(file1, delimiter=',')
    reader1 = list(reader1)
    reader1 = [list(x) for x in zip(*reader1)]

    reader2 = csv.reader(file2, delimiter=',')
    reader2 = list(reader2)
    reader2 = [list(x) for x in zip(*reader2)]

    reader3 = csv.reader(file3, delimiter=',')
    reader3 = list(reader3)
    reader3 = [list(x) for x in zip(*reader3)]

    reader4 = csv.reader(file4, delimiter=',')
    reader4 = list(reader4)
    reader4 = [list(x) for x in zip(*reader4)]

    r1 = [float(i) for i in reader1[0]]
    theta1 = [float(i) for i in reader1[2]]
    theta1 = [math.radians(i) for i in theta1]

    r2 = [float(i) for i in reader2[0]]
    theta2 = [float(i) for i in reader2[2]]
    theta2 = [math.radians(i) for i in theta2]

    r3 = [float(i) for i in reader3[0]]
    theta3 = [float(i) for i in reader3[2]]
    theta3 = [math.radians(i) for i in theta3]

    r4 = [float(i) for i in reader4[0]]
    theta4 = [float(i) for i in reader4[2]]
    theta4 = [math.radians(i) for i in theta4]

    ax = plt.subplot(111, projection='polar')
    ax.plot(theta1, r1)
    ax.plot(theta2, r2)
    ax.plot(theta3, r3)
    ax.plot(theta4, r4)
    ax.set_rmax(1.5)
    ax.set_rticks([0.5, 1, 1.5])  # less radial ticks
    ax.set_rlabel_position(-90)  # get radial labels away from plotted line
    ax.set_theta_zero_location("N")
    ax.grid(True)

    ax.set_title("Sailboat polar diagram on usv_sim", va='bottom')
    plt.legend(['2 m/s', '2,5 m/s', '3 m/s', '3,5 m/s'], loc='upper right')
    plt.show()
