'''
This is a waypoint generator for circular roads.
'''

from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

def main():
    mode = raw_input("Point or Angle mode (p/a): ")

    # Point mode
    if mode == "p":
        point_mode()
    elif mode == "a":
        angle_mode()
    else:
        print("Invalid input.")
        main()

def point_mode():
    try:
        path = raw_input("Path mode (inner/outer/custom): ")
        if path == "inner":
            r = 101.5
        elif path == "outer":
            r = 104.5
        elif path == "custom":
            r = float(input("Radius in metres (103.67): "))
        else:
            print("Invalid input.")
            point_mode()
        
        points = int(input("Number of points: "))
    
    except:
        print("Invalid input.")
        point_mode()

    print("\nRadius: ", r, "\nPoints: ", points)

    theta = 0

    X = []
    Y = []

    angle = 2*np.pi/points
    runs = 0

    for n in np.arange(0, 2*np.pi, angle):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        theta = theta + angle
        X.append(x)
        Y.append(y)
        runs = runs + 1

    X.append(r)
    Y.append(0)
    print(X)
    print(Y)
    print("\n This program has looped ", runs, " times.")

    axis = {'X-axis': X, 'Y-axis': Y}
    df = pd.DataFrame(axis, columns= ['X-axis', 'Y-axis'])
    df.to_csv("waypoints.csv", index = False)
    plot_waypoints(df)

def angle_mode():
    try:
        path = raw_input("Path mode (inner/outer/custom): ")
        if path == "inner":
            r = 101.5
        elif path == "outer":
            r = 104.5
        elif path == "custom":
            r = float(input("Radius in metres (103.67): "))
        else:
            print("Invalid input.")
            point_mode()
            
        degrees = float(input("Angle in degrees: "))

    except:
        print("Invalid input.")
        angle_mode()

    print("\nRadius: ", r, "\nAngle: ", degrees)

    angle = np.radians(degrees)
    theta = 0

    X = []
    Y = []
    runs = int(2*np.pi/angle)

    for n in np.arange(0, 2*np.pi, angle):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        theta = theta + angle
        X.append(x)
        Y.append(y)

    X.append(r)
    Y.append(0)
    print(X)
    print(Y)
    print("\n This program has looped ", runs, " times.")

    axis = {'X-axis': X, 'Y-axis': Y}
    df = pd.DataFrame(axis, columns= ['X-axis', 'Y-axis'])
    df.to_csv("waypoints.csv", index = False)
    plot_waypoints(df)

def plot_waypoints(df):
    figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    x = df['X-axis']
    y = df['Y-axis']
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.xlim(-110, 110)
    plt.ylim(-110, 110)
    plt.plot(x, y)
    plt.show()
    
if __name__ == "__main__":
    main()
