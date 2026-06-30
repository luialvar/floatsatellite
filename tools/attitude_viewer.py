#!/usr/bin/env python3
"""Minimal serial 3D attitude viewer for the debug CSV printed by the firmware."""

import argparse
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation


angles = np.zeros(3)


def rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotation_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def reader(port, baud):
    global angles
    last_print = 0.0

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()
        while True:
            line = ser.readline().decode("ascii", "ignore").strip()
            if not line:
                continue

            try:
                yaw, pitch, roll = (float(value) for value in line.split(","))
            except ValueError:
                continue

            angles[:] = [pitch, yaw, roll]
            now = time.time()
            if now - last_print > 0.1:
                print(f"yaw={yaw:.3f} rad, pitch={pitch:.3f} rad, roll={roll:.3f} rad", flush=True)
                last_print = now


def main():
    parser = argparse.ArgumentParser(description="Live FloatSat attitude viewer")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", default=115200, type=int)
    args = parser.parse_args()

    threading.Thread(target=reader, args=(args.port, args.baud), daemon=True).start()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel("World X")
    ax.set_ylabel("World Y")
    ax.set_zlabel("World Z")

    x_line = ax.plot([0, 1], [0, 0], [0, 0], label="Body X")[0]
    y_line = ax.plot([0, 0], [0, 1], [0, 0], label="Body Y")[0]
    z_line = ax.plot([0, 0], [0, 0], [0, 1], label="Body Z")[0]
    lines = [x_line, y_line, z_line]
    ax.legend()

    def update(_):
        pitch, yaw, roll = angles
        rotation = rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)
        for index, line in enumerate(lines):
            vector = rotation[:, index]
            line.set_data([0, vector[0]], [0, vector[1]])
            line.set_3d_properties([0, vector[2]])
        return lines

    FuncAnimation(fig, update, interval=30)
    plt.show()


if __name__ == "__main__":
    main()
