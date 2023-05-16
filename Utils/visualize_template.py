import csv
import numpy as np
from matplotlib import pyplot as plt


def visualize(fname: str, title: str, xlabel: str, ylabel: str):
    with open(fname, 'r') as f:
        reader = csv.reader(f)
        data = np.array(list(reader)[1:])
        data = data.astype(float)

        plt.plot(data[:, 0], data[:, 1])
        plt.grid()
        plt.minorticks_on()
        plt.grid(which='minor', linewidth='0.1', color='black')
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        # plt.legend([r"$\pi \lambda o \tau_1$"])
        # plt.xlim(left=10)
        # plt.semilogx(base=10)
        plt.show()


def main():
    files_dir = "D:\dir\with\csv\data"
    plots = {
        "data1.csv": "plot title",
    }
    for file_name, title in plots.items():
        visualize(files_dir + file_name, title,
                  xlabel=r'$xval$ $[units]$', ylabel=r'$yval$ $[units]$')


if __name__ == "__main__":
    main()
        