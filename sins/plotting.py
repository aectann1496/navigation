import matplotlib.pyplot as plt


class Plotter(object):
    def __init__(self):
        self.figures = []
        self.subplots = []

    def figure(self, name, size):
        self.figures.append((name, str(size[0]) + str(size[1])))
        self.subplots.append([])
        return self

    def subplot(self, title, xlabel, ylabel, x, y):
        self.subplots[-1].append((title, xlabel, ylabel, x, y))
        return self

    def plot(self):
        for fig_count, figure in enumerate(self.figures):
            name, size = figure
            plt.figure(name)
            for count, subplot in enumerate(self.subplots[fig_count], 1):
                title, xlabel, ylabel, x, y = subplot
                plt.subplot(int(size + str(count)))
                plt.plot(x, y)
                plt.ylabel(ylabel)
                plt.xlabel(xlabel)
                plt.title(title)
                plt.grid(True)

            plt.tight_layout()

        plt.show()
