import matplotlib.pyplot as plt


class Plotter(object):
    def __init__(self):
        self.figures = []
        self.subplots = []

    def figure(self, name):
        self.figures.append(name)
        self.subplots.append([])
        return self

    def subplot(self, title, xlabel, ylabel, x, y):
        self.subplots[-1].append((title, xlabel, ylabel, x, y))
        return self

    def plot(self):
        for fig_count, name in enumerate(self.figures):
            plt.figure(name)
            for count, subplot in enumerate(self.subplots[fig_count], 1):
                title, xlabel, ylabel, x, y = subplot
                plt.subplot(int(str(len(self.subplots[fig_count])) + '1' + str(count)))
                plt.plot(x, y)
                plt.ylabel(ylabel)
                plt.xlabel(xlabel)
                plt.title(title)
                plt.grid(True)

            plt.tight_layout()

        plt.show()
