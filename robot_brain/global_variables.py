import plotly.express as px

CREATE_SERVER_DASHBOARD = True

PLOT_CONTROLLER = False


class Figures:

    def __init__(self):
        self.controllerPlot = px.scatter(x=[0, 1, 2, 3, 4], y=[0, 0, 0, 0, 0])

    @property
    def controllerPlot(self):
        return self._controllerPlot

    @controllerPlot.setter
    def controllerPlot(self, val):
        # check that the plot is actually a plot
        self._controllerPlot = val


figures = Figures()
