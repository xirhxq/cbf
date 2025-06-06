class PlotComponent:
    def setup(self, fig, gs, config):
        raise NotImplementedError

    def update(self, num):
        raise NotImplementedError