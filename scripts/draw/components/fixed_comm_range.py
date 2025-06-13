from utils import *

from .base import PlotComponent

class FixedCommRangeComponent(PlotComponent):
    def __init__(self, ax, data, robot_id, name=None):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.name = name or f"Robot #{robot_id + 1} Comm Distance"

        runtime = [dt["runtime"] for dt in self.data]
        id2Position = [
            {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in frame["robots"]}
            for frame in self.data
        ]
        self.runtime = runtime
        self.id2Position = id2Position

        self.dists = {}
        for idx, frame in enumerate(self.data):
            myId = robot_id + 1
            myFormation = frame.get("formation", [])
            if not isinstance(myFormation, list):
                myFormation = [myFormation]

            for fm in myFormation:
                if fm["id"] == myId:
                    myState = frame["robots"][robot_id]["state"]
                    myX, myY = myState["x"], myState["y"]

                    for anchorId in fm.get("anchorIds", []):
                        anchorText = f'To UAV #{anchorId}'
                        if anchorText not in self.dists:
                            self.dists[anchorText] = []

                        neighbourPos = id2Position[idx].get(anchorId, None)
                        if neighbourPos is not None:
                            dist = np.sqrt((myX - neighbourPos[0])**2 + (myY - neighbourPos[1])**2)
                            self.dists[anchorText].append({"dist": dist, "time": frame["runtime"]})

                    for anchorPoint in fm.get("anchorPoints", []):
                        pos_str = f'({anchorPoint[0]:.1f}, {anchorPoint[1]:.1f})'
                        anchorText = f'To {pos_str}'
                        if anchorText not in self.dists:
                            self.dists[anchorText] = []
                        dist = np.sqrt(
                            (myX - anchorPoint[0]) ** 2 +
                            (myY - anchorPoint[1]) ** 2
                        )
                        self.dists[anchorText].append({"dist": dist, "time": frame["runtime"]})

        self.lines = {}
        self._initialize_plot()

    def _initialize_plot(self):
        self.ax.set_title(self.name)
        self.ax.set_xlabel('Time / s')
        self.ax.set_ylabel('Distance / m')

        for label in self.dists:
            times = [d['time'] for d in self.dists[label]]
            dists = [d['dist'] for d in self.dists[label]]
            line, = self.ax.plot(times, dists, label=label)
            self.lines[label] = line

        self.ax.legend(loc='best')

    def setup(self, fig, gs, config=None):
        pass

    def update(self, num, dataNow=None):
        self.ax.clear()
        self._initialize_plot()
        self.ax.axvline(x=self.runtime[num], color='red', linestyle='--', alpha=0.3)
