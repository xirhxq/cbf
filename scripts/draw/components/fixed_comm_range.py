from utils import *

from .base import BaseComponent

class FixedCommRangeComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, mode='separate', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = title or f"Comm Distance, Robot #{robot_id + 1}"
        self.mode = mode

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
        self.ax.set_title(self.title)
        self.ax.set_xlabel('Time / s')
        self.ax.set_ylabel('Distance / m')

        for label in self.dists:
            times = [d['time'] for d in self.dists[label]]
            dists = [d['dist'] for d in self.dists[label]]
            line, = self.ax.plot(times, dists, label=label)
            self.lines[label] = line

        self.ax.legend(loc='best')

        if self.mode == 'animation':
            self.setup()

    def setup(self):
        self.vline = self.ax.plot([0, 0], [0, 1], 'r--', alpha=0.3)[0]
        self.y_limits = self.ax.get_ylim()

    def update(self, num, dataNow=None):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)
