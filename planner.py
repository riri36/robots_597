# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1
PARABOLA=0; SIGMOID=1

import numpy as np

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner() #default is Parabola

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, shape):
                
        traj = {
            "PARABOLA": {
                "min": 0,
                "max": 1.5,
                "func": lambda x: x**2,
            },
            "SIGMOID": {
                "min": 0,
                "max": 2.5,
                "func": lambda x: 2 / (1 + np.exp(-2 * x)) - 1
            }
        }["SIGMOID"]

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        x = np.linspace(traj["min"], traj["max"], 10)
        y = np.vectorize(traj['func'])(x)

        return list(zip(x,y))


