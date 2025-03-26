from ompl import base as ob
from ompl import geometric as og

from ompl_planner.path_planner.state_validity_checker import StateValidityChecker

from enum import Enum

class PlannerType(Enum):
    RRT = 0
    RRTstar = 1
    PRM = 2
    LazyPRM = 3

class PathPlanner:
    def __init__(self, state_valid_checker: StateValidityChecker,
                 planner_type: PlannerType,
                 bounds: tuple[float, float],
                 start: tuple[float, float, float],
                 goal: tuple[float, float, float],
                 start_quat: tuple[float, float, float, float] = (0, 0, 0, 1),
                 goal_quat: tuple[float, float, float, float] = (0, 0, 0, 1)):
        self.state_valid_checker = state_valid_checker

        self.space = ob.SE3StateSpace()

        self.bounds = ob.RealVectorBounds(3)
        self.bounds.setLow(bounds[0])
        self.bounds.setHigh(bounds[1])

        self.space.setBounds(self.bounds)

        self.simple_setup = og.SimpleSetup(self.space)

        self.start = ob.State(self.space)
        self.set_start(start, start_quat)
        self.goal = ob.State(self.space)
        self.set_goal(goal, goal_quat)

        self.set_state_valid_checker(self.state_valid_checker)

        self.set_start_goal_states()

        self.planner = None
        self.set_planner(planner_type)


    def set_start(self, pos: tuple[float, float, float], quat: tuple[float, float, float, float]):
        self.start().setXYZ(pos[0], pos[1], pos[2])
        self.start().rotation().setAxisAngle(quat[0], quat[1], quat[2], quat[3])

    def set_goal(self, pos: tuple[float, float, float], quat = (0, 0, 0, 1)):
        self.goal().setXYZ(pos[0], pos[1], pos[2])
        self.goal().rotation().setAxisAngle(quat[0], quat[1], quat[2], quat[3])

    def set_state_valid_checker(self, state_valid_checker: StateValidityChecker):
        self.simple_setup.setStateValidityChecker(
            ob.StateValidityCheckerFn(state_valid_checker.isStateValid)
        )

    def set_start_goal_states(self):
        self.simple_setup.setStartAndGoalStates(self.start, self.goal, 0.05)

    def set_planner(self, planner_type: PlannerType):
        if planner_type == PlannerType.RRT:
            self.planner = og.RRT(self.simple_setup.getSpaceInformation())
        elif planner_type == PlannerType.RRTstar:
            self.planner = og.RRTstar(self.simple_setup.getSpaceInformation())
        elif planner_type == PlannerType.PRM:
            self.planner = og.PRM(self.simple_setup.getSpaceInformation())
        elif planner_type == PlannerType.LazyPRM:
            self.planner = og.LazyPRM(self.simple_setup.getSpaceInformation())

        self.simple_setup.setPlanner(self.planner)
        self.simple_setup.setup()

    def plan(self, time: float):
        solved = self.simple_setup.solve(time)
        if solved:
            return self.simple_setup.getSolutionPath()
        else:
            return None