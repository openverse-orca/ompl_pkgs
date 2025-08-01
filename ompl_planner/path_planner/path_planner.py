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
                 low_bound: tuple[float, float, float], 
                 high_bound: list[float, float, float],
                 start: tuple[float, float, float],
                 goal: tuple[float, float, float],
                 start_quat: tuple[float, float, float, float] = (0, 0, 0, 1),
                 goal_quat: tuple[float, float, float, float] = (0, 0, 0, 1)):
        self.state_valid_checker = state_valid_checker

        self.space = ob.SE3StateSpace()

        self.bounds = ob.RealVectorBounds(3)
        self.set_bounds(low_bound, high_bound)

        self.space.setBounds(self.bounds)

        self.simple_setup = og.SimpleSetup(self.space)

        self.start = ob.State(self.space)
        self.set_start(start, start_quat)
        self.goal = ob.State(self.space)
        self.set_goal(goal, goal_quat)

        self.set_state_valid_checker(self.state_valid_checker)

        self.set_start_goal_states()

        self.planner = None
        self.solution_path = None
        self.set_planner(planner_type)

    def set_bounds(self, low_bound: tuple[float, float, float], high_bound: tuple[float, float, float]):
        self.bounds.setLow(0, low_bound[0])
        self.bounds.setHigh(0, high_bound[0])

        self.bounds.setLow(1, low_bound[1])
        self.bounds.setHigh(1, high_bound[1])

        self.bounds.setLow(2, low_bound[2])
        self.bounds.setHigh(2, high_bound[2])


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
            self.solution_path = self.simple_setup.getSolutionPath()
            print("Found solution:", self.solution_path)
        return solved
    
    def get_solution_path(self):
        return self.solution_path

    def interpolate_path(self, num_points: int):
        if self.solution_path is None:
            return None
        return self.solution_path.interpolate(num_points)

if __name__ == "__main__":
    pathPlanner = PathPlanner(StateValidityChecker(), PlannerType.RRTstar, (-10, -10, -10), (10, 10, 10), (0, 0, 0), (1, 1, 1))
    pathPlanner.plan(10)