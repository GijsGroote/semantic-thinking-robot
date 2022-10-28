from robot_brain.global_planning.hgraph.hgraph import HGraph

from casadi import vertcat
from robot_brain.controller.mpc.mpc import Mpc
from robot_brain.controller.mppi.mppi import Mppi
from robot_brain.global_planning.hgraph.local_planning.graph_based.circle_robot_configuration_grid_map import (
    CircleRobotConfigurationGridMap,
)

from robot_brain.global_planning.kgraph.kgraph import KGraph
from robot_brain.global_planning.edge import Edge
from robot_brain.state import State

class PointRobotVelHGraph(HGraph):
    """
    Hypothesis graph for a Point Robot accepting velocity input.
    """
    def __init__(self, robot):
        HGraph.__init__(self)

        print("the pointrobot is now created")
        
        self.robot = robot
    
    def estimate_robot_path_existance(self, target_state, obstacles):

        occ_graph = CircleRobotConfigurationGridMap(cell_size=0.5,
                grid_x_length= 10,
                grid_y_length= 12,
                obstacles= obstacles,
                robot_cart_2d= self.robot.state.get_xy_position(),
                robot_radius= 0.4)
        

        occ_graph.setup()
        occ_graph.visualise()
        return occ_graph.shortest_path(self.robot.state.get_xy_position(), target_state.get_xy_position())


    def create_driving_controller(self):
        # TODO: randomly sample over existing controller for driving
        return self.create_mpc_driving_controller()

        # return self.create_mppi_driving_controller()


    def create_mppi_driving_controller(self):
        """ create MPPI controller for driving an point robot velocity. """
        controller = Mppi()

        def dyn_model(x, u):
            # TODO: this should simulate the dynamics 
            return x

        controller.setup(dyn_model, self.robot.state, self.robot.state)

        return controller


    def create_mpc_driving_controller(self):


        print('creating an mpc controller from inside the pointrobot thingy')
        controller = Mpc()
        # dyn_model = Dynamics()
        # dyn_model.set_boxer_model()
        def dyn_model(x, u):
            dx_next = vertcat(
                x[0] + 0.05 *  u[0],
                x[1] + 0.05 *  u[1],
                x[2],
            )
            return dx_next

        # TODO: should the target state not also be passed insead of the robot state?
        controller.setup(dyn_model, self.robot.state, self.robot.state)
        
        return controller


    def robot(self):
        # TODO: sanitize and make private
        return self.robot
