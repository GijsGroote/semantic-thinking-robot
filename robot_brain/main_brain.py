from robot_brain.RBrain import RBrain


def main_b(p, parent_conn):
    # initialize robot brain
    brain = RBrain(parent_conn)

    # receive static information of the world
    stat_world_info = parent_conn.recv()

    # setup brain parameters of current world
    brain.setup(stat_world_info)

    # TODO: set a assignment/task for the robot

    while p.is_alive():
        # keep on receiving information
        data = parent_conn.recv()

        if data["kill_world_process"]:
            p.kill()
            break

        # TODO: potentially already calculate a plan before it is accepted by the robot
        if data["robot_accepts_input"] and brain.is_doing == "executing":
            # send input to the robot
            brain.send_input()

        elif data["robot_accepts_input"] and brain.is_doing == "nothing":
            # For the goal set, calculate a plan
            brain.calculate_plan()

        elif data["robot_accepts_input"] and brain.is_doing == "thinking":
            # update world that thinking of a plan is not yet done
            # todo: if the robot is thinking it will not be able to send this message
            parent_conn.send({"RBState": brain.is_doing, "x": 0, "y": 0})
        elif data["robot_accepts_input"]:
            print("input error, undefined Brain State")
            # send zero input
            parent_conn.send({"x": 0, "y": 0})
        else:
            # fallback uption
            print("the robot is not yet ready to receive input")


    # update user that this process terminates
    print("hey my child process died, I will now commit suicide")



