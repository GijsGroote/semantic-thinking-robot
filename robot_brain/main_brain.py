from robot_brain.RBrain import RBrain

from pynput import keyboard
from pynput.keyboard import Key, Listener


def on_press_outer(parent_conn):

    def on_press(key):
        # send the pressed key as input
        print("key pressed: {}".format(key))
        if key == 'w' or key == Key.up:
            parent_conn.send({"x": 1.0, "y": 0.0})
        if key == 's' or key == Key.down:
            parent_conn.send({"x": -1.0, "y": 0.0})
        if key == 'a' or key == Key.left:
            parent_conn.send({"x": 0.0, "y": 1.0})
        if key == 'd' or key == Key.right:
            parent_conn.send({"x": 0.0, "y": -1.0})

    return on_press



def on_release_outer(parent_conn):
    def on_press(key):
        # set input to 0 if key is released

        print("key released: {}".format(key))
        parent_conn.send({"x": 0.0, "y": 0.0})

    return on_press



def main_b(p, parent_conn):
    # initialize robot brain
    brain = RBrain(parent_conn)

    # receive static information of the world
    # stat_world_info = parent_conn.recv()
    stat_world_info = {"temp": "info"}

    # setup brain parameters of current world
    brain.setup(stat_world_info)

    # TODO: set a assignment/task for the robot
    listener = keyboard.Listener(
        on_press=on_press_outer(parent_conn),
        on_release=on_release_outer(parent_conn))
    listener.start()



    while p.is_alive():

        # keep on receiving information
        parent_conn.send({"x": 0.0, "y": 0.0})

        data = parent_conn.recv()

        # if data["kill_world_process"]:
        #     p.kill()
        #     break

        # # TODO: potentially already calculate a plan before it is accepted by the robot
        # if data["robot_accepts_input"] and brain.is_doing == "executing":
        #     # send input to the robot
        #     brain.send_input()
        #
        # elif data["robot_accepts_input"] and brain.is_doing == "nothing":
        #     # For the goal set, calculate a plan
        #     brain.calculate_plan()
        #
        # elif data["robot_accepts_input"] and brain.is_doing == "thinking":
        #     # update world that thinking of a plan is not yet done
        #     # todo: if the robot is thinking it will not be able to send this message
        #     parent_conn.send({"RBState": brain.is_doing, "x": 0, "y": 0})
        # elif data["robot_accepts_input"]:
        #     print("input error, undefined Brain State")
        #     # send zero input
        #     parent_conn.send({"x": 0, "y": 0})
        # else:
        #     # fallback uption
        #     print("the robot is not yet ready to receive input")


    # update user that this process terminates
    print("hey my child process died, I will now commit suicide")



