import do_mpc

def template_simulator(model, dt):
    # Obtain an instance of the do-mpc simulator class
    simulator = do_mpc.simulator.Simulator(model)

    # Set parameter(s):
    simulator.set_param(t_step=dt)

    # Optional: Set function for parameters and time-varying parameters.

    # Setup simulator:
    simulator.setup()

    return simulator
