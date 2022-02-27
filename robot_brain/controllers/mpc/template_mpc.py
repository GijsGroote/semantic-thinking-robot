import do_mpc


def template_mpc(model, targetState, dt):
    # Obtain an instance of the do-mpc MPC class
    mpc = do_mpc.controller.MPC(model)

    # suppress output
    suppress_ipopt = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
    mpc.set_param(nlpsol_opts=suppress_ipopt)

    # Set parameters:
    setup_mpc = {
        'n_horizon': 20,
        't_step': dt,
        'n_robust': 1,
        'store_full_solution': True,
    }

    rterm_u1 = 1e-2
    rterm_u2 = 1e-2
    mpc.set_param(**setup_mpc)

    print("setting the target State")
    print(targetState.toString())

    mterm = (model._x["pos_x"]-targetState.pos[0]) ** 2 + (model._x["pos_y"]-targetState.pos[1]) ** 2 + 0.1*(model._x["ang_p"]-targetState.ang_p[2]) ** 2
    lterm = (model._x["pos_x"]-targetState.pos[0]) ** 2 + (model._x["pos_y"]-targetState.pos[1]) ** 2 + 0.1*(model._x["ang_p"]-targetState.ang_p[2]) ** 2
    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(
        u1=rterm_u1,
        u2=rterm_u2
    )

    # Lower bounds on states:
    # mpc.bounds['lower', '_x', 'ang_p'] = -2 * np.pi
    # Upper bounds on states
    # mpc.bounds['upper', '_x', 'ang_p'] = 2 * np.pi

    # Lower bounds on inputs:
    mpc.bounds['lower', '_u', 'u1'] = -1.5
    mpc.bounds['lower', '_u', 'u2'] = -1.5
    # upper bounds on inputs:
    mpc.bounds['upper', '_u', 'u1'] = 1.5
    mpc.bounds['upper', '_u', 'u2'] = 1.5

    mpc.setup()

    return mpc
