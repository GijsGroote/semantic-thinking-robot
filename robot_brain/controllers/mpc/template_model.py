import numpy as np
import do_mpc

def template_model():
    # Obtain an instance of the do-mpc model class
    model_type = 'discrete'     # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # state variables
    pos_x = model.set_variable(var_type='_x', var_name='pos_x', shape=(1, 1))
    pos_y = model.set_variable(var_type='_x', var_name='pos_y', shape=(1, 1))
    ang_p = model.set_variable(var_type='_x', var_name='ang_p', shape=(1, 1))

    # dx = model.set_variable(var_type='_x', var_name='x_kp1', shape=(3, 1))

    # inputs
    u1 = model.set_variable(var_type='_u', var_name='u1', shape=(1, 1))
    u2 = model.set_variable(var_type='_u', var_name='u2', shape=(1, 1))

    # set right hand site, todo: this should come from the dynamics model
    model.set_rhs('pos_x', pos_x)
    model.set_rhs('pos_y', pos_y)
    model.set_rhs('ang_p', ang_p)

    # right hand side equation f(x)
    model.set_rhs('pos_x', pos_x + 0.101*np.cos(ang_p) * u1)
    model.set_rhs('pos_y', pos_y + 0.101*np.sin(ang_p) * u1)
    model.set_rhs('ang_p', ang_p + 0.05 * u2)

    model.setup()

    return model
