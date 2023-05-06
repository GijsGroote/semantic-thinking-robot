import random

# not used any more
# def discrete_colorscale(bvals, colors):
#     """
#     bvals - list of values bounding intervals/ranges of interest
#     colors - list of rgb or hex colorcodes for values in [bvals[k], bvals[k+1]],0<=k < len(bvals)-1
#     returns the plotly  discrete colorscale
#     """
#     if len(bvals) != len(colors)+1:
#         raise ValueError('len(boundary values) should be equal to  len(colors)+1')
#     bvals = sorted(bvals)
#     nvals = [(v-bvals[0])/(bvals[-1]-bvals[0]) for v in bvals]  #normalized values
#
#     dcolorscale = [] #discrete colorscale
#     for (k, color) in enumerate(colors):
#         dcolorscale.extend([[nvals[k], color]])
#         dcolorscale.extend([[nvals[k+1], color]])
#     return dcolo wrscale

def get_random_color():
    """ return a random color. """
    return [random.random(), random.random(), random.random(), 1]
