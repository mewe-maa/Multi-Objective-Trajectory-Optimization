import numpy as np

# rocket length
l = 2.89      

# surface area
S = 0.07068375

# inertia
Ix = 7.48
Iy = 70.85
Iz = 70.85

# mars planet specifications
miu_mars = 4.282837e13
mars_radius = 3.3895e3
mass = 191

# gimble deflection distance from center of gravity
d = -1.58

def velocity(altitude):
    return np.sqrt(miu_mars / (mars_radius + altitude))

