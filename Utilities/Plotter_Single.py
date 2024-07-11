import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import numpy as np

def plotResults(VarContainer, m):
    

    color_1 = 'r'
    color_2 = 'coral'
    color_3 = '#00A86B'
    color_4 = 'b'
    color_5 = 'coral'
    color_6 = 'dodgerblue'
    color_7 = 'darkgreen'
    
    plt.figure(1)
    plt.plot(VarContainer.time1, VarContainer.x_1, '.-', color='c', label='Launch')
    plt.xlabel('time [s]')
    plt.ylabel('Range [km]') 
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(2)
    plt.plot(VarContainer.time1, VarContainer.y_1, '.-', color='c', label='Launch')
    plt.xlabel('time [s]')
    plt.ylabel('Displacement Y [km]')
    plt.grid()   

# #######################################################################################

    plt.figure(3)
    plt.plot(VarContainer.time1, VarContainer.z_1, '.-', color='c', label='Launch')
    plt.xlabel('time [s]')
    plt.ylabel('Altitude [km]')
    plt.legend()
    plt.grid()   
    
# #######################################################################################

    plt.figure(4)
    plt.plot(VarContainer.time1, VarContainer.mass_1, '.-',  color='c', label='First burn')
    plt.xlabel('time [s]')
    plt.ylabel('Total Vehicle Mass [kg]')
    plt.legend()   
    plt.grid()   


# #######################################################################################

    plt.figure(5)
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.kap_1), '--', linewidth=2, color='0', label=r'$\kappa$')

    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.eps_1), '--', linewidth=2, color='r', label=r'$\epsilon$')

    plt.xlabel('time [s]')
    plt.ylabel('Thrust Angles [deg]')
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(6)
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.phi_1), linestyle='--', linewidth=2, color='0', label=r'$\phi$')

    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.psi_1), linestyle='--', linewidth=2, color='r', label=r'$\psi$')
    
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.the_1), '--', linewidth=2, color='grey', label=r'$\theta$')

    plt.xlabel('time [s]')
    plt.ylabel('Roll, Pitch, and Yaw angles [deg]')
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(7)
    plt.plot(VarContainer.time1, VarContainer.u_1, '--', linewidth=2, color='0', label="u")
    plt.plot(VarContainer.time1, VarContainer.v_1, '--', linewidth=2, color='r', label="v")
    plt.plot(VarContainer.time1, VarContainer.w_1, '--', linewidth=2, color='grey', label="w")
    plt.xlabel('time [s]')
    plt.ylabel('Body  Velocity [m/s]') 
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(8)
    plt.plot(VarContainer.downrange_1,VarContainer.z_1, '.-',  color='c', label='First burn')
    plt.xlabel('Downrange [km]')
    plt.ylabel('Altitude [km]')
    plt.legend()
    plt.grid()  

    return
