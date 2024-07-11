import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import numpy as np

def plotResults(VarContainer, m):
    
    plt.figure(1)
    plt.plot(VarContainer.time1, VarContainer.x_1, '.-', color='c', label='Launch')
    plt.plot(VarContainer.time2, VarContainer.x_2, '.-', color='r', label='First burn')
    plt.plot(VarContainer.time3, VarContainer.x_3, '.-', color='g' , label='Coast phase I')
    plt.xlabel('time [s]')
    plt.ylabel('Range [km]') 
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(2)
    plt.plot(VarContainer.time1, VarContainer.y_1, '.-', color='c', label='Launch')
    plt.plot(VarContainer.time2, VarContainer.y_2, '.-', color='r', label='First Burn')
    plt.plot(VarContainer.time3, VarContainer.y_3, '.-', color='g' , label='Coast Phase-I')
    plt.xlabel('time [s]')
    plt.ylabel('Displacement Y [km]')
    plt.grid()   

# #######################################################################################

    plt.figure(3)
    plt.plot(VarContainer.time1, VarContainer.z_1, '.-', color='c', label='Launch')
    plt.plot(VarContainer.time2, VarContainer.z_2, '.-', color='r', label='First Burn')
    plt.plot(VarContainer.time3, VarContainer.z_3, '.-', color='g' , label='Coast Phase-I')
    plt.xlabel('time [s]')
    plt.ylabel('Altitude [km]')
    plt.legend()
    plt.grid()   
    
# #######################################################################################

    plt.figure(4)
    plt.plot(VarContainer.time1, VarContainer.mass_1, '.-',  color='c', label='First burn')
    plt.plot(VarContainer.time2, VarContainer.mass_2, '.-',  color='r', label='Coast')
    plt.plot(VarContainer.time3, VarContainer.mass_3, '.-',  color='g', label='Second burn')
    plt.xlabel('time [s]')
    plt.ylabel('Total Vehicle Mass [kg]')
    plt.legend()   
    plt.grid()   


# #######################################################################################

    plt.figure(5)
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.kap_1), '--', linewidth=2, color='0', label=r'$\kappa$')
    plt.plot(VarContainer.time2, np.rad2deg(VarContainer.kap_2), '--', linewidth=2, color='0')
    plt.plot(VarContainer.time3, np.rad2deg(VarContainer.kap_3), '--', linewidth=2, color='0')

    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.eps_1), '--', linewidth=2, color='r', label=r'$\epsilon$')
    plt.plot(VarContainer.time2, np.rad2deg(VarContainer.eps_2), '--', linewidth=2, color='r')
    plt.plot(VarContainer.time3, np.rad2deg(VarContainer.eps_3), '--', linewidth=2, color='r')

    plt.xlabel('time [s]')
    plt.ylabel('Thrust Angles [deg]')
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(6)
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.phi_1), linestyle='--', linewidth=2, color='0', label=r'$\phi$')
    plt.plot(VarContainer.time2, np.rad2deg(VarContainer.phi_2), linestyle='--', linewidth=2, color='0')
    plt.plot(VarContainer.time3, np.rad2deg(VarContainer.phi_3), linestyle='--', linewidth=2, color='0')

    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.psi_1), linestyle='--', linewidth=2, color='r', label=r'$\psi$')
    plt.plot(VarContainer.time2, np.rad2deg(VarContainer.psi_2), linestyle='--', linewidth=2, color='r')
    plt.plot(VarContainer.time3, np.rad2deg(VarContainer.psi_3), linestyle='--', linewidth=2, color='r')
    
    plt.plot(VarContainer.time1, np.rad2deg(VarContainer.the_1), '--', linewidth=2, color='grey', label=r'$\theta$')
    plt.plot(VarContainer.time2, np.rad2deg(VarContainer.the_2), '--', linewidth=2, color='grey')
    plt.plot(VarContainer.time3, np.rad2deg(VarContainer.the_3), '--', linewidth=2, color='grey')

    plt.xlabel('time [s]')
    plt.ylabel('Roll, Pitch, and Yaw angles [deg]')
    plt.legend()
    plt.grid()

# #######################################################################################
    
    plt.figure(7)
    plt.plot(VarContainer.time1, VarContainer.u_1, '--', linewidth=2, color='0', label="u")
    plt.plot(VarContainer.time2, VarContainer.u_2, '--', linewidth=2, color='0')
    plt.plot(VarContainer.time3, VarContainer.u_3, '--', linewidth=2, color='0')

    plt.plot(VarContainer.time1, VarContainer.v_1, '--', linewidth=2, color='r', label="v")
    plt.plot(VarContainer.time2, VarContainer.v_2, '--', linewidth=2, color='r')
    plt.plot(VarContainer.time3, VarContainer.v_3, '--', linewidth=2, color='r')

    plt.plot(VarContainer.time1, VarContainer.w_1, '--', linewidth=2, color='grey', label="w")
    plt.plot(VarContainer.time2, VarContainer.w_2, '--', linewidth=2, color='grey')
    plt.plot(VarContainer.time3, VarContainer.w_3, '--', linewidth=2, color='grey')

    plt.xlabel('time [s]')
    plt.ylabel('Body  Velocity [m/s]') 
    plt.legend()
    plt.grid()

# #######################################################################################

    plt.figure(8)
    plt.plot(VarContainer.downrange_1,VarContainer.z_1, '.-',  color='c', label='First burn')
    plt.plot(VarContainer.downrange_2,VarContainer.z_2, '.-',  color='r', label='Coast')
    plt.plot(VarContainer.downrange_3,VarContainer.z_3, '.-',  color='g', label='Second burn')

    plt.xlabel('Downrange [km]')
    plt.ylabel('Altitude [km]')
    plt.legend()
    plt.grid()   

    return
