<<<<<<< HEAD
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
=======
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

# #######################################################################################

    plt.figure(9)
    plt.plot(VarContainer.time1, VarContainer.udot_1, '--', linewidth=2, color='0', label="udot")
    plt.plot(VarContainer.time1, VarContainer.vdot_1, '--', linewidth=2, color='r', label="vdot")
    plt.plot(VarContainer.time1, VarContainer.wdot_1, '--', linewidth=2, color='grey', label="wdot")
    plt.xlabel('time [s]')
    plt.ylabel('Body  Acceleration [m^2/s]') 
    plt.legend()
    plt.grid()

    # #######################################################################################

    plt.figure(10)
    plt.plot(VarContainer.time1, VarContainer.xdot_1, '--', linewidth=2, color='0', label="xdot")
    plt.plot(VarContainer.time1, VarContainer.ydot_1, '--', linewidth=2, color='r', label="ydot")
    plt.plot(VarContainer.time1, VarContainer.zdot_1, '--', linewidth=2, color='grey', label="zdot")
    plt.xlabel('time [s]')
    plt.ylabel('Inertial Velocity [m/s]') 
    plt.legend()
    plt.grid()

    # #######################################################################################

    plt.figure(11)
    plt.plot(VarContainer.time1, VarContainer.p_1, '--', linewidth=2, color='0', label="p")
    plt.plot(VarContainer.time1, VarContainer.q_1, '--', linewidth=2, color='r', label="q")
    plt.plot(VarContainer.time1, VarContainer.r_1, '--', linewidth=2, color='grey', label="r")
    plt.xlabel('time [s]')
    plt.ylabel('Angular Velocity [rad/s]') 
    plt.legend()
    plt.grid()

    # #######################################################################################

    plt.figure(12)
    plt.plot(VarContainer.time1, VarContainer.mpdot_1, '--', linewidth=2, color='0', label="p")
    plt.xlabel('time [s]')
    plt.ylabel('Mass Flow Rate [kg/s]') 
    plt.legend()
    plt.grid()

    plt.figure(13)
    plt.plot(VarContainer.time1, VarContainer.q0_1, '--', linewidth=2, color='0', label="q0")
    plt.plot(VarContainer.time1, VarContainer.q1_1, '--', linewidth=2, color='r', label="q1")
    plt.plot(VarContainer.time1, VarContainer.q2_1, '--', linewidth=2, color='grey', label="q2")
    plt.plot(VarContainer.time1, VarContainer.q3_1, '--', linewidth=2, color='g', label="q3")
    plt.xlabel('time [s]')
    plt.ylabel('Quaternions') 
    plt.legend()
    plt.grid()

    
    # plt.figure(14)
    # plt.plot(VarContainer.time1, VarContainer.q0dot_1, '--', linewidth=2, color='0', label="q0")
    # plt.plot(VarContainer.time1, VarContainer.q1dot_1, '--', linewidth=2, color='r', label="q1")
    # plt.plot(VarContainer.time1, VarContainer.q2dot_1, '--', linewidth=2, color='grey', label="q2")
    # plt.plot(VarContainer.time1, VarContainer.q3dot_1, '--', linewidth=2, color='g', label="q3")
    # plt.xlabel('time [s]')
    # plt.ylabel('Quaternions') 
    # plt.legend()
    # plt.grid()





    return
>>>>>>> 90adc6a (Update MAV trajectory optimization code, remove obsolete files, add new adaptive notebooks)
