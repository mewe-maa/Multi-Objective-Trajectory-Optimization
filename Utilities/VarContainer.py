import numpy as np 
from pyomo.environ import ConcreteModel, TransformationFactory, Var, \
                          NonNegativeReals, Constraint, ConstraintList, \
                          SolverFactory, Objective, cos, sin, minimize, maximize,  \
                          NonNegativeReals, NegativeReals, Param, sqrt 

# Store the values of the optimal solution
class VarContainer():
    
    def __init__(self, m):

        # time
        self.t1 = m.t1 
        self.t2 = m.t2

        self.time1 = np.dot(m.t1,m.tf1() * m.tf_scale)
        self.time2 = np.dot(m.t2,m.tf2() * m.tf_scale) + m.tf1() * m.tf_scale
        self.time3 = np.dot(m.t3,m.tf3() * m.tf_scale) + m.tf1() * m.tf_scale + m.tf2() * m.tf_scale

        ## Phase 1
        self.x_1 = [m.x_1[t]() * m.x_scale * (1e-3) for t in m.t1]
        self.y_1 = [m.y_1[t]() * m.y_scale * (1e-3) for t in m.t1]
        self.z_1 = [m.z_1[t]() * m.z_scale * (1e-3) for t in m.t1]

        self.downrange_1 =  [np.sqrt((m.x_1[t]() * m.x_scale)**2 + (m.y_1[t]() * m.y_scale)**2) * (1e-3) for t in m.t1]

        self.xdot_1 = [m.xdot_1[t]() * m.x_scale * m.xdot_scale  for t in m.t1]
        self.ydot_1 = [m.ydot_1[t]() * m.y_scale * m.ydot_scale  for t in m.t1]
        self.zdot_1 = [m.zdot_1[t]() * m.z_scale * m.zdot_scale  for t in m.t1]

        self.u_1 = [m.u_1[t]() * m.u_scale for t in m.t1]
        self.v_1 = [m.v_1[t]() * m.v_scale for t in m.t1]
        self.w_1 = [m.w_1[t]() * m.w_scale for t in m.t1]

        self.udot_1 = [m.udot_1[t]() * m.u_scale * m.udot_scale for t in m.t1]
        self.vdot_1 = [m.vdot_1[t]() * m.v_scale * m.vdot_scale for t in m.t1]
        self.wdot_1 = [m.wdot_1[t]() * m.w_scale * m.wdot_scale for t in m.t1]

        self.phi_1 = [m.phi_1[t]() * m.phi_scale for t in m.t1]
        self.the_1 = [m.the_1[t]() * m.the_scale for t in m.t1]
        self.psi_1 = [m.psi_1[t]() * m.psi_scale for t in m.t1]

        self.p_1 = [m.p_1[t]() * m.p_scale for t in m.t1]
        self.q_1 = [m.q_1[t]() * m.q_scale for t in m.t1]
        self.r_1 = [m.r_1[t]() * m.r_scale for t in m.t1]

        self.eps_1 = [m.eps_1[t]() * m.eps_scale for t in m.t1]
        self.kap_1 = [m.kap_1[t]() * m.kap_scale for t in m.t1]

        self.mpdot_1 = [m.mpdot_1[t]() * m.mpdot_scale for t in m.t1]
        self.mass_1 = [m.mass_1[t]() * m.mass_scale for t in m.t1]


        # Phase 2
        self.x_2 = [m.x_2[t]() * m.x_scale * (1e-3) for t in m.t2]
        self.y_2 = [m.y_2[t]() * m.y_scale * (1e-3) for t in m.t2]
        self.z_2 = [m.z_2[t]() * m.z_scale * (1e-3) for t in m.t2]

        self.downrange_2 =  [np.sqrt((m.x_2[t]() * m.x_scale)**2 + (m.y_2[t]() * m.y_scale)**2) * (1e-3)for t in m.t2]

        self.xdot_2 = [m.xdot_2[t]() * m.x_scale * m.xdot_scale for t in m.t2]
        self.ydot_2 = [m.ydot_2[t]() * m.y_scale * m.ydot_scale for t in m.t2]
        self.zdot_2 = [m.zdot_2[t]() * m.z_scale * m.zdot_scale for t in m.t2]

        self.u_2 = [m.u_2[t]() * m.u_scale for t in m.t2]
        self.v_2 = [m.v_2[t]() * m.v_scale for t in m.t2]
        self.w_2 = [m.w_2[t]() * m.w_scale for t in m.t2]

        self.udot_2 = [m.udot_2[t]() * m.u_scale * m.udot_scale for t in m.t2]
        self.vdot_2 = [m.vdot_2[t]() * m.v_scale * m.vdot_scale for t in m.t2]
        self.wdot_2 = [m.wdot_2[t]() * m.w_scale * m.wdot_scale for t in m.t2]

        self.phi_2 = [m.phi_2[t]() * m.phi_scale for t in m.t2]
        self.the_2 = [m.the_2[t]() * m.the_scale for t in m.t2]
        self.psi_2 = [m.psi_2[t]() * m.psi_scale for t in m.t2]

        self.p_2 = [m.p_2[t]() * m.p_scale for t in m.t2]
        self.q_2 = [m.q_2[t]() * m.q_scale for t in m.t2]
        self.r_2 = [m.r_2[t]() * m.r_scale for t in m.t2]

        self.eps_2 = [m.eps_2[t]() * m.eps_scale for t in m.t2]
        self.kap_2 = [m.kap_2[t]() * m.kap_scale for t in m.t2]

        self.mpdot_2 = [m.mpdot_2[t]() * m.mpdot_scale for t in m.t2]
        self.mass_2 = [m.mass_2[t]() * m.mass_scale for t in m.t2]

        # Phase 3
        self.x_3 = [m.x_3[t]() * m.x_scale * (1e-3) for t in m.t3]
        self.y_3 = [m.y_3[t]() * m.y_scale * (1e-3) for t in m.t3]
        self.z_3 = [m.z_3[t]() * m.z_scale * (1e-3) for t in m.t3]

        self.downrange_3 =  [np.sqrt((m.x_3[t]() * m.x_scale)**2 + (m.y_3[t]() * m.y_scale)**2) * (1e-3)for t in m.t3]

        self.xdot_3 = [m.xdot_3[t]() * m.x_scale * m.xdot_scale for t in m.t3]
        self.ydot_3 = [m.ydot_3[t]() * m.y_scale * m.ydot_scale for t in m.t3]
        self.zdot_3 = [m.zdot_3[t]() * m.z_scale * m.zdot_scale for t in m.t3]

        self.u_3 = [m.u_3[t]() * m.u_scale for t in m.t3]
        self.v_3 = [m.v_3[t]() * m.v_scale for t in m.t3]
        self.w_3 = [m.w_3[t]() * m.w_scale for t in m.t3]

        self.udot_3 = [m.udot_3[t]() * m.u_scale * m.udot_scale for t in m.t3]
        self.vdot_3 = [m.vdot_3[t]() * m.v_scale * m.vdot_scale for t in m.t3]
        self.wdot_3 = [m.wdot_3[t]() * m.w_scale * m.wdot_scale for t in m.t3]

        self.phi_3 = [m.phi_3[t]() * m.phi_scale for t in m.t3]
        self.the_3 = [m.the_3[t]() * m.the_scale for t in m.t3]
        self.psi_3 = [m.psi_3[t]() * m.psi_scale for t in m.t3]
        
        self.p_3 = [m.p_3[t]() * m.p_scale for t in m.t3]
        self.q_3 = [m.q_3[t]() * m.q_scale for t in m.t3]
        self.r_3 = [m.r_3[t]() * m.r_scale for t in m.t3]

        self.eps_3 = [m.eps_3[t]() * m.eps_scale for t in m.t3]
        self.kap_3 = [m.kap_3[t]() * m.kap_scale for t in m.t3]

        self.mpdot_3 = [m.mpdot_3[t]() * m.mpdot_scale for t in m.t3]
        self.mass_3 = [m.mass_3[t]() * m.mass_scale for t in m.t3]

        return
    
    
        