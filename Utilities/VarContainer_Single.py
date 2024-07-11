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

        self.time1 = np.dot(m.t1,m.tf1() * m.tf_scale)

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

        self.eps_1 = [m.eps_1[t]() * m.eps_scale for t in m.t1]
        self.kap_1 = [m.kap_1[t]() * m.kap_scale for t in m.t1]

        self.mpdot_1 = [m.mpdot_1[t]() * m.mpdot_scale for t in m.t1]
        self.mass_1 = [m.mass_1[t]() * m.mass_scale for t in m.t1]


        return
    
    
        