from pyomo.dae import ContinuousSet, DerivativeVar, Integral
from pyomo.environ import ConcreteModel, TransformationFactory, Var, \
                          NonNegativeReals, Constraint, ConstraintList, \
                          SolverFactory, Objective, cos, sin, tan, sqrt, atan, asin, minimize,  \
                          NonNegativeReals, NegativeReals, Param
from pyomo.environ import *
from pyomo.dae import *
import numpy as np
import matplotlib.pyplot as plt

from Utilities.Phase_Variables_2 import getPhaseVariables
import Aerodynamics as aero
import Propulsion as prop 
import Parameters as param
import Equations as eom
import Atmospheric as atm

m = ConcreteModel("MAV")

class MAV():
    
    def __init__(self):
    
        super().__init__()
        
        # model
        self.m               = ConcreteModel('MAV')

        # bounds
        self.m.x_max         = Param(initialize = 5000e3)
        self.m.y_max         = Param(initialize = 5000e3)
        self.m.z_max         = Param(initialize = 250e3)

        self.m.x_min         = Param(initialize = 0.01)
        self.m.y_min         = Param(initialize = 0.01) 
        self.m.z_min         = Param(initialize = 0.01)

        self.m.xdot_min      = Param(initialize = 0.01)
        self.m.ydot_min      = Param(initialize = 0.01)
        self.m.zdot_min      = Param(initialize = 0.01)

        self.m.xdot_max      = Param(initialize = 3451)
        self.m.ydot_max      = Param(initialize = 3451)
        self.m.zdot_max      = Param(initialize = 3451)

        self.m.u_min         = Param(initialize = -500)
        self.m.v_min         = Param(initialize = -20)
        self.m.w_min         = Param(initialize = -20)

        self.m.u_max         = Param(initialize = 3350)
        self.m.v_max         = Param(initialize = 20)       
        self.m.w_max         = Param(initialize = 20)

        self.m.udot_min      = Param(initialize = -200)
        self.m.vdot_min      = Param(initialize = -50)
        self.m.wdot_min      = Param(initialize = -50)

        self.m.udot_max      = Param(initialize = 500)
        self.m.vdot_max      = Param(initialize = 50)
        self.m.wdot_max      = Param(initialize = 50)
    
        self.m.phi_min       = Param(initialize = -np.pi)
        self.m.the_min       = Param(initialize = -np.pi/2)
        self.m.psi_min       = Param(initialize = -np.pi)
        
        self.m.phi_max       = Param(initialize = np.pi)
        self.m.the_max       = Param(initialize = np.pi / 2)
        self.m.psi_max       = Param(initialize = np.pi)

        self.m.p_min         = Param(initialize = -5)
        self.m.q_min         = Param(initialize = -5)
        self.m.r_min         = Param(initialize = -5)

        self.m.p_max         = Param(initialize = 5)
        self.m.q_max         = Param(initialize = 5)
        self.m.r_max         = Param(initialize = 5)

        self.m.quat_min      = Param(initialize = -2.0)
        self.m.quat_max      = Param(initialize = 2.0)

        self.m.q0_min        = Param(initialize = -20.0)
        self.m.q1_min        = Param(initialize = -20.0)
        self.m.q2_min        = Param(initialize = -20.0)
        self.m.q3_min        = Param(initialize = -20.0)

        self.m.q0_max        = Param(initialize = 20.0)
        self.m.q1_max        = Param(initialize = 20.0)
        self.m.q2_max        = Param(initialize = 20.0)
        self.m.q3_max        = Param(initialize = 20.0)

        self.m.mass_min      = Param(initialize = 48)
        self.m.mass_max      = Param(initialize = 291)

        self.m.mfuel_min      = Param(initialize = 1e-8)
        self.m.mfuel_max      = Param(initialize = 241)

        self.m.mpayload_min   = Param(initialize = 2)
        self.m.mpayload_max   = Param(initialize = 241)

        self.m.mpdot_min     = Param(initialize = 0.00001)
        self.m.mpdot_max     = Param(initialize = 7.5)

        self.m.massdot_min   = Param(initialize = -0.0001)
        self.m.massdot_max   = Param(initialize = -7.5)

        self.m.eps_min       = Param(initialize = -0.261799)
        self.m.eps_max       = Param(initialize = 0.261799)

        self.m.kap_min       = Param(initialize = -0.261799)
        self.m.kap_max       = Param(initialize = 0.261799)

        self.m.delta_min       = Param(initialize = -0.261799)
        self.m.delta_max       = Param(initialize = 0.261799)

        self.m.gamma_min       = Param(initialize = -0.261799)
        self.m.gamma_max       = Param(initialize = 0.261799)
        
        self.m.alpha_min     = Param(initialize = -0.261799)
        self.m.alpha_max     = Param(initialize =  0.261799)
        
        self.m.beta_min      = Param(initialize =  -0.261799)
        self.m.beta_max      = Param(initialize =  0.261799)
        
        # scaling
        self.m.x_scale      = Param(initialize = 500e3)
        self.m.y_scale      = Param(initialize = 500e3)
        self.m.z_scale      = Param(initialize = 200e3)

        self.m.xdot_scale   = Param(initialize = 0.005)
        self.m.ydot_scale   = Param(initialize = 0.005)
        self.m.zdot_scale   = Param(initialize = 0.005)

        self.m.u_scale      = Param(initialize = 600)
        self.m.v_scale      = Param(initialize = 890)
        self.m.w_scale      = Param(initialize = 890)

        self.m.udot_scale   = Param(initialize = 0.005)
        self.m.vdot_scale   = Param(initialize = 0.005)
        self.m.wdot_scale   = Param(initialize = 0.005)

        self.m.phi_scale     = Param(initialize = 1)
        self.m.the_scale     = Param(initialize = 1)
        self.m.psi_scale     = Param(initialize = 1)

        self.m.phidot_scale     = Param(initialize = 0.005)
        self.m.thedot_scale     = Param(initialize = 0.005)
        self.m.psidot_scale     = Param(initialize = 0.005)

        self.m.p_scale      = Param(initialize = 0.3)
        self.m.q_scale      = Param(initialize = 0.3)
        self.m.r_scale      = Param(initialize = 0.3)

        self.m.pdot_scale   = Param(initialize = 0.005)
        self.m.qdot_scale   = Param(initialize = 0.005)
        self.m.rdot_scale   = Param(initialize = 0.005)

        self.m.q0_scale     = Param(initialize = 1)
        self.m.q1_scale     = Param(initialize = 1)
        self.m.q2_scale     = Param(initialize = 1)
        self.m.q3_scale     = Param(initialize = 1)

        self.m.q0dot_scale  = Param(initialize = 0.005)
        self.m.q1dot_scale  = Param(initialize = 0.005)
        self.m.q2dot_scale  = Param(initialize = 0.005)
        self.m.q3dot_scale  = Param(initialize = 0.005)

        self.m.mass_scale    = Param(initialize = 291)
        self.m.massdot_scale = Param(initialize = 0.005)
        self.m.mpayload_scale    = Param(initialize = 180)
        self.m.mfuel_scale    = Param(initialize = 150)

        self.m.mpdot_scale   = Param(initialize = 1)

        self.m.eps_scale     = Param(initialize = 1)
        self.m.kap_scale     = Param(initialize = 1)

        self.m.delta_scale     = Param(initialize = 1)
        self.m.gamma_scale     = Param(initialize = 1)

        self.m.alpha_scale   = Param(initialize = 1)
        self.m.beta_scale    = Param(initialize = 1)

        self.m.tf_scale    = Param(initialize = 1)

        #=============Phase 1===========================================

        # time
        self.m.t1  = ContinuousSet(bounds=(0,1))
        self.m.tf1 = Var(initialize=12, bounds=(10 / self.m.tf_scale , 250 / self.m.tf_scale ))

        # inertial displacement state variables
        self.m.x_1 = Var(self.m.t1, bounds=(self.m.x_min / self.m.x_scale, self.m.x_max / self.m.x_scale))
        self.m.y_1 = Var(self.m.t1, bounds=(self.m.y_min / self.m.y_scale, self.m.y_max / self.m.y_scale))
        self.m.z_1 = Var(self.m.t1, bounds=(self.m.z_min / self.m.z_scale, self.m.z_max / self.m.z_scale))

        self.m.dx_dtau_1 = DerivativeVar(self.m.x_1, wrt=self.m.t1)
        self.m.dy_dtau_1 = DerivativeVar(self.m.y_1, wrt=self.m.t1) 
        self.m.dz_dtau_1 = DerivativeVar(self.m.z_1, wrt=self.m.t1) 

        # inertial velocity state variables
        self.m.xdot_1 = Var(self.m.t1, bounds=(self.m.xdot_min / self.m.x_scale / self.m.xdot_scale, None))
        self.m.ydot_1 = Var(self.m.t1, bounds=(self.m.ydot_min / self.m.y_scale / self.m.ydot_scale, None))
        self.m.zdot_1 = Var(self.m.t1)
                          
        # body angular rates 
        self.m.phi_1 = Var(self.m.t1, bounds=(self.m.phi_min / self.m.phi_scale, self.m.phi_max / self.m.phi_scale))
        self.m.the_1 = Var(self.m.t1, bounds=(self.m.the_min / self.m.the_scale, self.m.the_max / self.m.the_scale))
        self.m.psi_1 = Var(self.m.t1, bounds=(self.m.psi_min / self.m.psi_scale, self.m.psi_max / self.m.psi_scale))
        
        self.m.dphi_dtau_1 = DerivativeVar(self.m.phi_1, wrt=self.m.t1, initialize=0.0)
        self.m.dthe_dtau_1 = DerivativeVar(self.m.the_1, wrt=self.m.t1, initialize=0.0)
        self.m.dpsi_dtau_1 = DerivativeVar(self.m.psi_1, wrt=self.m.t1, initialize=0.0)

        # attitude angles
        self.m.phidot_1 = Var(self.m.t1, initialize=0.0)
        self.m.thedot_1 = Var(self.m.t1, initialize=0.0)
        self.m.psidot_1 = Var(self.m.t1, initialize=0.0)
        
        # body velocity state variables
        self.m.u_1 = Var(self.m.t1, bounds=(self.m.u_min / self.m.u_scale, None))
        self.m.v_1 = Var(self.m.t1, bounds=(self.m.v_min / self.m.v_scale, self.m.v_max / self.m.v_scale))
        self.m.w_1 = Var(self.m.t1, bounds=(self.m.w_min / self.m.w_scale, self.m.w_max / self.m.w_scale))

        self.m.du_dtau_1 = DerivativeVar(self.m.u_1, wrt=self.m.t1)
        self.m.dv_dtau_1 = DerivativeVar(self.m.v_1, wrt=self.m.t1)
        self.m.dw_dtau_1 = DerivativeVar(self.m.w_1, wrt=self.m.t1)

        # body acceleration state variables
        self.m.udot_1 = Var(self.m.t1)
        self.m.vdot_1 = Var(self.m.t1, bounds=(self.m.vdot_min / self.m.v_scale /  self.m.vdot_scale, self.m.vdot_max / self.m.v_scale /  self.m.vdot_scale,))
        self.m.wdot_1 = Var(self.m.t1, bounds=(self.m.wdot_min / self.m.w_scale /  self.m.wdot_scale, self.m.wdot_max / self.m.w_scale /  self.m.wdot_scale,))

        # attitude rate wrt to body axis state variables
        self.m.p_1 = Var(self.m.t1)
        self.m.q_1 = Var(self.m.t1)
        self.m.r_1 = Var(self.m.t1)

        self.m.dp_dtau_1 = DerivativeVar(self.m.p_1, wrt=self.m.t1)
        self.m.dq_dtau_1 = DerivativeVar(self.m.q_1, wrt=self.m.t1)
        self.m.dr_dtau_1 = DerivativeVar(self.m.r_1, wrt=self.m.t1)

        # attitude acceleration wrt to body axis state variables
        self.m.pdot_1 = Var(self.m.t1)
        self.m.qdot_1 = Var(self.m.t1)
        self.m.rdot_1 = Var(self.m.t1)

        # quaternion representation state variables
        self.m.q0_1 = Var(self.m.t1, bounds=(-1,1))
        self.m.q1_1 = Var(self.m.t1, bounds=(-1,1))
        self.m.q2_1 = Var(self.m.t1, bounds=(-1,1))
        self.m.q3_1 = Var(self.m.t1, bounds=(-1,1))
        
        self.m.dq0_dtau_1 = DerivativeVar(self.m.q0_1, wrt=self.m.t1)
        self.m.dq1_dtau_1 = DerivativeVar(self.m.q1_1, wrt=self.m.t1)
        self.m.dq2_dtau_1 = DerivativeVar(self.m.q2_1, wrt=self.m.t1)
        self.m.dq3_dtau_1 = DerivativeVar(self.m.q3_1, wrt=self.m.t1)

        self.m.q0dot_1 = Var(self.m.t1)
        self.m.q1dot_1 = Var(self.m.t1)
        self.m.q2dot_1 = Var(self.m.t1)
        self.m.q3dot_1 = Var(self.m.t1)

        # mass properties state variables
        self.m.mass_1     = Var(self.m.t1, bounds=(self.m.mass_min / self.m.mass_scale, self.m.mass_max / self.m.mass_scale))
        self.m.mpayload = Var(bounds=(self.m.mpayload_min / self.m.mpayload_scale, self.m.mpayload_max / self.m.mpayload_scale))
        self.m.mfuel_1 = Var(self.m.t1, bounds=(self.m.mfuel_min / self.m.mfuel_scale, self.m.mfuel_max / self.m.mfuel_scale))
        self.m.massdot_1  = Var(self.m.t1, bounds=(self.m.massdot_max/ self.m.mfuel_scale / self.m.massdot_scale, -0.65 / self.m.mfuel_scale / self.m.massdot_scale))
        self.m.dmass_dtau_1 = DerivativeVar(self.m.mfuel_1, wrt=self.m.t1)

        # thrust representation control parameters
        self.m.kap_1    = Var(self.m.t1, initialize = self.m.kap_max / self.m.kap_scale, bounds=(self.m.kap_min / self.m.kap_scale, self.m.kap_max / self.m.kap_scale))
        self.m.eps_1    = Var(self.m.t1, initialize = self.m.kap_max / self.m.kap_scale, bounds=(self.m.eps_min / self.m.eps_scale, self.m.eps_max / self.m.eps_scale))
        self.m.mpdot_1  = Var(self.m.t1, bounds=(self.m.mpdot_min / self.m.mpdot_scale, self.m.mpdot_max / self.m.mpdot_scale))

        self.m.delta_1    = Var(self.m.t1, initialize = self.m.delta_max / self.m.delta_scale, bounds=(self.m.delta_min / self.m.delta_scale, self.m.delta_max / self.m.delta_scale))
        self.m.gamma_1    = Var(self.m.t1, initialize = self.m.gamma_max / self.m.gamma_scale, bounds=(self.m.gamma_min / self.m.gamma_scale, self.m.gamma_max / self.m.gamma_scale))

        #=============Phase 2===========================================
        
        # time
        self.m.t2  = ContinuousSet(bounds=(0,1))
        self.m.tf2 = Var( bounds=(1 / self.m.tf_scale , 1000 / self.m.tf_scale ))

        # inertial displacement state variables
        self.m.x_2 = Var(self.m.t2, bounds=(self.m.x_min / self.m.x_scale, self.m.x_max / self.m.x_scale))
        self.m.y_2 = Var(self.m.t2, bounds=(self.m.y_min / self.m.y_scale, self.m.y_max / self.m.y_scale))
        self.m.z_2 = Var(self.m.t2, bounds=(self.m.z_min / self.m.z_scale, self.m.z_max / self.m.z_scale))

        self.m.dx_dtau_2 = DerivativeVar(self.m.x_2, wrt=self.m.t2)
        self.m.dy_dtau_2 = DerivativeVar(self.m.y_2, wrt=self.m.t2)
        self.m.dz_dtau_2 = DerivativeVar(self.m.z_2, wrt=self.m.t2) 

        # inertial velocity state variables
        self.m.xdot_2 = Var(self.m.t2, bounds=(self.m.xdot_min / self.m.x_scale / self.m.xdot_scale, None))
        self.m.ydot_2 = Var(self.m.t2, bounds=(self.m.ydot_min / self.m.y_scale / self.m.ydot_scale, None))
        self.m.zdot_2 = Var(self.m.t2)

        # attitude wrt to inertial axis state variables 
        self.m.phi_2 = Var(self.m.t2, bounds=(self.m.phi_min / self.m.phi_scale, self.m.phi_max / self.m.phi_scale))
        self.m.the_2 = Var(self.m.t2, bounds=(self.m.the_min / self.m.the_scale, self.m.the_max / self.m.the_scale))
        self.m.psi_2 = Var(self.m.t2, bounds=(self.m.psi_min / self.m.psi_scale, self.m.psi_max / self.m.psi_scale))
        
        self.m.dphi_dtau_2 = DerivativeVar(self.m.phi_2, wrt=self.m.t2, initialize=0.0)
        self.m.dthe_dtau_2 = DerivativeVar(self.m.the_2, wrt=self.m.t2, initialize=0.0)
        self.m.dpsi_dtau_2 = DerivativeVar(self.m.psi_2, wrt=self.m.t2, initialize=0.0)

        # attitude angles
        self.m.phidot_2 = Var(self.m.t2, initialize=0.0)
        self.m.thedot_2 = Var(self.m.t2, initialize=0.0)
        self.m.psidot_2 = Var(self.m.t2, initialize=0.0)
        
        # body velocity state variables
        self.m.u_2 = Var(self.m.t2, bounds=(self.m.u_min / self.m.u_scale, None))
        self.m.v_2 = Var(self.m.t2, bounds=( self.m.v_min /  self.m.v_scale,  self.m.v_max  / self.m.v_scale))
        self.m.w_2 = Var(self.m.t2, bounds=( self.m.w_min / self.m.w_scale,  self.m.w_max  / self.m.w_scale))

        self.m.du_dtau_2 = DerivativeVar(self.m.u_2, wrt=self.m.t2)
        self.m.dv_dtau_2 = DerivativeVar(self.m.v_2, wrt=self.m.t2)
        self.m.dw_dtau_2 = DerivativeVar(self.m.w_2, wrt=self.m.t2)

        # body acceleration state variables
        self.m.udot_2 = Var(self.m.t2, bounds=(self.m.udot_min / self.m.u_scale /  self.m.udot_scale, self.m.udot_max / self.m.u_scale /  self.m.udot_scale,))
        self.m.vdot_2 = Var(self.m.t2, bounds=(self.m.vdot_min / self.m.v_scale /  self.m.vdot_scale, self.m.vdot_max / self.m.v_scale /  self.m.vdot_scale,))
        self.m.wdot_2 = Var(self.m.t2, bounds=(self.m.wdot_min / self.m.w_scale /  self.m.wdot_scale, self.m.wdot_max / self.m.w_scale /  self.m.wdot_scale,))

        # attitude rate wrt to body axis state variables
        self.m.p_2 = Var(self.m.t2)
        self.m.q_2 = Var(self.m.t2)
        self.m.r_2 = Var(self.m.t2)

        self.m.dp_dtau_2 = DerivativeVar(self.m.p_2, wrt=self.m.t2)
        self.m.dq_dtau_2 = DerivativeVar(self.m.q_2, wrt=self.m.t2)
        self.m.dr_dtau_2 = DerivativeVar(self.m.r_2, wrt=self.m.t2)

        # attitude acceleration wrt to body axis state variables
        self.m.pdot_2 = Var(self.m.t2)
        self.m.qdot_2 = Var(self.m.t2)
        self.m.rdot_2 = Var(self.m.t2)

        # quaternion representation 
        self.m.q0_2 = Var(self.m.t2, bounds=(-1,1))
        self.m.q1_2 = Var(self.m.t2, bounds=(-1,1))
        self.m.q2_2 = Var(self.m.t2, bounds=(-1,1))
        self.m.q3_2 = Var(self.m.t2, bounds=(-1,1))
        
        self.m.dq0_dtau_2 = DerivativeVar(self.m.q0_2, wrt=self.m.t2)
        self.m.dq1_dtau_2 = DerivativeVar(self.m.q1_2, wrt=self.m.t2)
        self.m.dq2_dtau_2 = DerivativeVar(self.m.q2_2, wrt=self.m.t2)
        self.m.dq3_dtau_2 = DerivativeVar(self.m.q3_2, wrt=self.m.t2)

        self.m.q0dot_2 = Var(self.m.t2)
        self.m.q1dot_2 = Var(self.m.t2)
        self.m.q2dot_2 = Var(self.m.t2)
        self.m.q3dot_2 = Var(self.m.t2)

        # mass properties
        self.m.mass_2    = Var(self.m.t2, bounds=(self.m.mass_min / self.m.mass_scale, self.m.mass_max / self.m.mass_scale))
        # self.m.mpayload_2 = Var(bounds=(self.m.mpayload_min / self.m.mass_scale, self.m.mpayload_max / self.m.mass_scale))
        self.m.mfuel_2 = Var(self.m.t2, bounds=(self.m.mfuel_min / self.m.mfuel_scale, self.m.mfuel_max / self.m.mfuel_scale))
        self.m.massdot_2 = Var(self.m.t2, bounds=(self.m.massdot_max / self.m.mfuel_scale / self.m.massdot_scale, self.m.massdot_min / self.m.mfuel_scale / self.m.massdot_scale))
        self.m.dmass_dtau_2 = DerivativeVar(self.m.mfuel_2, wrt=self.m.t2)

        # thrust representation control parameters
        self.m.kap_2    = Var(self.m.t2, initialize=0, bounds=(self.m.kap_min / self.m.kap_scale, self.m.kap_max / self.m.kap_scale))
        self.m.eps_2    = Var(self.m.t2, initialize=0, bounds=(self.m.eps_min / self.m.eps_scale, self.m.eps_max / self.m.eps_scale))
        self.m.mpdot_2  = Var(self.m.t2,  bounds=(self.m.mpdot_min / self.m.mpdot_scale, self.m.mpdot_max / self.m.mpdot_scale))

        self.m.delta_2    = Var(self.m.t2, initialize = self.m.delta_max / self.m.delta_scale, bounds=(self.m.delta_min / self.m.delta_scale, self.m.delta_max / self.m.delta_scale))
        self.m.gamma_2    = Var(self.m.t2, initialize = self.m.gamma_max / self.m.gamma_scale, bounds=(self.m.gamma_min / self.m.gamma_scale, self.m.gamma_max / self.m.gamma_scale))


        #=============Phase 3===========================================
    
        # time
        self.m.t3  = ContinuousSet(bounds=(0,1))
        self.m.tf3 = Var(bounds=(1 / self.m.tf_scale , 10 / self.m.tf_scale))

        # inertial displacement state variables
        self.m.x_3 = Var(self.m.t3, bounds=(self.m.x_min / self.m.x_scale, self.m.x_max / self.m.x_scale))
        self.m.y_3 = Var(self.m.t3, bounds=(self.m.y_min / self.m.y_scale, self.m.y_max / self.m.y_scale))
        self.m.z_3 = Var(self.m.t3, bounds=(self.m.z_min / self.m.z_scale, self.m.z_max / self.m.z_scale))

        self.m.dx_dtau_3 = DerivativeVar(self.m.x_3, wrt=self.m.t3)
        self.m.dy_dtau_3 = DerivativeVar(self.m.y_3, wrt=self.m.t3) 
        self.m.dz_dtau_3 = DerivativeVar(self.m.z_3, wrt=self.m.t3) 

        # inertial velocity state variables
        self.m.xdot_3 = Var(self.m.t3, bounds=(self.m.xdot_min / self.m.x_scale / self.m.xdot_scale, None))
        self.m.ydot_3 = Var(self.m.t3, bounds=(self.m.ydot_min / self.m.y_scale / self.m.ydot_scale, None))
        self.m.zdot_3 = Var(self.m.t3)
                          
        # attitude wrt to inertial axis state variables 
        self.m.phi_3 = Var(self.m.t3, bounds=(self.m.phi_min / self.m.phi_scale, self.m.phi_max / self.m.phi_scale))
        self.m.the_3 = Var(self.m.t3, bounds=(self.m.the_min / self.m.the_scale, self.m.the_max / self.m.the_scale))
        self.m.psi_3 = Var(self.m.t3, bounds=(self.m.psi_min / self.m.psi_scale, self.m.psi_max / self.m.psi_scale))
        
        self.m.dphi_dtau_3 = DerivativeVar(self.m.phi_3, wrt=self.m.t3, initialize=0.0)
        self.m.dthe_dtau_3 = DerivativeVar(self.m.the_3, wrt=self.m.t3, initialize=0.0)
        self.m.dpsi_dtau_3 = DerivativeVar(self.m.psi_3, wrt=self.m.t3, initialize=0.0)

        # attitude angles
        self.m.phidot_3 = Var(self.m.t3, initialize=0.0)
        self.m.thedot_3 = Var(self.m.t3, initialize=0.0)
        self.m.psidot_3 = Var(self.m.t3, initialize=0.0)
        
        # body velocity state variables
        self.m.u_3 = Var(self.m.t3, bounds=(self.m.u_min / self.m.u_scale, None))
        self.m.v_3 = Var(self.m.t3, bounds=(self.m.v_min / self.m.v_scale, self.m.v_max / self.m.v_scale))
        self.m.w_3 = Var(self.m.t3, bounds=(self.m.w_min / self.m.w_scale, self.m.w_max / self.m.w_scale))

        self.m.du_dtau_3 = DerivativeVar(self.m.u_3, wrt=self.m.t3)
        self.m.dv_dtau_3 = DerivativeVar(self.m.v_3, wrt=self.m.t3)
        self.m.dw_dtau_3 = DerivativeVar(self.m.w_3, wrt=self.m.t3)

        # body acceleration state variables
        self.m.udot_3 = Var(self.m.t3, bounds=(self.m.udot_min / self.m.u_scale / self.m.udot_scale, self.m.udot_max / self.m.u_scale / self.m.udot_scale))
        self.m.vdot_3 = Var(self.m.t3, bounds=(self.m.vdot_min / self.m.v_scale / self.m.udot_scale, self.m.vdot_max / self.m.v_scale / self.m.vdot_scale))
        self.m.wdot_3 = Var(self.m.t3, bounds=(self.m.wdot_min / self.m.w_scale / self.m.udot_scale, self.m.wdot_max / self.m.w_scale / self.m.wdot_scale))

        # attitude rate wrt to body axis state variables
        self.m.p_3 = Var(self.m.t3)
        self.m.q_3 = Var(self.m.t3)
        self.m.r_3 = Var(self.m.t3)

        self.m.dp_dtau_3 = DerivativeVar(self.m.p_3, wrt=self.m.t3)
        self.m.dq_dtau_3 = DerivativeVar(self.m.q_3, wrt=self.m.t3)
        self.m.dr_dtau_3 = DerivativeVar(self.m.r_3, wrt=self.m.t3)

        # attitude acceleration wrt to body axis state variables
        self.m.pdot_3 = Var(self.m.t3)
        self.m.qdot_3 = Var(self.m.t3)
        self.m.rdot_3 = Var(self.m.t3)

        # quaternion representation state variables
        self.m.q0_3 = Var(self.m.t3, bounds=(-1,1))
        self.m.q1_3 = Var(self.m.t3, bounds=(-1,1))
        self.m.q2_3 = Var(self.m.t3, bounds=(-1,1))
        self.m.q3_3 = Var(self.m.t3, bounds=(-1,1))

        self.m.dq0_dtau_3 = DerivativeVar(self.m.q0_3, wrt=self.m.t3)
        self.m.dq1_dtau_3 = DerivativeVar(self.m.q1_3, wrt=self.m.t3)
        self.m.dq2_dtau_3 = DerivativeVar(self.m.q2_3, wrt=self.m.t3)
        self.m.dq3_dtau_3 = DerivativeVar(self.m.q3_3, wrt=self.m.t3)

        self.m.q0dot_3 = Var(self.m.t3)
        self.m.q1dot_3 = Var(self.m.t3)
        self.m.q2dot_3 = Var(self.m.t3)
        self.m.q3dot_3 = Var(self.m.t3)

        # mass properties state variables
        self.m.mass_3    = Var(self.m.t3, bounds=(self.m.mass_min / self.m.mass_scale, self.m.mass_max / self.m.mass_scale))
        # self.m.mpayload_3 = Var(bounds=(self.m.mpayload_min / self.m.mass_scale, self.m.mpayload_max / self.m.mass_scale))
        self.m.mfuel_3 = Var(self.m.t3, bounds=(self.m.mfuel_min / self.m.mfuel_scale, self.m.mfuel_max / self.m.mfuel_scale))
        self.m.massdot_3 = Var(self.m.t3, bounds=(self.m.massdot_max / self.m.mfuel_scale / self.m.massdot_scale, -0.05 / self.m.mfuel_scale / self.m.massdot_scale))
        self.m.dmass_dtau_3 = DerivativeVar(self.m.mfuel_3, wrt=self.m.t3)

        # thrust representation control parameters
        self.m.kap_3    = Var(self.m.t3, bounds=(self.m.kap_min / self.m.kap_scale, self.m.kap_max / self.m.kap_scale))
        self.m.eps_3    = Var(self.m.t3, bounds=(self.m.eps_min / self.m.eps_scale, self.m.eps_max / self.m.eps_scale))
        self.m.mpdot_3  = Var(self.m.t3, bounds=(self.m.mpdot_min / self.m.mpdot_scale, self.m.mpdot_max / self.m.mpdot_scale))

        self.m.delta_3    = Var(self.m.t3, initialize = self.m.delta_max / self.m.delta_scale, bounds=(self.m.delta_min / self.m.delta_scale, self.m.delta_max / self.m.delta_scale))
        self.m.gamma_3    = Var(self.m.t3, initialize = self.m.gamma_max / self.m.gamma_scale, bounds=(self.m.gamma_min / self.m.gamma_scale, self.m.gamma_max / self.m.gamma_scale))

        # discretize problem euler backward finite difference
        # discretizer = TransformationFactory('dae.finite_difference')
        # discretizer.apply_to(self.m, nfe=100, wrt=self.m.t1, scheme='BACKWARD')
        # discretizer = TransformationFactory('dae.finite_difference')
        # discretizer.apply_to(self.m, nfe=100, wrt=self.m.t2, scheme='BACKWARD')
        # discretizer = TransformationFactory('dae.finite_difference')
        # discretizer.apply_to(self.m, nfe=100, wrt=self.m.t3, scheme='BACKWARD')

        # discretize problem orthogonal collocation
        discretizer = TransformationFactory('dae.collocation')
        discretizer.apply_to(self.m, wrt=self.m.t1, nfe=70, ncp=3)
        discretizer.reduce_collocation_points(self.m, var=self.m.mpdot_1, ncp=1, contset=self.m.t1)
        discretizer.reduce_collocation_points(self.m, var=self.m.eps_1, ncp=1, contset=self.m.t1)
        discretizer.reduce_collocation_points(self.m, var=self.m.kap_1, ncp=1, contset=self.m.t1)

        discretizer.reduce_collocation_points(self.m, var=self.m.delta_1, ncp=1, contset=self.m.t1)
        discretizer.reduce_collocation_points(self.m, var=self.m.gamma_1, ncp=1, contset=self.m.t1)
        discretizer = TransformationFactory('dae.collocation')
        discretizer.apply_to(self.m, wrt=self.m.t2, nfe=70, ncp=3)

        discretizer.reduce_collocation_points(self.m, var=self.m.delta_2, ncp=1, contset=self.m.t2)
        discretizer.reduce_collocation_points(self.m, var=self.m.gamma_2, ncp=1, contset=self.m.t2)
   
        discretizer = TransformationFactory('dae.collocation')
        discretizer.apply_to(self.m, wrt=self.m.t3, nfe=70, ncp=3)
        discretizer.reduce_collocation_points(self.m, var=self.m.eps_3, ncp=1, contset=self.m.t3)
        discretizer.reduce_collocation_points(self.m, var=self.m.kap_3, ncp=1, contset=self.m.t3)

        discretizer.reduce_collocation_points(self.m, var=self.m.delta_3, ncp=1, contset=self.m.t3)
        discretizer.reduce_collocation_points(self.m, var=self.m.gamma_3, ncp=1, contset=self.m.t3)

        return
    
    # mass and rates
    def Q_masstotal(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot)= getPhaseVariables(m, n, t)
                
        return mass == mfuel + mpayload + 48
    
    def Q_massdot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot)= getPhaseVariables(m, n, t)
                
        return (massdot) == -(mpdot)
    
    def Q_mass_dot_2(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot)  = getPhaseVariables(m, n, t)
        
        return mpdot <= 0.001
    
    # quaternions  
    def Q_normality(self, m, n,t):

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)

        return 1 - (((q0**2) + (q1**2) + (q2**2) + (q3**2)) )**2 <= 1e-6
    
    def Q_q0(self, m, n,t):

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)

        return  q0 == ((cos((psi) / 2) * cos((the) / 2) * cos((phi) / 2)) + (sin((psi) / 2) * sin((the) / 2) * sin((phi) / 2)))

    def Q_q1(self, m, n, t):

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        return  q1 == (cos((psi) / 2) * cos((the) / 2) * sin((phi) / 2)) - (sin((psi) / 2) * sin((the) / 2) * cos((phi) / 2))

    def Q_q2(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        return  q2 == (cos((psi) / 2) * sin((the) / 2) * cos((phi) / 2)) + (sin((psi) / 2) * cos((the) / 2) * sin((phi) / 2)) 
    
    def Q_q3(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        return  q3 == (sin((psi) / 2) * cos((the) / 2) * cos((phi) / 2)) - (cos((psi) / 2) * sin((the) / 2) * sin((phi) / 2)) 

     # body velocity   
    
    def Q_u(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        t11 = cos(the) * cos(psi)
        t12 = cos(psi) * sin(the) * sin(phi) - sin(psi) * cos(phi)
        t13 = cos(psi) * sin(the) * cos(phi) + sin(psi) * sin(phi)
        # return ((xdot)) * eom.inverse_quaternion.Q_prime(q0, q1, q2, q3) == (((u) * eom.inverse_quaternion.Q11_prime(q0, q1, q2, q3)) + ((v) * eom.inverse_quaternion.Q12_prime(q0, q1, q2, q3)) + ((w) * eom.inverse_quaternion.Q13_prime(q0, q1, q2, q3))) 
        return u == (eom.quaternion.Q11(q0, q1, q2, q3) * xdot) + (eom.quaternion.Q12(q0, q1, q2, q3) * ydot) + (eom.quaternion.Q13(q0, q1, q2, q3) * -zdot)
        
    def Q_v(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        t21 = sin(psi) * cos(the)
        t22 = sin(psi) * sin(the) * sin(phi) + cos(psi) * cos(phi)
        t23 = sin(psi) * sin(the) * cos(phi) - cos(psi) * sin(phi)
        # return  ((ydot)) * eom.inverse_quaternion.Q_prime(q0, q1, q2, q3) == (((u) * eom.inverse_quaternion.Q21_prime(q0, q1, q2, q3)) + ((v) * eom.inverse_quaternion.Q22_prime(q0, q1, q2, q3)) + ((w) * eom.inverse_quaternion.Q23_prime(q0, q1, q2, q3)))
        return v == (eom.quaternion.Q21(q0, q1, q2, q3) * xdot) + (eom.quaternion.Q22(q0, q1, q2, q3) * ydot) + (eom.quaternion.Q23(q0, q1, q2, q3) * -zdot)
        
    def Q_w(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        t31 = -sin(the)
        t32 = cos(the) * sin(phi)
        t33 = cos(the) * cos(phi)
        # return (-(zdot)) * eom.inverse_quaternion.Q_prime(q0, q1, q2, q3) == (((u) * eom.inverse_quaternion.Q31_prime(q0, q1, q2, q3)) + ((v) * eom.inverse_quaternion.Q32_prime(q0, q1, q2, q3)) + (w * eom.inverse_quaternion.Q33_prime(q0, q1, q2, q3)))
        return w == (eom.quaternion.Q31(q0, q1, q2, q3) * xdot) + (eom.quaternion.Q32(q0, q1, q2, q3) * ydot) + (eom.quaternion.Q33(q0, q1, q2, q3) * -zdot)
         
    # body acceleration
    def Q_udot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = eps * 180 / 3.1415
        beta = kap * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CX = (aero.forces.CX_alpha(Mach, alpha, beta)) + (aero.forces.CX_beta(Mach, alpha, beta))
        AX = 0.5 * atm.rho(z) * (u ** 2)  * param.S * CX
        FX = (((mpdot) * prop.Isp * 9.81) * cos(kap) * cos(eps)) - AX
        return (udot) == (((FX / mass)) - ((w)  * (q)) + ((v) * (r)) + ((eom.quaternion.Q13(q0, q1, q2, q3)* atm.gravity(z))))

    def Q_vdot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = eps * 180 / 3.1415
        beta = kap * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CY = aero.forces.CN_beta(Mach, alpha, beta)
        AY = 0.5 * atm.rho(z) * ((v) ** 2) * param.S * CY
        FY = -(((mpdot) * prop.Isp * 9.81) * cos(kap) * sin(eps)) - AY
        return (vdot) == ((( (FY) / mass)) - ((u) * (r)) + ((w) * (p)) + ((eom.quaternion.Q23(q0, q1, q2, q3) * atm.gravity(z))))
    
    def Q_wdot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = eps * 180 / 3.1415
        beta = kap * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CZ = aero.forces.CN_alpha(Mach, alpha, beta)
        AZ = 0.5 * atm.rho(z) * ((w) ** 2) * param.S * CZ
        FZ = -(((mpdot) * prop.Isp * 9.81) * sin(kap)) - AZ
        return (wdot) == ((( (FZ) / mass)) - ((v) * (p)) + ((u) * (q)) + ((eom.quaternion.Q33(q0, q1, q2, q3) * atm.gravity(z))))

    def Q_udot_2(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = delta * 180 / 3.1415
        beta = gamma * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CX = (aero.forces.CX_alpha(Mach, alpha, beta)) + (aero.forces.CX_beta(Mach, alpha, beta))
        AX = 0.5 * atm.rho(z) * (u ** 2)  * param.S * CX
        FX = -AX
        return (udot) == ( (( (FX) / mass)) - ((w)  * (q)) + ((v) * (r)) + ((eom.quaternion.Q13(q0, q1, q2, q3)* atm.gravity(z))))
    
    def Q_vdot_2(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = delta * 180 / 3.1415
        beta = gamma * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        R_MY = (-((0.034) * prop.R_Isp * 9.81) * cos(delta) * sin(gamma)) * param.d 
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CY = aero.forces.CN_beta(Mach, alpha, beta)
        AY = 0.5 * atm.rho(z) * ((v) ** 2) * param.S * CY
        FY = R_MY - AY
        return (vdot) == ((( (FY) / mass)) - ((u) * (r)) + ((w) * (p)) + ((eom.quaternion.Q23(q0, q1, q2, q3) * atm.gravity(z))))
    
    def Q_wdot_2(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = delta * 180 / 3.1415
        beta = gamma * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        R_MZ = (-((0.034) * prop.R_Isp * 9.81) * sin(delta)) * param.d
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CZ = aero.forces.CN_alpha(Mach, alpha, beta)
        AZ = 0.5 * atm.rho(z) * ((w) ** 2) * param.S * CZ
        FZ = R_MZ - AZ
        return (wdot) == ((( (FZ) / mass)) - ((v) * (p)) + ((u) * (q)) + ((eom.quaternion.Q33(q0, q1, q2, q3) * atm.gravity(z))))
    
    #quaternion rate 
    def Q_q0dot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        k =0.005
        error = 1 - ((q0**2) + (q1**2) + (q2**2) + (q3**2))
        return (q0dot) == ((-0.5 * ( ((p) * (q1)) + ((q) * (q2)) + ((r) * (q3)))) + k*error*q0)

    def Q_q1dot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        k =  0.005
        error = 1 - ((q0**2) + (q1**2) + (q2**2) + (q3**2))
        return (q1dot) == ((0.5 * (((p) * (q0)) + ((r) * (q2)) - ((q) * (q3)))) + k*error*q1)
    
    def Q_q2dot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        k = 0.005
        error = 1 - ((q0**2) + (q1**2) + (q2**2) + (q3**2))
        return (q2dot) == ((0.5 * (((q) * (q0)) - ((r) * (q1)) + ((p) * (q3)))) + k*error*q2)
    
    def Q_q3dot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        k = 0.005
        error = 1 - ((q0**2) + (q1**2) + (q2**2) + (q3**2))
        return (q3dot) == ((0.5 * (((r) * (q0)) + ((q) * (q1)) - ((p) * (q2)))) + k*error*q3)

    # body angular acceleration    
    def Q_pdot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        CL = 10e-7
        AL = 0.5 * atm.rho(z) * ((u) ** 2) * param.S * param.l * CL
        return (pdot) == (((q) * (r) ) * ((param.Iy - param.Iz) / param.Ix)) + AL

    def Q_qdot(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = eps * 180 / 3.1415
        beta = kap * 180 / 3.1415
        R_MZ = (-((0.034) * prop.R_Isp * 9.81) * sin(delta)) * param.d
        MZ = (-((mpdot) * prop.Isp * 9.81) * sin(kap)) * param.d
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CM = aero.moments.CM_alpha(Mach, alpha, beta)
        AM = 0.5 * atm.rho(z) * ((v) ** 2) * param.S * param.l * CM
        return (qdot) == ((((p) * (r)) * ((param.Iz - param.Ix) / param.Iy)) - ((AM + MZ) / param.Iy))
    
    def Q_rdot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = eps * 180 / 3.1415
        beta = kap * 180 / 3.1415
        MY = (-((mpdot) * prop.Isp * 9.81) * cos(kap) * sin(eps)) * param.d 
        R_MY = (-((0.034) * prop.R_Isp * 9.81) * cos(delta) * sin(gamma)) * param.d 
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        CN = aero.moments.CM_beta(Mach, alpha, beta)
        AN = 0.5 * atm.rho(z) * ((w) ** 2) * param.S * param.l * CN
        return (rdot) == ((((p) * (q)) * ((param.Ix - param.Iy) / param.Iz)) + (( AN + MY) / param.Iz))

    def Q_qdot_2(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = delta * 180 / 3.1415
        beta = gamma * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        R_MZ = (-((0.034) * prop.R_Isp * 9.81) * sin(delta)) * param.d
        CM = aero.moments.CM_alpha(Mach, alpha, beta)
        AM = 0.5 * atm.rho(z) * ((v) ** 2) * param.S * param.l * CM
        return (qdot) == ((((p) * (r)) * ((param.Iz - param.Ix) / param.Iy)) - ((R_MZ + AM) / param.Iy))
    
    def Q_rdot_2(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        alpha = delta * 180 / 3.1415
        beta = gamma * 180 / 3.1415
        a = (atm.gamma * atm.R_const * atm.temperature(z))**0.5
        # Mach = (((u**2 + v**2 + w**2) + 0.1)**0.5) / a
        Mach = 2.8
        R_MY = (-((0.034) * prop.R_Isp * 9.81) * cos(delta) * sin(gamma)) * param.d 
        CN = aero.moments.CM_beta(Mach, alpha, beta)
        AN = 0.5 * atm.rho(z) * ((w) ** 2) * param.S * param.l * CN
        return (rdot) == ((((p) * (q)) * ((param.Ix - param.Iy) / param.Iz)) + ((R_MY + AN) / param.Iz))
    
    # body angular rates 
    def Q_phidot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return p == phidot - (sin(the) * psidot)
    
    def Q_thedot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return q == (cos(phi) * thedot) + (sin(phi) * cos(the) * psidot)
    
    def Q_psidot(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return r == (-sin(phi) * thedot) + (cos(phi) * cos(the) * psidot)
    
    # attitude angles
    def Q_phi(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return tan(phi) * (q0**2 - q1**2 - q2**2 - q3**2) ==  (2 * (q2 * q3 + q0 * q1)) 
    
    def Q_the(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return sin(the) ==  (-2 * (q1 * q3 - q0 * q2))
    
    def Q_psi(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return tan(psi) * (q0**2 + q1**2 - q2**2 - q3**2) == ( (2 * (q1 * q2 + q0 * q3))  )
    
    # derivatives
    def Q_dx_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dx_dtau) == (xdot) * tf
    
    def Q_dy_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dy_dtau) == (ydot) * tf
    
    def Q_dz_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        return (dz_dtau) == (zdot) * tf
    
    def Q_du_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (du_dtau) == (udot) * tf
    
    def Q_dv_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dv_dtau) == (vdot) * tf
    
    def Q_dw_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dw_dtau) == (wdot) * tf
    
    def Q_dp_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dp_dtau) == (pdot) * tf
    
    def Q_dq_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dq_dtau) == (qdot) * tf
    
    def Q_dr_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dr_dtau) == (rdot) * tf
    
    def Q_dq0_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dq0_dtau) == (q0dot) * tf
    
    def Q_dq1_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dq1_dtau) == (q1dot) * tf
    
    def Q_dq2_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dq2_dtau) == (q2dot) * tf
    
    def Q_dq3_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dq3_dtau) == (q3dot) * tf
    
    def Q_dphi_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dphi_dtau) == (phidot) * tf
    
    def Q_dthe_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dthe_dtau) == (thedot) * tf
    
    def Q_dpsi_dtau(self, m, n, t): 
        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot) = getPhaseVariables(m, n, t)
        
        return (dpsi_dtau) == (psidot) * tf
    
    def Q_dmass_dtau(self, m, n, t): 

        (tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, mfuel, mpayload, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau, delta, gamma, \
            dq0_dtau, dq1_dtau, dq2_dtau, dq3_dtau, q0dot, q1dot, q2dot, q3dot)= getPhaseVariables(m, n, t)
                
        return (dmass_dtau) == (massdot) * tf
   
    # Boundary conditions   
    def BCs(self, m):
    
#================Initialize Phase==============================================
        yield m.x_1[0]          == 1 / m.x_scale
        yield m.y_1[0]          == 1 / m.y_scale   
        yield m.z_1[0]          == 1 / m.z_scale   

        yield m.q0_1[0]          == 0.7384685587295879/ m.q0_scale
        # yield m.q1_1[0]          == 0 / m.q1_scale
        yield m.q2_1[0]          == 0.674287911628145/ m.q2_scale
        yield m.q3_1[0]          == 0 / m.q3_scale

        yield m.u_1[0]          == 1 / m.u_scale
        # yield m.v_1[0]          == 1 / m.v_scale
        # yield m.w_1[0]          == 1 / m.w_scale

        # yield m.the_1[0]        == (1.48) / m.the_scale 
        # yield m.psi_1[0]       == (0.001745277) / m.psi_scale  
        # yield m.phi_1[0]       == (0.0001) / m.phi_scale  

        # yield m.mass_1[0]       == 291 / m.mass_scale
        
        # yield m.mpdot_1[0]       == 0.001 / m.mpdot_scale 


        # Boundary conditions 1-> 2

        yield m.x_1[1]         == m.x_2[0] 
        yield m.y_1[1]         == m.y_2[0]  
        yield m.z_1[1]         == m.z_2[0]

        yield m.u_1[1]         == m.u_2[0]
        yield m.v_1[1]         == m.v_2[0]
        yield m.w_1[1]         == m.w_2[0]

        yield m.p_1[1]         == m.p_2[0]
        yield m.q_1[1]         == m.q_2[0]
        yield m.r_1[1]         == m.r_2[0]

        # yield m.phi_1[1]       == m.phi_2[0]
        # yield m.the_1[1]       == m.the_2[0]
        # yield m.psi_1[1]       == m.psi_2[0]

        yield m.q0_1[1]         == m.q0_2[0]
        yield m.q1_1[1]         == m.q1_2[0]
        yield m.q2_1[1]         == m.q2_2[0]
        yield m.q3_1[1]         == m.q3_2[0]

        yield m.mass_1[1]      == m.mass_2[0]
        yield m.mfuel_1[1]      == m.mfuel_2[0]


        # yield m.mpdot_1[1]      == m.mpdot_2[0]

        yield m.kap_1[1]        == m.kap_2[0]
        yield m.eps_1[1]        == m.eps_2[0]

        # Boundary conditions 2-> 3
        
        yield m.x_2[1]         == m.x_3[0] 
        yield m.y_2[1]         == m.y_3[0]  
        yield m.z_2[1]         == m.z_3[0]
            
        yield m.u_2[1]         == m.u_3[0]
        yield m.v_2[1]         == m.v_3[0]
        yield m.w_2[1]         == m.w_3[0]

        yield m.p_2[1]         == m.p_3[0]
        yield m.q_2[1]         == m.q_3[0]
        yield m.r_2[1]         == m.r_3[0]

        # yield m.phi_2[1]       == m.phi_3[0]
        # yield m.the_2[1]       == m.the_3[0]
        # yield m.psi_2[1]       == m.psi_3[0]
        
        yield m.q0_2[1]       == m.q0_3[0]
        yield m.q1_2[1]       == m.q1_3[0]
        yield m.q2_2[1]       == m.q2_3[0]
        yield m.q3_2[1]       == m.q3_3[0]

        yield m.mass_2[1]      == m.mass_3[0]
        yield m.mfuel_2[1]      == m.mfuel_3[0]

        # yield m.mpdot_2[1]      == m.mpdot_3[0]

        yield m.kap_2[1]        == m.kap_3[0]
        yield m.eps_2[1]        == m.eps_3[0]

#================Finalize Phase==============================================
        yield m.z_2[1]         == 200e3 /  self.m.z_scale
        # yield m.u_3[1]         == 3346 /  self.m.u_scale
        yield m.zdot_2[1]         == 1 /  self.m.z_scale / self.m.zdot_scale
        # yield m.zdot_2[1]         >= 1e- /  self.m.z_scale / self.m.zdot_scale

        # yield m.mfuel_3[1]         ==  0 /  self.m.mfuel_scale 
