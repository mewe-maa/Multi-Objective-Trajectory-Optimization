# -*- coding: utf-8 -*-
from pyomo.dae import ContinuousSet, DerivativeVar, Integral
from pyomo.environ import ConcreteModel, TransformationFactory, Var, \
                          NonNegativeReals, Constraint, ConstraintList, \
                          SolverFactory, Objective, cos, sin, minimize,  \
                          NonNegativeReals, NegativeReals, Param

def getPhaseVariables(m, n, t):
    
    if n == 1:
        # time
        tf = m.tf1 * m.tf_scale 

        # state variables
        x = m.x_1[t] * m.x_scale
        y = m.y_1[t] * m.y_scale
        z = m.z_1[t] * m.z_scale

        dx_dtau = m.dx_dtau_1[t] * m.x_scale
        dy_dtau = m.dy_dtau_1[t] * m.y_scale
        dz_dtau = m.dz_dtau_1[t] * m.z_scale
        
        xdot = m.xdot_1[t] * m.x_scale * m.xdot_scale
        ydot = m.ydot_1[t] * m.y_scale * m.ydot_scale
        zdot = m.zdot_1[t] * m.z_scale * m.zdot_scale

        u = m.u_1[t] * m.u_scale
        v = m.v_1[t] * m.v_scale
        w = m.w_1[t] * m.w_scale
        
        du_dtau = m.du_dtau_1[t] * m.u_scale
        dv_dtau = m.dv_dtau_1[t] * m.v_scale
        dw_dtau = m.dw_dtau_1[t] * m.w_scale

        udot = m.udot_1[t] * m.u_scale * m.udot_scale
        vdot = m.vdot_1[t] * m.v_scale * m.vdot_scale
        wdot = m.wdot_1[t] * m.w_scale * m.wdot_scale

        p = m.p_1[t] * m.p_scale
        q = m.q_1[t] * m.q_scale
        r = m.r_1[t] * m.r_scale

        dp_dtau = m.dp_dtau_1[t] * m.p_scale
        dq_dtau = m.dq_dtau_1[t] * m.q_scale
        dr_dtau = m.dr_dtau_1[t] * m.r_scale

        pdot = m.pdot_1[t] * m.p_scale * m.pdot_scale
        qdot = m.qdot_1[t] * m.q_scale * m.qdot_scale
        rdot = m.rdot_1[t] * m.r_scale * m.rdot_scale

        phi = m.phi_1[t] * m.phi_scale
        the = m.the_1[t] * m.the_scale
        psi = m.psi_1[t] * m.psi_scale

        dphi_dtau = m.dphi_dtau_1[t] * m.phi_scale
        dthe_dtau = m.dthe_dtau_1[t] * m.the_scale
        dpsi_dtau = m.dpsi_dtau_1[t] * m.psi_scale

        phidot = m.phidot_1[t] * m.phi_scale * m.phidot_scale
        thedot = m.thedot_1[t] * m.the_scale * m.thedot_scale
        psidot = m.psidot_1[t] * m.psi_scale * m.psidot_scale

        q0 = m.q0_1[t] * m.q0_scale
        q1 = m.q1_1[t] * m.q1_scale
        q2 = m.q2_1[t] * m.q2_scale
        q3 = m.q3_1[t] * m.q3_scale

        # mass properties
        mass = m.mass_1[t] * m.mass_scale
        massdot = m.massdot_1[t] * m.mass_scale * m.massdot_scale
        dmass_dtau = m.dmass_dtau_1[t] * m.mass_scale

        # control parameters
        kap = m.kap_1[t] * m.kap_scale 
        eps = m.eps_1[t] * m.eps_scale 
        mpdot = m.mpdot_1[t] * m.mpdot_scale

        alpha = m.alpha_1[t] * m.alpha_scale 
        beta  = m.beta_1[t] * m.beta_scale 

    elif n == 2:
        # time
        tf = m.tf2 * m.tf_scale 

        # state variables
        x = m.x_2[t] * m.x_scale
        y = m.y_2[t] * m.y_scale
        z = m.z_2[t] * m.z_scale

        dx_dtau = m.dx_dtau_2[t] * m.x_scale
        dy_dtau = m.dy_dtau_2[t] * m.y_scale
        dz_dtau = m.dz_dtau_2[t] * m.z_scale

        xdot = m.xdot_2[t] * m.x_scale * m.xdot_scale
        ydot = m.ydot_2[t] * m.y_scale * m.ydot_scale
        zdot = m.zdot_2[t] * m.z_scale * m.zdot_scale

        u = m.u_2[t] * m.u_scale
        v = m.v_2[t] * m.v_scale
        w = m.w_2[t] * m.w_scale
        
        du_dtau = m.du_dtau_2[t] * m.u_scale
        dv_dtau = m.dv_dtau_2[t] * m.v_scale
        dw_dtau = m.dw_dtau_2[t] * m.w_scale

        udot = m.udot_2[t] * m.u_scale * m.udot_scale
        vdot = m.vdot_2[t] * m.v_scale * m.vdot_scale
        wdot = m.wdot_2[t] * m.w_scale * m.wdot_scale

        p = m.p_2[t] * m.p_scale
        q = m.q_2[t] * m.q_scale
        r = m.r_2[t] * m.r_scale

        dp_dtau = m.dp_dtau_2[t] * m.p_scale
        dq_dtau = m.dq_dtau_2[t] * m.q_scale
        dr_dtau = m.dr_dtau_2[t] * m.r_scale

        pdot = m.pdot_2[t] * m.p_scale * m.pdot_scale
        qdot = m.qdot_2[t] * m.q_scale * m.qdot_scale
        rdot = m.rdot_2[t] * m.r_scale * m.rdot_scale

        phi = m.phi_2[t] * m.phi_scale
        the = m.the_2[t] * m.the_scale
        psi = m.psi_2[t] * m.psi_scale

        dphi_dtau = m.dphi_dtau_2[t] * m.phi_scale
        dthe_dtau = m.dthe_dtau_2[t] * m.the_scale
        dpsi_dtau = m.dpsi_dtau_2[t] * m.psi_scale

        phidot = m.phidot_2[t] * m.phi_scale * m.phidot_scale
        thedot = m.thedot_2[t] * m.the_scale * m.thedot_scale
        psidot = m.psidot_2[t] * m.psi_scale * m.psidot_scale

        q0 = m.q0_2[t] * m.q0_scale
        q1 = m.q1_2[t] * m.q1_scale
        q2 = m.q2_2[t] * m.q2_scale
        q3 = m.q3_2[t] * m.q3_scale

        mass = m.mass_2[t] * m.mass_scale
        massdot = m.massdot_2[t] * m.mass_scale * m.massdot_scale
        dmass_dtau = m.dmass_dtau_2[t] * m.mass_scale

        # control parameters
        kap = m.kap_2[t] * m.kap_scale 
        eps = m.eps_2[t] * m.eps_scale 
        mpdot = m.mpdot_2[t] * m.mpdot_scale

        alpha = m.alpha_2[t] * m.alpha_scale 
        beta  = m.beta_2[t] * m.beta_scale 

    elif n == 3:
        # time
        tf = m.tf3 * m.tf_scale 

        # state variables
        x = m.x_3[t] * m.x_scale
        y = m.y_3[t] * m.y_scale
        z = m.z_3[t] * m.z_scale

        dx_dtau = m.dx_dtau_3[t] * m.x_scale
        dy_dtau = m.dy_dtau_3[t] * m.y_scale
        dz_dtau = m.dz_dtau_3[t] * m.z_scale

        xdot = m.xdot_3[t] * m.x_scale * m.xdot_scale
        ydot = m.ydot_3[t] * m.y_scale * m.ydot_scale
        zdot = m.zdot_3[t] * m.z_scale * m.zdot_scale

        u = m.u_3[t] * m.u_scale
        v = m.v_3[t] * m.v_scale
        w = m.w_3[t] * m.w_scale

        du_dtau = m.du_dtau_3[t] * m.u_scale
        dv_dtau = m.dv_dtau_3[t] * m.v_scale
        dw_dtau = m.dw_dtau_3[t] * m.w_scale

        udot = m.udot_3[t] * m.u_scale * m.udot_scale
        vdot = m.vdot_3[t] * m.v_scale * m.vdot_scale
        wdot = m.wdot_3[t] * m.w_scale * m.wdot_scale

        p = m.p_3[t] * m.p_scale
        q = m.q_3[t] * m.q_scale
        r = m.r_3[t] * m.r_scale

        dp_dtau = m.dp_dtau_3[t] * m.p_scale
        dq_dtau = m.dq_dtau_3[t] * m.q_scale
        dr_dtau = m.dr_dtau_3[t] * m.r_scale

        pdot = m.pdot_3[t] * m.p_scale * m.pdot_scale
        qdot = m.qdot_3[t] * m.q_scale * m.qdot_scale
        rdot = m.rdot_3[t] * m.r_scale * m.rdot_scale

        q0 = m.q0_3[t] * m.q0_scale
        q1 = m.q1_3[t] * m.q1_scale
        q2 = m.q2_3[t] * m.q2_scale
        q3 = m.q3_3[t] * m.q3_scale

        phi = m.phi_3[t] * m.phi_scale
        the = m.the_3[t] * m.the_scale
        psi = m.psi_3[t] * m.psi_scale

        dphi_dtau = m.dphi_dtau_3[t] * m.phi_scale
        dthe_dtau = m.dthe_dtau_3[t] * m.the_scale
        dpsi_dtau = m.dpsi_dtau_3[t] * m.psi_scale

        phidot = m.phidot_3[t] * m.phi_scale * m.phidot_scale
        thedot = m.thedot_3[t] * m.the_scale * m.thedot_scale
        psidot = m.psidot_3[t] * m.psi_scale * m.psidot_scale

        mass = m.mass_3[t] * m.mass_scale
        massdot = m.massdot_3[t] * m.mass_scale * m.massdot_scale
        dmass_dtau = m.dmass_dtau_3[t] * m.mass_scale

        # control parameters
        kap = m.kap_3[t] * m.kap_scale 
        eps = m.eps_3[t] * m.eps_scale 
        mpdot = m.mpdot_3[t] * m.mpdot_scale

        alpha = m.alpha_3[t] * m.alpha_scale 
        beta  = m.beta_3[t] * m.beta_scale       

    return(tf, x, y, z, xdot, ydot, zdot, \
            u, v, w, udot, vdot, wdot, \
            p, q, r, pdot, qdot, rdot, \
            q0, q1, q2, q3, \
            phi, the, psi, \
            mass, massdot, mpdot, kap, eps, \
            du_dtau, dv_dtau, dw_dtau, dx_dtau, dy_dtau, dz_dtau, \
            dp_dtau, dq_dtau, dr_dtau,\
            alpha, beta, \
            dphi_dtau, dthe_dtau, dpsi_dtau, phidot, psidot, thedot, \
            dmass_dtau)