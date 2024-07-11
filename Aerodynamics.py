import numpy as np
import matplotlib.pyplot as plt
import Parameters as param
import Atmospheric as atm


class forces:   
    def CX_alpha(Mach, alpha, beta):    
        p00 = -0.04657 
        p10 = -0.0001731
        p01 = 0.1301
        p11 = 0.0002995 
        p02 = -0.01938 
        p12 = -8.086e-05 
        p03 = -0.002438 
        CX_alpha = p00 + (p10 * alpha) + (p01 * Mach) + (p11 * alpha * Mach) + (p02 * (Mach**2)) + (p12 * alpha * (Mach**2)) + (p03 * (Mach**3))
        return CX_alpha   
     
    def CX_beta(Mach, alpha, beta):      
        p00 = -0.04657 
        p10 = -0.0001731
        p01 = 0.1301
        p11 = 0.0002995 
        p02 = -0.01938 
        p12 = -8.086e-05 
        p03 = -0.002438 
        CX_beta = p00 + (p10 * beta) + (p01 * Mach) + (p11 * beta * Mach) + (p02 * (Mach**2)) + (p12 * beta * (Mach**2)) + (p03 * (Mach**3))
        return CX_beta   
        
    def CN_alpha(Mach, alpha, beta):      
        p00 = 0.001807
        p10 = 0.0004407
        p01 = -0.008166
        p11 = -0.00123
        p02 = 0.008475
        p12 = 0.0009847
        p03 = -0.00199
        CN_alpha = p00 + (p10 * alpha) + (p01 * Mach) + (p11 * alpha * Mach) + (p02 * (Mach**2)) + (p12 * alpha * (Mach**2)) + (p03 * (Mach**3))
        return CN_alpha

    def CN_beta(Mach, alpha, beta):       
        p00 = 0.001807
        p10 = 0.0004407
        p01 = -0.008166
        p11 = -0.00123
        p02 = 0.008475
        p12 = 0.0009847
        p03 = -0.00199
        CN_beta = p00 + (p10 * beta) + (p01 * Mach) + (p11 * beta * Mach) + (p02 * (Mach**2)) + (p12 * beta * (Mach**2)) + (p03 * (Mach**3))    
        return CN_beta

class moments:    
    def CM_alpha(Mach, alpha, beta):       
        p00 = 0.004021 
        p10 = -0.001558 
        p01 = -0.01086 
        p11 = 0.00374 
        p02 = 0.008531 
        p12 = -0.0006538 
        p03 = -0.001788 
        CM_alpha = p00 + (p10 * alpha) + (p01 * Mach) + (p11 * alpha * Mach) + (p02 * (Mach**2)) + (p12 * alpha * (Mach**2)) + (p03 * (Mach**3))
        return CM_alpha

    def CM_beta(Mach, alpha, beta):        
        p00 = 0.004021 
        p10 = -0.001558 
        p01 = -0.01086 
        p11 = 0.00374 
        p02 = 0.008531 
        p12 = -0.0006538 
        p03 = -0.001788 
        CM_beta = p00 + (p10 * beta) + (p01 * Mach) + (p11 * beta * Mach) + (p02 * (Mach**2)) + (p12 * beta * (Mach**2)) + (p03 * (Mach**3))
        return CM_beta
