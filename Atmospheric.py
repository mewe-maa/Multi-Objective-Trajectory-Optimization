import numpy as np
import matplotlib.pyplot as plt


g_mars = 3.71
r_mars = 3389.5e3

R_const = 188
gamma = 1.3

def gravity(z):
    h = z
    g = g_mars / ((1 + ( h / r_mars))**2)
    return g

def temperature(z):
    # Temp = 249.7 - 0.000998 * z
    Temp = 250
    return Temp


def pressure(z):
    pr = 699 * np.exp(-0.00009 * z)
    return pr


def rho(z):
    # Temp = 249.7 - 0.000998 * z
    Temp = 250
    pr = 699 *  2.71828**(-0.00009 * z)
    rho = pr / (192.1 * Temp)
    return rho
