
from pyomo.environ import ConcreteModel, TransformationFactory, Var, \
                          NonNegativeReals, Constraint, \
                          SolverFactory, Objective, cos, sin, minimize, \
                          NonNegativeReals

class quaternion:
    def Q11(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        return Q11 
    def Q12(q0, q1, q2, q3):
        Q12 = 2 * (q1 * q2 + q0 * q3)
        return Q12
    def Q13(q0, q1, q2, q3):
        Q13 = 2 * (q1 * q3 - q0 * q2)
        return Q13
    
    def Q21(q0, q1, q2, q3):
        Q21 = 2 * (q1 * q2 - q0 * q3)
        return Q21
    def Q22(q0, q1, q2, q3):
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        return Q22
    def Q23(q0, q1, q2, q3):
        Q23 = 2 * (q2 * q3 + q0 * q1)
        return Q23

    def Q31(q0, q1, q2, q3):
        Q31 = 2 * (q1 * q3 + q0 * q2)
        return Q31
    def Q32(q0, q1, q2, q3):
        Q32 = 2 * (q2 * q3 - q0 * q1)
        return Q32
    def Q33(q0, q1, q2, q3):
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)
        return Q33
        
class inverse_quaternion:
    def Q11_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q11_prime = (Q22*Q33 - Q23*Q32)

        return Q11_prime
    
    def Q12_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q12_prime = -(Q12*Q33 - Q13*Q32)

        return Q12_prime
    
    def Q13_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q13_prime = (Q12*Q23 - Q13*Q22)

        return Q13_prime
    
    def Q21_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q21_prime = -(Q21*Q33 - Q23*Q31)

        return Q21_prime
    
    def Q22_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q22_prime = (Q11*Q33 - Q13*Q31) 

        return Q22_prime
    
    def Q23_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q23_prime = -(Q11*Q23 - Q13*Q21)

        return Q23_prime
    
    def Q31_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q31_prime = (Q21*Q32 - Q22*Q31)

        return Q31_prime
    
    def Q32_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q32_prime = -(Q11*Q32 - Q12*Q31)

        return Q32_prime
    
    def Q33_prime(q0, q1, q2, q3):
        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        Q33_prime = (Q11*Q22 - Q12*Q21)

        return Q33_prime
    
    def Q_prime(q0, q1, q2, q3):

        Q11 = (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)
        Q12 = 2 * (q1 * q2 + q0 * q3)
        Q13 = 2 * (q1 * q3 - q0 * q2)
        
        Q21 = 2 * (q1 * q2 - q0 * q3)
        Q22 = (q0** 2) - (q1 ** 2) + (q2 ** 2) - (q3 ** 2)
        Q23 = 2 * (q2 * q3 + q0 * q1)

        Q31 = 2 * (q1 * q3 + q0 * q2)
        Q32 = 2 * (q2 * q3 - q0 * q1)
        Q33 = (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)

        det_Q = Q11*Q22*Q33 + Q12*Q23*Q31 + Q13*Q21*Q32 - Q13*Q22*Q31 - Q12*Q21*Q33- Q11*Q23*Q32

        return det_Q