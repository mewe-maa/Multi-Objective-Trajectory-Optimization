from math import pi
from pyomo.environ import *
from pyomo.dae import *


# Save results to use in warm start
def saveOptimizationVariables(mav):
    
    myProfile = {}
    for myVar in mav.m.component_objects(Var, active= True):
        print("Saving Variable: ",myVar) # doctest: +SKIP
        for index in myVar:
            myProfile[myVar.name] = [value(myVar[index]) for index in myVar]
            
    myPyomoVars = {}
    for myVar in mav.m.component_objects(ctype=[Var, ContinuousSet], active= True):        
        myPyomoVars[myVar.name] = myVar
            
    return myProfile, myPyomoVars
        
