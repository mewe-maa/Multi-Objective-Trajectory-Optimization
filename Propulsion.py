from pyomo.environ import ConcreteModel, TransformationFactory, Var, \
                          NonNegativeReals, Constraint, \
                          SolverFactory, Objective, cos, sin, minimize, \
                          NonNegativeReals

# characterisitcs velocity efiiciency
eff_cstr = 0.96

# thrust coefficient efiiciency
eff_CF = 0.98

# fuel length
Lb = 0.8

# th
Ath = 5

# specific impulse
Isp = 270

# nozzle expansion ratio (E)
E = 50

# chamber pressure
pc = 3.8e6

# port radius
J = 0.075

# pre-exponential factor
a = 0.5

# oxidizer mass flux index
n = 2

# fuel density
rhof = 1475