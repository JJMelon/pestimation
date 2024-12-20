from pyomo.environ import Var, ConcreteModel, ConstraintList, NonNegativeReals, Objective, minimize, SolverFactory, value, expr
from models.components import unbalanced_line
from models.components import slack
from models.components import transformer
from pyomo.environ import Constraint
import sys
def example(network, settings):
    # Assign nodes
    num_buses = len(network.buses) # Each node can have up to 4 buses
    num_slack = len(network.slack)
    num_xfmr = len(network.transformers)
     
    print("num_buses: ", num_buses)
    print("num_slack_nodes: ", num_slack)
    print("num_xfmr_nodes: ", num_xfmr)
    # Print the total number of equations
    num_eq = 2*num_buses + 2*num_slack + 4*num_xfmr
    print("Number of ipopt equations: ", num_eq)
    print("-----------------------------------------------------")

    model = ConcreteModel()
    model.name = "dist_opt"

    model.ipopt_vr_list = Var(range(num_buses))
    model.ipopt_vi_list = Var(range(num_buses))

    



   
    


    


    
    





