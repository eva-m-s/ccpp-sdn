from gekko import GEKKO
import numpy as np
import random

def min_num_controllers(switches_loads, switches_amount, distance_matrix, radius, k_controllers):
    # Set variables
    num_switches = switches_amount
    num_controllers = num_switches
    d = distance_matrix
    r = radius

    # Initialize Model
    m = GEKKO(remote=False)

    # Define the maximum load for each controller
    # max_load = [20, 20, 20, 20,  30]
    # max_load = np.random.randint(20, 51, size=num_controllers)
    # max_load = np.full(num_controllers, 25)
    max_load = [1, 50, 1, 1, 30, 1, 1, 1]

    # Define the loads needed to control each switch
    # Make a copy of the primary array
    random_max_load = max_load.copy()
    random.shuffle(random_max_load)
    k_max_loads = random_max_load[:k_controllers]
    # k_max_loads = random_max_load[:4]
    # switch_loads = np.random.randint(5, sum(k_max_loads) / num_switches, size=num_switches)
    # switch_loads = np.random.randint(1, sum(max_load)/num_switches, size=num_switches)
    switch_loads = [10, 10, 10, 10, 10, 10, 10, 10]
    switch_loads = switches_loads

    # Define the decision variables
    z = m.Array(m.Var, (num_switches, num_controllers))
    c = m.Array(m.Var, num_controllers)

    # Initialize the variables
    for i in range(num_switches):
        for j in range(num_controllers):
            z[i, j] = m.Var(lb=0, ub=1)
    for j in range(num_controllers):
        c[j] = m.Var(lb=0, ub=1)

    # Define constraints
    # The loads on each controller cannot exceed its capacity
    for j in range(num_controllers):
        m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j] * c[j])


    # Each switch must be assigned to exactly one controller
    for i in range(num_switches):
        m.Equation(m.sum(z[i, :]) == 1)

    # # Each switch must be assigned to exactly one controller
    # for i in range(num_controllers):
    #     m.Equation(m.sum(z[:, i]) <= 1)

    # Distance between each switch and its assigned controller must be less than or equal to r
    for i in range(num_switches):
        for j in range(num_controllers):
            m.Equation(d[i][j] * z[i, j] <= r)


    # Define the objective function to minimize the number of used controllers'
    obj = m.sum(c)
    m.Obj(obj)

    # Set solver options
    m.options.SOLVER = 1
    m.options.IMODE = 3
    m.solve()


    if m.options.APPSTATUS == 1:  # solution successful
        #print("Objective function value =", m.options.ObjFcnVal)
        print('Z: ', z)
        print('C: ', c)
        return m.options.ObjFcnVal
    else:
        return False
        print("Solution not found")


