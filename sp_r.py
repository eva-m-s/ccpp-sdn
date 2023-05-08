from gekko import GEKKO
import numpy as np
import random


def min_num_controllers(switches_amount, distance_matrix, radius):
    # Set variables
    num_switches = switches_amount
    num_controllers = num_switches
    d = distance_matrix
    r = radius

    # Initialize Model
    m = GEKKO(remote=False)

    # Define the maximum load for each controller
    max_load = [20, 20, 20, 20, 30]
    #max_load = np.random.randint(20, 51, size=num_controllers)

    # Define the loads needed to control each switch
    #switch_loads = np.random.randint(1, sum(max_load)/num_switches, size=num_switches)
    switch_loads = [20, 11, 8, 7]

    # Define the decision variables
    z = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)
    c = m.Array(m.Var, num_controllers, lb=0, ub=1, integer=True)

    # Initialize a binary variable z[i, j]
    for i in range(num_switches):
        for j in range(num_controllers):
            z[i, j] = m.Var(lb=0, ub=1, integer=True)

    # Define constraints
    # The loads on each controller cannot exceed its capacity
    for j in range(num_controllers):
        m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j])


    # Each switch must be assigned to exactly one controller
    for i in range(num_switches):
        m.Equation(m.sum(z[i, :]) == 1)

    # Calculate the number of controllers used
    # Each switch must be assigned to at least one controller
    # The equation ensures that the sum of z[i,j] for all j is at least 1
    for i in range(num_switches):
        m.Equation(m.sum([z[i, j] for j in range(num_controllers)]) * num_controllers >= c[i])

    # Calculate the total number of switches assigned to controller j
    # The equation ensures that the sum of z[i,j] for all i is less than or equal to c[j] times the total number of switches
    for j in range(num_controllers):
        m.Equation(m.sum([z[i, j] for i in range(num_switches)]) <= num_switches * c[j])

    # Distance between each switch and its assigned controller must be less than or equal to r
    for i in range(num_switches):
        for j in range(num_controllers):
            m.Equation(d[i][j] * z[i, j] <= r)


    # Define the objective function to minimize the number of used controllers
    obj = m.sum(c)
    m.Obj(obj)

    # Set solver options
    m.options.SOLVER = 1
    m.solve()


    # Initialize an empty list to store used distances
    used_distances = []
    if m.options.APPSTATUS == 1:  # solution successful
        # Iterate through the z array and check if z[i,j] is equal to 1
        for i in range(num_switches):
            for j in range(num_controllers):
                if z[i, j].value[0] == 1:
                    # If switch i is assigned to controller j, append distance d[i][j] to the list
                    used_distances.append(d[i][j])

        # Convert the list of used distances to a numpy array if needed
        used_distances = np.array(used_distances)
        total_distance = np.sum(used_distances)
        print('Distances array:', used_distances)
        print('Num of controllers: ', m.options.ObjFcnVal)
        print('Total distance:', total_distance)
        print(z)
        return True
    else:
        print('Placement not found')
        return False



