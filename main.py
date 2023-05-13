from gekko import GEKKO
import numpy as np
import random
import tester
import tester_arg
import algorithm


def sdn_opt():
    # Initialize Model
    m = GEKKO(remote=False)

    # Define the number of switches and controllers
    num_switches = 8
    num_controllers = 4

    # Define the maximum load for each controller
    #max_load = [20, 30]
    #max_load = np.random.randint(20, 51, size=num_controllers)
    max_load = np.full(num_controllers, 25)



    # Define the loads needed to control each switch
    switch_loads = np.random.randint(1, sum(max_load)/num_switches, size=num_switches)
    # switch_loads = [6, 7, 8, 5, 9, 4]

    # Create set of switches
    switches = list(range(num_switches))

    # Define the range of x and y coordinates
    x_range = (0, 90)
    y_range = (0, 180)

    # Assign a location to each switch
    locations = {}
    for switch in switches:
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        locations[switch] = (x, y)
    # print(locations)

    # locations = {0: (87.13480429958184, 131.46035514677558), 1: (71.9067902975842, 99.83029253343719),
    #               2: (77.73349581052071, 119.17652894822236), 3: (34.78931461059261, 1.5007109517516648),
    #               4: (38.911456566397185, 135.4659654996168), 5: (63.77012157403705, 123.31182629614602)}

    # Define the distance matrix as a 2D array
    d = np.zeros((len(switches), len(switches)))
    for i, switch_i in enumerate(switches):
        for j, switch_j in enumerate(switches):
            if i != j:
                xi, yi = locations[switch_i]
                xj, yj = locations[switch_j]
                dist = np.sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
                d[i][j] = d[j][i] = dist

    # Define the decision variables
    z = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)
    p = m.Array(m.Var, (num_switches, num_controllers), lb=0, ub=1, integer=True)

    # Initialize a binary variable z[i, j]
    for i in range(num_switches):
        for j in range(num_controllers):
            z[i, j] = m.Var(lb=0, ub=1, integer=True)

    # Initialize a binary variable p[i, j]
    for i in range(num_switches):
        for j in range(num_controllers):
            p[i, j] = m.Var(lb=0, ub=1, integer=True)

    # Each switch must be assigned to exactly one controller
    for i in range(num_switches):
        m.Equation(m.sum(z[i, :]) == 1)

    # The loads on each controller cannot exceed its capacity
    for j in range(num_controllers):
        m.Equation(m.sum([switch_loads[i] * z[i, j] for i in range(num_switches)]) <= max_load[j])

    # Each controller must be placed at exactly one location
    for j in range(num_controllers):
        m.Equation(m.sum([p[i, j] for i in range(num_switches)]) == 1)

    # Each location for controller can only be used once
    for i in range(num_switches):
        m.Equation(m.sum([p[i, j] for j in range(num_controllers)]) <= 1)

    # Define the objective function
    obj = 0
    for i in range(num_switches):
        for j in range(num_controllers):
            for k in range(num_switches):
                # obj = m.max2(m.sum([d[i][k] * p[k][j] * z[i][j]]), 0)
                # obj += m.max2(m.sum([d[i][k] * p[k][j] * z[i][j]]), 0)
                obj += m.sum([d[i][k] * p[k][j] * z[i][j]])

    m.Obj(obj)
    m.options.SOLVER = 1
    m.solve()
    #
    # # tester.test_opt()
    print("Switches assignment: \n", z)
    print("\nControllers placement: \n", p)
    #
    if m.options.APPSTATUS == 1:  # solution successful
        print("Objective function value =", m.options.ObjFcnVal)
    else:
        print("Solution not found")

    # Tester
    # tester_min = tester_arg.test_opt(num_switches, num_controllers, d, max_load, switch_loads)
    # print("Minimum distance: ", tester_min)

    # Algorithm
    algorithm.cap_controller_placement(switches_loads=switch_loads, switches_amount=num_switches, distances_matrix=d)


if __name__ == '__main__':
    sdn_opt()

