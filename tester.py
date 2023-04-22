import itertools
import numpy as np
import random


# def test_opt(num_switches, num_controllers, distances, max_load, switch_loads, locations):
def test_opt():
    num_switches = 5
    num_controllers = 2
    max_load = [20, 30]
    switch_loads = [10, 9, 8, 7, 9]
    locations = {0: (87.13480429958184, 131.46035514677558), 1: (71.9067902975842, 99.83029253343719),
                 2: (77.73349581052071, 119.17652894822236), 3: (34.78931461059261, 1.5007109517516648),
                 4: (38.911456566397185, 135.4659654996168)}
    switches = list(range(num_switches))  # Define the distance matrix as a 2D array
    d = np.zeros((len(switches), len(switches)))
    for i, switch_i in enumerate(switches):
        for j, switch_j in enumerate(switches):
            if i != j:
                xi, yi = locations[switch_i]
                xj, yj = locations[switch_j]
                dist = np.sqrt((xi - xj) ** 2 + (yi - yj) ** 2)
                d[i][j] = d[j][i] = dist

    # generate all possible binary arrays of size n x m
    pz = list(itertools.product([0, 1], repeat=num_switches * num_controllers))

    # convert binary arrays to 2D numpy arrays
    p = np.array(pz).reshape(-1, num_switches, num_controllers)
    z = np.array(pz).reshape(-1, num_switches, num_controllers)

    # -----------------------------------------------------------------------------------------------------------------
    # filtered2_z = np.array(filtered2_z)
    print("Step 0: len(z): ", len(z))
    filtered_z = []
    # Iterate over each array in z
    for array in z:
        # Check if each switch is assigned to exactly one controller
        if np.all(np.sum(array == 1, axis=1) == 1):
            # Check if the loads on each controller cannot exceed its capacity
            if np.all([np.sum(switch_loads * array[:, j]) <= max_load[j] for j in range(num_controllers)]):
                # If both constraints are satisfied, append the array to the filtered_z list
                filtered_z.append(array)

    filtered_z = np.array(filtered_z)
    print("Step 1: len(filtered_z): ", len(filtered_z))
    print(filtered_z)

    # -----------------------------------------------------------------------------------------------------------------
    print("Step 0: len(p): ", len(p))
    filtered_p = []
    # Iterate over each array in p
    for array in p:
        # Check if each controller is placed at exactly one location
        if np.all(np.sum(array, axis=0) == 1):
            # Check if each location for controller can only be used once
            if np.all(np.sum(array, axis=1) <= 1):
                # Append the array to the list of valid arrays
                filtered_p.append(array)
    print("Step 2: len(filtered2_p): ", len(filtered_p))
    filtered_p = np.array(filtered_p)

    #------------------------------------------------------------------------------------------------------------------
    # for p_v in filtered_z:
    #     for z_v in filtered_z:
    #         obj = 0
    #         obj_array = []
    #         for i in range(num_switches):
    #             for j in range(num_controllers):
    #                 for k in range(num_switches):
    #                     obj += sum([d[i][k] * p_v[k][j] * z_v[i][j]])
    obj_array = []
    for p_array in filtered_p:
        for z_array in filtered_z:
            obj = 0
            for i in range(num_switches):
                for j in range(num_controllers):
                    for k in range(num_switches):
                        obj += d[i][k] * p_array[k][j] * z_array[i][j]
            obj_array.append(obj)
    min_obj = np.min(obj_array)

    print("Minimum distance: ", min_obj)

    return min_obj

  # filtered_z = []
    # # Each switch must be assigned to exactly one controller
    # for array in p:
    #     if np.all(np.sum(array == 1, axis=1) == 1):
    #         filtered_z.append(array)
    # print("Step 1: len(z): ", len(filtered_z))
    #
    # # The loads on each controller cannot exceed its capacity for every possible switch-controller assignment
    # filtered2_z = []
    # for array in filtered_z:
    #     if np.all(np.sum(array == 1, axis=1) == 1):
    #         if np.all([np.sum(switch_loads * array[:, j]) <= max_load[j] for j in range(num_controllers)]):
    #             filtered2_z.append(array)