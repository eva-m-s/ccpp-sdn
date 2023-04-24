import itertools
import numpy as np
import math


def test_opt(num_switches, num_controllers, distances, max_load, switch_loads):
    # generate all possible binary arrays of size n x m
    pz = list(itertools.product([0, 1], repeat=num_switches * num_controllers))

    # convert binary arrays to 2D numpy arrays
    p = np.array(pz).reshape(-1, num_switches, num_controllers)
    z = np.array(pz).reshape(-1, num_switches, num_controllers)

    # -----------------------------------------------------------------------------------------------------------------
    # filtered2_z = np.array(filtered2_z)
    #print("Step 0: len(z): ", len(z))
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
    # print("Step 1: len(filtered_z): ", len(filtered_z))
    # print(filtered_z)

    # -----------------------------------------------------------------------------------------------------------------
    #print("Step 0: len(p): ", len(p))
    filtered_p = []
    # Iterate over each array in p
    for array in p:
        # Check if each controller is placed at exactly one location
        if np.all(np.sum(array, axis=0) == 1):
            # Check if each location for controller can only be used once
            if np.all(np.sum(array, axis=1) <= 1):
                # Append the array to the list of valid arrays
                filtered_p.append(array)
    #print("Step 2: len(filtered2_p): ", len(filtered_p))
    filtered_p = np.array(filtered_p)

    #------------------------------------------------------------------------------------------------------------------

    obj_array = []
    for p_array in filtered_p:
        for z_array in filtered_z:
            obj = 0
            for i in range(num_switches):
                for j in range(num_controllers):
                    for k in range(num_switches):
                        #obj += distances[i][k] * p_array[k][j] * z_array[i][j]
                        obj += np.max(np.sum(distances[i][k] * p_array[k][j] * z_array[i][j]), 0)

            obj_array.append(obj)
    min_obj = np.min(obj_array)

    #print("Minimum distance: ", min_obj)

    return min_obj
