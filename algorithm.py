import numpy as np
import sp_lr_r
import sp_r
import sp_r_1
import sp_r_1_nr
import sp_r_2


def cap_controller_placement(switches_loads, switches_amount, distances_matrix):
    # Step 1: Sort distances in ascending order
    # Flatten the distance matrix into a 1D array
    dis_array = distances_matrix.flatten()
    # Sort the 1D array in ascending order
    dis_array = np.sort(dis_array)

    # Step 2: Binary search for the minimum radius r
    min_radius = None
    min_controllers = None
    k = 3
    lower = 0
    upper = len(dis_array) - 1
    # mid = (lower + upper) // 2
    # r = dis_array[upper]
    # num_controllers = sp_r_1.min_num_controllers(switches_loads=switches_loads, switches_amount=switches_amount,
    #                                                     distance_matrix=distances_matrix, radius=r, k_controllers=k)
    while lower < upper:
        mid = (lower + upper) // 2
        r = dis_array[mid]
        num_controllers = sp_r_1_nr.min_num_controllers(switches_loads=switches_loads, switches_amount=switches_amount,
                                                        distance_matrix=distances_matrix, radius=r, k_controllers=k)
        # num_controllers = sp_r_1.min_num_controllers(switches_loads=switches_loads, switches_amount=switches_amount,
        # distance_matrix = distances_matrix, radius = r, k_controllers = k)

        if num_controllers:
            if num_controllers > k:
                lower = mid + 1
            else:
                min_radius = r
                min_controllers = num_controllers
                upper = mid

    print('Step 1:')
    print('Minimum radius: ', min_radius)
    print('Number of controllers: ', min_controllers)
    print('Lower', lower)

    # Step 3: Find placement for minimum radius
    index = lower
    r2 = dis_array[index]
    num_controllers = sp_r_2.min_num_controllers(switches_loads=switches_loads, switches_amount=switches_amount,
                                                    distance_matrix=distances_matrix, radius=r2, k_controllers=k)
    if num_controllers:
        while num_controllers > k:
            index += 1
            r2 = dis_array[index]
            num_controllers = sp_r_2.min_num_controllers(switches_loads=switches_loads, switches_amount=switches_amount,
                                                         distance_matrix=distances_matrix, radius=r2, k_controllers=k)
    print('Step 1:')
    print('Minimum radius: ', min_radius)
    print('Number of controllers: ', min_controllers)
    print('Lower', lower, '\n')
    print('Step 2:')
    print('Minimum radius: ', r2)
    print('Number of controllers: ', num_controllers)
    print('Lower', index)