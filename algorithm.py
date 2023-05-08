import numpy as np
import sp_lr_r
import sp_r

def cap_controller_placement(switches_amount, distances_matrix):
    # Step 1: Sort distances in ascending order
    # Flatten the distance matrix into a 1D array
    disArray = distances_matrix.flatten()
    # Sort the 1D array in ascending order
    disArray = np.sort(disArray)

    # Step 2: Binary search for the minimum radius r
    min_radius = None
    min_controllers = None
    k = 3
    lower = 0
    upper = len(disArray) - 1

    while lower < upper:
        mid = (lower + upper) // 2
        r = disArray[mid]
        num_controllers = sp_lr_r.min_num_controllers(switches_amount=switches_amount, distance_matrix=distances_matrix, radius=r)

        if num_controllers:
            if num_controllers > k:
                lower = mid + 1
            else:
                min_radius = r
                min_controllers = num_controllers
                upper = mid

    print('Minimum radius: ', min_radius)
    print('Number of controllers: ', min_controllers)
    print('Lower', lower)

    # Step 3: Find placement for minimum radius
    r2 = disArray[lower]
    controllers = sp_r.min_num_controllers(switches_amount=switches_amount, distance_matrix=distances_matrix, radius=r2)
    if controllers:
        if controllers > k:
            lower += 1
    print(disArray[lower])
    print(controllers)