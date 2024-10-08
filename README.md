## **Compile and run**

My code runs on the Windows system, and after successfully compiling it using **Visual Studio 2022**, the following message appears when running it:

`run mode:       node mode:      run times:      start idx:      end idx:`

There are five parameters that need to be filled in, and I provide the meaning of each parameter below:

**-run_mode    -node_mode    -run_times    -start_idx    -end_idx**

**[ run_mode ]**: You can choose one running mode from these two choices:  "*all*" or  "*choose*" .

    *all*: Run all the instances with the code.

    *choose*: Run the selected instances from the **[ start_idx ]**-th to the **[ end_idx ]**-th with the code.

**[ node_mode ]**: You can only use "*normal*" node mode.

    *normal*: Perform  the *problem reduction* process

**[ run_times ]**: let each instance run for run_times times

**[ start_idx ]**: is valid when run_mode is *choose*.  It represents the first chosen instance's index. 

**[ end_idx ]**: is valid when run_mode is *choose*. It represents the last chosen instance's index.

---

## **Folder**

.data/  folder stores all the results.

.Instance/ folder stores all the instances.

.Result/ folder stores the generating resulting.

---

## **Instance file**

This section will explain the meanings of the parameters in the instance file.

**[ NAME ]** : the name of the instance.

**[ COMMENT ]** : the objective value of the best result obtained by state-of-the-art algorithm.

**[ VEHICLE ]** : the number of the vehicles of the best result obtained by state-of-the-art algorithm.

**[ TIME ]** : the longest run time of the current instance (unit: seconds).

**[ DIMENSION ]** : the number of vertices, including depot points and customer points.

**[ CAPACITY ]** : the capacity of vehicle.

**[ DELIVERY_SECTION ]** : each of the following lines will provide three numbers: the first number represents the auto-incrementing sequence number of the delivery demand, the second number indicates which vertice the demand belongs to, and the third number represents the value of this delivery demand.

**[ PICKUP_SECTION ]** : each of the following lines will provide three numbers: the first number represents the auto-incrementing sequence number of the pickup demand, the second number indicates which vertice the demand belongs to, and the third number represents the value of this pickup demand.

**[ NODE_SECTION ]** : each of the following lines will provide the planar coordinates of the vetices in order: the first number represents the x-coordinate, and the second number represents the y-coordinate.

**[ DISTANCE_SECTION ]** : each of the following lines will provide the distance from vertex i ( where i = 0,...,N ) to vertex j ( where j = i+1,...,N). If there is data in the NODE_SECTION part, this information will not be provided.

**[ DEPOT_SECTION ]** : the following number indicates which vertex is the depot.

**[ EOF ]** : the marker for the end of the file.
