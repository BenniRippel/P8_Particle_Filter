#Particle Filter

3rd project in the second term of the Udacity Self Driving Car Nano Degree.
The (incomplete) provided project code can be found [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

Some parts of the following text were copied from the original project description.

## Project Introduction

In this project I implemented a 2D particle filter for localization. 

Input data is:
- Map (sparse, landmark coordinates)
- Initial Location (noisy GPS data)
- Control Data (noisy velocity and yaw angle for each time step)
- Observation Data (noisy observations of landmarks for each time step)

## Running the particle Filter
Once compiled, run the particle filter from the top directory containing the 'data'-folder


If everything worked you should see something like the following output:

```
Time step: 2444

Cumulative mean weighted error: x .1 y .1 yaw .003

Runtime (sec): 1.8

Success! Your particle filter passed!
```


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

> **NOTE**
> The vehicle's coordinate system is NOT the map coordinate system.

## Success Criteria
If the particle filter passes the current grading code (you can make sure you have the current version at any time by doing a `git pull`), then you should pass! 

The two things the grading code is looking for are:

1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` (maximum allowed error in x or y) and `max_yaw_error` in `src/main.cpp`.
2. **Performance**: your particle filter should complete execution within the time specified by `max_runtime` in `src/main.cpp`.



