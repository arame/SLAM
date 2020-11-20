import numpy as np
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns
from robot_class import Robot
from helpers import display_world, make_data

def main():
    # world parameters
    num_landmarks      = 5        # number of landmarks
    N                  = 20       # time steps
    world_size         = 100.0    # size of world (square)

    # robot parameters
    measurement_range  = 50.0     # range at which we can sense landmarks
    motion_noise       = 2.0      # noise in robot motion
    measurement_noise  = 2.0      # noise in the measurements
    distance           = 20.0     # distance by which robot (intends to) move each iteratation 

    # make_data instantiates a robot, AND generates random landmarks for a given world size and number of landmarks
    data = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)    
    # print out some stats about the data
    time_step = 0

    print('Example measurements: \n', data[time_step][0])
    print('\n')
    print('Example motion: \n', data[time_step][1])

    # define a small N and world_size (small for ease of visualization)
    N_test = 5
    num_landmarks_test = 2
    small_world = 10

    # initialize the constraints
    initial_omega, initial_xi = initialize_constraints(N_test, num_landmarks_test, small_world)

    plt.rcParams["figure.figsize"] = (10,7)

    # display omega (need to convert omega to a 2x2 matrix for the heatmap to show)
    display_omega = []
    for i in range(len(initial_omega)):
        for k in range(2):
            row = []
            for j in range(len(initial_omega)):
                item = initial_omega[i][j]
                for l in range(2):
                    row.append(item[k][l])
            display_omega.append(row)

    sns.heatmap(display_omega, cmap='Blues', annot=True, linewidths=.5)
    plt.show()
    # define  figure size
    plt.rcParams["figure.figsize"] = (1,7)

    # display xi
    sns.heatmap(DataFrame(initial_xi), cmap='Oranges', annot=True, linewidths=.5)
    #plt.show()
    ## TODO: Complete the code to implement SLAM
    # call your implementation of slam, passing in the necessary parameters
    mu = slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise)

    # print out the resulting landmarks and poses
    if(mu is not None):
        # get the lists of poses and landmarks
        # and print them out
        poses, landmarks = get_poses_landmarks(mu, N, num_landmarks)
        print_all(poses, landmarks)

    print("*** THE END ***")

## slam takes in 6 arguments and returns mu, 
## mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    ## TODO: Use your initilization to create constraint matrices, omega and xi
    omega, xi = initialize_constraints(N, num_landmarks, world_size)
    ## TODO: Iterate through each time step in the data
    ## get all the motion and measurement data as you iterate
    for (measurement, motion) in data:
        i = 0
    ## TODO: update the constraint matrix/vector to account for all *measurements*
    ## this should be a series of additions that take into account the measurement noise

    ## TODO: update the constraint matrix/vector to account for all *motion* and motion noise

    ## TODO: After iterating through all the data
    ## Compute the best estimate of poses and landmark positions
    ## using the formula, omega_inverse * Xi
    mu = []
    
    return mu # return `mu`


def get_poses_landmarks(mu, N, num_landmarks):
    # create a list of poses
    poses = []
    for i in range(N):
        poses.append((mu[2*i].item(), mu[2*i+1].item()))

    # create a list of landmarks
    landmarks = []
    for i in range(num_landmarks):
        landmarks.append((mu[2*(N+i)].item(), mu[2*(N+i)+1].item()))

    # return completed lists
    return poses, landmarks

def print_all(poses, landmarks):
    print('\n')
    print('Estimated Poses:')
    for i in range(len(poses)):
        print('['+', '.join('%.3f'%p for p in poses[i])+']')
    print('\n')
    print('Estimated Landmarks:')
    for i in range(len(landmarks)):
        print('['+', '.join('%.3f'%l for l in landmarks[i])+']')

def initialize_constraints(N, num_landmarks, world_size):
    ''' This function takes in a number of time steps N, number of landmarks, and a world_size,
        and returns initialized constraint matrices, omega and xi.'''
    
    ## Recommended: Define and store the size (rows/cols) of the constraint matrix in a variable
    
    ## TODO: Define the constraint matrix, Omega, with two initial "strength" values
    ## for the initial x, y location of our robot
    middle_loc = [[1, 0], [0, 1]]
    side_len = N + num_landmarks
    coords = [[0, 0], [0, 0]]
    omega = [[middle_loc if x==0 and y==0 else coords for x in range(side_len)] for y in range(side_len)]
    xi = [[0] for x in range(side_len)]
    xi[0][0] = int(world_size / 2)
    return omega, xi


def main1():
    print("started")
    print("-------")
    world_size         = 10.0    # size of world (square)
    measurement_range  = 5.0     # range at which we can sense landmarks
    motion_noise       = 0.2      # noise in robot motion
    measurement_noise  = 0.2      # noise in the measurements

    # instantiate a robot, r
    r = Robot(world_size, measurement_range, motion_noise, measurement_noise)

    # print out the location of r
    print(r)

    # define figure size
    plt.rcParams["figure.figsize"] = (5,5)

    # call display_world and display the robot in it's grid world
    print(r)
    display_world(int(world_size), [r.x, r.y])

    # choose values of dx and dy (negative works, too)
    dx = 1
    dy = 2
    r.move(dx, dy)

    # print out the exact location
    print(r)

    # display the world after movement, not that this is the same call as before
    # the robot tracks its own movement
    display_world(int(world_size), [r.x, r.y])

    # create any number of landmarks
    num_landmarks = 3
    r.make_landmarks(num_landmarks)

    # print out our robot's exact location
    print(r)

    # display the world including these landmarks
    display_world(int(world_size), [r.x, r.y], r.landmarks)

    # print the locations of the landmarks
    print('Landmark locations [x,y]: ', r.landmarks)
    # try to sense any surrounding landmarks
    measurements = r.sense()

    # this will print out an empty list if `sense` has not been implemented
    print(measurements)

    data = []

    # after a robot first senses, then moves (one time step)
    # that data is appended like so:
    data.append([measurements, [dx, dy]])

    # for our example movement and measurement
    print(data)
    # in this example, we have only created one time step (0)
    time_step = 0

    # so you can access robot measurements:
    print('Measurements: ', data[time_step][0])

    # and its motion for a given time step:
    print('Motion: ', data[time_step][1])

if __name__ == "__main__":
    main()
