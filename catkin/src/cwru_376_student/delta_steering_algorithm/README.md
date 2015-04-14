# delta_steering_algorithm

Code that implements a "clever steering algorithm" to maintain the robot at the proper speed and 
adhere to commands sent by the delta_des_state_generator 

## Example usage
Start up Jinx or cwruBot

Start up desired state generation:
`rosrun example_des_state_generator des_state_generator`

Start the steering algorithm with:
`rosrun example_steering_algorithm example_steering_algorithm'

Give the desired state generator a path, e.g. by running:
`rosrun example_des_state_generator example_path_sender`



## Running tests/demos
    