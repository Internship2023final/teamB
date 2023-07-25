import threading
from controller import Robot

# Import the two controllers
from head_controller import head_controller
from ik_walk import ik_walk

# Function to stop both controllers gracefully
def stop_controllers(controllers):
    for controller in controllers:
        controller.stop()
        controller.join()
# Create instances of the two controllers
def main():
    robot = Robot()
    head_controller = head_controller(robot)
    ik_walk = ik_walk(robot)
    
    # Start the two controllers as separate threads
    ik_walk_thread = threading.Thread(target=ik_walk.run)
    head_controller_thread = threading.Thread(target=head_controller.run)

    ik_walk_thread.start()
    head_controller_thread.start()

    # Move the robot forward (add your own walking logic here)
    #ik_walk.move_forward()
    # Wait for some time (you can replace this with your own logic)
    robot.step(500)  # Wait for 500 milliseconds (0.5 seconds)

    # Stop the threads when done
    stop_controllers([ik_walk, head_controller])

    # The main thread will continue after the threads are done.
    print("Main thread is done.")


    # Main control loop for walking and tracking
    while robot.step(timestep) != -1:
        # Get the target ball position from the tracking controller
        target_ball_position = head_controller.get_target_ball_position()

        # Update the walking controller with the target ball position
        ik_walk.update_target_position(target_ball_position)

        # Update the walking controller to control the robot's walking
        ik_walk.update_movement()

        # Add a delay to adjust the tracking frequency (you can tune this)
        robot.step(50)  # Wait for 50 milliseconds
    # Stop the tracking controller when the walking loop is done
    head_controller.stop()
    head_controller_thread.join()
    # The main thread will continue after the threads are done.
    print("Main thread is done.")



if __name__ == "__main__":
    main()    

# Main control loop
#while robot.step(timestep) != -1:
#how to implement threads in python
#multi threading