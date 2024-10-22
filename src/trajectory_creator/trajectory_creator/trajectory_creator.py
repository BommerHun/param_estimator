from vehicle_state_msgs.msg import VehicleStateStamped
from rclpy.node import Node
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
import select
import sys
import matplotlib.pyplot as plt

running = False

position_data = np.array([[]])

# Create a blank figure and axis
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()

# Set up the axis limits
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

# Initialize an empty line
line, = ax.plot([], [], 'o')

def update_plot():
    global position_data
    line.set_xdata(position_data[0,:])
    line.set_ydata(position_data[1,:])
    
    
    
    # Redraw the figure
    fig.canvas.draw()
    fig.canvas.flush_events()
# Function to hide the plot
def hide_plot():
    line.set_visible(False)
    fig.canvas.draw()
    fig.canvas.flush_events()

# Function to show the plot
def show_plot():
    line.set_visible(True)
    fig.canvas.draw()
    fig.canvas.flush_events()

class Listener(Node):
    def __init__(self):
        super().__init__("Listener")
        self.listener = self.create_subscription(VehicleStateStamped, '/JoeBush1_state',self.callback, 0)
    def callback(self, msg):
        state = {
            "x": float(msg.position_x),
            "y": float(msg.position_y),
            "phi": float(msg.heading_angle),
            "v_xi": float(msg.velocity_x),
            "v_eta": float(msg.velocity_y),
            "omega": float(msg.omega),
            "d": float(msg.duty_cycle),
            "delta": float(msg.delta),
            "erpm": float(msg.erpm)
        }
        state_vector = np.array([state["x"], state["y"], state["phi"], state["v_xi"],
        state["v_eta"], state["omega"], state["d"], state["delta"]])
        state_vector = np.reshape(state_vector, (-1,1))

        global position_data
        try:
            position_data = np.append(position_data, np.reshape(state_vector[:2, 0],(-1,1)), axis = 1)
        except Exception as e:
            position_data = np.reshape(state_vector[:2, 0],(-1,1))

def input_peek():
    """ Function to check if user input is available without blocking """
    # Use select to check if data is available on stdin
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def main():
    print('Hi from trajectory_creator.')
    rclpy.init()
    listener = Listener()  # Create the listener node
    executor = SingleThreadedExecutor()  # Create a single-threaded executor
    executor.add_node(listener)  # Add the node to the executor

    global running
    global position_data

    try:
        print("Spinning the node... (type 'exit' to stop)")
        while True:
            if running:
                executor.spin_once(timeout_sec=0.1)  # Spin with a small timeout
                update_plot()

            
            # Check if user types "exit" to break the loop
            user_input = input_peek()
            if user_input == None:
                continue
            elif user_input == "clear":
                position_data = np.array([[]])
            if user_input == "run":
                running = True
                show_plot()
            elif user_input == "stop":
                running = False
            elif user_input == "data":
                position_data
                print(position_data)
            elif  user_input.strip().lower() == "exit":
                break

    except KeyboardInterrupt:
        print("Node interrupted by the user")

    finally:
        executor.shutdown()  # Shutdown the executor
        listener.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown ROS2




if __name__ == '__main__':
    main()
