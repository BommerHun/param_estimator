from vehicle_state_msgs.msg import VehicleStateStamped
from rclpy.node import Node
import casadi as cs
import rclpy
import numpy as np
import matplotlib.pyplot as plt
import time


l = 0.168 + 0.163 #distance of wheelbase


class Listener(Node):
    def __init__(self):
        super().__init__("Listener")
        self.listener = self.create_subscription(VehicleStateStamped, '/JoeBush1_state',self.callback, 0 )
        self.data = np.array([[]])
        self.start_time = time.time()
        self.duration = 3
        self.R = None


    def clear_data(self):
        self.data = None
        self.R = None

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
        try:
            self.data = np.append(self.data, state_vector, axis = 1)
        except Exception as e:
            self.data = state_vector

        if time.time()- self.start_time > self.duration:
            self.interpolate()
            print("Run time ended")
            raise Exception

    def interpolate(self):
        x_data = self.data[0,:]
        y_data = self.data[1,:]

        s = np.linspace(0,np.pi*2, 100)

        """ #TEST DATA
        x_data = np.cos(s)[::10]
        y_data = np.sin(s)[::10]
        """
        
        
        n_points = np.shape(x_data)[0]
        x_c = cs.MX.sym('x_c')
        y_c = cs.MX.sym('y_c')
        R = cs.MX.sym('R')

        objective = 0
        for i in range(n_points):
            dist_squared = (x_data[i]- x_c)**2 + (y_data[i]-y_c)**2
            objective = objective + (cs.sqrt(dist_squared)-R)**2
        
        opt_vars = cs.vertcat(x_c, y_c, R)

        nlp = {'x': opt_vars, 'f': objective}
        opts = {'ipopt.print_level': 0, 'print_time':0}
        solver = cs.nlpsol('solver', 'ipopt', nlp, opts)

        x0 = np.array([0.0,0.0,0.0])
        solution = solver(x0 = x0)

        R = solution['x'][2].full()[0]

        #plt.plot(x_data, y_data)
        #plt.plot(R*np.cos(s), R*np.sin(s))
        #plt.axis("equal")
        #plt.show()

        print(f"solution (x0,y0, R): {solution['x']}")
        self.R = R[0]




def servo_interpolate(data):

    n_points = np.shape(data[0,:])[0]


    k_1 = cs.MX.sym('k_1') 
    k_0 = cs.MX.sym('k_0')

    objective = 0
    

    for i in range(n_points):
        dist_squared = (k_1*data[0,i] + k_0-data[1,i])**2
        objective = objective + dist_squared

    opt_vars = cs.vertcat(k_1, k_0)

    nlp = {'x': opt_vars, 'f': objective}

    opts = {'ipopt.print_level': 0, 'print_time':0}
    solver = cs.nlpsol('solver', 'ipopt', nlp, opts)

    x0 = np.array([1,0])

    solution = solver(x0 = x0)

    k1 = solution['x'][0].full()[0]
    k0 = solution['x'][1].full()[0]

    return k1, k0




def main():
    print('Hi from param_estimator.')
    rclpy.init()
    listener = Listener()

    delta_ref = 0

    data = np.array([])


    command = input()

    while(command != "exit"):
        if command == "help":
            print("Available commands:\nrun-start data collection\nset_duration-set duration of data collection\nset_delta- set value of delta ref[-0.5:0.5]\ninterpolate-interpolate data\nexit-exit")
        elif command == "run":
            try:
                listener.clear_data()
                listener.start_time = time.time()
                rclpy.spin(listener)
            except Exception as e:  
                pass

            try:
                new_data = np.array([delta_ref, cs.arcsin(l/listener.R)])
                data = np.append(data, np.reshape(new_data, (-1,1)), axis = 1)
            except:
                new_data = np.array([delta_ref, cs.arcsin(l/listener.R)])
                data = np.reshape(new_data, (-1,1))
        elif command == "exit":
            break
        elif command == "set_duration":
            listener.duration = float(input("Enter duration:\t"))
        elif command == "set_delta":
            delta_ref = float(input("Enter delta reference:\t"))+0.5
        elif command == "plot_data":
            plt.plot(data[0,:], data[1,:], 'o')
            print(data)
            plt.show()
        elif command == "interpolate":
            k1,k0 = servo_interpolate(data)

            print(f"k1: {k1}\t k0: {k0}")
        else:
            print("help-show help")

        command = input()

        

if __name__ == '__main__':
    main()
