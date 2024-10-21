from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
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
        self.d_listener = self.create_subscription(InputValues, '/JoeBush1_control',self.d_callback, 0 )
        self.data = np.array([[]])
        self.start_time = time.time()
        self.duration = 3
        self.dt = 1/30
        self.m = 3
        self.d = 0.15

    def clear_data(self):
        self.data = None
        self.d = 0

    def d_callback(self, msg):
        self.d = float(msg.d)
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

        state_vector[6] = self.d
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
        x_data = self.data[6,:]
        y_data = self.data[3,:]

        

        """ #TEST DATA
        x_data = np.cos(s)[::10]
        y_data = np.sin(s)[::10]
        """
        
        
        n_points = np.shape(x_data)[0]
        C_m1 = cs.MX.sym('C_m1')
        C_m2 = cs.MX.sym('C_m2')
        C_m3 = cs.MX.sym('C_m3')

        sim_y = np.array([y_data[0]])

        for i in range(n_points-1):
            new_y = sim_y[-1] + (2/self.m*(C_m1*x_data[i]-C_m2*sim_y[-1] -cs.sign(sim_y[-1] )*C_m3))*self.dt
            sim_y = np.append(sim_y, new_y)

        objective = 0
        for i in range(n_points):
            dist_squared = (y_data[i]-sim_y[i])**2
            objective = objective + dist_squared
        
        opt_vars = cs.vertcat(C_m1, C_m2, C_m3)

        nlp = {'x': opt_vars, 'f': objective}
        opts = {'ipopt.print_level': 5, 'print_time':0}
        solver = cs.nlpsol('solver', 'ipopt', nlp, opts)

        x0 = np.array([30,3.0,0.8])
        solution = solver(x0 = x0)


        print(f"solution (C_m1,C_m2, C_m3): {solution['x']}")
        
        self.C_m1 = solution['x'][0].full()[0]
        self.C_m2 = solution['x'][1].full()[0]
        self.C_m3 = solution['x'][2].full()[0]



        plt.plot(np.arange(0,n_points), y_data)

        sim_y = np.array([y_data[0]])

        for i in range(n_points-1):
            new_y = sim_y[-1] + (2/self.m*(self.C_m1*x_data[i]-self.C_m2*sim_y[-1] -cs.sign(sim_y[-1] )*self.C_m3))*self.dt
            sim_y = np.append(sim_y, new_y)

        plt.plot(np.arange(0,n_points), sim_y)
        plt.show()
        

        

def drivetrain_interpolate(data):

    n_points = np.shape(data[0,:])[0]

    C_m1 = 0
    C_m2 = 0
    C_m3 = 0
    for i in range(n_points):
        C_m1 += data[0,i]
        C_m2 += data[1,i]
        C_m3 += data[2,i]

    return C_m1, C_m2, C_m3





def main():
    print('Hi from param_estimator.')
    rclpy.init()
    listener = Listener()
    data = np.array([])


    command = input()

    while(command != "exit"):
        if command == "help":
            print("Available commands:\nrun-start data collection\nset_duration-set duration of data collection\nset_delta- set value of delta ref[-0.5:0.5]\ninterpolate-interpolate data\nexit-exit")
        elif command == "run":
            try:
                listener.clear_data()
                listener.start_time = time.time()
                print("spin started")
                rclpy.spin(listener)
            except Exception as e:  
                print(e)
                command = input()
                continue

            try:
                new_data = np.array([listener.C_m1, listener.C_m2, listener.C_m3])
                data = np.append(data, np.reshape(new_data, (-1,1)), axis = 1)
                pass
            except Exception as e:
                print(e)
        elif command == "exit":
            break
        elif command == "set_duration":
            listener.duration = float(input("Enter duration:\t"))
        elif command == "set_dt":
            listener.dt = float(input("Enter sampling time:\t"))
        elif command == "set_freq":
            listener.dt = 1/float(input("Enter sampling frequency:\t"))
        elif command == "plot_data":
            plt.plot(data[0,:], data[1,:], 'o')
            print(data)
            plt.show()
        elif command == "set_d":
            listener.d = float(input("Enter motor reference, d:\t"))
        elif command == "interpolate":
            C_m1, C_m2, C_m3 = listener.interpolate(data)
            print(f"solution C_m1: {C_m1}\tC_m2: {C_m2}\tC_m3: {C_m3}")

        else:
            print("help-show help")

        command = input()

        

if __name__ == '__main__':
    main()
