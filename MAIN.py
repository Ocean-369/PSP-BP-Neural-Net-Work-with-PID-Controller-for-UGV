from simulation import generate_pid_data
from control import PSOBPNNPIDController
from plotter import plot_results
from binder_theo import BinderSpace

X, Y, time = generate_pid_data()
controller = PSOBPNNPIDController()
controller.run(X, Y)

space = BinderSpace()
space.add_page(controller)

plot_results(time, controller)
print("Kp first 10:", controller.kp[:10])
print("Ki first 10:", controller.ki[:10])
print("Kd first 10:", controller.kd[:10])
