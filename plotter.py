import matplotlib.pyplot as plt

def plot_results(time, controller):
    plt.figure(figsize=(12, 8))
    plt.subplot(311); plt.plot(time, controller.kp, 'r'); plt.ylabel('Kp')
    plt.subplot(312); plt.plot(time, controller.ki, 'g'); plt.ylabel('Ki')
    plt.subplot(313); plt.plot(time, controller.kd, 'b'); plt.ylabel('Kd'); plt.xlabel('Time')
    plt.suptitle("Kp, Ki, Kd")

    plt.figure(); plt.plot(time, controller.yout, label='yout')
    plt.plot(time, [1.0]*len(time), 'r--', label='Setpoint')
    plt.legend(); plt.title("System Response")

    plt.figure(); plt.plot(time, controller.error); plt.title("Error")

    plt.figure(); plt.plot(time, controller.u); plt.title("Control Output u(t)")
    plt.tight_layout(); plt.show()
""",

    "binder.py": """
class BinderPage:
    def __init__(self, controller):
        self.controller = controller

class BinderSpace:
    def __init__(self):
        self.pages = []

    def add_page(self, controller):
        self.pages.append(BinderPage(controller))
