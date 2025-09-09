import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

def plot(t, y, title="Step Response"):
    plt.plot(t, y)
    plt.xlabel("Tid [s]")
    plt.ylabel("Output")
    plt.title(title)
    plt.grid()
    plt.show()

def create_loop(Kp, Ki, Kd, Process):
    Controller = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])
    System = ctrl.series(Controller, Process)
    Loop = ctrl.feedback(System)
    return Loop


def autotune(Process):
    t, y = ctrl.step_response(Process)
    u = np.ones(len(t))

    Kp = (y[-1]-y[0]) / (u[-1]-u[0])

    y_target = y[0] + 0.63 * (y[-1] - y[0])
    idx = np.where(y >= y_target)[0][0]
    T63 = t[idx]

    T, L = T63, 0

    Kc = (1.2 * T) / (Kp * max(L, 1e-6)) if L > 0 else 0.6 / Kp
    Ti = 2 * L if L > 0 else T
    Td = 0.5 * L

    return Kc, 1/Ti if Ti > 0 else 0, Kc*Td

#---------------------------------------------------------------------------------------------

Kp, Ki, Kd = 1, 0.1, 0.001
Process = ctrl.TransferFunction([3], [3, 2, 5])

Loop1 = create_loop(Kp, Ki, Kd, Process)
Loop2 = create_loop(*autotune(Process), Process)
plot(*ctrl.step_response(Loop1))
plot(*ctrl.step_response(Loop2))
