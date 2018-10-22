import numpy as np
import matplotlib.pyplot as plt

def main():

    # our system definition - vanilla second order system
    A = np.matrix([[1, 0.1], [0, 1]])
    B = np.matrix([[0], [0.1]])

    # run for 100 timesteps
    T = 100

    # start out at (10, 0)
    q = np.array([[10], [0]])
    qdata = []

    # our fixed gain matrix
    K_fixed = np.matrix([[0.5, 1.5]])

    ##################################################
    # simulate the system

    # for each timestep
    for i in range(T):

        # append the current state
        qdata.append(q)

        # compute a control
        u = -K_fixed*q

        # append the next state
        q = A*q + B*u

    # plot the trajectory
    qdata = np.array(qdata)
    plt.plot(qdata[:,0])
    plt.xlabel('Timestep')
    plt.ylabel('Position (m)')
    plt.title('Non-optimal control')
    plt.show()

if __name__ == '__main__':
    main()
