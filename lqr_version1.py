import numpy as np
import matplotlib.pyplot as plt

def main():

    # our system definition - vanilla second order system
    A = np.matrix([[1, 0.1], [0, 1]])
    B = np.matrix([[0], [0.1]])

    # penalty for position, velocity
    Q = np.matrix([[5, 0], [0, 1]])

    # penalty for controls
    R = np.matrix([[0.1]])

    # run for 100 timesteps
    T = 100

    ##################################################
    # pre-compute my K, V matrices

    Kmats = [ np.matrix([[0, 0]]) ]

    V = Q
    Vmats = [ V ]

    for i in range(T-1):

        K = np.linalg.inv( B.T * V * B + R ) * B.T * V * A

        ABthing = A - B*K
        V = Q + K.T * R * K + ABthing.T * V * ABthing

        Kmats.insert(0, K)
        Vmats.insert(0, V)

    plt.plot([Ki[0,0] for Ki in Kmats], label='pos gain (kp)')
    plt.plot([Ki[0,1] for Ki in Kmats], label='vel gain (kd)')

    plt.legend()
    plt.xlabel('Timestep')
    plt.ylabel('Gain value')
    plt.title('Optimized gains from LQR')
    plt.show()

    '''
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
    '''

if __name__ == '__main__':
    main()
