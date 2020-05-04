import numpy as np

def polynomial_interp_5ord(state1, state2, dt_des):
    """Function that fits a fifth order polynomial interpolation between
    2 given states
    Args:
        state1: initial state [x,y,z,vx,vy,vz,ax,ay,az]
        state2: final state
        dt_des: desired time step

    Returns:
        cost: cost of taking this path 
        X: a matrix containing path states
    """
    
    coeffs = []
    
    #calculate the distance between 2 states and use that to get dt
    dist = np.linalg.norm(state1[0:3] - state2[0:3])
    if dist < 0.5:
        dt = 1
    else:
        dt = dist/0.5  #move at 1 m/s
    
    for i in range (3):
        x1 = state1[i]
        x1d = state1[i+3]
        x1dd = state1[i+6]
        
        x2 = state2[i]
        x2d = state2[i+3]
        x2dd = state2[i+6] 
        
    
        A = np.array([[0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 2, 0, 0],
                      [1*dt**5, 1*dt**4, 1*dt**3, 1*dt**2, 1*dt, 1],
                      [5*dt**4, 4*dt**3, 3*dt**2, 2*dt, 1, 0],
                      [20*dt**3, 12*dt**2, 6*dt, 2, 0, 0]])
        
        b = np.array([x1, x1d, x1dd, x2, x2d, x2dd]).reshape(-1,1)
        
        y = np.linalg.solve(A,b)   
                
        coeffs.append(y.reshape(1,-1))   
    
    # Calculate cost due to distance
    N = int(dt/float(dt_des))
    t = np.linspace(0,dt - dt/N, N)

    T = np.zeros((6,N))
    T[0,:] = t**5
    T[1,:] = t**4
    T[2,:] = t**3
    T[3,:] = t**2
    T[4,:] = t
    T[5,:] = np.ones(N)

    x = coeffs[0].dot(T)
    y = coeffs[1].dot(T)
    z = coeffs[2].dot(T)

    dx = x[0,1:N] - x[0,0:N-1]
    dy = y[0,1:N] - y[0,0:N-1]
    dz = z[0,1:N] - z[0,0:N-1]
    
    ds2 = dx**2 + dy**2 + dz**2
    ds = np.sqrt(ds2)
    s = np.sum(ds)
    
    #calculate velocities
    T[0,:] = t**4
    T[1,:] = t**3
    T[2,:] = t**2
    T[3,:] = t**1
    T[4,:] = np.ones(N)
    T[5,:] = np.zeros(N)

    c_vel = np.diag([5, 4, 3, 2, 1, 0])
    
    vx = coeffs[0].dot(c_vel).dot(T)
    vy = coeffs[1].dot(c_vel).dot(T)
    vz = coeffs[2].dot(c_vel).dot(T)
    
    v2 = vx**2 + vy**2 + vz**2
    v = np.sum(np.sqrt(v2))
    
    #calculate accelerations
    T[0,:] = t**3
    T[1,:] = t**2
    T[2,:] = t**1
    T[3,:] = np.ones(N)
    T[4,:] = np.zeros(N)
    T[5,:] = np.zeros(N)

    c_acc = np.diag([20, 12, 6, 2, 0, 0])
    
    ax = coeffs[0].dot(c_acc).dot(T)
    ay = coeffs[1].dot(c_acc).dot(T)
    az = coeffs[2].dot(c_acc).dot(T)
    
    a2 = ax**2 + ay**2 + az**2
    a = np.sum(np.sqrt(a2))
    
    
    #Combine distance and input to cost
    cost = 1.0 * s + 100 * v + 1000* a 

    #Create a matrix of all points
    X = np.zeros((9,N))
    X[0,:] = x.flatten()
    X[1,:] = y.flatten()
    X[2,:] = z.flatten()
    X[3,:] = vx.flatten()
    X[4,:] = vy.flatten()
    X[5,:] = vz.flatten()
    X[6,:] = ax.flatten()
    X[7,:] = ay.flatten()
    X[8,:] = az.flatten()
    
    return cost, X