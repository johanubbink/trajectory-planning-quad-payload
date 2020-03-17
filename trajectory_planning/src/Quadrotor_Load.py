import numpy as np
import matplotlib.pylab as plt
from scipy.integrate import odeint
from scipy.integrate import solve_ivp

class Quadrotor:
    def __init__(self, M = 1, m = 0.5, L = 0.5, g = 9.81):
        #constants used in model
        self.M = M
        self.m = m
        self.L = L
        self.g = g
        self.counter  = 0;
        self.trim = (M + m)*g
    #dynamic model of the quadrotor
    def quad(self,t,y):

        #create a local copy of constants
        M = self.M
        m = self.m
        L = self.L
        g = self.g
        
        #get inputs for a given timestep
        #fz, fx = u(t)
        fz = self.trim + self.fz
        #fz = self.trim
        fx = self.fx        
        
        #fz = g*(M + m)
        #fx = 1
        self.counter = self.counter + 1
        #unpack the state vector
        z, zdot, x, xdot, a, adot = y
        
        #calculate the inverse mass matrix
        m_inverse = np.array([[(M + m*np.sin(a)**2)/(M*(M + m)),
                               -m*np.sin(2*a)/(2*M*(M + m)),
                               -np.sin(a)/(L*M)],
                              [-m*np.sin(2*a)/(2*M*(M + m)),
                               (M + m*np.cos(a)**2)/(M*(M + m)),
                               np.cos(a)/(L*M)],
                              [-np.sin(a)/(L*M),
                               np.cos(a)/(L*M),
                               (M + m)/(L**2*M*m)]])
        
        #the dynamic equations for the system
        E = np.array([[fz - m* L * np.cos(a) * adot * adot - (M + m)*g],
                      [fx - m * L * np.sin(a) * adot * adot],
                      [- m * g * L * np.sin(a)]])
        
        #mass matrix time the dynamics
        dynamics = m_inverse.dot(E)
        
        #create state matrix for next time step
        dydt = np.zeros_like(y)
        dydt[0] = zdot
        dydt[1] = dynamics[0] 
        dydt[2] = xdot
        dydt[3] = dynamics[1] 
        dydt[4] = adot
        dydt[5] = dynamics[2] 
        return dydt   

    #implement a quadrotor with a velocity feedback
    def quad_velocity(self,t,y):

        #create a local copy of constants
        M = self.M
        m = self.m
        L = self.L
        g = self.g
        
        #declare the feedback gains
        kz = 15
        kx = 7
        
        #get the velocity references
        
        z_ref = self.z_ref
        x_ref = self.x_ref
        
        #fz = g*(M + m)
        #fx = 1
        self.counter = self.counter + 1
        #unpack the state vector
        z, zdot, x, xdot, a, adot = y
        
        #apply the feedback force
        #fz = kz*(z_ref - zdot) + self.trim
        fz = kz*(z_ref - zdot) + self.trim
        fx = kx*(x_ref - xdot)             
        
        #saturate the controllers if input force is to big
        fz_max = 3.5*(M+m)*g
        fx_max = (M+m)*g*np.sin(np.pi/4)
        
        if (fz > fz_max):
            fz = fz_max
        if (fz < 0):
            fz = 0

        if (fx > fx_max):
            fx = fx_max
        if (fx < -fx_max):
            fx = -fx_max
            
        
        
        #calculate the inverse mass matrix
        m_inverse = np.array([[(M + m*np.sin(a)**2)/(M*(M + m)),
                               -m*np.sin(2*a)/(2*M*(M + m)),
                               -np.sin(a)/(L*M)],
                              [-m*np.sin(2*a)/(2*M*(M + m)),
                               (M + m*np.cos(a)**2)/(M*(M + m)),
                               np.cos(a)/(L*M)],
                              [-np.sin(a)/(L*M),
                               np.cos(a)/(L*M),
                               (M + m)/(L**2*M*m)]])
        
        #the dynamic equations for the system
        E = np.array([[fz - m* L * np.cos(a) * adot * adot - (M + m)*g],
                      [fx - m * L * np.sin(a) * adot * adot],
                      [- m * g * L * np.sin(a)]])
        
        #mass matrix time the dynamics
        dynamics = m_inverse.dot(E)
        
        #create state matrix for next time step
        dydt = np.zeros_like(y)
        dydt[0] = zdot
        dydt[1] = dynamics[0] 
        dydt[2] = xdot
        dydt[3] = dynamics[1] 
        dydt[4] = adot
        dydt[5] = dynamics[2] 
        return dydt       

    #function to apply input at given sampling rate
    def solve_velocity(self,ts,y0,Z_ref,X_ref, max_step = np.inf, t_eval = 0.1):
        y_initial = y0
        result = np.copy(np.array([y0])).T
        t = np.array([0])
        for z_ref, x_ref in zip(Z_ref,X_ref):
            self.z_ref = z_ref
            self.x_ref = x_ref
            out = solve_ivp(self.quad_velocity, [0, ts + t_eval], 
                            y_initial, t_eval = np.arange(0,ts + t_eval,t_eval), 
                            max_step=max_step)
            result = np.concatenate([result, out.y[:,1:]], axis = 1)
            t = np.concatenate([t, out.t[1:] + t[-1]])
            y_initial = out.y[:,-1]
        self.result = result
        self.t = t
        return result,t

    #function to apply input at given sampling rate
    def solve_force(self,ts,y0,Fz,Fx, max_step = np.inf, t_eval = 0.1):
        y_initial = y0
        result = np.copy(np.array([y0])).T
        t = np.array([0])
        for fz, fx in zip(Fz,Fx):
            self.fz = fz
            self.fx = fx
            out = solve_ivp(self.quad, [0, ts + t_eval], 
                            y_initial, t_eval = np.arange(0,ts + t_eval,t_eval), 
                            max_step=max_step)
            #out = solve_ivp(self.quad, [0, ts], y_initial, max_step=0.1)
            result = np.concatenate([result, out.y[:,1:]], axis = 1)
            t = np.concatenate([t[:-1], out.t[:] + t[-1]])
            y_initial = out.y[:,-1]
        self.result = result
        self.t = t
        return result,t    
    
    def solve(self,ts,y0,Z_ref,X_ref,Sim_type,max_step = np.inf,t_eval = 0.1):
        y_initial = y0
        result = np.copy(np.array([y0])).T
        t = np.array([0])
        for z_ref, x_ref, sim_type in zip(Z_ref,X_ref,Sim_type):
            #Force Based
            #if (sim_type == 0):
                
            #Velocity Based
            if (sim_type == 1) or (sim_type == 2):
                self.z_ref = z_ref
                self.x_ref = x_ref
                out = solve_ivp(self.quad_velocity, [0, ts + t_eval], 
                                y_initial, t_eval = np.arange(0,ts + t_eval,t_eval), 
                                max_step=max_step)
                result = np.concatenate([result, out.y[:,1:]], axis = 1)
                t = np.concatenate([t, out.t[1:] + t[-1]])
                y_initial = out.y[:,-1]

            elif (sim_type == 0):

                self.fz = z_ref
                self.fx = x_ref
                out = solve_ivp(self.quad, [0, ts + t_eval], 
                                y_initial, t_eval = np.arange(0,ts + t_eval,t_eval), 
                                max_step=max_step)
                result = np.concatenate([result, out.y[:,1:]], axis = 1)
                t = np.concatenate([t[:-1], out.t[:] + t[-1]])
                y_initial = out.y[:,-1]
            else:
                print ("Not a valid input")
        self.result = result
        self.t = t
        
        return result,t            
        
                
    def simulate(self,y0, t, fz,fx):
        
        self.fz = fz
        self.fx = fx
        self.solution = odeint(self.quad, y0, t, tfirst = True)
        
        
    #return the last simmulated state of quadrotor
    def final_state(self):
        return self.solution[-1,:]
                     
    #function to the states of the quadrotor vs time
    def plot_states(self):
        result_a = self.result
        t_a = self.t
        plt.figure()
        plt.title("Quadrotor States vs Time")
        #plt.plot(t_a,result_a[0,:], label = 'z')
        #plt.plot(t_a,result_a[1,:], label = 'zdot')
        plt.plot(t_a,result_a[2,:], label = r'$x$ (m)')
        plt.plot(t_a,result_a[3,:], label = r'$\dot{x}$ (m/s)')
        plt.plot(t_a,result_a[4,:]*180/np.pi, label = r'$\alpha$ (deg)')
        #plt.plot(t_a,result_a[5,:], label = 'alphadot')
        plt.xlabel('Time')
        plt.ylabel('Magnitude')
        plt.legend(loc= 'best')
        
    #function to plot the postion of the quadrotor
    def plot_position(self):
        result_a = self.result
        t_a = self.t
    
        z_quad_a = result_a[0,:]
        x_quad_a = result_a[2,:]
        alpha_a = result_a[4,:]
        L = 0.5

        z_mass_a = z_quad_a-L*np.cos(alpha_a)
        x_mass_a = x_quad_a-L*np.sin(alpha_a)                 
                     
        
        plt.title("Path Followed by Quadrotor")
        plt.plot(x_mass_a, z_mass_a, label = 'Mass')
        plt.plot(x_quad_a,z_quad_a, label = 'Quadrotor')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        #plt.legend(loc = 'best')
        #plt.show()                 
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     
                     