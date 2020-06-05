import numpy as np
import scipy.io as sio
import sys
import matplotlib.pyplot as plt
from matplotlib import animation

def overlay_bounds():
    #Add obstacles
    points1 = [[-1, 0.1],[16,0.1],[16, 5],[30, 5],[30,4],[20,4],[20,-0.7],[-1,-0.7]]
    polygon1 = plt.Polygon(points1, fill=None, edgecolor='tab:brown')
    plt.gca().add_patch(polygon1)
    
z = sio.loadmat('expected_trajectory.mat')['path_z_pos_dat'].ravel()
x = sio.loadmat('expected_trajectory.mat')['path_x_pos_dat'].ravel().ravel()
alpha = sio.loadmat('expected_trajectory.mat')['path_a_dat'].ravel().ravel()
theta = sio.loadmat('expected_trajectory.mat')['path_theta_pos_dat'].ravel()
L = sio.loadmat('expected_trajectory.mat')['L'][0][0]
tout = sio.loadmat('expected_trajectory.mat')['tout'].ravel()

#Set path to use correct ffmpeg
plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'
quad_l = 0.2
framerate = 25
ts = 1/framerate
dpi = 300



tout_interp = np.arange(0,tout[-1] +ts,ts)
z_interp = np.interp(tout_interp, tout, z)
x_interp = np.interp(tout_interp, tout, x)
alpha_interp = np.interp(tout_interp, tout, alpha)
theta_interp = np.interp(tout_interp, tout, theta)

#Create and set figure
fig = plt.figure()
fig.set_dpi(dpi)
fig.set_size_inches(12*1.5, 3*1.5)
# plt.axis("equal")
ax = plt.axes(xlim=(-1, 27), ylim=(-1,6))

#Create the components of the animation
link = plt.Line2D((-1, 1), (-1, 1), lw=0.5)
mass = plt.Circle((5, -5), 0.05, fc='b')
quad = plt.Line2D((-1, 1), (-1, 1), lw=2,marker='.',markersize=2)

#This function runs to initialize animation
def init():
    overlay_bounds()
    ax.add_line(link)
    ax.add_patch(mass)
    ax.add_line(quad)
    return link, mass, quad

#function that is called by the animation
def animate(i):
    print (i)
    #Calculate positions
    z_quad_center = z_interp[i]
    x_quad_center = x_interp[i]
    z_mass = z_quad_center-L*np.cos(alpha_interp[i])
    x_mass = x_quad_center-L*np.sin(alpha_interp[i])
    
    x_quad = [x_quad_center - quad_l*np.cos(theta_interp[i])  , 
              x_quad_center + quad_l*np.cos(theta_interp[i])]
    y_quad = [z_quad_center + quad_l*np.sin(theta_interp[i])  , 
              z_quad_center - quad_l*np.sin(theta_interp[i])]
    
    #update quad position
    quad.set_xdata(x_quad)
    quad.set_ydata(y_quad)
    
    #Update mass position
    mass.center = (x_mass, z_mass)
    
    #Update link position
    link.set_xdata([x_quad_center,x_mass])
    link.set_ydata([z_quad_center,z_mass])
    

    return link, mass, quad
 


anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=z_interp.shape[0], 
                               interval=ts*1000,
                               blit=True)
if (len(sys.argv) > 1):

    video_name = sys.argv[1]
    anim.save(video_name,dpi = 400, writer='ffmpeg')
else:
    plt.show()