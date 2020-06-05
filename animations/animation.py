import numpy as np
import scipy.io as sio
import sys
import matplotlib.pyplot as plt
from matplotlib import animation

def overlay_bounds():
    #Add obstacles
    points1 = [[-1, 0.2],[16,0.2],[16, 5],[30, 5],[30,4],[20,4],[20,-0.7],
               [5.5,-0.7],[5.5,-0.45],[5,-0.45],[5,-0.7],[-1,-0.7],[-1,-0.7]]
    polygon1 = plt.Polygon(points1, fill=None, edgecolor='tab:brown', lw = 3)
    plt.gca().add_patch(polygon1)

#Set path to use correct ffmpeg
plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'



quad_l = 0.2
sampled_delay = 0.001 #fixed step of Matlab simulation

desired_interval = 0.02 #animation frame interval
desired_interval_ms = desired_interval * 1000
#ratio of interval differance
#converted to integer
ratio = np.floor( desired_interval / sampled_delay).astype('i8') 

# Load all the values from mat file.
#select every 'ratio' item.
z = sio.loadmat('out.mat')['z'][0,0,::ratio]
x = sio.loadmat('out.mat')['x'][0,0,::ratio]
alpha = sio.loadmat('out.mat')['a'][0,0,::ratio]
theta = sio.loadmat('out.mat')['theta'][::ratio,0]
L = sio.loadmat('out.mat')['L'][0][0]

#Create and set figure
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(12*1.5, 3*1.5)
ax = plt.axes(xlim=(-1, 27), ylim=(-1,6))
#fig.set_size_inches(12,12)
#ax = plt.axes(xlim=(-1, 27), ylim=(-14,14))

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
    #Calculate positions
    z_quad_center = z[i]
    x_quad_center = x[i]
    z_mass = z_quad_center-L*np.cos(alpha[i])
    x_mass = x_quad_center-L*np.sin(alpha[i])
    
    x_quad = [x_quad_center - quad_l*np.cos(theta[i])  , x_quad_center + quad_l*np.cos(theta[i])]
    y_quad = [z_quad_center + quad_l*np.sin(theta[i])  , z_quad_center - quad_l*np.sin(theta[i])]
    
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
                               frames=z.shape[0], 
                               interval=desired_interval_ms,
                               blit=True)

# If argument received, save video with argument name


if (len(sys.argv) > 1):

    video_name = sys.argv[1]
    anim.save(video_name,dpi = 400, writer='ffmpeg')

# plt.show()

