from random import random
import cmath
import numpy as np
import matplotlib.pyplot as plt
"""
def vector (a,b):
    vec = a-b


x_initial = 0
y_initial = 0
z_nitial = complex(x_initial,y_initial)

x_goal = 10
y_goal = 10
z_goal = complex(x_goal,y_goal)

x = np.arange(-10,10,1)
y = np.arange(-10,10,1)

X,Y = np.meshgrid(x,y)

delx = np.zeros(X)
dely = np.zeros(Y)

goal = [50,50]
obstacle = [25,25] """
# creating two evenly spaced array with ranging from 
# -10 to 10
x = np.arange(-10,10,1)
y = np.arange(-10,10,1)

# Creating the meshGrid
X, Y = np.meshgrid(x,y)

# creating the delx and Dely
delx = np.zeros_like(X)
dely = np.zeros_like(Y)
s = 100
r=2
for i in range(len(x)):
  for j in range(len(y)):
        
    d= np.sqrt(X[i][j]**2 + Y[i][j]**2)
    #print(f"{i} and {j}")
    theta = np.arctan2(Y[i][j],X[i][j])

    # using the Formula of avoiding obstacle
    if d< 2:
      delx[i][j] = np.sign(np.cos(theta))
      dely[i][j] = np.sign(np.cos(theta))
    elif d>r+s:
      delx[i][j] = 0
      dely[i][j] = 0
    else:
      delx[i][j] = 1000*(s+r-d)* np.cos(theta) #it just givrs the component by getting a large value it makes more accurate
      dely[i][j] = 1000*(s+r-d)*np.sin(theta)
      print(delx[i][j])
      print(dely[i][j])

fig, ax = plt.subplots(figsize = (10,10))
ax.quiver(X, Y, delx, dely)
ax.add_patch(plt.Circle((0, 0), 2, color='m'))
#ax.annotate("Obstacle", xy=(0, 0), fontsize=20, ha="center")
#ax.set_title('Repulsive field of the obstacle')
plt.show() 