import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.transforms as trans

def main():

    #### Pista 1
    data_vfh = np.genfromtxt('Pista1_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista1_Brait_t_data.csv', delimiter=',')

    D_CIL = 0.1
    ob1 = patch.Ellipse([0.0,0.25],D_CIL,D_CIL,facecolor='grey')

    area = [-0.5,0.5,-0.25,0.75]
    plt.figure(1)
    plt.plot(data_vfh[:,1],data_vfh[:,2],c='blue',linestyle='--',label='VFH')
    plt.plot(data_brait[:,1],data_brait[:,2],c='orange',linestyle='--',label='Braitenberg')
    plt.legend(loc=2)
    plt.scatter(data_vfh[0,1],data_vfh[0,2],c='green')
    plt.scatter(data_vfh[data_vfh.shape[0]-1,1],data_vfh[data_vfh.shape[0]-2,2],c='red')
    #plt.scatter([0.01],[0.25],s=100.0,c='grey')
    plt.axis(area)
    plt.gca().add_patch(ob1)
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Pista 1 con seguimiento')

    plt.figure(2)
    plt.plot(data_vfh[:,0]/1000,data_vfh[:,4],c='blue',label='VFH')
    plt.plot(data_brait[:,0]/1000,data_brait[:,4],c='orange',label='Braitenberg')
    plt.xlabel('t [s]')
    plt.ylabel('d [m]')
    plt.grid(True)
    plt.legend(loc=1)
    plt.title('Pista 1 distancia al objetivo')


    #### Pista 2
    data_vfh = np.genfromtxt('Pista2_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista2_Brait_t_data.csv', delimiter=',')

    D_PLANT = 0.15
    ob1 = patch.Ellipse([0.1,0.325],D_CIL,D_CIL,facecolor='grey')
    ob2 = patch.Ellipse([-0.1,0.575],D_CIL,D_CIL,facecolor='grey')
    ob3 = patch.Ellipse([-0.533,0.2395],D_PLANT,D_PLANT,facecolor='grey')

    area = [-0.75,0.25,-0.25,0.75]
    plt.figure(3)
    plt.plot(data_vfh[:,1],data_vfh[:,2],c='blue',linestyle='--',label='VFH')
    plt.plot(data_brait[:,1],data_brait[:,2],c='orange',linestyle='--',label='Braitenberg')
    plt.scatter(data_vfh[0,1],data_vfh[0,2],c='green')
    plt.scatter(data_vfh[data_vfh.shape[0]-1,1],data_vfh[data_vfh.shape[0]-2,2],c='red')
    plt.legend(loc=2)
    plt.axis(area)
    plt.gca().add_patch(ob1)
    plt.gca().add_patch(ob2)
    plt.gca().add_patch(ob3)
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Pista 2 con seguimiento')

    plt.figure(4)
    plt.plot(data_vfh[:,0]/1000,data_vfh[:,4],c='blue',label='VFH')
    plt.plot(data_brait[:,0]/1000,data_brait[:,4],c='orange',label='Braitenberg')
    plt.xlabel('t [s]')
    plt.ylabel('d [m]')
    plt.grid(True)
    plt.legend(loc=1)
    plt.title('Pista 2 distancia al objetivo')


    #### Pista 3
    data_vfh = np.genfromtxt('Pista3_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista3_Brait_t_data.csv', delimiter=',')


    area = [-0.75,1.0,-0.5,1.25]
    plt.figure(5)
    plt.plot(data_vfh[:,1],data_vfh[:,2],c='blue',linestyle='--',label='VFH')
    plt.plot(data_brait[:,1],data_brait[:,2],c='orange',linestyle='--',label='Braitenberg')
    plt.scatter(data_vfh[0,1],data_vfh[0,2],c='green')
    plt.scatter(data_vfh[data_vfh.shape[0]-1,1],data_vfh[data_vfh.shape[0]-2,2],c='red')
    plt.legend(loc=2)
    plt.axis(area)

    RECT_W = 0.05
    RECT_H = 0.5

    x=0.575
    y=0.7
    g=60.0
    cos = np.cos(np.radians(g))
    sin = np.sin(np.radians(g))
    x_c = RECT_W*cos/2 - RECT_H*sin/2
    y_c = RECT_H*cos/2 - RECT_W*sin/2
    center = [x-x_c,y-y_c]
    ob1 = patch.Rectangle(center,RECT_W,RECT_H,g,facecolor='grey')

    x=0.55
    y=0.325
    g=-70.0
    cos = np.cos(np.radians(g))
    sin = np.sin(np.radians(g))
    x_c = RECT_W*cos/2 - RECT_H*sin/2
    y_c = RECT_H*cos/2 - RECT_W*sin/2
    center = [x-x_c,y-y_c]
    ob2 = patch.Rectangle(center,RECT_W,RECT_H,g,facecolor='grey')

    ob3 = patch.Rectangle([0.225 -(RECT_W/2),-0.025 -(RECT_H/2)],RECT_W,RECT_H,0.0,facecolor='grey')

    x=-0.1
    y=-0.35
    g=60.0
    cos = np.cos(np.radians(g))
    sin = np.sin(np.radians(g))
    x_c = RECT_W*cos/2 - RECT_H*sin/2
    y_c = RECT_H*cos/2 - RECT_W*sin/2
    center = [x-x_c,y-y_c]
    ob4 = patch.Rectangle(center,RECT_W,RECT_H,g,facecolor='grey')

    ob5 = patch.Rectangle([-0.275 -(RECT_W/2),0.125 -(RECT_H/2)],RECT_W,RECT_H,0.0,facecolor='grey')

    x=-0.075
    y=0.525
    g=110.0
    cos = np.cos(np.radians(g))
    sin = np.sin(np.radians(g))
    x_c = RECT_W*cos/2 - RECT_H*sin/2
    y_c = RECT_H*cos/2 - RECT_W*sin/2
    center = [x-x_c,y-y_c]
    ob6 = patch.Rectangle(center,RECT_W,RECT_H,g,facecolor='grey')

    plt.gca().add_patch(ob1)
    plt.gca().add_patch(ob2)
    plt.gca().add_patch(ob3)
    plt.gca().add_patch(ob4)
    plt.gca().add_patch(ob5)
    plt.gca().add_patch(ob6)
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Pista 3 con seguimiento')

    plt.figure(6)
    plt.plot(data_vfh[:,0]/1000,data_vfh[:,4],c='blue',label='VFH')
    plt.plot(data_brait[:,0]/1000,data_brait[:,4],c='orange',label='Braitenberg')
    plt.xlabel('t [s]')
    plt.ylabel('d [m]')
    plt.grid(True)
    plt.legend(loc=1)
    plt.title('Pista 3 distancia al objetivo')

    plt.show()



if __name__ == "__main__":
    main()
