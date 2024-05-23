import numpy as np
import matplotlib.pyplot as plt

class Draw:
    
    # CONSTRUCTOR
    # Parece ser que al usar matplotlib junto con cv2, primero se tiene que 
    # inicializar la figura y ejes de matplot antes de abrir la camara y 
    # desplegar imagenes con cv2. Si no, ocurre un crash. Por lo tanto se 
    # debe inicializar este objeto antes de abrir camara. 
    def __init__(self):
        # DRAW
        l = 500
        # ORIGINAL POSITION IN SPACE
        fig = plt.figure()
        self.ax = plt.axes(projection='3d')
        self.ax.set_xlim([-l, l])
        self.ax.set_ylim([-l, l])
        self.ax.set_zlim([-l, l])
        self.ax.set_xlabel('X')  
        self.ax.set_ylabel('Y')  
        self.ax.set_zlabel('Z')
        self.ax.view_init(30,45)
        
        # POSICION DE ORIGEN DE MUNDO
        self.Pw = np.array([[0, 0, 0, 1]]).T
        # FRAME DE MUNDO
        self.Fw = np.array([[-62, 0, 0], [0, 62, 0], [0, 0, 62]])
    
    def cal(self, T, R):
        # MATRIZ HOMOGENEA
        H = np.concatenate((np.concatenate((R , T), axis=1),[[0,0,0,1]]), axis=0)
        # PERFORM ROTATION
        self.Fw_r = R @ self.Fw
        # TRASLACION CON MATRIZ HOMOGENEA
        self.Pw_n = H @ self.Pw
        #VOLTEAR EJE Z DEL REFERENCIAL DE CAMARA (CONVENCION)
        # self.Fw_r[2] = -1 * self.Fw_r[2] 
    
    def draw_ax(self, pos, mag):
        self.ax.quiver(pos[0], pos[1], pos[2], mag[0][0], mag[1][0], mag[2][0], color='b') # X
        self.ax.quiver(pos[0], pos[1], pos[2], mag[0][1], mag[1][1], mag[2][1], color='r') # Y
        self.ax.quiver(pos[0], pos[1], pos[2], mag[0][2], mag[1][2], mag[2][2], color='g') # Z
    
    def draw_ve(self, pos, mag):
        self.ax.quiver(pos[0], pos[1], pos[2], mag[0], mag[1], mag[2], color='k')
    
    def draw_space(self):
        self.draw_ax(self.Pw, self.Fw)  
        self.draw_ve(self.Pw, self.Pw_n)
        self.draw_ax(self.Pw_n, self.Fw_r)  

    def draw_point(self, x, y, z, c):
        self.ax.plot(x, y, z, markerfacecolor=c, markeredgecolor=c, marker='o', markersize=5, alpha=0.5)

    def show_plot(self):
        plt.show()





