import serial
import numpy as np
import matplotlib.pyplot as plt


def read_1_matrix(s) :

    """
    Fonction qui scrute l'entrée série jusqu'à ce que la valeur 0xff apparraisse,
    puis lis 256 octets et les renvois sous la forme d'une matrice 16x16
    """
    res = b'\x00'


    while res!=b'\xff':
        res = s.read()
        if res == b'\xff':

            read = s.read(256)

            list = [k for k in read]
            mat = (np.reshape(list, (16, 16)))

    return mat

def calibrate(s,n=10):
    L=[]
    Mat_cal=np.zeros((16,16))

    for k in range(n):
        L.append(read_1_matrix(s))

    for i in range(16):
        for j in range(16):
            Mat_cal[i,j] = np.mean([k[i,j] for k in L])
    return np.fliplr(Mat_cal)


def getmax(s):
    return 80

def boucle():
    cpt = 0
    s = serial.Serial('/dev/ttyACM0', baudrate=115200)

    mat_cal = calibrate(s)

    mat = read_1_matrix(s)
    im = plt.imshow(mat)

    max = getmax(s)
    coef= 255/max

    while True :
        cpt+=1
        mat_brut = np.fliplr(read_1_matrix(s)) #matrice à récupérer pour l'IA
        mat = coef*(mat_brut-mat_cal)
        
        if cpt%3 == 0:
            
            im.set_data(mat)
            plt.draw()
            plt.pause(0.01)
            cpt = 0

'''boucle()'''


# Configure the serial reading



# pour tester

#cal=print (calibrate(s))
