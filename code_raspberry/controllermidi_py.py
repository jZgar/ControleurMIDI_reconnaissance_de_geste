# -*- coding: utf-8 -*-
"""controllerMIDI.py"""
print("Initialisation")
## Import lib
import tensorflow as tf
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split 
from sklearn.preprocessing import StandardScaler
import pandas as pd
import numpy as np
import os
import pathlib
import sys
import numpy as np
import random
import socket

## Port et IP 
UDP_IP = "192.168.43.221"
UDP_PORT = 5000

## import de uart_read
sys.path.insert(0, '/home/theophile/Documents/Projet/controllerMIDI')
from uart_read import *

## Téléchargement de l'IA entrainée
print("Awakening of the AI")
model = tf.keras.models.load_model('/home/theophile/Documents/Projet/controllerMIDI/controllerMIDI.h5')


def boucle():
    # connexion au port usb reliant à la STM32
    s = serial.Serial('/dev/ttyACM0', baudrate=115200)

    mat_cal = calibrate(s)

    mat = read_1_matrix(s)

    max = getmax(s)
    coef= 255/max
    
    MaxDeMat = np.max(np.max(mat))
    
    print('Ready')
    # prêt pour l'utilisation de l'interface
    while True :
        aff = 0
        mat_brut = np.fliplr(read_1_matrix(s)) #matrice à récupérer pour l'IA
        mat = coef*(mat_brut-mat_cal)
        
        
        for i in mat:
            for e in i:
                if e > 2*MaxDeMat:
                    aff+=1
                    
        
        
        if aff != 0:
            cpt = 0
            L = []
            while cpt < 10:
                # on récupère 10 matrices de données
                mat_brut = np.fliplr(read_1_matrix(s)) #matrice à récupérer pour l'IA
                mat = coef*(mat_brut-mat_cal)
                L.append(mat)
                cpt+=1
                
            # on utilise l'IA sur la 5e matrice récupérée qu'on met en forme pour le réseau CNN
            file_to_predict = np.reshape(L[5], (1, 16, 16, 1))
            prediction = model.predict(file_to_predict)
            print(prediction)
            
            MESSAGE = []
            prediction *= 100
            prediction_int = prediction.astype(int)
            print(prediction_int)
            # Mise en forme du message à envoyer à ableton
            for i in range(8):
                prediction_int[0][i] = format(prediction_int[0][i], 'o')
                MESSAGE.append(prediction_int[0][i])

            '''
            print("UDP target IP: %s" % UDP_IP)
            print("UDP target port: %s" % UDP_PORT)
            print(type(MESSAGE[0]))
            print("message: %s" % MESSAGE)'''
            # envoie du message en UDP
            for i in range(1):
                for j in range(8):
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
                    sock.sendto(MESSAGE[j], (UDP_IP, UDP_PORT))
                    print(f"{MESSAGE[j]} sent")
            print("Ready")
        aff = 0
            

boucle()

