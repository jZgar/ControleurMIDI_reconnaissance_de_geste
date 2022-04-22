# -*- coding: utf-8 -*-
"""controllerMIDI_CNN.py"""

from google.colab import drive
drive.mount('/content/drive')

import tensorflow as tf
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import pathlib
import sys
import numpy as np
import random

## On génère les datas et les targets
def matGen(n):
  m = np.zeros((n, n))
  for i in range(n):
    for j in range(n):
      m[i, j] = random.randint(0, 254)
  return m

def dataGen(n, m, nb_sorties):
  data = []
  targets = []
  for i in range(n):
    t = []
    data.append(matGen(m))
    for j in range(nb_sorties):
      t.append(random.randint(0, 254)/254)
    targets.append(t)
  return [data, targets]

dataset = dataGen(20000, 16, 8)

data = dataset[0]
targets = dataset[1]

images, images_valid, targets, targets_valid = train_test_split(data, targets, test_size=0.2) # cinde images et targets en 2 vecteur chacun, images_valid correspondant à 20% de images initiale.

print(np.shape(images))
print(np.shape(targets))

images = np.reshape(images, (16000, 1, 16, 16, 1))
images_valid = np.reshape(images_valid, (4000, 1, 16, 16, 1))
targets = np.reshape(targets, (16000, 1, 8))
targets_valid = np.reshape(targets_valid, (4000, 1, 8))
print(np.shape(images))
print(np.shape(images_valid))
print(np.shape(targets))

train_dataset = tf.data.Dataset.from_tensor_slices(images) # on "officialise" la dataset de manière à s'y déplacer plus
valid_dataset = tf.data.Dataset.from_tensor_slices(images_valid) # facilement

train_dataset = tf.data.Dataset.from_tensor_slices((images, targets)) # on "officialise" la dataset de manière à s'y déplacer plus
valid_dataset = tf.data.Dataset.from_tensor_slices((images_valid, targets_valid)) # facilement

epoch = 1
batch_size = 32;
for images_batch, targets_batch in train_dataset.repeat(epoch).batch(batch_size):
    print(images_batch.shape, targets_batch.shape)
    break

"""un batch est un sous-ensemble de notr edataset puisque l'ordinateur n'est pas capable de les traiter toutes simultanément.
Quand on a parcouru l'ensemble de la dataset par batch, on a fait une epoch.
tupple : ensemble image + target associée
"""

epoch = 1
batch_size = 32;

num_param = 8

model = tf.keras.Sequential([
    layers.experimental.preprocessing.Rescaling(1./255),
    layers.Conv2D(32,4, padding='same',activation='relu'),
    layers.Conv2D(64,3, padding='same', activation='relu'),
    layers.Conv2D(128,3, padding='same', activation='relu'),
    layers.Flatten(),
    layers.Dense(128,activation='relu'),
    layers.Dense(num_param, activation='Softmax')
])

model.compile(optimizer='adam',
              loss=tf.losses.CategoricalCrossentropy(from_logits=True),
  metrics=['accuracy'],)

logdir="logs"

tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=logdir,histogram_freq=1, write_images=logdir,
                                                   embeddings_data=train_dataset)

model.fit(train_dataset, validation_data=valid_dataset, epochs=2, callbacks=[tensorboard_callback])

model.summary()

'''sys.path.insert(0, '/content/drive/MyDrive/Colab Notebooks')
from uart_read import *
s = serial.Serial('COM5', baudrate=115200)
file_to_predict = np.reshape(read_1_matrix(s), (1, 1, 16, 16, 1))
prediction = model.predict(file_to_predict)'''

#save keras model
model.save("/content/drive/MyDrive/Colab Notebooks/controllerMIDI.h5")