import usb.core, usb.util
import numpy as np
import matplotlib.pyplot as plt

dev = usb.core.find(idVendor=0x26ac, idProduct=0x0015)
endpoint = dev[0][(2,0)][0]

SIZE = 64
image = np.zeros((SIZE, SIZE), dtype='uint8')
imshow = plt.imshow(image, cmap=plt.cm.gray)
imshow.norm.vmin = 0
imshow.norm.vmax = 255
plt.ion()
plt.show()

while True:
    data = endpoint.read(SIZE*SIZE)
    image = np.frombuffer(data, dtype='uint8').reshape(SIZE, SIZE)
    imshow.set_data(image)
    plt.draw()
