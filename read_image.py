import usb.core, usb.util
import numpy as np
import matplotlib.pyplot as plt

dev = usb.core.find(idVendor=0x26ac, idProduct=0x0015)
endpoint = dev[0][(2,0)][0]

SIZE = 320
image = np.zeros((SIZE, SIZE), dtype='uint8')
imshow = plt.imshow(image, cmap=plt.cm.gray, vmin=0, vmax=255)
plt.ion()
plt.show()

while True:
    data = endpoint.read(64 * (1 + (SIZE*SIZE) / 64), timeout=20000)
    image = np.frombuffer(data, dtype='uint8').reshape(SIZE, SIZE)
    imshow.set_data(image)
    plt.pause(1e-9)
