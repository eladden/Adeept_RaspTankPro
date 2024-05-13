#!usr/bin/python3

from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)
def mpu6050test():
  x = 0
  y = 0
  z = 0
  for i in range(0,10):
    gyro_data = sensor.get_gyro_data()
    x = x + gyro_data['x']
    y = y + gyro_data['y']
    z = z + gyro_data['z']
  print('X=%.3f, Y=%.3f, Z=%.3f'%(x/10.0,y/10.0,z/10.0))
  time.sleep(0.3)

if __name__ == "__main__":
  try:
    while True:
      mpu6050test()
  except:
    pass
