# -*- coding: utf-8 -*-
from cf.hal.dxlpro import SerialDevice
import time

s = SerialDevice('/dev/ttyArm', baudrate=3000000)

print("scan--------------------------------------")
print (s.scan())
print("ping--------------------------------------")
for i in range (1,6):
  print(s.ping(i))


print("get present position----------------------")
print(s.get_present_position([1,2,3,4,5]))
