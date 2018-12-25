#Remote-Controlled Car Script
#Integrated for Toyota Prius Prime 2018
#--Ethan Shaotran--

import sys
from panda import Panda
import threading
from threading import Thread
import termios
import tty
import select
from hexdump import hexdump

KPH_to_MPH = 0.621371

class CAR:
  PRIUS = "TOYOTA PRIUS 2017"
  RAV4H = "TOYOTA RAV4 HYBRID 2017"
  RAV4 = "TOYOTA RAV4 2017"
  COROLLA = "TOYOTA COROLLA 2017"
  LEXUS_RXH = "LEXUS RX HYBRID 2017"
  CHR = "TOYOTA C-HR 2018"
  CHRH = "TOYOTA C-HR HYBRID 2018"
  CAMRY = "TOYOTA CAMRY 2018"
  CAMRYH = "TOYOTA CAMRY HYBRID 2018"
  HIGHLANDER = "TOYOTA HIGHLANDER 2017"
  HIGHLANDERH = "TOYOTA HIGHLANDER HYBRID 2018"

car_fingerprint = "TOYOTA PRIUS 2017"

class ECU:
  CAM = 0 # camera
  DSU = 1 # driving support unit
  APGS = 2 # advanced parking guidance system

enable_camera = False
enable_dsu = True
enable_apg = False

fake_ecus = set()
if enable_camera: fake_ecus.add(ECU.CAM)
if enable_dsu: fake_ecus.add(ECU.DSU)
if enable_apg: fake_ecus.add(ECU.APGS)

# addr: (ecu, cars, bus, 1/freq*100, vl)
STATIC_MSGS = [
  (0x130, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1, 100, '\x00\x00\x00\x00\x00\x00\x38'),
  (0x240, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
  (0x241, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
  (0x244, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
  (0x245, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
  (0x248, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   5, '\x00\x00\x00\x00\x00\x00\x01'),
  (0x367, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0,  40, '\x06\x00'),
  (0x414, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0, 100, '\x00\x00\x00\x00\x00\x00\x17\x00'),
  (0x466, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1, 100, '\x20\x20\xAD'),
  (0x466, ECU.CAM, (CAR.COROLLA), 1, 100, '\x24\x20\xB1'),
  (0x489, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0, 100, '\x00\x00\x00\x00\x00\x00\x00'),
  (0x48a, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0, 100, '\x00\x00\x00\x00\x00\x00\x00'),
  (0x48b, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0, 100, '\x66\x06\x08\x0a\x02\x00\x00\x00'),
  (0x4d3, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x1C\x00\x00\x01\x00\x00\x00\x00'),

  (0x128, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   3, '\xf4\x01\x90\x83\x00\x37'),
  (0x128, ECU.DSU, (CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   3, '\x03\x00\x20\x00\x00\x52'),
  (0x141, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   2, '\x00\x00\x00\x46'),
  (0x160, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,   7, '\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   7, '\x00\x1e\x00\x00\x00\x80\x07'),
  (0X161, ECU.DSU, (CAR.HIGHLANDERH, CAR.HIGHLANDER), 1,  7, '\x00\x1e\x00\xd4\x00\x00\x5b'),
  (0x283, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0,   3, '\x00\x00\x00\x00\x00\x00\x8c'),
  (0x2E6, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, '\xff\xf8\x00\x08\x7f\xe0\x00\x4e'),
  (0x2E7, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, '\xa8\x9c\x31\x9c\x00\x00\x00\x02'),
  (0x33E, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,  20, '\x0f\xff\x26\x40\x00\x1f\x00'),
  (0x344, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH), 0,   5, '\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.HIGHLANDERH), 0,  20, '\x00\x00\x00\x80\x03\x00\x08'),
  (0x365, ECU.DSU, (CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER), 0,  20, '\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.HIGHLANDERH), 0,  20, '\x00\x00\x4d\x82\x40\x02\x00'),
  (0x366, ECU.DSU, (CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER), 0,  20, '\x00\x72\x07\xff\x09\xfe\x00'),
  (0x470, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 1, 100, '\x00\x00\x02\x7a'),
  (0x470, ECU.DSU, (CAR.HIGHLANDER, CAR.HIGHLANDERH), 1,  100, '\x00\x00\x01\x79'),
  (0x4CB, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDERH, CAR.HIGHLANDER), 0, 100, '\x0c\x00\x00\x00\x00\x00\x00\x00'),

  (0x292, ECU.APGS, (CAR.PRIUS), 0,   3, '\x00\x00\x00\x00\x00\x00\x00\x9e'),
  (0x32E, ECU.APGS, (CAR.PRIUS), 0,  20, '\x00\x00\x00\x00\x00\x00\x00\x00'),
  (0x396, ECU.APGS, (CAR.PRIUS), 0, 100, '\xBD\x00\x00\x00\x60\x0F\x02\x00'),
  (0x43A, ECU.APGS, (CAR.PRIUS), 0, 100, '\x84\x00\x00\x00\x00\x00\x00\x00'),
  (0x43B, ECU.APGS, (CAR.PRIUS), 0, 100, '\x00\x00\x00\x00\x00\x00\x00\x00'),
  (0x497, ECU.APGS, (CAR.PRIUS), 0, 100, '\x00\x00\x00\x00\x00\x00\x00\x00'),
  (0x4CC, ECU.APGS, (CAR.PRIUS), 0, 100, '\x0D\x00\x00\x00\x00\x00\x00\x00'),
]

goLeft = False
goRight = False
goAccel = False
goBrake = False
MPH = 0
Torque = 0
Frame = 0

def isData():
  return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getKeys():
  global goLeft, goRight, goAccel, goBrake
  old_settings = termios.tcgetattr(sys.stdin)
  try:
    tty.setcbreak(sys.stdin.fileno())

    while 1:
      if isData():
        c = sys.stdin.read(1)
        if c == 'a':
          #print 'left'
          goLeft = True
          goRight = False
        elif c == 'd':
          #print 'right'
          goRight = True
          goLeft = False
        elif c == 'w':
          #print 'accel'
          goAccel = True
          goBrake = False
        elif c == 's':
          #print 'brake'
          goBrake = True
          goAccel = False
        elif c == '\x1b': # x1b = ESC
          return
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def updateFrame():
  global Frame
  #threading.Timer(0.01, updateFrame).start()
  Frame += 1

def checksum(addr,arr):
  IDH = addr >> 8
  IDL = addr & 0xff
  Len = len(arr)+1
  Sum = 0
  for msg in arr:
    Sum += msg
  return hex((IDH+IDL+Len+Sum) & 0xFF)

def send(addr, dat):
  global panda
  panda.can_send(addr, dat, 0)

def send(addr, dat, bus):
  global panda
  panda.can_send(addr, dat, bus)




#READERS
def updateMPH(dat):
  global MPH
  s1 = hex(dat[5])[2:]
  s2 = hex(dat[6])[2:]
  sp = int(s1+s2,16)
  MPH = sp/100 * KPH_to_MPH

def updateTorque(dat):
  global Torque
  t1 = hex(dat[2])[2:]
  t2 = hex(dat[3])[2:]
  Torque = int(t1+t2,16)







#EDITORS
def sendStatics():
  global Frame
  threading.Timer(0.01, sendStatics).start()
  for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
    if Frame % fr_step == 0 and ecu in fake_ecus and car_fingerprint in cars:
      # special cases
      if fr_step == 5 and ecu == ECU.CAM and bus == 1:
        cnt = (((Frame / 5) % 7) + 1) << 5
        vl = chr(cnt) + vl
      elif addr in (0x489, 0x48a) and bus == 0:
        # add counter for those 2 messages (last 4 bits)
        cnt = ((Frame/100)%0xf) + 1
        if addr == 0x48a:
          # 0x48a has a 8 preceding the counter
          cnt += 1 << 7
        vl += chr(cnt)
      print(bus)
      send(addr,vl,bus)
  updateFrame()

def sendAccel():
  global goAccel, goBrake
  threading.Timer(0.01, sendAccel).start()
  if goAccel:
    accRate = 900  #-20k to 20k
  elif goBrake:
    accRate = -400
  else:
    accRate = 0
  values = [accRate >> 8, accRate & 0xff, 255, int('11000000',2), 0, 0, 0]
  values = values + [int(calc_checksum(values, len(values)+1))]
  print(bytearray(values))
  send(0x343, str(bytearray(values))) #m/s2, 0x343 (ACC_CONTROL), 0x245 (GAS_PEDAL), 0x200 (GAS_COMMAND)

str_cntr = 0
def sendSteer2():
  global goRight, goLeft, str_cntr
  str_cntr+=1
  if str_cntr == 64:
    str_cntr = 0
  threading.Timer(0.01, sendSteer2).start()
  if goLeft:
    steer = 0 - 200
    req = 1
  elif goRight:
    steer = 0 + 200
    req = 1
  else:
    steer = 0
    req = 0

  values = {
    "LKA_STATE": 0,
    "STEER_REQUEST": 0x1, #0,1
    "COUNTER": 0,
    "SET_ME_1": 1,
    "STEER_TORQUE_CMD": bin(steer),
    "CHECKSUM": bin(str_cntr),
  }
  values = [0, req, str_cntr, 1, steer >> 8, steer & 0xff, 0]
  values = values + [calc_checksum(values, len(values)+1)]
  send(0x2E4, str(bytearray(values))) #0x2E4 (STEERING_LKA), 0x266 (STEERING_IPAS), 0x167 (STEERING_IPAS_COMMA)

def goAcc():
  threading.Timer(0.1, goAcc).start()
  values = [0x05,0xa0,0x63,0xc0,0x00,0x00,0x00]
  values = values + [checksum(0x343,values)]
  send(0x343,str(bytearray(values)))

  #['\x01\x00`h\x08\x08\x00\x00\x02\x88c\xc0\x00\x00\x00M']
  #\IDH\IDL   h\bushex2,len\0,bushex1\00\00\msg\

cnt = 0
def goStr():
  global cnt
  threading.Timer(0.1, goStr).start()
  if cnt>62: cnt=-1
  cnt+=1
  mid = cnt
  lenmid = int(len(str(bin(mid))))-2
  midstr = str(bin(mid)[2:])
  while (lenmid<6): #"0b" takes 2 spaces
    midstr = '0'+midstr
    lenmid+=1
  b1 = '1' + str(midstr) + '1'
  b1 = int(b1,2)
  torque=64256
  b2 = torque >> 8
  b3 = torque & 0xFF
  b4 = 0x80
  arr = [b1,b2,b3,b4]
  b5 = checksum(0x2E4,arr)
  arr = arr + [int(b5,16)]
  print(arr)
  send(0x2E4, str(bytearray(arr)))

def HighBeam():
  threading.Timer(0.1, HighBeam).start()
  vals = [0x29,0,0x60,0x20,00,00,0x07,0xc5]
  send(0x614, str(bytearray(vals)))

spdcnt = -3
def ChangeSpeed():
  global spdcnt
  if spdcnt>252: spdcnt=-3
  spdcnt+=4

  threading.Timer(0.01, ChangeSpeed).start()
  vals = [0,0,0,0,spdcnt,06,66]
  vals = vals + [int(checksum(0xB4,vals),16)]
  print(vals)
  send(0xB4, str(bytearray(vals)))





panda = None
def main():
  global panda
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  panda.can_clear(0)

  sendStatics()
  #updateFrame()

  # thread = Thread(target = getKeys)
  # thread.start()

  #ChangeSpeed()
  #HighBeam()
  #goStr()
  #goAcc()

  #sendSteer()
  #sendAccel()
  #sendBrake()

  # while True:
  #   can_recv = panda.can_recv()
  #   for address, _, dat, src in can_recv:
  #     if src == 0 and address == 0xB4: #SPEED
  #       updateMPH(dat) #kph
  #     if src == 0 and address == 0x24: #KINEMATICS
  #       updateTorque(dat)


if __name__ == "__main__":
  main()
