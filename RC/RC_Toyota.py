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

def send(addr, dat):
  global panda
  panda.can_send(addr, dat, 2)

def sendGear(): #0 "P" 1 "R" 2 "N" 3 "D" 4 "B"
  global goAccel, goBrake, goRight, goLeft
  gr = 0 #P
  if goAccel or goBrake or goRight or goLeft:
    gr = 3
  send(0x127, str(gr))

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


def sendBrake():
  global goBrake
  threading.Timer(0.01, sendBrake).start()
  brakeval = 0
  if goBrake:
    brakeval = 125

  values = {
    "BRAKE_PRESSURE": brakeval,
    "BRAKE_POSITION": brakeval,
    "BRAKE_PRESSED": 0x1, #0,1
  }
  send (0x226, values) #0x226 (brake_module), 0xA6 (brake)

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

def calc_checksum(data, length):
  # from http://illmatics.com/Remote%20Car%20Hacking.pdf
  end_index = length - 1
  index = 0
  checksum = 0xFF
  temp_chk = 0;
  bit_sum = 0;
  if(end_index <= index):
    return False
  for index in range(0, end_index):
    shift = 0x80
    curr = data[index]
    iterate = 8
    while(iterate > 0):
      iterate -= 1
      bit_sum = curr & shift;
      temp_chk = checksum & 0x80
      if (bit_sum != 0):
        bit_sum = 0x1C
        if (temp_chk != 0):
          bit_sum = 1
        checksum = checksum << 1
        temp_chk = checksum | 1
        bit_sum ^= temp_chk
      else:
        if (temp_chk != 0):
          bit_sum = 0x1D
        checksum = checksum << 1
        bit_sum ^= checksum
      checksum = bit_sum
      shift = shift >> 1
  return ~checksum & 0xFF

counter = 0
torque = 0
def sendSteer():  # every 0.01 seconds
  global counter, torque
  global goLeft, goRight
  threading.Timer(0.01, sendSteer).start()
  #start = [0x14, 00, 00, 00]  # 0x04 if too slow, 0x14 if LKAS enabled.
  start = [1, 0, counter, 1, 00]
  # The 0x04 makes up the high nibble of 1024 (0x04 0x00) for a straight-steer.
  if goLeft or goRight:
    print torque
    combined_torque = torque + 1024  # 1024 is straight. more is left, less is right.
    start = [0x10 | (combined_torque >> 8), combined_torque & 0xff, 00, 00]
    if goLeft:
      torque += 1  # has increasing torque
    elif goRight:
      torque -= 1  # right is negative
  if torque >= 256 or torque <= -256:  # stop torque.
    goRight = False
    goLeft = False
    torque = 0
  dat = start + [counter]
  dat = dat + [calc_checksum(dat, len(dat)+1)]  # checksum len include checksum itself
  counter = (counter + 0x10) % 0x100
  send(0x2E4, str(bytearray(dat)))

MPH = 0
def updateMPH(dat):
  global MPH
  s1 = hex(dat[5])[2:]
  s2 = hex(dat[6])[2:]
  sp = int(s1+s2,16)
  MPH = sp/100 * KPH_to_MPH
  print(MPH)
  

goLeft = False
goRight = False
goAccel = False
goBrake = False

#GETKEYS() - REF'D IN MAIN() - Constantly update keyinputs
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

def checksum(arr):
  IDH = 0x0 #0x03
  IDL = 0xB4 #0x43
  Len = len(arr)+1
  Sum = 0
  for msg in arr:
    Sum += msg
  return hex((IDH+IDL+Len+Sum) & 0xFF)
  #return hex(IDH+IDL+Len+Sum)

def goAcc():
  threading.Timer(0.1, goAcc).start()
  values = [0x02,0x58,0x63,0xc0,0x00,0x00,0x00,0xcb]
  #print(bytearray(values))
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
  b5 = checksum(arr)
  arr = arr + [int(b5,16)]
  #print(arr)
  send(0x2E4, str(bytearray(arr)))

def HighBeam():
  threading.Timer(0.1, HighBeam).start()
  vals = [0x29,0,0x60,0x20,00,00,0x07,0xc5]
  send(0x614, str(bytearray(vals)))

spdcnt = 0
def ChangeSpeed():
  global spdcnt
  if spdcnt>254: spdcnt=-1
  spdcnt+=1

  #threading.Timer(0.1, ChangeSpeed).start()
  vals = [0,0,0,0,spdcnt,06,66]
  vals = vals + [int(checksum(vals),16)]
  send(0xB4, str(bytearray(vals)))


panda = None
def main():
  global panda
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  panda.can_clear(0)

  thread = Thread(target = getKeys)
  thread.start()

  ChangeSpeed()

  HighBeam()
  goStr()
  goAcc()

  

  #sendSteer()
  #sendAccel()
  #sendBrake()

  while True:
    can_recv = panda.can_recv()
    for address, _, dat, src in can_recv:
      if src == 0 and address == 0xB4:
        updateMPH(dat) #kph


if __name__ == "__main__":
  main()
