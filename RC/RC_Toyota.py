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
  panda.can_send(addr, dat, 0)

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
    accRate = 3000  #-20k to 20k
  elif goBrake:
    accRate = -4000
  else:
    accRate = 0

  values = {
    "ACCEL_CMD": hex(accRate),
    "SET_ME_X63": 0x63,
    "SET_ME_1": 0x1,
    "RELEASE_STANDSTILL": 0x0, #0,1
    "CANCEL_REQ": 0x1, #0,1
  }
  values = [hex(accRate), 0x63, 0x1, 0x1, 0x0]
  #[accRate >> 8, accRate && 0xff, 0x63, 0x1, 0x0, 0x1]

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

cnter = 0
def sendSteer2():
  global goRight, goLeft, cnter
  cnter+=1
  threading.Timer(0.01, sendSteer2).start()
  if goLeft:
    steer = 32700 - 15000
  elif goRight:
    steer = 32700 + 15000
  else:
    steer = 32700

  values = {
    "STEER_REQUEST": bin(0x1), #0,1
    "STEER_TORQUE_CMD": bin(steer),
    "COUNTER": bin(cnter),
    "SET_ME_1": bin(0x1),
  }
  values = '\x01\x3E8\x01\x01'
  send(0x2E4, values) #0x2E4 (STEERING_LKA), 0x266 (STEERING_IPAS), 0x167 (STEERING_IPAS_COMMA)

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
  start = [0x14, 00, 00, 00]  # 0x04 if too slow, 0x14 if LKAS enabled.
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
  send("STEERING_LKA", str(bytearray(dat)))

MPH = 0
def updateMPH(dat):
  global MPH
  #MPH = int(dat,16) * KPH_to_MPH
  MPH = dat
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
          print 'left'
          goLeft = True
          goRight = False
        elif c == 'd':
          print 'right'
          goRight = True
          goLeft = False
        elif c == 'w':
          print 'accel'
          goAccel = True
          goBrake = False
        elif c == 's':
          print 'brake'
          goBrake = True
          goAccel = False
        elif c == '\x1b': # x1b = ESC
          return
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

panda = None
def main():
  global panda
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  panda.can_clear(0)

  # while 1:
  #   vals = [0x3E8, 0x63, 0x1, 0x0, 0x1]
  #   valstr = '\x3E8\x63\x01\x00\x01'
  #   send(0x343, valstr)

  thread = Thread(target = getKeys)
  thread.start()

  #sendGear()
  sendSteer2()
  sendAccel()
  #sendBrake()

  # while True:
  #   can_recv = panda.can_recv()
  #   for address, _, dat, src in can_recv:
  #     if src == 0 and address == 0xB4:
  #       updateMPH(dat) #kph

if __name__ == "__main__":
  main()
