#Remote-Controlled Car Script
#Integrated for Toyota Prius Prime 2018
#--Ethan Shaotran--

import sys
from Measures import Conversions as CV
from panda import Panda
import threading
from threading import Thread
from can.packer import CANPacker
import termios
import tty
import select
import realtime
from cereal import log


#getTorque(), getAccel() - REF'D IN MAIN() - Constantly update
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

#TODO: Find if Toyota Prius Prime uses 1024 as center
counter = 0
torque = 0
def getTorque():  # every 0.01 seconds
  global counter, torque
  global goLeft, goRight
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
  return dat


def getAccel():
  global goAccel, goBrake
  if goAccel:
    accRate = 0.05
    # goBrake = False
    # mph = getMPH() #m/s
    # if mph<(10*0.44704): #10mph in M/S form
    #   accRate = 0.1
  elif goBrake:
    accRate = -0.1
  else:
    accRate = 0

  return accRate #m/s2


def getMPH():
  cp = interface.get_params()
  v_FL = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
  v_FR = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
  v_RL = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
  v_RR = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
  v_car = float(np.mean([v_FL, v_FR, v_RL, v_RR]))
  return v_car #m/s

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

  thread = Thread(target = getKeys)
  thread.start()

  while 1:    
    tq = getTorque()
    acc = getAccel()




if __name__ == "__main__":
  main()
