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

# MAKE_CAN_MSG (REF'D END OF CREATE_STEER/ACCEL)
# #Combines a msg and addr into one data structure
# def fix(msg, addr):
#   checksum = 0
#   idh = (addr & 0xff00) >> 8
#   idl = (addr & 0xff)

#   checksum = idh + idl + len(msg) + 1
#   for d_byte in msg:
#     checksum += ord(d_byte)
#   #return msg + chr(checksum & 0xFF)
#   return msg + struct.pack("B", checksum & 0xFF)

# def make_can_msg(addr, dat, alt, cks=False):
#   if cks:
#     dat = fix(dat, addr)
#   return [addr, 0, dat, alt]


# CAN_LIST_TO_CANCAPNP (REF'D END OF MAIN)
def new_message():
  dat = log.Event.new_message()
  dat.logMonoTime = int(realtime.sec_since_boot() * 1e9)
  return dat

def can_list_to_can_capnp(can_msgs, msgtype='can'): #from boardd>boardd.py
  dat = new_message()
  dat.init(msgtype, len(can_msgs))
  for i, can_msg in enumerate(can_msgs):
    if msgtype == 'sendcan':
      cc = dat.sendcan[i]
    else:
      cc = dat.can[i]
    print("Addr"+can_msg[0])
    print("busTime"+str(can_msg[1]))
    print("dat"+str(can_msg[2]))
    print("src"+str(can_msg[3]))
    cc.address = "hi" #can_msg[0]
    cc.busTime = can_msg[1]
    cc.dat = str(can_msg[2])
    cc.src = can_msg[3]
  return dat

#ETHAN: A general way to send steering commands to CAN. I need to use this if I want to steer.
def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)

#ETHAN: A general way to send acceleration/(hopefully deceleration) commands to CAN. I need to use this for driving.
def create_accel_command(packer, accel, pcm_cancel, standstill_req):
  values = {
    "ACCEL_CMD": accel,
    "SET_ME_X63": 0x63,
    "SET_ME_1": 1,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)

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
pker = None
def main():
  # global panda
  # panda = Panda()
  # panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  # panda.can_clear(0)

  pker = CANPacker ("toyota_prius_2017_pt_generated.dbc")

  thread = Thread(target = getKeys)
  thread.start()

  raw_cnt = 0;
  while 1:
    can_sends = []
    
    tq = getTorque()
    can_sends.append(create_steer_command(pker, 1, raw_cnt)) #packer, steer, steer_req, raw_cnt)
    raw_cnt += 1

    acc = getAccel()
    can_sends.append(create_accel_command(pker, True, True)) #packer, accel, pcm_cancel, standstill_req
    #standstill_req is a boolean that determines if the PCM will be asked to fubar stop and go without a Pedal. It tells the pcm the car has stopped and to toss up the nag message about tapping the pedal or cruise lever to continue.
    #print(can_sends)
    #panda.can_send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
    test = can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes()



if __name__ == "__main__":
  main()
