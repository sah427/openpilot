from numpy import interp

from cereal import car, messaging
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

from common.travis_checker import travis

splmoffsetmphBp = [30, 40, 55, 60, 70]
splmoffsetmphV = [0, 0, 0, 0, 0]

splmoffsetkphBp = [30, 40, 55, 70]
splmoffsetkphV = [0, 0, 0, 0]

VisualAlert = car.CarControl.HUDControl.VisualAlert


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert == VisualAlert.steerRequired)

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    if enabled or sys_warning:
      sys_state = 3
    else:
      sys_state = 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.last_resume_frame = 0

    self.sm = 0
    self.smartspeed = 0
    self.setspeed = 0
    self.minsetspeed = 20
    self.smartspeed_old = 0
    self.smartspeedupdate = False
    self.fixed_offset = 0
    self.button_stop = 0
    if not travis:
      self.sm = messaging.SubMaster(['liveMapData'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart):

    if self.car_fingerprint in FEATURES["send_lfa_mfa"]:
      self.lfa_available = True
    else:
      self.lfa_available = False
    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90.

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = False

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []
    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available))

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
    elif CS.out.cruiseState.standstill and CS.vrelative > 0:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL))

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.lfa_available:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    # Speed Limit Related Stuff  Lot's of comments for others to understand!
    # Run this twice a second

    self.smartspeed_old = self.smartspeed
    if not travis:
      self.sm.update(0)
      self.smartspeed = self.sm['liveMapData'].speedLimitAhead
      if CS.is_set_speed_in_mph:
        self.smartspeed = self.sm['liveMapData'].speedLimit * CV.MS_TO_MPH
        self.fixed_offset = interp(self.smartspeed, splmoffsetmphBp, splmoffsetmphV)
        self.smartspeed = self.smartspeed + int(self.fixed_offset)
        self.setspeed = CS.cruisesetspeed * CV.MS_TO_MPH
        self.minsetspeed = 20 * CV.MPH_TO_MS * CV.MS_TO_MPH
        self.currentspeed = int(CS.out.vEgo * CV.MS_TO_MPH)
      else:
        self.smartspeed = self.sm['liveMapData'].speedLimit * CV.MS_TO_KPH
        self.fixed_offset = interp(self.smartspeed, splmoffsetkphBp, splmoffsetkphV)
        self.smartspeed = self.smartspeed + int(self.fixed_offset)
        self.setspeed = CS.cruisesetspeed * CV.MS_TO_KPH
        self.minsetspeed = 20 * CV.MPH_TO_MS * CV.MS_TO_KPH
        self.currentspeed = int(CS.out.vEgo * CV.MS_TO_KPH)

    if self.smartspeed_old != self.smartspeed:
      self.smartspeedupdate = True

    if enabled and CS.rawcruiseStateenabled and self.smartspeedupdate and (CS.cruise_buttons != 4)\
            and (self.minsetspeed <= self.smartspeed):
        if self.setspeed > (self.smartspeed * 1.005):
          can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.SET_DECEL, self.currentspeed))
          if CS.cruise_buttons == 1:
             self.button_stop +=1
          else:
             self.button_stop = 0
        elif self.setspeed < (self.smartspeed / 1.005):
          can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.RES_ACCEL, self.currentspeed))
          if CS.cruise_buttons == 2:
             self.button_stop +=1
          else:
             self.button_stop = 0
        else:
          self.button_stop = 0

        if abs(self.smartspeed - self.setspeed) < 0.5 or self.button_stop > 10:
          self.smartspeedupdate = False

    return can_sends
