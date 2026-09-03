#!/usr/bin/env python3
import subprocess
import time
import threading
from cereal import car, messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.carrot.speed_camera import apn_speed_camera_active, vehicle_speed_camera_active

AudibleAlert = car.CarControl.HUDControl.AudibleAlert

class Beepd:
  def __init__(self):
    self.params = Params()
    self.current_alert = AudibleAlert.none
    self.speed_camera_zone_active = False
    self.last_speed_camera_sound_time = 0.0
    self.enable_gpio()
    self.startup_beep()

  def enable_gpio(self):
    # 尝试 export，忽略已 export 的错误
    try:
      subprocess.run("echo 42 | sudo tee /sys/class/gpio/export",
                     shell=True,
                     stderr=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL,
                     encoding='utf8')
    except Exception:
      pass
    subprocess.run("echo \"out\" | sudo tee /sys/class/gpio/gpio42/direction",
                   shell=True,
                   stderr=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL,
                   encoding='utf8')

  def _beep(self, on, force=False):
    if not force and self.params.get_int("SoundVolumeAdjust") <= 5:
      on = False
    val = "1" if on else "0"
    subprocess.run(f"echo \"{val}\" | sudo tee /sys/class/gpio/gpio42/value",
                   shell=True,
                   stderr=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL,
                   encoding='utf8')

  def engage(self):
    self._beep(True)
    time.sleep(0.05)
    self._beep(False)

  def disengage(self):
    for _ in range(2):
      self._beep(True)
      time.sleep(0.01)
      self._beep(False)
      time.sleep(0.01)

  def warning(self):
    for _ in range(3):
      self._beep(True)
      time.sleep(0.01)
      self._beep(False)
      time.sleep(0.01)

  def startup_beep(self):
    self._beep(True)
    time.sleep(0.1)
    self._beep(False)

  def ding(self):
    self._beep(True)
    time.sleep(0.02)
    self._beep(False)

  def dong(self):
    self._beep(True)
    time.sleep(0.03)
    self._beep(False)

  def beep(self):
    self._beep(True)
    time.sleep(0.04)
    self._beep(False)

  def speed_camera(self):
    # Two short pulses mark the approach to a speed-camera zone.
    for index in range(2):
      self._beep(True, force=True)
      time.sleep(0.06)
      self._beep(False, force=True)
      if index < 1:
        time.sleep(0.04)

  def speed_camera_end(self):
    # One short pulse marks the end of a speed-camera zone.
    self._beep(True, force=True)
    time.sleep(0.06)
    self._beep(False, force=True)

  def dispatch_beep(self, func):
    threading.Thread(target=func, daemon=True).start()

  def dispatch_speed_camera_sound(self, entering):
    self.last_speed_camera_sound_time = time.monotonic()
    self.dispatch_beep(self.speed_camera if entering else self.speed_camera_end)

  def speed_camera_beep_enabled(self):
    try:
      return self.params.get_int("SpeedCameraBeep") > 0
    except UnknownKeyName:
      return True

  def update_speed_camera(self, sm):
    if not sm.updated['carrotMan'] and not sm.updated['carState']:
      return

    apn_camera_active = False
    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      apn_camera_active = apn_speed_camera_active(carrot_man.activeCarrot, carrot_man.xSpdType,
                                                  carrot_man.xSpdLimit, carrot_man.xSpdDist)

    vehicle_camera_active = False
    if sm.alive['carState']:
      car_state = sm['carState']
      vehicle_camera_active = vehicle_speed_camera_active(car_state.speedLimit, car_state.speedLimitDistance)

    camera_active = apn_camera_active or vehicle_camera_active
    if camera_active != self.speed_camera_zone_active:
      self.speed_camera_zone_active = camera_active
      if self.speed_camera_beep_enabled():
        self.dispatch_speed_camera_sound(camera_active)

  def update_alert(self, new_alert):
    if new_alert != self.current_alert:
      self.current_alert = new_alert
      print(f"[BEEP] New alert: {new_alert}")
      if new_alert == AudibleAlert.engage:
        self.dispatch_beep(self.engage)
      elif new_alert == AudibleAlert.disengage:
        self.dispatch_beep(self.disengage)
      elif new_alert in [AudibleAlert.refuse, AudibleAlert.prompt, AudibleAlert.warningImmediate,AudibleAlert.warningSoft]:
        self.dispatch_beep(self.warning)
      elif new_alert in [AudibleAlert.longEngaged, AudibleAlert.longDisengaged, AudibleAlert.trafficSignGreen, AudibleAlert.trafficSignChanged, AudibleAlert.trafficError, AudibleAlert.bsdWarning, AudibleAlert.laneChange]:
        self.dispatch_beep(self.ding)
      elif new_alert in [AudibleAlert.stopStop, AudibleAlert.stopping, AudibleAlert.autoHold, AudibleAlert.engage2, AudibleAlert.disengage2, AudibleAlert.speedDown, AudibleAlert.audioTurn, AudibleAlert.reverseGear]:
        self.dispatch_beep(self.dong)
      elif new_alert in [AudibleAlert.audio1, AudibleAlert.audio2, AudibleAlert.audio3, AudibleAlert.audio4, AudibleAlert.audio5,
                         AudibleAlert.audio6, AudibleAlert.audio7, AudibleAlert.audio8, AudibleAlert.audio9, AudibleAlert.audio10]:
        if new_alert == AudibleAlert.audio1 and self.speed_camera_beep_enabled():
          if time.monotonic() - self.last_speed_camera_sound_time > 1.0:
            self.dispatch_speed_camera_sound(True)
        elif new_alert == AudibleAlert.audio2 and self.speed_camera_beep_enabled():
          if time.monotonic() - self.last_speed_camera_sound_time > 1.0:
            self.dispatch_speed_camera_sound(False)
        else:
          self.dispatch_beep(self.beep)

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)

  def test_beepd_thread(self):
    frame = 0
    rk = Ratekeeper(20)
    pm = messaging.PubMaster(['selfdriveState'])
    while True:
      cs = messaging.new_message('selfdriveState')
      if frame == 40:
        cs.selfdriveState.alertSound = AudibleAlert.engage
      if frame == 60:
        cs.selfdriveState.alertSound = AudibleAlert.disengage
      if frame == 80:
        cs.selfdriveState.alertSound = AudibleAlert.prompt

      pm.send("selfdriveState", cs)
      frame += 1
      rk.keep_time()

  def beepd_thread(self, test=False):
    if test:
      threading.Thread(target=self.test_beepd_thread, daemon=True).start()

    sm = messaging.SubMaster(['selfdriveState', 'carrotMan', 'carState'])
    rk = Ratekeeper(20)

    while True:
      sm.update(0)
      self.update_speed_camera(sm)
      self.get_audible_alert(sm)
      rk.keep_time()

def main():
  s = Beepd()
  s.beepd_thread(test=False)  # 改成 True 可启用模拟测试数据

if __name__ == "__main__":
  main()
