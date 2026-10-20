#!/usr/bin/env python3
import os
import subprocess
import time

from openpilot.common.spinner import Spinner


def ui_is_running() -> bool:
  for pid in os.listdir("/proc"):
    if not pid.isdigit():
      continue
    try:
      if os.path.basename(os.readlink(f"/proc/{pid}/exe")) == "ui":
        return True
    except (FileNotFoundError, PermissionError, ProcessLookupError):
      continue
  return False


def main() -> int:
  spinner = Spinner()
  spinner.update("Starting")
  manager = subprocess.Popen(["./manager.py"])

  try:
    while manager.poll() is None and not ui_is_running():
      time.sleep(0.2)
  finally:
    spinner.close()

  return manager.wait()


if __name__ == "__main__":
  raise SystemExit(main())
