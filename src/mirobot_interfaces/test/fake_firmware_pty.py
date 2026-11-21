#!/usr/bin/env python3
"""Fake Mirobot firmware on a pty (socat) for the G-code writer: telemetry frames at 2.5 Hz.

--start Hold|Alarm|Idle   state at start
--home ok|ignore          what $H does: ok = Home for 2 s, then Idle; ignore = nothing (as in Hold)
--reset-after S           after S seconds go to Alarm (someone pressed reset), 0 = never
"""

import argparse
import os
import time

ap = argparse.ArgumentParser()
ap.add_argument('pty')
ap.add_argument('--start', default='Alarm')
ap.add_argument('--home', default='ok')
ap.add_argument('--reset-after', type=float, default=0.0)
a = ap.parse_args()

fd = os.open(a.pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
state, t0, home_until, buf = a.start, time.monotonic(), 0.0, b''
J = '1.000,2.000,3.000,0.000,10.000,20.000,30.000' if a.start == 'Idle' else '0.000,0.000,0.000,0.000,0.000,0.000,0.000'
while True:
    try:
        buf += os.read(fd, 4096)
    except BlockingIOError:
        pass
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        s = line.strip().decode(errors='replace')
        print('RX', repr(s), flush=True)
        if s == '$H' and a.home == 'ok':
            state, home_until = 'Home', time.monotonic() + 2.0
        if s:
            os.write(fd, b'ok\r\n')
    if home_until and time.monotonic() > home_until:
        state, home_until, J = 'Idle', 0.0, '1.000,2.000,3.000,0.000,10.000,20.000,30.000'
    if a.reset_after and time.monotonic() - t0 > a.reset_after and state == 'Hold':
        state = 'Alarm'
        print('RESET -> Alarm', flush=True)
    frame = (
        f'<{state},Angle(ABCDXYZ):{J},Cartesian coordinate(XYZ RxRyRz):198.670,0.000,230.720,0.000,0.000,0.000,'
        'Pump PWM:0,Valve PWM:0,Motion_MODE:0>\r\n'
    )
    os.write(fd, frame.encode())
    time.sleep(0.4)
