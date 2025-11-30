#!/usr/bin/env python3
"""
Keyboard Control for UGV Robot
"""
import sys
import tty
import termios
import subprocess
import signal

CONTAINER_NAME = "ugv_rpi_ros_humble"

msg = """
UGV Keyboard Control
---------------------------
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease speed
space, k : STOP
CTRL-C : quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': 1.1,
    'z': 0.9,
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send_cmd(linear_x, angular_z):
    cmd = f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}'"
    subprocess.Popen(
        ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', f'source /opt/ros/humble/setup.bash && {cmd}'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def main():
    settings = termios.tcgetattr(sys.stdin)

    def cleanup(sig=None, frame=None):
        send_cmd(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nStopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)

    speed = 0.2
    turn = 0.5

    print(msg)
    print(f"Speed: {speed:.2f}  Turn: {turn:.2f}")

    try:
        while True:
            key = get_key(settings)

            if key in moveBindings:
                x, th = moveBindings[key]
                send_cmd(speed * x, turn * th)
                print(f"\rMoving: {speed*x:+.2f}, {turn*th:+.2f}   ", end='', flush=True)

            elif key in speedBindings:
                speed = min(max(speed * speedBindings[key], 0.1), 1.0)
                turn = min(max(turn * speedBindings[key], 0.2), 2.0)
                print(f"\nSpeed: {speed:.2f}  Turn: {turn:.2f}")

            elif key == ' ' or key == 'k':
                send_cmd(0.0, 0.0)
                print("\rSTOPPED              ", end='', flush=True)

            elif key == '\x03':
                break
    finally:
        cleanup()

if __name__ == '__main__':
    main()
