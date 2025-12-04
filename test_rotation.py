#!/usr/bin/env python3
"""
Test script for robot rotation functions.

Run this to test the robot's in-place rotation capabilities.
Tests both left and right rotation in forward and backward modes.

Usage:
    ./test_rotation.py              # Interactive menu
    ./test_rotation.py left         # Quick left rotation test
    ./test_rotation.py right        # Quick right rotation test
    ./test_rotation.py all          # Run all tests sequentially
    ./test_rotation.py --degrees 90 # Rotate specific angle
"""
import sys
import time
import argparse

from rotation_utils import rotate_left, rotate_right, rotate_degrees, stop


def test_left_forward(duration: float = 2.0, speed: float = 0.5):
    """Test left rotation in forward mode."""
    print("\n" + "="*50)
    print("TEST: Left rotation (forward)")
    print("="*50)
    input("Press Enter to start (robot will turn LEFT)...")
    rotate_left(duration=duration, speed=speed, backward=False)
    print("Test complete.\n")


def test_left_backward(duration: float = 2.0, speed: float = 0.5):
    """Test left rotation in backward mode."""
    print("\n" + "="*50)
    print("TEST: Left rotation (backward/inverted)")
    print("="*50)
    input("Press Enter to start (robot will turn LEFT with inverted direction)...")
    rotate_left(duration=duration, speed=speed, backward=True)
    print("Test complete.\n")


def test_right_forward(duration: float = 2.0, speed: float = 0.5):
    """Test right rotation in forward mode."""
    print("\n" + "="*50)
    print("TEST: Right rotation (forward)")
    print("="*50)
    input("Press Enter to start (robot will turn RIGHT)...")
    rotate_right(duration=duration, speed=speed, backward=False)
    print("Test complete.\n")


def test_right_backward(duration: float = 2.0, speed: float = 0.5):
    """Test right rotation in backward mode."""
    print("\n" + "="*50)
    print("TEST: Right rotation (backward/inverted)")
    print("="*50)
    input("Press Enter to start (robot will turn RIGHT with inverted direction)...")
    rotate_right(duration=duration, speed=speed, backward=True)
    print("Test complete.\n")


def test_90_degrees():
    """Test 90-degree rotations."""
    print("\n" + "="*50)
    print("TEST: 90-degree rotations")
    print("="*50)

    input("Press Enter to rotate 90° LEFT...")
    rotate_degrees(90, speed=0.35)
    time.sleep(1)

    input("Press Enter to rotate 90° RIGHT...")
    rotate_degrees(-90, speed=0.35)
    print("Test complete.\n")


def test_full_rotation():
    """Test a full 360-degree rotation."""
    print("\n" + "="*50)
    print("TEST: Full 360° rotation")
    print("="*50)

    input("Press Enter to rotate 360° LEFT (full circle)...")
    rotate_degrees(360, speed=0.4)
    print("Test complete.\n")


def interactive_menu():
    """Show interactive test menu."""
    while True:
        print("\n" + "="*50)
        print("ROTATION TEST MENU")
        print("="*50)
        print("1. Test LEFT rotation (forward)")
        print("2. Test LEFT rotation (backward)")
        print("3. Test RIGHT rotation (forward)")
        print("4. Test RIGHT rotation (backward)")
        print("5. Test 90° rotations")
        print("6. Test full 360° rotation")
        print("7. Custom rotation (enter degrees)")
        print("8. Run ALL tests")
        print("0. Exit")
        print("-"*50)

        try:
            choice = input("Select test (0-8): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break

        if choice == '0':
            print("Exiting...")
            break
        elif choice == '1':
            test_left_forward()
        elif choice == '2':
            test_left_backward()
        elif choice == '3':
            test_right_forward()
        elif choice == '4':
            test_right_backward()
        elif choice == '5':
            test_90_degrees()
        elif choice == '6':
            test_full_rotation()
        elif choice == '7':
            try:
                degrees = float(input("Enter degrees (positive=left, negative=right): "))
                speed = float(input("Enter speed (0.1-1.0, default 0.35): ") or "0.35")
                backward = input("Backward mode? (y/n, default n): ").lower() == 'y'
                print(f"\nRotating {degrees}° at {speed} rad/s {'(backward)' if backward else '(forward)'}...")
                rotate_degrees(degrees, speed=speed, backward=backward)
            except ValueError:
                print("Invalid input!")
        elif choice == '8':
            print("\nRunning ALL tests sequentially...")
            test_left_forward()
            time.sleep(1)
            test_right_forward()
            time.sleep(1)
            test_left_backward()
            time.sleep(1)
            test_right_backward()
            time.sleep(1)
            test_90_degrees()
            print("\nAll tests complete!")
        else:
            print("Invalid choice!")


def main():
    parser = argparse.ArgumentParser(description='Test robot rotation functions')
    parser.add_argument('command', nargs='?', default='menu',
                        choices=['menu', 'left', 'right', 'all', 'degrees'],
                        help='Test command (default: menu)')
    parser.add_argument('--degrees', '-d', type=float, default=90,
                        help='Degrees to rotate (for degrees command)')
    parser.add_argument('--speed', '-s', type=float, default=0.5,
                        help='Rotation speed in rad/s (default: 0.5)')
    parser.add_argument('--duration', '-t', type=float, default=2.0,
                        help='Duration in seconds for left/right commands')
    parser.add_argument('--backward', '-b', action='store_true',
                        help='Use backward rotation mode')

    args = parser.parse_args()

    print("="*50)
    print("UGV ROTATION TEST")
    print("="*50)
    print("Press Ctrl+C at any time to stop the robot")
    print()

    try:
        if args.command == 'menu':
            interactive_menu()
        elif args.command == 'left':
            print(f"Quick test: LEFT rotation for {args.duration}s at {args.speed} rad/s")
            rotate_left(duration=args.duration, speed=args.speed, backward=args.backward)
        elif args.command == 'right':
            print(f"Quick test: RIGHT rotation for {args.duration}s at {args.speed} rad/s")
            rotate_right(duration=args.duration, speed=args.speed, backward=args.backward)
        elif args.command == 'degrees':
            print(f"Rotating {args.degrees}° at {args.speed} rad/s")
            rotate_degrees(args.degrees, speed=args.speed, backward=args.backward)
        elif args.command == 'all':
            print("Running all rotation tests...")
            test_left_forward(args.duration, args.speed)
            time.sleep(1)
            test_right_forward(args.duration, args.speed)
            time.sleep(1)
            test_left_backward(args.duration, args.speed)
            time.sleep(1)
            test_right_backward(args.duration, args.speed)
            print("\nAll tests complete!")

    except KeyboardInterrupt:
        print("\n\nInterrupted! Stopping robot...")
        stop()
        print("Robot stopped.")
        sys.exit(0)

    finally:
        # Ensure robot is stopped on exit
        stop()


if __name__ == '__main__':
    main()
