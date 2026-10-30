"""
Tension the tendons of the ORCA v2 hand: the motors wind the tendons taut and then
hold them under current while you adjust them. Press Ctrl+C when done.
Stop the hand controller node first: both need the hand's serial port.
"""
import argparse
import os

from orca_core import load_hand


def print_progress(event: dict):
    phase = event.get("phase") if event.get("event") == "phase" else None
    if phase == "winding":
        print("Winding tendons taut...")
    elif phase == "holding":
        print("Holding motors for manual tensioning - press Ctrl+C when done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hold the ORCA v2 hand under tension so that you can tension the tendons")
    parser.add_argument(
        "--config_dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs", "orcahand_v2"),
        help="Directory containing the per-side ORCA v2 config folders.",
    )
    parser.add_argument("--left_hand", action="store_true", help="Use the left hand configuration (default is right hand).")
    args = parser.parse_args()

    hand = load_hand(config_path=os.path.join(args.config_dir, "left" if args.left_hand else "right", "config.yaml"))

    ok, message = hand.connect(interactive=False)
    print(message)
    if ok:
        try:
            hand.tension(progress_callback=print_progress)
        except KeyboardInterrupt:
            print("\nTensioning stopped.")
        finally:
            hand.disconnect()
