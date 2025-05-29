import os
import time
import subprocess
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import psutil

# === CONFIGURATION ===
CHECKPOINT_DIR = "phase3_switch"
RUN_ID = "phase3_switch"  
CHECK_INTERVAL_MINUTES = 5
REWARD_THRESHOLD = 0.0
last_reward = 1

BEHAVIOR_NAME = "DroneAttacker"
YAML_DEFENDER = "phase_defender_active.yaml"
YAML_ATTACKER = "phase_attacker_active.yaml"

mlagents_process = None  # Variable globale


mlagents_process = None

def terminate_process_tree(proc):
    """Terminate a process and all its children using psutil."""
    if proc is None:
        return

    try:
        parent = psutil.Process(proc.pid)
        children = parent.children(recursive=True)
        for child in children:
            print(f"Terminating child process PID={child.pid}")
            child.terminate()
        gone, alive = psutil.wait_procs(children, timeout=5)

        if parent.is_running():
            print(f"Terminating main process PID={parent.pid}")
            parent.terminate()
            parent.wait(timeout=5)
    except Exception as e:
        print(f"Error terminating process tree: {e}")


def get_latest_mean_reward():
    event_dir = os.path.join("results", CHECKPOINT_DIR, BEHAVIOR_NAME)
    if not os.path.exists(event_dir):
        print("No event directory found.")
        return None

    event_files = [f for f in os.listdir(event_dir) if f.startswith("events.out.tfevents")]
    if not event_files:
        print("No TensorBoard event files found.")
        return None

    latest_event_file = max(event_files, key=lambda f: os.path.getmtime(os.path.join(event_dir, f)))
    ea = EventAccumulator(os.path.join(event_dir, latest_event_file))
    ea.Reload()

    tag = "Environment/Cumulative Reward"
    if tag not in ea.Tags().get("scalars", []):
        print(f"Tag {tag} not found in TensorBoard logs.")
        return None

    values = ea.Scalars(tag)
    if len(values) < 10:
        print("Not enough reward data yet. Using last known value:", last_reward)
        return last_reward

    last_10 = [v.value for v in values[-10:]]
    mean_reward = sum(last_10) / len(last_10)
    return mean_reward

def run_training(yaml_file):
    global mlagents_process

    if mlagents_process and mlagents_process.poll() is None:
        print("Stopping previous training process...")
        terminate_process_tree(mlagents_process)

    print(f"Launching training with: {yaml_file}")
    mlagents_process = subprocess.Popen([
        "cmd.exe", "/C",
        f"mlagents-learn config/{yaml_file} "
        f"--run-id={RUN_ID} "
        f"--env=build\\phase3_switch.exe "
        f"--num-envs=2 "
        f"--no-graphics --resume"
    ])

def main():
    iteration = 0
    while True:
        print(f"--- Check #{iteration} ---")
        reward = get_latest_mean_reward()
        print(f"Mean reward (DroneAttacker): {reward}")

        if reward is not None:
            if reward > REWARD_THRESHOLD:
                print("Attacker dominates. Training defenders.")
                run_training(YAML_DEFENDER)
            else:
                print("Defenders dominate or balanced. Training attacker.")
                run_training(YAML_ATTACKER)

        iteration += 1
        time.sleep(CHECK_INTERVAL_MINUTES * 60)

if __name__ == "__main__":
    main()
