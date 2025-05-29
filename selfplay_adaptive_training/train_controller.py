import os
import time
import subprocess
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

# === CONFIGURATION ===
CHECKPOINT_DIR = "results/phase3_5x256_101"
RUN_ID_PREFIX = "auto_phase3"
CHECK_INTERVAL_MINUTES = 10
REWARD_THRESHOLD = 0.0

BEHAVIOR_NAME = "DroneAttacker"
YAML_DEFENDER = "phase_defender_active.yaml"
YAML_ATTACKER = "phase_attacker_active.yaml"

def get_latest_mean_reward():
    event_dir = os.path.join(CHECKPOINT_DIR, BEHAVIOR_NAME)
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
        print("Not enough reward data yet.")
        return None

    last_10 = [v.value for v in values[-10:]]
    mean_reward = sum(last_10) / len(last_10)
    return mean_reward

def run_training(yaml_file, run_id):
    print(f"Launching training with: {yaml_file}")
    subprocess.Popen([
        "cmd.exe", "/C",
        f"mlagents-learn config/{yaml_file} --run-id={run_id} --initialize-from={CHECKPOINT_DIR} --force"
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
                run_training(YAML_DEFENDER, f"{RUN_ID_PREFIX}_def_{iteration}")
            else:
                print("Defenders dominate or balanced. Training attacker.")
                run_training(YAML_ATTACKER, f"{RUN_ID_PREFIX}_att_{iteration}")

        iteration += 1
        time.sleep(CHECK_INTERVAL_MINUTES * 60)

if __name__ == "__main__":
    main()
