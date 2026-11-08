#!/usr/bin/env python3
"""Run the CabinLake episode 6 umbrella example using the SOP planner."""

import argparse, datetime, json, math, os, signal, socket, subprocess, time
from pathlib import Path
import airsim
import cv2
import numpy as np
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[2]


def listening(port):
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=int, default=600)
    args = parser.parse_args()
    if listening(11311) or listening(41451):
        raise SystemExit(
            "Stop the existing ROS master / AirSim before running this example."
        )
    if not (ROOT / "ros_ws/devel/setup.bash").exists():
        raise SystemExit("Run scripts/build.sh first.")
    exe = Path(os.environ["AIRHUNT_ENV_EXECUTABLE"])
    weights = Path(os.environ["AIRHUNT_CHECKPOINTS"])
    if not exe.is_file():
        raise SystemExit("AIRHUNT_ENV_EXECUTABLE does not exist.")
    for name in ["sam2.1_hiera_large.pt", "groundingdino_swint_ogc.pth"]:
        if not (weights / name).is_file():
            raise SystemExit(f"Missing model: {weights/name}")
    if not os.environ.get("DASHSCOPE_API_KEY"):
        raise SystemExit("Configure DASHSCOPE_API_KEY in .env.")
    out = (
        ROOT / "outputs" / datetime.datetime.now().strftime("navigation-%Y%m%d-%H%M%S")
    )
    out.mkdir(parents=True)
    episode = json.loads((Path(__file__).parent / "episode.json").read_text())
    (out / "episode.json").write_text(json.dumps(episode, indent=2))
    cfg = OmegaConf.load(ROOT / "configs/perception.yaml")
    dataset = {"scenes": {cfg.default_env: {"episodes": [episode]}}}
    (out / "dataset.json").write_text(json.dumps(dataset, indent=2))
    cfg.dataset_path = str(out / "dataset.json")
    for k in ["concept_graph_config_path", "grounding_dino_config"]:
        cfg[k] = str(ROOT / cfg[k])
    for k in ["sam2_checkpoint", "grounding_dino_checkpoint"]:
        cfg[k] = str(weights / cfg[k])
    cfg.output_parent_dir = str(out / "perception")
    OmegaConf.save(cfg, out / "perception.yaml")
    children = []
    handles = []
    client = None
    monitor = None

    def start(command, name, cwd=out):
        f = (out / (name + ".log")).open("w")
        handles.append(f)
        p = subprocess.Popen(
            command, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
        )
        children.append(p)
        return p

    def wait_for(condition, process, seconds, description):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if condition():
                return
            if process.poll() is not None:
                raise RuntimeError(f"{description} exited; see {out}")
            time.sleep(0.5)
        raise RuntimeError(f"{description} startup timeout; see {out}")

    def ros_command(*command):
        return [
            "bash",
            "-c",
            'source "$AIRHUNT_ROOT/ros_ws/devel/setup.bash"; exec "$@"',
            "--",
            *command,
        ]

    print("Output:", out, flush=True)
    try:
        os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        os.environ["__VK_LAYER_NV_optimus"] = "NVIDIA_only"
        simulator = start(
            [
                str(exe),
                exe.name.split("-Linux")[0],
                "-settings=" + str(ROOT / "configs/airsim.json"),
                "-vulkan",
                "-windowed",
                "-ResX=960",
                "-ResY=720",
                "-nosound",
            ],
            "environment",
            exe.parent,
        )
        wait_for(lambda: listening(41451), simulator, 120, "AirSim")
        client = airsim.VehicleClient(timeout_value=15)
        client.simSetVehiclePose(
            airsim.Pose(
                airsim.Vector3r(*episode["start_position"]),
                airsim.Quaternionr(*episode["start_rotation"]),
            ),
            True,
            vehicle_name="drone_1",
        )
        master = start(ros_command("roscore"), "roscore")
        wait_for(lambda: listening(11311), master, 30, "ROS master")
        semantic = start(
            ros_command(
                "python",
                "-u",
                "-m",
                "airhunt.server",
                "-cf",
                str(out / "perception.yaml"),
            ),
            "perception",
        )
        wait_for(
            lambda: "Starting continuous camera data collection"
            in (out / "perception.log").read_text(errors="replace"),
            semantic,
            180,
            "Perception",
        )
        monitor = start(
            ros_command(
                "python",
                "-u",
                "-m",
                "airhunt.recording",
                "--output",
                str(out),
                "--timeout",
                str(args.timeout),
            ),
            "recording",
        )
        x, y, z = episode["start_position"]
        qx, qy, qz, qw = episode["start_rotation"]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        planner = start(
            ros_command(
                "roslaunch",
                str(ROOT / "configs/navigation.launch"),
                f"init_x:={y}",
                f"init_y:={x}",
                f"init_z:={-z}",
                f"init_yaw:={math.pi/2-yaw}",
            ),
            "planner",
        )
        while monitor.poll() is None:
            if planner.poll() is not None or semantic.poll() not in (None, 0):
                raise RuntimeError(
                    "A runtime process exited; inspect planner/perception.log"
                )
            if "process has died" in (out / "planner.log").read_text(errors="replace"):
                raise RuntimeError("A ROS node died; inspect planner.log")
            time.sleep(0.5)
    finally:
        # Stop the ROS/perception group before taking the actual final camera image.
        for p in reversed(children[1:]):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for p in reversed(children[1:]):
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()
        if client is not None:
            try:
                pose = client.simGetVehiclePose(vehicle_name="drone_1").position
                (out / "final-pose.json").write_text(
                    json.dumps([pose.x_val, pose.y_val, pose.z_val])
                )
                requests = [
                    airsim.ImageRequest(camera, airsim.ImageType.Scene, False, False)
                    for camera in ["front_camera", "left_camera", "right_camera"]
                ]
                responses = client.simGetImages(requests, vehicle_name="drone_1")
                for name, response in zip(["camera", "left", "right"], responses):
                    frame = np.frombuffer(
                        response.image_data_uint8, dtype=np.uint8
                    ).reshape(response.height, response.width, 3)
                    cv2.imwrite(str(out / f"final-{name}.png"), frame)
            except Exception as exc:
                print("Final capture:", type(exc).__name__, flush=True)
        for p in children:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()
        for f in handles:
            f.close()
    result = json.loads((out / "result.json").read_text())
    print(json.dumps(result, indent=2), flush=True)
    print(
        "Review final-camera.png and detections before claiming category success.",
        flush=True,
    )
    return 0 if result["completed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
