"""Record example telemetry; destination coordinates are evaluation-only inputs."""

import argparse, csv, json, math, time, subprocess
from pathlib import Path
import rospy
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--timeout", type=float, default=300)
    args = p.parse_args()
    out = args.output
    episode = json.loads((out / "episode.json").read_text())
    target = episode["goals"][0]["target_position"]
    state = dict(fsm=None, position=None, selected=None, image=None)
    events = []
    start = time.monotonic()
    bridge = CvBridge()

    def fsm(msg):
        if msg.data != state["fsm"]:
            events.append(dict(time=time.monotonic() - start, fsm=msg.data))
        state["fsm"] = msg.data

    def odom(msg):
        a = msg.pose.pose.position
        state["position"] = [a.y, a.x, -a.z]

    def selected(msg):
        a = msg.pose.position
        state["selected"] = [a.y, a.x, -a.z]
        events.append(
            dict(time=time.monotonic() - start, selected_ned=state["selected"])
        )

    def camera(msg):
        state["image"] = msg

    rospy.init_node("example_recorder", anonymous=True)
    rospy.Subscriber("/fsm_state", String, fsm)
    rospy.Subscriber("/simulator/odom", Odometry, odom)
    rospy.Subscriber("/task2/pose", PoseStamped, selected)
    rospy.Subscriber("/camera/front/rgb", Image, camera, queue_size=1)
    video = None
    frames = 0
    last = None
    length = 0.0
    positions = []
    times = []
    next_video = 0
    with (out / "trajectory.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "north_m", "east_m", "down_m", "fsm"])
        while not rospy.is_shutdown() and time.monotonic() - start < args.timeout:
            t = time.monotonic() - start
            a = state["position"]
            if a:
                if last:
                    length += math.dist(a, last)
                positions.append(a[:])
                times.append(t)
                last = a[:]
                writer.writerow([t, *a, state["fsm"]])
                f.flush()
            if state["image"] is not None and t >= next_video:
                frame = bridge.imgmsg_to_cv2(state["image"], "bgr8")
                if video is None:
                    video = cv2.VideoWriter(
                        str(out / "camera-raw.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        10.0,
                        (frame.shape[1], frame.shape[0]),
                    )
                if not video.isOpened():
                    raise RuntimeError("Cannot create video")
                cv2.putText(
                    frame,
                    f"{t:.1f}s {state['fsm'] or 'initializing'}",
                    (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 255),
                    1,
                )
                video.write(frame)
                frames += 1
                next_video = t + 0.095
            if state["fsm"] == "FINISH":
                break
            time.sleep(0.1)
    if video:
        video.release()
    elapsed = time.monotonic() - start
    completed = state["fsm"] == "FINISH" and state["selected"] is not None
    result = dict(
        completed=completed,
        stop_reason="planner_finish" if state["fsm"] == "FINISH" else ("interrupted" if rospy.is_shutdown() else "timeout"),
        elapsed_s=elapsed,
        path_length_m=length,
        final_position_ned=last,
        selected_object_ned=state["selected"],
        annotated_goal_ned=target,
        distance_to_annotation_m=math.dist(last, target) if last else None,
        events=events,
        visual_confirmation_required=True,
    )
    (out / "result.json").write_text(json.dumps(result, indent=2))
    if frames:
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-i",
                str(out / "camera-raw.mp4"),
                "-vf",
                f"setpts={elapsed / (frames / 10):.8f}*PTS",
                "-r",
                "10",
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "22",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(out / "camera.mp4"),
            ],
            check=True,
        )
        (out / "camera-raw.mp4").unlink()
    if positions:
        import matplotlib.pyplot as plt

        origin = episode["start_position"]
        x = [a[1] - origin[1] for a in positions]
        y = [a[0] - origin[0] for a in positions]
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(x, y, label=f"SOP: {length:.1f} m")
        ax.scatter(0, 0, c="green", label="Start")
        ax.scatter(
            target[1] - origin[1],
            target[0] - origin[0],
            c="red",
            marker="*",
            s=170,
            label="JSON destination",
        )
        ax.scatter(x[-1], y[-1], marker="x", c="black", label="Stop")
        if state["selected"] is not None:
            a = state["selected"]
            ax.scatter(
                a[1] - origin[1],
                a[0] - origin[0],
                marker="D",
                facecolors="none",
                edgecolors="purple",
                label="Selected object",
            )
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.2)
        ax.set_xlabel("East from start (m)")
        ax.set_ylabel("North from start (m)")
        ax.set_title(episode["goals"][0]["query"])
        ax.legend()
        fig.tight_layout()
        fig.savefig(out / "trajectory.png", dpi=160)
        plt.close(fig)
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
