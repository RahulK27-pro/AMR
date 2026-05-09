#!/usr/bin/env python3
"""
slam_diagnostics.py — Run this WHILE master_launch.py is running.
Usage: python3 ~/agv_ws/src/agv_description/launch/slam_diagnostics.py
"""
import subprocess, sys, time

ROS = "source /opt/ros/jazzy/setup.bash && source ~/agv_ws/install/setup.bash"

def run(cmd, timeout=6):
    try:
        r = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True, text=True, timeout=timeout
        )
        return (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return "__TIMEOUT__"
    except Exception as e:
        return f"__ERROR__: {e}"

def check(label, cmd, ok_fn, timeout=6):
    out = run(cmd, timeout)
    if "__TIMEOUT__" in out:
        print(f"  ⏱  TIMEOUT — {label}")
        return False
    passed = ok_fn(out)
    mark = "✅ PASS" if passed else "❌ FAIL"
    print(f"  {mark} — {label}")
    if not passed:
        snippet = out[:200].replace("\n", " ") if out else "(no output)"
        print(f"           {snippet}")
    return passed

print("\n" + "="*60)
print("  SLAM DIAGNOSTIC — checking all prerequisites")
print("="*60)

# ── 1. Packages ───────────────────────────────────────────────
print("\n[1] INSTALLED PACKAGES")
check("slam_toolbox installed",
      f"{ROS} && ros2 pkg list 2>/dev/null | grep slam_toolbox",
      lambda o: "slam_toolbox" in o)
check("robot_localization installed",
      f"{ROS} && ros2 pkg list 2>/dev/null | grep robot_localization",
      lambda o: "robot_localization" in o)
check("libasync_slam_toolbox.so exists",
      "find /opt/ros/jazzy -name 'libasync_slam_toolbox.so' 2>/dev/null | head -1",
      lambda o: "libasync" in o)

# ── 2. Running Nodes ──────────────────────────────────────────
print("\n[2] RUNNING NODES")
nodes = run(f"{ROS} && ros2 node list 2>/dev/null", timeout=8)
for name in ["ros_gz_bridge", "robot_state_publisher", "ekf_filter_node"]:
    ok = name in nodes
    print(f"  {'✅ PASS' if ok else '❌ FAIL'} — {name} running")
    if not ok:
        print(f"           Active nodes: {nodes[:100]}")
        break

# ── 3. Critical Topics ────────────────────────────────────────
print("\n[3] CRITICAL TOPICS")
for topic in ["/clock", "/scan", "/odom", "/tf"]:
    check(f"{topic} publishing",
          f"{ROS} && timeout 3 ros2 topic hz {topic} 2>&1 | grep 'average rate'",
          lambda o: "average rate" in o, timeout=6)

# ── 4. Scan timestamp ─────────────────────────────────────────
print("\n[4] SCAN TIMESTAMP (must be non-zero)")
out = run(f"{ROS} && ros2 topic echo /scan --once 2>/dev/null | grep -A2 'stamp:'", timeout=8)
print(f"  Raw stamp output: {out[:100]}")
if "sec: 0" in out or out == "":
    print("  ❌ FAIL — timestamps are ZERO or no scan received!")
else:
    print("  ✅ PASS — scan has real timestamps")

# ── 5. TF Chain ───────────────────────────────────────────────
print("\n[5] TF CHAIN (odom → base_link → lidar_link)")
for src, dst in [("odom","base_link"), ("base_link","lidar_link"), ("odom","lidar_link")]:
    check(f"{src}→{dst} exists",
          f"{ROS} && timeout 4 ros2 run tf2_ros tf2_echo {src} {dst} 2>&1 | grep 'Translation'",
          lambda o: "Translation" in o, timeout=7)

# ── 6. TF Timestamps ──────────────────────────────────────────
print("\n[6] TF TIMESTAMPS (must NOT be 'At time 0.0')")
out = run(f"{ROS} && timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep 'At time'", timeout=7)
print(f"  Raw: {out[:80]}")
if "At time 0" in out:
    print("  ❌ FAIL — TF stamps are zero! /clock not reaching RSP/EKF at startup.")
elif "At time" in out:
    print("  ✅ PASS — TF has real timestamps")
else:
    print("  ❌ FAIL — No TF output at all (odom→base_link missing)")

# ── 7. /tf_static QoS ────────────────────────────────────────
print("\n[7] /tf_static QoS")
out = run(f"{ROS} && ros2 topic info /tf_static --verbose 2>/dev/null | head -25", timeout=6)
print(f"  {out[:300]}")

# ── 8. ldd check ─────────────────────────────────────────────
print("\n[8] SLAM LIBRARY LOADING")
out = run("ldd /opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node 2>&1 | grep 'not found'")
if out.strip() == "":
    print("  ✅ PASS — no missing libraries")
else:
    print(f"  ❌ FAIL — missing: {out}")

# ── 9. SLAM init with explicit DEBUG log level ───────────────
print("\n[9] SLAM INIT TEST (with --log-level DEBUG)")
log_test = run(
    f"{ROS} && RCUTILS_LOGGING_BUFFERED_STREAM=0 "
    f"timeout 5 ros2 run slam_toolbox async_slam_toolbox_node "
    f"--ros-args -p use_sim_time:=false --log-level DEBUG "
    f"> /tmp/slam_test.log 2>&1 ; cat /tmp/slam_test.log | head -25",
    timeout=10
)
print(f"  {log_test[:600]}")
if "CeresSolver" in log_test or "Mapper" in log_test or "plugin" in log_test.lower():
    print("  ✅ PASS — SLAM initializes correctly")
elif log_test.strip() == "":
    print("  ❌ FAIL — Absolutely no output at all")
else:
    print("  ⚠️  Got partial output — see above")

# ── 10. THE KEY CHECK: /scan frame_id ────────────────────────
print("\n[10] /scan FRAME_ID — THE SILENT SLAM KILLER")
out = run(
    f"{ROS} && ros2 topic echo /scan --once 2>/dev/null | grep 'frame_id' | head -3",
    timeout=8
)
print(f"  Raw: '{out}'")
if "lidar_link" in out and "warehouse_agv" not in out:
    print("  ✅ PASS — frame_id='lidar_link' (SLAM can find TF)")
elif "warehouse_agv" in out or "/" in out:
    print("  ❌ FAIL — frame_id is Gazebo-prefixed (e.g. warehouse_agv/lidar_link)!")
    print("     SLAM looks up TF for this frame — it doesn't exist — EVERY scan dropped silently")
elif out.strip() == "":
    print("  ⚠️  Could not get a scan — is robot spawned?")
else:
    print(f"  ⚠️  Unexpected: {out}")

# ── 11. /map topic ────────────────────────────────────────────
print("\n[11] /map TOPIC (only if SLAM is running and robot has moved)")
out = run(f"{ROS} && ros2 topic list 2>/dev/null | grep '/map'", timeout=5)
if "/map" in out:
    print("  ✅ PASS — /map topic exists! SLAM is publishing.")
else:
    print("  ❌ FAIL — /map not found. Start SLAM, drive robot, re-run diagnostic.")

print("\n" + "="*60)
print("  Done. Fix all ❌ items above, then re-run slam_launch.py.")
print("="*60 + "\n")

