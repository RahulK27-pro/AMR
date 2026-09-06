import 'package:flutter/material.dart';
import '../services/amr_bridge_service.dart';

/// MonitorTab — expanded real-time telemetry display.
class MonitorTab extends StatelessWidget {
  final AmrBridgeService service;
  final Telemetry? telemetry;

  const MonitorTab({super.key, required this.service, this.telemetry});

  @override
  Widget build(BuildContext context) {
    final t = telemetry;

    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Obstacle Alert
          if (t?.obstacleAlert != null)
            _AlertBanner(message: t!.obstacleAlert!),

          // Pose
          _TelemCard(
            title: 'Robot Pose',
            icon: Icons.place_outlined,
            children: [
              _TelemRow('X',         '${(t?.x ?? 0).toStringAsFixed(3)} m'),
              _TelemRow('Y',         '${(t?.y ?? 0).toStringAsFixed(3)} m'),
              _TelemRow('Yaw',       '${_rad2deg(t?.yaw ?? 0).toStringAsFixed(1)}°'),
              _TelemRow('Yaw (rad)', '${(t?.yaw ?? 0).toStringAsFixed(4)} rad'),
            ],
          ),

          const SizedBox(height: 12),

          // Velocity
          _TelemCard(
            title: 'Velocity',
            icon: Icons.speed_outlined,
            children: [
              _TelemRow('Linear',  '${(t?.linear  ?? 0).toStringAsFixed(3)} m/s'),
              _TelemRow('Angular', '${(t?.angular ?? 0).toStringAsFixed(3)} rad/s'),
            ],
          ),

          const SizedBox(height: 12),

          // Proximity
          _TelemCard(
            title: 'Obstacle Proximity',
            icon: Icons.radar_outlined,
            children: [
              _ProximityBar(value: t?.minObstacleDist),
            ],
          ),

          const SizedBox(height: 12),

          // IMU
          _TelemCard(
            title: 'IMU Orientation',
            icon: Icons.compass_calibration_outlined,
            children: [
              _TelemRow('Roll',  '${_rad2deg(t?.imu.roll  ?? 0).toStringAsFixed(2)}°'),
              _TelemRow('Pitch', '${_rad2deg(t?.imu.pitch ?? 0).toStringAsFixed(2)}°'),
              _TelemRow('Yaw',   '${_rad2deg(t?.imu.yaw   ?? 0).toStringAsFixed(2)}°'),
            ],
          ),

          const SizedBox(height: 12),

          // Mission
          if (t?.mission.state != null)
            _TelemCard(
              title: 'Mission',
              icon: Icons.checklist_rtl_outlined,
              children: [
                _TelemRow('State', t!.mission.state ?? '—'),
                if (t.mission.total > 0)
                  _TelemRow('Progress', '${t.mission.current}/${t.mission.total}'),
                if (t.mission.goalNode != null)
                  _TelemRow('Goal Node', t.mission.goalNode!),
              ],
            ),

          const SizedBox(height: 12),

          // Path info
          _TelemCard(
            title: 'Active Path',
            icon: Icons.route_outlined,
            children: [
              _TelemRow('Waypoints', '${t?.path.length ?? 0}'),
              if ((t?.path.length ?? 0) > 0)
                _TelemRow('Final WP', '(${t!.path.last[0].toStringAsFixed(2)}, ${t.path.last[1].toStringAsFixed(2)})'),
            ],
          ),

        ],
      ),
    );
  }

  static double _rad2deg(double r) => r * 180 / 3.14159;
}

// ── Sub-widgets ──────────────────────────────────────────────────────────────

class _AlertBanner extends StatelessWidget {
  final String message;
  const _AlertBanner({required this.message});

  @override
  Widget build(BuildContext context) => Container(
    margin: const EdgeInsets.only(bottom: 12),
    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
    decoration: BoxDecoration(
      color: Colors.red.withOpacity(0.15),
      border: Border.all(color: Colors.redAccent.withOpacity(0.5)),
      borderRadius: BorderRadius.circular(8),
    ),
    child: Row(children: [
      const Icon(Icons.warning_amber_outlined, color: Colors.redAccent, size: 18),
      const SizedBox(width: 8),
      Expanded(child: Text(message, style: const TextStyle(color: Colors.redAccent, fontSize: 13))),
    ]),
  );
}

class _TelemCard extends StatelessWidget {
  final String title;
  final IconData icon;
  final List<Widget> children;
  const _TelemCard({required this.title, required this.icon, required this.children});

  @override
  Widget build(BuildContext context) => Card(
    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
    child: Padding(
      padding: const EdgeInsets.all(14),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Icon(icon, size: 16, color: Colors.white54),
            const SizedBox(width: 6),
            Text(title, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w700, color: Colors.white54, letterSpacing: 0.8)),
          ]),
          const Divider(height: 16, thickness: 0.5),
          ...children,
        ],
      ),
    ),
  );
}

class _TelemRow extends StatelessWidget {
  final String label, value;
  const _TelemRow(this.label, this.value);

  @override
  Widget build(BuildContext context) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 3),
    child: Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(label, style: const TextStyle(fontSize: 13, color: Colors.white54)),
        Text(value,  style: const TextStyle(fontSize: 13, fontFamily: 'monospace', fontWeight: FontWeight.w600, color: Colors.cyanAccent)),
      ],
    ),
  );
}

class _ProximityBar extends StatelessWidget {
  final double? value;
  const _ProximityBar({this.value});

  @override
  Widget build(BuildContext context) {
    final v     = value ?? 8;
    final pct   = (v / 8).clamp(0.0, 1.0);
    final color = v < 0.5 ? Colors.redAccent : v < 1.2 ? Colors.orangeAccent : Colors.greenAccent;
    return Column(children: [
      Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [
        const Text('Nearest obstacle', style: TextStyle(fontSize: 13, color: Colors.white54)),
        Text(value != null ? '${v.toStringAsFixed(2)} m' : '—',
          style: TextStyle(fontSize: 13, fontFamily: 'monospace', fontWeight: FontWeight.w600, color: color)),
      ]),
      const SizedBox(height: 6),
      ClipRRect(
        borderRadius: BorderRadius.circular(3),
        child: LinearProgressIndicator(
          value: pct,
          minHeight: 6,
          backgroundColor: Colors.white12,
          valueColor: AlwaysStoppedAnimation(color),
        ),
      ),
    ]);
  }
}
