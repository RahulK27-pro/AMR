import 'dart:async';
import 'package:flutter/material.dart';
import '../services/amr_bridge_service.dart';

const double _vMax = 0.8;
const double _wMax = 1.8;

/// ControlTab — D-pad teleop, E-STOP, speed slider, manual goal / initial pose,
/// goal sequence runner.
class ControlTab extends StatefulWidget {
  final AmrBridgeService service;
  final Telemetry? telemetry;

  const ControlTab({super.key, required this.service, this.telemetry});

  @override
  State<ControlTab> createState() => _ControlTabState();
}

class _ControlTabState extends State<ControlTab> {
  double _speedFactor = 0.5;
  Timer? _cmdTimer;
  double _joyLinear = 0, _joyAngular = 0;

  // Goal form
  final _goalXCtrl = TextEditingController();
  final _goalYCtrl = TextEditingController();

  // Sequence form
  final _seqCtrl = TextEditingController();

  bool get _estopActive => widget.telemetry?.navState == 'ESTOP';

  @override
  void initState() {
    super.initState();
    // Send cmd_vel at 10 Hz while a D-pad button is held
    _cmdTimer = Timer.periodic(const Duration(milliseconds: 100), (_) {
      if (_joyLinear != 0 || _joyAngular != 0) {
        widget.service.sendCmdVel(_joyLinear, _joyAngular);
      }
    });
  }

  @override
  void dispose() {
    _cmdTimer?.cancel();
    _goalXCtrl.dispose();
    _goalYCtrl.dispose();
    _seqCtrl.dispose();
    super.dispose();
  }

  void _startMove(double linear, double angular) {
    setState(() {
      _joyLinear  = linear  * _speedFactor;
      _joyAngular = angular * _speedFactor;
    });
  }

  void _stopMove() {
    setState(() { _joyLinear = 0; _joyAngular = 0; });
    widget.service.sendCmdVel(0, 0);
  }

  Widget _dpadBtn(IconData icon, double lin, double ang, {double size = 56}) {
    return GestureDetector(
      onTapDown:   (_) => _startMove(lin, ang),
      onTapUp:     (_) => _stopMove(),
      onTapCancel:  () => _stopMove(),
      child: Container(
        width: size, height: size,
        decoration: BoxDecoration(
          color: Theme.of(context).colorScheme.surfaceVariant,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: Colors.white24),
        ),
        child: Icon(icon, size: 24),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [

          // ── E-STOP ──────────────────────────────────────────────────────
          if (_estopActive)
            _EstopActive(onClear: () async {
              await widget.service.clearEstop();
            })
          else
            SizedBox(
              width: double.infinity,
              height: 56,
              child: ElevatedButton.icon(
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.redAccent,
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                ),
                icon: const Icon(Icons.stop_circle_outlined),
                label: const Text('EMERGENCY STOP', style: TextStyle(fontWeight: FontWeight.w800, letterSpacing: 1)),
                onPressed: () async { await widget.service.sendStop(); },
              ),
            ),

          const SizedBox(height: 20),

          // ── D-pad ────────────────────────────────────────────────────────
          Card(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(children: [
                Text('Teleop', style: Theme.of(context).textTheme.labelLarge?.copyWith(color: Colors.white54)),
                const SizedBox(height: 12),
                // Speed slider
                Row(children: [
                  const Icon(Icons.slow_motion_video_outlined, size: 16, color: Colors.white38),
                  Expanded(child: Slider(
                    value: _speedFactor,
                    min: 0.1, max: 1.0, divisions: 9,
                    label: '${(_speedFactor * 100).round()}%',
                    onChanged: (v) => setState(() => _speedFactor = v),
                  )),
                  const Icon(Icons.fast_forward_outlined, size: 16, color: Colors.white38),
                ]),
                const SizedBox(height: 8),
                // Up
                Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                  _dpadBtn(Icons.keyboard_arrow_up, _vMax, 0),
                ]),
                const SizedBox(height: 4),
                // Left | Stop | Right
                Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                  _dpadBtn(Icons.keyboard_arrow_left,  0,     _wMax),
                  const SizedBox(width: 4),
                  GestureDetector(
                    onTap: () { _stopMove(); widget.service.sendCmdVel(0, 0); },
                    child: Container(
                      width: 56, height: 56,
                      decoration: BoxDecoration(
                        color: Colors.red.withOpacity(0.2),
                        borderRadius: BorderRadius.circular(12),
                        border: Border.all(color: Colors.redAccent.withOpacity(0.5)),
                      ),
                      child: const Icon(Icons.stop, color: Colors.redAccent),
                    ),
                  ),
                  const SizedBox(width: 4),
                  _dpadBtn(Icons.keyboard_arrow_right, 0, -_wMax),
                ]),
                const SizedBox(height: 4),
                // Down
                Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                  _dpadBtn(Icons.keyboard_arrow_down, -_vMax, 0),
                ]),
              ]),
            ),
          ),

          const SizedBox(height: 16),

          // ── Manual goal ──────────────────────────────────────────────────
          Card(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(children: [
                    const Icon(Icons.navigation_outlined, size: 16, color: Colors.white54),
                    const SizedBox(width: 6),
                    Text('Send Goal', style: Theme.of(context).textTheme.labelLarge?.copyWith(color: Colors.white54)),
                  ]),
                  const SizedBox(height: 12),
                  Row(children: [
                    Expanded(child: TextField(
                      controller: _goalXCtrl,
                      decoration: const InputDecoration(labelText: 'X (m)', isDense: true, border: OutlineInputBorder()),
                      keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
                    )),
                    const SizedBox(width: 8),
                    Expanded(child: TextField(
                      controller: _goalYCtrl,
                      decoration: const InputDecoration(labelText: 'Y (m)', isDense: true, border: OutlineInputBorder()),
                      keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
                    )),
                  ]),
                  const SizedBox(height: 10),
                  Row(children: [
                    Expanded(child: FilledButton.icon(
                      icon: const Icon(Icons.play_arrow, size: 18),
                      label: const Text('Navigate'),
                      onPressed: () async {
                        final x = double.tryParse(_goalXCtrl.text);
                        final y = double.tryParse(_goalYCtrl.text);
                        if (x == null || y == null) return;
                        await widget.service.sendGoal(x, y);
                        if (mounted) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(content: Text('Goal sent → ($x, $y)'), duration: const Duration(seconds: 2)));
                        }
                      },
                    )),
                    const SizedBox(width: 8),
                    OutlinedButton.icon(
                      icon: const Icon(Icons.my_location, size: 16),
                      label: const Text('Init Pose'),
                      onPressed: () async {
                        final x = double.tryParse(_goalXCtrl.text) ?? 0;
                        final y = double.tryParse(_goalYCtrl.text) ?? 0;
                        await widget.service.sendInitialPose(x, y, 0);
                        if (mounted) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(content: Text('Initial pose set at ($x, $y)'), duration: const Duration(seconds: 2)));
                        }
                      },
                    ),
                  ]),
                ],
              ),
            ),
          ),

          const SizedBox(height: 16),

          // ── Goal Sequence ─────────────────────────────────────────────────
          Card(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(children: [
                    const Icon(Icons.checklist_rtl_outlined, size: 16, color: Colors.white54),
                    const SizedBox(width: 6),
                    Text('Goal Sequence', style: Theme.of(context).textTheme.labelLarge?.copyWith(color: Colors.white54)),
                  ]),
                  const SizedBox(height: 12),
                  TextField(
                    controller: _seqCtrl,
                    decoration: const InputDecoration(
                      labelText: 'Node IDs (e.g. N5, N12, N40)',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                  const SizedBox(height: 10),
                  SizedBox(
                    width: double.infinity,
                    child: FilledButton.tonal(
                      child: const Text('Run Sequence'),
                      onPressed: () async {
                        final nodes = _seqCtrl.text.split(',').map((s) => s.trim()).where((s) => s.isNotEmpty).toList();
                        if (nodes.isEmpty) return;
                        await widget.service.sendGoalSequence(nodes);
                        if (mounted) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(content: Text('Sequence sent: ${nodes.join(" → ")}'), duration: const Duration(seconds: 3)));
                        }
                      },
                    ),
                  ),
                ],
              ),
            ),
          ),

        ],
      ),
    );
  }
}

class _EstopActive extends StatelessWidget {
  final VoidCallback onClear;
  const _EstopActive({required this.onClear});

  @override
  Widget build(BuildContext context) => Container(
    width: double.infinity,
    padding: const EdgeInsets.all(16),
    decoration: BoxDecoration(
      color: Colors.red.withOpacity(0.1),
      border: Border.all(color: Colors.redAccent),
      borderRadius: BorderRadius.circular(10),
    ),
    child: Column(children: [
      const Text('⛔ E-STOP ACTIVE', style: TextStyle(color: Colors.redAccent, fontWeight: FontWeight.w800, fontSize: 16, letterSpacing: 1)),
      const SizedBox(height: 10),
      SizedBox(
        width: double.infinity,
        child: OutlinedButton(
          onPressed: onClear,
          style: OutlinedButton.styleFrom(foregroundColor: Colors.greenAccent),
          child: const Text('✓ Clear E-Stop & Resume'),
        ),
      ),
    ]),
  );
}
