import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import '../services/amr_bridge_service.dart';

// Map calibration constants — must match bridge_server.py
const double _mapResolution = 0.05; // m/px
const double _mapOriginX    = -8.3;
const double _mapOriginY    = -7.3;
const int    _mapHeightPx   = 275;

/// MapScreen — shows the static graph_visualization.png overlaid with
/// live robot pose (handled server-side, fetched every 2 s from /api/map).
/// Tap anywhere to send a navigation goal.
class MapScreen extends StatefulWidget {
  final AmrBridgeService service;
  final Telemetry? telemetry;

  const MapScreen({super.key, required this.service, this.telemetry});

  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  Uint8List? _mapBytes;
  Timer? _fetchTimer;
  bool _loading = true;
  String? _feedback;
  Timer? _feedbackTimer;

  // Initial pose mode
  bool _initialPoseMode = false;
  final _initXCtrl  = TextEditingController();
  final _initYCtrl  = TextEditingController();
  final _initThCtrl = TextEditingController(text: '0');

  @override
  void initState() {
    super.initState();
    _fetchMap();
    _fetchTimer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchMap());
  }

  @override
  void dispose() {
    _fetchTimer?.cancel();
    _feedbackTimer?.cancel();
    _initXCtrl.dispose();
    _initYCtrl.dispose();
    _initThCtrl.dispose();
    super.dispose();
  }

  Future<void> _fetchMap() async {
    final b64 = await widget.service.getMapImageB64();
    if (b64 == null || !mounted) return;
    try {
      final bytes = base64Decode(b64);
      setState(() { _mapBytes = bytes; _loading = false; });
    } catch (_) {}
  }

  void _showFeedback(String msg) {
    setState(() => _feedback = msg);
    _feedbackTimer?.cancel();
    _feedbackTimer = Timer(const Duration(seconds: 3), () {
      if (mounted) setState(() => _feedback = null);
    });
  }

  Future<void> _onTap(BuildContext context, TapDownDetails details, Size imgSize) async {
    // Actually: use displayed image widget dimensions
    final col = details.localPosition.dx / imgSize.width  * 329; // approx native width 329px
    final row = details.localPosition.dy / imgSize.height * _mapHeightPx;
    final wx = col * _mapResolution + _mapOriginX;
    final wy = (_mapHeightPx - row) * _mapResolution + _mapOriginY;

    if (_initialPoseMode) {
      _initXCtrl.text = wx.toStringAsFixed(2);
      _initYCtrl.text = wy.toStringAsFixed(2);
      return;
    }

    _showFeedback('Sending goal (${wx.toStringAsFixed(2)}, ${wy.toStringAsFixed(2)})…');
    await widget.service.sendGoal(wx, wy);
    _showFeedback('Goal sent → (${wx.toStringAsFixed(2)}, ${wy.toStringAsFixed(2)})');
  }

  @override
  Widget build(BuildContext context) {
    final t = widget.telemetry;
    final navState = t?.navState ?? 'IDLE';

    return Column(
      children: [
        // Toolbar
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          child: Row(children: [
            _NavStateBadge(state: navState),
            const Spacer(),
            if (t != null)
              Text(
                '(${t.x.toStringAsFixed(2)}, ${t.y.toStringAsFixed(2)}) m',
                style: const TextStyle(fontFamily: 'monospace', fontSize: 12, color: Colors.cyan),
              ),
            const SizedBox(width: 8),
            TextButton.icon(
              icon: Icon(_initialPoseMode ? Icons.check : Icons.my_location_outlined, size: 16),
              label: Text(_initialPoseMode ? 'Cancel' : 'Set Pose'),
              style: TextButton.styleFrom(
                foregroundColor: _initialPoseMode ? Colors.amberAccent : Colors.blue,
              ),
              onPressed: () => setState(() => _initialPoseMode = !_initialPoseMode),
            ),
          ]),
        ),

        // Initial pose form
        if (_initialPoseMode)
          Card(
            margin: const EdgeInsets.symmetric(horizontal: 12),
            child: Padding(
              padding: const EdgeInsets.all(12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text('Set AMCL Initial Pose', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 13)),
                  const SizedBox(height: 8),
                  Row(children: [
                    Expanded(child: TextField(controller: _initXCtrl, decoration: const InputDecoration(labelText: 'X (m)', isDense: true, border: OutlineInputBorder()), keyboardType: TextInputType.number)),
                    const SizedBox(width: 8),
                    Expanded(child: TextField(controller: _initYCtrl, decoration: const InputDecoration(labelText: 'Y (m)', isDense: true, border: OutlineInputBorder()), keyboardType: TextInputType.number)),
                    const SizedBox(width: 8),
                    Expanded(child: TextField(controller: _initThCtrl, decoration: const InputDecoration(labelText: 'θ (°)', isDense: true, border: OutlineInputBorder()), keyboardType: TextInputType.number)),
                  ]),
                  const SizedBox(height: 8),
                  SizedBox(
                    width: double.infinity,
                    child: FilledButton(
                      onPressed: () async {
                        final x  = double.tryParse(_initXCtrl.text)  ?? 0;
                        final y  = double.tryParse(_initYCtrl.text)  ?? 0;
                        final th = (double.tryParse(_initThCtrl.text) ?? 0) * 3.14159 / 180;
                        await widget.service.sendInitialPose(x, y, th);
                        setState(() => _initialPoseMode = false);
                        _showFeedback('Initial pose set at ($x, $y)');
                      },
                      child: const Text('Apply Initial Pose'),
                    ),
                  ),
                ],
              ),
            ),
          ),

        // Map
        Expanded(
          child: Padding(
            padding: const EdgeInsets.all(8),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(12),
              child: _loading
                  ? Center(
                      child: Column(mainAxisSize: MainAxisSize.min, children: const [
                        CircularProgressIndicator(),
                        SizedBox(height: 16),
                        Text('Loading map…', style: TextStyle(color: Colors.grey)),
                      ]),
                    )
                  : _mapBytes == null
                      ? const Center(child: Text('Map not available.\nIs bridge_server.py running?', textAlign: TextAlign.center))
                      : LayoutBuilder(builder: (ctx, constraints) {
                          final imgSize = Size(constraints.maxWidth, constraints.maxHeight);
                          return GestureDetector(
                            onTapDown: (d) => _onTap(ctx, d, imgSize),
                            child: Stack(
                              children: [
                                Image.memory(
                                  _mapBytes!,
                                  fit: BoxFit.contain,
                                  width: double.infinity,
                                  height: double.infinity,
                                ),
                                // Feedback toast
                                if (_feedback != null)
                                  Positioned(
                                    top: 8, left: 8, right: 8,
                                    child: Material(
                                      color: Colors.black87,
                                      borderRadius: BorderRadius.circular(8),
                                      child: Padding(
                                        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                                        child: Text(_feedback!, style: const TextStyle(color: Colors.cyanAccent, fontFamily: 'monospace', fontSize: 12)),
                                      ),
                                    ),
                                  ),
                                // Tap hint
                                Positioned(
                                  bottom: 8, right: 8,
                                  child: Container(
                                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                                    decoration: BoxDecoration(
                                      color: Colors.black54,
                                      borderRadius: BorderRadius.circular(6),
                                    ),
                                    child: const Text('Tap to navigate', style: TextStyle(fontSize: 11, color: Colors.white54)),
                                  ),
                                ),
                              ],
                            ),
                          );
                        }),
            ),
          ),
        ),
      ],
    );
  }
}

class _NavStateBadge extends StatelessWidget {
  final String state;
  const _NavStateBadge({required this.state});

  Color get _color {
    switch (state) {
      case 'NAVIGATING': return Colors.greenAccent;
      case 'PLANNING':   return Colors.blueAccent;
      case 'ARRIVED':    return Colors.amberAccent;
      case 'YIELDING':   return Colors.orangeAccent;
      case 'ESTOP':      return Colors.redAccent;
      default:           return Colors.grey;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
      decoration: BoxDecoration(
        color: _color.withOpacity(0.1),
        border: Border.all(color: _color, width: 1),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Row(mainAxisSize: MainAxisSize.min, children: [
        Container(width: 6, height: 6, decoration: BoxDecoration(color: _color, shape: BoxShape.circle)),
        const SizedBox(width: 6),
        Text(state, style: TextStyle(fontSize: 12, fontWeight: FontWeight.w700, color: _color, letterSpacing: 0.5)),
      ]),
    );
  }
}
