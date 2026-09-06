import 'package:flutter/material.dart';
import '../services/amr_bridge_service.dart';
import 'control_tab.dart';
import 'monitor_tab.dart';
import 'map_screen.dart';

class HomeScreen extends StatefulWidget {
  final AmrBridgeService service;
  const HomeScreen({super.key, required this.service});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  Telemetry? _latestTelemetry;
  bool _connected = false;

  @override
  void initState() {
    super.initState();
    widget.service.connectTelemetry();
    widget.service.telemetryStream.listen((t) {
      if (mounted) setState(() => _latestTelemetry = t);
    });
    widget.service.connectionStream.listen((c) {
      if (mounted) setState(() => _connected = c);
    });
  }

  @override
  void dispose() {
    widget.service.dispose();
    super.dispose();
  }

  Color _navStateColor(String? state) {
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
    final navState = _latestTelemetry?.navState ?? 'IDLE';
    final stateColor = _navStateColor(navState);

    return DefaultTabController(
      length: 3,
      child: Scaffold(
        appBar: AppBar(
          title: Row(children: [
            const Text('AMR Control'),
            const SizedBox(width: 12),
            // Nav state badge
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
              decoration: BoxDecoration(
                color: stateColor.withOpacity(0.15),
                border: Border.all(color: stateColor, width: 1),
                borderRadius: BorderRadius.circular(12),
              ),
              child: Text(
                navState,
                style: TextStyle(
                  fontSize: 11, fontWeight: FontWeight.w700,
                  color: stateColor, letterSpacing: 0.8,
                ),
              ),
            ),
          ]),
          actions: [
            // Connection indicator
            Padding(
              padding: const EdgeInsets.only(right: 8),
              child: Icon(
                Icons.circle,
                size: 10,
                color: _connected ? Colors.greenAccent : Colors.redAccent,
              ),
            ),
            // E-STOP action button
            IconButton(
              icon: const Icon(Icons.stop_circle_outlined, color: Colors.redAccent),
              tooltip: 'Emergency Stop',
              onPressed: () async {
                await widget.service.sendStop();
                if (mounted) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('⛔ E-STOP sent!'),
                      backgroundColor: Colors.redAccent,
                      duration: Duration(seconds: 2),
                    ),
                  );
                }
              },
            ),
          ],
          bottom: const TabBar(tabs: [
            Tab(icon: Icon(Icons.map_outlined),       text: 'Map'),
            Tab(icon: Icon(Icons.gamepad_outlined),   text: 'Control'),
            Tab(icon: Icon(Icons.monitor_heart_outlined), text: 'Monitor'),
          ]),
        ),
        body: TabBarView(
          children: [
            MapScreen(service: widget.service, telemetry: _latestTelemetry),
            ControlTab(service: widget.service, telemetry: _latestTelemetry),
            MonitorTab(service: widget.service, telemetry: _latestTelemetry),
          ],
        ),
      ),
    );
  }
}
