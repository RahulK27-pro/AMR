import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------

class ImuData {
  final double roll, pitch, yaw;
  const ImuData({required this.roll, required this.pitch, required this.yaw});
  factory ImuData.fromJson(Map<String, dynamic> j) => ImuData(
        roll:  (j['roll']  as num).toDouble(),
        pitch: (j['pitch'] as num).toDouble(),
        yaw:   (j['yaw']   as num).toDouble(),
      );
  static const zero = ImuData(roll: 0, pitch: 0, yaw: 0);
}

class MissionData {
  final String? state;
  final int current, total;
  final String? goalNode;
  const MissionData({this.state, this.current = 0, this.total = 0, this.goalNode});
  factory MissionData.fromJson(Map<String, dynamic> j) => MissionData(
        state:    j['state'] as String?,
        current:  (j['current'] as num?)?.toInt() ?? 0,
        total:    (j['total']   as num?)?.toInt() ?? 0,
        goalNode: j['goal_node'] as String?,
      );
}

class Telemetry {
  final double x, y, yaw;
  final double linear, angular;
  final double? minObstacleDist;
  final String navState;
  final ImuData imu;
  final List<List<double>> path;
  final String? obstacleAlert;
  final MissionData mission;
  final List<double> scan;
  final double scanAngleMin, scanAngleInc;
  final double? ts;

  const Telemetry({
    required this.x,
    required this.y,
    required this.yaw,
    required this.linear,
    required this.angular,
    required this.minObstacleDist,
    required this.navState,
    required this.imu,
    required this.path,
    required this.obstacleAlert,
    required this.mission,
    required this.scan,
    required this.scanAngleMin,
    required this.scanAngleInc,
    required this.ts,
  });

  factory Telemetry.fromJson(Map<String, dynamic> j) {
    final pose = j['pose']     as Map<String, dynamic>? ?? {};
    final vel  = j['velocity'] as Map<String, dynamic>? ?? {};
    final imuJ = j['imu']     as Map<String, dynamic>?;
    final msJ  = j['mission'] as Map<String, dynamic>?;

    List<List<double>> path = [];
    if (j['path'] is List) {
      for (final p in (j['path'] as List)) {
        if (p is List && p.length >= 2) {
          path.add([(p[0] as num).toDouble(), (p[1] as num).toDouble()]);
        }
      }
    }

    List<double> scan = [];
    if (j['scan'] is List) {
      scan = (j['scan'] as List).map((v) => (v as num).toDouble()).toList();
    }

    return Telemetry(
      x:              (pose['x']   as num?)?.toDouble() ?? 0,
      y:              (pose['y']   as num?)?.toDouble() ?? 0,
      yaw:            (pose['yaw'] as num?)?.toDouble() ?? 0,
      linear:         (vel['linear']  as num?)?.toDouble() ?? 0,
      angular:        (vel['angular'] as num?)?.toDouble() ?? 0,
      minObstacleDist:(j['min_obstacle_dist'] as num?)?.toDouble(),
      navState:        j['nav_state'] as String? ?? 'UNKNOWN',
      imu:             imuJ != null ? ImuData.fromJson(imuJ) : ImuData.zero,
      path:            path,
      obstacleAlert:   j['obstacle_alert'] as String?,
      mission:         msJ != null ? MissionData.fromJson(msJ) : const MissionData(),
      scan:            scan,
      scanAngleMin:   (j['scan_angle_min'] as num?)?.toDouble() ?? -3.14159,
      scanAngleInc:   (j['scan_angle_inc'] as num?)?.toDouble() ?? 0.0174,
      ts:             (j['ts'] as num?)?.toDouble(),
    );
  }
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

class AmrBridgeService {
  final String baseUrl;

  WebSocketChannel? _channel;
  Timer? _reconnectTimer;
  final _telemetryController = StreamController<Telemetry>.broadcast();
  final _connController      = StreamController<bool>.broadcast();

  AmrBridgeService(this.baseUrl);

  Stream<Telemetry> get telemetryStream  => _telemetryController.stream;
  Stream<bool>      get connectionStream => _connController.stream;

  Uri get _httpBase => Uri.parse(baseUrl);
  String get _wsUrl => '${baseUrl.replaceFirst('http', 'ws')}/ws/telemetry';

  Future<bool> testConnection() async {
    try {
      final res = await http
          .get(_httpBase.replace(path: '/api/status'))
          .timeout(const Duration(seconds: 3));
      return res.statusCode == 200;
    } catch (_) {
      return false;
    }
  }

  void connectTelemetry() {
    _connectWs();
  }

  void _connectWs() {
    try {
      _channel = WebSocketChannel.connect(Uri.parse(_wsUrl));
      _connController.add(true);
      _channel!.stream.listen(
        (data) {
          try {
            final json = jsonDecode(data as String) as Map<String, dynamic>;
            _telemetryController.add(Telemetry.fromJson(json));
          } catch (_) {}
        },
        onError: (_) {
          _connController.add(false);
          _scheduleReconnect();
        },
        onDone: () {
          _connController.add(false);
          _scheduleReconnect();
        },
      );
    } catch (_) {
      _connController.add(false);
      _scheduleReconnect();
    }
  }

  void _scheduleReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 2), _connectWs);
  }

  void disconnectTelemetry() {
    _reconnectTimer?.cancel();
    _channel?.sink.close();
    _channel = null;
  }

  // ---- REST helpers ----

  Future<void> sendCmdVel(double linear, double angular) async {
    try {
      await http.post(
        _httpBase.replace(path: '/api/cmd_vel'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'linear': linear, 'angular': angular}),
      );
    } catch (_) {}
  }

  Future<void> sendGoal(double x, double y, {double yaw = 0}) async {
    try {
      await http.post(
        _httpBase.replace(path: '/api/goal'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': x, 'y': y, 'yaw': yaw}),
      );
    } catch (_) {}
  }

  Future<void> sendStop() async {
    try { await http.post(_httpBase.replace(path: '/api/stop')); } catch (_) {}
  }

  Future<void> clearEstop() async {
    try { await http.post(_httpBase.replace(path: '/api/estop/clear')); } catch (_) {}
  }

  Future<void> sendInitialPose(double x, double y, double theta) async {
    try {
      await http.post(
        _httpBase.replace(path: '/api/initial_pose'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': x, 'y': y, 'theta': theta}),
      );
    } catch (_) {}
  }

  Future<void> sendGoalSequence(List<String> nodes) async {
    try {
      await http.post(
        _httpBase.replace(path: '/api/goal_sequence'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'nodes': nodes}),
      );
    } catch (_) {}
  }

  Future<String?> getMapImageB64() async {
    try {
      final res = await http.get(_httpBase.replace(path: '/api/map'));
      if (res.statusCode != 200) return null;
      final j = jsonDecode(res.body) as Map<String, dynamic>;
      return j['image'] as String?;
    } catch (_) {
      return null;
    }
  }

  void dispose() {
    disconnectTelemetry();
    _telemetryController.close();
    _connController.close();
  }
}
