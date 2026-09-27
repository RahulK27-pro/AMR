/**
 * amrBridge.js — WebSocket + REST service for the AMR Control Dashboard
 *
 * Usage:
 *   import bridge from './amrBridge';
 *   bridge.connect('192.168.1.100', 8000);
 *   bridge.onTelemetry((data) => { ... });
 *   bridge.sendGoal(x, y);
 *   bridge.sendCmdVel(linear, angular);
 *   bridge.sendStop();
 */

const DEFAULT_HOST = window.location.hostname;
const DEFAULT_PORT = 8000;

class AmrBridgeService {
  constructor() {
    this._host     = DEFAULT_HOST;
    this._port     = DEFAULT_PORT;
    this._ws       = null;
    this._telemetryCbs = [];
    this._statusCbs    = [];
    this._eventCbs     = [];
    this._status   = 'disconnected'; // 'connected' | 'connecting' | 'disconnected' | 'error'
    this._reconnectTimer = null;
    this._latestTelemetry = null;
    this._lastNavState = null;
    this._lastAlert = null;
    this._lastWaypoint = null;
  }

  get baseUrl()   { return `http://${this._host}:${this._port}`; }
  get wsUrl()     { return `ws://${this._host}:${this._port}/ws/telemetry`; }
  get rawMapUrl() { return `${this.baseUrl}/api/map/raw`; }

  // ---- Connection management ----

  connect(host = DEFAULT_HOST, port = DEFAULT_PORT) {
    this._host = host;
    this._port = port;
    this._doConnect();
  }

  _doConnect() {
    this._setStatus('connecting');
    if (this._ws) {
      this._ws.onclose = null;
      this._ws.close();
    }
    const ws = new WebSocket(this.wsUrl);
    this._ws = ws;

    ws.onopen = () => {
      this._setStatus('connected');
      this.emitEvent('info', `Connected to AMR Bridge at ${this._host}:${this._port}`);
      clearTimeout(this._reconnectTimer);
    };

    ws.onmessage = (e) => {
      try {
        const data = JSON.parse(e.data);
        this._handleTelemetry(data);
      } catch (_) {}
    };

    ws.onerror = () => this._setStatus('error');

    ws.onclose = () => {
      this._setStatus('disconnected');
      // Auto-reconnect after 2 s
      this._reconnectTimer = setTimeout(() => this._doConnect(), 2000);
    };
  }

  _handleTelemetry(data) {
    this._latestTelemetry = data;
    this._telemetryCbs.forEach(cb => cb(data));

    // Nav state transition detection
    if (data.nav_state && data.nav_state !== this._lastNavState) {
      const prev = this._lastNavState;
      this._lastNavState = data.nav_state;
      if (prev !== null) {
        const level = data.nav_state === 'ESTOP' ? 'critical'
                    : data.nav_state === 'YIELDING' ? 'warn'
                    : data.nav_state === 'ARRIVED' ? 'success'
                    : 'info';
        this.emitEvent(level, `Navigation State: ${data.nav_state} (from ${prev})`);
      }
    }

    // Obstacle alert detection
    if (data.obstacle_alert && data.obstacle_alert !== this._lastAlert) {
      this._lastAlert = data.obstacle_alert;
      this.emitEvent('warn', `Obstacle Alert: ${data.obstacle_alert}`);
    } else if (!data.obstacle_alert && this._lastAlert) {
      this._lastAlert = null;
    }

    // Mission waypoint tracking
    if (data.mission?.current != null && data.mission.current !== this._lastWaypoint) {
      this._lastWaypoint = data.mission.current;
      if (data.mission.total > 0) {
        this.emitEvent('info', `Mission Progress: Waypoint ${data.mission.current} of ${data.mission.total} (${data.mission.goal_node || 'target'})`);
      }
    }
  }

  disconnect() {
    clearTimeout(this._reconnectTimer);
    if (this._ws) { this._ws.onclose = null; this._ws.close(); }
    this._setStatus('disconnected');
  }

  _setStatus(s) {
    this._status = s;
    this._statusCbs.forEach(cb => cb(s));
  }

  // ---- Event subscriptions ----

  onTelemetry(cb)      { this._telemetryCbs.push(cb); return () => { this._telemetryCbs = this._telemetryCbs.filter(x => x !== cb); }; }
  onStatusChange(cb)   { this._statusCbs.push(cb);    return () => { this._statusCbs    = this._statusCbs.filter(x => x !== cb); }; }
  onEvent(cb)          { this._eventCbs.push(cb);     return () => { this._eventCbs     = this._eventCbs.filter(x => x !== cb); }; }
  getLatestTelemetry() { return this._latestTelemetry; }

  emitEvent(level, message) {
    const evt = {
      id: Date.now() + Math.random().toString(36).substring(2, 6),
      time: new Date().toLocaleTimeString(),
      timestamp: Date.now(),
      level, // 'info' | 'warn' | 'critical' | 'success'
      message,
    };
    this._eventCbs.forEach(cb => cb(evt));
  }

  // ---- REST helpers ----

  async _post(path, body = {}) {
    try {
      const res = await fetch(`${this.baseUrl}${path}`, {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify(body),
      });
      return res.ok;
    } catch (_) { return false; }
  }

  async _get(path) {
    try {
      const res = await fetch(`${this.baseUrl}${path}`);
      if (!res.ok) return null;
      return res.json();
    } catch (_) { return null; }
  }

  // ---- Commands ----

  sendCmdVel(linear, angular)       { return this._post('/api/cmd_vel', { linear, angular }); }
  sendGoal(x, y, yaw = 0) {
    this.emitEvent('info', `Dispatched Single Goal → (${x.toFixed(2)}, ${y.toFixed(2)}) m`);
    return this._post('/api/goal', { x, y, yaw });
  }
  sendStop() {
    this.emitEvent('critical', 'EMERGENCY STOP Triggered');
    return this._post('/api/stop');
  }
  clearEstop() {
    this.emitEvent('info', 'E-STOP Cleared — Resuming Operation');
    return this._post('/api/estop/clear');
  }
  sendInitialPose(x, y, theta = 0) {
    this.emitEvent('info', `Set AMCL Initial Pose → (${x.toFixed(2)}, ${y.toFixed(2)}) θ=${theta.toFixed(2)}rad`);
    return this._post('/api/initial_pose', { x, y, theta });
  }
  sendGoalSequence(nodes) {
    this.emitEvent('info', `Dispatched Mission Queue [${nodes.join(' → ')}]`);
    return this._post('/api/goal_sequence', { nodes });
  }
  sendObstacleCmdVel(linear, angular) {
    return this._post('/api/obstacle/cmd_vel', { linear, angular });
  }

  async getMapImage()    { return this._get('/api/map'); }
  async getMapMetadata() { return this._get('/api/map/metadata'); }
  async getGraphData()   { return this._get('/api/graph'); }
  async getStatus()      { return this._get('/api/status'); }

  // ---- Mapping Pipeline (Phase 1) ----
  async getMappingStatus() { return this._get('/api/mapping/status'); }
  async startMapping(mapName = 'warehouse_01', world = 'test1.world', runExplore = false) {
    this.emitEvent('info', `Started mapping pipeline: "${mapName}" (world: ${world}, explore: ${runExplore ? 'autonomous' : 'manual'})`);
    return this._post('/api/mapping/start', { map_name: mapName, world, run_explore: runExplore });
  }
  async stopMapping() {
    this.emitEvent('warn', 'Stopped mapping session');
    return this._post('/api/mapping/stop');
  }
  async redoMapping() {
    this.emitEvent('info', 'Reset mapping session (Redo)');
    return this._post('/api/mapping/redo');
  }
  async startAutoExplore() {
    this.emitEvent('info', 'Started autonomous frontier exploration');
    return this._post('/api/mapping/explore/start');
  }
  async pauseAutoExplore() {
    this.emitEvent('warn', 'Paused autonomous exploration');
    return this._post('/api/mapping/explore/pause');
  }
  async resumeAutoExplore() {
    this.emitEvent('info', 'Resumed autonomous exploration');
    return this._post('/api/mapping/explore/resume');
  }
  async getLiveMap() { return this._get('/api/mapping/live_map'); }
}

const bridge = new AmrBridgeService();
export default bridge;
