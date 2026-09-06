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
    this._status   = 'disconnected'; // 'connected' | 'connecting' | 'disconnected' | 'error'
    this._reconnectTimer = null;
    this._latestTelemetry = null;
  }

  get baseUrl() { return `http://${this._host}:${this._port}`; }
  get wsUrl()   { return `ws://${this._host}:${this._port}/ws/telemetry`; }

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
      clearTimeout(this._reconnectTimer);
    };

    ws.onmessage = (e) => {
      try {
        const data = JSON.parse(e.data);
        this._latestTelemetry = data;
        this._telemetryCbs.forEach(cb => cb(data));
      } catch (_) {}
    };

    ws.onerror = () => this._setStatus('error');

    ws.onclose = () => {
      this._setStatus('disconnected');
      // Auto-reconnect after 2 s
      this._reconnectTimer = setTimeout(() => this._doConnect(), 2000);
    };
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
  getLatestTelemetry() { return this._latestTelemetry; }

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
  sendGoal(x, y, yaw = 0)           { return this._post('/api/goal',    { x, y, yaw });       }
  sendStop()                         { return this._post('/api/stop');                          }
  clearEstop()                       { return this._post('/api/estop/clear');                   }
  sendInitialPose(x, y, theta = 0)  { return this._post('/api/initial_pose', { x, y, theta });}
  sendGoalSequence(nodes)            { return this._post('/api/goal_sequence', { nodes });      }

  async getMapImage()  { return this._get('/api/map');    }
  async getStatus()    { return this._get('/api/status'); }
}

const bridge = new AmrBridgeService();
export default bridge;
