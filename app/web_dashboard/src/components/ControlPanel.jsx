import { useEffect, useRef, useState } from 'react';
import nipplejs from 'nipplejs';
import bridge from '../services/amrBridge';

const MAX_LINEAR  = 0.8;
const MAX_ANGULAR = 1.8;

export default function ControlPanel({
  telemetry,
  selectedNodes = [],
  setSelectedNodes,
}) {
  const joystickRef = useRef(null);
  const managerRef  = useRef(null);
  const cmdTimerRef = useRef(null);
  const cmdRef      = useRef({ linear: 0, angular: 0 });

  // Mode Interlock: 'auto' | 'manual'
  const [controlMode, setControlMode] = useState('auto');

  // Goal & sequence states
  const [goalX, setGoalX] = useState('');
  const [goalY, setGoalY] = useState('');
  const [speed, setSpeed] = useState(0.8);
  const [estopActive, setEstopActive] = useState(false);

  // Dynamic Obstacle Simulation Panel state
  const [showObstacleControls, setShowObstacleControls] = useState(false);
  const [obsSpeed, setObsSpeed] = useState(0.45);

  const navState = telemetry?.nav_state;

  useEffect(() => {
    setEstopActive(navState === 'ESTOP');
    // If robot starts actively navigating or planning, default to auto mode
    if (navState === 'NAVIGATING' || navState === 'PLANNING') {
      setControlMode('auto');
    }
  }, [navState]);

  // ── Joystick Setup ──
  useEffect(() => {
    if (!joystickRef.current) return;

    const manager = nipplejs.create({
      zone: joystickRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: controlMode === 'manual' ? '#00f0ff' : '#4b5563',
      size: 130,
      restJoystick: true,
    });
    managerRef.current = manager;

    manager.on('move', (_, data) => {
      if (controlMode !== 'manual') return;
      if (!data.vector) return;
      const linearRaw  = data.vector.y;
      const angularRaw = -data.vector.x;
      const f = data.force ? Math.min(data.force, 1) : 1;
      cmdRef.current = {
        linear:  linearRaw  * f * MAX_LINEAR  * speed,
        angular: angularRaw * f * MAX_ANGULAR * speed,
      };
    });

    manager.on('end', () => {
      cmdRef.current = { linear: 0, angular: 0 };
      if (controlMode === 'manual') {
        bridge.sendCmdVel(0, 0);
      }
    });

    cmdTimerRef.current = setInterval(() => {
      if (controlMode !== 'manual') return;
      const { linear, angular } = cmdRef.current;
      if (linear !== 0 || angular !== 0) {
        bridge.sendCmdVel(linear, angular);
      }
    }, 100);

    return () => {
      manager.destroy();
      clearInterval(cmdTimerRef.current);
    };
  }, [speed, controlMode]);

  // ── Handlers ──
  const handleEstop = async () => {
    await bridge.sendStop();
    setEstopActive(true);
  };

  const handleEstopClear = async () => {
    await bridge.clearEstop();
    setEstopActive(false);
  };

  const handleSendGoal = async () => {
    const x = parseFloat(goalX);
    const y = parseFloat(goalY);
    if (isNaN(x) || isNaN(y)) return;
    setControlMode('auto');
    await bridge.sendGoal(x, y);
  };

  const handleInitialPose = async () => {
    const x = parseFloat(goalX);
    const y = parseFloat(goalY);
    if (isNaN(x) || isNaN(y)) return;
    await bridge.sendInitialPose(x, y, 0);
  };

  const handleRemoveNode = (index) => {
    if (setSelectedNodes) {
      setSelectedNodes(selectedNodes.filter((_, i) => i !== index));
    }
  };

  const handleClearNodes = () => {
    if (setSelectedNodes) setSelectedNodes([]);
  };

  const handleDispatchMission = async () => {
    if (selectedNodes.length === 0) return;
    setControlMode('auto');
    await bridge.sendGoalSequence(selectedNodes);
  };

  const applyPreset = (presetNodes) => {
    if (setSelectedNodes) setSelectedNodes(presetNodes);
  };

  // Obstacle teleop
  const sendObstacle = (linear, angular) => {
    bridge.sendObstacleCmdVel(linear, angular);
  };

  return (
    <div className="flex-col" style={{ gap: 12 }}>

      {/* ── 1. EMERGENCY STOP ── */}
      {estopActive ? (
        <div className="estop-banner-active">
          <div className="estop-text">⛔ EMERGENCY STOP ACTIVE</div>
          <button className="btn btn--success btn--full mt-sm" onClick={handleEstopClear}>
            ✓ Clear E-Stop &amp; Resume
          </button>
        </div>
      ) : (
        <button className="btn-estop" onClick={handleEstop}>
          ⛔ EMERGENCY STOP
        </button>
      )}

      {/* ── 2. CONTROL MODE SWITCHER (INTERLOCK) ── */}
      <div className="card">
        <div className="card__title flex-row" style={{ justifyContent: 'space-between' }}>
          <span>🕹️ Control Interlock</span>
          <span className={`badge ${controlMode === 'auto' ? 'badge--cyan' : 'badge--yellow'}`}>
            {controlMode === 'auto' ? 'AUTONOMOUS' : 'MANUAL TELEOP'}
          </span>
        </div>

        <div className="mode-toggle-group mt-sm">
          <button
            className={`btn-mode ${controlMode === 'auto' ? 'active' : ''}`}
            onClick={() => {
              setControlMode('auto');
              bridge.sendCmdVel(0, 0);
            }}
          >
            🤖 Autonomous
          </button>
          <button
            className={`btn-mode ${controlMode === 'manual' ? 'active' : ''}`}
            onClick={() => setControlMode('manual')}
          >
            🎮 Manual Drive
          </button>
        </div>

        {/* Teleop Joystick Zone */}
        <div style={{ position: 'relative', height: 160, marginTop: 12 }}>
          <div
            ref={joystickRef}
            className="joystick-zone"
            style={{
              width: '100%',
              height: '100%',
              opacity: controlMode === 'manual' ? 1 : 0.35,
              pointerEvents: controlMode === 'manual' ? 'auto' : 'none',
            }}
          />
          {controlMode === 'auto' && (
            <div className="joystick-lock-overlay">
              <span>🔒 Joystick Locked in Auto Mode</span>
              <button
                className="btn btn--xs btn--ghost mt-xs"
                onClick={() => setControlMode('manual')}
              >
                Override to Manual
              </button>
            </div>
          )}
        </div>

        {/* Speed throttle slider */}
        <div style={{ marginTop: 10 }}>
          <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 4 }}>
            <span className="text-dim text-xs">Drive Speed Throttle</span>
            <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)' }}>
              {Math.round(speed * 100)}% ({(MAX_LINEAR * speed).toFixed(2)} m/s)
            </span>
          </div>
          <input
            type="range"
            min="0.1"
            max="1.0"
            step="0.05"
            value={speed}
            disabled={controlMode !== 'manual'}
            onChange={(e) => setSpeed(Number(e.target.value))}
            style={{ width: '100%', accentColor: 'var(--accent-cyan)' }}
          />
        </div>

        {/* Manual quick actions */}
        {controlMode === 'manual' && (
          <div className="quick-actions-grid mt-sm">
            <button className="btn btn--ghost btn--sm" onClick={() => bridge.sendCmdVel(0.35 * speed, 0)}>↑ Fwd</button>
            <button className="btn btn--ghost btn--sm" onClick={() => bridge.sendCmdVel(-0.35 * speed, 0)}>↓ Rev</button>
            <button className="btn btn--ghost btn--sm" onClick={() => bridge.sendCmdVel(0, 0.9 * speed)}>↺ Left</button>
            <button className="btn btn--ghost btn--sm" onClick={() => bridge.sendCmdVel(0, -0.9 * speed)}>↻ Right</button>
            <button className="btn btn--ghost btn--sm" style={{ gridColumn: 'span 4', color: '#ff3355' }} onClick={() => bridge.sendCmdVel(0, 0)}>■ Stop Motors</button>
          </div>
        )}
      </div>

      {/* ── 3. TOPOLOGICAL MISSION BUILDER ── */}
      <div className="card">
        <div className="card__title flex-row" style={{ justifyContent: 'space-between' }}>
          <span>📋 Mission Waypoints</span>
          {selectedNodes.length > 0 && (
            <button className="btn-link text-xs" onClick={handleClearNodes}>
              Clear
            </button>
          )}
        </div>

        <div className="waypoint-queue-container mt-sm">
          {selectedNodes.length === 0 ? (
            <div className="text-dim text-xs" style={{ textAlign: 'center', padding: '12px 6px' }}>
              No waypoints selected. Turn on <b>"🗺️ Nodes"</b> on the map and click nodes, or use a preset below.
            </div>
          ) : (
            <div className="waypoint-chip-list">
              {selectedNodes.map((nodeId, idx) => (
                <span key={`${nodeId}-${idx}`} className="waypoint-chip">
                  <span className="chip-idx">{idx + 1}</span>
                  <span className="chip-name">{nodeId}</span>
                  <button
                    className="chip-remove"
                    onClick={() => handleRemoveNode(idx)}
                    title="Remove"
                  >
                    ✕
                  </button>
                </span>
              ))}
            </div>
          )}
        </div>

        {/* Mission Presets */}
        <div className="preset-row mt-sm">
          <span className="text-dim text-xs">Presets:</span>
          <button className="btn-preset" onClick={() => applyPreset(['N0', 'N5', 'N12'])}>North Aisle</button>
          <button className="btn-preset" onClick={() => applyPreset(['N3', 'N8', 'N15', 'N2'])}>Aisle Loop</button>
          <button className="btn-preset" onClick={() => applyPreset(['N1', 'N10', 'N4'])}>Docking</button>
        </div>

        <button
          className="btn btn--success btn--full mt-sm"
          disabled={selectedNodes.length === 0}
          onClick={handleDispatchMission}
        >
          ▶ Dispatch Mission Sequence ({selectedNodes.length})
        </button>
      </div>

      {/* ── 4. MANUAL COORDINATE DISPATCH ── */}
      <div className="card">
        <div className="card__title">🎯 Coordinate Dispatch</div>
        <div className="flex-row mt-sm" style={{ gap: 8 }}>
          <input
            className="input-field"
            placeholder="X (m)"
            value={goalX}
            onChange={(e) => setGoalX(e.target.value)}
          />
          <input
            className="input-field"
            placeholder="Y (m)"
            value={goalY}
            onChange={(e) => setGoalY(e.target.value)}
          />
        </div>
        <div className="flex-row mt-sm" style={{ gap: 8 }}>
          <button className="btn btn--primary btn--full" onClick={handleSendGoal}>
            ▶ Navigate
          </button>
          <button className="btn btn--ghost" onClick={handleInitialPose} title="Set AMCL Initial Pose">
            📍 Init AMCL
          </button>
        </div>
      </div>

      {/* ── 5. DYNAMIC OBSTACLE SIMULATOR TESTER ── */}
      <div className="card">
        <div
          className="card__title flex-row cursor-pointer"
          style={{ justifyContent: 'space-between' }}
          onClick={() => setShowObstacleControls(!showObstacleControls)}
        >
          <span>🚧 Dynamic Obstacle Tester</span>
          <span className="text-xs text-dim">{showObstacleControls ? '▲ Hide' : '▼ Test'}</span>
        </div>

        {showObstacleControls && (
          <div className="obstacle-tester-body mt-sm">
            <div className="text-dim text-xs" style={{ lineHeight: 1.4 }}>
              Simulate human/cart traffic to trigger AMR <b>MPPI Evasion (&lt;0.85m)</b>, <b>Corridor Yielding</b>, and <b>Dijkstra Re-routing (&gt;4.5s)</b>.
            </div>

            <div className="obstacle-pad-grid mt-sm">
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(obsSpeed, 0)}>↑ Fwd</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(-obsSpeed, 0)}>↓ Back</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(0, 0.8)}>↺ Turn L</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(0, -0.8)}>↻ Turn R</button>
              <button
                className="btn btn--ghost btn--sm"
                style={{ gridColumn: 'span 4', color: '#ff3355' }}
                onClick={() => sendObstacle(0, 0)}
              >
                ■ Stop Obstacle
              </button>
            </div>
          </div>
        )}
      </div>

    </div>
  );
}
