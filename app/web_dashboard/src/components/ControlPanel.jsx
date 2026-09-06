import { useEffect, useRef, useState } from 'react';
import nipplejs from 'nipplejs';
import bridge from '../services/amrBridge';

const MAX_LINEAR  = 0.8;
const MAX_ANGULAR = 1.8;

/**
 * ControlPanel — right sidebar with:
 *  - Virtual joystick (nipplejs) → continuous cmd_vel
 *  - E-STOP and clear buttons
 *  - Manual goal input (X, Y coordinates)
 *  - Goal sequence input (comma-separated node IDs)
 *  - Quick preset actions
 */
export default function ControlPanel({ telemetry }) {
  const joystickRef = useRef(null);
  const managerRef  = useRef(null);
  const cmdTimerRef = useRef(null);
  const cmdRef      = useRef({ linear: 0, angular: 0 });

  const [goalX, setGoalX]   = useState('');
  const [goalY, setGoalY]   = useState('');
  const [seqInput, setSeqInput] = useState('');
  const [speed, setSpeed]   = useState(1.0); // velocity scale factor
  const [estopActive, setEstopActive] = useState(false);

  const navState = telemetry?.nav_state;
  useEffect(() => {
    setEstopActive(navState === 'ESTOP');
  }, [navState]);

  // ---- Joystick ----
  useEffect(() => {
    if (!joystickRef.current) return;
    const manager = nipplejs.create({
      zone:   joystickRef.current,
      mode:   'static',
      position: { left: '50%', top: '50%' },
      color:  '#00d4ff',
      size:   130,
      restJoystick: true,
    });
    managerRef.current = manager;

    manager.on('move', (_, data) => {
      if (!data.vector) return;
      // nipplejs: vector.x = right (+), vector.y = up (+)
      const linearRaw  = data.vector.y;  // forward
      const angularRaw = -data.vector.x; // left turn = positive
      const f = data.force ? Math.min(data.force, 1) : 1;
      cmdRef.current = {
        linear:  linearRaw  * f * MAX_LINEAR  * speed,
        angular: angularRaw * f * MAX_ANGULAR * speed,
      };
    });

    manager.on('end', () => {
      cmdRef.current = { linear: 0, angular: 0 };
      bridge.sendCmdVel(0, 0);
    });

    // Send cmd_vel at 10 Hz while joystick active
    cmdTimerRef.current = setInterval(() => {
      const { linear, angular } = cmdRef.current;
      if (linear !== 0 || angular !== 0) {
        bridge.sendCmdVel(linear, angular);
      }
    }, 100);

    return () => {
      manager.destroy();
      clearInterval(cmdTimerRef.current);
    };
  }, [speed]);

  // ---- Handlers ----

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
    await bridge.sendGoal(x, y);
  };

  const handleSendSequence = async () => {
    const nodes = seqInput.split(',').map(s => s.trim()).filter(Boolean);
    if (nodes.length === 0) return;
    await bridge.sendGoalSequence(nodes);
  };

  const handleInitialPose = async () => {
    const x = parseFloat(goalX);
    const y = parseFloat(goalY);
    if (isNaN(x) || isNaN(y)) return;
    await bridge.sendInitialPose(x, y, 0);
  };

  return (
    <div className="flex-col" style={{ gap: 12 }}>

      {/* E-STOP */}
      {estopActive ? (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
          <div style={{
            padding: '12px', borderRadius: 10, textAlign: 'center',
            background: 'rgba(255,51,85,0.1)', border: '1px solid var(--accent-red)',
            color: 'var(--accent-red)', fontWeight: 700, fontSize: 13,
            letterSpacing: '0.05em'
          }}>⛔ E-STOP ACTIVE</div>
          <button className="btn btn--ghost btn--full" onClick={handleEstopClear}>
            ✓ Clear E-Stop &amp; Resume
          </button>
        </div>
      ) : (
        <button className="btn-estop" onClick={handleEstop}>
          ⛔ EMERGENCY STOP
        </button>
      )}

      {/* Joystick */}
      <div className="card">
        <div className="card__title">🕹 Teleop Joystick</div>
        <div style={{ position: 'relative', height: 160, marginTop: 8 }}>
          <div ref={joystickRef} className="joystick-zone"
            style={{ width: '100%', height: '100%' }} />
        </div>
        {/* Speed scale */}
        <div style={{ marginTop: 12 }}>
          <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 4 }}>
            <span className="text-dim text-xs">Speed</span>
            <span className="text-mono" style={{ fontSize: 12, color: 'var(--accent-cyan)' }}>
              {Math.round(speed * 100)}%
            </span>
          </div>
          <input
            type="range" min="0.1" max="1" step="0.05"
            value={speed}
            onChange={e => setSpeed(Number(e.target.value))}
            style={{ width: '100%', accentColor: 'var(--accent-cyan)' }}
          />
        </div>
        <button className="btn btn--ghost btn--full mt-sm"
          onClick={() => bridge.sendCmdVel(0, 0)}>
          ■ Stop
        </button>
      </div>

      {/* Goal input */}
      <div className="card">
        <div className="card__title">🎯 Send Goal</div>
        <div className="flex-row mt-sm">
          <input className="input-field" placeholder="X (m)"
            value={goalX} onChange={e => setGoalX(e.target.value)} />
          <input className="input-field" placeholder="Y (m)"
            value={goalY} onChange={e => setGoalY(e.target.value)} />
        </div>
        <div className="flex-row mt-sm">
          <button className="btn btn--primary btn--full" onClick={handleSendGoal}>▶ Navigate</button>
          <button className="btn btn--ghost" onClick={handleInitialPose} title="Set AMCL initial pose">📍 Init</button>
        </div>
      </div>

      {/* Goal sequence */}
      <div className="card">
        <div className="card__title">📋 Goal Sequence</div>
        <input
          className="input-field mt-sm"
          placeholder="N5, N12, N40, …"
          value={seqInput}
          onChange={e => setSeqInput(e.target.value)}
        />
        <button className="btn btn--success btn--full mt-sm" onClick={handleSendSequence}>
          ▶ Run Sequence
        </button>
      </div>

      {/* Quick actions */}
      <div className="card">
        <div className="card__title">⚡ Quick Actions</div>
        <div className="flex-col mt-sm" style={{ gap: 6 }}>
          <button className="btn btn--ghost btn--full"
            onClick={() => bridge.sendCmdVel(0.3, 0)}>
            ↑ Forward (slow)
          </button>
          <button className="btn btn--ghost btn--full"
            onClick={() => bridge.sendCmdVel(-0.3, 0)}>
            ↓ Reverse (slow)
          </button>
          <button className="btn btn--ghost btn--full"
            onClick={() => bridge.sendCmdVel(0, 0.8)}>
            ↺ Rotate Left
          </button>
          <button className="btn btn--ghost btn--full"
            onClick={() => bridge.sendCmdVel(0, -0.8)}>
            ↻ Rotate Right
          </button>
        </div>
      </div>

    </div>
  );
}
