import { useEffect, useRef, useState } from 'react';
import nipplejs from 'nipplejs';
import bridge from '../services/amrBridge';

const MAX_LINEAR = 0.8;
const MAX_ANGULAR = 1.8;

/**
 * ControlInterlock — Manual teleop joystick & drive mode interlock component.
 * Positioned in the bottom dock beside the Autonomous Decisions & Event Log.
 */
export default function ControlInterlock({
  telemetry,
  controlMode = 'auto',
  setControlMode,
}) {
  const joystickRef = useRef(null);
  const managerRef = useRef(null);
  const cmdTimerRef = useRef(null);
  const cmdRef = useRef({ linear: 0, angular: 0 });

  const [speed, setSpeed] = useState(0.8);
  const navState = telemetry?.nav_state;

  // If robot starts actively navigating or planning, switch mode to auto
  useEffect(() => {
    if (navState === 'NAVIGATING' || navState === 'PLANNING') {
      if (setControlMode) setControlMode('auto');
    }
  }, [navState, setControlMode]);

  // NippleJS Virtual Joystick Setup
  useEffect(() => {
    if (!joystickRef.current) return;

    const manager = nipplejs.create({
      zone: joystickRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: controlMode === 'manual' ? '#00d4ff' : '#475569',
      size: 110,
      restJoystick: true,
    });
    managerRef.current = manager;

    manager.on('move', (_, data) => {
      if (controlMode !== 'manual') return;
      if (!data.vector) return;
      const linearRaw = data.vector.y;
      const angularRaw = -data.vector.x;
      const f = data.force ? Math.min(data.force, 1) : 1;
      cmdRef.current = {
        linear: linearRaw * f * MAX_LINEAR * speed,
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

  const handleSwitchMode = (mode) => {
    if (setControlMode) setControlMode(mode);
    if (mode === 'auto') {
      bridge.sendCmdVel(0, 0);
    }
  };

  const handleManualAction = (linear, angular) => {
    if (controlMode !== 'manual') return;
    bridge.sendCmdVel(linear * speed, angular * speed);
  };

  const handleStopMotors = () => {
    cmdRef.current = { linear: 0, angular: 0 };
    bridge.sendCmdVel(0, 0);
  };

  return (
    <div className="card control-interlock-card" style={{ display: 'flex', flexDirection: 'column', height: '100%', overflow: 'hidden' }}>
      {/* Header */}
      <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 8 }}>
        <span style={{ fontWeight: 700, letterSpacing: '0.06em', color: '#e2e8f0' }}>
          CONTROL INTERLOCK
        </span>
        <span className={`badge ${controlMode === 'auto' ? 'badge--cyan' : 'badge--yellow'}`}>
          {controlMode === 'auto' ? 'AUTONOMOUS' : 'MANUAL TELEOP'}
        </span>
      </div>

      {/* Main Body: Left Controls + Right Joystick */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 140px', gap: 10, flex: 1, minHeight: 0 }}>
        {/* Left Column: Mode Toggle + Speed Slider + Directional Buttons */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'space-between', gap: 6 }}>
          {/* Mode Switcher */}
          <div className="mode-toggle-group">
            <button
              type="button"
              className={`btn-mode ${controlMode === 'auto' ? 'active' : ''}`}
              onClick={() => handleSwitchMode('auto')}
            >
              Autonomous
            </button>
            <button
              type="button"
              className={`btn-mode ${controlMode === 'manual' ? 'active' : ''}`}
              onClick={() => handleSwitchMode('manual')}
            >
              Manual Drive
            </button>
          </div>

          {/* Drive Speed Throttle */}
          <div>
            <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 2 }}>
              <span className="text-dim text-xs" style={{ letterSpacing: '0.04em', textTransform: 'uppercase' }}>
                Speed Throttle
              </span>
              <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)', fontWeight: 600 }}>
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
              style={{ width: '100%', height: 4, accentColor: 'var(--accent-cyan)', cursor: controlMode === 'manual' ? 'pointer' : 'not-allowed' }}
            />
          </div>

          {/* Quick Directional Actions */}
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 4 }}>
            <button
              className="btn btn--ghost btn--sm"
              disabled={controlMode !== 'manual'}
              onClick={() => handleManualAction(0.35, 0)}
              style={{ fontSize: 10, padding: '4px 2px' }}
              title="Drive Forward"
            >
              FWD
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={controlMode !== 'manual'}
              onClick={() => handleManualAction(-0.35, 0)}
              style={{ fontSize: 10, padding: '4px 2px' }}
              title="Drive Reverse"
            >
              REV
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={controlMode !== 'manual'}
              onClick={() => handleManualAction(0, 0.9)}
              style={{ fontSize: 10, padding: '4px 2px' }}
              title="Turn Left"
            >
              TURN L
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={controlMode !== 'manual'}
              onClick={() => handleManualAction(0, -0.9)}
              style={{ fontSize: 10, padding: '4px 2px' }}
              title="Turn Right"
            >
              TURN R
            </button>
          </div>

          {/* Stop Motors button */}
          <button
            className="btn btn--sm"
            onClick={handleStopMotors}
            disabled={controlMode !== 'manual'}
            style={{
              padding: '4px',
              fontSize: 10,
              fontWeight: 700,
              background: 'rgba(239, 68, 68, 0.15)',
              border: '1px solid rgba(239, 68, 68, 0.4)',
              color: '#f87171',
              cursor: controlMode === 'manual' ? 'pointer' : 'not-allowed',
            }}
          >
            STOP MOTORS
          </button>
        </div>

        {/* Right Column: Virtual Joystick Zone */}
        <div style={{ position: 'relative', width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <div
            ref={joystickRef}
            className="joystick-zone"
            style={{
              width: 125,
              height: 125,
              opacity: controlMode === 'manual' ? 1 : 0.25,
              pointerEvents: controlMode === 'manual' ? 'auto' : 'none',
              transition: 'opacity 0.2s ease',
            }}
          />
          {controlMode === 'auto' && (
            <div className="joystick-lock-overlay" style={{ padding: 6, gap: 4 }}>
              <span style={{ fontSize: 9, fontWeight: 600, color: '#94a3b8' }}>
                LOCKED (AUTO)
              </span>
              <button
                className="btn btn--xs btn--ghost"
                onClick={() => handleSwitchMode('manual')}
                style={{ fontSize: 9, padding: '2px 6px', fontWeight: 600 }}
              >
                Enable Manual
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
