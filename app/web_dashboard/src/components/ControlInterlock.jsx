import { useEffect, useRef, useState } from 'react';
import nipplejs from 'nipplejs';
import bridge from '../services/amrBridge';

const SIM_MAX_LINEAR = 0.8;
const SIM_MAX_ANGULAR = 1.8;

// Physical Hardware Teleop Limits (1:1 parity with teleop_wasd.py / ./teleop_hardware.sh)
const HW_MIN_LINEAR = 0.04;
const HW_MAX_LINEAR = 0.45;
const HW_DEF_LINEAR = 0.18;
const HW_STEP_LINEAR = 0.02;

const HW_MIN_ANGULAR = 0.20;
const HW_MAX_ANGULAR = 2.00;
const HW_DEF_ANGULAR = 0.80;
const HW_STEP_ANGULAR = 0.10;

const HW_PRESETS = [
  { key: '1', label: 'Slow', linear: 0.10, angular: 0.50 },
  { key: '2', label: 'Normal', linear: 0.20, angular: 0.80 },
  { key: '3', label: 'Fast', linear: 0.35, angular: 1.40 },
];

const MAX_OBS_LINEAR = 0.8;
const MAX_OBS_ANGULAR = 1.6;

/**
 * ControlInterlock — Unified Manual Teleop Joystick & Drive Mode Interlock component.
 * Supports both Simulation (throttle 0.1-1.0 up to 0.8 m/s + Dyn Obstacle) and
 * Real Hardware (variable speeds matching teleop_wasd.py: 0.04-0.45 m/s, presets, +/- steps).
 */
export default function ControlInterlock({
  telemetry,
  controlMode = 'auto',
  setControlMode,
  mode = 'sim',
}) {
  const joystickRef = useRef(null);
  const managerRef = useRef(null);
  const cmdTimerRef = useRef(null);
  const cmdRef = useRef({ linear: 0, angular: 0 });

  // Control target: 'robot' (AMR) | 'obstacle' (Dynamic Obstacle, sim only)
  const [teleopTarget, setTeleopTarget] = useState('robot');

  // Simulation drive speed throttle (0.1 - 1.0)
  const [robotSpeed, setRobotSpeed] = useState(0.8);
  // Hardware variable speeds matching teleop_wasd.py
  const [hwLinear, setHwLinear] = useState(HW_DEF_LINEAR);
  const [hwAngular, setHwAngular] = useState(HW_DEF_ANGULAR);
  // Obstacle steering speed throttle (0.1 - 1.0)
  const [obsSpeed, setObsSpeed] = useState(0.5);

  const navState = telemetry?.nav_state;
  const isEstopActive = navState === 'ESTOP' || Boolean(telemetry?.estop_active);
  const dynObs = telemetry?.dynamic_obstacle;
  const isHardware = mode === 'hardware';

  const activeMaxLinear = isHardware ? hwLinear : SIM_MAX_LINEAR * robotSpeed;
  const activeMaxAngular = isHardware ? hwAngular : SIM_MAX_ANGULAR * robotSpeed;

  useEffect(() => {
    if (isHardware && teleopTarget === 'obstacle') {
      setTeleopTarget('robot');
    }
  }, [isHardware, teleopTarget]);

  useEffect(() => {
    if (isEstopActive) {
      cmdRef.current = { linear: 0, angular: 0 };
    }
  }, [isEstopActive]);

  // External listener to switch teleop target (e.g. from ControlPanel)
  useEffect(() => {
    const handleTargetSwitch = (e) => {
      if (e?.detail?.target && !(isHardware && e.detail.target === 'obstacle')) {
        setTeleopTarget(e.detail.target);
      }
    };
    window.addEventListener('amr-select-teleop-target', handleTargetSwitch);
    return () => window.removeEventListener('amr-select-teleop-target', handleTargetSwitch);
  }, [isHardware]);

  // If robot starts actively navigating or planning, switch robot mode to auto
  useEffect(() => {
    if (navState === 'NAVIGATING' || navState === 'PLANNING') {
      if (setControlMode) setControlMode('auto');
    }
  }, [navState, setControlMode]);

  // NippleJS Virtual Joystick Setup (Dynamic target & color)
  useEffect(() => {
    if (!joystickRef.current) return;

    const isObstacle = !isHardware && teleopTarget === 'obstacle';
    const isRobotManual = teleopTarget === 'robot' && controlMode === 'manual' && !isEstopActive;
    const isInteractive = isObstacle || isRobotManual;

    const isLight = document.documentElement.getAttribute('data-theme') === 'light';
    const joyColor = isEstopActive
      ? '#ef4444'
      : isObstacle
        ? '#ea580c'
        : isRobotManual
          ? (isLight ? '#0284c7' : '#00d4ff')
          : (isLight ? '#94a3b8' : '#475569');

    const manager = nipplejs.create({
      zone: joystickRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: joyColor,
      size: 110,
      restJoystick: true,
    });
    managerRef.current = manager;

    manager.on('move', (_, data) => {
      if (!isInteractive || !data.vector) return;
      const linearRaw = data.vector.y;
      const angularRaw = -data.vector.x;
      const f = data.force ? Math.min(data.force, 1) : 1;

      if (isObstacle) {
        cmdRef.current = {
          linear: linearRaw * f * MAX_OBS_LINEAR * obsSpeed,
          angular: angularRaw * f * MAX_OBS_ANGULAR * obsSpeed,
        };
      } else {
        cmdRef.current = {
          linear: linearRaw * f * activeMaxLinear,
          angular: angularRaw * f * activeMaxAngular,
        };
      }
    });

    manager.on('end', () => {
      cmdRef.current = { linear: 0, angular: 0 };
      if (isObstacle) {
        bridge.sendObstacleCmdVel(0, 0);
      } else if (isRobotManual) {
        bridge.sendCmdVel(0, 0);
      }
    });

    cmdTimerRef.current = setInterval(() => {
      if (!isInteractive) return;
      const { linear, angular } = cmdRef.current;
      if (linear !== 0 || angular !== 0) {
        if (isObstacle) {
          bridge.sendObstacleCmdVel(linear, angular);
        } else if (isRobotManual) {
          bridge.sendCmdVel(linear, angular);
        }
      }
    }, 100);

    return () => {
      manager.destroy();
      clearInterval(cmdTimerRef.current);
    };
  }, [teleopTarget, controlMode, robotSpeed, hwLinear, hwAngular, obsSpeed, isHardware, isEstopActive, activeMaxLinear, activeMaxAngular]);

  // Keyboard navigation listener (W/A/S/D, Arrow keys, Q/E curve, +/- speed, 1/2/3 presets)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName)) return;
      const isObstacle = !isHardware && teleopTarget === 'obstacle';
      const isRobotManual = teleopTarget === 'robot' && controlMode === 'manual' && !isEstopActive;
      if (!isObstacle && !isRobotManual) return;

      if (isHardware && !isObstacle) {
        if (e.key === '+' || e.key === '=') {
          setHwLinear((v) => Math.min(HW_MAX_LINEAR, Number((v + HW_STEP_LINEAR).toFixed(2))));
          setHwAngular((w) => Math.min(HW_MAX_ANGULAR, Number((w + HW_STEP_ANGULAR).toFixed(2))));
          return;
        }
        if (e.key === '-' || e.key === '_') {
          setHwLinear((v) => Math.max(HW_MIN_LINEAR, Number((v - HW_STEP_LINEAR).toFixed(2))));
          setHwAngular((w) => Math.max(HW_MIN_ANGULAR, Number((w - HW_STEP_ANGULAR).toFixed(2))));
          return;
        }
        const preset = HW_PRESETS.find((p) => p.key === e.key);
        if (preset) {
          setHwLinear(preset.linear);
          setHwAngular(preset.angular);
          return;
        }
      }

      const spd = isObstacle ? obsSpeed * MAX_OBS_LINEAR : activeMaxLinear;
      const ang = isObstacle ? obsSpeed * MAX_OBS_ANGULAR : activeMaxAngular;

      if (e.key === 'ArrowUp' || e.key === 'w' || e.key === 'W') {
        cmdRef.current = { linear: spd, angular: 0 };
      } else if (e.key === 'ArrowDown' || e.key === 's' || e.key === 'S') {
        cmdRef.current = { linear: -spd, angular: 0 };
      } else if (e.key === 'ArrowLeft' || e.key === 'a' || e.key === 'A') {
        cmdRef.current = { linear: 0, angular: ang };
      } else if (e.key === 'ArrowRight' || e.key === 'd' || e.key === 'D') {
        cmdRef.current = { linear: 0, angular: -ang };
      } else if (e.key === 'q' || e.key === 'Q') {
        cmdRef.current = { linear: spd, angular: ang * 0.7 };
      } else if (e.key === 'e' || e.key === 'E') {
        cmdRef.current = { linear: spd, angular: -ang * 0.7 };
      } else if (e.key === ' ' || e.key === 'x' || e.key === 'X') {
        cmdRef.current = { linear: 0, angular: 0 };
        if (isObstacle) bridge.sendObstacleCmdVel(0, 0);
        else bridge.sendCmdVel(0, 0);
      }
    };

    const handleKeyUp = (e) => {
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement?.tagName)) return;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'q', 'e', 'W', 'A', 'S', 'D', 'Q', 'E'].includes(e.key)) {
        cmdRef.current = { linear: 0, angular: 0 };
        if (!isHardware && teleopTarget === 'obstacle') {
          bridge.sendObstacleCmdVel(0, 0);
        } else if (controlMode === 'manual' && !isEstopActive) {
          bridge.sendCmdVel(0, 0);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [teleopTarget, controlMode, robotSpeed, obsSpeed, isHardware, isEstopActive, activeMaxLinear, activeMaxAngular]);

  const handleSwitchRobotMode = (nextMode) => {
    if (setControlMode) setControlMode(nextMode);
    if (nextMode === 'auto') {
      bridge.sendCmdVel(0, 0);
    }
  };

  const handleManualAction = (linScale, angScale) => {
    if (!isHardware && teleopTarget === 'obstacle') {
      bridge.sendObstacleCmdVel(linScale * obsSpeed, angScale * obsSpeed);
    } else {
      if (controlMode !== 'manual' || isEstopActive) return;
      if (isHardware) {
        const linCmd = linScale > 0 ? hwLinear : linScale < 0 ? -hwLinear : 0;
        const angCmd = angScale > 0 ? hwAngular : angScale < 0 ? -hwAngular : 0;
        bridge.sendCmdVel(linCmd, angCmd);
      } else {
        bridge.sendCmdVel(linScale * robotSpeed, angScale * robotSpeed);
      }
    }
  };

  const handleStopMotors = () => {
    cmdRef.current = { linear: 0, angular: 0 };
    if (!isHardware && teleopTarget === 'obstacle') {
      bridge.sendObstacleCmdVel(0, 0);
    } else {
      bridge.sendCmdVel(0, 0);
    }
  };

  const handleTriggerDropObstacle = () => {
    window.dispatchEvent(new CustomEvent('amr-trigger-drop-obstacle'));
  };

  const handleRemoveObstacle = async () => {
    await bridge.removeObstacle();
  };

  const isObstacleMode = !isHardware && teleopTarget === 'obstacle';
  const obsDist = dynObs?.dist_to_robot;
  const isEvading = dynObs?.avoidance_status === 'TACTICAL_EVASION' || (obsDist != null && obsDist < 0.85);
  const isProximity = dynObs?.avoidance_status === 'PROXIMITY_SLOWDOWN' || (obsDist != null && obsDist < 1.5 && !isEvading);

  return (
    <div
      className="card control-interlock-card"
      style={{
        display: 'flex',
        flexDirection: 'column',
        height: '100%',
        overflow: 'hidden',
        border: isEstopActive
          ? '1px solid #ef4444'
          : isObstacleMode
            ? '1px solid rgba(234, 88, 12, 0.45)'
            : '1px solid var(--border-subtle)',
        boxShadow: isObstacleMode ? '0 0 16px rgba(234, 88, 12, 0.12)' : 'none',
        transition: 'border-color 0.2s ease, box-shadow 0.2s ease',
      }}
    >
      {/* Top Header Bar & Teleop Target Switcher Tabs */}
      <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
        <div className="flex-row" style={{ gap: 6, alignItems: 'center' }}>
          <button
            type="button"
            className={`btn btn--xs ${!isObstacleMode ? 'btn--primary' : 'btn--ghost'}`}
            onClick={() => setTeleopTarget('robot')}
            style={{ fontWeight: 700, padding: '3px 8px', fontSize: 11 }}
          >
            {isHardware ? 'HW TELEOP' : 'ROBOT DRIVE'}
          </button>
          {!isHardware && (
            <button
              type="button"
              className={`btn btn--xs ${isObstacleMode ? 'btn--warning' : 'btn--ghost'}`}
              onClick={() => setTeleopTarget('obstacle')}
              style={{
                fontWeight: 700,
                padding: '3px 8px',
                fontSize: 11,
                background: isObstacleMode ? '#ea580c' : 'transparent',
                color: isObstacleMode ? '#ffffff' : 'var(--text-secondary)',
                borderColor: isObstacleMode ? '#f97316' : 'var(--border-subtle)',
              }}
            >
              🚧 DYN OBSTACLE
            </button>
          )}
        </div>

        {/* Status Badge */}
        {isEstopActive ? (
          <span className="badge badge--fail" style={{ fontWeight: 700 }}>
            E-STOP LOCKED
          </span>
        ) : !isObstacleMode ? (
          <span className={`badge ${controlMode === 'auto' ? 'badge--cyan' : 'badge--yellow'}`}>
            {controlMode === 'auto' ? 'AUTONOMOUS' : 'MANUAL TELEOP'}
          </span>
        ) : (
          <span
            className="badge"
            style={{
              background: isEvading ? 'rgba(239, 68, 68, 0.2)' : isProximity ? 'rgba(249, 115, 22, 0.2)' : 'rgba(16, 185, 129, 0.15)',
              color: isEvading ? '#f87171' : isProximity ? '#fb923c' : '#34d399',
              border: `1px solid ${isEvading ? '#ef4444' : isProximity ? '#ea580c' : '#059669'}`,
              fontWeight: 700,
            }}
          >
            {isEvading ? 'EVASION ACTIVE' : isProximity ? 'PROXIMITY ALERT' : dynObs?.active ? 'TRACKED & ACTIVE' : 'OBSTACLE READY'}
          </span>
        )}
      </div>

      {/* Main Body: Left Controls + Right Joystick */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 140px', gap: 10, flex: 1, minHeight: 0 }}>
        {/* Left Column */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'space-between', gap: 5 }}>
          {!isObstacleMode ? (
            /* Robot Controls View */
            <>
              {/* Mode Switcher */}
              <div className="mode-toggle-group">
                <button
                  type="button"
                  className={`btn-mode ${controlMode === 'auto' ? 'active' : ''}`}
                  onClick={() => handleSwitchRobotMode('auto')}
                >
                  Autonomous
                </button>
                <button
                  type="button"
                  className={`btn-mode ${controlMode === 'manual' ? 'active' : ''}`}
                  onClick={() => handleSwitchRobotMode('manual')}
                >
                  Manual Drive
                </button>
              </div>

              {isHardware ? (
                /* Hardware Variable Speeds matching teleop_wasd.py */
                <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <span className="text-dim text-xs" style={{ textTransform: 'uppercase', fontSize: 10 }}>
                      Linear / Turn
                    </span>
                    <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)', fontWeight: 700 }}>
                      {hwLinear.toFixed(2)} m/s · {hwAngular.toFixed(2)} rad/s
                    </span>
                  </div>
                  <div className="flex-row" style={{ gap: 6, alignItems: 'center' }}>
                    <button
                      type="button"
                      className="btn btn--xs btn--ghost"
                      disabled={controlMode !== 'manual' || isEstopActive}
                      onClick={() => {
                        setHwLinear((v) => Math.max(HW_MIN_LINEAR, Number((v - HW_STEP_LINEAR).toFixed(2))));
                        setHwAngular((w) => Math.max(HW_MIN_ANGULAR, Number((w - HW_STEP_ANGULAR).toFixed(2))));
                      }}
                      style={{ padding: '2px 7px', fontWeight: 700 }}
                      title="Decrease speed (-0.02 m/s)"
                    >
                      −
                    </button>
                    <input
                      type="range"
                      min={HW_MIN_LINEAR}
                      max={HW_MAX_LINEAR}
                      step={HW_STEP_LINEAR}
                      value={hwLinear}
                      disabled={controlMode !== 'manual' || isEstopActive}
                      onChange={(e) => {
                        const v = Number(e.target.value);
                        setHwLinear(v);
                        const ratio = (v - HW_MIN_LINEAR) / (HW_MAX_LINEAR - HW_MIN_LINEAR);
                        setHwAngular(Number((HW_MIN_ANGULAR + ratio * (HW_MAX_ANGULAR - HW_MIN_ANGULAR)).toFixed(2)));
                      }}
                      style={{ flex: 1, height: 4, accentColor: 'var(--accent-cyan)', cursor: controlMode === 'manual' ? 'pointer' : 'not-allowed' }}
                    />
                    <button
                      type="button"
                      className="btn btn--xs btn--ghost"
                      disabled={controlMode !== 'manual' || isEstopActive}
                      onClick={() => {
                        setHwLinear((v) => Math.min(HW_MAX_LINEAR, Number((v + HW_STEP_LINEAR).toFixed(2))));
                        setHwAngular((w) => Math.min(HW_MAX_ANGULAR, Number((w + HW_STEP_ANGULAR).toFixed(2))));
                      }}
                      style={{ padding: '2px 7px', fontWeight: 700 }}
                      title="Increase speed (+0.02 m/s)"
                    >
                      +
                    </button>
                  </div>
                  <div className="flex-row" style={{ gap: 4 }}>
                    {HW_PRESETS.map((p) => (
                      <button
                        key={p.key}
                        type="button"
                        disabled={controlMode !== 'manual' || isEstopActive}
                        className={`btn-preset ${Math.abs(hwLinear - p.linear) < 0.005 ? 'btn-preset--active' : ''}`}
                        onClick={() => {
                          setHwLinear(p.linear);
                          setHwAngular(p.angular);
                        }}
                        style={{ flex: 1, fontSize: 10, padding: '2px 4px' }}
                        title={`Preset [${p.key}]: ${p.linear} m/s, ${p.angular} rad/s`}
                      >
                        {p.label} ({p.linear.toFixed(2)})
                      </button>
                    ))}
                  </div>
                </div>
              ) : (
                /* Simulation Speed Throttle */
                <div>
                  <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 2 }}>
                    <span className="text-dim text-xs" style={{ letterSpacing: '0.04em', textTransform: 'uppercase' }}>
                      Speed Throttle
                    </span>
                    <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)', fontWeight: 600 }}>
                      {Math.round(robotSpeed * 100)}% ({(SIM_MAX_LINEAR * robotSpeed).toFixed(2)} m/s)
                    </span>
                  </div>
                  <input
                    type="range"
                    min="0.1"
                    max="1.0"
                    step="0.05"
                    value={robotSpeed}
                    disabled={controlMode !== 'manual' || isEstopActive}
                    onChange={(e) => setRobotSpeed(Number(e.target.value))}
                    style={{ width: '100%', height: 4, accentColor: 'var(--accent-cyan)', cursor: controlMode === 'manual' ? 'pointer' : 'not-allowed' }}
                  />
                </div>
              )}
            </>
          ) : (
            /* Obstacle Steering View (Simulation only) */
            <>
              <div
                style={{
                  background: 'var(--bg-elevated)',
                  padding: '5px 8px',
                  borderRadius: 6,
                  border: '1px solid var(--border)',
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                }}
              >
                <div style={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
                  <span className="text-xs" style={{ fontWeight: 600, color: 'var(--text-primary)' }}>
                    {dynObs?.x != null ? `Pos: (${dynObs.x.toFixed(2)}, ${dynObs.y.toFixed(2)})` : 'Pos: Not Dropped'}
                  </span>
                  <span className="text-dim text-xs">
                    {obsDist != null ? `Clearance: ${obsDist.toFixed(2)}m` : 'Clearance: —'}
                  </span>
                </div>
                <div className="flex-row" style={{ gap: 4 }}>
                  <button
                    className="btn btn--xs btn--secondary"
                    onClick={handleTriggerDropObstacle}
                    title="Drop on Map Canvas"
                    style={{ fontSize: 10, padding: '2px 6px', fontWeight: 600 }}
                  >
                    🎯 Drop
                  </button>
                  {dynObs?.active && (
                    <button
                      className="btn btn--xs btn--ghost"
                      onClick={handleRemoveObstacle}
                      title="Remove Obstacle from simulation"
                      style={{ fontSize: 10, padding: '2px 5px', color: '#f87171' }}
                    >
                      ✕
                    </button>
                  )}
                </div>
              </div>

              <div>
                <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 2 }}>
                  <span className="text-dim text-xs" style={{ letterSpacing: '0.04em', textTransform: 'uppercase' }}>
                    Obstacle Speed
                  </span>
                  <span className="text-mono text-xs" style={{ color: '#f97316', fontWeight: 600 }}>
                    {Math.round(obsSpeed * 100)}% ({(MAX_OBS_LINEAR * obsSpeed).toFixed(2)} m/s)
                  </span>
                </div>
                <input
                  type="range"
                  min="0.1"
                  max="1.0"
                  step="0.05"
                  value={obsSpeed}
                  onChange={(e) => setObsSpeed(Number(e.target.value))}
                  style={{ width: '100%', height: 4, accentColor: '#ea580c', cursor: 'pointer' }}
                />
              </div>
            </>
          )}

          {/* Quick Directional Actions */}
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 4 }}>
            <button
              className="btn btn--ghost btn--sm"
              disabled={!isObstacleMode && (controlMode !== 'manual' || isEstopActive)}
              onClick={() => handleManualAction(0.35, 0)}
              style={{ fontSize: 10, padding: '4px 2px', borderColor: isObstacleMode ? 'rgba(234, 88, 12, 0.4)' : undefined }}
              title="Steer Forward (W / Up)"
            >
              FWD
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={!isObstacleMode && (controlMode !== 'manual' || isEstopActive)}
              onClick={() => handleManualAction(-0.35, 0)}
              style={{ fontSize: 10, padding: '4px 2px', borderColor: isObstacleMode ? 'rgba(234, 88, 12, 0.4)' : undefined }}
              title="Steer Reverse (S / Down)"
            >
              REV
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={!isObstacleMode && (controlMode !== 'manual' || isEstopActive)}
              onClick={() => handleManualAction(0, 0.9)}
              style={{ fontSize: 10, padding: '4px 2px', borderColor: isObstacleMode ? 'rgba(234, 88, 12, 0.4)' : undefined }}
              title="Turn Left (A / Left)"
            >
              TURN L
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={!isObstacleMode && (controlMode !== 'manual' || isEstopActive)}
              onClick={() => handleManualAction(0, -0.9)}
              style={{ fontSize: 10, padding: '4px 2px', borderColor: isObstacleMode ? 'rgba(234, 88, 12, 0.4)' : undefined }}
              title="Turn Right (D / Right)"
            >
              TURN R
            </button>
          </div>

          {/* Stop Button */}
          <button
            className="btn btn--sm"
            onClick={handleStopMotors}
            disabled={!isObstacleMode && controlMode !== 'manual'}
            style={{
              padding: '4px',
              fontSize: 10,
              fontWeight: 700,
              background: isObstacleMode ? 'rgba(234, 88, 12, 0.15)' : 'rgba(239, 68, 68, 0.15)',
              border: isObstacleMode ? '1px solid rgba(234, 88, 12, 0.5)' : '1px solid rgba(239, 68, 68, 0.4)',
              color: isObstacleMode ? '#fb923c' : '#f87171',
              cursor: isObstacleMode || controlMode === 'manual' ? 'pointer' : 'not-allowed',
            }}
          >
            {isObstacleMode ? 'STOP OBSTACLE' : 'STOP MOTORS'}
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
              opacity: isObstacleMode || (controlMode === 'manual' && !isEstopActive) ? 1 : 0.25,
              pointerEvents: isObstacleMode || (controlMode === 'manual' && !isEstopActive) ? 'auto' : 'none',
              transition: 'opacity 0.2s ease',
            }}
          />
          {!isObstacleMode && (isEstopActive || controlMode === 'auto') && (
            <div className="joystick-lock-overlay" style={{ padding: 6, gap: 4 }}>
              <span style={{ fontSize: 9, fontWeight: 700, color: isEstopActive ? '#f87171' : '#94a3b8' }}>
                {isEstopActive ? 'E-STOP LOCKED' : 'LOCKED (AUTO)'}
              </span>
              {!isEstopActive && (
                <button
                  className="btn btn--xs btn--ghost"
                  onClick={() => handleSwitchRobotMode('manual')}
                  style={{ fontSize: 9, padding: '2px 6px', fontWeight: 600 }}
                >
                  Enable Manual
                </button>
              )}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
