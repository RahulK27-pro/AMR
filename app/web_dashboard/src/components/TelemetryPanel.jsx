/**
 * TelemetryPanel — left sidebar with all real-time sensor data.
 * Shows pose, velocity, IMU, obstacle proximity, mission status, obstacle alert.
 */

function TelemItem({ label, value, unit, color }) {
  return (
    <div className="telem-item">
      <div className="telem-item__label">{label}</div>
      <div className="telem-item__value" style={color ? { color } : {}}>
        {value}
        {unit && <span className="telem-item__unit">{unit}</span>}
      </div>
    </div>
  );
}

function DistBar({ value, max = 8 }) {
  const pct    = value != null ? Math.min((value / max) * 100, 100) : 0;
  const color  =
    value == null     ? '#4a5580' :
    value < 0.5       ? 'var(--accent-red)'    :
    value < 1.2       ? 'var(--accent-orange)' :
                        'var(--accent-green)';
  return (
    <div className="dist-bar-wrapper">
      <div className="dist-bar-label">
        <span>Nearest obstacle</span>
        <span style={{ color, fontFamily: 'var(--text-mono)', fontWeight: 600 }}>
          {value != null ? `${value.toFixed(2)} m` : '—'}
        </span>
      </div>
      <div className="dist-bar-track">
        <div className="dist-bar-fill" style={{ width: `${pct}%`, background: color }} />
      </div>
    </div>
  );
}

export default function TelemetryPanel({ telemetry }) {
  const p  = telemetry?.pose     ?? { x: 0, y: 0, yaw: 0 };
  const v  = telemetry?.velocity ?? { linear: 0, angular: 0 };
  const im = telemetry?.imu      ?? { roll: 0, pitch: 0, yaw: 0 };
  const ms = telemetry?.mission  ?? {};
  const obs = telemetry?.obstacle_alert;
  const dist = telemetry?.min_obstacle_dist;

  const rad2deg = r => ((r ?? 0) * 180 / Math.PI).toFixed(1);
  const fmt2    = n => (n ?? 0).toFixed(2);
  const fmt3    = n => (n ?? 0).toFixed(3);

  return (
    <div className="flex-col" style={{ gap: 12 }}>

      {/* Obstacle Alert Banner */}
      {obs && (
        <div className="alert-banner">
          <span>⚠</span>
          <span>{obs}</span>
        </div>
      )}

      {/* Pose */}
      <div className="card">
        <div className="card__title">📍 Robot Pose</div>
        <div className="telem-grid">
          <TelemItem label="X"   value={fmt2(p.x)}   unit="m"   />
          <TelemItem label="Y"   value={fmt2(p.y)}   unit="m"   />
          <TelemItem label="Yaw" value={rad2deg(p.yaw)} unit="°" color="var(--accent-cyan)" />
          <TelemItem label="Yaw (rad)" value={fmt3(p.yaw)} unit="rad" />
        </div>
      </div>

      {/* Velocity */}
      <div className="card">
        <div className="card__title">⚡ Velocity</div>
        <div className="telem-grid">
          <TelemItem label="Linear"  value={fmt2(v.linear)}  unit="m/s"   color="var(--accent-green)"  />
          <TelemItem label="Angular" value={fmt2(v.angular)} unit="rad/s" color="var(--accent-yellow)" />
        </div>
      </div>

      {/* Proximity */}
      <div className="card">
        <div className="card__title">🔭 Proximity</div>
        <DistBar value={dist} />
      </div>

      {/* IMU */}
      <div className="card">
        <div className="card__title">🧭 IMU Orientation</div>
        <div className="telem-grid">
          <TelemItem label="Roll"  value={rad2deg(im.roll)}  unit="°" />
          <TelemItem label="Pitch" value={rad2deg(im.pitch)} unit="°" />
          <TelemItem label="Yaw"   value={rad2deg(im.yaw)}   unit="°" color="var(--accent-purple)" />
        </div>
      </div>

      {/* Mission */}
      {ms.state && (
        <div className="card">
          <div className="card__title">🎯 Mission</div>
          <div className="flex-col" style={{ gap: 6 }}>
            <div className="flex-row" style={{ justifyContent: 'space-between' }}>
              <span className="text-dim text-sm">State</span>
              <span className="text-mono" style={{ color: 'var(--accent-cyan)', fontSize: 12 }}>{ms.state}</span>
            </div>
            {ms.total > 0 && (
              <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                <span className="text-dim text-sm">Progress</span>
                <span className="text-mono" style={{ fontSize: 12 }}>{ms.current}/{ms.total}</span>
              </div>
            )}
            {ms.goal_node && (
              <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                <span className="text-dim text-sm">Goal node</span>
                <span className="text-mono" style={{ fontSize: 12, color: 'var(--accent-yellow)' }}>{ms.goal_node}</span>
              </div>
            )}
          </div>
        </div>
      )}

    </div>
  );
}
