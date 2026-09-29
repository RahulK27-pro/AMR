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

export default function TelemetryPanel({ telemetry, activeMap = 'warehouse_01' }) {
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

      {/* Operating Map Card */}
      <div className="card" style={{ borderLeft: '3px solid #0284c7' }}>
        <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
          <span className="text-dim text-xs" style={{ textTransform: 'uppercase', letterSpacing: '0.05em', fontWeight: 700 }}>
            🗺️ Active Map
          </span>
          <span style={{ fontSize: 9, background: '#0284c7', color: '#fff', padding: '1px 6px', borderRadius: 4, fontWeight: 700 }}>
            ONLINE
          </span>
        </div>
        <div style={{ marginTop: 4, fontSize: 13, fontWeight: 700, color: '#38bdf8', fontFamily: 'var(--text-mono)' }}>
          {activeMap}
        </div>
      </div>

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

      {/* Station Navigation or Topological Mission */}
      {telemetry?.active_place_nav ? (
        <div className="card">
          <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
            <span>🎯 Station Navigation</span>
            <span className={`badge ${
              telemetry.active_place_nav.status === 'NAVIGATING' ? 'badge--pass' : 'badge--cyan'
            }`} style={{ fontSize: 10 }}>
              {telemetry.active_place_nav.status === 'NAVIGATING' ? '● NAVIGATING' : telemetry.active_place_nav.status}
            </span>
          </div>
          <div className="flex-col mt-xs" style={{ gap: 6 }}>
            <div className="flex-row" style={{ justifyContent: 'space-between' }}>
              <span className="text-dim text-sm">Station</span>
              <span className="text-mono font-bold" style={{ color: 'var(--accent-cyan)', fontSize: 12 }}>
                {telemetry.active_place_nav.target_place}
              </span>
            </div>
            <div className="flex-row" style={{ justifyContent: 'space-between' }}>
              <span className="text-dim text-sm">Target Node</span>
              <span className="text-mono" style={{ color: 'var(--accent-green)', fontSize: 12 }}>
                {telemetry.active_place_nav.target_node}
              </span>
            </div>
            {telemetry.active_place_nav.distance_to_target_m != null && (
              <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                <span className="text-dim text-sm">Dist. Remaining</span>
                <span className="text-mono font-bold" style={{ color: 'var(--accent-yellow)', fontSize: 12 }}>
                  {telemetry.active_place_nav.distance_to_target_m} m
                </span>
              </div>
            )}
            {telemetry.active_place_nav.is_sequence && (
              <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                <span className="text-dim text-sm">Mission Stop</span>
                <span className="text-mono" style={{ fontSize: 12 }}>
                  Stop {telemetry.active_place_nav.current_stop} / {telemetry.active_place_nav.total_stops}
                </span>
              </div>
            )}
          </div>
        </div>
      ) : ms.state ? (
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
      ) : null}

    </div>
  );
}
