import { useState, useEffect } from 'react';
import bridge from './services/amrBridge';
import StatusBar from './components/StatusBar';
import TelemetryPanel from './components/TelemetryPanel';
import MapView from './components/MapView';
import ScanRing from './components/ScanRing';
import ControlPanel from './components/ControlPanel';

export default function App() {
  const [telemetry, setTelemetry] = useState(null);

  useEffect(() => {
    const unsub = bridge.onTelemetry(setTelemetry);
    return unsub;
  }, []);

  const scan = telemetry?.scan ?? [];
  const angleMin = telemetry?.scan_angle_min ?? -Math.PI;
  const angleInc = telemetry?.scan_angle_inc ?? 0.0174;

  return (
    <div className="dashboard-layout">

      {/* ── Top bar ── */}
      <StatusBar telemetry={telemetry} />

      {/* ── Left: Telemetry ── */}
      <aside className="panel-left">
        <TelemetryPanel telemetry={telemetry} />
      </aside>

      {/* ── Centre: Map + LiDAR ring ── */}
      <main className="panel-center">
        <MapView telemetry={telemetry} />

        {/* LiDAR ring below map */}
        <div className="card" style={{ flexShrink: 0 }}>
          <div className="card__title">🔴 LiDAR Scan</div>
          <ScanRing
            scan={scan}
            angleMin={angleMin}
            angleInc={angleInc}
            size={180}
          />
        </div>
      </main>

      {/* ── Right: Control ── */}
      <aside className="panel-right">
        <ControlPanel telemetry={telemetry} />
      </aside>

    </div>
  );
}
