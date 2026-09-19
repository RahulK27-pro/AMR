import { useState, useEffect } from 'react';
import bridge from './services/amrBridge';
import StatusBar from './components/StatusBar';
import TelemetryPanel from './components/TelemetryPanel';
import MapView from './components/MapView';
import ScanRing from './components/ScanRing';
import ControlPanel from './components/ControlPanel';
import EventLog from './components/EventLog';

export default function App() {
  const [telemetry, setTelemetry] = useState(null);
  const [selectedNodes, setSelectedNodes] = useState([]);

  useEffect(() => {
    const unsub = bridge.onTelemetry(setTelemetry);
    return unsub;
  }, []);

  const handleSelectNode = (nodeId) => {
    if (!selectedNodes.includes(nodeId)) {
      setSelectedNodes((prev) => [...prev, nodeId]);
    }
  };

  const scan = telemetry?.scan ?? [];
  const angleMin = telemetry?.scan_angle_min ?? -Math.PI;
  const angleInc = telemetry?.scan_angle_inc ?? 0.0174;

  return (
    <div className="dashboard-layout">
      {/* ── Top Bar ── */}
      <StatusBar telemetry={telemetry} />

      {/* ── Left: Telemetry Data ── */}
      <aside className="panel-left">
        <TelemetryPanel telemetry={telemetry} />
      </aside>

      {/* ── Centre: 60 FPS Vector Map + Polar Radar + Event Timeline ── */}
      <main className="panel-center">
        {/* Dynamic Vector Map */}
        <div style={{ flex: 1, minHeight: 440, position: 'relative' }}>
          <MapView
            telemetry={telemetry}
            selectedNodes={selectedNodes}
            onSelectNode={handleSelectNode}
          />
        </div>

        {/* Center Bottom Grid: LiDAR Scan + Diagnostics Timeline */}
        <div className="center-bottom-grid">
          {/* Polar LiDAR Ring */}
          <div className="card scan-card">
            <div className="card__title">📡 360° LiDAR Radar</div>
            <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '6px 0' }}>
              <ScanRing
                scan={scan}
                angleMin={angleMin}
                angleInc={angleInc}
                size={180}
              />
            </div>
          </div>

          {/* Autonomous Decisions Event Log */}
          <EventLog />
        </div>
      </main>

      {/* ── Right: Control Panel, Mode Interlocks, Mission Queue ── */}
      <aside className="panel-right">
        <ControlPanel
          telemetry={telemetry}
          selectedNodes={selectedNodes}
          setSelectedNodes={setSelectedNodes}
        />
      </aside>
    </div>
  );
}
