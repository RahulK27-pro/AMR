import React, { useState, useEffect } from 'react';
import bridge from './services/amrBridge';
import StatusBar from './components/StatusBar';
import TelemetryPanel from './components/TelemetryPanel';
import MapView from './components/MapView';
import ScanRing from './components/ScanRing';
import ControlPanel from './components/ControlPanel';
import EventLog from './components/EventLog';
import MappingScreen from './components/MappingScreen';

class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }
  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }
  componentDidCatch(error, errorInfo) {
    console.error("MappingScreen render error:", error, errorInfo);
  }
  render() {
    if (this.state.hasError) {
      return (
        <div style={{ padding: 32, maxWidth: 640, margin: '60px auto', background: '#1e293b', border: '1px solid #ef4444', borderRadius: 8, color: '#f8fafc' }}>
          <h2 style={{ color: '#ef4444', marginTop: 0 }}>⚠️ Mapping Screen Error</h2>
          <p style={{ color: '#94a3b8', fontSize: 14 }}>An error occurred while rendering the Mapping workflow:</p>
          <pre style={{ background: '#0f172a', padding: 14, borderRadius: 6, color: '#fca5a5', overflowX: 'auto', fontSize: 12 }}>
            {this.state.error?.toString()}
          </pre>
          <div style={{ display: 'flex', gap: 12, marginTop: 16 }}>
            <button
              className="btn btn--primary"
              onClick={() => {
                this.setState({ hasError: false, error: null });
                window.location.reload();
              }}
            >
              🔄 Reload Page
            </button>
            <button
              className="btn btn--outline"
              onClick={() => {
                this.setState({ hasError: false, error: null });
                if (this.props.onReturnToDashboard) this.props.onReturnToDashboard();
              }}
            >
              Return to Dashboard
            </button>
          </div>
        </div>
      );
    }
    return this.props.children;
  }
}

export default function App() {
  const [telemetry, setTelemetry] = useState(null);
  const [selectedNodes, setSelectedNodes] = useState([]);
  const [currentView, setCurrentView] = useState('dashboard'); // 'dashboard' | 'mapping'

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
    <div className={`dashboard-layout ${currentView === 'mapping' ? 'dashboard-layout--mapping' : ''}`}>
      {/* ── Top Bar ── */}
      <StatusBar
        telemetry={telemetry}
        currentView={currentView}
        onViewChange={setCurrentView}
      />

      {currentView === 'mapping' ? (
        <main className="mapping-screen-wrapper">
          <ErrorBoundary onReturnToDashboard={() => setCurrentView('dashboard')}>
            <MappingScreen
              telemetry={telemetry}
              onReturnToDashboard={() => setCurrentView('dashboard')}
            />
          </ErrorBoundary>
        </main>
      ) : (
        <>
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
              onOpenMapping={() => setCurrentView('mapping')}
            />
          </aside>
        </>
      )}
    </div>
  );
}
