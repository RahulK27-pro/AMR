import React, { useState, useEffect } from 'react';
import bridge from './services/amrBridge';
import StatusBar from './components/StatusBar';
import TelemetryPanel from './components/TelemetryPanel';
import MapView from './components/MapView';
import ControlPanel from './components/ControlPanel';
import EventLog from './components/EventLog';
import ControlInterlock from './components/ControlInterlock';
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
  const [controlMode, setControlMode] = useState('auto');
  const [currentView, setCurrentView] = useState('dashboard'); // 'dashboard' | 'mapping'
  const [runtimeMode, setRuntimeMode] = useState(() => localStorage.getItem('amr_runtime_mode') || 'sim');
  const [piIp, setPiIp] = useState(() => localStorage.getItem('amr_pi_ip') || '10.244.248.208');
  const [isSwitchingMode, setIsSwitchingMode] = useState(false);
  const [activeMap, setActiveMap] = useState(() => {
    return localStorage.getItem('amr_active_map') || 'hospital_map';
  });
  const [theme, setTheme] = useState(() => {
    return localStorage.getItem('amr_theme') || 'dark';
  });

  const effectiveMode = telemetry?.mode || runtimeMode;

  // Synchronize data-theme on html & body tags for CSS cascade
  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
    document.body.setAttribute('data-theme', theme);
    localStorage.setItem('amr_theme', theme);
  }, [theme]);

  const handleToggleTheme = () => {
    setTheme((prev) => (prev === 'light' ? 'dark' : 'light'));
  };

  useEffect(() => {
    const unsub = bridge.onTelemetry(setTelemetry);
    return unsub;
  }, []);

  // Fetch initial active map and runtime mode from bridge
  useEffect(() => {
    bridge.getMode().then((res) => {
      if (res?.ok && res.mode) {
        setRuntimeMode(res.mode);
        localStorage.setItem('amr_runtime_mode', res.mode);
        if (res.pi_ip) {
          setPiIp(res.pi_ip);
          localStorage.setItem('amr_pi_ip', res.pi_ip);
        }
      }
    });
    bridge.getActiveMap().then((res) => {
      if (res?.ok && res.active_map) {
        setActiveMap(res.active_map);
        localStorage.setItem('amr_active_map', res.active_map);
      }
    });
  }, []);

  // Keep activeMap & mode in sync if telemetry updates them from backend
  useEffect(() => {
    if (telemetry?.mode && telemetry.mode !== runtimeMode) {
      setRuntimeMode(telemetry.mode);
      localStorage.setItem('amr_runtime_mode', telemetry.mode);
      setIsSwitchingMode(false);
    }
    if (telemetry?.pi_ip && telemetry.pi_ip !== piIp) {
      setPiIp(telemetry.pi_ip);
      localStorage.setItem('amr_pi_ip', telemetry.pi_ip);
    }
    if (telemetry?.active_map && telemetry.active_map !== activeMap) {
      setActiveMap(telemetry.active_map);
      localStorage.setItem('amr_active_map', telemetry.active_map);
    }
  }, [telemetry?.mode, telemetry?.pi_ip, telemetry?.active_map]);

  const handleSwitchMode = async (nextMode, nextPiIp = piIp) => {
    const cleanMode = nextMode === 'hardware' ? 'hardware' : 'sim';
    if (cleanMode === effectiveMode && nextPiIp === piIp) return;
    setIsSwitchingMode(true);
    setSelectedNodes([]);
    const res = await bridge.switchMode(cleanMode, nextPiIp);
    if (res?.ok) {
      setRuntimeMode(cleanMode);
      localStorage.setItem('amr_runtime_mode', cleanMode);
      if (nextPiIp) {
        setPiIp(nextPiIp);
        localStorage.setItem('amr_pi_ip', nextPiIp);
      }
      // Poll mode endpoint cleanly after bridge restarts to sync active_map immediately
      setTimeout(() => {
        bridge.getActiveMap().then((mres) => {
          if (mres?.ok && mres.active_map) {
            setActiveMap(mres.active_map);
            localStorage.setItem('amr_active_map', mres.active_map);
          }
          setIsSwitchingMode(false);
        });
      }, 2200);
    } else {
      setIsSwitchingMode(false);
    }
  };

  const handleMapChange = async (newMap) => {
    setActiveMap(newMap);
    localStorage.setItem('amr_active_map', newMap);
    setSelectedNodes([]); // Clear mission waypoints when switching maps!
    bridge.emitEvent('info', `Switched active roadmap to "${newMap}"`);
    try {
      await bridge.setActiveMap(newMap);
    } catch (_) {}
  };

  const handleSelectNode = (nodeId) => {
    if (!selectedNodes.includes(nodeId)) {
      setSelectedNodes((prev) => [...prev, nodeId]);
    }
  };

  return (
    <div className={`dashboard-layout ${currentView === 'mapping' ? 'dashboard-layout--mapping' : ''}`}>
      {/* ── Top Bar ── */}
      <StatusBar
        telemetry={telemetry}
        currentView={currentView}
        onViewChange={setCurrentView}
        activeMap={activeMap}
        onMapChange={handleMapChange}
        theme={theme}
        onToggleTheme={handleToggleTheme}
        mode={effectiveMode}
        piIp={piIp}
        onSwitchMode={handleSwitchMode}
        isSwitchingMode={isSwitchingMode}
      />

      {currentView === 'mapping' ? (
        <main className="mapping-screen-wrapper">
          <ErrorBoundary onReturnToDashboard={() => setCurrentView('dashboard')}>
            <MappingScreen
              telemetry={telemetry}
              onReturnToDashboard={() => setCurrentView('dashboard')}
              onMapChange={handleMapChange}
              activeMap={activeMap}
              theme={theme}
            />
          </ErrorBoundary>
        </main>
      ) : (
        <>
          {/* ── Left: Telemetry Data ── */}
          <aside className="panel-left">
            <TelemetryPanel telemetry={telemetry} activeMap={activeMap} mode={effectiveMode} />
          </aside>

          {/* ── Centre: 60 FPS Vector Map + Event Timeline ── */}
          <main className="panel-center">
            {/* Dynamic Vector Map */}
            <div style={{ flex: 1, minHeight: 440, position: 'relative' }}>
              <MapView
                telemetry={telemetry}
                selectedNodes={selectedNodes}
                onSelectNode={handleSelectNode}
                activeMap={activeMap}
                theme={theme}
                mode={effectiveMode}
              />
            </div>

            {/* Center Bottom: Control Interlock & Autonomous Decisions Event Log */}
            <div className="center-bottom-grid">
              <ControlInterlock
                telemetry={telemetry}
                controlMode={controlMode}
                setControlMode={setControlMode}
                mode={effectiveMode}
              />
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
              activeMap={activeMap}
              onMapChange={handleMapChange}
              controlMode={controlMode}
              setControlMode={setControlMode}
              mode={effectiveMode}
              piIp={piIp}
            />
          </aside>
        </>
      )}
    </div>
  );
}
