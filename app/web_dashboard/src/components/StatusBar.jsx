import { useState, useEffect } from 'react';
import bridge from '../services/amrBridge';

/**
 * StatusBar — top header bar showing connection state, bridge URL, and key global info.
 */
export default function StatusBar({
  telemetry,
  currentView = 'dashboard',
  onViewChange,
  activeMap = 'hospital_map',
  onMapChange,
  theme = 'dark',
  onToggleTheme,
}) {
  const [status, setStatus] = useState('connecting');
  const [host, setHost]     = useState(window.location.hostname);
  const [port, setPort]     = useState('8000');
  const [editing, setEditing] = useState(false);
  const [availableMaps, setAvailableMaps] = useState([]);
  const [localEstop, setLocalEstop] = useState(false);

  useEffect(() => {
    if (telemetry?.nav_state === 'ESTOP') {
      setLocalEstop(true);
    } else if (telemetry?.nav_state && telemetry.nav_state !== 'ESTOP') {
      setLocalEstop(false);
    }
  }, [telemetry?.nav_state]);

  const isEstopActive = telemetry?.nav_state === 'ESTOP' || localEstop;

  const handleEstop = async () => {
    setLocalEstop(true);
    try {
      await bridge.sendStop();
    } catch (_) {}
  };

  const handleEstopClear = async () => {
    setLocalEstop(false);
    try {
      await bridge.clearEstop();
    } catch (_) {}
  };

  useEffect(() => {
    bridge.connect(host, Number(port));
    const unsub = bridge.onStatusChange(setStatus);
    return unsub;
  }, []);

  // Poll / refresh available maps list
  useEffect(() => {
    let mounted = true;
    const fetchMaps = () => {
      bridge.getSavedMaps().then((res) => {
        if (mounted && res?.maps) {
          setAvailableMaps(res.maps);
        }
      });
    };
    fetchMaps();
    const interval = setInterval(fetchMaps, 8000);

    const handleCustomRefresh = () => {
      fetchMaps();
    };
    window.addEventListener('amr_maps_updated', handleCustomRefresh);
    window.addEventListener('amr_map_saved', handleCustomRefresh);

    return () => {
      mounted = false;
      clearInterval(interval);
      window.removeEventListener('amr_maps_updated', handleCustomRefresh);
      window.removeEventListener('amr_map_saved', handleCustomRefresh);
    };
  }, [activeMap, currentView]);

  const handleConnect = () => {
    bridge.connect(host, Number(port));
    setEditing(false);
  };

  const dotClass =
    status === 'connected'    ? 'dot dot--connected'  :
    status === 'connecting'   ? 'dot dot--connecting' :
                                'dot dot--error';

  const statusLabel =
    status === 'connected'    ? 'Live' :
    status === 'connecting'   ? 'Connecting…' :
    status === 'error'        ? 'Error' : 'Offline';

  const navState = telemetry?.nav_state ?? '—';

  return (
    <header className="status-bar">
      {/* Logo */}
      <span className="status-bar__logo" onClick={() => onViewChange && onViewChange('dashboard')} style={{ cursor: 'pointer' }}>
        ⬡ AMR CONTROL
      </span>

      {/* Connection status */}
      <div className="status-bar__pill" title={`ws://${host}:${port}/ws/telemetry`}>
        <span className={dotClass} />
        <span>{statusLabel}</span>
        <span className="text-dim" style={{ fontSize: 11, marginLeft: 4 }}>
          {host}:{port}
        </span>
      </div>

      {/* Nav state */}
      <div className={`nav-state-badge nav-state--${navState}`}>
        <span style={{ width: 7, height: 7, borderRadius: '50%', background: 'currentColor', display: 'inline-block' }} />
        {navState}
      </div>

      {/* ── Active Map Switcher (Multi-Map Separation) ── */}
      <div
        className="status-bar__map-switcher"
        title="Switch active operating map and topological roadmap"
      >
        <span style={{ fontSize: 13, display: 'flex', alignItems: 'center', gap: 4 }}>
          <span>🗺️</span>
          <span style={{ fontSize: 11, fontWeight: 700, letterSpacing: '0.05em', color: 'var(--text-dim)', textTransform: 'uppercase' }}>
            Map:
          </span>
        </span>
        <select
          id="select-active-map"
          className="status-bar__map-select"
          value={activeMap}
          onChange={(e) => onMapChange && onMapChange(e.target.value)}
          onFocus={() => {
            bridge.getSavedMaps().then((res) => {
              if (res?.maps) setAvailableMaps(res.maps);
            });
          }}
        >
          {availableMaps.length > 0 ? (
            <>
              {availableMaps.map((m) => (
                <option key={m.name} value={m.name}>
                  {m.name} {m.node_count ? `(${m.node_count} nodes)` : ''} {m.place_count ? `• ${m.place_count} stns` : ''}
                </option>
              ))}
              {activeMap && !availableMaps.some((m) => m.name === activeMap) && (
                <option key={activeMap} value={activeMap}>
                  {activeMap} (newly saved)
                </option>
              )}
            </>
          ) : (
            <>
              {activeMap && activeMap !== 'hospital_map' && activeMap !== 'warehouse_map' && (
                <option value={activeMap}>{activeMap}</option>
              )}
              <option value="hospital_map">hospital_map (661 nodes)</option>
              <option value="warehouse_map">warehouse_map (764 nodes)</option>
            </>
          )}
        </select>
        <span
          style={{
            fontSize: 9,
            background: '#0284c7',
            color: '#ffffff',
            padding: '1px 5px',
            borderRadius: 4,
            fontWeight: 700,
            letterSpacing: '0.04em',
            textTransform: 'uppercase',
          }}
        >
          Active
        </span>
      </div>

      {/* Dedicated Pipeline Entry Point: Map New Place */}
      {onViewChange && (
        <div className="status-bar__view-toggle">
          {currentView === 'mapping' ? (
            <button
              id="btn-nav-dashboard"
              className="btn btn--outline"
              onClick={() => onViewChange('dashboard')}
              title="Return to Live Monitoring Dashboard"
              style={{ fontWeight: 600, padding: '6px 14px' }}
            >
              📊 Live Dashboard
            </button>
          ) : (
            <button
              id="btn-nav-map-new-place"
              className="btn btn--map-entry"
              onClick={() => onViewChange('mapping')}
              title="Open Dedicated Mapping Workflow"
            >
              🗺️ Map New Place
            </button>
          )}
        </div>
      )}

      <div className="status-bar__sep" />

      {/* Small Emergency Stop */}
      {isEstopActive ? (
        <button
          id="btn-header-estop"
          className="btn-estop-compact btn-estop-compact--active"
          onClick={handleEstopClear}
          title="Emergency Stop is ACTIVE! Click to clear and restore navigation."
        >
          CLEAR E-STOP
        </button>
      ) : (
        <button
          id="btn-header-estop"
          className="btn-estop-compact"
          onClick={handleEstop}
          title="Emergency Stop: Halt robot immediately"
        >
          E-STOP
        </button>
      )}

      {/* Bridge host editor */}
      {editing ? (
        <div className="flex-row" style={{ gap: 6 }}>
          <input
            className="input-field"
            style={{ width: 130 }}
            value={host}
            onChange={e => setHost(e.target.value)}
            placeholder="host / IP"
          />
          <input
            className="input-field"
            style={{ width: 70 }}
            value={port}
            onChange={e => setPort(e.target.value)}
            placeholder="port"
          />
          <button className="btn btn--primary" onClick={handleConnect}>Connect</button>
          <button className="btn btn--ghost" onClick={() => setEditing(false)}>✕</button>
        </div>
      ) : (
        <button className="btn btn--ghost" onClick={() => setEditing(true)} style={{ fontSize: 12 }}>
          ⚙ Bridge
        </button>
      )}

      {/* ── Theme Mode Toggle Button (Light / Dark) ── */}
      <button
        id="btn-theme-toggle"
        className={`btn-theme-toggle ${theme === 'light' ? 'btn-theme-toggle--light' : 'btn-theme-toggle--dark'}`}
        onClick={onToggleTheme}
        title={theme === 'light' ? 'Switch to Dark Theme' : 'Switch to Professional Light Theme (Clinical Blue & White)'}
        aria-label="Toggle theme mode"
      >
        <span className="theme-toggle__icon">{theme === 'light' ? '☀️' : '🌙'}</span>
        <span className="theme-toggle__text">{theme === 'light' ? 'Light Mode' : 'Dark Mode'}</span>
      </button>

      {/* Clock */}
      <span className="text-dim text-mono" style={{ fontSize: 12, minWidth: 60 }}>
        {telemetry?.ts ? new Date(telemetry.ts * 1000).toLocaleTimeString() : '--:--:--'}
      </span>
    </header>
  );
}
