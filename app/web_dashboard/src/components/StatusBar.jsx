import { useState, useEffect } from 'react';
import bridge from '../services/amrBridge';

/**
 * StatusBar — top header bar showing connection state, bridge URL, and key global info.
 */
export default function StatusBar({
  telemetry,
  currentView = 'dashboard',
  onViewChange,
  activeMap = 'warehouse_01',
  onMapChange,
}) {
  const [status, setStatus] = useState('connecting');
  const [host, setHost]     = useState(window.location.hostname);
  const [port, setPort]     = useState('8000');
  const [editing, setEditing] = useState(false);
  const [availableMaps, setAvailableMaps] = useState([]);

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
    return () => { mounted = false; clearInterval(interval); };
  }, []);

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

      {/* ── Active Map Switcher (Phase 7 / Multi-Map Separation) ── */}
      <div
        className="status-bar__map-switcher"
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: 6,
          background: 'rgba(15, 23, 42, 0.85)',
          border: '1px solid #334155',
          borderRadius: 8,
          padding: '3px 10px',
          boxShadow: '0 1px 3px rgba(0,0,0,0.3)',
        }}
        title="Switch active operating map and topological roadmap"
      >
        <span style={{ fontSize: 13, display: 'flex', alignItems: 'center', gap: 4 }}>
          <span>🗺️</span>
          <span style={{ fontSize: 11, fontWeight: 700, letterSpacing: '0.05em', color: '#94a3b8', textTransform: 'uppercase' }}>
            Map:
          </span>
        </span>
        <select
          id="select-active-map"
          value={activeMap}
          onChange={(e) => onMapChange && onMapChange(e.target.value)}
          style={{
            background: 'transparent',
            color: '#38bdf8',
            border: 'none',
            outline: 'none',
            fontSize: 12,
            fontWeight: 700,
            cursor: 'pointer',
            padding: '2px 4px',
          }}
        >
          {availableMaps.length > 0 ? (
            availableMaps.map((m) => (
              <option key={m.name} value={m.name} style={{ background: '#1e293b', color: '#f8fafc' }}>
                {m.name} {m.node_count ? `(${m.node_count} nodes)` : ''} {m.place_count ? `• ${m.place_count} stns` : ''}
              </option>
            ))
          ) : (
            <>
              <option value="warehouse_01" style={{ background: '#1e293b', color: '#f8fafc' }}>warehouse_01 (565 nodes)</option>
              <option value="warehouse_map" style={{ background: '#1e293b', color: '#f8fafc' }}>warehouse_map (764 nodes)</option>
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

      {/* Clock */}
      <span className="text-dim text-mono" style={{ fontSize: 12, minWidth: 60 }}>
        {telemetry?.ts ? new Date(telemetry.ts * 1000).toLocaleTimeString() : '--:--:--'}
      </span>
    </header>
  );
}
