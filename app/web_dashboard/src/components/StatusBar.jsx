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
  mode = 'sim',
  piIp = '10.244.248.208',
  onSwitchMode,
  isSwitchingMode = false,
}) {
  const [status, setStatus] = useState('connecting');
  const [host, setHost]     = useState(window.location.hostname);
  const [port, setPort]     = useState('8000');
  const [editPiIp, setEditPiIp] = useState(piIp);
  const [editing, setEditing] = useState(false);
  const [availableMaps, setAvailableMaps] = useState([]);
  const [localEstop, setLocalEstop] = useState(false);

  useEffect(() => {
    setEditPiIp(piIp);
  }, [piIp]);

  useEffect(() => {
    const backendEstop = Boolean(telemetry?.estop_active) || telemetry?.nav_state === 'ESTOP';
    if (backendEstop) {
      setLocalEstop(true);
    } else if (telemetry && telemetry.estop_active === false && telemetry.nav_state !== 'ESTOP') {
      setLocalEstop(false);
    }
  }, [telemetry?.estop_active, telemetry?.nav_state]);

  const isEstopActive = Boolean(telemetry?.estop_active) || telemetry?.nav_state === 'ESTOP' || localEstop;
  const hwLink = telemetry?.hardware_link;

  const handleEstop = async () => {
    setLocalEstop(true);
    try {
      await bridge.sendStop();
    } catch (_) {}
  };

  const handleEstopClear = async () => {
    try {
      const ok = await bridge.clearEstop();
      if (ok) {
        setLocalEstop(false);
      }
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
  }, [activeMap, currentView, mode]);

  const handleConnect = () => {
    bridge.connect(host, Number(port));
    const trimmedPi = (editPiIp || '').trim();
    if (onSwitchMode && trimmedPi && trimmedPi !== piIp) {
      onSwitchMode(mode, trimmedPi);
    }
    setEditing(false);
  };

  const hardwareMaps = availableMaps.filter((m) => m.category === 'hardware');
  const simMaps = availableMaps.filter((m) => m.category !== 'hardware');
  const orderedMapGroups = mode === 'hardware'
    ? [
        { label: 'Hardware Maps', list: hardwareMaps },
        { label: 'Simulation Maps', list: simMaps },
      ]
    : [
        { label: 'Simulation Maps', list: simMaps },
        { label: 'Hardware Maps', list: hardwareMaps },
      ];

  const dotClass =
    status === 'connected'    ? 'dot dot--connected'  :
    status === 'connecting'   ? 'dot dot--connecting' :
                                'dot dot--error';

  const statusLabel =
    isSwitchingMode           ? 'Switching…' :
    status === 'connected'    ? 'Live' :
    status === 'connecting'   ? 'Connecting…' :
    status === 'error'        ? 'Error' : 'Offline';

  const navState = telemetry?.nav_state ?? '—';

  return (
    <header className="status-bar" style={{ gap: 12, padding: '0 14px' }}>
      {/* Logo */}
      <span className="status-bar__logo" onClick={() => onViewChange && onViewChange('dashboard')} style={{ cursor: 'pointer', fontSize: 15 }}>
        ⬡ AMR CONTROL
      </span>

      {/* Runtime Mode Switcher: Simulation vs Physical Hardware */}
      <div
        className="status-bar__pill"
        style={{ padding: '2px 4px', gap: 3, borderColor: mode === 'hardware' ? 'rgba(16, 185, 129, 0.5)' : 'var(--border)' }}
        title="Switch between Gazebo Simulation and Real Hardware (Pi 5)"
      >
        <button
          id="btn-mode-sim"
          type="button"
          disabled={isSwitchingMode}
          onClick={() => onSwitchMode && onSwitchMode('sim')}
          style={{
            border: 'none',
            borderRadius: 14,
            padding: '3px 9px',
            fontSize: 11,
            fontWeight: 700,
            cursor: isSwitchingMode ? 'wait' : 'pointer',
            background: mode === 'sim' ? 'var(--accent-cyan)' : 'transparent',
            color: mode === 'sim' ? '#0a0d14' : 'var(--text-secondary)',
          }}
        >
          SIM
        </button>
        <button
          id="btn-mode-hardware"
          type="button"
          disabled={isSwitchingMode}
          onClick={() => onSwitchMode && onSwitchMode('hardware', editPiIp)}
          style={{
            border: 'none',
            borderRadius: 14,
            padding: '3px 9px',
            fontSize: 11,
            fontWeight: 700,
            cursor: isSwitchingMode ? 'wait' : 'pointer',
            background: mode === 'hardware' ? '#10b981' : 'transparent',
            color: mode === 'hardware' ? '#ffffff' : 'var(--text-secondary)',
          }}
        >
          HARDWARE
        </button>
      </div>

      {/* Connection status */}
      <div className="status-bar__pill" title={`ws://${host}:${port}/ws/telemetry`}>
        <span className={dotClass} />
        <span>{statusLabel}</span>
        {mode === 'hardware' && hwLink && (
          <span
            style={{
              fontSize: 10,
              marginLeft: 4,
              padding: '1px 6px',
              borderRadius: 8,
              background: hwLink.sensors_ready
                ? 'rgba(16, 185, 129, 0.16)'
                : hwLink.pi_ssh?.state === 'STARTING'
                ? 'rgba(56, 189, 248, 0.18)'
                : 'rgba(245, 158, 11, 0.16)',
              color: hwLink.sensors_ready
                ? '#34d399'
                : hwLink.pi_ssh?.state === 'STARTING'
                ? '#38bdf8'
                : '#fbbf24',
              fontWeight: 700,
            }}
            title={`Pi ${piIp} | SSH: ${hwLink.pi_ssh?.ssh_ready ? 'OK' : 'NO'} | /scan: ${hwLink.scan_alive ? 'OK' : 'NO'} | /odom: ${hwLink.odom_alive ? 'OK' : 'NO'} | TF: ${hwLink.map_tf_locked ? 'OK' : 'NO'}`}
          >
            {hwLink.sensors_ready
              ? 'PI OK'
              : hwLink.pi_ssh?.state === 'STARTING'
              ? 'PI BOOT'
              : hwLink.pi_ssh?.ssh_ready
              ? 'SSH OK'
              : 'WAIT PI'}
          </span>
        )}
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
              {orderedMapGroups.map(({ label, list }) =>
                list.length > 0 ? (
                  <optgroup key={label} label={label}>
                    {list.map((m) => (
                      <option key={m.name} value={m.name}>
                        {m.name} {m.node_count ? `(${m.node_count} nodes)` : ''} {m.place_count ? `• ${m.place_count} stns` : ''}
                      </option>
                    ))}
                  </optgroup>
                ) : null
              )}
              {activeMap && !availableMaps.some((m) => m.name === activeMap) && (
                <option key={activeMap} value={activeMap}>
                  {activeMap} (newly saved)
                </option>
              )}
            </>
          ) : (
            <>
              {activeMap && !['hospital_map', 'warehouse_map', 'prakash_room'].includes(activeMap) && (
                <option value={activeMap}>{activeMap}</option>
              )}
              {(mode === 'hardware'
                ? ['prakash_room', 'hospital_map', 'warehouse_map']
                : ['hospital_map', 'warehouse_map', 'prakash_room']
              ).map((fallbackName) => (
                <option key={fallbackName} value={fallbackName}>
                  {fallbackName}
                </option>
              ))}
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
              disabled={
                isSwitchingMode
                || telemetry?.system_status?.state === 'RUNNING'
                || Boolean(telemetry?.system_status?.external_running)
              }
              onClick={() => onViewChange('mapping')}
              title={
                telemetry?.system_status?.state === 'RUNNING' || telemetry?.system_status?.external_running
                  ? 'Stop active Navigation stack before opening SLAM Mapping'
                  : mode === 'hardware'
                  ? 'Open Physical Hardware SLAM Mapping Workflow'
                  : 'Open Dedicated Mapping Workflow'
              }
              style={
                (isSwitchingMode || telemetry?.system_status?.state === 'RUNNING' || telemetry?.system_status?.external_running)
                  ? { opacity: 0.5, cursor: 'not-allowed' }
                  : undefined
              }
            >
              {mode === 'hardware' ? '🗺️ Map Room (HW)' : '🗺️ Map New Place'}
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

      {/* Bridge host & Pi IP editor */}
      {editing ? (
        <div className="flex-row" style={{ gap: 6 }}>
          <input
            className="input-field"
            style={{ width: 115 }}
            value={host}
            onChange={e => setHost(e.target.value)}
            placeholder="bridge host"
            title="Bridge server host/IP"
          />
          <input
            className="input-field"
            style={{ width: 58 }}
            value={port}
            onChange={e => setPort(e.target.value)}
            placeholder="port"
            title="Bridge server port"
          />
          <input
            className="input-field"
            style={{ width: 115 }}
            value={editPiIp}
            onChange={e => setEditPiIp(e.target.value)}
            placeholder="Pi IP"
            title="Raspberry Pi 5 IP"
          />
          <button className="btn btn--primary" onClick={handleConnect}>Apply</button>
          <button className="btn btn--ghost" onClick={() => setEditing(false)}>✕</button>
        </div>
      ) : (
        <button className="btn btn--ghost" onClick={() => setEditing(true)} style={{ fontSize: 12 }}>
          ⚙ {mode === 'hardware' ? `Pi ${piIp}` : 'Bridge'}
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
