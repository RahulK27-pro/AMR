import { useState, useEffect } from 'react';
import bridge from '../services/amrBridge';

/**
 * StatusBar — top header bar showing connection state, bridge URL, and key global info.
 */
export default function StatusBar({ telemetry }) {
  const [status, setStatus] = useState('connecting');
  const [host, setHost]     = useState(window.location.hostname);
  const [port, setPort]     = useState('8000');
  const [editing, setEditing] = useState(false);

  useEffect(() => {
    bridge.connect(host, Number(port));
    const unsub = bridge.onStatusChange(setStatus);
    return unsub;
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
      <span className="status-bar__logo">⬡ AMR CONTROL</span>

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
