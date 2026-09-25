import { useState, useEffect, useRef, useCallback } from 'react';
import bridge from '../services/amrBridge';

/**
 * MappingScreen — Dedicated "Map New Place" workflow (Phase 1).
 * Features:
 *   - Pipeline Stepper (Phase 1 active)
 *   - Map Name & Target World input (default: test1.world)
 *   - Mode selection: Manual Drive (Teleop Keypad/WASD) vs Autonomous Exploration
 *   - Status badge (● Ready / ● Mapping in Progress / ● Stopped)
 *   - Live Elapsed Time timer (MM:SS)
 *   - Map Status: "Receiving map data..." indicator
 *   - Action Controls: [ START MAPPING ], [ STOP MAPPING ], [ REDO MAPPING ]
 *   - Dedicated Teleoperation Keypad for stable, controlled manual mapping (no running wild)
 *   - Real-time SLAM Map Viewport with pan, zoom (+ / - / reset) & live /map stream
 */
export default function MappingScreen({ telemetry, onReturnToDashboard }) {
  // Mapping state from telemetry or local fallback
  const mappingData = telemetry?.mapping ?? {};
  const [mapName, setMapName] = useState(mappingData.map_name || 'warehouse_01');
  const [world, setWorld] = useState('test1.world');
  const [runExplore, setRunExplore] = useState(false); // default: false (manual teleop mapping, no wild runs)
  const [status, setStatus] = useState(mappingData.state || 'READY'); // 'READY' | 'MAPPING' | 'STOPPED'
  const [elapsedSec, setElapsedSec] = useState(mappingData.elapsed_sec || 0);
  const [isProcessing, setIsProcessing] = useState(false);
  const [liveMapImg, setLiveMapImg] = useState(null);
  const [teleopSpeed, setTeleopSpeed] = useState(0.4); // m/s safe mapping speed
  const [activeDir, setActiveDir] = useState(null); // 'fwd' | 'back' | 'left' | 'right' | null

  // Viewport pan & zoom
  const [zoom, setZoom] = useState(1.0);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const dragStartRef = useRef({ x: 0, y: 0 });

  // Sync state with telemetry mapping updates
  useEffect(() => {
    if (telemetry?.mapping) {
      if (telemetry.mapping.state) {
        setStatus(telemetry.mapping.state);
      }
      if (telemetry.mapping.elapsed_sec != null) {
        setElapsedSec(telemetry.mapping.elapsed_sec);
      }
      if (telemetry.mapping.map_name) {
        setMapName(telemetry.mapping.map_name);
      }
    }
  }, [telemetry?.mapping]);

  // Local ticker for smooth second-by-second elapsed time when MAPPING
  useEffect(() => {
    let timer = null;
    if (status === 'MAPPING') {
      timer = setInterval(() => {
        setElapsedSec((prev) => prev + 1);
      }, 1000);
    }
    return () => {
      if (timer) clearInterval(timer);
    };
  }, [status]);

  // Poll live map image during mapping
  useEffect(() => {
    let mapTimer = null;
    const fetchLiveMap = async () => {
      try {
        const res = await bridge.getLiveMap();
        if (res?.has_data && res.image) {
          setLiveMapImg(`data:image/png;base64,${res.image}`);
        }
      } catch (_) {}
    };

    fetchLiveMap();
    if (status === 'MAPPING') {
      mapTimer = setInterval(fetchLiveMap, 1500); // 1.5s refresh for occupancy grid
    }
    return () => {
      if (mapTimer) clearInterval(mapTimer);
    };
  }, [status]);

  // Teleop command helper
  const sendDrive = useCallback((linear, angular) => {
    bridge.sendCmdVel(linear, angular);
  }, []);

  // Keyboard driving controls (W/A/S/D & Arrow keys) during mapping
  useEffect(() => {
    if (status !== 'MAPPING' || runExplore) return;

    const handleKeyDown = (e) => {
      if (e.target.tagName === 'INPUT') return;
      if (e.repeat) return;
      switch (e.key.toLowerCase()) {
        case 'w':
        case 'arrowup':
          e.preventDefault();
          setActiveDir('fwd');
          sendDrive(teleopSpeed, 0);
          break;
        case 's':
        case 'arrowdown':
          e.preventDefault();
          setActiveDir('back');
          sendDrive(-teleopSpeed * 0.7, 0);
          break;
        case 'a':
        case 'arrowleft':
          e.preventDefault();
          setActiveDir('left');
          sendDrive(0, 0.9);
          break;
        case 'd':
        case 'arrowright':
          e.preventDefault();
          setActiveDir('right');
          sendDrive(0, -0.9);
          break;
        case ' ':
          e.preventDefault();
          setActiveDir(null);
          sendDrive(0, 0);
          break;
        default:
          break;
      }
    };

    const handleKeyUp = (e) => {
      if (e.target.tagName === 'INPUT') return;
      if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(e.key.toLowerCase())) {
        setActiveDir(null);
        sendDrive(0, 0);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [status, runExplore, teleopSpeed, sendDrive]);

  // Format MM:SS
  const formatTime = (secs) => {
    const m = Math.floor(secs / 60);
    const s = Math.floor(secs % 60);
    return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  };

  // Actions
  const handleStartMapping = async () => {
    setIsProcessing(true);
    try {
      const ok = await bridge.startMapping(mapName, world, runExplore);
      if (ok) {
        setStatus('MAPPING');
        setElapsedSec(0);
      }
    } finally {
      setIsProcessing(false);
    }
  };

  const handleStopMapping = async () => {
    setIsProcessing(true);
    try {
      sendDrive(0, 0);
      const ok = await bridge.stopMapping();
      if (ok) {
        setStatus('STOPPED');
      }
    } finally {
      setIsProcessing(false);
    }
  };

  const handleRedoMapping = async () => {
    setIsProcessing(true);
    try {
      sendDrive(0, 0);
      const ok = await bridge.redoMapping();
      if (ok) {
        setStatus('READY');
        setElapsedSec(0);
        setLiveMapImg(null);
      }
    } finally {
      setIsProcessing(false);
    }
  };

  // Canvas pan & zoom handlers
  const handleWheel = (e) => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.15 : 0.85;
    setZoom((z) => Math.min(Math.max(z * factor, 0.4), 5.0));
  };

  const handleMouseDown = (e) => {
    if (e.button === 0 || e.button === 1) {
      setIsDragging(true);
      dragStartRef.current = { x: e.clientX - pan.x, y: e.clientY - pan.y };
    }
  };

  const handleMouseMove = (e) => {
    if (isDragging) {
      setPan({
        x: e.clientX - dragStartRef.current.x,
        y: e.clientY - dragStartRef.current.y,
      });
    }
  };

  const handleMouseUp = () => setIsDragging(false);

  // Status badges & text
  const hasMapData = mappingData.has_map_data || Boolean(liveMapImg);
  const mapStatusText = hasMapData
    ? (mappingData.map_status_text || 'Receiving map data from SLAM Toolbox (/map active)')
    : status === 'MAPPING'
    ? 'Waiting for SLAM Toolbox to publish /map...'
    : 'Ready to start mapping';

  const pose = telemetry?.pose ?? { x: 0, y: 0, yaw: 0 };
  const vel = telemetry?.velocity ?? { linear: 0, angular: 0 };
  const minObs = telemetry?.min_obstacle_dist;

  return (
    <div className="mapping-workflow-container">
      {/* ── Pipeline Progress Header ── */}
      <div className="mapping-pipeline-header">
        <div className="mapping-pipeline-header__left">
          <button
            className="btn btn--outline"
            onClick={onReturnToDashboard}
            style={{ fontSize: 13, gap: 6, padding: '6px 14px' }}
          >
            ← Back to Dashboard
          </button>
          <div className="pipeline-title">
            <span className="pipeline-title__icon">🗺️</span>
            <div>
              <h2>AMR Operational Pipeline</h2>
              <span className="text-dim text-sm">Step 1: SLAM Mapping & Exploration</span>
            </div>
          </div>
        </div>

        {/* Pipeline Stepper */}
        <div className="pipeline-stepper">
          <div className="stepper-step stepper-step--active">
            <span className="step-num">1</span>
            <span className="step-label">Map New Place</span>
          </div>
          <div className="stepper-arrow">➔</div>
          <div className="stepper-step stepper-step--pending">
            <span className="step-num">2</span>
            <span className="step-label">Map Verification</span>
          </div>
          <div className="stepper-arrow">➔</div>
          <div className="stepper-step stepper-step--pending">
            <span className="step-num">3</span>
            <span className="step-label">Graph Extraction</span>
          </div>
          <div className="stepper-arrow">➔</div>
          <div className="stepper-step stepper-step--pending">
            <span className="step-num">4</span>
            <span className="step-label">Graph Verification</span>
          </div>
          <div className="stepper-arrow">➔</div>
          <div className="stepper-step stepper-step--pending">
            <span className="step-num">5</span>
            <span className="step-label">Place Naming</span>
          </div>
          <div className="stepper-arrow">➔</div>
          <div className="stepper-step stepper-step--pending">
            <span className="step-num">6</span>
            <span className="step-label">Navigation</span>
          </div>
        </div>
      </div>

      {/* ── Main 2-Column Grid: Controls Left, Live SLAM Map Right ── */}
      <div className="mapping-main-grid">
        {/* Left Column: Controls & Teleop */}
        <div className="mapping-controls-panel">
          {/* Card 1: Setup & Status */}
          <div className="card mapping-card">
            <div className="card__title" style={{ fontSize: 15 }}>
              ⚙️ Mapping Controls
            </div>

            {/* Map Name input */}
            <div className="mapping-field">
              <label className="mapping-label">Map Name</label>
              <input
                className="input-field mapping-input"
                type="text"
                value={mapName}
                onChange={(e) => setMapName(e.target.value)}
                placeholder="e.g. test1_map or warehouse_01"
                disabled={status === 'MAPPING'}
              />
            </div>

            {/* Target Simulation World */}
            <div className="mapping-field">
              <label className="mapping-label">Simulation World</label>
              <div className="flex-row" style={{ gap: 8 }}>
                <input
                  className="input-field mapping-input text-mono"
                  type="text"
                  value={world}
                  onChange={(e) => setWorld(e.target.value)}
                  placeholder="test1.world"
                  disabled={status === 'MAPPING'}
                />
                <span className="world-tag-badge">SDF World</span>
              </div>
            </div>

            {/* Exploration Mode */}
            <div className="mapping-field">
              <label className="mapping-label">Exploration Mode</label>
              <div className="mode-toggle-group">
                <button
                  type="button"
                  className={`btn-mode-toggle ${!runExplore ? 'btn-mode-toggle--active' : ''}`}
                  onClick={() => setRunExplore(false)}
                  disabled={status === 'MAPPING'}
                  title="Manual driving with joystick/keys so robot doesn't run wild"
                >
                  🎮 Manual Drive
                </button>
                <button
                  type="button"
                  className={`btn-mode-toggle ${runExplore ? 'btn-mode-toggle--active' : ''}`}
                  onClick={() => setRunExplore(true)}
                  disabled={status === 'MAPPING'}
                  title="Autonomous frontier exploration with explore_lite"
                >
                  🤖 Auto Explore
                </button>
              </div>
            </div>

            {/* Mapping Status Badge */}
            <div className="mapping-field">
              <label className="mapping-label">Mapping Status</label>
              <div className="mapping-status-box">
                {status === 'READY' && (
                  <span className="status-badge status-badge--ready">
                    <span className="dot dot--connected" />
                    Ready
                  </span>
                )}
                {status === 'MAPPING' && (
                  <span className="status-badge status-badge--mapping">
                    <span className="dot dot--pulsing" />
                    Mapping in Progress
                  </span>
                )}
                {status === 'STOPPED' && (
                  <span className="status-badge status-badge--stopped">
                    <span className="dot dot--warning" />
                    Mapping Stopped
                  </span>
                )}
              </div>
            </div>

            {/* Elapsed Time */}
            <div className="mapping-field">
              <label className="mapping-label">Elapsed Time</label>
              <div className="mapping-timer">
                <span className="mapping-timer__icon">⏱️</span>
                <span className="mapping-timer__digits text-mono">
                  {formatTime(elapsedSec)}
                </span>
                {status === 'MAPPING' && (
                  <span className="mapping-timer__live-tag">LIVE</span>
                )}
              </div>
            </div>

            {/* Map Status Info */}
            <div className="mapping-field">
              <label className="mapping-label">Map Status</label>
              <div className="mapping-info-box">
                <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
                  <span
                    className={`status-dot ${hasMapData ? 'status-dot--green' : 'status-dot--amber'}`}
                  />
                  <span className="text-sm">{mapStatusText}</span>
                </div>
                {hasMapData && mappingData.map_info && (
                  <div className="mapping-meta-details text-dim text-mono">
                    {mappingData.map_info.width} × {mappingData.map_info.height} px •{' '}
                    {mappingData.map_info.resolution} m/px
                  </div>
                )}
              </div>
            </div>

            {/* Action Buttons */}
            <div className="mapping-actions">
              {status === 'READY' && (
                <button
                  id="btn-start-mapping"
                  className="btn btn--primary btn--lg btn--start-mapping"
                  onClick={handleStartMapping}
                  disabled={isProcessing}
                >
                  🚀 START MAPPING
                </button>
              )}

              {status === 'MAPPING' && (
                <button
                  id="btn-stop-mapping"
                  className="btn btn--danger btn--lg btn--stop-mapping"
                  onClick={handleStopMapping}
                  disabled={isProcessing}
                >
                  ⏹ STOP MAPPING
                </button>
              )}

              {status === 'STOPPED' && (
                <div className="flex-col" style={{ gap: 10 }}>
                  <div className="stopped-notice">
                    ✓ Mapping completed. Ready for verification or redo.
                  </div>
                  <button
                    id="btn-redo-mapping"
                    className="btn btn--outline btn--lg"
                    onClick={handleRedoMapping}
                    disabled={isProcessing}
                  >
                    🔄 REDO MAPPING
                  </button>
                </div>
              )}

              {status === 'MAPPING' && (
                <button
                  id="btn-redo-mapping-active"
                  className="btn btn--ghost btn--sm"
                  onClick={handleRedoMapping}
                  style={{ alignSelf: 'center', marginTop: 2 }}
                >
                  Restart (Redo)
                </button>
              )}
            </div>
          </div>

          {/* Card 2: Manual Driving Keypad (active during Manual Mode) */}
          {!runExplore && (
            <div className="card teleop-mapping-card">
              <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                <div className="card__title" style={{ fontSize: 13 }}>
                  🎮 Manual Drive Keypad
                </div>
                <span className="text-dim text-xs">WASD / Arrow Keys</span>
              </div>

              {/* Speed Slider */}
              <div className="flex-row mt-xs" style={{ alignItems: 'center', gap: 8 }}>
                <span className="text-dim text-xs">Speed:</span>
                <input
                  type="range"
                  min="0.1"
                  max="0.8"
                  step="0.05"
                  value={teleopSpeed}
                  onChange={(e) => setTeleopSpeed(Number(e.target.value))}
                  style={{ flex: 1 }}
                />
                <span className="text-mono text-xs">{teleopSpeed.toFixed(2)} m/s</span>
              </div>

              {/* Directional Pad */}
              <div className="dpad-grid">
                <div />
                <button
                  className={`btn-dpad ${activeDir === 'fwd' ? 'btn-dpad--active' : ''}`}
                  onMouseDown={() => { setActiveDir('fwd'); sendDrive(teleopSpeed, 0); }}
                  onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                  title="Forward (W)"
                >
                  ▲
                </button>
                <div />

                <button
                  className={`btn-dpad ${activeDir === 'left' ? 'btn-dpad--active' : ''}`}
                  onMouseDown={() => { setActiveDir('left'); sendDrive(0, 0.9); }}
                  onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                  title="Turn Left (A)"
                >
                  ◀
                </button>
                <button
                  className="btn-dpad btn-dpad--stop"
                  onClick={() => { setActiveDir(null); sendDrive(0, 0); }}
                  title="Halt / Stop (Space)"
                >
                  ■
                </button>
                <button
                  className={`btn-dpad ${activeDir === 'right' ? 'btn-dpad--active' : ''}`}
                  onMouseDown={() => { setActiveDir('right'); sendDrive(0, -0.9); }}
                  onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                  title="Turn Right (D)"
                >
                  ▶
                </button>

                <div />
                <button
                  className={`btn-dpad ${activeDir === 'back' ? 'btn-dpad--active' : ''}`}
                  onMouseDown={() => { setActiveDir('back'); sendDrive(-teleopSpeed * 0.7, 0); }}
                  onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                  title="Reverse (S)"
                >
                  ▼
                </button>
                <div />
              </div>
            </div>
          )}

          {/* Card 3: Robot SLAM Telemetry */}
          <div className="card mapping-telemetry-card">
            <div className="card__title" style={{ fontSize: 13 }}>
              🤖 Robot SLAM Telemetry
            </div>
            <div className="mapping-telemetry-grid text-mono">
              <div className="telemetry-item">
                <span className="telemetry-label">X Pose</span>
                <span className="telemetry-val">{pose.x.toFixed(2)} m</span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Y Pose</span>
                <span className="telemetry-val">{pose.y.toFixed(2)} m</span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Yaw</span>
                <span className="telemetry-val">{((pose.yaw * 180) / Math.PI).toFixed(1)}°</span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Linear Vel</span>
                <span className="telemetry-val">{vel.linear.toFixed(2)} m/s</span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Angular Vel</span>
                <span className="telemetry-val">{vel.angular.toFixed(2)} r/s</span>
              </div>
              <div className="telemetry-item">
                <span className="telemetry-label">Min Obstacle</span>
                <span className="telemetry-val">
                  {minObs != null ? `${minObs.toFixed(2)} m` : '—'}
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Right Column: Live SLAM Map Viewport */}
        <div className="mapping-viewport-panel card">
          <div className="mapping-viewport-header">
            <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
              <span className="card__title">📡 Live SLAM Map Stream</span>
              {status === 'MAPPING' && (
                <span className="live-pill">
                  <span className="pulse-dot" /> STREAMING /map
                </span>
              )}
            </div>

            {/* Zoom / Pan controls */}
            <div className="viewport-controls">
              <button
                className="btn btn--ghost btn--icon"
                onClick={() => setZoom((z) => Math.min(z * 1.2, 5.0))}
                title="Zoom In"
              >
                +
              </button>
              <button
                className="btn btn--ghost btn--icon"
                onClick={() => setZoom((z) => Math.max(z * 0.8, 0.4))}
                title="Zoom Out"
              >
                −
              </button>
              <button
                className="btn btn--ghost btn--icon"
                onClick={() => {
                  setZoom(1.0);
                  setPan({ x: 0, y: 0 });
                }}
                title="Reset View"
              >
                ⟲
              </button>
              <span className="text-dim text-mono text-sm" style={{ padding: '0 6px' }}>
                {Math.round(zoom * 100)}%
              </span>
            </div>
          </div>

          {/* Canvas Viewport */}
          <div
            className="mapping-canvas-container"
            onWheel={handleWheel}
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
          >
            {liveMapImg ? (
              <div
                className="mapping-image-wrapper"
                style={{
                  transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
                  transformOrigin: 'center center',
                }}
              >
                <img
                  src={liveMapImg}
                  alt="Live SLAM Occupancy Grid"
                  className="mapping-slam-img"
                  draggable={false}
                />
              </div>
            ) : (
              <div className="mapping-empty-state">
                <div className="empty-state-icon">📡</div>
                <h3>SLAM Toolbox Live Map</h3>
                <p className="text-secondary" style={{ maxWidth: 440 }}>
                  {status === 'MAPPING'
                    ? 'Connecting to ROS 2 /map topic. SLAM Toolbox is processing initial scans to construct the occupancy grid...'
                    : 'Click [ START MAPPING ] to trigger SLAM Toolbox and begin mapping test1.world. The live map will render here in real time.'}
                </p>
                {status === 'MAPPING' && (
                  <div className="mapping-spinner-ring" />
                )}
              </div>
            )}

            {/* Bottom HUD Overlay */}
            <div className="mapping-hud-footer">
              <span className="text-dim text-sm">
                Pan: Click & Drag • Zoom: Scroll Wheel • Topic:{' '}
                <span className="text-mono" style={{ color: 'var(--accent-cyan)' }}>/map</span>
              </span>
              <span className="text-dim text-sm text-mono">
                AMR: ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) • World: {world}
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
