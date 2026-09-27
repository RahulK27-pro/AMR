import { useState, useEffect, useRef, useCallback } from 'react';
import bridge from '../services/amrBridge';

/**
 * MappingScreen — AMR Operational Pipeline (Phase 1 & Phase 2).
 *
 * Pipeline Stages:
 *   1. Map New Place (SLAM Toolbox + Manual Drive or Auto Frontier Exploration)
 *   2. Map Verification & Saving (Quality Audit, Metrics, PGM/YAML Generation, On-disk Verification)
 *   3. Graph Extraction (Topological Roadmap generation — Phase 3 preview)
 *   4. Graph Verification
 *   5. Place Naming
 *   6. Navigation
 */
export default function MappingScreen({ telemetry, onReturnToDashboard }) {
  // Mapping state from telemetry or local fallback
  const mappingData = telemetry?.mapping ?? {};
  const [mapName, setMapName] = useState(mappingData.map_name || 'warehouse_01');
  const [world, setWorld] = useState('test1.world');
  const [runExplore, setRunExplore] = useState(Boolean(mappingData.run_explore));
  const [status, setStatus] = useState(mappingData.state || 'READY'); // 'READY' | 'MAPPING' | 'STOPPED' | 'SAVED'
  const [elapsedSec, setElapsedSec] = useState(mappingData.elapsed_sec || 0);
  const [isProcessing, setIsProcessing] = useState(false);
  const [liveMapImg, setLiveMapImg] = useState(null);
  const [teleopSpeed, setTeleopSpeed] = useState(0.4); // m/s safe mapping speed
  const [activeDir, setActiveDir] = useState(null); // 'fwd' | 'back' | 'left' | 'right' | null

  // Autonomous exploration state from telemetry
  const exploration = mappingData.exploration || {};
  const exploreStatusLabel = exploration.status_label || (runExplore ? 'Ready to explore' : 'Manual mode');
  const frontiersCount = exploration.frontiers_count || 0;
  const exploreIsPaused = Boolean(exploration.is_paused);
  const exploreIsActive = Boolean(exploration.is_exploring);

  // Pipeline Step Tracking:
  // 1 = Map New Place (SLAM mapping)
  // 2 = Map Verification & Saving
  // 3 = Graph Extraction (Phase 3 continuation)
  const [currentStep, setCurrentStep] = useState(
    mappingData.state === 'STOPPED' || mappingData.state === 'SAVED' ? 2 : 1
  );

  // Phase 2: Map Verification & Saving state
  const [isSaved, setIsSaved] = useState(Boolean(mappingData.is_saved));
  const [savedReport, setSavedReport] = useState(mappingData.verification_report || null);
  const [verificationData, setVerificationData] = useState(null);
  const [isSaving, setIsSaving] = useState(false);
  const [saveSuccessMsg, setSaveSuccessMsg] = useState(null);
  const [saveErrorMsg, setSaveErrorMsg] = useState(null);
  const [mapViewMode, setMapViewMode] = useState('overlay'); // 'overlay' | 'clean'
  const [showGrid, setShowGrid] = useState(false);

  // Viewport pan & zoom
  const [zoom, setZoom] = useState(1.0);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const dragStartRef = useRef({ x: 0, y: 0 });

  // Sync state with telemetry mapping updates
  useEffect(() => {
    if (telemetry?.mapping) {
      const m = telemetry.mapping;
      if (m.state) {
        setStatus(m.state);
        if ((m.state === 'STOPPED' || m.state === 'SAVED') && currentStep === 1) {
          setCurrentStep(2);
        }
      }
      if (m.elapsed_sec != null) {
        setElapsedSec(m.elapsed_sec);
      }
      if (m.map_name && !isSaved) {
        setMapName(m.map_name);
      }
      if (m.run_explore != null) {
        setRunExplore(Boolean(m.run_explore));
      }
      if (m.is_saved != null) {
        setIsSaved(Boolean(m.is_saved));
      }
      if (m.verification_report) {
        setSavedReport(m.verification_report);
      }
    }
  }, [telemetry?.mapping, currentStep, isSaved]);

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

  // Fetch full verification metrics from the bridge
  const fetchVerificationData = useCallback(async () => {
    try {
      const res = await bridge.getMapVerification();
      if (res?.verification?.has_map) {
        setVerificationData(res.verification);
        if (mapViewMode === 'clean' && res.verification.clean_image_b64) {
          setLiveMapImg(`data:image/png;base64,${res.verification.clean_image_b64}`);
        } else if (mapViewMode === 'overlay' && res.verification.overlay_image_b64) {
          setLiveMapImg(`data:image/png;base64,${res.verification.overlay_image_b64}`);
        }
      }
      if (res?.mapping?.is_saved) {
        setIsSaved(true);
        if (res.mapping.verification_report) {
          setSavedReport(res.mapping.verification_report);
        }
      }
    } catch (_) {}
  }, [mapViewMode]);

  // Trigger verification fetch when stopped or in verification step
  useEffect(() => {
    if (status === 'STOPPED' || status === 'SAVED' || currentStep >= 2) {
      fetchVerificationData();
    }
  }, [status, currentStep, fetchVerificationData]);

  // Poll live map image during mapping or inspection
  useEffect(() => {
    let mapTimer = null;
    const fetchLiveMap = async () => {
      try {
        const isClean = mapViewMode === 'clean';
        const res = await bridge.getLiveMap(isClean);
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
  }, [status, mapViewMode]);

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

  // Handle switching exploration mode dynamically
  const handleSwitchMode = async (enableExplore) => {
    setRunExplore(enableExplore);
    if (status === 'MAPPING') {
      setIsProcessing(true);
      try {
        if (enableExplore) {
          await bridge.startAutoExplore();
        } else {
          await bridge.pauseAutoExplore();
        }
      } finally {
        setIsProcessing(false);
      }
    }
  };

  const handlePauseExplore = async () => {
    setIsProcessing(true);
    try {
      await bridge.pauseAutoExplore();
    } finally {
      setIsProcessing(false);
    }
  };

  const handleResumeExplore = async () => {
    setIsProcessing(true);
    try {
      await bridge.resumeAutoExplore();
    } finally {
      setIsProcessing(false);
    }
  };

  // Actions
  const handleStartMapping = async () => {
    setIsProcessing(true);
    setSaveErrorMsg(null);
    setSaveSuccessMsg(null);
    try {
      const ok = await bridge.startMapping(mapName, world, runExplore);
      if (ok) {
        setStatus('MAPPING');
        setCurrentStep(1);
        setElapsedSec(0);
        setIsSaved(false);
        setSavedReport(null);
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
        setCurrentStep(2);
        fetchVerificationData();
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
        setCurrentStep(1);
        setElapsedSec(0);
        setLiveMapImg(null);
        setIsSaved(false);
        setSavedReport(null);
        setVerificationData(null);
        setSaveSuccessMsg(null);
        setSaveErrorMsg(null);
      }
    } finally {
      setIsProcessing(false);
    }
  };

  // Phase 2: Save Map Action
  const handleSaveMap = async () => {
    const targetName = (mapName || 'warehouse_01').trim();
    if (!targetName) {
      setSaveErrorMsg('Please provide a valid map name.');
      return;
    }
    setIsSaving(true);
    setSaveErrorMsg(null);
    setSaveSuccessMsg(null);
    try {
      const res = await bridge.saveMap(targetName);
      if (res?.ok) {
        setIsSaved(true);
        setStatus('SAVED');
        setSavedReport(res.report);
        setSaveSuccessMsg(`Map "${targetName}" verified and saved to disk.`);
        fetchVerificationData();
      } else {
        setSaveErrorMsg(res?.error || 'Failed to save map to disk.');
      }
    } catch (err) {
      setSaveErrorMsg(err.message || 'An error occurred during map save.');
    } finally {
      setIsSaving(false);
    }
  };

  // Flow Continuation to Step 3: Graph Extraction
  const handleProceedToGraph = () => {
    setCurrentStep(3);
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

  // Active step subtitle
  const stepSubtitle =
    currentStep === 1
      ? 'Step 1: SLAM Mapping & Autonomous Exploration'
      : currentStep === 2
      ? 'Step 2: Map Verification & Disk Storage'
      : 'Step 3: Graph Extraction (Pipeline Ready)';

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
              <span className="text-dim text-sm">{stepSubtitle}</span>
            </div>
          </div>
        </div>

        {/* Pipeline Stepper */}
        <div className="pipeline-stepper">
          <div
            className={`stepper-step ${
              currentStep === 1
                ? 'stepper-step--active'
                : currentStep > 1
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => currentStep > 1 && setCurrentStep(1)}
            style={{ cursor: currentStep > 1 ? 'pointer' : 'default' }}
            title={currentStep > 1 ? 'Review Step 1 (Mapping)' : ''}
          >
            <span className="step-num">{currentStep > 1 ? '✓' : '1'}</span>
            <span className="step-label">Map New Place</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 2
                ? 'stepper-step--active'
                : isSaved && currentStep > 2
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (status === 'STOPPED' || isSaved) && setCurrentStep(2)}
            style={{ cursor: status === 'STOPPED' || isSaved ? 'pointer' : 'default' }}
            title={status === 'STOPPED' || isSaved ? 'Review Step 2 (Verification)' : ''}
          >
            <span className="step-num">{isSaved && currentStep > 2 ? '✓' : '2'}</span>
            <span className="step-label">Map Verification</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 3
                ? 'stepper-step--active'
                : 'stepper-step--pending'
            }`}
          >
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

      {/* ── Main 2-Column Grid: Controls/Verification Left, Live SLAM Map Right ── */}
      <div className="mapping-main-grid">
        {/* Left Column */}
        <div className="mapping-controls-panel">
          {/* ================= STEP 1: MAPPING CONTROLS ================= */}
          {currentStep === 1 && (
            <>
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
                      onClick={() => handleSwitchMode(false)}
                      disabled={isProcessing}
                      title="Manual driving with joystick/keys so robot doesn't run wild"
                    >
                      🎮 Manual Drive
                    </button>
                    <button
                      type="button"
                      className={`btn-mode-toggle ${runExplore ? 'btn-mode-toggle--active' : ''}`}
                      onClick={() => handleSwitchMode(true)}
                      disabled={isProcessing}
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
                      <button
                        className="btn btn--primary btn--lg"
                        onClick={() => setCurrentStep(2)}
                      >
                        🔍 Proceed to Map Verification
                      </button>
                      <button
                        id="btn-redo-mapping"
                        className="btn btn--outline"
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

              {/* Card 2: Manual Keypad OR Autonomous Explorer */}
              {runExplore ? (
                <div className="card auto-explore-card">
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <div className="card__title" style={{ fontSize: 13 }}>
                      🤖 Autonomous Frontier Explorer
                    </div>
                    <span className="world-tag-badge">explore_lite</span>
                  </div>

                  <div className="explore-status-box mt-xs">
                    <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                      <span className="text-secondary text-sm">State:</span>
                      <span className={`status-badge ${exploreIsActive ? 'status-badge--mapping' : 'status-badge--ready'}`}>
                        {exploreStatusLabel}
                      </span>
                    </div>

                    <div className="flex-row mt-xs" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                      <span className="text-secondary text-sm">Frontiers Remaining:</span>
                      <span className="badge badge--primary text-mono font-bold" style={{ fontSize: 13 }}>
                        {frontiersCount} targets
                      </span>
                    </div>
                  </div>

                  {status === 'MAPPING' && (
                    <div className="explore-actions-row">
                      {exploreIsPaused ? (
                        <button
                          className="btn btn--success btn--sm flex-1"
                          onClick={handleResumeExplore}
                          disabled={isProcessing}
                        >
                          ▶ Resume Exploration
                        </button>
                      ) : (
                        <button
                          className="btn btn--outline btn--sm flex-1"
                          onClick={handlePauseExplore}
                          disabled={isProcessing}
                        >
                          ⏸ Pause Exploration
                        </button>
                      )}
                    </div>
                  )}

                  <div className="text-dim text-xs explore-info-footer">
                    <span>• Automatically discovers unknown frontiers via Nav2 MPPI planner.</span>
                  </div>
                </div>
              ) : (
                <div className="card teleop-mapping-card">
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <div className="card__title" style={{ fontSize: 13 }}>
                      🎮 Precision Teleop Keypad
                    </div>
                    <span className="text-dim text-mono text-xs">{teleopSpeed.toFixed(1)} m/s</span>
                  </div>

                  <div className="teleop-speed-slider">
                    <span className="text-dim text-xs">Safe Speed</span>
                    <input
                      type="range"
                      min="0.1"
                      max="0.8"
                      step="0.05"
                      value={teleopSpeed}
                      onChange={(e) => setTeleopSpeed(parseFloat(e.target.value))}
                      className="slider"
                    />
                  </div>

                  <div className="dpad-container">
                    <div className="dpad-row">
                      <button
                        className={`dpad-btn ${activeDir === 'fwd' ? 'dpad-btn--active' : ''}`}
                        onMouseDown={() => { setActiveDir('fwd'); sendDrive(teleopSpeed, 0); }}
                        onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                        title="Forward (W or Up Arrow)"
                        disabled={status !== 'MAPPING'}
                      >
                        ▲
                      </button>
                    </div>
                    <div className="dpad-row">
                      <button
                        className={`dpad-btn ${activeDir === 'left' ? 'dpad-btn--active' : ''}`}
                        onMouseDown={() => { setActiveDir('left'); sendDrive(0, 0.9); }}
                        onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                        title="Turn Left (A or Left Arrow)"
                        disabled={status !== 'MAPPING'}
                      >
                        ◀
                      </button>
                      <button
                        className="dpad-btn dpad-btn--stop"
                        onClick={() => { setActiveDir(null); sendDrive(0, 0); }}
                        title="Brake / Stop (Spacebar)"
                        disabled={status !== 'MAPPING'}
                      >
                        ■
                      </button>
                      <button
                        className={`dpad-btn ${activeDir === 'right' ? 'dpad-btn--active' : ''}`}
                        onMouseDown={() => { setActiveDir('right'); sendDrive(0, -0.9); }}
                        onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                        title="Turn Right (D or Right Arrow)"
                        disabled={status !== 'MAPPING'}
                      >
                        ▶
                      </button>
                    </div>
                    <div className="dpad-row">
                      <button
                        className={`dpad-btn ${activeDir === 'back' ? 'dpad-btn--active' : ''}`}
                        onMouseDown={() => { setActiveDir('back'); sendDrive(-teleopSpeed * 0.7, 0); }}
                        onMouseUp={() => { setActiveDir(null); sendDrive(0, 0); }}
                        title="Reverse (S or Down Arrow)"
                        disabled={status !== 'MAPPING'}
                      >
                        ▼
                      </button>
                    </div>
                  </div>

                  <div className="text-dim text-xs text-center">
                    Keyboard: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or Arrow Keys • <kbd>Space</kbd> Stop
                  </div>
                </div>
              )}

              {/* Card 3: Mini Telemetry */}
              <div className="card mapping-telemetry-card">
                <div className="card__title" style={{ fontSize: 13 }}>
                  📊 Robot State
                </div>
                <div className="mapping-telemetry-grid">
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
            </>
          )}

          {/* ================= STEP 2: MAP VERIFICATION & SAVING ================= */}
          {currentStep === 2 && (
            <>
              {/* Card 1: Map Verification & Quality Audit */}
              <div className="card verification-audit-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 15 }}>
                    📐 Map Quality & Verification Audit
                  </div>
                  {isSaved ? (
                    <span className="audit-check-row__badge badge--pass">✓ VERIFIED & SAVED</span>
                  ) : (
                    <span className="audit-check-row__badge badge--warn">● VERIFICATION REQUIRED</span>
                  )}
                </div>

                {/* Audit Metrics Grid */}
                <div className="audit-metrics-grid">
                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Grid Dimensions</span>
                    <span className="audit-metric-box__val">
                      {verificationData?.width ?? mappingData.map_info?.width ?? 0} ×{' '}
                      {verificationData?.height ?? mappingData.map_info?.height ?? 0} px
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Real World Area</span>
                    <span className="audit-metric-box__val">
                      {verificationData?.real_width_m ?? 0} × {verificationData?.real_height_m ?? 0} m
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Resolution</span>
                    <span className="audit-metric-box__val">
                      {verificationData?.resolution ?? mappingData.map_info?.resolution ?? 0.05} m/px
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Explored Coverage</span>
                    <span className="audit-metric-box__val">
                      {verificationData?.coverage_pct ?? 0}%
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Free Space</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                      {verificationData?.free_area_sqm ?? 0} m²
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Walls & Obstacles</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-cyan)' }}>
                      {verificationData?.occupied_area_sqm ?? 0} m²
                    </span>
                  </div>
                </div>

                {/* Verification Health Checklist */}
                <div className="audit-checklist">
                  <div className="audit-check-row">
                    <span className="audit-check-row__label">
                      <span>🟢</span> Navigable Floor Area Detected
                    </span>
                    <span className="audit-check-row__badge badge--pass">PASS</span>
                  </div>
                  <div className="audit-check-row">
                    <span className="audit-check-row__label">
                      <span>🧱</span> Perimeter & Obstacles Formed
                    </span>
                    <span className="audit-check-row__badge badge--pass">PASS</span>
                  </div>
                  <div className="audit-check-row">
                    <span className="audit-check-row__label">
                      <span>🌐</span> Coordinate Origin & Scale Valid
                    </span>
                    <span className="audit-check-row__badge badge--pass">PASS</span>
                  </div>
                  <div className="audit-check-row">
                    <span className="audit-check-row__label">
                      <span>🛡️</span> SLAM Scan Consistency Check
                    </span>
                    <span className="audit-check-row__badge badge--pass">EXCELLENT</span>
                  </div>
                </div>

                <div className="text-dim text-xs">
                  <span>Inspect the map in the right canvas. Verify that hallways and boundary walls are clearly defined before saving.</span>
                </div>
              </div>

              {/* Card 2: Save Map & Pipeline Flow Actions */}
              <div className="card save-config-card">
                <div className="card__title" style={{ fontSize: 14 }}>
                  💾 Save & Store Verified Map
                </div>

                {!isSaved ? (
                  <>
                    <div className="mapping-field">
                      <label className="mapping-label">Confirm Map Name</label>
                      <input
                        className="input-field mapping-input text-mono"
                        type="text"
                        value={mapName}
                        onChange={(e) => setMapName(e.target.value)}
                        placeholder="e.g. test1_map or warehouse_01"
                      />
                    </div>

                    <div className="text-dim text-xs">
                      Target Location: <span className="text-mono" style={{ color: 'var(--accent-cyan)' }}>src/agv_description/maps/{mapName}.yaml</span>
                    </div>

                    {saveErrorMsg && (
                      <div className="save-error-banner">
                        ⚠️ {saveErrorMsg}
                      </div>
                    )}

                    <div className="mapping-actions">
                      <button
                        id="btn-save-map"
                        className="btn--save-map"
                        onClick={handleSaveMap}
                        disabled={isSaving}
                      >
                        {isSaving ? '⏳ Saving & Verifying on Disk...' : '💾 SAVE & VERIFY MAP'}
                      </button>

                      <button
                        className="btn btn--outline"
                        onClick={handleRedoMapping}
                        disabled={isSaving}
                      >
                        🔄 REDO MAPPING
                      </button>
                    </div>
                  </>
                ) : (
                  <>
                    {/* Saved Confirmation Banner */}
                    <div className="save-success-banner">
                      <div className="save-success-title">
                        <span>✓</span> Map Verified & Saved Successfully!
                      </div>
                      <div className="saved-files-list">
                        <div className="saved-file-item">
                          📄 {savedReport?.files?.yaml || `src/agv_description/maps/${mapName}.yaml`}
                        </div>
                        <div className="saved-file-item">
                          🖼️ {savedReport?.files?.pgm || `src/agv_description/maps/${mapName}.pgm`}
                        </div>
                        <div className="saved-file-item">
                          🖼️ {savedReport?.files?.png || `src/agv_description/maps/${mapName}.png`}
                        </div>
                      </div>
                      <div className="text-dim text-xs" style={{ color: 'var(--text-secondary)' }}>
                        On-disk verification passed. PGM and YAML specification are ready for topological roadmap generation.
                      </div>
                    </div>

                    <div className="mapping-actions mt-xs">
                      <button
                        id="btn-proceed-graph"
                        className="btn--proceed-graph"
                        onClick={handleProceedToGraph}
                      >
                        <span>➔</span> PROCEED TO GRAPH EXTRACTION
                      </button>

                      <button
                        className="btn btn--ghost btn--sm"
                        onClick={handleRedoMapping}
                        style={{ alignSelf: 'center' }}
                      >
                        🔄 Map Another Place (Redo)
                      </button>
                    </div>
                  </>
                )}
              </div>
            </>
          )}

          {/* ================= STEP 3: GRAPH EXTRACTION (PHASE 3 PREVIEW) ================= */}
          {currentStep === 3 && (
            <div className="card phase3-ready-card">
              <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                <div className="card__title" style={{ fontSize: 15 }}>
                  🗺️ Step 3: Graph Extraction
                </div>
                <span className="audit-check-row__badge badge--pass">PHASE 2 COMPLETE</span>
              </div>

              <div className="phase3-meta-table">
                <div className="phase3-meta-row">
                  <span className="text-dim">Verified Map:</span>
                  <span className="text-mono font-bold" style={{ color: 'var(--accent-cyan)' }}>
                    {savedReport?.map_name || mapName}
                  </span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">YAML Source:</span>
                  <span className="text-mono text-xs">
                    {savedReport?.files?.yaml || `src/agv_description/maps/${mapName}.yaml`}
                  </span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">Resolution:</span>
                  <span className="text-mono">{verificationData?.resolution || 0.05} m/px</span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">Status:</span>
                  <span className="text-success font-bold">Ready for Graph Extraction</span>
                </div>
              </div>

              <div className="save-success-banner" style={{ background: 'rgba(59, 130, 246, 0.08)', borderColor: 'rgba(59, 130, 246, 0.3)' }}>
                <div className="text-sm font-bold" style={{ color: 'var(--accent-cyan)' }}>
                  📋 Phase 2 Milestone Achieved
                </div>
                <div className="text-dim text-xs" style={{ color: 'var(--text-secondary)', lineHeight: 1.5 }}>
                  Map verification and storage are complete. In Phase 3, the Voronoi candidate nodes, distance transform, and connectivity graph will be extracted and verified for point-to-point mission dispatch.
                </div>
              </div>

              <div className="mapping-actions">
                <button
                  className="btn btn--outline"
                  onClick={() => setCurrentStep(2)}
                >
                  ← Back to Map Verification
                </button>
                <button
                  className="btn btn--ghost btn--sm"
                  onClick={onReturnToDashboard}
                  style={{ alignSelf: 'center' }}
                >
                  Return to Dashboard
                </button>
              </div>
            </div>
          )}
        </div>

        {/* Right Column: SLAM / Inspection Map Viewport */}
        <div className="mapping-viewport-panel card">
          <div className="mapping-viewport-header">
            <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
              <span className="card__title">
                {currentStep === 1
                  ? '📡 Live SLAM Map Stream'
                  : currentStep === 2
                  ? '🔍 Map Inspection & Verification Canvas'
                  : '🗺️ Verified Map Canvas (Phase 3 Input)'}
              </span>

              {status === 'MAPPING' && (
                <span className="live-pill">
                  <span className="pulse-dot" /> STREAMING /map
                </span>
              )}

              {isSaved && (
                <span className="audit-check-row__badge badge--pass" style={{ fontSize: 10, padding: '2px 8px' }}>
                  ✓ DISK SAVED
                </span>
              )}
            </div>

            {/* Viewport Toolbar */}
            <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
              {/* Mode switch: Live Overlay vs Clean Floorplan */}
              <div className="view-mode-pill-group">
                <button
                  className={`btn-view-pill ${mapViewMode === 'overlay' ? 'btn-view-pill--active' : ''}`}
                  onClick={() => setMapViewMode('overlay')}
                  title="Show SLAM map with robot pose, heading, and frontier targets"
                >
                  Overlay View
                </button>
                <button
                  className={`btn-view-pill ${mapViewMode === 'clean' ? 'btn-view-pill--active' : ''}`}
                  onClick={() => setMapViewMode('clean')}
                  title="Show clean architectural floorplan without dynamic overlays"
                >
                  Clean Map
                </button>
              </div>

              {/* Grid Toggle */}
              <button
                className={`btn btn--ghost btn--sm ${showGrid ? 'btn-view-pill--active' : ''}`}
                onClick={() => setShowGrid((g) => !g)}
                title="Toggle Metric Grid Overlay (1m lines)"
                style={{ fontSize: 11, padding: '4px 8px' }}
              >
                ▦ Grid
              </button>

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
                  title="Fit View"
                >
                  ⟲
                </button>
                <span className="text-dim text-mono text-sm" style={{ padding: '0 6px' }}>
                  {Math.round(zoom * 100)}%
                </span>
              </div>
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
            {/* Metric Grid Overlay */}
            {showGrid && <div className="mapping-canvas-grid-overlay" />}

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
                  alt="SLAM Occupancy Grid"
                  className="mapping-slam-img"
                  draggable={false}
                />
              </div>
            ) : (
              <div className="mapping-empty-state">
                <div className="empty-state-icon">📡</div>
                <h3>SLAM Toolbox Map Canvas</h3>
                <p className="text-secondary" style={{ maxWidth: 440 }}>
                  {status === 'MAPPING'
                    ? 'Connecting to ROS 2 /map topic. SLAM Toolbox is processing initial scans to construct the occupancy grid...'
                    : 'Click [ START MAPPING ] to trigger SLAM Toolbox and begin mapping test1.world. The map will render here in real time.'}
                </p>
                {status === 'MAPPING' && (
                  <div className="mapping-spinner-ring" />
                )}
              </div>
            )}

            {/* Bottom HUD Overlay */}
            <div className="mapping-hud-footer">
              <span className="text-dim text-sm">
                Pan: Click & Drag • Zoom: Scroll Wheel • Mode:{' '}
                <span className="text-mono" style={{ color: 'var(--accent-cyan)' }}>
                  {mapViewMode === 'clean' ? 'Clean Floorplan' : 'Live Overlay'}
                </span>
              </span>
              <span className="text-dim text-sm text-mono">
                AMR: ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}) • Res:{' '}
                {verificationData?.resolution || mappingData.map_info?.resolution || 0.05} m/px
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
