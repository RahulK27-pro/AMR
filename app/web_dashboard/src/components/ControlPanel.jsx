import { useEffect, useMemo, useState } from 'react';
import bridge from '../services/amrBridge';

export default function ControlPanel({
  telemetry,
  selectedNodes = [],
  setSelectedNodes,
  onOpenMapping,
  activeMap = 'hospital_map',
  onMapChange,
  controlMode = 'auto',
  setControlMode,
  mode = 'sim',
  piIp = '10.244.248.208',
}) {

  // Dynamic Obstacle Simulation & Avoidance state
  const [showObstacleControls, setShowObstacleControls] = useState(true);
  const [isDroppingObs, setIsDroppingObs] = useState(false);
  const [obsFeedback, setObsFeedback] = useState(null);

  // Fleet Stations (Place Dispatch) state
  const [namedPlaces, setNamedPlaces] = useState([]);
  const [selectedPlaceId, setSelectedPlaceId] = useState('');
  const [isDispatchingPlace, setIsDispatchingPlace] = useState(false);
  const activePlaceNav = telemetry?.active_place_nav;

  // New Place creation state (provision to add places anytime after mapping)
  const [isAddingPlace, setIsAddingPlace] = useState(false);
  const [newPlaceName, setNewPlaceName] = useState('');
  const [newPlaceNodeId, setNewPlaceNodeId] = useState('');
  const [nodeFilterQuery, setNodeFilterQuery] = useState('');
  const [isSavingNewPlace, setIsSavingNewPlace] = useState(false);
  const [newPlaceFeedback, setNewPlaceFeedback] = useState(null);
  const [graphNodes, setGraphNodes] = useState([]);

  // Load places and graph nodes for the active map
  useEffect(() => {
    let mounted = true;
    bridge.getNamedPlaces(activeMap).then((res) => {
      if (mounted) {
        const places = res?.ok && Array.isArray(res.places) ? res.places : [];
        setNamedPlaces(places);
        setSelectedPlaceId(places.length > 0 ? places[0].id : '');
      }
    });
    bridge.getGraphData(activeMap).then((data) => {
      if (mounted) {
        setGraphNodes(data?.nodes || []);
      }
    });
    setIsAddingPlace(false);
    setNewPlaceName('');
    setNewPlaceNodeId('');
    setNewPlaceFeedback(null);
    return () => { mounted = false; };
  }, [activeMap, mode]);

  // Synchronize places when saved, added, or deleted anywhere
  useEffect(() => {
    const handlePlacesUpdated = (e) => {
      const targetMap = e.detail?.mapName || activeMap;
      if (targetMap === activeMap) {
        if (Array.isArray(e.detail?.places)) {
          setNamedPlaces(e.detail.places);
          if (e.detail.places.length > 0 && (!selectedPlaceId || !e.detail.places.some((p) => p.id === selectedPlaceId))) {
            setSelectedPlaceId(e.detail.places[e.detail.places.length - 1].id);
          }
        } else {
          bridge.getNamedPlaces(activeMap).then((res) => {
            if (res?.ok && Array.isArray(res.places)) {
              setNamedPlaces(res.places);
            }
          });
        }
      }
    };
    window.addEventListener('amr-places-updated', handlePlacesUpdated);
    return () => window.removeEventListener('amr-places-updated', handlePlacesUpdated);
  }, [activeMap, selectedPlaceId]);

  // Listen for clicks on the MapView to auto-populate node when in "New Place" mode
  useEffect(() => {
    if (!isAddingPlace) return;
    const handleNodeClicked = (e) => {
      if (e.detail?.id) {
        setNewPlaceNodeId(e.detail.id);
        setNewPlaceFeedback(`Selected node ${e.detail.id} from map!`);
        setTimeout(() => setNewPlaceFeedback(null), 3000);
      }
    };
    window.addEventListener('amr-node-clicked', handleNodeClicked);
    return () => window.removeEventListener('amr-node-clicked', handleNodeClicked);
  }, [isAddingPlace]);

  // Naturally sorted nodes list for clean picking (N0, N1, N2, ..., N100)
  const sortedNodes = useMemo(() => {
    return [...graphNodes].sort((a, b) => {
      const numA = parseInt(a.id.replace(/\D/g, ''), 10);
      const numB = parseInt(b.id.replace(/\D/g, ''), 10);
      if (!isNaN(numA) && !isNaN(numB)) return numA - numB;
      return a.id.localeCompare(b.id);
    });
  }, [graphNodes]);

  const filteredNodes = useMemo(() => {
    if (!nodeFilterQuery.trim()) return sortedNodes;
    const q = nodeFilterQuery.toLowerCase().trim();
    return sortedNodes.filter((n) => n.id.toLowerCase().includes(q));
  }, [sortedNodes, nodeFilterQuery]);

  const selectedNodeObj = useMemo(() => {
    return graphNodes.find((n) => n.id === newPlaceNodeId) || null;
  }, [graphNodes, newPlaceNodeId]);

  const handleToggleAddPlace = () => {
    const next = !isAddingPlace;
    setIsAddingPlace(next);
    setNewPlaceFeedback(null);
    if (next) {
      window.dispatchEvent(new CustomEvent('amr-show-graph-nodes'));
      if (sortedNodes.length > 0 && !newPlaceNodeId) {
        setNewPlaceNodeId(sortedNodes[0].id);
      }
    }
  };

  const handleSaveNewPlace = async (e) => {
    if (e) e.preventDefault();
    if (!newPlaceName.trim()) {
      setNewPlaceFeedback('⚠️ Please enter a place name');
      return;
    }
    if (!newPlaceNodeId) {
      setNewPlaceFeedback('⚠️ Please select a node on the roadmap');
      return;
    }
    const nodeObj = graphNodes.find((n) => n.id === newPlaceNodeId);
    if (!nodeObj) {
      setNewPlaceFeedback(`⚠️ Node ${newPlaceNodeId} not found in active graph`);
      return;
    }

    setIsSavingNewPlace(true);
    setNewPlaceFeedback('Saving station...');

    const newPlace = {
      id: `place_${Date.now()}`,
      name: newPlaceName.trim(),
      node_id: nodeObj.id,
      type: 'place',
      x: Number(nodeObj.x),
      y: Number(nodeObj.y),
      px: nodeObj.px != null ? Math.round(nodeObj.px) : 0,
      py: nodeObj.py != null ? Math.round(nodeObj.py) : 0,
      theta: 0.0,
      icon: '📍',
      color: '#06b6d4',
      description: '',
    };

    const updatedPlaces = [...namedPlaces, newPlace];

    try {
      const res = await bridge.saveNamedPlaces(updatedPlaces, activeMap);
      if (res?.ok) {
        setNamedPlaces(updatedPlaces);
        setSelectedPlaceId(newPlace.id);
        setNewPlaceName('');
        setIsAddingPlace(false);
        setNewPlaceFeedback(null);
        window.dispatchEvent(new CustomEvent('amr-places-updated', {
          detail: { mapName: activeMap, places: updatedPlaces }
        }));
      } else {
        setNewPlaceFeedback(`❌ Failed: ${res?.error || 'Save failed'}`);
      }
    } catch (err) {
      setNewPlaceFeedback(`❌ Error: ${err.message}`);
    } finally {
      setIsSavingNewPlace(false);
    }
  };

  const handleDeleteSelectedPlace = async () => {
    if (!selectedPlaceId) return;
    const target = namedPlaces.find((p) => p.id === selectedPlaceId);
    if (!target) return;
    if (!window.confirm(`Delete station "${target.name}" (${target.node_id}) from ${activeMap}?`)) return;

    const updatedPlaces = namedPlaces.filter((p) => p.id !== selectedPlaceId);
    try {
      const res = await bridge.saveNamedPlaces(updatedPlaces, activeMap);
      if (res?.ok) {
        setNamedPlaces(updatedPlaces);
        setSelectedPlaceId(updatedPlaces.length > 0 ? updatedPlaces[0].id : '');
        window.dispatchEvent(new CustomEvent('amr-places-updated', {
          detail: { mapName: activeMap, places: updatedPlaces }
        }));
      }
    } catch (err) {
      bridge.emitEvent('critical', `Failed to delete station: ${err.message}`);
    }
  };

  const navState = telemetry?.nav_state;
  const isEstopActive = navState === 'ESTOP' || Boolean(telemetry?.estop_active);

  const handleDispatchSelectedPlace = async () => {
    if (isEstopActive) return;
    const target = namedPlaces.find((p) => p.id === selectedPlaceId);
    if (!target) return;
    setIsDispatchingPlace(true);
    if (setControlMode) setControlMode('auto');
    try {
      await bridge.dispatchPlace(target, activeMap);
    } catch (_) {}
    setIsDispatchingPlace(false);
  };

  const handleAddPlaceToMission = () => {
    const target = namedPlaces.find((p) => p.id === selectedPlaceId);
    if (target && target.node_id && setSelectedNodes) {
      setSelectedNodes((prev) => [...prev, target.node_id]);
    }
  };

  const handleCancelPlaceNav = async () => {
    await bridge.cancelPlaceNavigation();
  };

  useEffect(() => {
    // If robot starts actively navigating or planning, default to auto mode
    if (navState === 'NAVIGATING' || navState === 'PLANNING') {
      if (setControlMode) setControlMode('auto');
    }
  }, [navState, setControlMode]);

  // ── Handlers ──
  const handleRemoveNode = (index) => {
    if (setSelectedNodes) {
      setSelectedNodes(selectedNodes.filter((_, i) => i !== index));
    }
  };

  const handleClearNodes = () => {
    if (setSelectedNodes) setSelectedNodes([]);
  };

  const handleDispatchMission = async () => {
    if (isEstopActive || selectedNodes.length === 0) return;
    if (setControlMode) setControlMode('auto');
    await bridge.sendGoalSequence(selectedNodes);
  };


  // Obstacle drop & remove handlers
  const handleTriggerMapDrop = () => {
    window.dispatchEvent(new CustomEvent('amr-trigger-drop-obstacle'));
    setObsFeedback('Click anywhere on the map to place obstacle');
    setTimeout(() => setObsFeedback(null), 3000);
  };

  const handleDropInFrontOfRobot = async () => {
    const rPose = telemetry?.pose || { x: 0, y: 0, yaw: 0 };
    const targetX = rPose.x + 1.5 * Math.cos(rPose.yaw);
    const targetY = rPose.y + 1.5 * Math.sin(rPose.yaw);
    setIsDroppingObs(true);
    try {
      const res = await bridge.spawnObstacle(targetX, targetY, rPose.yaw);
      if (res?.ok) {
        setObsFeedback(`Dropped 1.5m ahead at (${targetX.toFixed(2)}, ${targetY.toFixed(2)})`);
      } else {
        setObsFeedback(`Failed to spawn: ${res?.error || 'Request rejected'}`);
      }
    } catch (err) {
      setObsFeedback(`Failed to spawn: ${err.message}`);
    }
    setIsDroppingObs(false);
    setTimeout(() => setObsFeedback(null), 3500);
  };

  const handleRemoveObstacle = async () => {
    const res = await bridge.removeObstacle();
    if (res?.ok) {
      setObsFeedback('Obstacle removed from simulation');
    } else {
      setObsFeedback(`Failed to remove: ${res?.error || 'Request rejected'}`);
    }
    setTimeout(() => setObsFeedback(null), 2500);
  };

  // ── Navigation System Stack Lifecycle Controls ──
  const systemStatus = telemetry?.system_status;
  const systemState = systemStatus?.state || 'STOPPED';
  const externalRunning = Boolean(systemStatus?.external_running);
  const isStackOnline = systemState === 'RUNNING' || externalRunning;
  const hwLink = telemetry?.hardware_link;
  const piSsh = hwLink?.pi_ssh;
  const [isSystemLoading, setIsSystemLoading] = useState(false);
  const [isPiLoading, setIsPiLoading] = useState(false);
  const [piLogOpen, setPiLogOpen] = useState(false);
  const [piLogsText, setPiLogsText] = useState('');
  const [autoStartPi, setAutoStartPi] = useState(() => {
    const saved = localStorage.getItem('amr_auto_start_pi');
    return saved === null ? true : saved === 'true';
  });
  const [stopPiOnStop, setStopPiOnStop] = useState(() => {
    return localStorage.getItem('amr_stop_pi_on_stop') === 'true';
  });

  const handleToggleAutoStartPi = (checked) => {
    setAutoStartPi(checked);
    localStorage.setItem('amr_auto_start_pi', String(checked));
  };

  const handleToggleStopPiOnStop = (checked) => {
    setStopPiOnStop(checked);
    localStorage.setItem('amr_stop_pi_on_stop', String(checked));
  };

  const handleStartPiHardware = async () => {
    setIsPiLoading(true);
    try {
      await bridge.startPiHardware();
    } finally {
      setIsPiLoading(false);
    }
  };

  const handleStopPiHardware = async () => {
    setIsPiLoading(true);
    try {
      await bridge.stopPiHardware();
    } finally {
      setIsPiLoading(false);
    }
  };

  const handleTogglePiLogs = async () => {
    const next = !piLogOpen;
    setPiLogOpen(next);
    if (next) {
      const res = await bridge.getPiLogs(60);
      setPiLogsText(res?.ok ? (res.logs || '(empty log)') : `Error: ${res?.error || 'Could not read Pi log'}`);
    }
  };

  const handleStartNavigationSystem = async () => {
    setIsSystemLoading(true);
    try {
      await bridge.startNavigationSystem(activeMap, '', {
        autoStartPi: mode === 'hardware' ? autoStartPi : false,
        stopPiOnStop: mode === 'hardware' ? stopPiOnStop : false,
      });
    } catch (err) {
      bridge.emitEvent('critical', `Failed to start navigation system: ${err.message}`);
    } finally {
      setIsSystemLoading(false);
    }
  };

  const handleStopNavigationSystem = async () => {
    setIsSystemLoading(true);
    try {
      await bridge.stopNavigationSystem({
        stopPi: mode === 'hardware' ? stopPiOnStop : false,
      });
    } catch (err) {
      bridge.emitEvent('critical', `Failed to stop navigation system: ${err.message}`);
    } finally {
      setIsSystemLoading(false);
    }
  };

  return (
    <div className="flex-col" style={{ gap: 12 }}>

      {/* ── 0. NAVIGATION SYSTEM LIFECYCLE CONTROLLER (SIM + HARDWARE) ── */}
      <div
        className={`card lifecycle-card lifecycle-card--${isStackOnline ? 'running' : systemState.toLowerCase()}`}
      >
        <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
          <span style={{ fontWeight: 700, letterSpacing: '0.07em', color: 'var(--text-primary)', textTransform: 'uppercase' }}>
            System Lifecycle ({mode === 'hardware' ? 'HW' : 'SIM'})
          </span>
          <span
            className={`badge ${
              isStackOnline
                ? 'badge--green'
                : systemState === 'STARTING'
                ? 'badge--yellow'
                : 'badge--gray'
            }`}
            style={{ letterSpacing: '0.05em', fontWeight: 700 }}
          >
            {externalRunning ? 'ONLINE (EXT)' : systemState === 'RUNNING' ? 'ONLINE' : systemState === 'STARTING' ? 'INITIALIZING' : 'STANDBY'}
          </span>
        </div>

        <div className="flex-col mt-sm" style={{ gap: 8 }}>
          <div
            style={{
              fontSize: 12,
              background: 'var(--bg-elevated)',
              padding: '8px 10px',
              borderRadius: 6,
              border: '1px solid var(--border)',
            }}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
              <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                Environment:
              </span>
              <strong style={{ color: 'var(--accent-cyan)', fontFamily: 'var(--text-mono)' }}>
                {systemStatus?.active_map || activeMap}
              </strong>
            </div>
            {mode === 'hardware' && (
              <>
                <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
                  <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                    Pi 5 ({piIp}):
                  </span>
                  <span
                    style={{
                      color: hwLink?.sensors_ready
                        ? '#10b981'
                        : piSsh?.state === 'STARTING'
                        ? '#38bdf8'
                        : piSsh?.ssh_ready
                        ? '#f59e0b'
                        : '#f87171',
                      fontSize: 11,
                      fontWeight: 700,
                    }}
                  >
                    {hwLink?.sensors_ready
                      ? 'Sensors Streaming'
                      : piSsh?.state === 'STARTING'
                      ? 'Booting Pi Stack (Hold Still)...'
                      : piSsh?.remote_running
                      ? 'Pi Booting Sensors...'
                      : piSsh?.ssh_ready
                      ? 'SSH Ready (Pi Standby)'
                      : 'Run ./start_hardware.sh on Pi'}
                  </span>
                </div>
                <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
                  <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                    Nav Speeds / BT:
                  </span>
                  <span className="text-mono" style={{ fontSize: 10, color: 'var(--text-secondary)' }}>
                    {telemetry?.nav_limits?.max_linear_speed != null
                      ? `${Number(telemetry.nav_limits.max_linear_speed).toFixed(2)} m/s`
                      : '0.13 m/s'}
                    {' · '}
                    {telemetry?.nav_limits?.use_bt ? 'BT On' : 'BT Off'}
                  </span>
                </div>

                {/* Optional SSH Pi Hardware Controls */}
                <div
                  style={{
                    marginTop: 6,
                    marginBottom: 6,
                    padding: '6px 8px',
                    borderRadius: 5,
                    border: '1px solid var(--border)',
                    background: 'var(--bg-surface)',
                    display: 'flex',
                    flexDirection: 'column',
                    gap: 6,
                  }}
                >
                  <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: 6 }}>
                    <span className="text-dim" style={{ fontSize: 10, fontWeight: 700, letterSpacing: '0.04em', textTransform: 'uppercase' }}>
                      Pi Bringup (SSH)
                    </span>
                    <div style={{ display: 'flex', gap: 4 }}>
                      {hwLink?.sensors_ready || piSsh?.remote_running ? (
                        <button
                          id="btn-stop-pi-ssh"
                          type="button"
                          className="btn btn--outline"
                          disabled={isPiLoading || isSystemLoading}
                          onClick={handleStopPiHardware}
                          style={{ padding: '3px 8px', fontSize: 10, borderColor: '#ef4444', color: '#f87171' }}
                          title="Stop ./start_hardware.sh and Pi hardware nodes cleanly over SSH"
                        >
                          {isPiLoading ? 'Stopping…' : 'Stop Pi'}
                        </button>
                      ) : (
                        <button
                          id="btn-start-pi-ssh"
                          type="button"
                          className="btn btn--outline"
                          disabled={isPiLoading || isSystemLoading || piSsh?.state === 'STARTING' || (piSsh && piSsh.ssh_ready === false)}
                          onClick={handleStartPiHardware}
                          style={{ padding: '3px 8px', fontSize: 10, borderColor: '#10b981', color: '#34d399' }}
                          title="Start ./start_hardware.sh on Pi in background via SSH"
                        >
                          {isPiLoading || piSsh?.state === 'STARTING' ? 'Starting…' : 'Start Pi'}
                        </button>
                      )}
                      <button
                        type="button"
                        className="btn btn--ghost"
                        onClick={handleTogglePiLogs}
                        style={{ padding: '3px 6px', fontSize: 10 }}
                        title="View recent Pi ./start_hardware.sh console log"
                      >
                        {piLogOpen ? 'Hide Log' : 'Pi Log'}
                      </button>
                    </div>
                  </div>

                  <div style={{ display: 'flex', flexWrap: 'wrap', gap: 10, fontSize: 11, color: 'var(--text-secondary)' }}>
                    <label style={{ display: 'flex', alignItems: 'center', gap: 5, cursor: 'pointer' }}>
                      <input
                        id="chk-auto-start-pi"
                        type="checkbox"
                        checked={autoStartPi}
                        onChange={(e) => handleToggleAutoStartPi(e.target.checked)}
                      />
                      <span>Auto-start Pi on Nav Launch</span>
                    </label>
                    <label style={{ display: 'flex', alignItems: 'center', gap: 5, cursor: 'pointer' }}>
                      <input
                        id="chk-stop-pi-on-stop"
                        type="checkbox"
                        checked={stopPiOnStop}
                        onChange={(e) => handleToggleStopPiOnStop(e.target.checked)}
                      />
                      <span>Stop Pi on Stop</span>
                    </label>
                  </div>

                  {piSsh?.last_error && !hwLink?.sensors_ready && (
                    <div style={{ fontSize: 10, color: '#fbbf24' }}>
                      {piSsh.last_error}
                    </div>
                  )}

                  {piLogOpen && (
                    <pre
                      style={{
                        margin: 0,
                        maxHeight: 120,
                        overflowY: 'auto',
                        padding: 6,
                        borderRadius: 4,
                        background: 'rgba(0,0,0,0.35)',
                        fontSize: 10,
                        lineHeight: 1.35,
                        color: 'var(--text-secondary)',
                        whiteSpace: 'pre-wrap',
                        wordBreak: 'break-word',
                      }}
                    >
                      {piLogsText || 'Loading...'}
                    </pre>
                  )}
                </div>
              </>
            )}
            {externalRunning && (
              <div style={{ color: '#10b981', fontSize: 11, fontWeight: 600 }}>
                External navigation session detected (terminal launch active).
              </div>
            )}
            {!externalRunning && systemState === 'RUNNING' && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                  Stack Status:
                </span>
                <span style={{ color: '#10b981', fontSize: 11, fontWeight: 600 }}>
                  {mode === 'hardware'
                    ? 'Active (AMCL, Auto-Loc, MPPI)'
                    : 'Active (Gazebo, Route Runner, BT)'}
                </span>
              </div>
            )}
            {systemState === 'STARTING' && (
              <div style={{ color: '#f59e0b', fontSize: 11, fontWeight: 600 }}>
                {systemStatus?.stage_text || `Stage ${systemStatus?.stage || 1} of 3 in progress...`}
              </div>
            )}
            {systemState === 'ERROR' && systemStatus?.error && (
              <div style={{ color: '#f87171', fontSize: 11, fontWeight: 600 }}>
                {systemStatus.error}
              </div>
            )}
            {(telemetry?.localization_status?.status === 'MAP_MISMATCH' || navState === 'MAP_MISMATCH') && (
              <div
                style={{
                  marginTop: 6,
                  padding: '6px 8px',
                  borderRadius: 5,
                  background: 'rgba(239, 68, 68, 0.14)',
                  border: '1px solid rgba(239, 68, 68, 0.45)',
                  color: '#f87171',
                  fontSize: 11,
                  fontWeight: 600,
                }}
              >
                {telemetry?.localization_status?.message ||
                  'MAP_MISMATCH: Live LiDAR scan does not match this floorplan. Switch to the matching map before navigating.'}
              </div>
            )}
            {navState === 'WHEEL_SLIP' && (
              <div
                style={{
                  marginTop: 6,
                  padding: '6px 8px',
                  borderRadius: 5,
                  background: 'rgba(245, 158, 11, 0.14)',
                  border: '1px solid rgba(245, 158, 11, 0.45)',
                  color: '#fbbf24',
                  fontSize: 11,
                  fontWeight: 600,
                }}
              >
                WHEEL_SLIP HALT: Encoders moved while LiDAR scan stayed stationary. Place wheels firmly on the floor before dispatching.
              </div>
            )}
            {!externalRunning && systemState === 'STOPPED' && (
              <div style={{ color: 'var(--text-secondary)', fontSize: 11 }}>
                {mode === 'hardware'
                  ? autoStartPi
                    ? '1-Click Launch: SSH-starts ./start_hardware.sh on Pi (if needed), then starts AMCL & MPPI.'
                    : 'Launches AMCL, Auto-Localizer & MPPI (assumes ./start_hardware.sh is running on Pi).'
                  : 'Launches Gazebo simulation, Route Runner & BT Manager.'}
              </div>
            )}
          </div>

          {systemState === 'RUNNING' ? (
            <button
              id="btn-stop-system-panel"
              className="btn btn--outline btn--full"
              onClick={handleStopNavigationSystem}
              disabled={isSystemLoading}
              style={{
                borderColor: '#ef4444',
                color: '#f87171',
                padding: '10px',
                fontWeight: 700,
                fontSize: 12,
                letterSpacing: '0.05em',
                textTransform: 'uppercase',
                cursor: isSystemLoading ? 'wait' : 'pointer',
              }}
            >
              {isSystemLoading ? 'Stopping Stack...' : 'STOP SYSTEM'}
            </button>
          ) : (
            <button
              id="btn-start-system-panel"
              className="btn btn--full"
              onClick={handleStartNavigationSystem}
              disabled={isSystemLoading || systemState === 'STARTING' || externalRunning}
              style={{
                background:
                  externalRunning
                    ? '#0284c7'
                    : systemState === 'STARTING'
                    ? '#d97706'
                    : 'linear-gradient(135deg, #059669 0%, #10b981 100%)',
                color: '#ffffff',
                border: 'none',
                padding: '11px',
                fontWeight: 700,
                fontSize: 12,
                letterSpacing: '0.06em',
                textTransform: 'uppercase',
                boxShadow: '0 2px 12px rgba(16, 185, 129, 0.3)',
                cursor: (isSystemLoading || systemState === 'STARTING' || externalRunning) ? 'not-allowed' : 'pointer',
              }}
            >
              {externalRunning
                ? `EXTERNAL NAV ACTIVE • ${activeMap}`
                : systemState === 'STARTING' || isSystemLoading
                ? 'INITIALIZING SYSTEM...'
                : mode === 'hardware'
                ? `START HARDWARE NAV • ${activeMap}`
                : `START SYSTEM • ${activeMap}`}
            </button>
          )}

          {/* Desktop Visualizers */}
          <div style={{ marginTop: 6, paddingTop: 8, borderTop: '1px solid var(--border)' }}>
            <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
              <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                Desktop Visualizers:
              </span>
              <span className="text-mono" style={{ fontSize: 10, color: 'var(--text-secondary)' }}>
                {mode === 'hardware' ? 'Domain 7' : 'WSL2 GUI'}
              </span>
            </div>
            <div className="flex-row" style={{ gap: 6 }}>
              {mode !== 'hardware' && (
                <button
                  type="button"
                  className="btn btn--outline"
                  style={{ flex: 1, fontSize: 11, padding: '6px 8px', borderColor: 'rgba(56, 189, 248, 0.4)', color: '#38bdf8' }}
                  onClick={() => bridge.launchGui('gazebo')}
                  title="Open Gazebo Harmonic 3D window (or run ./view_gazebo.sh in terminal)"
                >
                  🖥️ Gazebo 3D
                </button>
              )}
              <button
                type="button"
                className="btn btn--outline"
                style={{ flex: 1, fontSize: 11, padding: '6px 8px', borderColor: 'rgba(168, 85, 247, 0.4)', color: '#c084fc' }}
                onClick={() => bridge.launchGui('rviz', mode === 'hardware' ? 'hw_nav' : 'nav')}
                title="Open RViz2 Navigation window (or run ./view_rviz.sh in terminal)"
              >
                🗺️ RViz2 {mode === 'hardware' ? '(Hardware)' : 'Nav'}
              </button>
            </div>
          </div>
        </div>
      </div>


      {/* ── 2. FLEET STATIONS & PLACE DISPATCH ── */}
      <div className="card">
        <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
          <span style={{ letterSpacing: '0.06em' }}>FLEET STATIONS ({activeMap})</span>
          <div className="flex-row" style={{ gap: 6, alignItems: 'center' }}>
            <span className="badge badge--cyan" style={{ fontSize: 10 }}>
              {namedPlaces.length} Stations
            </span>
            <button
              className={`btn btn--sm ${isAddingPlace ? 'btn--ghost' : 'btn--secondary'}`}
              onClick={handleToggleAddPlace}
              style={{ fontSize: 11, padding: '3px 8px', fontWeight: 600 }}
              title="Add a new named place to this map anytime"
            >
              {isAddingPlace ? '✕ Close' : '+ New Place'}
            </button>
          </div>
        </div>

        {/* New Place Inline Drawer (Provision to add/name places anytime) */}
        {isAddingPlace && (
          <div
            className="new-place-drawer mt-sm"
            style={{
              padding: '10px 12px',
              backgroundColor: 'rgba(6, 182, 212, 0.07)',
              border: '1px solid rgba(6, 182, 212, 0.3)',
              borderRadius: 6,
            }}
          >
            <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 8 }}>
              <span style={{ fontSize: 12, fontWeight: 700, color: 'var(--accent-cyan)' }}>
                Anchor New Station to {activeMap}
              </span>
              <span className="text-dim text-xs">Node + Name</span>
            </div>

            {/* Step 1: Pick Node */}
            <div style={{ marginBottom: 8 }}>
              <div className="flex-row" style={{ justifyContent: 'space-between', marginBottom: 4 }}>
                <label className="text-xs" style={{ color: 'var(--text-secondary)', fontWeight: 600 }}>
                  1. Pick Roadmap Node:
                </label>
                {selectedNodeObj && (
                  <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)' }}>
                    x: {selectedNodeObj.x.toFixed(2)}m, y: {selectedNodeObj.y.toFixed(2)}m
                  </span>
                )}
              </div>
              <div className="flex-row" style={{ gap: 6 }}>
                <select
                  className="input-field"
                  style={{ flex: 1, padding: '5px 8px', fontSize: 12 }}
                  value={newPlaceNodeId}
                  onChange={(e) => setNewPlaceNodeId(e.target.value)}
                >
                  <option value="">-- Select Node ({sortedNodes.length} available) --</option>
                  {filteredNodes.slice(0, 300).map((n) => (
                    <option key={n.id} value={n.id}>
                      {n.id} (x: {n.x.toFixed(2)}, y: {n.y.toFixed(2)})
                    </option>
                  ))}
                </select>
                {sortedNodes.length > 30 && (
                  <input
                    type="text"
                    placeholder="Search node..."
                    value={nodeFilterQuery}
                    onChange={(e) => setNodeFilterQuery(e.target.value)}
                    className="input-field"
                    style={{ width: 95, padding: '5px 6px', fontSize: 11 }}
                  />
                )}
              </div>
              <div className="text-dim" style={{ fontSize: 10, marginTop: 3 }}>
                Tip: You can also click any node on the map canvas to select it.
              </div>
            </div>

            {/* Step 2: Name the Place */}
            <div style={{ marginBottom: 8 }}>
              <label className="text-xs" style={{ color: 'var(--text-secondary)', fontWeight: 600, display: 'block', marginBottom: 4 }}>
                2. Station / Place Name:
              </label>
              <input
                type="text"
                className="input-field"
                style={{ width: '100%', padding: '6px 8px', fontSize: 12 }}
                placeholder="e.g. Charging Dock, Packing Bay, Gate 2"
                value={newPlaceName}
                onChange={(e) => setNewPlaceName(e.target.value)}
                onKeyDown={(e) => { if (e.key === 'Enter') handleSaveNewPlace(e); }}
              />
              <div className="flex-row mt-xs" style={{ gap: 4, flexWrap: 'wrap' }}>
                {['Charging Dock', 'Station A', 'Station B', 'Docking Bay', 'Storage Bay'].map((sug) => (
                  <button
                    key={sug}
                    type="button"
                    className="btn-preset"
                    style={{ fontSize: 10, padding: '2px 6px' }}
                    onClick={() => setNewPlaceName(sug)}
                  >
                    + {sug}
                  </button>
                ))}
              </div>
            </div>

            {/* Feedback alert */}
            {newPlaceFeedback && (
              <div
                style={{
                  fontSize: 11,
                  padding: '4px 8px',
                  marginBottom: 8,
                  borderRadius: 4,
                  backgroundColor: newPlaceFeedback.startsWith('✓') ? 'rgba(16, 185, 129, 0.15)' : 'rgba(239, 68, 68, 0.15)',
                  color: newPlaceFeedback.startsWith('✓') ? '#34d399' : '#f87171',
                }}
              >
                {newPlaceFeedback}
              </div>
            )}

            {/* Save / Cancel buttons */}
            <div className="flex-row" style={{ gap: 8 }}>
              <button
                className="btn btn--primary btn--full btn--sm"
                onClick={handleSaveNewPlace}
                disabled={isSavingNewPlace || !newPlaceName.trim() || !newPlaceNodeId}
                style={{ fontWeight: 700 }}
              >
                {isSavingNewPlace ? 'Saving...' : 'Save Station'}
              </button>
              <button
                className="btn btn--ghost btn--sm"
                onClick={() => setIsAddingPlace(false)}
                disabled={isSavingNewPlace}
              >
                Cancel
              </button>
            </div>
          </div>
        )}

        {/* Active Navigation HUD if AMR is currently navigating to a place */}
        {activePlaceNav && (activePlaceNav.status === 'NAVIGATING' || navState === 'NAVIGATING') && (
          <div className="active-place-hud-banner mt-sm">
            <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
              <span className="live-pill" style={{ fontSize: 10, padding: '2px 8px' }}>
                <span className="pulse-dot" /> NAVIGATING
              </span>
              <span className="text-mono text-xs" style={{ color: 'var(--accent-cyan)' }}>
                {activePlaceNav.distance_to_target_m != null ? `${activePlaceNav.distance_to_target_m} m away` : ''}
              </span>
            </div>
            <div style={{ marginTop: 4, fontWeight: 700, fontSize: 13, color: 'var(--text-primary)' }}>
              Target: {activePlaceNav.target_place || 'Active Station'} <span className="text-dim text-xs">({activePlaceNav.target_node})</span>
            </div>
            {activePlaceNav.is_sequence && (
              <div className="text-dim text-xs" style={{ marginTop: 2 }}>
                Stop {activePlaceNav.current_stop} of {activePlaceNav.total_stops}
              </div>
            )}
            <button
              className="btn btn--danger btn--full btn--sm mt-xs"
              onClick={handleCancelPlaceNav}
              style={{ padding: '4px 8px', fontSize: 11 }}
            >
              Cancel Navigation
            </button>
          </div>
        )}

        <div className="mt-sm">
          <div className="flex-row" style={{ gap: 8 }}>
            <select
              className="input-field"
              style={{ flex: 1, padding: '6px 8px', fontSize: 12 }}
              value={selectedPlaceId}
              onChange={(e) => setSelectedPlaceId(e.target.value)}
            >
              {namedPlaces.length === 0 ? (
                <option value="">No stations registered on {activeMap}</option>
              ) : (
                namedPlaces.map((p) => (
                  <option key={p.id} value={p.id}>
                    {p.name} ({p.node_id})
                  </option>
                ))
              )}
            </select>
            {selectedPlaceId && (
              <button
                className="btn btn--ghost btn--sm"
                onClick={handleDeleteSelectedPlace}
                title="Delete this station from map"
                style={{ padding: '6px 8px', color: '#f87171', border: '1px solid rgba(239, 68, 68, 0.3)' }}
              >
                Del
              </button>
            )}
          </div>

          {/* Empty state prompt */}
          {namedPlaces.length === 0 && !isAddingPlace && (
            <div
              style={{
                marginTop: 8,
                padding: '8px 10px',
                textAlign: 'center',
                backgroundColor: 'var(--bg-elevated)',
                borderRadius: 6,
                border: '1px dashed var(--border)',
              }}
            >
              <div className="text-dim text-xs" style={{ marginBottom: 6 }}>
                No places marked for <b>{activeMap}</b> yet.
              </div>
              <button
                className="btn btn--secondary btn--sm"
                onClick={() => {
                  setIsAddingPlace(true);
                  window.dispatchEvent(new CustomEvent('amr-show-graph-nodes'));
                  if (sortedNodes.length > 0 && !newPlaceNodeId) {
                    setNewPlaceNodeId(sortedNodes[0].id);
                  }
                }}
                style={{ fontSize: 11, padding: '4px 10px' }}
              >
                + Add First Station
              </button>
            </div>
          )}

          {/* Quick station chips */}
          {namedPlaces.length > 0 && (
            <div className="preset-row mt-sm" style={{ flexWrap: 'wrap', gap: 6 }}>
              {namedPlaces.slice(0, 4).map((p) => (
                <button
                  key={p.id}
                  className={`btn-preset ${selectedPlaceId === p.id ? 'btn-preset--active' : ''}`}
                  onClick={() => setSelectedPlaceId(p.id)}
                  style={{ fontSize: 11, padding: '3px 8px' }}
                >
                  {p.name}
                </button>
              ))}
            </div>
          )}

          <div className="flex-row mt-sm" style={{ gap: 8 }}>
            <button
              className="btn btn--primary btn--full btn--sm"
              disabled={!selectedPlaceId || isDispatchingPlace || isEstopActive}
              onClick={handleDispatchSelectedPlace}
              title={isEstopActive ? 'E-STOP active — clear E-STOP before dispatching' : undefined}
              style={{ fontWeight: 700, padding: '7px 12px' }}
            >
              {isEstopActive ? 'E-STOP LOCKED' : isDispatchingPlace ? 'Dispatching...' : 'Dispatch to Station'}
            </button>
            <button
              className="btn btn--ghost btn--sm"
              disabled={!selectedPlaceId}
              onClick={handleAddPlaceToMission}
              title="Add this station's node to mission waypoints queue"
              style={{ fontSize: 11, whiteSpace: 'nowrap', padding: '7px 10px' }}
            >
              + Queue
            </button>
          </div>
        </div>
      </div>

      {/* ── 3. TOPOLOGICAL MISSION BUILDER ── */}
      <div className="card">
        <div className="card__title flex-row" style={{ justifyContent: 'space-between' }}>
          <span style={{ letterSpacing: '0.06em' }}>MISSION WAYPOINTS</span>
          {selectedNodes.length > 0 && (
            <button className="btn-link text-xs" onClick={handleClearNodes}>
              Clear
            </button>
          )}
        </div>

        <div className="waypoint-queue-container mt-sm">
          {selectedNodes.length === 0 ? (
            <div className="text-dim text-xs" style={{ textAlign: 'center', padding: '12px 6px' }}>
              No waypoints selected. Turn on Nodes on the map canvas to select.
            </div>
          ) : (
            <div className="waypoint-chip-list">
              {selectedNodes.map((nodeId, idx) => (
                <span key={`${nodeId}-${idx}`} className="waypoint-chip">
                  <span className="chip-idx">{idx + 1}</span>
                  <span className="chip-name">{nodeId}</span>
                  <button
                    className="chip-remove"
                    onClick={() => handleRemoveNode(idx)}
                    title="Remove"
                  >
                    ✕
                  </button>
                </span>
              ))}
            </div>
          )}
        </div>

        <button
          className="btn btn--success btn--full mt-sm"
          disabled={selectedNodes.length === 0 || isEstopActive}
          onClick={handleDispatchMission}
          title={isEstopActive ? 'E-STOP active — clear E-STOP before dispatching' : undefined}
          style={{ letterSpacing: '0.04em', fontWeight: 700 }}
        >
          {isEstopActive ? 'E-STOP LOCKED' : `Dispatch Mission Sequence (${selectedNodes.length})`}
        </button>
      </div>

      {/* ── 4. DYNAMIC OBSTACLE AVOIDANCE (MODE-AWARE) ── */}
      {mode === 'hardware' ? (
        <div className="card">
          <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
            <span style={{ letterSpacing: '0.06em', display: 'flex', alignItems: 'center', gap: 6 }}>
              <span>🛡️</span>
              <span>LIDAR OBSTACLE GUARD</span>
            </span>
            <span
              className="badge"
              style={{
                fontSize: 9,
                fontWeight: 700,
                background: telemetry?.nav_state === 'YIELDING' ? 'rgba(249, 115, 22, 0.2)' : 'rgba(16, 185, 129, 0.15)',
                color: telemetry?.nav_state === 'YIELDING' ? '#fb923c' : '#34d399',
              }}
            >
              {telemetry?.nav_state === 'YIELDING' ? 'YIELDING' : 'ACTIVE'}
            </span>
          </div>
          <div className="mt-sm" style={{ fontSize: 12, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <span className="text-dim">Nearest LiDAR Return:</span>
            <strong className="text-mono" style={{ color: (telemetry?.min_obstacle_dist ?? 9) < 0.4 ? '#f87171' : '#34d399' }}>
              {telemetry?.min_obstacle_dist != null ? `${telemetry.min_obstacle_dist.toFixed(2)} m` : '—'}
            </strong>
          </div>
          <div className="text-dim text-xs mt-xs">
            Real-time obstacle detection &amp; MPPI dynamic evasion run directly on physical YDLIDAR X2 scans.
          </div>
        </div>
      ) : (() => {
        const dynObs = telemetry?.dynamic_obstacle;
        const obsDist = dynObs?.dist_to_robot;
        const isEvading = dynObs?.avoidance_status === 'TACTICAL_EVASION' || (obsDist != null && obsDist < 0.85);
        const isProximity = dynObs?.avoidance_status === 'PROXIMITY_SLOWDOWN' || (obsDist != null && obsDist < 1.5 && !isEvading);

        return (
          <div
            className="card"
            style={{
              border: dynObs?.active ? (isEvading ? '1px solid #ef4444' : isProximity ? '1px solid #ea580c' : '1px solid rgba(234, 88, 12, 0.4)') : undefined,
            }}
          >
            <div
              className="card__title flex-row cursor-pointer"
              style={{ justifyContent: 'space-between', alignItems: 'center' }}
              onClick={() => setShowObstacleControls(!showObstacleControls)}
            >
              <span style={{ letterSpacing: '0.06em', display: 'flex', alignItems: 'center', gap: 6 }}>
                <span>🚧</span>
                <span>DYNAMIC OBSTACLE AVOIDANCE</span>
              </span>
              <div className="flex-row" style={{ gap: 6, alignItems: 'center' }}>
                <span
                  className="badge"
                  style={{
                    fontSize: 9,
                    fontWeight: 700,
                    background: isEvading ? 'rgba(239, 68, 68, 0.2)' : isProximity ? 'rgba(249, 115, 22, 0.2)' : dynObs?.active ? 'rgba(16, 185, 129, 0.15)' : 'rgba(255,255,255,0.06)',
                    color: isEvading ? '#f87171' : isProximity ? '#fb923c' : dynObs?.active ? '#34d399' : 'var(--text-dim)',
                    border: `1px solid ${isEvading ? '#ef4444' : isProximity ? '#ea580c' : dynObs?.active ? '#059669' : 'transparent'}`,
                  }}
                >
                  {isEvading ? 'EVADING' : isProximity ? 'PROXIMITY' : dynObs?.active ? 'TRACKED' : 'READY'}
                </span>
                <span className="text-xs text-dim">{showObstacleControls ? '▲' : '▼'}</span>
              </div>
            </div>

            {showObstacleControls && (
              <div className="obstacle-tester-body mt-sm" style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
                {/* ── Real-time Metrics: Both Values Prominently Displayed ── */}
                <div
                  style={{
                    display: 'grid',
                    gridTemplateColumns: '1fr 1fr',
                    gap: 8,
                  }}
                >
                  {/* Value 1: Avoidance Status & Obstacle Speed */}
                  <div
                    style={{
                      background: isEvading
                        ? 'rgba(239, 68, 68, 0.15)'
                        : isProximity
                          ? 'rgba(249, 115, 22, 0.15)'
                          : 'var(--bg-elevated)',
                      border: isEvading
                        ? '1px solid #ef4444'
                        : isProximity
                          ? '1px solid #ea580c'
                          : '1px solid var(--border)',
                      borderRadius: 8,
                      padding: '8px 10px',
                      display: 'flex',
                      flexDirection: 'column',
                      gap: 4,
                      boxShadow: isEvading ? '0 0 14px rgba(239, 68, 68, 0.25)' : undefined,
                    }}
                  >
                    <span className="text-dim text-xs" style={{ textTransform: 'uppercase', letterSpacing: '0.04em', fontWeight: 600 }}>
                      Avoidance Status
                    </span>
                    <div style={{ display: 'flex', alignItems: 'center', gap: 5, marginTop: 1 }}>
                      <span style={{ fontSize: 13 }}>
                        {isEvading ? '⚠️' : isProximity ? '⚡' : dynObs?.active ? '🛡️' : '⚪'}
                      </span>
                      <span
                        style={{
                          fontSize: 12,
                          fontWeight: 800,
                          color: isEvading ? '#f87171' : isProximity ? '#fb923c' : dynObs?.active ? '#34d399' : 'var(--text-dim)',
                          letterSpacing: '0.03em',
                        }}
                      >
                        {isEvading ? 'EVADING' : isProximity ? 'PROXIMITY' : dynObs?.active ? 'NOMINAL' : 'IDLE'}
                      </span>
                    </div>
                    <div className="text-mono" style={{ fontSize: 11, color: 'var(--text-secondary)', marginTop: 2 }}>
                      Speed: <strong style={{ color: 'var(--text-primary)' }}>{(dynObs?.speed || 0).toFixed(1)} m/s</strong>
                    </div>
                  </div>

                  {/* Value 2: Real-time Clearance Distance */}
                  <div
                    style={{
                      background: isEvading
                        ? 'rgba(239, 68, 68, 0.15)'
                        : isProximity
                          ? 'rgba(249, 115, 22, 0.15)'
                          : 'var(--bg-elevated)',
                      border: isEvading
                        ? '1px solid #ef4444'
                        : isProximity
                          ? '1px solid #ea580c'
                          : '1px solid var(--border)',
                      borderRadius: 8,
                      padding: '8px 10px',
                      display: 'flex',
                      flexDirection: 'column',
                      gap: 4,
                      boxShadow: isEvading ? '0 0 14px rgba(239, 68, 68, 0.25)' : undefined,
                    }}
                  >
                    <span className="text-dim text-xs" style={{ textTransform: 'uppercase', letterSpacing: '0.04em', fontWeight: 600 }}>
                      AMR Clearance
                    </span>
                    <div
                      className="text-mono font-bold"
                      style={{
                        fontSize: 17,
                        color: isEvading ? '#ef4444' : isProximity ? '#ea580c' : dynObs?.active ? '#10b981' : 'var(--text-dim)',
                        lineHeight: 1.2,
                        marginTop: 1,
                      }}
                    >
                      {obsDist != null ? `${obsDist.toFixed(2)} m` : '—'}
                    </div>
                    <span className="text-dim text-xs">
                      {dynObs?.x != null ? `Pos: (${dynObs.x.toFixed(2)}, ${dynObs.y.toFixed(2)})` : 'Not deployed'}
                    </span>
                  </div>
                </div>

                {/* Real-time Clearance Proximity Bar Track */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
                  <div className="flex-row" style={{ justifyContent: 'space-between', fontSize: 10, color: 'var(--text-dim)' }}>
                    <span>0m</span>
                    <span style={{ color: isEvading ? '#f87171' : isProximity ? '#fb923c' : '#34d399', fontWeight: 700 }}>
                      {isEvading ? '⚠️ Evasion Active (<0.85m)' : isProximity ? '⚡ Slowdown Active (<1.5m)' : 'Clearance Safe'}
                    </span>
                    <span>3m+</span>
                  </div>
                  <div style={{ height: 6, background: 'var(--border)', borderRadius: 3, overflow: 'hidden' }}>
                    <div
                      style={{
                        height: '100%',
                        width: obsDist != null ? `${Math.min(100, Math.max(8, (obsDist / 3.0) * 100))}%` : '0%',
                        background: isEvading
                          ? '#ef4444'
                          : isProximity
                            ? '#ea580c'
                            : '#10b981',
                        transition: 'width 0.2s ease, background 0.2s ease',
                      }}
                    />
                  </div>
                </div>

                {obsFeedback && (
                  <div
                    style={{
                      background: 'rgba(234, 88, 12, 0.15)',
                      color: '#fb923c',
                      border: '1px solid rgba(234, 88, 12, 0.4)',
                      padding: '5px 8px',
                      borderRadius: 4,
                      fontSize: 11,
                      textAlign: 'center',
                    }}
                  >
                    {obsFeedback}
                  </div>
                )}

                {/* ── 1. DROP OBSTACLE CONTROLS ── */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: 5 }}>
                  <span className="text-dim text-xs" style={{ letterSpacing: '0.04em', textTransform: 'uppercase' }}>
                    Deploy / Drop Position
                  </span>
                  <button
                    className="btn btn--warning btn--full"
                    onClick={handleTriggerMapDrop}
                    style={{
                      fontWeight: 700,
                      fontSize: 11,
                      letterSpacing: '0.04em',
                      padding: '8px',
                      background: 'linear-gradient(135deg, #ea580c 0%, #c2410c 100%)',
                      border: '1px solid #f97316',
                      color: '#ffffff',
                    }}
                    title="Click here, then click anywhere on the live map canvas"
                  >
                    🎯 CLICK TO DROP ON MAP CANVAS
                  </button>

                  <button
                    className="btn btn--ghost btn--sm"
                    onClick={handleDropInFrontOfRobot}
                    disabled={isDroppingObs}
                    style={{ fontSize: 11, padding: '5px' }}
                    title="Calculate pose 1.5m ahead of current robot heading and drop"
                  >
                    📍 Drop 1.5m In Front of Robot
                  </button>

                  {/* Remove Obstacle Button */}
                  {dynObs?.active && (
                    <button
                      className="btn btn--ghost btn--sm mt-xs"
                      onClick={handleRemoveObstacle}
                      style={{ color: '#f87171', borderColor: 'rgba(239, 68, 68, 0.3)', fontSize: 11, padding: '6px' }}
                    >
                      ✕ Remove Obstacle from Simulation
                    </button>
                  )}
                </div>
              </div>
            )}
          </div>
        );
      })()}
    </div>
  );
}
