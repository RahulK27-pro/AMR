import { useEffect, useMemo, useState } from 'react';
import bridge from '../services/amrBridge';

export default function ControlPanel({
  telemetry,
  selectedNodes = [],
  setSelectedNodes,
  onOpenMapping,
  activeMap = 'warehouse_01',
  onMapChange,
  controlMode = 'auto',
  setControlMode,
}) {

  // Dynamic Obstacle Simulation Panel state
  const [showObstacleControls, setShowObstacleControls] = useState(false);
  const [obsSpeed, setObsSpeed] = useState(0.45);

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
  }, [activeMap]);

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

  const handleDispatchSelectedPlace = async () => {
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

  const navState = telemetry?.nav_state;

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
    if (selectedNodes.length === 0) return;
    if (setControlMode) setControlMode('auto');
    await bridge.sendGoalSequence(selectedNodes);
  };


  // Obstacle teleop
  const sendObstacle = (linear, angular) => {
    bridge.sendObstacleCmdVel(linear, angular);
  };

  // ── Navigation System Stack Lifecycle Controls ──
  const systemStatus = telemetry?.system_status;
  const systemState = systemStatus?.state || 'STOPPED';
  const [isSystemLoading, setIsSystemLoading] = useState(false);

  const handleStartNavigationSystem = async () => {
    setIsSystemLoading(true);
    try {
      await bridge.startNavigationSystem(activeMap);
    } catch (err) {
      bridge.emitEvent('critical', `Failed to start navigation system: ${err.message}`);
    } finally {
      setIsSystemLoading(false);
    }
  };

  const handleStopNavigationSystem = async () => {
    setIsSystemLoading(true);
    try {
      await bridge.stopNavigationSystem();
    } catch (err) {
      bridge.emitEvent('critical', `Failed to stop navigation system: ${err.message}`);
    } finally {
      setIsSystemLoading(false);
    }
  };

  return (
    <div className="flex-col" style={{ gap: 12 }}>

      {/* ── 0. NAVIGATION SYSTEM LIFECYCLE CONTROLLER (PROFESSIONAL / NO EMOJIS) ── */}
      <div
        className="card"
        style={{
          background:
            systemState === 'RUNNING'
              ? 'linear-gradient(180deg, rgba(6, 78, 59, 0.28) 0%, rgba(15, 23, 42, 0.85) 100%)'
              : systemState === 'STARTING'
              ? 'linear-gradient(180deg, rgba(120, 53, 15, 0.28) 0%, rgba(15, 23, 42, 0.85) 100%)'
              : 'rgba(15, 23, 42, 0.65)',
          border:
            systemState === 'RUNNING'
              ? '1px solid #059669'
              : systemState === 'STARTING'
              ? '1px solid #d97706'
              : '1px solid #334155',
          boxShadow:
            systemState === 'RUNNING'
              ? '0 0 16px rgba(16, 185, 129, 0.12)'
              : 'none',
        }}
      >
        <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
          <span style={{ fontWeight: 700, letterSpacing: '0.07em', color: '#e2e8f0', textTransform: 'uppercase' }}>
            System Lifecycle
          </span>
          <span
            className={`badge ${
              systemState === 'RUNNING'
                ? 'badge--green'
                : systemState === 'STARTING'
                ? 'badge--yellow'
                : 'badge--gray'
            }`}
            style={{ letterSpacing: '0.05em', fontWeight: 700 }}
          >
            {systemState === 'RUNNING' ? 'ONLINE' : systemState === 'STARTING' ? 'INITIALIZING' : 'STANDBY'}
          </span>
        </div>

        <div className="flex-col mt-sm" style={{ gap: 8 }}>
          <div
            style={{
              fontSize: 12,
              background: 'rgba(0,0,0,0.3)',
              padding: '8px 10px',
              borderRadius: 6,
              border: '1px solid rgba(255,255,255,0.06)',
            }}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
              <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                Environment:
              </span>
              <strong style={{ color: '#38bdf8', fontFamily: 'var(--text-mono)' }}>
                {systemStatus?.active_map || activeMap}
              </strong>
            </div>
            {systemState === 'RUNNING' && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span className="text-dim" style={{ textTransform: 'uppercase', fontSize: 10, letterSpacing: '0.04em' }}>
                  Stack Status:
                </span>
                <span style={{ color: '#10b981', fontSize: 11, fontWeight: 600 }}>
                  Active (Gazebo, Route Runner, BT)
                </span>
              </div>
            )}
            {systemState === 'STARTING' && (
              <div style={{ color: '#f59e0b', fontSize: 11, fontWeight: 600 }}>
                {systemStatus?.message || `Stage ${systemStatus?.stage || 1} of 3 in progress...`}
              </div>
            )}
            {systemState === 'STOPPED' && (
              <div style={{ color: '#94a3b8', fontSize: 11 }}>
                Launches Gazebo simulation, Route Runner &amp; BT Manager.
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
              disabled={isSystemLoading || systemState === 'STARTING'}
              style={{
                background:
                  systemState === 'STARTING'
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
                cursor: isSystemLoading || systemState === 'STARTING' ? 'wait' : 'pointer',
              }}
            >
              {systemState === 'STARTING' || isSystemLoading
                ? 'INITIALIZING SYSTEM...'
                : `START SYSTEM • ${activeMap}`}
            </button>
          )}
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
                <label className="text-xs" style={{ color: '#cbd5e1', fontWeight: 600 }}>
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
              <label className="text-xs" style={{ color: '#cbd5e1', fontWeight: 600, display: 'block', marginBottom: 4 }}>
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
            <div style={{ marginTop: 4, fontWeight: 700, fontSize: 13, color: '#f8fafc' }}>
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
                backgroundColor: 'rgba(255, 255, 255, 0.02)',
                borderRadius: 6,
                border: '1px dashed rgba(255, 255, 255, 0.1)',
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
              disabled={!selectedPlaceId || isDispatchingPlace}
              onClick={handleDispatchSelectedPlace}
              style={{ fontWeight: 700, padding: '7px 12px' }}
            >
              {isDispatchingPlace ? 'Dispatching...' : 'Dispatch to Station'}
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
          disabled={selectedNodes.length === 0}
          onClick={handleDispatchMission}
          style={{ letterSpacing: '0.04em', fontWeight: 700 }}
        >
          Dispatch Mission Sequence ({selectedNodes.length})
        </button>
      </div>

      {/* ── 4. DYNAMIC OBSTACLE SIMULATOR TESTER ── */}
      <div className="card">
        <div
          className="card__title flex-row cursor-pointer"
          style={{ justifyContent: 'space-between' }}
          onClick={() => setShowObstacleControls(!showObstacleControls)}
        >
          <span style={{ letterSpacing: '0.06em' }}>DYNAMIC OBSTACLE SIMULATOR</span>
          <span className="text-xs text-dim">{showObstacleControls ? '▲ Hide' : '▼ Test'}</span>
        </div>

        {showObstacleControls && (
          <div className="obstacle-tester-body mt-sm">
            <div className="text-dim text-xs" style={{ lineHeight: 1.4 }}>
              Simulate human/cart traffic to trigger AMR <b>MPPI Evasion (&lt;0.85m)</b>, <b>Corridor Yielding</b>, and <b>Dijkstra Re-routing (&gt;4.5s)</b>.
            </div>

            <div className="obstacle-pad-grid mt-sm">
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(obsSpeed, 0)}>↑ Fwd</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(-obsSpeed, 0)}>↓ Back</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(0, 0.8)}>↺ Turn L</button>
              <button className="btn btn--ghost btn--sm" onClick={() => sendObstacle(0, -0.8)}>↻ Turn R</button>
              <button
                className="btn btn--ghost btn--sm"
                style={{ gridColumn: 'span 4', color: '#ff3355' }}
                onClick={() => sendObstacle(0, 0)}
              >
                Stop Obstacle
              </button>
            </div>
          </div>
        )}
      </div>

    </div>
  );
}
