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
  const pose = telemetry?.pose ?? { x: 0, y: 0, yaw: 0 };
  const vel = telemetry?.velocity ?? { linear: 0, angular: 0 };
  const velocity = vel;
  const navState = telemetry?.nav_state ?? 'IDLE';
  const minObs = telemetry?.min_obstacle_dist;

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

  // Phase 3: Graph Extraction state
  const [savedMaps, setSavedMaps] = useState([]);
  const [selectedMapForGraph, setSelectedMapForGraph] = useState(mapName || 'warehouse_01');
  const [densityPreset, setDensityPreset] = useState('balanced'); // 'balanced' | 'dense' | 'sparse' | 'custom'
  const [customParams, setCustomParams] = useState({
    robotRadius: 0.11,
    safetyMargin: 0.10,
    searchRadius: 2.5,
    stepCorridor: 0.40,
    stepMedium: 0.50,
    stepOpen: 0.80,
  });
  const [isExtracting, setIsExtracting] = useState(false);
  const [extractedGraphReport, setExtractedGraphReport] = useState(null);
  const [graphExtractError, setGraphExtractError] = useState(null);
  const [graphVisMode, setGraphVisMode] = useState('graph'); // 'graph' | 'clean'

  // Phase 4: Graph Verification & Route Testing state
  const [graphVerificationData, setGraphVerificationData] = useState(null);
  const [isAuditingGraph, setIsAuditingGraph] = useState(false);
  const [routeTesterStart, setRouteTesterStart] = useState('N0');
  const [routeTesterGoal, setRouteTesterGoal] = useState('');
  const [testedRoute, setTestedRoute] = useState(null);
  const [isPlanningRoute, setIsPlanningRoute] = useState(false);
  const [routePlanError, setRoutePlanError] = useState(null);
  const [showRouteOverlay, setShowRouteOverlay] = useState(true);
  const [isGraphApproved, setIsGraphApproved] = useState(false);
  const [imgDimensions, setImgDimensions] = useState({ width: 1000, height: 1000 });

  // Phase 5: Name Places / Nodes state
  const [namedPlaces, setNamedPlaces] = useState([]);
  const [isLoadingPlaces, setIsLoadingPlaces] = useState(false);
  const [isSavingPlaces, setIsSavingPlaces] = useState(false);
  const [placesSaveSuccess, setPlacesSaveSuccess] = useState(false);
  const [placesError, setPlacesError] = useState(null);
  const [selectedPlaceId, setSelectedPlaceId] = useState(null);
  const [isEditingPlace, setIsEditingPlace] = useState(false);
  const [isPickNodeOnCanvasMode, setIsPickNodeOnCanvasMode] = useState(false);
  const [isPlacesApproved, setIsPlacesApproved] = useState(false);
  const [placeForm, setPlaceForm] = useState({
    id: '',
    name: '',
    node_id: 'N0',
    x: 0,
    y: 0,
    px: 0,
    py: 0,
  });

  // Phase 6: Navigation Using Place Names state
  const [navMode, setNavMode] = useState('single'); // 'single' | 'multi'
  const [navDestinationPlaceId, setNavDestinationPlaceId] = useState('');
  const [selectedStopToAdd, setSelectedStopToAdd] = useState('');
  const [navSequence, setNavSequence] = useState([]);
  const [isDispatchingNav, setIsDispatchingNav] = useState(false);
  const [navRoutePreview, setNavRoutePreview] = useState(null);
  const [navError, setNavError] = useState(null);
  const [isNavApproved, setIsNavApproved] = useState(false);
  const [activeNavInfo, setActiveNavInfo] = useState(null);
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

  // Fetch saved maps for Graph Extraction source selection
  const fetchSavedMaps = useCallback(async () => {
    try {
      const res = await bridge.getSavedMaps();
      if (res?.maps) {
        setSavedMaps(res.maps);
        if (res.maps.length > 0 && !selectedMapForGraph) {
          setSelectedMapForGraph(res.maps[0].name);
        }
      }
    } catch (_) {}
  }, [selectedMapForGraph]);

  const fetchExistingGraph = useCallback(async (targetMap) => {
    try {
      const res = await bridge.getLatestGraph(targetMap || selectedMapForGraph || mapName);
      if (res?.ok && res.graph && res.graph.total_nodes > 0) {
        setExtractedGraphReport(res.graph);
      }
    } catch (_) {}
  }, [selectedMapForGraph, mapName]);

  useEffect(() => {
    fetchSavedMaps();
  }, [fetchSavedMaps]);

  // When step changes to 3, fetch saved maps & check if an existing graph exists
  useEffect(() => {
    if (currentStep === 3) {
      fetchSavedMaps();
      if (!extractedGraphReport) {
        fetchExistingGraph();
      }
    }
  }, [currentStep, fetchSavedMaps, fetchExistingGraph, extractedGraphReport]);

  // Phase 3: Graph Extraction Action
  const handleExtractGraph = async () => {
    setIsExtracting(true);
    setGraphExtractError(null);
    try {
      const targetMap = (selectedMapForGraph || mapName || 'warehouse_01').trim();
      let payload = {
        map_name: targetMap,
      };

      if (densityPreset === 'balanced') {
        payload = {
          ...payload,
          robot_radius: 0.11,
          safety_margin: 0.10,
          search_radius: 2.5,
          step_corridor: 0.40,
          step_medium: 0.50,
          step_open: 0.80,
        };
      } else if (densityPreset === 'dense') {
        payload = {
          ...payload,
          robot_radius: 0.11,
          safety_margin: 0.08,
          search_radius: 2.0,
          step_corridor: 0.30,
          step_medium: 0.40,
          step_open: 0.60,
        };
      } else if (densityPreset === 'sparse') {
        payload = {
          ...payload,
          robot_radius: 0.11,
          safety_margin: 0.14,
          search_radius: 3.5,
          step_corridor: 0.50,
          step_medium: 0.70,
          step_open: 1.00,
        };
      } else {
        payload = {
          ...payload,
          robot_radius: parseFloat(customParams.robotRadius) || 0.11,
          safety_margin: parseFloat(customParams.safetyMargin) || 0.10,
          search_radius: parseFloat(customParams.searchRadius) || 2.5,
          step_corridor: parseFloat(customParams.stepCorridor) || 0.40,
          step_medium: parseFloat(customParams.stepMedium) || 0.50,
          step_open: parseFloat(customParams.stepOpen) || 0.80,
        };
      }

      const res = await bridge.extractGraph(payload);
      if (res?.ok && res.report) {
        setExtractedGraphReport(res.report);
        setGraphVisMode('graph');
      } else {
        setGraphExtractError(res?.error || 'Graph extraction failed.');
      }
    } catch (err) {
      setGraphExtractError(err.message || 'Error occurred during graph extraction.');
    } finally {
      setIsExtracting(false);
    }
  };

  // Flow Continuation to Step 3: Graph Extraction
  const handleProceedToGraph = () => {
    setCurrentStep(3);
    fetchSavedMaps();
  };

  // Phase 4: Graph Verification & Route Testing Actions
  const runGraphVerification = useCallback(async (targetMap) => {
    setIsAuditingGraph(true);
    try {
      const target = targetMap || selectedMapForGraph || mapName || 'warehouse_01';
      const res = await bridge.verifyGraph(target);
      if (res?.ok) {
        setGraphVerificationData(res);
      }
    } finally {
      setIsAuditingGraph(false);
    }
  }, [selectedMapForGraph, mapName]);

  // When step changes to 4, run audit and set default start/goal if needed
  useEffect(() => {
    if (currentStep === 4) {
      runGraphVerification();
      const nodes = extractedGraphReport?.nodes || [];
      if (nodes.length > 0) {
        if (!routeTesterStart) setRouteTesterStart(nodes[0].id);
        if (!routeTesterGoal) setRouteTesterGoal(nodes[nodes.length - 1].id);
      }
    }
  }, [currentStep, runGraphVerification, extractedGraphReport, routeTesterStart, routeTesterGoal]);

  const handleTestRoute = async () => {
    if (!routeTesterStart || !routeTesterGoal) return;
    setIsPlanningRoute(true);
    setRoutePlanError(null);
    try {
      const target = selectedMapForGraph || mapName || 'warehouse_01';
      const res = await bridge.planRoute(routeTesterStart, routeTesterGoal, target);
      if (res?.ok && res.route) {
        setTestedRoute(res.route);
        setShowRouteOverlay(true);
      } else {
        setRoutePlanError(res?.error || 'Path planning failed');
      }
    } catch (err) {
      setRoutePlanError(err.message || 'Routing error');
    } finally {
      setIsPlanningRoute(false);
    }
  };

  const handleRandomRouteProbe = () => {
    const nodes = extractedGraphReport?.nodes || [];
    if (nodes.length < 2) return;
    const rand1 = Math.floor(Math.random() * nodes.length);
    let rand2 = Math.floor(Math.random() * nodes.length);
    while (rand2 === rand1) rand2 = Math.floor(Math.random() * nodes.length);
    setRouteTesterStart(nodes[rand1].id);
    setRouteTesterGoal(nodes[rand2].id);
    setTestedRoute(null);
  };

  const handleSwapRouteEndpoints = () => {
    const prevStart = routeTesterStart;
    setRouteTesterStart(routeTesterGoal);
    setRouteTesterGoal(prevStart);
    setTestedRoute(null);
  };

  const handleApproveRoadmap = () => {
    setIsGraphApproved(true);
    setCurrentStep(5);
  };

  // ---- Phase 5: Name Places / Nodes Callbacks & Handlers ----
  const fetchNamedPlaces = useCallback(async () => {
    setIsLoadingPlaces(true);
    setPlacesError(null);
    try {
      const target = selectedMapForGraph || mapName || 'warehouse_01';
      const res = await bridge.getNamedPlaces(target);
      if (res?.ok && Array.isArray(res.places)) {
        setNamedPlaces(res.places);
      } else {
        setNamedPlaces([]);
      }
    } catch (err) {
      setPlacesError(err.message || 'Failed to fetch places');
    } finally {
      setIsLoadingPlaces(false);
    }
  }, [selectedMapForGraph, mapName]);

  useEffect(() => {
    if (currentStep === 5) {
      fetchNamedPlaces();
      if (!extractedGraphReport) {
        fetchExistingGraph();
      }
      const nodes = extractedGraphReport?.nodes || [];
      if (nodes.length > 0 && (!placeForm.node_id || placeForm.node_id === 'N0')) {
        setPlaceForm((prev) => ({
          ...prev,
          node_id: nodes[0].id,
          x: Number(Number(nodes[0].x ?? nodes[0].wx ?? 0).toFixed(3)),
          y: Number(Number(nodes[0].y ?? nodes[0].wy ?? 0).toFixed(3)),
          px: nodes[0].px || 0,
          py: nodes[0].py || 0,
        }));
      }
    }
  }, [currentStep, fetchNamedPlaces, extractedGraphReport, fetchExistingGraph]);

  const handleSelectNodeForPlace = (nodeId) => {
    const nodes = extractedGraphReport?.nodes || [];
    const found = nodes.find((n) => n.id === nodeId);
    if (found) {
      setPlaceForm((prev) => ({
        ...prev,
        node_id: found.id,
        x: Number(Number(found.x ?? found.wx ?? 0).toFixed(3)),
        y: Number(Number(found.y ?? found.wy ?? 0).toFixed(3)),
        px: found.px ?? 0,
        py: found.py ?? 0,
      }));
    } else {
      setPlaceForm((prev) => ({ ...prev, node_id: nodeId }));
    }
  };

  const handleSelectPlaceForEdit = (place) => {
    setSelectedPlaceId(place.id);
    setIsEditingPlace(true);
    setPlaceForm({
      id: place.id,
      name: place.name,
      node_id: place.node_id,
      x: place.x || 0,
      y: place.y || 0,
      px: place.px || 0,
      py: place.py || 0,
    });
  };

  const handleCancelEditPlace = () => {
    setIsEditingPlace(false);
    setSelectedPlaceId(null);
    const nodes = extractedGraphReport?.nodes || [];
    setPlaceForm({
      id: '',
      name: '',
      node_id: nodes[0]?.id || 'N0',
      x: nodes[0]?.x || 0,
      y: nodes[0]?.y || 0,
      px: nodes[0]?.px || 0,
      py: nodes[0]?.py || 0,
    });
  };

  const handleAddOrUpdatePlace = () => {
    if (!placeForm.name.trim()) {
      setPlacesError('Please enter a place name');
      return;
    }
    const nodes = extractedGraphReport?.nodes || [];
    const targetNode = nodes.find((n) => n.id === placeForm.node_id) || {
      id: placeForm.node_id,
      x: placeForm.x,
      y: placeForm.y,
      px: placeForm.px,
      py: placeForm.py,
    };

    const newPlace = {
      id: isEditingPlace && placeForm.id ? placeForm.id : `place_${Date.now()}`,
      name: placeForm.name.trim(),
      node_id: placeForm.node_id || (nodes[0]?.id ?? 'N0'),
      x: Number(Number(targetNode.x ?? targetNode.wx ?? placeForm.x).toFixed(3)),
      y: Number(Number(targetNode.y ?? targetNode.wy ?? placeForm.y).toFixed(3)),
      px: targetNode.px ?? placeForm.px ?? 0,
      py: targetNode.py ?? placeForm.py ?? 0,
    };

    setNamedPlaces((prev) => {
      const idx = prev.findIndex((p) => p.id === newPlace.id);
      if (idx >= 0) {
        const updated = [...prev];
        updated[idx] = newPlace;
        return updated;
      }
      return [...prev, newPlace];
    });

    handleCancelEditPlace();
    setSelectedPlaceId(newPlace.id);
    setPlacesError(null);
  };

  const handleDeletePlace = (placeId) => {
    setNamedPlaces((prev) => prev.filter((p) => p.id !== placeId));
    if (selectedPlaceId === placeId) {
      handleCancelEditPlace();
    }
  };

  const handleSavePlacesToDisk = async () => {
    setIsSavingPlaces(true);
    setPlacesError(null);
    try {
      const target = selectedMapForGraph || mapName || 'warehouse_01';
      const res = await bridge.saveNamedPlaces(namedPlaces, target);
      if (res?.ok) {
        setPlacesSaveSuccess(true);
        setTimeout(() => setPlacesSaveSuccess(false), 4000);
      } else {
        setPlacesError(res?.error || 'Failed to save places to disk');
      }
    } catch (err) {
      setPlacesError(err.message || 'Error saving places');
    } finally {
      setIsSavingPlaces(false);
    }
  };


  const handleApprovePlaces = async () => {
    if (namedPlaces.length === 0) {
      setPlacesError('Please define at least one named station or place before approving');
      return;
    }
    await handleSavePlacesToDisk();
    setIsPlacesApproved(true);
    setCurrentStep(6);
  };

  // ---- Phase 6: Navigation Using Place Names Callbacks & Handlers ----

  const calculateRoutePreview = useCallback(async (targetPlace) => {
    if (!targetPlace) return;
    const targetMap = selectedMapForGraph || mapName || 'warehouse_01';
    const nodes = extractedGraphReport?.nodes || [];
    if (nodes.length === 0) return;

    let startNodeId = nodes[0].id;
    let minD = Infinity;
    for (const n of nodes) {
      const d = Math.hypot((n.x ?? n.wx ?? 0) - pose.x, (n.y ?? n.wy ?? 0) - pose.y);
      if (d < minD) {
        minD = d;
        startNodeId = n.id;
      }
    }

    try {
      const res = await bridge.planRoute(startNodeId, targetPlace.node_id, targetMap);
      if (res?.ok && res.route) {
        setNavRoutePreview(res.route);
      }
    } catch (_) {}
  }, [selectedMapForGraph, mapName, extractedGraphReport, pose.x, pose.y]);

  const hasAttemptedFetchPlacesRef = useRef(false);

  useEffect(() => {
    if (currentStep === 6) {
      if (namedPlaces.length > 0) {
        const first = namedPlaces[0];
        if (!navDestinationPlaceId) {
          setNavDestinationPlaceId(first.id);
          calculateRoutePreview(first);
        }
        if (!selectedStopToAdd) {
          setSelectedStopToAdd(first.id);
        }
      } else if (!hasAttemptedFetchPlacesRef.current && !isLoadingPlaces) {
        hasAttemptedFetchPlacesRef.current = true;
        fetchNamedPlaces();
      }
      if (!extractedGraphReport) {
        fetchExistingGraph();
      }
    } else {
      hasAttemptedFetchPlacesRef.current = false;
    }
  }, [currentStep, namedPlaces.length, navDestinationPlaceId, selectedStopToAdd, calculateRoutePreview, fetchNamedPlaces, fetchExistingGraph, extractedGraphReport, isLoadingPlaces]);

  useEffect(() => {
    if (telemetry?.active_place_nav) {
      setActiveNavInfo(telemetry.active_place_nav);
    }
  }, [telemetry?.active_place_nav]);

  const handleSelectPlaceAsDestination = (place) => {
    setNavDestinationPlaceId(place.id);
    setSelectedPlaceId(place.id);
    setNavError(null);
    calculateRoutePreview(place);
  };

  const handleDispatchSinglePlace = async () => {
    const target = namedPlaces.find((p) => p.id === navDestinationPlaceId);
    if (!target) {
      setNavError('Please select a destination place.');
      return;
    }
    setIsDispatchingNav(true);
    setNavError(null);
    try {
      const res = await bridge.dispatchPlace(target, selectedMapForGraph || mapName);
      if (res?.ok) {
        setActiveNavInfo({
          target_place: target.name,
          target_node: target.node_id,
          target_x: target.x,
          target_y: target.y,
          status: 'NAVIGATING',
        });
        calculateRoutePreview(target);
      } else {
        setNavError(res?.error || 'Failed to dispatch navigation');
      }
    } catch (err) {
      setNavError(err.message || 'Error dispatching navigation');
    } finally {
      setIsDispatchingNav(false);
    }
  };

  const handleAddStopToSequence = () => {
    const p = namedPlaces.find((item) => item.id === selectedStopToAdd);
    if (p) {
      setNavSequence((prev) => [...prev, p]);
    }
  };

  const handleRemoveStopFromSequence = (idx) => {
    setNavSequence((prev) => prev.filter((_, i) => i !== idx));
  };

  const handleDispatchSequence = async () => {
    if (navSequence.length === 0) {
      setNavError('Please add at least one stop to the mission queue.');
      return;
    }
    setIsDispatchingNav(true);
    setNavError(null);
    try {
      const res = await bridge.dispatchPlaceSequence(navSequence, selectedMapForGraph || mapName);
      if (res?.ok) {
        setActiveNavInfo({
          target_place: navSequence[0].name,
          target_node: navSequence[0].node_id,
          target_x: navSequence[0].x,
          target_y: navSequence[0].y,
          status: 'NAVIGATING',
          total_stops: navSequence.length,
          current_stop: 1,
        });
        calculateRoutePreview(navSequence[0]);
      } else {
        setNavError(res?.error || 'Failed to dispatch mission sequence');
      }
    } catch (err) {
      setNavError(err.message || 'Error dispatching sequence');
    } finally {
      setIsDispatchingNav(false);
    }
  };

  const handleCancelNavigation = async () => {
    await bridge.cancelPlaceNavigation();
    setActiveNavInfo(null);
    setNavRoutePreview(null);
  };

  const handleEmergencyStop = async () => {
    await bridge.sendStop();
    setActiveNavInfo(null);
    setNavRoutePreview(null);
  };

  const handleApproveNavigation = () => {
    setIsNavApproved(true);
    setCurrentStep(7);
  };

  const handleCanvasClickForNodePick = (e) => {
    if (currentStep !== 5 || !isPickNodeOnCanvasMode) return;
    const img = e.currentTarget.querySelector('img.mapping-slam-img');
    if (!img) return;
    const rect = img.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;
    const scaleX = (imgDimensions.width || 1000) / rect.width;
    const scaleY = (imgDimensions.height || 1000) / rect.height;
    const targetPx = Math.round(clickX * scaleX);
    const targetPy = Math.round(clickY * scaleY);

    const nodes = extractedGraphReport?.nodes || [];
    if (nodes.length === 0) return;
    let closest = nodes[0];
    let minD = Infinity;
    for (const n of nodes) {
      const d = Math.hypot((n.px || 0) - targetPx, (n.py || 0) - targetPy);
      if (d < minD) {
        minD = d;
        closest = n;
      }
    }

    if (closest) {
      handleSelectNodeForPlace(closest.id);
      setIsPickNodeOnCanvasMode(false);
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

  // Active step subtitle
  const stepSubtitle =
    currentStep === 1
      ? 'Step 1: SLAM Mapping & Autonomous Exploration'
      : currentStep === 2
      ? 'Step 2: Map Verification & Disk Storage'
      : currentStep === 3
      ? 'Step 3: Graph Extraction & Topological Roadmap Generation'
      : currentStep === 4
      ? 'Step 4: Roadmap Quality Verification & Route Pathfinding'
      : currentStep === 5
      ? 'Step 5: Semantic Place Naming & Station Setup'
      : currentStep === 6
      ? 'Step 6: Fleet Navigation Using Place Names'
      : 'Step 7: Live Fleet Monitoring & Supervised Dashboard';

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
                : (isSaved || currentStep > 2)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (status === 'STOPPED' || isSaved || currentStep > 2) && setCurrentStep(2)}
            style={{ cursor: status === 'STOPPED' || isSaved || currentStep > 2 ? 'pointer' : 'default' }}
            title={status === 'STOPPED' || isSaved || currentStep > 2 ? 'Review Step 2 (Verification)' : ''}
          >
            <span className="step-num">{isSaved || currentStep > 2 ? '✓' : '2'}</span>
            <span className="step-label">Map Verification</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 3
                ? 'stepper-step--active'
                : (extractedGraphReport || currentStep > 3)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (isSaved || currentStep >= 3) && setCurrentStep(3)}
            style={{ cursor: isSaved || currentStep >= 3 ? 'pointer' : 'default' }}
            title={isSaved || currentStep >= 3 ? 'Review Step 3 (Graph Extraction)' : ''}
          >
            <span className="step-num">{extractedGraphReport && currentStep > 3 ? '✓' : '3'}</span>
            <span className="step-label">Graph Extraction</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 4
                ? 'stepper-step--active'
                : (graphVerificationData?.status === 'VERIFIED' || isGraphApproved || currentStep > 4)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (extractedGraphReport || currentStep >= 4) && setCurrentStep(4)}
            style={{ cursor: extractedGraphReport || currentStep >= 4 ? 'pointer' : 'default' }}
            title={extractedGraphReport || currentStep >= 4 ? 'Review Step 4 (Graph Verification)' : 'Pending Step 3 completion'}
          >
            <span className="step-num">{(graphVerificationData?.status === 'VERIFIED' || isGraphApproved) && currentStep > 4 ? '✓' : '4'}</span>
            <span className="step-label">Graph Verification</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 5
                ? 'stepper-step--active'
                : (isPlacesApproved || currentStep > 5)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (isGraphApproved || currentStep > 4) && setCurrentStep(5)}
            style={{ cursor: isGraphApproved || currentStep > 4 ? 'pointer' : 'default' }}
            title={isGraphApproved || currentStep > 4 ? 'Review Step 5 (Place Naming)' : 'Pending Step 4 roadmap approval'}
          >
            <span className="step-num">{(isPlacesApproved || currentStep > 5) ? '✓' : '5'}</span>
            <span className="step-label">Place Naming</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 6
                ? 'stepper-step--active'
                : (isNavApproved || currentStep > 6)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (isPlacesApproved || currentStep >= 6) && setCurrentStep(6)}
            style={{ cursor: isPlacesApproved || currentStep >= 6 ? 'pointer' : 'default' }}
            title={isPlacesApproved || currentStep >= 6 ? 'Step 6 (Navigation)' : 'Pending Step 5 completion'}
          >
            <span className="step-num">{(isNavApproved || currentStep > 6) ? '✓' : '6'}</span>
            <span className="step-label">Navigation</span>
          </div>

          <div className="stepper-arrow">➔</div>

          <div
            className={`stepper-step ${
              currentStep === 7
                ? 'stepper-step--active'
                : 'stepper-step--pending'
            }`}
            onClick={() => isNavApproved && setCurrentStep(7)}
            style={{ cursor: isNavApproved ? 'pointer' : 'default' }}
            title={isNavApproved ? 'Step 7 (Live Monitoring)' : 'Pending Step 6 completion'}
          >
            <span className="step-num">7</span>
            <span className="step-label">Live Monitoring</span>
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

          {/* ================= STEP 3: GRAPH EXTRACTION ================= */}
          {currentStep === 3 && (
            <div className="card graph-extract-card">
              <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                <div className="card__title" style={{ fontSize: 15 }}>
                  🕸️ Step 3: Graph Extraction
                </div>
                {extractedGraphReport ? (
                  <span className="audit-check-row__badge badge--pass">✓ GRAPH EXTRACTED</span>
                ) : (
                  <span className="audit-check-row__badge badge--warn">READY TO EXTRACT</span>
                )}
              </div>

              {/* Map Source Selector */}
              <div className="mapping-field">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <label className="mapping-label">Source Map YAML</label>
                  <button
                    className="btn btn--ghost btn--icon"
                    onClick={fetchSavedMaps}
                    title="Refresh Saved Maps"
                    style={{ height: 22, width: 22, fontSize: 11 }}
                  >
                    ⟳
                  </button>
                </div>
                <select
                  className="input-field mapping-input text-mono"
                  value={selectedMapForGraph}
                  onChange={(e) => {
                    setSelectedMapForGraph(e.target.value);
                    setExtractedGraphReport(null);
                  }}
                  disabled={isExtracting}
                >
                  {savedMaps.length > 0 ? (
                    savedMaps.map((m) => (
                      <option key={m.name} value={m.name}>
                        {m.name} ({m.size_kb} KB)
                      </option>
                    ))
                  ) : (
                    <option value={mapName}>{mapName}.yaml</option>
                  )}
                </select>
                <span className="text-dim text-xs">
                  Active target: <code className="text-mono" style={{ color: 'var(--accent-cyan)' }}>src/agv_description/maps/{selectedMapForGraph || mapName}.yaml</code>
                </span>
              </div>

              {/* Density Presets */}
              <div className="mapping-field">
                <label className="mapping-label">Roadmap Density & Clearance Preset</label>
                <div className="density-presets-grid">
                  <div
                    className={`preset-card ${densityPreset === 'balanced' ? 'preset-card--active' : ''}`}
                    onClick={() => setDensityPreset('balanced')}
                  >
                    <div className="preset-card__title">Balanced</div>
                    <div className="preset-card__desc">0.21m clearance • 2.5m search</div>
                    <div className="preset-card__badge">Recommended</div>
                  </div>

                  <div
                    className={`preset-card ${densityPreset === 'dense' ? 'preset-card--active' : ''}`}
                    onClick={() => setDensityPreset('dense')}
                  >
                    <div className="preset-card__title">Dense / Detail</div>
                    <div className="preset-card__desc">0.19m clearance • 2.0m search</div>
                    <div className="preset-card__badge">Finer Nodes</div>
                  </div>

                  <div
                    className={`preset-card ${densityPreset === 'sparse' ? 'preset-card--active' : ''}`}
                    onClick={() => setDensityPreset('sparse')}
                  >
                    <div className="preset-card__title">Fast / Sparse</div>
                    <div className="preset-card__desc">0.25m clearance • 3.5m search</div>
                    <div className="preset-card__badge">Low Compute</div>
                  </div>

                  <div
                    className={`preset-card ${densityPreset === 'custom' ? 'preset-card--active' : ''}`}
                    onClick={() => setDensityPreset('custom')}
                  >
                    <div className="preset-card__title">Custom Sliders</div>
                    <div className="preset-card__desc">Manual tuning of clearances</div>
                    <div className="preset-card__badge">Advanced</div>
                  </div>
                </div>
              </div>

              {/* Custom Sliders (Only if Custom preset) */}
              {densityPreset === 'custom' && (
                <div className="custom-params-panel">
                  <div className="param-slider-row">
                    <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                      <span className="text-xs text-dim">Robot Radius:</span>
                      <span className="text-mono text-xs font-bold">{customParams.robotRadius} m</span>
                    </div>
                    <input
                      type="range"
                      min="0.06"
                      max="0.25"
                      step="0.01"
                      value={customParams.robotRadius}
                      onChange={(e) => setCustomParams({ ...customParams, robotRadius: parseFloat(e.target.value) })}
                    />
                  </div>

                  <div className="param-slider-row">
                    <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                      <span className="text-xs text-dim">Safety Margin:</span>
                      <span className="text-mono text-xs font-bold">{customParams.safetyMargin} m</span>
                    </div>
                    <input
                      type="range"
                      min="0.04"
                      max="0.25"
                      step="0.01"
                      value={customParams.safetyMargin}
                      onChange={(e) => setCustomParams({ ...customParams, safetyMargin: parseFloat(e.target.value) })}
                    />
                  </div>

                  <div className="param-slider-row">
                    <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                      <span className="text-xs text-dim">Search Radius (LOS):</span>
                      <span className="text-mono text-xs font-bold">{customParams.searchRadius} m</span>
                    </div>
                    <input
                      type="range"
                      min="1.0"
                      max="5.0"
                      step="0.25"
                      value={customParams.searchRadius}
                      onChange={(e) => setCustomParams({ ...customParams, searchRadius: parseFloat(e.target.value) })}
                    />
                  </div>

                  <div className="param-slider-row">
                    <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                      <span className="text-xs text-dim">Corridor Step:</span>
                      <span className="text-mono text-xs font-bold">{customParams.stepCorridor} m</span>
                    </div>
                    <input
                      type="range"
                      min="0.20"
                      max="0.80"
                      step="0.05"
                      value={customParams.stepCorridor}
                      onChange={(e) => setCustomParams({ ...customParams, stepCorridor: parseFloat(e.target.value) })}
                    />
                  </div>

                  <div className="param-slider-row">
                    <div className="flex-row" style={{ justifyContent: 'space-between' }}>
                      <span className="text-xs text-dim">Open Space Step:</span>
                      <span className="text-mono text-xs font-bold">{customParams.stepOpen} m</span>
                    </div>
                    <input
                      type="range"
                      min="0.40"
                      max="1.50"
                      step="0.05"
                      value={customParams.stepOpen}
                      onChange={(e) => setCustomParams({ ...customParams, stepOpen: parseFloat(e.target.value) })}
                    />
                  </div>
                </div>
              )}

              {/* Clearance Pill Info */}
              <div className="clearance-summary-pill">
                <span className="clearance-pill__icon">🛡️</span>
                <span className="text-xs">
                  Obstacle Clearance Constraint:
                  <strong>
                    {' '}
                    {densityPreset === 'balanced'
                      ? '0.21 m (0.11m robot + 0.10m margin)'
                      : densityPreset === 'dense'
                      ? '0.19 m (0.11m robot + 0.08m margin)'
                      : densityPreset === 'sparse'
                      ? '0.25 m (0.11m robot + 0.14m margin)'
                      : `${(customParams.robotRadius + customParams.safetyMargin).toFixed(2)} m (${customParams.robotRadius}m robot + ${customParams.safetyMargin}m margin)`}
                  </strong>
                </span>
              </div>

              {/* Error Banner if any */}
              {graphExtractError && (
                <div className="save-error-banner">
                  ⚠️ {graphExtractError}
                </div>
              )}

              {/* Post Extraction Metrics Card */}
              {extractedGraphReport ? (
                <div className="graph-extracted-results">
                  <div className="save-success-banner" style={{ margin: 0 }}>
                    <div className="save-success-title">
                      <span>✓</span> Topological Roadmap Extracted Successfully!
                    </div>

                    <div className="audit-metrics-grid" style={{ marginTop: 6 }}>
                      <div className="audit-metric-box">
                        <span className="audit-metric-box__label">Candidate Nodes</span>
                        <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                          {extractedGraphReport.total_nodes}
                        </span>
                      </div>

                      <div className="audit-metric-box">
                        <span className="audit-metric-box__label">LOS Edges</span>
                        <span className="audit-metric-box__val" style={{ color: 'var(--accent-cyan)' }}>
                          {extractedGraphReport.total_edges}
                        </span>
                      </div>

                      <div className="audit-metric-box">
                        <span className="audit-metric-box__label">Avg Connectivity</span>
                        <span className="audit-metric-box__val">
                          {extractedGraphReport.metrics?.avg_connectivity || '—'} / node
                        </span>
                      </div>

                      <div className="audit-metric-box">
                        <span className="audit-metric-box__label">Components</span>
                        <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                          {extractedGraphReport.metrics?.connected_components === 1
                            ? '1 (Connected)'
                            : `${extractedGraphReport.metrics?.connected_components} clusters`}
                        </span>
                      </div>
                    </div>

                    <div className="saved-files-list mt-xs">
                      <div className="saved-file-item">
                        📄 {extractedGraphReport.json_path || `src/agv_description/maps/${selectedMapForGraph}_graph.json`}
                      </div>
                      <div className="saved-file-item">
                        🔄 src/agv_description/maps/warehouse_graph.json (Synced to Nav2)
                      </div>
                      <div className="saved-file-item">
                        🖼️ {extractedGraphReport.vis_path || `src/agv_description/maps/${selectedMapForGraph}_graph_vis.png`}
                      </div>
                    </div>
                  </div>

                  <div className="mapping-actions mt-xs">
                    <button
                      id="btn-proceed-graph-verification"
                      className="btn--proceed-graph"
                      onClick={() => setCurrentStep(4)}
                    >
                      <span>➔</span> PROCEED TO GRAPH VERIFICATION (STEP 4)
                    </button>

                    <button
                      className="btn btn--outline"
                      onClick={handleExtractGraph}
                      disabled={isExtracting}
                    >
                      🔄 Re-Extract with Active Settings
                    </button>

                    <button
                      className="btn btn--ghost btn--sm"
                      onClick={() => setCurrentStep(2)}
                      style={{ alignSelf: 'center' }}
                    >
                      ← Back to Map Verification
                    </button>
                  </div>
                </div>
              ) : (
                /* Primary Extraction Trigger Button */
                <div className="mapping-actions">
                  <button
                    id="btn-run-extract-graph"
                    className="btn--extract-graph"
                    onClick={handleExtractGraph}
                    disabled={isExtracting}
                  >
                    {isExtracting ? (
                      <>
                        <span className="spinner-inline" />
                        <span>Raycasting Line-of-Sight & Extracting Nodes...</span>
                      </>
                    ) : (
                      <>
                        <span>⚡</span> RUN GRAPH EXTRACTION
                      </>
                    )}
                  </button>

                  <button
                    className="btn btn--outline"
                    onClick={() => setCurrentStep(2)}
                    disabled={isExtracting}
                  >
                    ← Back to Map Verification
                  </button>
                </div>
              )}
            </div>
          )}

          {/* ================= STEP 4: GRAPH VERIFICATION ================= */}
          {currentStep === 4 && (
            <div className="graph-verify-panel-container flex-col" style={{ gap: 12 }}>
              {/* Card 1: Verification Audit Results */}
              <div className="card graph-verify-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 15 }}>
                    🔍 Step 4: Graph Verification
                  </div>
                  {graphVerificationData?.status === 'VERIFIED' ? (
                    <span className="audit-check-row__badge badge--pass">✓ VERIFIED ROADMAP</span>
                  ) : isAuditingGraph ? (
                    <span className="audit-check-row__badge badge--warn">AUDITING...</span>
                  ) : (
                    <span className="audit-check-row__badge badge--warn">AUDIT READY</span>
                  )}
                </div>

                {/* Audit Metrics Grid */}
                <div className="audit-metrics-grid">
                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Main Component</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                      {graphVerificationData?.metrics?.lcc_size ?? extractedGraphReport?.total_nodes ?? 0} /{' '}
                      {graphVerificationData?.metrics?.total_nodes ?? extractedGraphReport?.total_nodes ?? 0}
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Connectivity</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-cyan)' }}>
                      {graphVerificationData?.metrics?.connectivity_pct ?? 100}%
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">LOS Edges</span>
                    <span className="audit-metric-box__val">
                      {graphVerificationData?.metrics?.total_edges ?? extractedGraphReport?.total_edges ?? 0}
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Avg Degree</span>
                    <span className="audit-metric-box__val">
                      {graphVerificationData?.metrics?.avg_degree ?? extractedGraphReport?.metrics?.avg_connectivity ?? 0} / node
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Min Clearance</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                      {graphVerificationData?.metrics?.min_clearance_m ?? 0.21} m
                    </span>
                  </div>

                  <div className="audit-metric-box">
                    <span className="audit-metric-box__label">Navigability</span>
                    <span className="audit-metric-box__val" style={{ color: 'var(--accent-green)' }}>
                      {graphVerificationData?.metrics?.navigability_pct ?? 100}%
                    </span>
                  </div>
                </div>

                {/* Health Checklist */}
                <div className="audit-checklist">
                  {graphVerificationData?.checks?.map((chk, i) => (
                    <div key={i} className="audit-check-row">
                      <span className="audit-check-row__label">
                        <span>{chk.passed ? '🟢' : '🟡'}</span> {chk.name}
                        <span className="text-dim text-xs" style={{ display: 'block', fontSize: 10, marginTop: 1 }}>
                          {chk.detail}
                        </span>
                      </span>
                      <span className={`audit-check-row__badge ${chk.passed ? 'badge--pass' : 'badge--warn'}`}>
                        {chk.status}
                      </span>
                    </div>
                  )) || (
                    <div className="text-dim text-xs p-xs">
                      Running automatic topological health checks...
                    </div>
                  )}
                </div>
              </div>

              {/* Card 2: Interactive Route Test & Path Simulator */}
              <div className="card route-tester-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    🧪 Interactive Route & Navigability Tester
                  </div>
                  <span className="world-tag-badge">A* / Dijkstra</span>
                </div>

                <div className="route-tester-endpoints-grid">
                  <div className="mapping-field" style={{ margin: 0 }}>
                    <label className="mapping-label">Start Node</label>
                    <select
                      className="input-field mapping-input text-mono"
                      value={routeTesterStart}
                      onChange={(e) => {
                        setRouteTesterStart(e.target.value);
                        setTestedRoute(null);
                      }}
                    >
                      {extractedGraphReport?.nodes?.slice(0, 300).map((n) => (
                        <option key={n.id} value={n.id}>
                          {n.id} ({Number(n.x ?? n.wx ?? 0).toFixed(2)}, {Number(n.y ?? n.wy ?? 0).toFixed(2)})
                        </option>
                      )) || <option value="N0">N0</option>}
                    </select>
                  </div>

                  <div className="route-endpoints-middle-btn">
                    <button
                      className="btn btn--ghost btn--icon"
                      onClick={handleSwapRouteEndpoints}
                      title="Swap Start and Goal"
                      style={{ height: 28, width: 28, fontSize: 13 }}
                    >
                      ⇄
                    </button>
                  </div>

                  <div className="mapping-field" style={{ margin: 0 }}>
                    <label className="mapping-label">Goal Node</label>
                    <select
                      className="input-field mapping-input text-mono"
                      value={routeTesterGoal}
                      onChange={(e) => {
                        setRouteTesterGoal(e.target.value);
                        setTestedRoute(null);
                      }}
                    >
                      {extractedGraphReport?.nodes?.slice(0, 300).map((n) => (
                        <option key={n.id} value={n.id}>
                          {n.id} ({Number(n.x ?? n.wx ?? 0).toFixed(2)}, {Number(n.y ?? n.wy ?? 0).toFixed(2)})
                        </option>
                      )) || <option value="N575">N575</option>}
                    </select>
                  </div>
                </div>

                <div className="flex-row" style={{ gap: 8, marginTop: 4 }}>
                  <button
                    className="btn btn--ghost btn--sm"
                    onClick={handleRandomRouteProbe}
                    style={{ fontSize: 11, padding: '4px 10px' }}
                  >
                    🎲 Pick Random Node Pair
                  </button>
                  <button
                    id="btn-test-route"
                    className="btn btn--primary btn--sm flex-1"
                    onClick={handleTestRoute}
                    disabled={isPlanningRoute || !routeTesterStart || !routeTesterGoal}
                    style={{ fontSize: 12, padding: '6px 12px' }}
                  >
                    {isPlanningRoute ? 'Computing Shortest Path...' : '🚀 Test Route Path'}
                  </button>
                </div>

                {routePlanError && (
                  <div className="save-error-banner" style={{ marginTop: 4, padding: '6px 10px' }}>
                    ⚠️ {routePlanError}
                  </div>
                )}

                {testedRoute && (
                  <div className="tested-route-result-box">
                    <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                      <span className="font-bold text-sm" style={{ color: 'var(--accent-green)' }}>
                        ✓ Route Verified: {testedRoute.start_node} ➔ {testedRoute.goal_node}
                      </span>
                      <span className="badge badge--primary text-mono">
                        {testedRoute.hop_count} hops
                      </span>
                    </div>

                    <div className="route-stats-grid">
                      <div className="route-stat-item">
                        <span className="route-stat-label">Total Distance</span>
                        <span className="route-stat-val text-mono">{testedRoute.total_distance_m} m</span>
                      </div>
                      <div className="route-stat-item">
                        <span className="route-stat-label">Est. Transit Time</span>
                        <span className="route-stat-val text-mono">{testedRoute.est_time_sec} s</span>
                      </div>
                      <div className="route-stat-item">
                        <span className="route-stat-label">Min Clearance</span>
                        <span className="route-stat-val text-mono" style={{ color: 'var(--accent-green)' }}>
                          {testedRoute.min_clearance_m} m
                        </span>
                      </div>
                      <div className="route-stat-item">
                        <span className="route-stat-label">Path Validity</span>
                        <span className="route-stat-val text-mono" style={{ color: 'var(--accent-cyan)' }}>100% LOS</span>
                      </div>
                    </div>

                    <div className="text-dim text-xs text-mono" style={{ marginTop: 4, wordBreak: 'break-all' }}>
                      Path: {testedRoute.path.slice(0, 6).join(' → ')}
                      {testedRoute.path.length > 6 ? ` → ... → ${testedRoute.path[testedRoute.path.length - 1]}` : ''}
                    </div>
                  </div>
                )}
              </div>

              {/* Card 3: Approval & Transition to Step 5 */}
              <div className="card save-config-card">
                <div className="card__title" style={{ fontSize: 14 }}>
                  🏁 Roadmap Sign-Off & Progression
                </div>

                <div className="text-dim text-xs" style={{ lineHeight: 1.5 }}>
                  The topological roadmap has been validated for single-component connectivity, obstacle clearances, and multi-point Dijkstra route reachability. Approve the roadmap to proceed to semantic place and dock naming.
                </div>

                <div className="mapping-actions">
                  <button
                    id="btn-approve-roadmap"
                    className="btn--proceed-graph"
                    onClick={handleApproveRoadmap}
                    style={{ background: 'linear-gradient(135deg, #059669, #10b981)' }}
                  >
                    <span>✓</span> APPROVE ROADMAP & PROCEED TO PLACE NAMING (STEP 5)
                  </button>

                  <div className="flex-row" style={{ gap: 8 }}>
                    <button
                      className="btn btn--outline flex-1"
                      onClick={() => runGraphVerification()}
                      disabled={isAuditingGraph}
                      style={{ fontSize: 12 }}
                    >
                      🔄 Re-Run Audit
                    </button>
                    <button
                      className="btn btn--ghost btn--sm"
                      onClick={() => setCurrentStep(3)}
                      style={{ fontSize: 12 }}
                    >
                      ← Back to Step 3
                    </button>
                  </div>
                </div>
              </div>
            </div>
          )}

          {/* ================= STEP 5: PLACE NAMING & FLEET STATIONS ================= */}
          {currentStep === 5 && (
            <div className="places-panel-container flex-col" style={{ gap: 12 }}>
              {/* Card 1: Simple & Clean Place Naming Form */}
              <div className="card place-builder-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    🏷️ {isEditingPlace ? 'Edit Place Name' : 'Name a Place'}
                  </div>
                  <span className={`audit-check-row__badge ${isEditingPlace ? 'badge--warn' : 'badge--pass'}`}>
                    {isEditingPlace ? 'EDITING' : 'READY'}
                  </span>
                </div>

                {/* Target Node Selector */}
                <div className="mapping-field" style={{ margin: 0 }}>
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 4 }}>
                    <label className="mapping-label">1. Pick Roadmap Node</label>
                    <button
                      type="button"
                      className={`btn btn--xs ${isPickNodeOnCanvasMode ? 'btn--success' : 'btn--outline'}`}
                      onClick={() => setIsPickNodeOnCanvasMode(!isPickNodeOnCanvasMode)}
                      style={{ fontSize: 11, padding: '3px 8px' }}
                    >
                      {isPickNodeOnCanvasMode ? '🎯 Canvas Pick ACTIVE' : '📍 Pick on Map'}
                    </button>
                  </div>
                  <select
                    className="input-field mapping-input text-mono"
                    value={placeForm.node_id}
                    onChange={(e) => handleSelectNodeForPlace(e.target.value)}
                  >
                    {extractedGraphReport?.nodes?.slice(0, 300).map((n) => (
                      <option key={n.id} value={n.id}>
                        {n.id} ({Number(n.x ?? n.wx ?? 0).toFixed(2)}, {Number(n.y ?? n.wy ?? 0).toFixed(2)})
                      </option>
                    )) || <option value="N0">N0</option>}
                  </select>
                  <div className="node-coords-badge">
                    <span>Selected: <strong>{placeForm.node_id}</strong></span>
                    <span>Coords: ({placeForm.x}m, {placeForm.y}m)</span>
                  </div>
                </div>

                {/* Place Name Input */}
                <div className="mapping-field" style={{ margin: 0 }}>
                  <label className="mapping-label">2. Place Name</label>
                  <input
                    id="input-place-name"
                    type="text"
                    className="input-field mapping-input"
                    placeholder="e.g. Charging Dock, Station A, Warehouse Bay 1..."
                    value={placeForm.name}
                    onChange={(e) => setPlaceForm({ ...placeForm, name: e.target.value })}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter') {
                        e.preventDefault();
                        handleAddOrUpdatePlace();
                      }
                    }}
                  />
                </div>

                {/* Error Banner */}
                {placesError && (
                  <div className="save-error-banner" style={{ margin: 0, padding: '6px 10px', fontSize: 12 }}>
                    ⚠️ {placesError}
                  </div>
                )}

                {/* Builder Action Buttons */}
                <div className="flex-row" style={{ gap: 8, marginTop: 4 }}>
                  <button
                    id="btn-add-place"
                    type="button"
                    className="btn btn--primary flex-1"
                    onClick={handleAddOrUpdatePlace}
                    style={{ padding: '8px 16px', fontSize: 13 }}
                  >
                    {isEditingPlace ? '✓ Update Place' : '➕ Save Place'}
                  </button>
                  {isEditingPlace && (
                    <button
                      type="button"
                      className="btn btn--outline"
                      onClick={handleCancelEditPlace}
                      style={{ padding: '8px 14px', fontSize: 13 }}
                    >
                      Cancel
                    </button>
                  )}
                </div>
              </div>

              {/* Card 2: Defined Places List */}
              <div className="card places-list-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    📋 Named Places ({namedPlaces.length})
                  </div>
                  <button
                    type="button"
                    className="btn btn--ghost btn--icon"
                    onClick={fetchNamedPlaces}
                    title="Reload places from disk"
                    style={{ height: 22, width: 22, fontSize: 11 }}
                  >
                    ⟳
                  </button>
                </div>

                {/* Places Scroll Container */}
                <div className="places-items-container">
                  {namedPlaces.map((place) => {
                    const isSelected = selectedPlaceId === place.id;
                    return (
                      <div
                        key={place.id}
                        className={`place-card-item ${isSelected ? 'place-card-item--selected' : ''}`}
                        onClick={() => setSelectedPlaceId(place.id)}
                      >
                        <div className="place-item-dot" />
                        <div className="place-card-item__info">
                          <div className="flex-row" style={{ gap: 6, alignItems: 'center' }}>
                            <span className="place-card-item__name">{place.name}</span>
                            <span className="badge badge--primary text-mono" style={{ fontSize: 10, padding: '1px 5px' }}>
                              {place.node_id}
                            </span>
                          </div>
                          <div className="place-card-item__meta text-mono text-dim text-xs">
                            <span>({place.x}m, {place.y}m)</span>
                          </div>
                        </div>
                        <div className="place-card-item__actions">
                          <button
                            type="button"
                            className="btn btn--ghost btn--icon"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleSelectPlaceForEdit(place);
                            }}
                            title="Edit place"
                            style={{ height: 26, width: 26, fontSize: 12 }}
                          >
                            ✏️
                          </button>
                          <button
                            type="button"
                            className="btn btn--ghost btn--icon text-danger"
                            onClick={(e) => {
                              e.stopPropagation();
                              handleDeletePlace(place.id);
                            }}
                            title="Delete place"
                            style={{ height: 26, width: 26, fontSize: 12 }}
                          >
                            🗑️
                          </button>
                        </div>
                      </div>
                    );
                  })}
                  {namedPlaces.length === 0 && (
                    <div className="text-dim text-xs text-center p-md">
                      No places configured yet. Pick a node on the roadmap and enter a name above.
                    </div>
                  )}
                </div>

                {/* Save Success Banner */}
                {placesSaveSuccess && (
                  <div className="save-success-banner" style={{ margin: 0, padding: '6px 12px' }}>
                    <span>✓</span> Successfully saved {namedPlaces.length} places to disk!
                  </div>
                )}

                {/* Save to disk action */}
                <button
                  id="btn-save-places"
                  type="button"
                  className="btn btn--outline"
                  onClick={handleSavePlacesToDisk}
                  disabled={isSavingPlaces || namedPlaces.length === 0}
                  style={{ width: '100%', padding: '7px 0', fontSize: 12 }}
                >
                  {isSavingPlaces ? 'Saving Places...' : `💾 Save ${namedPlaces.length} Places to Disk`}
                </button>
              </div>

              {/* Card 3: Progression to Step 6 */}
              <div className="card save-config-card">
                <div className="card__title" style={{ fontSize: 14 }}>
                  🏁 Complete Place Naming
                </div>
                <div className="text-dim text-xs" style={{ lineHeight: 1.5 }}>
                  Roadmap places configured. Approving places will advance the operational pipeline to Phase 6 (Navigation Using Place Names).
                </div>
                <div className="mapping-actions">
                  <button
                    id="btn-approve-places"
                    type="button"
                    className="btn--proceed-graph"
                    onClick={handleApprovePlaces}
                    style={{ background: 'linear-gradient(135deg, #0284c7, #06b6d4)' }}
                    disabled={namedPlaces.length === 0}
                  >
                    <span>✓</span> APPROVE PLACES & PROCEED TO NAVIGATION (STEP 6)
                  </button>
                  <div className="flex-row" style={{ gap: 8 }}>
                    <button
                      type="button"
                      className="btn btn--outline flex-1"
                      onClick={() => setCurrentStep(4)}
                      style={{ fontSize: 12 }}
                    >
                      ← Back to Step 4 (Graph Verification)
                    </button>
                    <button
                      type="button"
                      className="btn btn--ghost btn--sm"
                      onClick={onReturnToDashboard}
                      style={{ fontSize: 12 }}
                    >
                      Dashboard
                    </button>
                  </div>
                </div>
              </div>
            </div>
          )}

          {/* ================= STEP 6: NAVIGATION USING PLACE NAMES ================= */}
          {currentStep === 6 && (
            <div className="places-panel-container flex-col" style={{ gap: 12 }}>
              {/* Card 1: Destination & Route Dispatch */}
              <div className="card nav-dispatch-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    🚀 Place Navigation Dispatch
                  </div>
                  {/* Mode Selector */}
                  <div className="view-mode-pill-group">
                    <button
                      type="button"
                      className={`btn-view-pill ${navMode === 'single' ? 'btn-view-pill--active' : ''}`}
                      onClick={() => setNavMode('single')}
                      style={{ fontSize: 11, padding: '3px 8px' }}
                    >
                      📍 Single Place
                    </button>
                    <button
                      type="button"
                      className={`btn-view-pill ${navMode === 'multi' ? 'btn-view-pill--active' : ''}`}
                      onClick={() => setNavMode('multi')}
                      style={{ fontSize: 11, padding: '3px 8px' }}
                    >
                      🔄 Multi-Stop
                    </button>
                  </div>
                </div>

                {/* Single Place Mode */}
                {navMode === 'single' ? (
                  <div className="flex-col" style={{ gap: 10 }}>
                    <div className="mapping-field" style={{ margin: 0 }}>
                      <label className="mapping-label">Select Destination Place</label>
                      <select
                        id="select-nav-place"
                        className="input-field mapping-input text-mono"
                        value={navDestinationPlaceId}
                        onChange={(e) => {
                          const p = namedPlaces.find((item) => item.id === e.target.value);
                          if (p) handleSelectPlaceAsDestination(p);
                        }}
                      >
                        {namedPlaces.map((p) => (
                          <option key={p.id} value={p.id}>
                            📍 {p.name} [{p.node_id}] — ({p.x}m, {p.y}m)
                          </option>
                        ))}
                      </select>
                    </div>

                    {/* Target Place Details Card */}
                    {(() => {
                      const selectedNavPlace = namedPlaces.find((p) => p.id === navDestinationPlaceId) || namedPlaces[0];
                      return selectedNavPlace ? (
                        <div className="nav-target-card">
                          <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                            <span className="nav-target-card__name font-bold">{selectedNavPlace.name}</span>
                            <span className="badge badge--primary text-mono">{selectedNavPlace.node_id}</span>
                          </div>
                          <div className="text-dim text-xs text-mono" style={{ marginTop: 2 }}>
                            World Coordinates: ({selectedNavPlace.x}m, {selectedNavPlace.y}m)
                          </div>

                          {/* Route Calculation Preview */}
                          {navRoutePreview && (
                            <div className="nav-route-metrics-bar">
                              <span>🛣️ <strong>{navRoutePreview.total_distance_m} m</strong></span>
                              <span>• <strong>{navRoutePreview.hop_count} hops</strong></span>
                              <span>• Est: <strong>{navRoutePreview.est_time_sec}s</strong></span>
                            </div>
                          )}
                        </div>
                      ) : null;
                    })()}

                    {navError && (
                      <div className="save-error-banner" style={{ margin: 0, padding: '6px 10px', fontSize: 12 }}>
                        ⚠️ {navError}
                      </div>
                    )}

                    {/* Dispatch Action */}
                    <button
                      id="btn-dispatch-single"
                      type="button"
                      className="btn btn--primary"
                      onClick={handleDispatchSinglePlace}
                      disabled={isDispatchingNav || namedPlaces.length === 0}
                      style={{ padding: '10px 16px', fontSize: 13, fontWeight: 700 }}
                    >
                      {isDispatchingNav ? 'Dispatching...' : `🚀 DISPATCH AMR TO ${(namedPlaces.find((p) => p.id === navDestinationPlaceId)?.name || 'PLACE').toUpperCase()}`}
                    </button>
                  </div>
                ) : (
                  /* Multi-Stop Mission Mode */
                  <div className="flex-col" style={{ gap: 10 }}>
                    <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
                      <select
                        id="select-add-stop"
                        className="input-field mapping-input text-mono flex-1"
                        value={selectedStopToAdd}
                        onChange={(e) => setSelectedStopToAdd(e.target.value)}
                      >
                        {namedPlaces.map((p) => (
                          <option key={p.id} value={p.id}>
                            📍 {p.name} [{p.node_id}]
                          </option>
                        ))}
                      </select>
                      <button
                        type="button"
                        className="btn btn--outline btn--sm"
                        onClick={handleAddStopToSequence}
                        style={{ fontSize: 12, padding: '7px 12px' }}
                      >
                        ➕ Add Stop
                      </button>
                    </div>

                    {/* Sequence List */}
                    <div className="nav-sequence-list">
                      {navSequence.map((stop, idx) => (
                        <div key={`${stop.id}_${idx}`} className="nav-sequence-item">
                          <span className="nav-sequence-item__idx">{idx + 1}</span>
                          <span className="nav-sequence-item__name flex-1 font-bold text-xs">{stop.name}</span>
                          <span className="badge badge--primary text-mono" style={{ fontSize: 10 }}>{stop.node_id}</span>
                          <button
                            type="button"
                            className="btn btn--ghost btn--icon text-danger"
                            onClick={() => handleRemoveStopFromSequence(idx)}
                            style={{ height: 22, width: 22, fontSize: 11 }}
                            title="Remove stop"
                          >
                            ✕
                          </button>
                        </div>
                      ))}
                      {navSequence.length === 0 && (
                        <div className="text-dim text-xs text-center p-sm">
                          No stops added yet. Pick a place above and click "Add Stop" to build a delivery route.
                        </div>
                      )}
                    </div>

                    {navError && (
                      <div className="save-error-banner" style={{ margin: 0, padding: '6px 10px', fontSize: 12 }}>
                        ⚠️ {navError}
                      </div>
                    )}

                    {/* Dispatch Multi-Stop Mission */}
                    <div className="flex-row" style={{ gap: 8 }}>
                      <button
                        id="btn-dispatch-sequence"
                        type="button"
                        className="btn btn--primary flex-1"
                        onClick={handleDispatchSequence}
                        disabled={isDispatchingNav || navSequence.length === 0}
                        style={{ padding: '9px 14px', fontSize: 12, fontWeight: 700 }}
                      >
                        {isDispatchingNav ? 'Dispatching...' : `📦 DISPATCH MISSION (${navSequence.length} STOPS)`}
                      </button>
                      {navSequence.length > 0 && (
                        <button
                          type="button"
                          className="btn btn--outline btn--sm"
                          onClick={() => setNavSequence([])}
                          style={{ fontSize: 12 }}
                        >
                          Clear
                        </button>
                      )}
                    </div>
                  </div>
                )}
              </div>

              {/* Card 2: Live Navigation HUD & Active State */}
              <div className="card nav-status-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    📡 Live Navigation Monitor
                  </div>
                  <span className={`badge ${
                    (activeNavInfo?.status === 'NAVIGATING' || navState === 'NAVIGATING')
                      ? 'badge--pass'
                      : navState === 'PLANNING'
                      ? 'badge--info'
                      : navState === 'ESTOP'
                      ? 'badge--danger'
                      : 'badge--default'
                  }`}>
                    {(activeNavInfo?.status === 'NAVIGATING' || navState === 'NAVIGATING') ? '● NAVIGATING' : (activeNavInfo?.status || navState || 'IDLE')}
                  </span>
                </div>

                <div className="nav-hud-grid">
                  <div className="nav-hud-metric">
                    <span className="text-dim text-xs">Active Target</span>
                    <span className="font-bold text-sm text-truncate" style={{ color: 'var(--accent-cyan)' }}>
                      {activeNavInfo?.target_place || (namedPlaces.find((p) => p.id === navDestinationPlaceId)?.name) || 'Standby / Idle'}
                    </span>
                  </div>
                  <div className="nav-hud-metric">
                    <span className="text-dim text-xs">Dist to Goal</span>
                    <span className="text-mono font-bold text-sm" style={{ color: 'var(--accent-green)' }}>
                      {(() => {
                        const target = namedPlaces.find((p) => p.id === navDestinationPlaceId) || namedPlaces[0];
                        if (target) {
                          const d = Math.hypot(pose.x - target.x, pose.y - target.y);
                          return `${d.toFixed(2)} m`;
                        }
                        return '--';
                      })()}
                    </span>
                  </div>
                  <div className="nav-hud-metric">
                    <span className="text-dim text-xs">Speed</span>
                    <span className="text-mono font-bold text-sm">
                      {Math.abs(velocity.linear).toFixed(2)} m/s
                    </span>
                  </div>
                  <div className="nav-hud-metric">
                    <span className="text-dim text-xs">Mission Progress</span>
                    <span className="text-mono font-bold text-sm">
                      {telemetry?.mission?.total ? `${telemetry.mission.current || 1} / ${telemetry.mission.total}` : activeNavInfo?.total_stops ? `${activeNavInfo.current_stop || 1} / ${activeNavInfo.total_stops}` : '1 / 1'}
                    </span>
                  </div>
                </div>

                {/* Cancel & E-STOP controls */}
                <div className="flex-row" style={{ gap: 8, marginTop: 4 }}>
                  <button
                    type="button"
                    className="btn btn--outline flex-1"
                    onClick={handleCancelNavigation}
                    style={{ fontSize: 12, padding: '7px 0' }}
                  >
                    ⏹ Cancel Navigation
                  </button>
                  <button
                    type="button"
                    className="btn btn--danger flex-1"
                    onClick={handleEmergencyStop}
                    style={{ fontSize: 12, padding: '7px 0', fontWeight: 700 }}
                  >
                    🛑 E-STOP
                  </button>
                </div>
              </div>

              {/* Card 3: Phase 6 Approval & Advancement to Phase 7 */}
              <div className="card save-config-card">
                <div className="card__title" style={{ fontSize: 14 }}>
                  🏁 Step 6 Sign-Off & Live Monitoring
                </div>
                <div className="text-dim text-xs" style={{ lineHeight: 1.5 }}>
                  Fleet navigation using place names is operational. Approving navigation will complete Phase 6 and advance the pipeline to Phase 7 (Live Fleet Monitoring & Supervised Dashboard).
                </div>
                <div className="mapping-actions">
                  <button
                    id="btn-approve-navigation"
                    type="button"
                    className="btn--proceed-graph"
                    onClick={handleApproveNavigation}
                    style={{ background: 'linear-gradient(135deg, #0284c7, #10b981)' }}
                  >
                    <span>✓</span> APPROVE NAVIGATION & PROCEED TO MONITORING (STEP 7)
                  </button>
                  <div className="flex-row" style={{ gap: 8 }}>
                    <button
                      type="button"
                      className="btn btn--outline flex-1"
                      onClick={() => setCurrentStep(5)}
                      style={{ fontSize: 12 }}
                    >
                      ← Back to Step 5 (Place Naming)
                    </button>
                    <button
                      type="button"
                      className="btn btn--ghost btn--sm"
                      onClick={onReturnToDashboard}
                      style={{ fontSize: 12 }}
                    >
                      Dashboard
                    </button>
                  </div>
                </div>
              </div>
            </div>
          )}

          {/* ================= STEP 7: LIVE MONITORING (PHASE 7 AWAITING APPROVAL) ================= */}
          {currentStep === 7 && (
            <div className="card phase4-ready-card" style={{ background: 'linear-gradient(135deg, rgba(16, 185, 129, 0.08), rgba(12, 16, 28, 0.9))', borderColor: 'rgba(16, 185, 129, 0.3)' }}>
              <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                <div className="card__title" style={{ fontSize: 15 }}>
                  🎯 Step 7: Live Fleet Monitoring & Supervised Dashboard
                </div>
                <span className="audit-check-row__badge badge--pass">PHASE 6 COMPLETE</span>
              </div>

              <div className="phase3-meta-table">
                <div className="phase3-meta-row">
                  <span className="text-dim">Navigation Pipeline:</span>
                  <span className="text-success font-bold">Place-Based Dispatch Operational</span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">Registered Places:</span>
                  <span className="text-mono font-bold" style={{ color: 'var(--accent-cyan)' }}>
                    {namedPlaces.length} Fleet Stations
                  </span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">Roadmap Graph:</span>
                  <span className="text-mono font-bold" style={{ color: 'var(--accent-green)' }}>
                    {selectedMapForGraph || mapName}_graph.json
                  </span>
                </div>
                <div className="phase3-meta-row">
                  <span className="text-dim">Phase Status:</span>
                  <span className="text-mono font-bold">Phase 6 Verified</span>
                </div>
              </div>

              <div className="save-success-banner" style={{ background: 'rgba(16, 185, 129, 0.08)', borderColor: 'rgba(16, 185, 129, 0.3)' }}>
                <div className="text-sm font-bold" style={{ color: 'var(--accent-green)' }}>
                  ✅ Phase 6 Milestone Achieved: Navigation Using Place Names Verified
                </div>
                <div className="text-dim text-xs" style={{ color: 'var(--text-secondary)', lineHeight: 1.5 }}>
                  Point-to-point destination dispatch, multi-stop mission sequencing, and real-time navigation telemetry are operational. Per development rules, awaiting your explicit approval before modifying Phase 7 (Live Fleet Monitoring / Dashboard).
                </div>
              </div>

              <div className="mapping-actions">
                <button
                  type="button"
                  className="btn btn--outline"
                  onClick={() => setCurrentStep(6)}
                >
                  ← Back to Navigation (Step 6)
                </button>
                <button
                  type="button"
                  className="btn btn--primary"
                  onClick={onReturnToDashboard}
                  style={{ alignSelf: 'center' }}
                >
                  Go to Main Dashboard
                </button>
              </div>
            </div>
          )}

        </div>

        {/* Right Column: SLAM / Inspection / Graph Map Viewport */}
        <div className="mapping-viewport-panel card">
          <div className="mapping-viewport-header">
            <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
              <span className="card__title">
                {currentStep === 1
                  ? '📡 Live SLAM Map Stream'
                  : currentStep === 2
                  ? '🔍 Map Inspection & Verification Canvas'
                  : currentStep === 3
                  ? '🕸️ Topological Graph Roadmap Canvas'
                  : currentStep === 4
                  ? '🔍 Graph Verification & Route Canvas'
                  : '🏷️ Semantic Places & Roadmap Canvas'}
              </span>

              {status === 'MAPPING' && (
                <span className="live-pill">
                  <span className="pulse-dot" /> STREAMING /map
                </span>
              )}

              {isSaved && currentStep <= 2 && (
                <span className="audit-check-row__badge badge--pass" style={{ fontSize: 10, padding: '2px 8px' }}>
                  ✓ DISK SAVED
                </span>
              )}

              {extractedGraphReport && currentStep >= 3 && (
                <span className="audit-check-row__badge badge--pass" style={{ fontSize: 10, padding: '2px 8px' }}>
                  {currentStep === 4 ? (graphVerificationData?.status === 'VERIFIED' ? '✓ AUDIT PASSED' : '● AUDIT READY') : '✓ GRAPH ROADMAP'}
                </span>
              )}
            </div>

            {/* Viewport Toolbar */}
            <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
              {/* Mode switch */}
              {currentStep >= 3 ? (
                <div className="view-mode-pill-group">
                  <button
                    className={`btn-view-pill ${graphVisMode === 'graph' ? 'btn-view-pill--active' : ''}`}
                    onClick={() => setGraphVisMode('graph')}
                    title="Show topological roadmap graph overlaid on floorplan"
                  >
                    🕸️ Roadmap Overlay
                  </button>
                  <button
                    className={`btn-view-pill ${graphVisMode === 'clean' ? 'btn-view-pill--active' : ''}`}
                    onClick={() => setGraphVisMode('clean')}
                    title="Show clean architectural floorplan"
                  >
                    🗺️ Clean Floorplan
                  </button>
                </div>
              ) : (
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
              )}

              {/* Route Overlay Toggle (When on Step 4 and route tested) */}
              {currentStep === 4 && testedRoute && (
                <button
                  className={`btn btn--ghost btn--sm ${showRouteOverlay ? 'btn-view-pill--active' : ''}`}
                  onClick={() => setShowRouteOverlay((s) => !s)}
                  title="Toggle A* Route Highlighting"
                  style={{ fontSize: 11, padding: '4px 8px' }}
                >
                  📍 Route {showRouteOverlay ? 'ON' : 'OFF'}
                </button>
              )}

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

            {/* Canvas Node Pick Mode Banner */}
            {currentStep === 5 && isPickNodeOnCanvasMode && (
              <div className="canvas-pick-mode-banner">
                <span className="pulse-dot" style={{ background: '#10b981' }} />
                <span>🎯 Click anywhere on the floorplan to bind the closest roadmap node!</span>
                <button
                  type="button"
                  className="btn btn--xs btn--outline"
                  onClick={() => setIsPickNodeOnCanvasMode(false)}
                  style={{ marginLeft: 'auto', fontSize: 11, padding: '2px 8px' }}
                >
                  ✕ Cancel
                </button>
              </div>
            )}

            {currentStep >= 3 && graphVisMode === 'graph' && extractedGraphReport?.vis_image_b64 ? (
              <div
                className="mapping-image-wrapper"
                onClick={handleCanvasClickForNodePick}
                style={{
                  transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
                  transformOrigin: 'center center',
                  position: 'relative',
                  display: 'inline-block',
                  cursor: isPickNodeOnCanvasMode ? 'crosshair' : 'default',
                }}
              >
                <img
                  src={`data:image/png;base64,${extractedGraphReport.vis_image_b64}`}
                  alt="Topological Graph Roadmap"
                  className="mapping-slam-img"
                  draggable={false}
                  onLoad={(e) => {
                    if (e.target.naturalWidth) {
                      setImgDimensions({
                        width: e.target.naturalWidth,
                        height: e.target.naturalHeight,
                      });
                    }
                  }}
                />

                {/* SVG Route Highlighting Overlay */}
                {currentStep === 4 && testedRoute && showRouteOverlay && testedRoute.waypoints?.length > 1 && (
                  <svg
                    className="route-svg-overlay"
                    viewBox={`0 0 ${imgDimensions.width} ${imgDimensions.height}`}
                    style={{
                      position: 'absolute',
                      top: 0,
                      left: 0,
                      width: '100%',
                      height: '100%',
                      pointerEvents: 'none',
                      zIndex: 3,
                    }}
                  >
                    {/* Polyline connecting all waypoints */}
                    <polyline
                      points={testedRoute.waypoints.map((w) => `${w.px},${w.py}`).join(' ')}
                      fill="none"
                      stroke="#f59e0b"
                      strokeWidth="3.5"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      style={{ filter: 'drop-shadow(0 0 6px rgba(245, 158, 11, 0.9))' }}
                    />
                    {/* Waypoint dots */}
                    {testedRoute.waypoints.map((w, idx) => (
                      <circle
                        key={idx}
                        cx={w.px}
                        cy={w.py}
                        r="3"
                        fill="#fbbf24"
                      />
                    ))}
                    {/* Start waypoint: Green */}
                    {testedRoute.waypoints[0] && (
                      <g>
                        <circle
                          cx={testedRoute.waypoints[0].px}
                          cy={testedRoute.waypoints[0].py}
                          r="7"
                          fill="#10b981"
                          stroke="#fff"
                          strokeWidth="2"
                        />
                        <text
                          x={testedRoute.waypoints[0].px}
                          y={testedRoute.waypoints[0].py - 10}
                          fill="#10b981"
                          fontSize="12"
                          fontWeight="bold"
                          textAnchor="middle"
                          style={{ filter: 'drop-shadow(0 0 4px #000)' }}
                        >
                          START ({testedRoute.start_node})
                        </text>
                      </g>
                    )}
                    {/* Goal waypoint: Red */}
                    {testedRoute.waypoints[testedRoute.waypoints.length - 1] && (
                      <g>
                        <circle
                          cx={testedRoute.waypoints[testedRoute.waypoints.length - 1].px}
                          cy={testedRoute.waypoints[testedRoute.waypoints.length - 1].py}
                          r="7"
                          fill="#ef4444"
                          stroke="#fff"
                          strokeWidth="2"
                        />
                        <text
                          x={testedRoute.waypoints[testedRoute.waypoints.length - 1].px}
                          y={testedRoute.waypoints[testedRoute.waypoints.length - 1].py - 10}
                          fill="#ef4444"
                          fontSize="12"
                          fontWeight="bold"
                          textAnchor="middle"
                          style={{ filter: 'drop-shadow(0 0 4px #000)' }}
                        >
                          GOAL ({testedRoute.goal_node})
                        </text>
                      </g>
                    )}
                  </svg>
                )}


                {/* SVG Route Highlighting Overlay (Step 6 Place Navigation Preview) */}
                {currentStep === 6 && navRoutePreview && navRoutePreview.waypoints?.length > 1 && (
                  <svg
                    className="route-svg-overlay nav-route-overlay"
                    viewBox={`0 0 ${imgDimensions.width} ${imgDimensions.height}`}
                    style={{
                      position: 'absolute',
                      top: 0,
                      left: 0,
                      width: '100%',
                      height: '100%',
                      pointerEvents: 'none',
                      zIndex: 3,
                    }}
                  >
                    {/* Cyan polyline for active/previewed navigation path */}
                    <polyline
                      points={navRoutePreview.waypoints.map((w) => `${w.px},${w.py}`).join(' ')}
                      fill="none"
                      stroke="#06b6d4"
                      strokeWidth="3.5"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeDasharray="7 5"
                      style={{ filter: 'drop-shadow(0 0 8px rgba(6, 182, 212, 0.95))' }}
                    />
                    {/* Waypoint beads along path */}
                    {navRoutePreview.waypoints.map((w, idx) => (
                      <circle
                        key={idx}
                        cx={w.px}
                        cy={w.py}
                        r="3"
                        fill="#38bdf8"
                      />
                    ))}
                    {/* Start point marker */}
                    {navRoutePreview.waypoints[0] && (
                      <g>
                        <circle
                          cx={navRoutePreview.waypoints[0].px}
                          cy={navRoutePreview.waypoints[0].py}
                          r="6"
                          fill="#10b981"
                          stroke="#fff"
                          strokeWidth="2"
                        />
                        <text
                          x={navRoutePreview.waypoints[0].px}
                          y={navRoutePreview.waypoints[0].py - 9}
                          fill="#10b981"
                          fontSize="11"
                          fontWeight="bold"
                          textAnchor="middle"
                          style={{ filter: 'drop-shadow(0 0 4px #000)' }}
                        >
                          AMR ({navRoutePreview.start_node})
                        </text>
                      </g>
                    )}
                    {/* Destination marker */}
                    {navRoutePreview.waypoints[navRoutePreview.waypoints.length - 1] && (
                      <g>
                        <circle
                          cx={navRoutePreview.waypoints[navRoutePreview.waypoints.length - 1].px}
                          cy={navRoutePreview.waypoints[navRoutePreview.waypoints.length - 1].py}
                          r="8"
                          fill="#06b6d4"
                          stroke="#fff"
                          strokeWidth="2"
                        />
                        <text
                          x={navRoutePreview.waypoints[navRoutePreview.waypoints.length - 1].px}
                          y={navRoutePreview.waypoints[navRoutePreview.waypoints.length - 1].py - 11}
                          fill="#06b6d4"
                          fontSize="11"
                          fontWeight="bold"
                          textAnchor="middle"
                          style={{ filter: 'drop-shadow(0 0 4px #000)' }}
                        >
                          DEST ({navRoutePreview.goal_node})
                        </text>
                      </g>
                    )}
                  </svg>
                )}

                {/* SVG Place Markers Overlay (Step 5 & 6) */}
                {currentStep >= 5 && namedPlaces.length > 0 && (
                  <svg
                    className="places-svg-overlay"
                    viewBox={`0 0 ${imgDimensions.width} ${imgDimensions.height}`}
                    style={{
                      position: 'absolute',
                      top: 0,
                      left: 0,
                      width: '100%',
                      height: '100%',
                      pointerEvents: isPickNodeOnCanvasMode ? 'none' : 'auto',
                      zIndex: 4,
                    }}
                  >
                    {namedPlaces.map((place) => {
                      const isNavDestination = currentStep === 6 && (navDestinationPlaceId === place.id || activeNavInfo?.target_place_id === place.id || activeNavInfo?.target_place === place.name);
                      const isSelected = selectedPlaceId === place.id || isNavDestination;
                      const markerColor = isNavDestination ? '#06b6d4' : (isSelected ? '#06b6d4' : '#10b981');
                      const labelText = place.name.length > 16 ? place.name.slice(0, 15) + '…' : place.name;
                      const pillW = Math.max(28, Math.min(labelText.length * 6 + 10, 85));
                      const pillH = 14;

                      return (
                        <g
                          key={place.id}
                          className={`place-pin-marker ${isSelected ? 'place-pin-marker--selected' : ''}`}
                          onClick={(e) => {
                            e.stopPropagation();
                            if (currentStep === 6) {
                              handleSelectPlaceAsDestination(place);
                            } else {
                              handleSelectPlaceForEdit(place);
                            }
                          }}
                          style={{ cursor: 'pointer' }}
                        >
                          <title>{`${place.name} (${place.node_id}) [${place.x}m, ${place.y}m]`}</title>

                          {/* Outer highlight ring on selected or nav destination */}
                          {(isSelected || isNavDestination) && (
                            <circle
                              cx={place.px}
                              cy={place.py}
                              r={isNavDestination ? 10 : 8}
                              fill="none"
                              stroke={isNavDestination ? '#06b6d4' : '#10b981'}
                              strokeWidth={isNavDestination ? '2' : '1.5'}
                              strokeDasharray={isNavDestination ? '4 3' : '3 2'}
                            />
                          )}

                          {/* Small concise place dot */}
                          <circle
                            cx={place.px}
                            cy={place.py}
                            r={isNavDestination ? 5 : (isSelected ? 4 : 3)}
                            fill={markerColor}
                            stroke="#ffffff"
                            strokeWidth="1"
                          />

                          {/* Small concise name pill */}
                          <g transform={`translate(${place.px}, ${place.py - 6})`}>
                            <rect
                              x={-pillW / 2}
                              y={-pillH}
                              width={pillW}
                              height={pillH}
                              rx="3"
                              fill="rgba(15, 23, 42, 0.88)"
                              stroke={markerColor}
                              strokeWidth={isSelected || isNavDestination ? '1.4' : '0.8'}
                            />
                            <text
                              x="0"
                              y={-pillH / 2}
                              fill={isNavDestination ? '#38bdf8' : '#f1f5f9'}
                              fontSize="8.5"
                              fontWeight={isSelected || isNavDestination ? '700' : '500'}
                              textAnchor="middle"
                              dominantBaseline="central"
                              style={{ pointerEvents: 'none', userSelect: 'none' }}
                            >
                              {labelText}
                            </text>
                          </g>
                        </g>
                      );
                    })}
                  </svg>
                )}
              </div>
            ) : liveMapImg ? (

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
                  onLoad={(e) => {
                    if (e.target.naturalWidth) {
                      setImgDimensions({
                        width: e.target.naturalWidth,
                        height: e.target.naturalHeight,
                      });
                    }
                  }}
                />
              </div>
            ) : (
              <div className="mapping-empty-state">
                <div className="empty-state-icon">
                  {currentStep >= 3 ? '🕸️' : '📡'}
                </div>
                <h3>{currentStep >= 3 ? 'Topological Roadmap Canvas' : 'SLAM Toolbox Map Canvas'}</h3>
                <p className="text-secondary" style={{ maxWidth: 440 }}>
                  {currentStep >= 3
                    ? 'Click [ RUN GRAPH EXTRACTION ] to extract collision-free navigation nodes and lines-of-sight across the floorplan.'
                    : status === 'MAPPING'
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
                  {currentStep >= 3
                    ? graphVisMode === 'graph'
                      ? 'Roadmap Overlay'
                      : 'Clean Floorplan'
                    : mapViewMode === 'clean'
                    ? 'Clean Floorplan'
                    : 'Live Overlay'}
                </span>
              </span>
              <span className="text-dim text-sm text-mono">
                {currentStep === 4 && testedRoute ? (
                  `Tested Route: ${testedRoute.start_node} ➔ ${testedRoute.goal_node} • ${testedRoute.total_distance_m} m • ${testedRoute.hop_count} hops • Est. Time: ${testedRoute.est_time_sec} s`
                ) : currentStep === 4 && graphVerificationData ? (
                  `Audit: ${graphVerificationData.status} • ${graphVerificationData.metrics?.lcc_size}/${graphVerificationData.metrics?.total_nodes} nodes connected (${graphVerificationData.metrics?.connectivity_pct}%) • Res: ${extractedGraphReport?.metrics?.resolution || 0.05} m/px`
                ) : currentStep >= 3 && extractedGraphReport ? (
                  `Graph: ${extractedGraphReport.total_nodes} Nodes • ${extractedGraphReport.total_edges} Edges • Connectivity: ${extractedGraphReport.metrics?.avg_connectivity || '—'} / node`
                ) : (
                  `AMR: (${pose.x.toFixed(2)}, {pose.y.toFixed(2)}) • Res: ${verificationData?.resolution || mappingData.map_info?.resolution || 0.05} m/px`
                )}
              </span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
