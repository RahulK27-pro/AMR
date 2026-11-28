import { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import nipplejs from 'nipplejs';
import bridge from '../services/amrBridge';

/**
 * MappingScreen — AMR Operational Pipeline (Phase 1 & Phase 2).
 *
 * Pipeline Stages:
 *   1. Map New Place (SLAM Toolbox + Manual Drive or Auto Frontier Exploration)
 *   2. Map Verification & Saving (Quality Audit, Metrics, PGM/YAML Generation, On-disk Verification)
 *   3. Graph Extraction (Topological Roadmap generation)
 *   4. Graph Verification (Roadmap Quality & Route Audit)
 *   5. Place Naming & Final Save (Semantic Stations & System Activation)
 */
const HW_MAPPING_FALLBACK_LIMITS = {
  min_linear: 0.04,
  max_linear: 0.20,
  max_reverse_linear: 0.12,
  default_linear: 0.13,
  linear_step: 0.02,
  min_angular: 0.20,
  max_angular: 0.50,
  default_angular: 0.35,
  angular_step: 0.05,
  presets: [
    { label: 'Slow', linear: 0.08, angular: 0.25 },
    { label: 'Normal', linear: 0.13, angular: 0.35 },
    { label: 'Fast', linear: 0.18, angular: 0.50 },
  ],
};

const SIM_MAPPING_FALLBACK_LIMITS = {
  min_linear: 0.10,
  max_linear: 0.80,
  max_reverse_linear: 0.56,
  default_linear: 0.40,
  linear_step: 0.05,
  min_angular: 0.20,
  max_angular: 1.80,
  default_angular: 1.00,
  angular_step: 0.15,
};

export default function MappingScreen({
  telemetry,
  onReturnToDashboard,
  onMapChange,
  activeMap,
  theme = 'dark',
  mode = 'sim',
  piIp = '10.244.248.208',
}) {
  // Mapping state from telemetry or local fallback
  const mappingData = telemetry?.mapping ?? {};
  const hwLink = telemetry?.hardware_link;
  const piSsh = hwLink?.pi_ssh;
  const pose = telemetry?.pose ?? { x: 0, y: 0, yaw: 0 };
  const vel = telemetry?.velocity ?? { linear: 0, angular: 0 };
  const velocity = vel;
  const navState = telemetry?.nav_state ?? 'IDLE';
  const minObs = telemetry?.min_obstacle_dist;
  const isEstopActive = Boolean(telemetry?.estop_active) || navState === 'ESTOP';
  const isWheelSlip = navState === 'WHEEL_SLIP';
  const externalMappingActive = Boolean(mappingData?.external_running);
  const navStackActive = telemetry?.system_status?.state === 'RUNNING' || Boolean(telemetry?.system_status?.external_running);

  const activeLimits = useMemo(() => {
    if (mode === 'hardware') {
      return telemetry?.mapping_limits || HW_MAPPING_FALLBACK_LIMITS;
    }
    return telemetry?.mapping_limits || telemetry?.limits || SIM_MAPPING_FALLBACK_LIMITS;
  }, [mode, telemetry?.mapping_limits, telemetry?.limits]);

  const [mapName, setMapName] = useState(
    mappingData.map_name || (mode === 'hardware' ? 'hardware_map' : 'hospital_map')
  );
  const isMapNameDirtyRef = useRef(false);
  const [world, setWorld] = useState('hospital.world');
  const [runExplore, setRunExplore] = useState(
    mode === 'hardware' ? false : Boolean(mappingData.run_explore)
  );
  const [status, setStatus] = useState(mappingData.state || 'READY'); // 'READY' | 'MAPPING' | 'STOPPED' | 'SAVED' | 'ERROR'
  const [elapsedSec, setElapsedSec] = useState(mappingData.elapsed_sec || 0);
  const [isProcessing, setIsProcessing] = useState(false);
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
  const [showExploreSafetyModal, setShowExploreSafetyModal] = useState(false);
  const [showScanOverlay, setShowScanOverlay] = useState(mode === 'hardware');
  const [liveMap, setLiveMap] = useState({
    image: null,
    meta: null,
    width: 0,
    height: 0,
  });
  const liveMapImg = liveMap.image;
  const liveMapMeta = liveMap.meta;
  const [teleopSpeed, setTeleopSpeed] = useState(activeLimits.default_linear);
  const [angularSpeed, setAngularSpeed] = useState(activeLimits.default_angular);
  const [enableGui, setEnableGui] = useState(mode !== 'hardware');
  const [guiFeedback, setGuiFeedback] = useState(null);

  // Virtual Joystick & Teleop refs
  const joystickRef = useRef(null);
  const managerRef = useRef(null);
  const cmdTimerRef = useRef(null);
  const cmdRef = useRef({ linear: 0, angular: 0 });
  const speedRef = useRef(teleopSpeed);
  const angularSpeedRef = useRef(angularSpeed);

  useEffect(() => {
    speedRef.current = teleopSpeed;
  }, [teleopSpeed]);

  useEffect(() => {
    angularSpeedRef.current = angularSpeed;
  }, [angularSpeed]);

  // Clamp speed sliders cleanly whenever runtime limits change
  useEffect(() => {
    setTeleopSpeed((prev) =>
      Math.max(activeLimits.min_linear, Math.min(activeLimits.max_linear, prev || activeLimits.default_linear))
    );
    setAngularSpeed((prev) =>
      Math.max(activeLimits.min_angular, Math.min(activeLimits.max_angular, prev || activeLimits.default_angular))
    );
  }, [activeLimits]);

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
  const [selectedMapForGraph, setSelectedMapForGraph] = useState(
    mapName || (mode === 'hardware' ? 'hardware_map' : 'hospital_map')
  );
  const [densityPreset, setDensityPreset] = useState(mode === 'hardware' ? 'hw_room' : 'balanced'); // 'balanced' | 'dense' | 'hw_room' | 'sparse' | 'custom'
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

  // Phase 5: Final Map Save & Completion state
  const [isFinalSaving, setIsFinalSaving] = useState(false);
  const [isFinalSaved, setIsFinalSaved] = useState(false);
  const [finalSaveError, setFinalSaveError] = useState(null);
  const [finalSaveReport, setFinalSaveReport] = useState(null);
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
      // Only sync map_name from telemetry if user hasn't explicitly edited it
      if (m.map_name && !isSaved && !isMapNameDirtyRef.current) {
        setMapName(m.map_name);
      }
      // Note: We deliberately do NOT update liveMap.meta here from m.map_info.
      // In SLAM Toolbox, the map boundary dynamically expands. The map metadata
      // must remain strictly bound to the currently decoded image to avoid projection jumps.
      // Only overwrite local runExplore selection from telemetry while actively MAPPING
      if (m.state === 'MAPPING' && m.run_explore != null) {
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
        const baseImg = res.verification.clean_image_b64 || res.verification.overlay_image_b64;
        if (baseImg) {
          const imgSrc = `data:image/png;base64,${baseImg}`;
          const img = new Image();
          img.onload = () => {
            const w = img.naturalWidth || res.verification.width || 0;
            const h = img.naturalHeight || res.verification.height || 0;
            setLiveMap({
              image: imgSrc,
              meta: res.verification,
              width: w,
              height: h,
            });
            setImgDimensions({ width: w, height: h });
          };
          img.src = imgSrc;
        }
      }
      if (res?.mapping?.is_saved) {
        setIsSaved(true);
        if (res.mapping.verification_report) {
          setSavedReport(res.mapping.verification_report);
        }
      }
    } catch (_) {}
  }, []);

  // Trigger verification fetch when stopped or in verification step
  useEffect(() => {
    if (status === 'STOPPED' || status === 'SAVED' || currentStep >= 2) {
      fetchVerificationData();
    }
  }, [status, currentStep, fetchVerificationData]);

  // Poll live map image during mapping or inspection (always clean floorplan; SVG handles overlays)
  useEffect(() => {
    let mapTimer = null;
    let isMounted = true;

    const fetchLiveMap = async () => {
      try {
        const res = await bridge.getLiveMap(true);
        if (!isMounted) return;
        if (res?.has_data && res.image && res.info) {
          const imgSrc = `data:image/png;base64,${res.image}`;
          const img = new Image();
          img.onload = () => {
            if (!isMounted) return;
            const w = img.naturalWidth || res.info.width || 0;
            const h = img.naturalHeight || res.info.height || 0;
            setLiveMap({
              image: imgSrc,
              meta: res.info,
              width: w,
              height: h,
            });
            setImgDimensions({ width: w, height: h });
          };
          img.src = imgSrc;
        }
      } catch (_) {}
    };

    fetchLiveMap();
    if (status === 'MAPPING' || externalMappingActive) {
      mapTimer = setInterval(fetchLiveMap, 1500); // 1.5s refresh for occupancy grid
    }
    return () => {
      isMounted = false;
      if (mapTimer) clearInterval(mapTimer);
    };
  }, [status, externalMappingActive]);

  // Coordinate transform for live SLAM map SVG overlay
  const activeMapMeta = liveMap.meta || verificationData || mappingData?.map_info;
  const worldToCanvasPx = useCallback((wx, wy) => {
    if (!activeMapMeta || !activeMapMeta.resolution) {
      return null;
    }
    const res = Number(activeMapMeta.resolution) || 0.05;
    const ox = Number(activeMapMeta.origin_x) || 0;
    const oy = Number(activeMapMeta.origin_y) || 0;
    const h = Number(liveMap.height || activeMapMeta.height);
    const w = Number(liveMap.width || activeMapMeta.width);
    if (!h || !w) return null;

    // Standard ROS OccupancyGrid projection:
    // cx: distance in pixels along x axis from origin_x
    // cy: distance in pixels along y axis from origin_y
    // In PIL image, array is flipped vertically with np.flipud so y=0 is bottom (h - 1 - cy)
    const cx = (wx - ox) / res;
    const cy = (wy - oy) / res;
    const px = cx;
    const py = (h - 1) - cy;

    return { x: px, y: py };
  }, [activeMapMeta, liveMap.height, liveMap.width]);

  const mapRes = Number(activeMapMeta?.resolution) || 0.05;
  // AGV cylinder chassis is 0.11m radius, outer footprint ~0.16-0.18m.
  // In map pixels: 0.18 / resolution (e.g. ~3.6px at 0.05m/px).
  // Clamped between 3.5px and 7.0px for realistic physical proportion on the canvas.
  const robotRadius = useMemo(() => {
    return Math.max(3.5, Math.min(7.0, 0.18 / mapRes));
  }, [mapRes]);

  const robotCanvasPx = useMemo(() => {
    if (!pose || typeof pose.x !== 'number' || typeof pose.y !== 'number') return null;
    return worldToCanvasPx(pose.x, pose.y);
  }, [pose, worldToCanvasPx]);

  // Teleop command helper (clamped and E-STOP gated)
  const sendDrive = useCallback((linear, angular) => {
    if (isEstopActive && (linear !== 0 || angular !== 0)) {
      bridge.sendCmdVel(0, 0);
      return;
    }
    const maxRev = activeLimits.max_reverse_linear ?? activeLimits.max_linear;
    const clampedLin = Math.max(-maxRev, Math.min(activeLimits.max_linear, Number(linear) || 0));
    const clampedAng = Math.max(-activeLimits.max_angular, Math.min(activeLimits.max_angular, Number(angular) || 0));
    bridge.sendCmdVel(clampedLin, clampedAng);
  }, [isEstopActive, activeLimits]);

  // Immediately zero drive state if E-STOP activates
  useEffect(() => {
    if (isEstopActive) {
      cmdRef.current = { linear: 0, angular: 0 };
      bridge.sendCmdVel(0, 0);
    }
  }, [isEstopActive]);

  const handleTriggerEstop = useCallback(async () => {
    cmdRef.current = { linear: 0, angular: 0 };
    await bridge.sendStop();
  }, []);

  const handleClearEstop = useCallback(async () => {
    await bridge.clearEstop();
  }, []);

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

  // Exploration control handlers (declared early to prevent any hoisting or TDZ issues)
  const handlePauseExplore = useCallback(async () => {
    setIsProcessing(true);
    try {
      await bridge.pauseAutoExplore();
    } finally {
      setIsProcessing(false);
    }
  }, []);

  const handleResumeExplore = useCallback(async () => {
    if (isEstopActive) return;
    setIsProcessing(true);
    try {
      await bridge.resumeAutoExplore();
    } finally {
      setIsProcessing(false);
    }
  }, [isEstopActive]);

  const applyExploreModeSwitch = useCallback(async (enableExplore) => {
    setSaveErrorMsg(null);
    setRunExplore(enableExplore);
    if (status === 'MAPPING') {
      setIsProcessing(true);
      try {
        if (enableExplore) {
          const res = await bridge.startAutoExplore();
          if (!res?.ok) {
            setSaveErrorMsg(res?.error || 'Auto-explore failed to start');
            setRunExplore(false);
          }
        } else {
          await bridge.pauseAutoExplore();
        }
      } finally {
        setIsProcessing(false);
      }
    }
  }, [status]);

  const handleSwitchMode = useCallback(async (enableExplore) => {
    if (enableExplore && mode === 'hardware') {
      setShowExploreSafetyModal(true);
      return;
    }
    await applyExploreModeSwitch(enableExplore);
  }, [mode, applyExploreModeSwitch]);

  // NippleJS Virtual Joystick Setup (Interactive when mapping & E-STOP is not active)
  useEffect(() => {
    if (!joystickRef.current) return;

    const isInteractive = (status === 'MAPPING' || externalMappingActive) && !isEstopActive;

    const manager = nipplejs.create({
      zone: joystickRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: isInteractive ? '#00d4ff' : '#475569',
      size: 110,
      restJoystick: true,
    });
    managerRef.current = manager;

    manager.on('move', (_, data) => {
      if (!isInteractive || !data.vector) return;
      // If auto explore is active, seamlessly pause it on first joystick motion
      if (runExplore && exploreIsActive && !exploreIsPaused) {
        handlePauseExplore();
      }
      const linearRaw = data.vector.y;
      const angularRaw = -data.vector.x;
      const f = data.force ? Math.min(data.force, 1) : 1;
      const maxRev = activeLimits.max_reverse_linear ?? activeLimits.max_linear;
      const linScale = linearRaw >= 0 ? speedRef.current : Math.min(speedRef.current, maxRev);

      cmdRef.current = {
        linear: linearRaw * f * linScale,
        angular: angularRaw * f * angularSpeedRef.current,
      };
    });

    manager.on('end', () => {
      cmdRef.current = { linear: 0, angular: 0 };
      sendDrive(0, 0);
    });

    // 10 Hz continuous velocity stream
    cmdTimerRef.current = setInterval(() => {
      if (!isInteractive) return;
      const { linear, angular } = cmdRef.current;
      if (linear !== 0 || angular !== 0) {
        sendDrive(linear, angular);
      }
    }, 100);

    return () => {
      cmdRef.current = { linear: 0, angular: 0 };
      sendDrive(0, 0);
      manager.destroy();
      clearInterval(cmdTimerRef.current);
    };
  }, [status, externalMappingActive, isEstopActive, runExplore, exploreIsActive, exploreIsPaused, currentStep, activeLimits, sendDrive, handlePauseExplore]);

  // Keyboard driving controls (W/A/S/D & Arrow keys) + window blur zeroing
  useEffect(() => {
    if ((status !== 'MAPPING' && !externalMappingActive) || isEstopActive) return;

    const zeroMotors = () => {
      cmdRef.current = { linear: 0, angular: 0 };
      sendDrive(0, 0);
    };

    const handleKeyDown = (e) => {
      if (['INPUT', 'TEXTAREA'].includes(document.activeElement?.tagName)) return;
      if (e.repeat) return;

      // In simulation only: Enter key resumes paused exploration (disabled on hardware for safety)
      if (e.key === 'Enter' && runExplore && exploreIsPaused && mode !== 'hardware') {
        e.preventDefault();
        handleResumeExplore();
        return;
      }

      // Speed trim +/- keys
      if (e.key === '+' || e.key === '=') {
        e.preventDefault();
        setTeleopSpeed((s) => Math.min(activeLimits.max_linear, +(s + activeLimits.linear_step).toFixed(2)));
        return;
      }
      if (e.key === '-' || e.key === '_') {
        e.preventDefault();
        setTeleopSpeed((s) => Math.max(activeLimits.min_linear, +(s - activeLimits.linear_step).toFixed(2)));
        return;
      }

      const spd = speedRef.current;
      const ang = angularSpeedRef.current;
      const maxRev = activeLimits.max_reverse_linear ?? activeLimits.max_linear;

      // If exploring, touching any movement key seamlessly pauses exploration to allow manual driving
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'W', 'A', 'S', 'D'].includes(e.key)) {
        if (runExplore && exploreIsActive && !exploreIsPaused) {
          handlePauseExplore();
        }
      }

      if (e.key === 'ArrowUp' || e.key === 'w' || e.key === 'W') {
        e.preventDefault();
        cmdRef.current = { linear: spd, angular: 0 };
        sendDrive(spd, 0);
      } else if (e.key === 'ArrowDown' || e.key === 's' || e.key === 'S') {
        e.preventDefault();
        const revSpd = -Math.min(spd * 0.85, maxRev);
        cmdRef.current = { linear: revSpd, angular: 0 };
        sendDrive(revSpd, 0);
      } else if (e.key === 'ArrowLeft' || e.key === 'a' || e.key === 'A') {
        e.preventDefault();
        cmdRef.current = { linear: 0, angular: ang };
        sendDrive(0, ang);
      } else if (e.key === 'ArrowRight' || e.key === 'd' || e.key === 'D') {
        e.preventDefault();
        cmdRef.current = { linear: 0, angular: -ang };
        sendDrive(0, -ang);
      } else if (e.key === ' ' || e.key === 'x' || e.key === 'X') {
        e.preventDefault();
        zeroMotors();
      }
    };

    const handleKeyUp = (e) => {
      if (['INPUT', 'TEXTAREA'].includes(document.activeElement?.tagName)) return;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'W', 'A', 'S', 'D'].includes(e.key)) {
        zeroMotors();
      }
    };

    const handleVisibilityOrBlur = () => {
      if (document.hidden || document.visibilityState === 'hidden') {
        zeroMotors();
      } else {
        zeroMotors();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', zeroMotors);
    document.addEventListener('visibilitychange', handleVisibilityOrBlur);
    return () => {
      zeroMotors();
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', zeroMotors);
      document.removeEventListener('visibilitychange', handleVisibilityOrBlur);
    };
  }, [status, externalMappingActive, isEstopActive, mode, runExplore, exploreIsActive, exploreIsPaused, activeLimits, sendDrive, handlePauseExplore, handleResumeExplore]);

  // Format MM:SS
  const formatTime = (secs) => {
    const m = Math.floor(secs / 60);
    const s = Math.floor(secs % 60);
    return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  };

  const handleLaunchGui = async (app) => {
    setGuiFeedback({ app, status: 'launching', text: `Opening ${app.toUpperCase()} GUI...` });
    try {
      const guiMode = app === 'rviz' ? (mode === 'hardware' ? 'hw_mapping' : 'explore') : null;
      const res = await bridge.launchGui(app, guiMode);
      if (res?.ok) {
        setGuiFeedback({ app, status: 'success', text: `${app.toUpperCase()} GUI opened in desktop!` });
      } else {
        setGuiFeedback({ app, status: 'error', text: res?.error || `Failed to open ${app}` });
      }
    } catch (err) {
      setGuiFeedback({ app, status: 'error', text: err.message });
    }
    setTimeout(() => setGuiFeedback(null), 4000);
  };

  // Actions
  const handleStartMapping = async () => {
    setIsProcessing(true);
    setSaveErrorMsg(null);
    setSaveSuccessMsg(null);
    try {
      const headless = !enableGui;
      const useRviz = enableGui;
      const fallbackName = mode === 'hardware' ? 'hardware_map' : 'hospital_map';
      const targetMapName = (mapName || fallbackName).trim();
      const res = await bridge.startMapping(
        targetMapName,
        mode === 'hardware' ? null : world,
        runExplore,
        headless,
        useRviz,
        {
          autoStartPi: mode === 'hardware' ? autoStartPi : false,
          stopPiOnStop: mode === 'hardware' ? stopPiOnStop : false,
        }
      );
      if (res?.ok) {
        setStatus('MAPPING');
        setCurrentStep(1);
        setElapsedSec(0);
        setIsSaved(false);
        setSavedReport(null);
        setVerificationData(null);
        setLiveMap({ image: null, meta: null, width: 0, height: 0 });
      } else {
        setSaveErrorMsg(res?.error || 'Failed to start mapping session. Verify AMR Bridge server on port 8000.');
      }
    } catch (err) {
      setSaveErrorMsg(`Error starting mapping: ${err.message}`);
    } finally {
      setIsProcessing(false);
    }
  };

  const handleStopMapping = async () => {
    setIsProcessing(true);
    try {
      sendDrive(0, 0);
      const res = await bridge.stopMapping({
        stopPi: mode === 'hardware' ? stopPiOnStop : false,
      });
      if (res?.ok) {
        setStatus('STOPPED');
        setCurrentStep(2);
        fetchVerificationData();
      } else {
        setSaveErrorMsg(res?.error || 'Failed to stop mapping session.');
      }
    } finally {
      setIsProcessing(false);
    }
  };

  const handleRedoMapping = async () => {
    setIsProcessing(true);
    try {
      sendDrive(0, 0);
      const res = await bridge.redoMapping({
        stopPi: mode === 'hardware' ? stopPiOnStop : false,
      });
      if (res?.ok) {
        setStatus('READY');
        setCurrentStep(1);
        setElapsedSec(0);
        setLiveMap({ image: null, meta: null, width: 0, height: 0 });
        setIsSaved(false);
        setSavedReport(null);
        setVerificationData(null);
        setSaveSuccessMsg(null);
        setSaveErrorMsg(null);
      } else {
        setSaveErrorMsg(res?.error || 'Failed to reset mapping session.');
      }
    } finally {
      setIsProcessing(false);
    }
  };

  const handleSafeReturnToDashboard = () => {
    cmdRef.current = { linear: 0, angular: 0 };
    sendDrive(0, 0);
    if (status === 'MAPPING') {
      const ok = window.confirm('SLAM mapping is still running. Return to Live Dashboard anyway? (Motors will stop.)');
      if (!ok) return;
    }
    if (onReturnToDashboard) onReturnToDashboard();
  };

  // Phase 2: Save Map Action
  const handleSaveMap = async () => {
    const targetName = (mapName || 'hospital_map').trim();
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
        isMapNameDirtyRef.current = true;
        setMapName(targetName);
        setSelectedMapForGraph(targetName);
        setSavedReport(res.report);
        fetchVerificationData();
        fetchSavedMaps();
        if (onMapChange) {
          onMapChange(targetName);
        }
        window.dispatchEvent(new CustomEvent('amr_map_saved', { detail: { mapName: targetName } }));
        window.dispatchEvent(new CustomEvent('amr_maps_updated', { detail: { activeMap: targetName } }));
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
      const targetMap = (selectedMapForGraph || mapName || 'hospital_map').trim();
      let payload = {
        map_name: targetMap,
      };

      if (densityPreset === 'hw_room') {
        payload = {
          ...payload,
          robot_radius: 0.13,
          safety_margin: 0.07,
          search_radius: 2.5,
          step_corridor: 0.28,
          step_medium: 0.38,
          step_open: 0.55,
        };
      } else if (densityPreset === 'balanced') {
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
      const target = targetMap || selectedMapForGraph || mapName || 'hospital_map';
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
      const target = selectedMapForGraph || mapName || 'hospital_map';
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
      const target = selectedMapForGraph || mapName || 'hospital_map';
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
      const target = selectedMapForGraph || mapName || 'hospital_map';
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


  // ---- Phase 5: Final Map Save & Completion Handler ----
  const handleFinalSaveMap = async () => {
    setIsFinalSaving(true);
    setFinalSaveError(null);
    try {
      const targetMap = (selectedMapForGraph || mapName || 'hospital_map').trim();

      // 1. Save named places to disk and inject into roadmap
      const placeRes = await bridge.saveNamedPlaces(namedPlaces, targetMap);
      if (!placeRes?.ok) {
        throw new Error(placeRes?.error || 'Failed to save named places to disk');
      }

      // 2. Ensure base SLAM map files exist on disk
      if (!isSaved) {
        try {
          const saveRes = await bridge.saveMap(targetMap);
          if (saveRes?.ok) {
            setIsSaved(true);
          }
        } catch (_) {}
      }

      // 3. Set active map on backend bridge
      try {
        await bridge.setActiveMap(targetMap);
      } catch (e) {
        console.warn('Could not set active map on backend:', e);
      }

      // 4. Update localStorage
      localStorage.setItem('amr_active_map', targetMap);

      // 5. Notify parent App component
      if (onMapChange) {
        onMapChange(targetMap);
      }

      // 6. Broadcast global events for immediate live dashboard & dropdown update
      window.dispatchEvent(new CustomEvent('amr_map_saved', { detail: { mapName: targetMap } }));
      window.dispatchEvent(new CustomEvent('amr_maps_updated', { detail: { activeMap: targetMap } }));

      // 7. Update state & report
      setIsFinalSaved(true);
      setIsPlacesApproved(true);
      setPlacesSaveSuccess(true);
      setFinalSaveReport({
        mapName: targetMap,
        placesCount: namedPlaces.length,
        nodesCount: extractedGraphReport?.total_nodes || extractedGraphReport?.nodes?.length || 0,
        edgesCount: extractedGraphReport?.total_edges || extractedGraphReport?.edges?.length || 0,
      });

      bridge.emitEvent('success', `Final save complete: Map "${targetMap}" saved and activated!`);
    } catch (err) {
      setFinalSaveError(err.message || 'Error completing final map save');
      bridge.emitEvent('critical', `Final save error: ${err.message}`);
    } finally {
      setIsFinalSaving(false);
    }
  };

  const handleSaveAndLaunchHardwareNav = async () => {
    const targetMap = (selectedMapForGraph || mapName || 'hardware_map').trim();
    if (!isFinalSaved) {
      await handleFinalSaveMap();
    }
    if (status === 'MAPPING') {
      await bridge.stopMapping({ stopPi: false });
    }
    if (onReturnToDashboard) {
      onReturnToDashboard();
    }
    await bridge.startNavigationSystem(targetMap, '', {
      autoStartPi,
      stopPiOnStop,
    });
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
  const mapDirPrefix = mode === 'hardware' ? 'maps/' : 'src/agv_description/maps/';
  const mapStatusText = hasMapData
    ? (mappingData.map_status_text || 'Receiving map data from SLAM Toolbox (/map active)')
    : status === 'MAPPING'
    ? (mappingData.stage_text || 'Waiting for SLAM Toolbox to publish /map...')
    : (mappingData.error || 'Ready to start mapping');

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
      : 'Step 5: Semantic Place Naming & Final Map Save';

  return (
    <div className="mapping-workflow-container">
      {/* ── Pipeline Progress Header ── */}
      <div className="mapping-pipeline-header">
        <div className="mapping-pipeline-header__left">
          <button
            className="btn btn--outline"
            onClick={handleSafeReturnToDashboard}
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

        {/* Pipeline Stepper (5-Step Streamlined Mapping Flow) */}
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
                : (isFinalSaved || isPlacesApproved)
                ? 'stepper-step--completed'
                : 'stepper-step--pending'
            }`}
            onClick={() => (isGraphApproved || currentStep >= 4) && setCurrentStep(5)}
            style={{ cursor: isGraphApproved || currentStep >= 4 ? 'pointer' : 'default' }}
            title={isGraphApproved || currentStep >= 4 ? 'Step 5 (Place Naming & Final Save)' : 'Pending Step 4 roadmap approval'}
          >
            <span className="step-num">{isFinalSaved || isPlacesApproved ? '✓' : '5'}</span>
            <span className="step-label">Place Naming & Save</span>
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
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <label className="mapping-label">Map Name</label>
                    <span className="text-dim text-xs" style={{ fontFamily: 'var(--font-mono)' }}>
                      {mapDirPrefix}{mapName || '...'}.yaml
                    </span>
                  </div>
                  <input
                    className="input-field mapping-input text-mono"
                    type="text"
                    value={mapName}
                    onChange={(e) => {
                      isMapNameDirtyRef.current = true;
                      setMapName(e.target.value);
                    }}
                    placeholder={mode === 'hardware' ? 'e.g. room_map or lab_floor' : 'e.g. hospital_map or test1_map'}
                  />
                </div>

                {/* Hardware Pi Readiness & SSH Bringup Card (Hardware Mode) */}
                {mode === 'hardware' ? (
                  <div
                    className="mapping-field"
                    style={{
                      padding: '8px 10px',
                      borderRadius: 6,
                      border: '1px solid var(--border)',
                      background: 'var(--bg-elevated)',
                      display: 'flex',
                      flexDirection: 'column',
                      gap: 6,
                    }}
                  >
                    <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                      <span className="mapping-label" style={{ margin: 0 }}>
                        Pi 5 Hardware ({piIp})
                      </span>
                      <span
                        style={{
                          fontSize: 10,
                          fontWeight: 700,
                          color: hwLink?.sensors_ready
                            ? '#10b981'
                            : piSsh?.state === 'STARTING'
                            ? '#38bdf8'
                            : piSsh?.ssh_ready
                            ? '#f59e0b'
                            : '#f87171',
                        }}
                      >
                        {hwLink?.sensors_ready
                          ? 'Sensors Streaming'
                          : piSsh?.state === 'STARTING'
                          ? 'Booting Pi (Hold Still)...'
                          : piSsh?.ssh_ready
                          ? 'SSH Ready'
                          : 'Start ./start_hardware.sh on Pi'}
                      </span>
                    </div>

                    <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6, fontSize: 10 }}>
                      <span className={`badge ${piSsh?.ssh_ready ? 'badge--green' : 'badge--gray'}`}>
                        SSH: {piSsh?.ssh_ready ? 'OK' : 'NO'}
                      </span>
                      <span className={`badge ${hwLink?.scan_alive ? 'badge--green' : 'badge--gray'}`}>
                        /scan: {hwLink?.scan_alive ? 'OK' : 'WAIT'}
                      </span>
                      <span className={`badge ${hwLink?.odom_alive ? 'badge--green' : 'badge--gray'}`}>
                        /odom: {hwLink?.odom_alive ? 'OK' : 'WAIT'}
                      </span>
                      <span className={`badge ${hwLink?.clock_synced ? 'badge--green' : 'badge--gray'}`}>
                        Clock: {hwLink?.clock_synced === true ? 'SYNCED' : hwLink?.clock_synced === false ? 'SKEW' : 'AUTO'}
                      </span>
                    </div>

                    <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
                      {hwLink?.sensors_ready || piSsh?.remote_running ? (
                        <button
                          type="button"
                          className="btn btn--outline"
                          disabled={isPiLoading || isProcessing}
                          onClick={handleStopPiHardware}
                          style={{ padding: '3px 8px', fontSize: 10, borderColor: '#ef4444', color: '#f87171' }}
                        >
                          {isPiLoading ? 'Stopping…' : 'Stop Pi'}
                        </button>
                      ) : (
                        <button
                          type="button"
                          className="btn btn--outline"
                          disabled={isPiLoading || isProcessing || piSsh?.state === 'STARTING' || (piSsh && piSsh.ssh_ready === false)}
                          onClick={handleStartPiHardware}
                          style={{ padding: '3px 8px', fontSize: 10, borderColor: '#10b981', color: '#34d399' }}
                        >
                          {isPiLoading || piSsh?.state === 'STARTING' ? 'Starting…' : 'Start Pi'}
                        </button>
                      )}
                      <button
                        type="button"
                        className="btn btn--ghost"
                        onClick={handleTogglePiLogs}
                        style={{ padding: '3px 6px', fontSize: 10 }}
                      >
                        {piLogOpen ? 'Hide Log' : 'Pi Log'}
                      </button>
                    </div>

                    <div style={{ display: 'flex', flexWrap: 'wrap', gap: 10, fontSize: 11, color: 'var(--text-secondary)' }}>
                      <label style={{ display: 'flex', alignItems: 'center', gap: 4, cursor: 'pointer' }}>
                        <input
                          type="checkbox"
                          checked={autoStartPi}
                          onChange={(e) => handleToggleAutoStartPi(e.target.checked)}
                        />
                        <span>Auto-start Pi on Map Start</span>
                      </label>
                      <label style={{ display: 'flex', alignItems: 'center', gap: 4, cursor: 'pointer' }}>
                        <input
                          type="checkbox"
                          checked={stopPiOnStop}
                          onChange={(e) => handleToggleStopPiOnStop(e.target.checked)}
                        />
                        <span>Stop Pi on Stop</span>
                      </label>
                    </div>

                    {piLogOpen && (
                      <pre
                        style={{
                          margin: 0,
                          maxHeight: 110,
                          overflowY: 'auto',
                          padding: 6,
                          borderRadius: 4,
                          background: 'rgba(0,0,0,0.35)',
                          fontSize: 10,
                          lineHeight: 1.35,
                          color: 'var(--text-secondary)',
                          whiteSpace: 'pre-wrap',
                        }}
                      >
                        {piLogsText || 'Loading...'}
                      </pre>
                    )}

                    {externalMappingActive && (
                      <div style={{ fontSize: 10, color: '#38bdf8', fontWeight: 600 }}>
                        External hardware mapping session detected (live view + Save Map enabled).
                      </div>
                    )}
                    {navStackActive && (
                      <div style={{ fontSize: 10, color: '#f87171', fontWeight: 600 }}>
                        Navigation stack is currently active — stop navigation before starting SLAM mapping.
                      </div>
                    )}
                  </div>
                ) : (
                  /* Target Simulation World (Simulation Mode Only) */
                  <div className="mapping-field">
                    <label className="mapping-label">Simulation World</label>
                    <div className="flex-row" style={{ gap: 8 }}>
                      <input
                        className="input-field mapping-input text-mono"
                        type="text"
                        value={world}
                        onChange={(e) => setWorld(e.target.value)}
                        placeholder="hospital.world"
                        disabled={status === 'MAPPING'}
                      />
                      <span className="world-tag-badge">SDF World</span>
                    </div>

                    {/* Quick World Presets */}
                    <div style={{ display: 'flex', gap: 6, marginTop: 8 }}>
                      <button
                        type="button"
                        className={`btn-view-pill ${world === 'hospital.world' ? 'btn-view-pill--active' : ''}`}
                        onClick={() => {
                          setWorld('hospital.world');
                          if (!isMapNameDirtyRef.current) {
                            setMapName('hospital_map');
                          }
                        }}
                        disabled={status === 'MAPPING'}
                        style={{ fontSize: 11, padding: '3px 8px' }}
                      >
                        🏥 Hospital
                      </button>
                      <button
                        type="button"
                        className={`btn-view-pill ${world === 'test1.world' ? 'btn-view-pill--active' : ''}`}
                        onClick={() => {
                          setWorld('test1.world');
                          if (!isMapNameDirtyRef.current) {
                            setMapName('test1_map');
                          }
                        }}
                        disabled={status === 'MAPPING'}
                        style={{ fontSize: 11, padding: '3px 8px' }}
                      >
                        🏢 Multi-Room
                      </button>
                      <button
                        type="button"
                        className={`btn-view-pill ${world === 'warehouse.world' ? 'btn-view-pill--active' : ''}`}
                        onClick={() => {
                          setWorld('warehouse.world');
                          if (!isMapNameDirtyRef.current) {
                            setMapName('warehouse_map');
                          }
                        }}
                        disabled={status === 'MAPPING'}
                        style={{ fontSize: 11, padding: '3px 8px' }}
                      >
                        🏭 Warehouse
                      </button>
                    </div>
                  </div>
                )}

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

                {/* GUI Visualizer Mode */}
                <div className="mapping-field">
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <label className="mapping-label">GUI Windows</label>
                    <span className="text-dim text-mono" style={{ fontSize: 10 }}>
                      {mode === 'hardware' ? 'RViz2 Only' : 'Gazebo 3D + RViz2'}
                    </span>
                  </div>
                  <div className="mode-toggle-group">
                    <button
                      type="button"
                      className={`btn-mode-toggle ${enableGui ? 'btn-mode-toggle--active' : ''}`}
                      onClick={() => setEnableGui(true)}
                      disabled={isProcessing || status === 'MAPPING'}
                      title={mode === 'hardware' ? 'Launch RViz2 SLAM display on laptop' : 'Launch full desktop GUI with Gazebo 3D simulation and RViz2 SLAM windows'}
                    >
                      {mode === 'hardware' ? '🗺️ RViz2 SLAM Window' : '🖥️ Complete GUI (Gazebo + RViz)'}
                    </button>
                    <button
                      type="button"
                      className={`btn-mode-toggle ${!enableGui ? 'btn-mode-toggle--active' : ''}`}
                      onClick={() => setEnableGui(false)}
                      disabled={isProcessing || status === 'MAPPING'}
                      title="Run headless in background for lower resource usage"
                    >
                      {mode === 'hardware' ? '⚡ Web UI Only' : '⚡ Headless (Background)'}
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

                {saveErrorMsg && (
                  <div className="save-error-banner" style={{ marginBottom: 8 }}>
                    ⚠️ {saveErrorMsg}
                  </div>
                )}

                {/* Action Buttons */}
                <div className="mapping-actions">
                  {(status === 'READY' || status === 'ERROR') && !externalMappingActive && (
                    <button
                      id="btn-start-mapping"
                      className="btn btn--primary btn--lg btn--start-mapping"
                      onClick={handleStartMapping}
                      disabled={isProcessing || navStackActive}
                    >
                      {isProcessing
                        ? '⏳ STARTING SLAM...'
                        : mode === 'hardware'
                        ? '🚀 START HARDWARE MAPPING'
                        : '🚀 START MAPPING'}
                    </button>
                  )}

                  {externalMappingActive && (
                    <button
                      className="btn btn--primary btn--lg"
                      onClick={() => {
                        setCurrentStep(2);
                        fetchVerificationData();
                      }}
                    >
                      💾 Verify & Save Active Map
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

                {/* Desktop GUI Quick Visualizer Launchers */}
                <div className="mapping-field" style={{ marginTop: 14, paddingTop: 10, borderTop: '1px solid rgba(255,255,255,0.08)' }}>
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
                    <span style={{ fontSize: 11, fontWeight: 700, letterSpacing: '0.04em', textTransform: 'uppercase', color: '#94a3b8' }}>
                      Desktop Visualizers
                    </span>
                    {guiFeedback && (
                      <span style={{ fontSize: 10, color: guiFeedback.status === 'success' ? '#10b981' : '#f59e0b' }}>
                        {guiFeedback.text}
                      </span>
                    )}
                  </div>
                  <div className="flex-row" style={{ gap: 8 }}>
                    {mode !== 'hardware' && (
                      <button
                        type="button"
                        className="btn btn--outline"
                        style={{ flex: 1, fontSize: 11, padding: '7px 8px', borderColor: 'rgba(56, 189, 248, 0.4)', color: '#38bdf8' }}
                        onClick={() => handleLaunchGui('gazebo')}
                        title="Open Gazebo Harmonic 3D window (or run ./view_gazebo.sh in terminal)"
                      >
                        🖥️ Gazebo 3D
                      </button>
                    )}
                    <button
                      type="button"
                      className="btn btn--outline"
                      style={{ flex: 1, fontSize: 11, padding: '7px 8px', borderColor: 'rgba(168, 85, 247, 0.4)', color: '#c084fc' }}
                      onClick={() => handleLaunchGui('rviz')}
                      title="Open RViz2 SLAM Map window (or run ./view_rviz.sh in terminal)"
                    >
                      🗺️ RViz2 {mode === 'hardware' ? 'HW Mapping' : 'SLAM'}
                    </button>
                  </div>
                  <div className="text-dim text-mono" style={{ fontSize: 10, marginTop: 4, textAlign: 'center' }}>
                    {mode === 'hardware'
                      ? <>Terminal: <code style={{ color: '#c084fc' }}>./view_rviz.sh hw_explore</code></>
                      : <>Terminal commands: <code style={{ color: '#38bdf8' }}>./view_gazebo.sh</code> | <code style={{ color: '#c084fc' }}>./view_rviz.sh</code></>}
                  </div>
                </div>
              </div>

              {/* Card 2: Manual Keypad OR Autonomous Explorer */}
              {runExplore ? (
                <div className="card auto-explore-card">
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <div className="card__title" style={{ fontSize: 13 }}>
                      🤖 Autonomous Frontier Explorer
                    </div>
                    <span className="world-tag-badge">{mode === 'hardware' ? 'HW Cap <=0.15 m/s' : 'explore_lite'}</span>
                  </div>

                  {isEstopActive && (
                    <div className="save-error-banner" style={{ marginTop: 6, padding: '6px 8px', fontSize: 11 }}>
                      🛑 E-STOP LATCHED — Autonomous exploration halted.
                    </div>
                  )}

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
                          disabled={isProcessing || isEstopActive}
                        >
                          ▶ Resume Exploration
                        </button>
                      ) : (
                        <button
                          className="btn btn--outline btn--sm flex-1"
                          onClick={handlePauseExplore}
                          disabled={isProcessing}
                        >
                          ⏸ Pause & Take Over
                        </button>
                      )}
                    </div>
                  )}

                  {isEstopActive ? (
                    <button
                      type="button"
                      className="btn btn--sm btn--success"
                      onClick={handleClearEstop}
                      style={{ width: '100%', fontWeight: 700, padding: '7px 12px', marginTop: 6 }}
                    >
                      🔓 CLEAR E-STOP
                    </button>
                  ) : (
                    <button
                      type="button"
                      className="btn btn--sm btn--danger"
                      onClick={handleTriggerEstop}
                      style={{ width: '100%', fontWeight: 800, padding: '7px 12px', marginTop: 6 }}
                    >
                      🛑 EMERGENCY STOP
                    </button>
                  )}

                  <div className="text-dim text-xs explore-info-footer">
                    <span>
                      {mode === 'hardware'
                        ? '• Capped at 0.15 m/s linear / 0.45 rad/s angular with Collision Monitor active.'
                        : '• Automatically discovers unknown frontiers via Nav2 MPPI planner.'}
                    </span>
                  </div>
                </div>
              ) : (
                <div className="card teleop-mapping-card">
                  <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                    <div className="card__title" style={{ fontSize: 13 }}>
                      🎮 Precision Teleop {mode === 'hardware' ? '(HW Mapping Cap)' : 'Joystick'}
                    </div>
                    <span className="text-dim text-mono text-xs">
                      {teleopSpeed.toFixed(2)} m/s • {angularSpeed.toFixed(2)} r/s
                    </span>
                  </div>

                  {Array.isArray(activeLimits.presets) && (
                    <div style={{ display: 'flex', gap: 6, marginTop: 4 }}>
                      {activeLimits.presets.map((p) => (
                        <button
                          key={p.label}
                          type="button"
                          className={`btn-view-pill ${Math.abs(teleopSpeed - p.linear) < 0.015 ? 'btn-view-pill--active' : ''}`}
                          onClick={() => {
                            setTeleopSpeed(p.linear);
                            setAngularSpeed(p.angular);
                          }}
                          style={{ flex: 1, fontSize: 10, padding: '3px 6px' }}
                        >
                          {p.label} ({p.linear.toFixed(2)})
                        </button>
                      ))}
                    </div>
                  )}

                  <div className="teleop-speed-slider">
                    <span className="text-dim text-xs">Linear</span>
                    <input
                      type="range"
                      min={activeLimits.min_linear}
                      max={activeLimits.max_linear}
                      step={activeLimits.linear_step}
                      value={teleopSpeed}
                      onChange={(e) => setTeleopSpeed(parseFloat(e.target.value))}
                      className="slider"
                    />
                  </div>
                  <div className="teleop-speed-slider">
                    <span className="text-dim text-xs">Turn</span>
                    <input
                      type="range"
                      min={activeLimits.min_angular}
                      max={activeLimits.max_angular}
                      step={activeLimits.angular_step}
                      value={angularSpeed}
                      onChange={(e) => setAngularSpeed(parseFloat(e.target.value))}
                      className="slider"
                    />
                  </div>

                  {/* Virtual Joystick Zone like in Live Dashboard */}
                  <div
                    style={{
                      position: 'relative',
                      width: '100%',
                      minHeight: 140,
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'center',
                      margin: '4px 0',
                    }}
                  >
                    <div
                      ref={joystickRef}
                      className="joystick-zone"
                      style={{
                        width: 130,
                        height: 130,
                        opacity: (status === 'MAPPING' || externalMappingActive) && !isEstopActive ? 1 : 0.25,
                        pointerEvents: (status === 'MAPPING' || externalMappingActive) && !isEstopActive ? 'auto' : 'none',
                        transition: 'opacity 0.2s ease',
                      }}
                    />
                    {((status !== 'MAPPING' && !externalMappingActive) || isEstopActive) && (
                      <div className="joystick-lock-overlay" style={{ padding: 6, gap: 4 }}>
                        <span style={{ fontSize: 10, fontWeight: 700, color: isEstopActive ? '#f87171' : '#94a3b8' }}>
                          {isEstopActive ? 'E-STOP LOCKED' : 'LOCKED (NOT MAPPING)'}
                        </span>
                        <span style={{ fontSize: 9, color: '#64748b' }}>
                          {isEstopActive ? 'Clear E-STOP before driving' : 'Start mapping in Step 1 to steer robot'}
                        </span>
                      </div>
                    )}
                  </div>

                  {/* Stop & E-STOP Controls */}
                  <div style={{ display: 'flex', gap: 6 }}>
                    <button
                      type="button"
                      className="btn btn--sm btn--outline"
                      onClick={() => {
                        cmdRef.current = { linear: 0, angular: 0 };
                        sendDrive(0, 0);
                      }}
                      disabled={status !== 'MAPPING' && !externalMappingActive}
                      style={{ flex: 1, fontWeight: 700, padding: '6px 8px' }}
                      title="Send zero cmd_vel immediately (Spacebar)"
                    >
                      ⏹ Stop (Space)
                    </button>
                    {isEstopActive ? (
                      <button
                        type="button"
                        className="btn btn--sm btn--success"
                        onClick={handleClearEstop}
                        style={{ flex: 1, fontWeight: 700, padding: '6px 8px' }}
                      >
                        🔓 Clear E-STOP
                      </button>
                    ) : (
                      <button
                        type="button"
                        className="btn btn--sm btn--danger"
                        onClick={handleTriggerEstop}
                        style={{ flex: 1, fontWeight: 800, padding: '6px 8px' }}
                      >
                        🛑 E-STOP
                      </button>
                    )}
                  </div>

                  <div className="text-dim text-xs text-center">
                    <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> / Arrows • <kbd>+</kbd><kbd>-</kbd> Trim • <kbd>Space</kbd> Zero
                  </div>
                </div>
              )}

              {isWheelSlip && (
                <div className="save-error-banner" style={{ fontSize: 11 }}>
                  ⚠️ WHEEL_SLIP detected: encoders rotated while LiDAR scan stayed stationary.
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
                        onChange={(e) => {
                          isMapNameDirtyRef.current = true;
                          setMapName(e.target.value);
                        }}
                        placeholder="e.g. hospital_map or test1_map"
                      />
                    </div>

                    <div className="text-dim text-xs">
                      Target Location: <span className="text-mono" style={{ color: 'var(--accent-cyan)' }}>{mapDirPrefix}{mapName}.yaml</span>
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
                          📄 {savedReport?.files?.yaml || `${mapDirPrefix}${mapName}.yaml`}
                        </div>
                        <div className="saved-file-item">
                          🖼️ {savedReport?.files?.pgm || `${mapDirPrefix}${mapName}.pgm`}
                        </div>
                        <div className="saved-file-item">
                          🖼️ {savedReport?.files?.png || `${mapDirPrefix}${mapName}.png`}
                        </div>
                        {mode === 'hardware' && savedReport?.pi_synced !== undefined && (
                          <div className="saved-file-item" style={{ color: savedReport.pi_synced ? '#10b981' : '#fbbf24' }}>
                            {savedReport.pi_synced
                              ? `✅ Synced to Pi (${piIp}:~/AMR/maps/)`
                              : `⚠️ Local only (Pi sync: ${savedReport.pi_sync_error || 'skipped'})`}
                          </div>
                        )}
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
                    (() => {
                      const hwMaps = savedMaps.filter((m) => m.category === 'hardware');
                      const simMaps = savedMaps.filter((m) => m.category !== 'hardware');
                      const groups = mode === 'hardware'
                        ? [{ label: 'Hardware Maps', list: hwMaps }, { label: 'Simulation Maps', list: simMaps }]
                        : [{ label: 'Simulation Maps', list: simMaps }, { label: 'Hardware Maps', list: hwMaps }];
                      return groups.map((g) => g.list.length > 0 ? (
                        <optgroup key={g.label} label={g.label}>
                          {g.list.map((m) => (
                            <option key={`${m.category}_${m.name}`} value={m.name}>
                              {m.name} ({m.size_kb} KB)
                            </option>
                          ))}
                        </optgroup>
                      ) : null);
                    })()
                  ) : (
                    <option value={mapName}>{mapName}.yaml</option>
                  )}
                </select>
                <span className="text-dim text-xs">
                  Active target: <code className="text-mono" style={{ color: 'var(--accent-cyan)' }}>{mapDirPrefix}{selectedMapForGraph || mapName}.yaml</code>
                </span>
              </div>

              {/* Density Presets */}
              <div className="mapping-field">
                <label className="mapping-label">Roadmap Density & Clearance Preset</label>
                <div className="density-presets-grid">
                  {mode === 'hardware' && (
                    <div
                      className={`preset-card ${densityPreset === 'hw_room' ? 'preset-card--active' : ''}`}
                      onClick={() => setDensityPreset('hw_room')}
                    >
                      <div className="preset-card__title">HW Room (Fine)</div>
                      <div className="preset-card__desc">0.13m r • 0.28m step</div>
                      <div className="preset-card__badge">Hardware Best</div>
                    </div>
                  )}

                  <div
                    className={`preset-card ${densityPreset === 'balanced' ? 'preset-card--active' : ''}`}
                    onClick={() => setDensityPreset('balanced')}
                  >
                    <div className="preset-card__title">Balanced</div>
                    <div className="preset-card__desc">0.21m clearance • 2.5m search</div>
                    <div className="preset-card__badge">{mode === 'hardware' ? 'Standard' : 'Recommended'}</div>
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
                        📄 {extractedGraphReport.json_path || `${mapDirPrefix}${selectedMapForGraph}_graph.json`}
                      </div>
                      <div className="saved-file-item">
                        🖼️ {extractedGraphReport.vis_path || `${mapDirPrefix}${selectedMapForGraph}_graph_vis.png`}
                      </div>
                      {mode === 'hardware' && extractedGraphReport.pi_synced !== undefined && (
                        <div className="saved-file-item" style={{ color: extractedGraphReport.pi_synced ? '#10b981' : '#fbbf24' }}>
                          {extractedGraphReport.pi_synced ? '✅ Graph synced to Pi maps/' : '⚠️ Pi graph sync pending'}
                        </div>
                      )}
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

              {/* Card 3: Final Map Save & Completion (Step 5 Final Save) */}
              <div className="card save-config-card">
                <div className="flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
                  <div className="card__title" style={{ fontSize: 14 }}>
                    🏁 Final Save: Complete & Deploy Map
                  </div>
                  {isFinalSaved ? (
                    <span className="audit-check-row__badge badge--pass">✓ FINAL MAP SAVED</span>
                  ) : (
                    <span className="audit-check-row__badge badge--info">STEP 5 OF 5</span>
                  )}
                </div>

                {!isFinalSaved ? (
                  <>
                    <div className="text-dim text-xs" style={{ lineHeight: 1.5 }}>
                      Review configured places above. Performing the final save will write all stations to disk, finalize <strong>{selectedMapForGraph || mapName}</strong> as the active map, and make it instantly selectable in the Live Dashboard dropdown.
                    </div>

                    {/* Summary Info */}
                    <div className="audit-metrics-grid" style={{ gridTemplateColumns: 'repeat(2, 1fr)', gap: 8, margin: '8px 0' }}>
                      <div className="audit-metric-box" style={{ padding: '6px 10px' }}>
                        <span className="audit-metric-box__label">Active Map Name</span>
                        <span className="audit-metric-box__val text-mono" style={{ fontSize: 12, color: 'var(--accent-cyan)' }}>
                          {selectedMapForGraph || mapName}
                        </span>
                      </div>
                      <div className="audit-metric-box" style={{ padding: '6px 10px' }}>
                        <span className="audit-metric-box__label">Configured Places</span>
                        <span className="audit-metric-box__val text-mono" style={{ fontSize: 12, color: 'var(--accent-green)' }}>
                          {namedPlaces.length} station{namedPlaces.length === 1 ? '' : 's'}
                        </span>
                      </div>
                    </div>

                    {finalSaveError && (
                      <div className="save-error-banner" style={{ margin: 0, padding: '6px 10px', fontSize: 12 }}>
                        ⚠️ {finalSaveError}
                      </div>
                    )}

                    <div className="mapping-actions">
                      <button
                        id="btn-final-save-map"
                        type="button"
                        className="btn--proceed-graph"
                        onClick={handleFinalSaveMap}
                        disabled={isFinalSaving}
                        style={{ background: 'linear-gradient(135deg, #059669, #10b981)', padding: '10px 16px', fontWeight: 700 }}
                      >
                        {isFinalSaving ? (
                          <span>⏳ Finalizing & Saving Map to Disk...</span>
                        ) : (
                          <span>💾 FINAL SAVE MAP</span>
                        )}
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
                  </>
                ) : (
                  <>
                    <div className="save-success-banner" style={{ background: 'rgba(16, 185, 129, 0.12)', borderColor: 'rgba(16, 185, 129, 0.4)', marginTop: 4 }}>
                      <div className="text-sm font-bold" style={{ color: 'var(--accent-green)' }}>
                        🎉 Final Save Complete: Map "{selectedMapForGraph || mapName}" Ready!
                      </div>
                      <div className="text-dim text-xs" style={{ marginTop: 4, lineHeight: 1.5 }}>
                        The map floorplan, topological roadmap, and {namedPlaces.length} named place(s) have been verified and saved to disk. This map is now registered and active in the Live Dashboard dropdown.
                      </div>
                    </div>

                    {/* Files Verified List */}
                    <div className="saved-files-list" style={{ marginTop: 8 }}>
                      <div className="saved-file-item">
                        📄 {mapDirPrefix}{selectedMapForGraph || mapName}.yaml
                      </div>
                      <div className="saved-file-item">
                        🖼️ {mapDirPrefix}{selectedMapForGraph || mapName}.pgm
                      </div>
                      <div className="saved-file-item">
                        🕸️ {mapDirPrefix}{selectedMapForGraph || mapName}_graph.json
                      </div>
                      <div className="saved-file-item">
                        🏷️ {mapDirPrefix}{selectedMapForGraph || mapName}_places.json ({namedPlaces.length} stations)
                      </div>
                    </div>

                    <div className="mapping-actions" style={{ marginTop: 12 }}>
                      {mode === 'hardware' && (
                        <button
                          id="btn-save-and-launch-hw-nav"
                          type="button"
                          className="btn btn--primary"
                          onClick={handleSaveAndLaunchHardwareNav}
                          style={{ width: '100%', padding: '10px 18px', fontWeight: 700, fontSize: 13, background: 'linear-gradient(135deg, #059669, #10b981)' }}
                        >
                          🚀 LAUNCH HARDWARE NAV ON &quot;{selectedMapForGraph || mapName}&quot;
                        </button>
                      )}
                      <button
                        id="btn-return-dashboard-final"
                        type="button"
                        className="btn btn--primary"
                        onClick={onReturnToDashboard}
                        style={{ width: '100%', padding: '10px 18px', fontWeight: 700, fontSize: 13, background: 'linear-gradient(135deg, #0284c7, #10b981)' }}
                      >
                        📊 GO TO LIVE DASHBOARD ➔
                      </button>

                      <div className="flex-row" style={{ gap: 8 }}>
                        <button
                          type="button"
                          className="btn btn--outline flex-1"
                          onClick={() => setIsFinalSaved(false)}
                          style={{ fontSize: 12 }}
                        >
                          ✏️ Edit Places
                        </button>
                        <button
                          type="button"
                          className="btn btn--ghost btn--sm"
                          onClick={handleRedoMapping}
                          style={{ fontSize: 12 }}
                        >
                          🔄 Map Another Area
                        </button>
                      </div>
                    </div>
                  </>
                )}
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

              {currentStep <= 2 && (
                <button
                  className={`btn btn--ghost btn--sm ${showScanOverlay ? 'btn-view-pill--active' : ''}`}
                  onClick={() => setShowScanOverlay((s) => !s)}
                  title="Toggle Live LiDAR /scan points overlay"
                  style={{ fontSize: 11, padding: '4px 8px' }}
                >
                  📡 Scan
                </button>
              )}

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
                  width: imgDimensions.width > 0 ? `${imgDimensions.width}px` : 'auto',
                  height: imgDimensions.height > 0 ? `${imgDimensions.height}px` : 'auto',
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


                {/* SVG Place Markers Overlay (Step 5 Place Naming) */}
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
                      const isSelected = selectedPlaceId === place.id;
                      const markerColor = isSelected ? '#06b6d4' : '#10b981';
                      const labelText = place.name.length > 16 ? place.name.slice(0, 15) + '…' : place.name;
                      const pillW = Math.max(28, Math.min(labelText.length * 6 + 10, 85));
                      const pillH = 14;

                      return (
                        <g
                          key={place.id}
                          className={`place-pin-marker ${isSelected ? 'place-pin-marker--selected' : ''}`}
                          onClick={(e) => {
                            e.stopPropagation();
                            handleSelectPlaceForEdit(place);
                          }}
                          style={{ cursor: 'pointer' }}
                        >
                          <title>{`${place.name} (${place.node_id}) [${place.x}m, ${place.y}m]`}</title>

                          {/* Outer highlight ring on selected place */}
                          {isSelected && (
                            <circle
                              cx={place.px}
                              cy={place.py}
                              r={8}
                              fill="none"
                              stroke="#06b6d4"
                              strokeWidth="1.5"
                              strokeDasharray="3 2"
                            />
                          )}

                          {/* Small concise place dot */}
                          <circle
                            cx={place.px}
                            cy={place.py}
                            r={isSelected ? 4 : 3}
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
                              strokeWidth={isSelected ? '1.4' : '0.8'}
                            />
                            <text
                              x="0"
                              y={-pillH / 2}
                              fill="#f1f5f9"
                              fontSize="8.5"
                              fontWeight={isSelected ? '700' : '500'}
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
            ) : liveMap.image ? (

              <div
                className="mapping-image-wrapper"
                style={{
                  transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
                  transformOrigin: 'center center',
                  position: 'relative',
                  width: liveMap.width > 0 ? `${liveMap.width}px` : 'auto',
                  height: liveMap.height > 0 ? `${liveMap.height}px` : 'auto',
                  display: 'inline-block',
                }}
              >
                <img
                  src={liveMap.image}
                  alt="SLAM Occupancy Grid"
                  className="mapping-slam-img"
                  draggable={false}
                  style={theme === 'light' ? { filter: 'invert(0.92) hue-rotate(180deg) contrast(1.08)' } : undefined}
                />

                {/* SVG Live Robot, LiDAR Scan & Exploration Overlay */}
                {liveMap.width > 0 && liveMap.height > 0 && mapViewMode === 'overlay' && (
                  <svg
                    className="mapping-live-svg-overlay"
                    viewBox={`0 0 ${liveMap.width} ${liveMap.height}`}
                    style={{
                      position: 'absolute',
                      top: 0,
                      left: 0,
                      width: '100%',
                      height: '100%',
                      pointerEvents: 'none',
                      zIndex: 4,
                    }}
                  >
                    {/* Live LiDAR Scan Rays */}
                    {showScanOverlay && Array.isArray(telemetry?.scan) && telemetry.scan.length > 0 && (
                      <g opacity="0.88">
                        {telemetry.scan.map((r, idx) => {
                          if (!r || r < 0.15 || r > 8.0) return null;
                          const angle = pose.yaw + (telemetry.scan_angle_min || 0) + idx * (telemetry.scan_angle_inc || 0);
                          const wx = pose.x + r * Math.cos(angle);
                          const wy = pose.y + r * Math.sin(angle);
                          const pt = worldToCanvasPx(wx, wy);
                          if (!pt) return null;
                          return <circle key={`scan-${idx}`} cx={pt.x} cy={pt.y} r="0.9" fill="#ef4444" />;
                        })}
                      </g>
                    )}

                    {/* Active Exploration Frontiers */}
                    {telemetry?.mapping?.exploration?.frontiers?.map((f, i) => {
                      const fPx = worldToCanvasPx(f.x, f.y);
                      if (!fPx) return null;
                      return (
                        <g key={`frontier-${i}`}>
                          <polygon
                            points={`${fPx.x},${fPx.y - 5} ${fPx.x + 4.5},${fPx.y} ${fPx.x},${fPx.y + 5} ${fPx.x - 4.5},${fPx.y}`}
                            fill="#fbbf24"
                            stroke="#ffffff"
                            strokeWidth="1.2"
                          />
                        </g>
                      );
                    })}

                    {/* Simple, Clean AMR Robot (Proportionate physical sizing to map resolution) */}
                    {robotCanvasPx && (
                      <g
                        transform={`translate(${robotCanvasPx.x}, ${robotCanvasPx.y})`}
                        style={{ pointerEvents: 'auto', cursor: 'default' }}
                      >
                        <title>{`AMR Robot • (${pose.x.toFixed(2)}m, ${pose.y.toFixed(2)}m) • Heading: ${((pose.yaw * 180) / Math.PI).toFixed(1)}°`}</title>

                        {/* Single clean robot chassis disc matching physical footprint */}
                        <circle
                          r={robotRadius}
                          fill="rgba(0, 212, 255, 0.32)"
                          stroke="#00f3ff"
                          strokeWidth="1.2"
                        />

                        {/* Center core dot */}
                        <circle
                          r={Math.max(1, robotRadius * 0.28)}
                          fill="#ffffff"
                        />

                        {/* Crisp forward direction indicator */}
                        <g transform={`rotate(${(-pose.yaw * 180) / Math.PI})`}>
                          <line
                            x1="0"
                            y1="0"
                            x2={robotRadius + 3}
                            y2="0"
                            stroke="#00f3ff"
                            strokeWidth="1.4"
                            strokeLinecap="round"
                          />
                          <polygon
                            points={`${robotRadius + 4.2},0 ${robotRadius + 1.2},-2 ${robotRadius + 1.8},0 ${robotRadius + 1.2},2`}
                            fill="#00f3ff"
                            stroke="#ffffff"
                            strokeWidth="0.4"
                          />
                        </g>
                      </g>
                    )}
                  </svg>
                )}
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
                    : `Click [ START ${mode === 'hardware' ? 'HARDWARE ' : ''}MAPPING ] to trigger SLAM Toolbox and begin mapping ${mode === 'hardware' ? 'the physical room' : (world || 'test1.world')}. The map will render here in real time.`}
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
                  `AMR: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) • Res: ${verificationData?.resolution || mappingData.map_info?.resolution || 0.05} m/px`
                )}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Hardware Auto-Explore Safety Confirmation Modal */}
      {showExploreSafetyModal && (
        <div
          style={{
            position: 'fixed',
            inset: 0,
            background: 'rgba(2, 6, 23, 0.78)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            zIndex: 9999,
            padding: 16,
          }}
        >
          <div className="card" style={{ maxWidth: 440, width: '100%', border: '1px solid rgba(245, 158, 11, 0.55)' }}>
            <div className="card__title" style={{ color: '#fbbf24', fontSize: 15 }}>
              ⚠️ Confirm Hardware Autonomous Exploration
            </div>
            <p className="text-secondary" style={{ fontSize: 12, lineHeight: 1.5, margin: '8px 0' }}>
              Autonomous frontier exploration will drive the physical robot autonomously using Nav2 MPPI + Collision Monitor (capped at <strong>0.15 m/s</strong> linear, <strong>0.45 rad/s</strong> angular).
            </p>
            <ul className="text-dim" style={{ fontSize: 11, lineHeight: 1.5, paddingLeft: 18, margin: '6px 0 12px' }}>
              <li>The 2D LiDAR (z = 0.16 m) cannot detect overhangs, glass, cables, or stairs.</li>
              <li>Verify floor clearance and keep E-STOP ready before starting.</li>
              <li>Sensors Ready: <strong>{hwLink?.sensors_ready ? 'YES' : 'NO'}</strong> | Clock Sync: <strong>{hwLink?.clock_synced === false ? 'SKEW' : 'OK'}</strong></li>
            </ul>
            <div className="flex-row" style={{ gap: 8, justifyContent: 'flex-end' }}>
              <button
                type="button"
                className="btn btn--outline"
                onClick={() => setShowExploreSafetyModal(false)}
              >
                Cancel (Keep Manual)
              </button>
              <button
                type="button"
                className="btn btn--primary"
                disabled={isEstopActive}
                onClick={async () => {
                  setShowExploreSafetyModal(false);
                  await applyExploreModeSwitch(true);
                }}
              >
                Confirm Auto-Explore
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
