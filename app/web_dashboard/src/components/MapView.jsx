import { useState, useEffect, useRef, useCallback } from 'react';
import bridge from '../services/amrBridge';

// ── Map constants (warehouse_map.yaml calibration) ──────────────────────────
const DEFAULT_MAP_RES  = 0.05;
const DEFAULT_ORIGIN_X = -7.397;
const DEFAULT_ORIGIN_Y = -6.596;
const MAP_WIDTH_PX     = 329;
const MAP_HEIGHT_PX    = 275;

// Breadcrumb trail settings
const TRAIL_MAX_POINTS = 120;   // ~24 s at 5 Hz
const TRAIL_MIN_DIST   = 0.05;  // metres — min movement before adding a crumb

// LERP smoothing: lower = smoother but slower response; 0.18 @ 60fps ≈ 100ms settle
const LERP_FACTOR = 0.18;

export default function MapView({ telemetry, selectedNodes = [], onSelectNode, activeMap = 'warehouse_01' }) {
  const canvasRef    = useRef(null);
  const containerRef = useRef(null);

  // ── Map state ─────────────────────────────────────────────────────────────
  const [meta, setMeta] = useState({
    resolution: DEFAULT_MAP_RES,
    origin_x:   DEFAULT_ORIGIN_X,
    origin_y:   DEFAULT_ORIGIN_Y,
    width:      MAP_WIDTH_PX,
    height:     MAP_HEIGHT_PX,
  });
  const [graphNodes, setGraphNodes] = useState([]);
  const [mapImage,   setMapImage]   = useState(null);

  // ── Viewport (pan & zoom) ─────────────────────────────────────────────────
  const [zoom, setZoom] = useState(1.8);
  const [pan,  setPan]  = useState({ x: 0, y: 0 });
  const isDraggingRef   = useRef(false);
  const dragStartRef    = useRef({ x: 0, y: 0 });
  const panStartRef     = useRef({ x: 0, y: 0 });

  // ── UI layer toggles ──────────────────────────────────────────────────────
  const [showPlaces, setShowPlaces] = useState(true);
  const [showGraph,  setShowGraph]  = useState(false);
  const [showLidar,  setShowLidar]  = useState(true);
  const [showPath,   setShowPath]   = useState(true);
  const [showTrail,  setShowTrail]  = useState(true);
  const [autoFollow, setAutoFollow] = useState(false);

  // ── Interaction state ─────────────────────────────────────────────────────
  const [namedPlaces,  setNamedPlaces]  = useState([]);
  const [hoveredPlace, setHoveredPlace] = useState(null);
  const [hoveredNode,  setHoveredNode]  = useState(null);
  const [cursorWorld,  setCursorWorld]  = useState(null);
  const [goalFeedback, setGoalFeedback] = useState(null);
  const [lastGoal,     setLastGoal]     = useState(null);

  // ── Smoothed robot pose (LERP target) ─────────────────────────────────────
  // Stored in a ref so the 60fps render loop reads the latest without re-renders
  const smoothPoseRef = useRef({ x: 0, y: 0, yaw: 0 });
  const rawPoseRef    = useRef({ x: 0, y: 0, yaw: 0 });

  // ── Breadcrumb trail ──────────────────────────────────────────────────────
  const trailRef = useRef([]); // [{ px, py }, …] in map-pixel coords

  // ── Pulsing goal animation ────────────────────────────────────────────────
  const pulseRef = useRef(0); // incremented each frame

  // ── Auto-follow pan ref (mutated by render loop, applied in state) ────────
  const autoFollowPanRef = useRef(null);

  // ─────────────────────────────────────────────────────────────────────────
  // 1. Load map image, metadata & graph on mount or when activeMap changes
  // ─────────────────────────────────────────────────────────────────────────
  useEffect(() => {
    let mounted = true;
    trailRef.current = []; // Reset trail so it doesn't carry old coordinates

    bridge.getMapMetadata(activeMap).then((data) => {
      if (data && mounted) {
        setMeta({
          resolution: data.resolution || DEFAULT_MAP_RES,
          origin_x:   data.origin_x   ?? (activeMap === 'warehouse_01' ? -6.976 : DEFAULT_ORIGIN_X),
          origin_y:   data.origin_y   ?? (activeMap === 'warehouse_01' ? -4.976 : DEFAULT_ORIGIN_Y),
          width:      data.width       || (activeMap === 'warehouse_01' ? 279 : MAP_WIDTH_PX),
          height:     data.height      || (activeMap === 'warehouse_01' ? 199 : MAP_HEIGHT_PX),
        });
      }
    });

    bridge.getGraphData(activeMap).then((data) => {
      if (mounted) {
        setGraphNodes(data?.nodes || []);
      }
    });

    bridge.getNamedPlaces(activeMap).then((res) => {
      if (mounted) {
        setNamedPlaces(res?.ok && Array.isArray(res.places) ? res.places : []);
      }
    });

    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload  = () => { if (mounted) setMapImage(img); };
    img.onerror = () => {
      bridge.getMapImage(activeMap).then((res) => {
        if (res?.image && mounted) {
          const fb = new Image();
          fb.onload = () => mounted && setMapImage(fb);
          fb.src = `data:image/jpeg;base64,${res.image}`;
        }
      });
    };
    img.src = bridge.getRawMapUrl(activeMap);

    return () => { mounted = false; };
  }, [activeMap]);

  // ─────────────────────────────────────────────────────────────────────────
  // 2. Track raw telemetry pose → trail & smooth pose ref
  // ─────────────────────────────────────────────────────────────────────────
  useEffect(() => {
    if (!telemetry?.pose) return;
    const { x, y, yaw } = telemetry.pose;

    // Initialise smooth pose on first data
    if (rawPoseRef.current.x === 0 && rawPoseRef.current.y === 0) {
      smoothPoseRef.current = { x, y, yaw };
    }
    rawPoseRef.current = { x, y, yaw };

    // Add breadcrumb if robot moved far enough
    const res = meta.resolution || DEFAULT_MAP_RES;
    const ox  = meta.origin_x   ?? DEFAULT_ORIGIN_X;
    const oy  = meta.origin_y   ?? DEFAULT_ORIGIN_Y;
    const ht  = meta.height      || MAP_HEIGHT_PX;
    const px  = (x - ox) / res;
    const py  = ht - (y - oy) / res;

    const trail = trailRef.current;
    const last  = trail[trail.length - 1];
    if (!last || Math.hypot(px - last.px, py - last.py) >= TRAIL_MIN_DIST / res) {
      trail.push({ px, py });
      if (trail.length > TRAIL_MAX_POINTS) trail.shift();
    }
  }, [telemetry, meta]);

  // ─────────────────────────────────────────────────────────────────────────
  // Coordinate helpers
  // ─────────────────────────────────────────────────────────────────────────
  const worldToPixel = useCallback(
    (wx, wy) => ({
      px: (wx - meta.origin_x) / meta.resolution,
      py: meta.height - (wy - meta.origin_y) / meta.resolution,
    }),
    [meta]
  );

  const screenToWorld = useCallback(
    (sx, sy, canvas) => {
      if (!canvas) return null;
      const rect      = canvas.getBoundingClientRect();
      const centerOffsetX = canvas.clientWidth  / 2 + pan.x;
      const centerOffsetY = canvas.clientHeight / 2 + pan.y;
      const px = (sx - rect.left  - centerOffsetX) / zoom + meta.width  / 2;
      const py = (sy - rect.top   - centerOffsetY) / zoom + meta.height / 2;
      return {
        wx: meta.origin_x + px * meta.resolution,
        wy: meta.origin_y + (meta.height - py) * meta.resolution,
        px, py,
      };
    },
    [meta, zoom, pan]
  );

  // ─────────────────────────────────────────────────────────────────────────
  // 3. 60 FPS Render Loop
  // ─────────────────────────────────────────────────────────────────────────
  useEffect(() => {
    let animId;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    const render = () => {
      const dpr = window.devicePixelRatio || 1;
      const w   = canvas.clientWidth;
      const h   = canvas.clientHeight;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width  = w * dpr;
        canvas.height = h * dpr;
      }

      ctx.save();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      // ── LERP smooth pose ────────────────────────────────────────────────
      const raw = rawPoseRef.current;
      const sp  = smoothPoseRef.current;

      sp.x   += (raw.x   - sp.x)   * LERP_FACTOR;
      sp.y   += (raw.y   - sp.y)   * LERP_FACTOR;
      // Yaw LERP with angle wrapping
      let dyaw = raw.yaw - sp.yaw;
      if (dyaw >  Math.PI) dyaw -= 2 * Math.PI;
      if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
      sp.yaw += dyaw * LERP_FACTOR;

      const { px: rx, py: ry } = worldToPixel(sp.x, sp.y);

      // ── Auto-follow: adjust pan so robot stays centred ─────────────────
      if (autoFollow) {
        // Where robot is in screen space (before pan applied)
        const targetPanX = -(rx - meta.width  / 2) * zoom;
        const targetPanY = -(ry - meta.height / 2) * zoom;
        // Smooth follow pan
        if (autoFollowPanRef.current === null) {
          autoFollowPanRef.current = { x: targetPanX, y: targetPanY };
        }
        const fp = autoFollowPanRef.current;
        fp.x += (targetPanX - fp.x) * 0.05;
        fp.y += (targetPanY - fp.y) * 0.05;
        // Apply to actual pan state only if noticeably different (avoids render storm)
        setPan(prev => {
          const dx = Math.abs(fp.x - prev.x);
          const dy = Math.abs(fp.y - prev.y);
          return (dx > 0.5 || dy > 0.5) ? { x: fp.x, y: fp.y } : prev;
        });
      } else {
        autoFollowPanRef.current = null;
      }

      // ── Apply pan & zoom ────────────────────────────────────────────────
      ctx.translate(w / 2 + pan.x, h / 2 + pan.y);
      ctx.scale(zoom, zoom);
      ctx.translate(-meta.width / 2, -meta.height / 2);

      // ── Layer 1: Base Map Image ─────────────────────────────────────────
      if (mapImage) {
        ctx.drawImage(mapImage, 0, 0, meta.width, meta.height);
      } else {
        // Charcoal grid placeholder
        ctx.fillStyle = '#0b0f19';
        ctx.fillRect(0, 0, meta.width, meta.height);
        ctx.strokeStyle = '#1a2035';
        ctx.lineWidth = 0.5;
        const gridPx = 1 / meta.resolution; // 1 m grid
        for (let gx = 0; gx < meta.width;  gx += gridPx) {
          ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, meta.height); ctx.stroke();
        }
        for (let gy = 0; gy < meta.height; gy += gridPx) {
          ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(meta.width, gy); ctx.stroke();
        }
      }

      // ── Layer 2: Topological Graph Nodes ───────────────────────────────
      if (showGraph && graphNodes.length > 0) {
        for (const n of graphNodes) {
          const isSel   = selectedNodes.includes(n.id);
          const isHover = hoveredNode?.id === n.id;
          ctx.beginPath();
          ctx.arc(n.px, n.py, isSel ? 4 : isHover ? 3.5 : 1.5, 0, Math.PI * 2);
          ctx.fillStyle = isSel ? '#00f0ff' : isHover ? '#ffaa00' : 'rgba(0,240,255,0.45)';
          ctx.fill();
          if (isSel) {
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth   = 1.2 / zoom;
            ctx.stroke();
          }
        }
      }

      // ── Layer 2.5: Named Places / Fleet Stations ────────────────────────
      if (showPlaces && namedPlaces.length > 0) {
        for (const p of namedPlaces) {
          const { px, py } = worldToPixel(p.x, p.y);
          const isHover = hoveredPlace?.id === p.id;
          const isNavTarget = Boolean(
            telemetry?.active_place_nav &&
            (telemetry.active_place_nav.target_place === p.name ||
             telemetry.active_place_nav.target_node === p.node_id)
          );
          const markerClr = isNavTarget ? '#06b6d4' : (isHover ? '#38bdf8' : '#10b981');

          ctx.save();

          // Pulsing halo ring if actively navigating to this station
          if (isNavTarget) {
            const pRing = 0.5 + 0.5 * Math.sin(pulseRef.current * 2);
            ctx.beginPath();
            ctx.arc(px, py, (7 + pRing * 6) / zoom, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(6, 182, 212, ${0.4 + pRing * 0.5})`;
            ctx.lineWidth = 2 / zoom;
            ctx.stroke();
          }

          // Station dot
          ctx.beginPath();
          ctx.arc(px, py, (isNavTarget ? 4.5 : isHover ? 4 : 3) / zoom, 0, Math.PI * 2);
          ctx.fillStyle = markerClr;
          ctx.fill();
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 1 / zoom;
          ctx.stroke();

          // Compact station pill label
          const label = p.name.length > 15 ? p.name.slice(0, 14) + '…' : p.name;
          const fontSize = Math.max(8, Math.min(10, 9 / Math.sqrt(zoom)));
          ctx.font = `${isNavTarget ? 'bold ' : ''}${fontSize}px Inter, sans-serif`;
          const textW = ctx.measureText(label).width;
          const pillW = textW + 8 / zoom;
          const pillH = 13 / zoom;
          const pillX = px - pillW / 2;
          const pillY = py - 6 / zoom - pillH;

          ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
          ctx.fillRect(pillX, pillY, pillW, pillH);
          ctx.strokeStyle = markerClr;
          ctx.lineWidth = (isNavTarget || isHover ? 1.4 : 0.8) / zoom;
          ctx.strokeRect(pillX, pillY, pillW, pillH);

          ctx.fillStyle = isNavTarget ? '#38bdf8' : '#f1f5f9';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          ctx.fillText(label, px, pillY + pillH / 2);

          ctx.restore();
        }
      }

      // ── Layer 3: MPPI Dense Path ────────────────────────────────────────
      const pathWpts = telemetry?.path || [];
      if (showPath && pathWpts.length >= 2) {
        ctx.save();
        ctx.strokeStyle = 'rgba(244,63,94,0.9)';
        ctx.lineWidth   = 2.5 / zoom;
        ctx.shadowColor = 'rgba(244,63,94,0.7)';
        ctx.shadowBlur  = 10;
        ctx.setLineDash([4 / zoom, 3 / zoom]);
        ctx.beginPath();
        for (let i = 0; i < pathWpts.length; i++) {
          const { px, py } = worldToPixel(pathWpts[i][0], pathWpts[i][1]);
          i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.restore();
      }

      // ── Layer 4: LiDAR Obstacle Points ─────────────────────────────────
      const pose     = telemetry?.pose || { x: 0, y: 0, yaw: 0 };
      const scan     = telemetry?.scan || [];
      const angMin   = telemetry?.scan_angle_min ?? -Math.PI;
      const angInc   = telemetry?.scan_angle_inc ?? 0.0174;

      if (showLidar && scan.length > 0) {
        ctx.save();
        ctx.fillStyle = 'rgba(239,68,68,0.8)';
        for (let i = 0; i < scan.length; i++) {
          const r = scan[i];
          if (r <= 0.05 || r > 8.0) continue;
          const angle = pose.yaw + (angMin + i * angInc);
          const { px: opx, py: opy } = worldToPixel(
            pose.x + r * Math.cos(angle),
            pose.y + r * Math.sin(angle)
          );
          ctx.beginPath();
          ctx.arc(opx, opy, 1.2 / zoom, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }

      // ── Layer 5: Breadcrumb Trail ───────────────────────────────────────
      const trail = trailRef.current;
      if (showTrail && trail.length >= 2) {
        ctx.save();
        for (let i = 1; i < trail.length; i++) {
          const alpha = i / trail.length;
          ctx.beginPath();
          ctx.strokeStyle = `rgba(0, 212, 255, ${alpha * 0.55})`;
          ctx.lineWidth   = (1.5 + alpha * 1.5) / zoom;
          ctx.moveTo(trail[i - 1].px, trail[i - 1].py);
          ctx.lineTo(trail[i].px,     trail[i].py);
          ctx.stroke();
        }
        // Trailing glow dot at tail
        const t0 = trail[0];
        ctx.beginPath();
        ctx.arc(t0.px, t0.py, 2 / zoom, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(0,212,255,0.3)';
        ctx.fill();
        ctx.restore();
      }

      // ── Layer 6: Pulsing Goal Destination Ring ──────────────────────────
      if (lastGoal) {
        const { px: gx, py: gy } = worldToPixel(lastGoal.x, lastGoal.y);
        pulseRef.current += 0.05;
        const pulse = 0.5 + 0.5 * Math.sin(pulseRef.current);

        ctx.save();
        // Outer pulsing ring
        ctx.beginPath();
        ctx.arc(gx, gy, (8 + pulse * 4) / zoom, 0, Math.PI * 2);
        ctx.strokeStyle = `rgba(0,255,136,${0.4 + pulse * 0.4})`;
        ctx.lineWidth   = 2 / zoom;
        ctx.shadowColor = 'rgba(0,255,136,0.6)';
        ctx.shadowBlur  = 12;
        ctx.stroke();

        // Inner solid ring
        ctx.beginPath();
        ctx.arc(gx, gy, 5 / zoom, 0, Math.PI * 2);
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth   = 1.5 / zoom;
        ctx.shadowBlur  = 6;
        ctx.stroke();

        // Crosshair
        ctx.strokeStyle = 'rgba(0,255,136,0.7)';
        ctx.lineWidth   = 1 / zoom;
        ctx.shadowBlur  = 0;
        const ch = 12 / zoom;
        ctx.beginPath();
        ctx.moveTo(gx - ch, gy); ctx.lineTo(gx + ch, gy);
        ctx.moveTo(gx, gy - ch); ctx.lineTo(gx, gy + ch);
        ctx.stroke();
        ctx.restore();
      }

      // ── Layer 7: Robot — Headlight Cone ────────────────────────────────
      const robotYaw    = sp.yaw;
      const headlightLen = 30 / zoom;  // ~1.5 m in world
      const coneAngle   = Math.PI / 5; // 36° half-angle

      ctx.save();
      const coneGrad = ctx.createRadialGradient(rx, ry, 0, rx, ry, headlightLen);
      coneGrad.addColorStop(0,   'rgba(255,255,200,0.25)');
      coneGrad.addColorStop(0.5, 'rgba(255,255,180,0.10)');
      coneGrad.addColorStop(1,   'rgba(255,255,150,0)');
      ctx.fillStyle = coneGrad;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      // ROS yaw: 0 = east (+x), positive = CCW. Canvas y is flipped.
      const coneDir = -robotYaw; // convert ROS to canvas angle
      ctx.arc(rx, ry, headlightLen, coneDir - coneAngle, coneDir + coneAngle);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      // ── Layer 8: Robot — Body, Arrow & Status Beacon ───────────────────
      const robotRadiusPx = 0.18 / meta.resolution;

      // Determine status colour from nav_state
      const navState  = telemetry?.nav_state || 'IDLE';
      const statusClr =
        navState === 'NAVIGATING' ? '#00ff9d' :
        navState === 'YIELDING'   ? '#ff8c00' :
        navState === 'ESTOP'      ? '#ff3355' :
        navState === 'ARRIVED'    ? '#ffd700' :
        navState === 'PLANNING'   ? '#3b82f6' :
        '#4a5580'; // IDLE

      ctx.save();

      // Safety clearance ring (faint)
      ctx.beginPath();
      ctx.arc(rx, ry, robotRadiusPx, 0, Math.PI * 2);
      ctx.fillStyle   = `${statusClr}18`;
      ctx.fill();
      ctx.strokeStyle = `${statusClr}60`;
      ctx.lineWidth   = 1 / zoom;
      ctx.stroke();

      // Robot body disc
      ctx.beginPath();
      ctx.arc(rx, ry, robotRadiusPx * 0.65, 0, Math.PI * 2);
      ctx.fillStyle   = 'rgba(0,212,255,0.22)';
      ctx.fill();
      ctx.strokeStyle = '#00d4ff';
      ctx.lineWidth   = 1.8 / zoom;
      ctx.shadowColor = '#00d4ff';
      ctx.shadowBlur  = 14;
      ctx.stroke();

      // Heading direction arrow
      const arrowLen = robotRadiusPx + 12 / zoom;
      const arrowEnd = {
        x: rx + arrowLen * Math.cos(-robotYaw), // canvas: CW positive
        y: ry + arrowLen * Math.sin(-robotYaw),
      };
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(arrowEnd.x, arrowEnd.y);
      ctx.strokeStyle = statusClr;
      ctx.lineWidth   = 2.5 / zoom;
      ctx.shadowColor = statusClr;
      ctx.shadowBlur  = 12;
      ctx.stroke();

      // Arrowhead
      const aw = 5 / zoom;
      const ang = Math.atan2(arrowEnd.y - ry, arrowEnd.x - rx);
      ctx.beginPath();
      ctx.moveTo(arrowEnd.x, arrowEnd.y);
      ctx.lineTo(
        arrowEnd.x - aw * Math.cos(ang - 0.5),
        arrowEnd.y - aw * Math.sin(ang - 0.5)
      );
      ctx.lineTo(
        arrowEnd.x - aw * Math.cos(ang + 0.5),
        arrowEnd.y - aw * Math.sin(ang + 0.5)
      );
      ctx.closePath();
      ctx.fillStyle = statusClr;
      ctx.fill();

      // Centre core dot
      ctx.beginPath();
      ctx.arc(rx, ry, 3 / zoom, 0, Math.PI * 2);
      ctx.fillStyle = '#ffffff';
      ctx.shadowBlur = 0;
      ctx.fill();

      // Status beacon pulsing ring (outermost)
      if (navState !== 'IDLE') {
        const bp = 0.5 + 0.5 * Math.sin(pulseRef.current * 1.8);
        ctx.beginPath();
        ctx.arc(rx, ry, robotRadiusPx + (3 + bp * 4) / zoom, 0, Math.PI * 2);
        ctx.strokeStyle = `${statusClr}${Math.round(40 + bp * 60).toString(16)}`;
        ctx.lineWidth   = 1.5 / zoom;
        ctx.stroke();
      }

      ctx.restore();
      ctx.restore(); // undo pan/zoom transform

      animId = requestAnimationFrame(render);
    };

    animId = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animId);
  }, [
    mapImage, meta, zoom, pan, telemetry,
    showPlaces, showGraph, showLidar, showPath, showTrail,
    graphNodes, selectedNodes, hoveredNode,
    namedPlaces, hoveredPlace,
    lastGoal, autoFollow, worldToPixel,
  ]);

  // ─────────────────────────────────────────────────────────────────────────
  // Pan & Zoom event handlers
  // ─────────────────────────────────────────────────────────────────────────
  const handleWheel = (e) => {
    e.preventDefault();
    setZoom(prev => Math.min(Math.max(prev * (e.deltaY < 0 ? 1.15 : 0.87), 0.5), 10));
  };

  const handleMouseDown = (e) => {
    if (e.button === 1 || e.button === 2 || e.shiftKey || e.altKey) {
      e.preventDefault();
      isDraggingRef.current = true;
      dragStartRef.current  = { x: e.clientX, y: e.clientY };
      panStartRef.current   = { ...pan };
    }
  };

  const handleMouseMove = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    if (isDraggingRef.current) {
      setPan({
        x: panStartRef.current.x + e.clientX - dragStartRef.current.x,
        y: panStartRef.current.y + e.clientY - dragStartRef.current.y,
      });
      return;
    }
    const world = screenToWorld(e.clientX, e.clientY, canvas);
    if (world) {
      setCursorWorld({ x: world.wx, y: world.wy });

      // Check hovered station / place
      if (showPlaces && namedPlaces.length > 0) {
        let nearestPlace = null, minPlaceDist = 14 / zoom;
        for (const p of namedPlaces) {
          const { px, py } = worldToPixel(p.x, p.y);
          const d = Math.hypot(px - world.px, py - world.py);
          if (d < minPlaceDist) { minPlaceDist = d; nearestPlace = p; }
        }
        setHoveredPlace(nearestPlace);
      } else {
        setHoveredPlace(null);
      }

      // Check hovered node
      if (showGraph && graphNodes.length > 0) {
        let nearest = null, minDist = 7 / zoom;
        for (const n of graphNodes) {
          const d = Math.hypot(n.px - world.px, n.py - world.py);
          if (d < minDist) { minDist = d; nearest = n; }
        }
        setHoveredNode(nearest);
      } else {
        setHoveredNode(null);
      }
    }
  };

  const handleMouseUp  = () => { isDraggingRef.current = false; };

  const handleClick = async (e) => {
    if (isDraggingRef.current || e.button !== 0) return;
    const canvas = canvasRef.current;
    const world  = screenToWorld(e.clientX, e.clientY, canvas);
    if (!world) return;

    // Prioritize clicking a station pin
    if (showPlaces && hoveredPlace) {
      onSelectNode?.(hoveredPlace.node_id);
      setGoalFeedback(`Station selected: ${hoveredPlace.name} (${hoveredPlace.node_id})`);
      setTimeout(() => setGoalFeedback(null), 2500);
      return;
    }

    if (showGraph && hoveredNode) {
      onSelectNode?.(hoveredNode.id);
      setGoalFeedback(`Added Waypoint ${hoveredNode.id}`);
      setTimeout(() => setGoalFeedback(null), 2500);
      return;
    }
    setLastGoal({ x: world.wx, y: world.wy });
    setGoalFeedback(`Dispatching goal (${world.wx.toFixed(2)}, ${world.wy.toFixed(2)})…`);
    const ok = await bridge.sendGoal(world.wx, world.wy);
    setGoalFeedback(ok ? `Goal active → (${world.wx.toFixed(2)}, ${world.wy.toFixed(2)})` : 'Goal rejected');
    setTimeout(() => setGoalFeedback(null), 3000);
  };

  const handleResetView = () => { setZoom(1.8); setPan({ x: 0, y: 0 }); };

  // ─────────────────────────────────────────────────────────────────────────
  // Render
  // ─────────────────────────────────────────────────────────────────────────
  return (
    <div
      ref={containerRef}
      className="map-viewport"
      onContextMenu={(e) => e.preventDefault()}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onWheel={handleWheel}
      onClick={handleClick}
      style={{
        position:     'relative',
        width:        '100%',
        height:       '100%',
        overflow:     'hidden',
        background:   '#080c16',
        borderRadius: 12,
        cursor:       isDraggingRef.current ? 'grabbing' : hoveredNode ? 'pointer' : 'crosshair',
        border:       '1px solid rgba(255,255,255,0.08)',
      }}
    >
      <canvas ref={canvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />

      {/* ── Top HUD bar ── */}
      <div className="map-hud-top">
        <div className="hud-badge" style={{ display: 'flex', alignItems: 'center', gap: 7 }}>
          <span className="hud-dot" />
          <span>Live Map · 60 FPS</span>
          <span
            style={{
              background: '#0284c7',
              color: '#ffffff',
              padding: '1px 7px',
              borderRadius: 4,
              fontSize: 10,
              fontWeight: 700,
              letterSpacing: '0.04em',
              textTransform: 'uppercase',
            }}
          >
            {activeMap}
          </span>
        </div>

        <div className="hud-controls">
          <button
            className={`btn-toggle ${showPlaces ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowPlaces(p => !p); }}
            title="Toggle named stations / places"
          >
            🏷️ Places {namedPlaces.length > 0 && `(${namedPlaces.length})`}
          </button>
          <button
            className={`btn-toggle ${autoFollow ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setAutoFollow(f => !f); }}
            title="Auto-follow: keep robot centred in viewport"
          >
            🎥 Follow
          </button>
          <button
            className={`btn-toggle ${showTrail ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowTrail(t => !t); }}
            title="Toggle breadcrumb motion trail"
          >
            ✦ Trail
          </button>
          <button
            className={`btn-toggle ${showGraph ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowGraph(g => !g); }}
            title="Toggle topological waypoint nodes"
          >
            🗺️ Nodes {graphNodes.length > 0 && `(${graphNodes.length})`}
          </button>
          <button
            className={`btn-toggle ${showLidar ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowLidar(l => !l); }}
            title="Toggle LiDAR obstacle hits"
          >
            🔴 LiDAR
          </button>
          <button
            className={`btn-toggle ${showPath ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowPath(p => !p); }}
            title="Toggle planned MPPI path"
          >
            〰️ Path
          </button>
        </div>
      </div>

      {/* ── Zoom controls ── */}
      <div className="map-hud-zoom">
        <button className="btn-icon" onClick={(e) => { e.stopPropagation(); setZoom(z => Math.min(z * 1.25, 10)); }}>+</button>
        <button className="btn-icon" onClick={(e) => { e.stopPropagation(); setZoom(z => Math.max(z * 0.8,  0.5)); }}>−</button>
        <button className="btn-icon btn-reset" onClick={(e) => { e.stopPropagation(); handleResetView(); }} title="Reset View">⊙</button>
      </div>

      {/* ── Bottom HUD ── */}
      <div className="map-hud-bottom">
        <div className="hud-coords">
          {hoveredPlace ? (
            <span style={{ color: '#10b981', fontWeight: 600 }}>
              🏷️ Station: {hoveredPlace.name} ({hoveredPlace.node_id}) · ({hoveredPlace.x.toFixed(2)}, {hoveredPlace.y.toFixed(2)}) m
            </span>
          ) : hoveredNode ? (
            <span style={{ color: '#00f0ff', fontWeight: 600 }}>
              📍 Node {hoveredNode.id}: ({hoveredNode.x.toFixed(2)}, {hoveredNode.y.toFixed(2)}) m
            </span>
          ) : cursorWorld ? (
            <span>Cursor: ({cursorWorld.x.toFixed(2)}, {cursorWorld.y.toFixed(2)}) m</span>
          ) : (
            <span>
              Robot: ({(telemetry?.pose?.x ?? 0).toFixed(2)}, {(telemetry?.pose?.y ?? 0).toFixed(2)}) m
              &nbsp;·&nbsp;{((telemetry?.pose?.yaw ?? 0) * 180 / Math.PI).toFixed(1)}°
            </span>
          )}
        </div>
        <div className="hud-hint">
          {showPlaces && hoveredPlace
            ? 'Click station pin to add to mission waypoints'
            : showGraph
            ? 'Click node to queue waypoint · Click floor to navigate · Shift+Drag to pan'
            : 'Click floor to navigate · Shift+Drag to pan · Scroll to zoom'}
        </div>
      </div>

      {/* ── Goal feedback toast ── */}
      {goalFeedback && (
        <div className="map-feedback-toast">{goalFeedback}</div>
      )}
    </div>
  );
}
