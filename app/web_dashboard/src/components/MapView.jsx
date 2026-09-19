import { useState, useEffect, useRef, useCallback } from 'react';
import bridge from '../services/amrBridge';

// Calibrated Map constants matching warehouse_map.yaml & warehouse_graph.json
const DEFAULT_MAP_RES = 0.05;
const DEFAULT_ORIGIN_X = -7.397;
const DEFAULT_ORIGIN_Y = -6.596;
const MAP_WIDTH_PX = 329;
const MAP_HEIGHT_PX = 275;

export default function MapView({ telemetry, selectedNodes = [], onSelectNode }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);

  // Map metadata & graph
  const [meta, setMeta] = useState({
    resolution: DEFAULT_MAP_RES,
    origin_x: DEFAULT_ORIGIN_X,
    origin_y: DEFAULT_ORIGIN_Y,
    width: MAP_WIDTH_PX,
    height: MAP_HEIGHT_PX,
  });
  const [graphNodes, setGraphNodes] = useState([]);
  const [mapImage, setMapImage] = useState(null);

  // Viewport transforms (Pan & Zoom)
  const [zoom, setZoom] = useState(1.8);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const isDraggingRef = useRef(false);
  const dragStartRef = useRef({ x: 0, y: 0 });
  const panStartRef = useRef({ x: 0, y: 0 });

  // UI layer toggles
  const [showGraph, setShowGraph] = useState(false);
  const [showLidar, setShowLidar] = useState(true);
  const [showPath, setShowPath] = useState(true);
  const [hoveredNode, setHoveredNode] = useState(null);
  const [cursorWorld, setCursorWorld] = useState(null);
  const [goalFeedback, setGoalFeedback] = useState(null);
  const [lastGoal, setLastGoal] = useState(null);

  // ── 1. Fetch metadata, raw map image & graph nodes on mount ──
  useEffect(() => {
    let mounted = true;

    // Fetch metadata
    bridge.getMapMetadata().then((data) => {
      if (data && mounted) {
        setMeta({
          resolution: data.resolution || DEFAULT_MAP_RES,
          origin_x: data.origin_x ?? DEFAULT_ORIGIN_X,
          origin_y: data.origin_y ?? DEFAULT_ORIGIN_Y,
          width: data.width || MAP_WIDTH_PX,
          height: data.height || MAP_HEIGHT_PX,
        });
      }
    });

    // Fetch topological graph nodes
    bridge.getGraphData().then((data) => {
      if (data?.nodes && mounted) {
        setGraphNodes(data.nodes);
      }
    });

    // Load base warehouse map image
    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => {
      if (mounted) setMapImage(img);
    };
    img.onerror = () => {
      // Fallback to /api/map base64 if raw file endpoint is not directly serving
      bridge.getMapImage().then((res) => {
        if (res?.image && mounted) {
          const fallbackImg = new Image();
          fallbackImg.onload = () => mounted && setMapImage(fallbackImg);
          fallbackImg.src = `data:image/jpeg;base64,${res.image}`;
        }
      });
    };
    img.src = bridge.rawMapUrl;

    return () => {
      mounted = false;
    };
  }, []);

  // ── Coordinate Transforms ──
  // World (m) -> Map Pixel (px)
  const worldToPixel = useCallback(
    (wx, wy) => {
      const px = (wx - meta.origin_x) / meta.resolution;
      const py = meta.height - (wy - meta.origin_y) / meta.resolution;
      return { px, py };
    },
    [meta]
  );

  // Canvas Screen (px) -> World (m)
  const screenToWorld = useCallback(
    (sx, sy, canvas) => {
      if (!canvas) return null;
      const rect = canvas.getBoundingClientRect();
      const clientX = sx - rect.left;
      const clientY = sy - rect.top;

      // Unapply pan & zoom:
      // clientX = canvas.width/2 + (px - meta.width/2)*zoom + pan.x
      const centerOffsetX = canvas.clientWidth / 2 + pan.x;
      const centerOffsetY = canvas.clientHeight / 2 + pan.y;

      const px = (clientX - centerOffsetX) / zoom + meta.width / 2;
      const py = (clientY - centerOffsetY) / zoom + meta.height / 2;

      const wx = meta.origin_x + px * meta.resolution;
      const wy = meta.origin_y + (meta.height - py) * meta.resolution;
      return { wx, wy, px, py };
    },
    [meta, zoom, pan]
  );

  // ── 2. Canvas 60 FPS Render Loop ──
  useEffect(() => {
    let animId;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    const render = () => {
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }

      ctx.save();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      // Apply Pan & Zoom around center of screen
      ctx.translate(w / 2 + pan.x, h / 2 + pan.y);
      ctx.scale(zoom, zoom);
      ctx.translate(-meta.width / 2, -meta.height / 2);

      // --- Layer 1: Base Map Image ---
      if (mapImage) {
        ctx.drawImage(mapImage, 0, 0, meta.width, meta.height);
      } else {
        // Dark grid placeholder
        ctx.fillStyle = '#111827';
        ctx.fillRect(0, 0, meta.width, meta.height);
        ctx.strokeStyle = '#1f2937';
        ctx.lineWidth = 1;
        for (let x = 0; x < meta.width; x += 20) {
          ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, meta.height); ctx.stroke();
        }
        for (let y = 0; y < meta.height; y += 20) {
          ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(meta.width, y); ctx.stroke();
        }
      }

      // --- Layer 2: Topological Graph Nodes & Edges ---
      if (showGraph && graphNodes.length > 0) {
        // Draw subtle node points
        for (let i = 0; i < graphNodes.length; i++) {
          const n = graphNodes[i];
          const isSelected = selectedNodes.includes(n.id);
          const isHovered = hoveredNode?.id === n.id;

          ctx.beginPath();
          ctx.arc(n.px, n.py, isSelected ? 4 : isHovered ? 3.5 : 1.5, 0, Math.PI * 2);
          if (isSelected) {
            ctx.fillStyle = '#00f0ff';
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.2;
            ctx.fill();
            ctx.stroke();
          } else if (isHovered) {
            ctx.fillStyle = '#ffaa00';
            ctx.fill();
          } else {
            ctx.fillStyle = 'rgba(0, 240, 255, 0.45)';
            ctx.fill();
          }
        }
      }

      // --- Layer 3: Dynamic MPPI Dense Path ---
      const pathWaypoints = telemetry?.path || [];
      if (showPath && pathWaypoints.length >= 2) {
        ctx.save();
        ctx.strokeStyle = '#f43f5e';
        ctx.lineWidth = 2.5 / zoom;
        ctx.shadowColor = 'rgba(244, 63, 94, 0.8)';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        for (let i = 0; i < pathWaypoints.length; i++) {
          const { px, py } = worldToPixel(pathWaypoints[i][0], pathWaypoints[i][1]);
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.restore();
      }

      // --- Layer 4: Projected 2D LiDAR Obstacle Hits ---
      const pose = telemetry?.pose || { x: 0, y: 0, yaw: 0 };
      const scan = telemetry?.scan || [];
      const angleMin = telemetry?.scan_angle_min ?? -Math.PI;
      const angleInc = telemetry?.scan_angle_inc ?? 0.0174;

      if (showLidar && scan.length > 0) {
        ctx.save();
        ctx.fillStyle = 'rgba(239, 68, 68, 0.85)';
        for (let i = 0; i < scan.length; i++) {
          const r = scan[i];
          if (r <= 0.05 || r > 8.0) continue;
          const beamAngle = pose.yaw + (angleMin + i * angleInc);
          const obsX = pose.x + r * Math.cos(beamAngle);
          const obsY = pose.y + r * Math.sin(beamAngle);
          const { px, py } = worldToPixel(obsX, obsY);

          ctx.beginPath();
          ctx.arc(px, py, 1.2 / zoom, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }

      // --- Layer 5: Target Goal Pin ---
      if (lastGoal) {
        const { px, py } = worldToPixel(lastGoal.x, lastGoal.y);
        ctx.save();
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 2 / zoom;
        ctx.beginPath();
        ctx.arc(px, py, 6 / zoom, 0, Math.PI * 2);
        ctx.stroke();

        // Crosshair
        ctx.beginPath();
        ctx.moveTo(px - 10 / zoom, py); ctx.lineTo(px + 10 / zoom, py);
        ctx.moveTo(px, py - 10 / zoom); ctx.lineTo(px, py + 10 / zoom);
        ctx.stroke();
        ctx.restore();
      }

      // --- Layer 6: Robot Pose & Footprint ---
      const robotPixel = worldToPixel(pose.x, pose.y);
      const robotRadiusPx = 0.18 / meta.resolution; // 0.18m collision radius in px

      ctx.save();
      // Body footprint circle
      ctx.beginPath();
      ctx.arc(robotPixel.px, robotPixel.py, robotRadiusPx, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(0, 240, 255, 0.25)';
      ctx.fill();
      ctx.strokeStyle = '#00f0ff';
      ctx.lineWidth = 1.8 / zoom;
      ctx.shadowColor = '#00f0ff';
      ctx.shadowBlur = 10;
      ctx.stroke();

      // Heading indicator arrow
      const arrowLength = robotRadiusPx + 10 / zoom;
      const endX = robotPixel.px + arrowLength * Math.cos(pose.yaw);
      const endY = robotPixel.py - arrowLength * Math.sin(pose.yaw);

      ctx.beginPath();
      ctx.moveTo(robotPixel.px, robotPixel.py);
      ctx.lineTo(endX, endY);
      ctx.strokeStyle = '#ffaa00';
      ctx.lineWidth = 2.5 / zoom;
      ctx.stroke();

      // Center core dot
      ctx.beginPath();
      ctx.arc(robotPixel.px, robotPixel.py, 3 / zoom, 0, Math.PI * 2);
      ctx.fillStyle = '#ffffff';
      ctx.fill();
      ctx.restore();

      ctx.restore();
      animId = requestAnimationFrame(render);
    };

    animId = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animId);
  }, [
    mapImage,
    meta,
    zoom,
    pan,
    telemetry,
    showGraph,
    showLidar,
    showPath,
    graphNodes,
    selectedNodes,
    hoveredNode,
    lastGoal,
    worldToPixel,
  ]);

  // ── Pan & Zoom Event Handlers ──
  const handleWheel = (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    setZoom((prev) => Math.min(Math.max(prev * zoomFactor, 0.5), 8.0));
  };

  const handleMouseDown = (e) => {
    // Right-click or middle-click or Space/Shift drags the canvas
    if (e.button === 1 || e.button === 2 || e.shiftKey || e.altKey) {
      e.preventDefault();
      isDraggingRef.current = true;
      dragStartRef.current = { x: e.clientX, y: e.clientY };
      panStartRef.current = { ...pan };
    }
  };

  const handleMouseMove = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    if (isDraggingRef.current) {
      const dx = e.clientX - dragStartRef.current.x;
      const dy = e.clientY - dragStartRef.current.y;
      setPan({
        x: panStartRef.current.x + dx,
        y: panStartRef.current.y + dy,
      });
      return;
    }

    // Coordinate hover detection
    const world = screenToWorld(e.clientX, e.clientY, canvas);
    if (world) {
      setCursorWorld({ x: world.wx, y: world.wy });

      // Check if hovering near a graph node (within 6px radius)
      if (showGraph && graphNodes.length > 0) {
        let nearest = null;
        let minDist = 7 / zoom;
        for (let i = 0; i < graphNodes.length; i++) {
          const n = graphNodes[i];
          const dist = Math.hypot(n.px - world.px, n.py - world.py);
          if (dist < minDist) {
            minDist = dist;
            nearest = n;
          }
        }
        setHoveredNode(nearest);
      } else {
        setHoveredNode(null);
      }
    }
  };

  const handleMouseUp = () => {
    isDraggingRef.current = false;
  };

  const handleClick = async (e) => {
    if (isDraggingRef.current) return;
    if (e.button !== 0) return; // Only left click

    const canvas = canvasRef.current;
    if (!canvas) return;

    const world = screenToWorld(e.clientX, e.clientY, canvas);
    if (!world) return;

    // 1. If clicking on a topological node
    if (showGraph && hoveredNode) {
      if (onSelectNode) {
        onSelectNode(hoveredNode.id);
        setGoalFeedback(`Added Waypoint ${hoveredNode.id}`);
        setTimeout(() => setGoalFeedback(null), 2500);
      }
      return;
    }

    // 2. Otherwise: send 2D navigation goal
    const targetX = world.wx;
    const targetY = world.wy;
    setLastGoal({ x: targetX, y: targetY });
    setGoalFeedback(`Dispatching goal (${targetX.toFixed(2)}, ${targetY.toFixed(2)})…`);

    const ok = await bridge.sendGoal(targetX, targetY);
    setGoalFeedback(ok ? `Goal active → (${targetX.toFixed(2)}, ${targetY.toFixed(2)})` : 'Goal rejected');
    setTimeout(() => setGoalFeedback(null), 3000);
  };

  const handleResetView = () => {
    setZoom(1.8);
    setPan({ x: 0, y: 0 });
  };

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
        position: 'relative',
        width: '100%',
        height: '100%',
        overflow: 'hidden',
        background: '#090d16',
        borderRadius: 12,
        cursor: isDraggingRef.current ? 'grabbing' : hoveredNode ? 'pointer' : 'crosshair',
        border: '1px solid rgba(255, 255, 255, 0.08)',
      }}
    >
      <canvas
        ref={canvasRef}
        style={{ width: '100%', height: '100%', display: 'block' }}
      />

      {/* ── Top Floating HUD ── */}
      <div className="map-hud-top">
        <div className="hud-badge">
          <span className="hud-dot" />
          <span>Vector Map 60 FPS</span>
        </div>

        {/* Layer Toggles */}
        <div className="hud-controls">
          <button
            className={`btn-toggle ${showGraph ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowGraph(!showGraph); }}
            title="Toggle Topological Graph Nodes (Click node to add to mission)"
          >
            🗺️ Nodes {graphNodes.length > 0 && `(${graphNodes.length})`}
          </button>
          <button
            className={`btn-toggle ${showLidar ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowLidar(!showLidar); }}
            title="Toggle 2D LiDAR Obstacle Hits"
          >
            🔴 LiDAR
          </button>
          <button
            className={`btn-toggle ${showPath ? 'active' : ''}`}
            onClick={(e) => { e.stopPropagation(); setShowPath(!showPath); }}
            title="Toggle MPPI Dense Path"
          >
            〰️ Path
          </button>
        </div>
      </div>

      {/* ── Zoom Controls ── */}
      <div className="map-hud-zoom">
        <button className="btn-icon" onClick={(e) => { e.stopPropagation(); setZoom(z => Math.min(z * 1.25, 8.0)); }}>+</button>
        <button className="btn-icon" onClick={(e) => { e.stopPropagation(); setZoom(z => Math.max(z * 0.8, 0.5)); }}>−</button>
        <button className="btn-icon btn-reset" onClick={(e) => { e.stopPropagation(); handleResetView(); }} title="Reset View">⊙</button>
      </div>

      {/* ── Hover Coordinates & Node Tooltip ── */}
      <div className="map-hud-bottom">
        <div className="hud-coords">
          {hoveredNode ? (
            <span style={{ color: '#00f0ff', fontWeight: 600 }}>
              📍 Node {hoveredNode.id}: ({hoveredNode.x.toFixed(2)}, {hoveredNode.y.toFixed(2)}) m
            </span>
          ) : cursorWorld ? (
            <span>
              Cursor: ({cursorWorld.x.toFixed(2)}, {cursorWorld.y.toFixed(2)}) m
            </span>
          ) : (
            <span>
              Robot: ({(telemetry?.pose?.x ?? 0).toFixed(2)}, {(telemetry?.pose?.y ?? 0).toFixed(2)}) m
            </span>
          )}
        </div>
        <div className="hud-hint">
          {showGraph ? 'Click node to add to sequence • Click map to navigate • Drag to pan' : 'Click map to navigate • Shift+Drag to pan • Scroll to zoom'}
        </div>
      </div>

      {/* ── Goal Feedback Toast ── */}
      {goalFeedback && (
        <div className="map-feedback-toast">
          {goalFeedback}
        </div>
      )}
    </div>
  );
}
