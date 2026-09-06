import { useState, useEffect, useRef, useCallback } from 'react';
import bridge from '../services/amrBridge';

/**
 * MapView — center panel
 * Renders the static graph_visualization.png from /api/map (base64 JPEG with robot overlay)
 * and allows click-to-goal interaction.
 *
 * The bridge server already renders the robot dot + path overlay on the server side
 * and returns a JPEG via /api/map. We re-fetch every 2 seconds to keep the overlay fresh.
 *
 * Click-to-goal: on left-click we compute map coordinates from the click position
 * using the known MAP_RESOLUTION and MAP_ORIGIN constants (must match bridge_server.py).
 */

// These MUST match bridge_server.py MAP_* constants
const MAP_RESOLUTION = 0.05;   // m/px
const MAP_ORIGIN_X   = -8.3;
const MAP_ORIGIN_Y   = -7.3;
const MAP_HEIGHT_PX  = 275;

function pixelToWorld(imgEl, clickX, clickY) {
  if (!imgEl) return null;
  const rect    = imgEl.getBoundingClientRect();
  const natW    = imgEl.naturalWidth  || imgEl.width;
  const natH    = imgEl.naturalHeight || imgEl.height;
  const scaleX  = natW / rect.width;
  const scaleY  = natH / rect.height;
  const px = (clickX - rect.left) * scaleX;
  const py = (clickY - rect.top)  * scaleY;
  const wx = px * MAP_RESOLUTION + MAP_ORIGIN_X;
  const wy = (MAP_HEIGHT_PX - py) * MAP_RESOLUTION + MAP_ORIGIN_Y;
  return { x: wx, y: wy };
}

export default function MapView({ telemetry }) {
  const [mapSrc, setMapSrc]     = useState(null);
  const [hoverPos, setHoverPos] = useState(null);
  const [lastGoal, setLastGoal] = useState(null);
  const [goalFeedback, setGoalFeedback] = useState(null);
  const imgRef  = useRef(null);
  const timerRef = useRef(null);

  const fetchMap = useCallback(async () => {
    const data = await bridge.getMapImage();
    if (data?.image) {
      setMapSrc(`data:image/jpeg;base64,${data.image}`);
    }
  }, []);

  useEffect(() => {
    fetchMap();
    timerRef.current = setInterval(fetchMap, 2000);
    return () => clearInterval(timerRef.current);
  }, [fetchMap]);

  const handleClick = async (e) => {
    if (e.button !== 0) return;
    const worldPos = pixelToWorld(imgRef.current, e.clientX, e.clientY);
    if (!worldPos) return;
    const { x, y } = worldPos;
    setLastGoal({ x, y });
    setGoalFeedback('Sending goal…');
    const ok = await bridge.sendGoal(x, y);
    setGoalFeedback(ok ? `Goal sent: (${x.toFixed(2)}, ${y.toFixed(2)})` : 'Failed to send goal');
    setTimeout(() => setGoalFeedback(null), 3000);
  };

  const handleMouseMove = (e) => {
    const worldPos = pixelToWorld(imgRef.current, e.clientX, e.clientY);
    if (worldPos) setHoverPos(worldPos);
  };

  const handleMouseLeave = () => setHoverPos(null);

  return (
    <div
      className="map-container"
      onClick={handleClick}
      onMouseMove={handleMouseMove}
      onMouseLeave={handleMouseLeave}
      title="Click to send navigation goal"
    >
      {mapSrc ? (
        <img
          ref={imgRef}
          src={mapSrc}
          alt="Warehouse map with robot overlay"
          style={{ width: '100%', height: '100%', objectFit: 'contain', display: 'block' }}
          draggable={false}
        />
      ) : (
        <div style={{
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          height: '100%', color: 'var(--text-dim)', flexDirection: 'column', gap: 12
        }}>
          <div style={{ fontSize: 40, opacity: 0.3 }}>🗺</div>
          <div style={{ fontFamily: 'var(--text-mono)', fontSize: 13 }}>
            Waiting for map…
          </div>
          <div style={{ fontSize: 11, color: 'var(--text-dim)' }}>
            Start bridge_server.py + ROS 2 navigation
          </div>
        </div>
      )}

      {/* Overlay label */}
      <div className="map-overlay-label">
        🗺 Warehouse — click to navigate
      </div>

      {/* Goal sent feedback */}
      {goalFeedback && (
        <div style={{
          position: 'absolute', top: 12, right: 12,
          background: 'rgba(0,212,255,0.15)',
          border: '1px solid rgba(0,212,255,0.4)',
          color: 'var(--accent-cyan)',
          padding: '6px 12px', borderRadius: 6,
          fontFamily: 'var(--text-mono)', fontSize: 12,
          animation: 'slideIn 0.2s ease',
        }}>
          {goalFeedback}
        </div>
      )}

      {/* Last goal marker info */}
      {lastGoal && !goalFeedback && (
        <div style={{
          position: 'absolute', top: 12, right: 12,
          background: 'rgba(10,13,20,0.7)',
          border: '1px solid var(--border)',
          color: 'var(--accent-yellow)',
          padding: '4px 10px', borderRadius: 6,
          fontFamily: 'var(--text-mono)', fontSize: 11,
        }}>
          Goal: ({lastGoal.x.toFixed(2)}, {lastGoal.y.toFixed(2)})
        </div>
      )}

      {/* Hover world coordinates */}
      <div className="map-coords-display">
        {hoverPos
          ? `(${hoverPos.x.toFixed(2)}, ${hoverPos.y.toFixed(2)}) m`
          : `Robot: (${(telemetry?.pose?.x ?? 0).toFixed(2)}, ${(telemetry?.pose?.y ?? 0).toFixed(2)}) m`
        }
      </div>
    </div>
  );
}
