import { useEffect, useRef } from 'react';

/**
 * ScanRing — polar LiDAR visualization
 * Renders a 360° obstacle ring on a canvas using the /scan data.
 */
export default function ScanRing({ scan = [], angleMin = -Math.PI, angleInc = 0.0174, size = 200 }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const maxRange = 8.0; // metres — scale to this
    const scale = (W / 2 - 8) / maxRange;

    ctx.clearRect(0, 0, W, H);

    // Background grid rings
    ctx.strokeStyle = 'rgba(37,47,74,0.7)';
    ctx.lineWidth = 1;
    for (let r of [1, 2, 4, 8]) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    // Cross-hair
    ctx.strokeStyle = 'rgba(37,47,74,0.5)';
    ctx.beginPath(); ctx.moveTo(cx - W/2, cy); ctx.lineTo(cx + W/2, cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, cy - H/2); ctx.lineTo(cx, cy + H/2); ctx.stroke();

    if (!scan || scan.length === 0) {
      // No data label
      ctx.fillStyle = 'rgba(74,85,128,0.6)';
      ctx.font = '12px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('No scan data', cx, cy + 4);
      return;
    }

    // Scan polygon fill
    ctx.beginPath();
    let first = true;
    scan.forEach((r, i) => {
      const ang = angleMin + i * angleInc;
      const dist = Math.min(r > 0 ? r : maxRange, maxRange);
      const px = cx + dist * scale * Math.cos(ang);
      const py = cy - dist * scale * Math.sin(ang);
      if (first) { ctx.moveTo(px, py); first = false; }
      else        { ctx.lineTo(px, py); }
    });
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,212,255,0.08)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0,212,255,0.5)';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // Obstacle dots (close points highlighted)
    scan.forEach((r, i) => {
      if (r <= 0 || r > maxRange) return;
      const ang = angleMin + i * angleInc;
      const dist = Math.min(r, maxRange);
      const px = cx + dist * scale * Math.cos(ang);
      const py = cy - dist * scale * Math.sin(ang);
      const close = r < 1.5;
      ctx.beginPath();
      ctx.arc(px, py, close ? 2.5 : 1.2, 0, Math.PI * 2);
      ctx.fillStyle = close ? 'rgba(255,51,85,0.9)' : 'rgba(0,212,255,0.7)';
      ctx.fill();
    });

    // Robot dot (center)
    const robotR = 6;
    ctx.beginPath();
    ctx.arc(cx, cy, robotR, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,140,0,1)';
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Range labels
    ctx.fillStyle = 'rgba(74,85,128,0.8)';
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.textAlign = 'left';
    [1, 2, 4].forEach(r => {
      ctx.fillText(`${r}m`, cx + r * scale + 3, cy - 2);
    });

  }, [scan, angleMin, angleInc, size]);

  return (
    <div className="scan-ring-container">
      <canvas
        ref={canvasRef}
        width={size}
        height={size}
        style={{ width: size, height: size }}
      />
    </div>
  );
}
