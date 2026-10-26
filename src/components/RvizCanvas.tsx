import React, { useEffect, useRef, useState } from 'react';
import {
  CellState,
  DeadEndRecord,
  DWARolloutArc,
  DynamicObstacle,
  FSMState,
  FrontierCluster,
  GridCoord,
  LayerVisibility,
  LidarRay,
  MapBrushTool,
  SectorClearance,
} from '../types/slam';

interface RvizCanvasProps {
  grid: CellState[][];
  robotPose: { x: number; y: number; theta: number };
  homeDock: GridCoord;
  lidarRays: LidarRay[];
  frontiers: FrontierCluster[];
  selectedFrontier: FrontierCluster | null;
  currentPath: GridCoord[];
  breadcrumbTrail: GridCoord[];
  deadEndRecords: DeadEndRecord[];
  dynamicObstacles: DynamicObstacle[];
  dwaRollouts: DWARolloutArc[];
  sectorClearance: SectorClearance;
  highlightedDeadEndId: number | null;
  fsmState: FSMState;
  layers: LayerVisibility;
  activeBrush: MapBrushTool;
  onCellInteract: (x: number, y: number, isDrag: boolean) => void;
}

export const RvizCanvas: React.FC<RvizCanvasProps> = ({
  grid,
  robotPose,
  homeDock,
  lidarRays,
  frontiers,
  selectedFrontier,
  currentPath,
  breadcrumbTrail,
  deadEndRecords,
  dynamicObstacles,
  dwaRollouts,
  sectorClearance,
  highlightedDeadEndId,
  fsmState,
  layers,
  activeBrush,
  onCellInteract,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [hoveredCell, setHoveredCell] = useState<CellState | null>(null);
  const [isMouseDown, setIsMouseDown] = useState(false);

  const rows = grid.length;
  const cols = grid[0]?.length || 44;
  const CELL_SIZE = 22;
  const canvasWidth = cols * CELL_SIZE;
  const canvasHeight = rows * CELL_SIZE;

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    if (
      canvas.width !== canvasWidth * dpr ||
      canvas.height !== canvasHeight * dpr
    ) {
      canvas.width = canvasWidth * dpr;
      canvas.height = canvasHeight * dpr;
    }
    ctx.save();
    ctx.scale(dpr, dpr);

    // 1. Clear background (Unknown Space Base)
    ctx.fillStyle = '#090D16';
    ctx.fillRect(0, 0, canvasWidth, canvasHeight);

    // 2. Render Grid Cells (Belief State + Costmap Inflation + Taboo Hatch)
    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        const cell = grid[y][x];
        const px = x * CELL_SIZE;
        const py = y * CELL_SIZE;

        if (cell.belief === 'UNKNOWN') {
          ctx.fillStyle = '#0B0F1A';
          ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);

          if (layers.showGroundTruthGhost && cell.groundTruth === 'WALL') {
            ctx.fillStyle = 'rgba(51, 65, 85, 0.28)';
            ctx.fillRect(px + 2, py + 2, CELL_SIZE - 4, CELL_SIZE - 4);
            ctx.strokeStyle = 'rgba(100, 116, 139, 0.35)';
            ctx.lineWidth = 1;
            ctx.strokeRect(px + 2.5, py + 2.5, CELL_SIZE - 5, CELL_SIZE - 5);
          }
        } else if (cell.belief === 'FREE') {
          if (layers.showDeadEndTaboo && cell.isDeadEndTaboo) {
            const isHighlighted =
              highlightedDeadEndId !== null &&
              cell.deadEndClusterId === highlightedDeadEndId;

            ctx.fillStyle = isHighlighted
              ? 'rgba(244, 63, 94, 0.42)'
              : 'rgba(244, 63, 94, 0.20)';
            ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);

            ctx.strokeStyle = isHighlighted
              ? 'rgba(251, 113, 133, 0.85)'
              : 'rgba(244, 63, 94, 0.45)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(px, py + CELL_SIZE);
            ctx.lineTo(px + CELL_SIZE, py);
            ctx.moveTo(px, py + CELL_SIZE * 0.5);
            ctx.lineTo(px + CELL_SIZE * 0.5, py);
            ctx.moveTo(px + CELL_SIZE * 0.5, py + CELL_SIZE);
            ctx.lineTo(px + CELL_SIZE, py + CELL_SIZE * 0.5);
            ctx.stroke();
          } else {
            ctx.fillStyle = '#162235';
            ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);

            // Costmap Safety Inflation Gradient Overlay
            if (layers.showCostmapInflation && cell.inflationCost > 0.05) {
              const alpha = Math.min(0.45, cell.inflationCost * 0.52);
              ctx.fillStyle = `rgba(99, 102, 241, ${alpha.toFixed(3)})`;
              ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);
            }
          }
        } else if (cell.belief === 'OCCUPIED') {
          ctx.fillStyle = '#CBD5E1';
          ctx.fillRect(px + 0.5, py + 0.5, CELL_SIZE - 1, CELL_SIZE - 1);
          ctx.fillStyle = '#94A3B8';
          ctx.fillRect(px + 2, py + 2, CELL_SIZE - 4, CELL_SIZE - 4);
        }

        // 3. Frontier Cells Overlay
        if (layers.showFrontiers && cell.isFrontier && !cell.isDeadEndTaboo) {
          ctx.fillStyle = 'rgba(245, 158, 11, 0.45)';
          ctx.fillRect(px + 1, py + 1, CELL_SIZE - 2, CELL_SIZE - 2);
          ctx.strokeStyle = '#F59E0B';
          ctx.lineWidth = 1;
          ctx.strokeRect(px + 1.5, py + 1.5, CELL_SIZE - 3, CELL_SIZE - 3);
        }

        // 4. Coordinate Grid Hairlines
        if (layers.showGridLines) {
          ctx.strokeStyle =
            cell.belief === 'UNKNOWN'
              ? 'rgba(30, 41, 59, 0.45)'
              : 'rgba(51, 65, 85, 0.45)';
          ctx.lineWidth = 0.5;
          ctx.strokeRect(px, py, CELL_SIZE, CELL_SIZE);
        }
      }
    }

    // 5. Render Dead-End Throat Seals & Cluster ID Labels
    if (layers.showDeadEndTaboo) {
      for (const rec of deadEndRecords) {
        const isHighlighted = highlightedDeadEndId === rec.id;
        ctx.save();
        ctx.strokeStyle = isHighlighted ? '#FDA4AF' : '#F43F5E';
        ctx.lineWidth = isHighlighted ? 2.5 : 1.75;
        ctx.strokeRect(
          rec.throatCoord.x * CELL_SIZE + 1,
          rec.throatCoord.y * CELL_SIZE + 1,
          CELL_SIZE - 2,
          CELL_SIZE - 2
        );

        const avgX =
          rec.cells.reduce((s, c) => s + c.x, 0) / Math.max(1, rec.cells.length);
        const avgY =
          rec.cells.reduce((s, c) => s + c.y, 0) / Math.max(1, rec.cells.length);
        const cx = (avgX + 0.5) * CELL_SIZE;
        const cy = (avgY + 0.5) * CELL_SIZE;

        const tag = `DE-${String(rec.id).padStart(2, '0')}`;
        ctx.font = '600 9px "JetBrains Mono", monospace';
        const textWidth = ctx.measureText(tag).width;
        ctx.fillStyle = isHighlighted ? '#E11D48' : 'rgba(15, 23, 42, 0.88)';
        ctx.fillRect(cx - textWidth / 2 - 4, cy - 7, textWidth + 8, 14);
        ctx.strokeStyle = '#F43F5E';
        ctx.lineWidth = 1;
        ctx.strokeRect(cx - textWidth / 2 - 4, cy - 7, textWidth + 8, 14);

        ctx.fillStyle = isHighlighted ? '#FFFFFF' : '#FDA4AF';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(tag, cx, cy);
        ctx.restore();
      }
    }

    // 6. Render 360° LiDAR Raycast Fan
    if (layers.showLidarRays && lidarRays.length > 0) {
      const rx = (robotPose.x + 0.5) * CELL_SIZE;
      const ry = (robotPose.y + 0.5) * CELL_SIZE;

      ctx.save();
      for (const ray of lidarRays) {
        const ex = (ray.endX + 0.5) * CELL_SIZE;
        const ey = (ray.endY + 0.5) * CELL_SIZE;

        ctx.strokeStyle = ray.hitDynamicObstacle
          ? 'rgba(251, 146, 60, 0.45)'
          : ray.hitOccupied
          ? 'rgba(6, 182, 212, 0.22)'
          : 'rgba(6, 182, 212, 0.10)';
        ctx.lineWidth = ray.hitDynamicObstacle ? 1.5 : 1;
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(ex, ey);
        ctx.stroke();

        if (ray.hitOccupied) {
          ctx.fillStyle = ray.hitDynamicObstacle ? '#FB923C' : '#22D3EE';
          ctx.beginPath();
          ctx.arc(ex, ey, ray.hitDynamicObstacle ? 2.6 : 1.8, 0, Math.PI * 2);
          ctx.fill();
        }
      }
      ctx.restore();
    }

    // 7. Render Historical Breadcrumb Trail
    if (layers.showBreadcrumbTrail && breadcrumbTrail.length > 1) {
      ctx.save();
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      for (let i = 0; i < breadcrumbTrail.length; i++) {
        const pt = breadcrumbTrail[i];
        const px = (pt.x + 0.5) * CELL_SIZE;
        const py = (pt.y + 0.5) * CELL_SIZE;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    }

    // 8. Render Active Global A* Planned Trajectory
    if (layers.showPlannedPath && currentPath.length > 1) {
      const isReturnHome = fsmState === 'RETURNING_HOME';
      ctx.save();
      ctx.strokeStyle = isReturnHome ? '#10B981' : '#06B6D4';
      ctx.lineWidth = 2.25;
      ctx.beginPath();
      for (let i = 0; i < currentPath.length; i++) {
        const pt = currentPath[i];
        const px = (pt.x + 0.5) * CELL_SIZE;
        const py = (pt.y + 0.5) * CELL_SIZE;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      ctx.fillStyle = isReturnHome ? '#34D399' : '#22D3EE';
      for (let i = 1; i < currentPath.length; i++) {
        const pt = currentPath[i];
        ctx.beginPath();
        ctx.arc(
          (pt.x + 0.5) * CELL_SIZE,
          (pt.y + 0.5) * CELL_SIZE,
          2.0,
          0,
          Math.PI * 2
        );
        ctx.fill();
      }
      ctx.restore();
    }

    // 9. Render DWA Local Velocity Rollout Arcs (Candidate & Winning Trajectories)
    if (layers.showDwaRollouts && dwaRollouts.length > 0) {
      ctx.save();
      for (const arc of dwaRollouts) {
        if (arc.points.length < 2) continue;
        ctx.beginPath();
        for (let i = 0; i < arc.points.length; i++) {
          const px = (arc.points[i].x + 0.5) * CELL_SIZE;
          const py = (arc.points[i].y + 0.5) * CELL_SIZE;
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        if (arc.isSelected) {
          ctx.strokeStyle = '#34D399';
          ctx.lineWidth = 2.4;
        } else if (arc.isCollision) {
          ctx.strokeStyle = 'rgba(244, 63, 94, 0.32)';
          ctx.lineWidth = 1.0;
        } else {
          ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
          ctx.lineWidth = 1.1;
        }
        ctx.stroke();
      }
      ctx.restore();
    }

    // 10. Render Moving Dynamic Obstacles (Patrolling AGVs / Workers)
    if (layers.showDynamicObstacles && dynamicObstacles.length > 0) {
      for (const obs of dynamicObstacles) {
        const ox = (obs.x + 0.5) * CELL_SIZE;
        const oy = (obs.y + 0.5) * CELL_SIZE;

        ctx.save();
        // Dynamic repulsive inflation ring
        ctx.strokeStyle = obs.detectedByLidar
          ? 'rgba(251, 146, 60, 0.65)'
          : 'rgba(251, 146, 60, 0.25)';
        ctx.fillStyle = obs.detectedByLidar
          ? 'rgba(251, 146, 60, 0.18)'
          : 'rgba(251, 146, 60, 0.07)';
        ctx.lineWidth = 1.25;
        ctx.beginPath();
        ctx.arc(ox, oy, CELL_SIZE * 1.15, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Obstacle core body
        ctx.fillStyle = '#EA580C';
        ctx.strokeStyle = '#FED7AA';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(ox, oy, CELL_SIZE * 0.38, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Velocity vector arrow
        ctx.strokeStyle = '#FFEDD5';
        ctx.lineWidth = 1.75;
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        ctx.lineTo(ox + obs.vx * 11, oy + obs.vy * 11);
        ctx.stroke();

        ctx.restore();
      }
    }

    // 11. Render Selected Frontier Target Reticle
    if (layers.showFrontiers && selectedFrontier) {
      const fx = (selectedFrontier.vantagePoint.x + 0.5) * CELL_SIZE;
      const fy = (selectedFrontier.vantagePoint.y + 0.5) * CELL_SIZE;
      ctx.save();
      ctx.strokeStyle = '#F59E0B';
      ctx.lineWidth = 1.75;
      ctx.beginPath();
      ctx.arc(fx, fy, CELL_SIZE * 0.48, 0, Math.PI * 2);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(fx - 8, fy);
      ctx.lineTo(fx + 8, fy);
      ctx.moveTo(fx, fy - 8);
      ctx.lineTo(fx, fy + 8);
      ctx.stroke();
      ctx.restore();
    }

    // 12. Render Home Dock Station (Origin)
    {
      const dx = homeDock.x * CELL_SIZE;
      const dy = homeDock.y * CELL_SIZE;
      ctx.save();
      ctx.fillStyle = 'rgba(16, 185, 129, 0.22)';
      ctx.fillRect(dx - 2, dy - 2, CELL_SIZE + 4, CELL_SIZE + 4);
      ctx.strokeStyle = '#10B981';
      ctx.lineWidth = 2;
      ctx.strokeRect(dx, dy, CELL_SIZE, CELL_SIZE);

      ctx.fillStyle = '#10B981';
      ctx.font = '700 10px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('H', dx + CELL_SIZE / 2, dy + CELL_SIZE / 2);
      ctx.restore();
    }

    // 13. Render AMR Robot Footprint, Safety Shield Halo, Heading Vector & LiDAR Turret
    {
      const rx = (robotPose.x + 0.5) * CELL_SIZE;
      const ry = (robotPose.y + 0.5) * CELL_SIZE;
      const radius = CELL_SIZE * 0.44;

      ctx.save();
      // Outer safety shield ring colored by obstacle avoidance state
      ctx.fillStyle =
        fsmState === 'AVOIDING_OBSTACLE'
          ? 'rgba(251, 146, 60, 0.34)'
          : fsmState === 'RETURNING_HOME' || fsmState === 'DOCKED_COMPLETE'
          ? 'rgba(16, 185, 129, 0.28)'
          : 'rgba(6, 182, 212, 0.28)';
      ctx.beginPath();
      ctx.arc(rx, ry, radius + 5, 0, Math.PI * 2);
      ctx.fill();

      // Robot chassis
      ctx.fillStyle = '#0F172A';
      ctx.strokeStyle =
        fsmState === 'AVOIDING_OBSTACLE'
          ? '#FB923C'
          : fsmState === 'RETURNING_HOME' || fsmState === 'DOCKED_COMPLETE'
          ? '#10B981'
          : '#06B6D4';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.arc(rx, ry, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Heading indicator line
      const hx = rx + Math.cos(robotPose.theta) * (radius + 5);
      const hy = ry + Math.sin(robotPose.theta) * (radius + 5);
      ctx.strokeStyle = '#F8FAFC';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(hx, hy);
      ctx.stroke();

      // Center LiDAR turret dome
      ctx.fillStyle = fsmState === 'AVOIDING_OBSTACLE' ? '#FB923C' : '#22D3EE';
      ctx.beginPath();
      ctx.arc(rx, ry, 3.2, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    // 14. Render Hovered Cell Crosshair Highlight
    if (hoveredCell) {
      const hx = hoveredCell.x * CELL_SIZE;
      const hy = hoveredCell.y * CELL_SIZE;
      ctx.save();
      ctx.strokeStyle =
        activeBrush === 'DRAW_WALL'
          ? '#F43F5E'
          : activeBrush === 'ERASE_WALL'
          ? '#38BDF8'
          : activeBrush === 'SPAWN_DYNAMIC_OBSTACLE'
          ? '#FB923C'
          : activeBrush === 'SET_HOME_DOCK'
          ? '#10B981'
          : '#F8FAFC';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(hx + 0.5, hy + 0.5, CELL_SIZE - 1, CELL_SIZE - 1);
      ctx.restore();
    }

    ctx.restore();
  }, [
    grid,
    robotPose,
    homeDock,
    lidarRays,
    frontiers,
    selectedFrontier,
    currentPath,
    breadcrumbTrail,
    deadEndRecords,
    dynamicObstacles,
    dwaRollouts,
    highlightedDeadEndId,
    fsmState,
    layers,
    hoveredCell,
    activeBrush,
    canvasWidth,
    canvasHeight,
    rows,
    cols,
  ]);

  const getCellCoordsFromEvent = (
    e: React.MouseEvent<HTMLCanvasElement>
  ): GridCoord | null => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvasWidth / rect.width;
    const scaleY = canvasHeight / rect.height;
    const x = Math.floor(((e.clientX - rect.left) * scaleX) / CELL_SIZE);
    const y = Math.floor(((e.clientY - rect.top) * scaleY) / CELL_SIZE);
    if (x >= 0 && x < cols && y >= 0 && y < rows) {
      return { x, y };
    }
    return null;
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const coords = getCellCoordsFromEvent(e);
    if (!coords) {
      setHoveredCell(null);
      return;
    }
    setHoveredCell(grid[coords.y][coords.x]);
    if (
      isMouseDown &&
      (activeBrush === 'DRAW_WALL' || activeBrush === 'ERASE_WALL')
    ) {
      onCellInteract(coords.x, coords.y, true);
    }
  };

  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    setIsMouseDown(true);
    const coords = getCellCoordsFromEvent(e);
    if (coords) {
      onCellInteract(coords.x, coords.y, false);
    }
  };

  const handleMouseUp = () => {
    setIsMouseDown(false);
  };

  return (
    <div className="relative w-full bg-[#090D16] border border-[#1E293B] rounded-lg overflow-hidden">
      {/* Top Canvas Telemetry & Legend Ribbon */}
      <div className="flex flex-wrap items-center justify-between gap-4 px-4 py-2.5 bg-[#111827] border-b border-[#1E293B] text-xs">
        <div className="flex flex-wrap items-center gap-3 text-slate-300 font-mono tabular-nums">
          <span className="text-cyan-400">
            POSE: ({robotPose.x}, {robotPose.y})
          </span>
          <span aria-hidden="true" className="text-slate-600">
            ·
          </span>
          <span
            className={
              sectorClearance.status === 'CRITICAL_BACKOFF'
                ? 'text-rose-400 font-semibold'
                : sectorClearance.status === 'DWA_EVASION'
                ? 'text-orange-400 font-semibold'
                : 'text-emerald-400'
            }
          >
            SHIELD: {sectorClearance.status} (F:{sectorClearance.front.toFixed(1)}c · L:{sectorClearance.frontLeft.toFixed(1)}c · R:{sectorClearance.frontRight.toFixed(1)}c)
          </span>
        </div>

        {/* Visual Map Key */}
        <div className="flex flex-wrap items-center gap-3.5 text-xs text-slate-300">
          <div className="flex items-center gap-1.5">
            <span className="w-3 h-3 inline-block bg-indigo-500/40 border border-indigo-400" />
            <span>Costmap Inflation</span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="w-3 h-3 inline-block rounded-full bg-orange-500 border border-orange-200" />
            <span>Dynamic Obstacle</span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="w-3 h-3 inline-block bg-emerald-400" />
            <span>DWA Rollout</span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="w-3 h-3 inline-block bg-amber-500/50 border border-amber-400" />
            <span>Frontier</span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="w-3 h-3 inline-block bg-rose-500/35 border border-rose-500" />
            <span>Locked Dead-End</span>
          </div>
        </div>
      </div>

      {/* Main Responsive RViz Canvas Container */}
      <div className="relative flex items-center justify-center p-3 bg-[#070A12] overflow-x-auto">
        <canvas
          ref={canvasRef}
          style={{
            width: '100%',
            maxWidth: `${canvasWidth}px`,
            aspectRatio: `${cols} / ${rows}`,
          }}
          onMouseMove={handleMouseMove}
          onMouseDown={handleMouseDown}
          onMouseUp={handleMouseUp}
          onMouseLeave={() => {
            setIsMouseDown(false);
            setHoveredCell(null);
          }}
          className="cursor-crosshair block border border-slate-800/80 shadow-inner"
        />

        {/* Floating Cell Probe HUD (Bottom-Right Corner of Canvas) */}
        {hoveredCell && (
          <div className="pointer-events-none absolute bottom-5 right-5 bg-[#0F172A]/95 border border-slate-700 px-3 py-2 rounded text-xs font-mono text-slate-200 shadow-lg">
            <div className="flex items-center gap-2 text-slate-400">
              <span>
                CELL ({hoveredCell.x}, {hoveredCell.y})
              </span>
              <span>·</span>
              <span
                className={
                  hoveredCell.belief === 'FREE'
                    ? 'text-cyan-400'
                    : hoveredCell.belief === 'OCCUPIED'
                    ? 'text-slate-100'
                    : 'text-slate-400'
                }
              >
                {hoveredCell.belief}
              </span>
            </div>
            <div className="mt-1 flex items-center gap-2 text-[11px] text-slate-300">
              <span>Inflation: {(hoveredCell.inflationCost * 100).toFixed(0)}%</span>
              <span>·</span>
              <span>Visits: {hoveredCell.visitCount}</span>
              {hoveredCell.isDeadEndTaboo && (
                <>
                  <span>·</span>
                  <span className="text-rose-400 font-semibold">
                    TABOO (DE-{String(hoveredCell.deadEndClusterId ?? 0).padStart(2, '0')})
                  </span>
                </>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};
