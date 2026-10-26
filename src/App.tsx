import React, { Suspense, lazy, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Bot,
  Box,
  Columns2,
  Compass,
  Eraser,
  Eye,
  Home,
  Layers,
  Map as MapIcon,
  Pause,
  Play,
  RotateCcw,
  ShieldAlert,
  SkipForward,
  Sliders,
  SquarePen,
} from 'lucide-react';
import {
  cloneGrid,
  createGridFromPreset,
  MAP_PRESETS,
} from './algorithms/maps';
import {
  computeDwaRolloutsAndReactiveStep,
  detectAndLockDeadEnds,
  extractFrontiers,
  findAStarPath,
  performLidarScan,
  spawnDynamicObstacles,
  stepDynamicObstacles,
} from './algorithms/slamEngine';
import { AlgorithmInspector } from './components/AlgorithmInspector';
import { DeadEndLedger } from './components/DeadEndLedger';
import { RvizCanvas } from './components/RvizCanvas';

const GazeboViewport3D = lazy(() => import('./components/GazeboViewport3D'));
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
  SimulationConfig,
  TelemetryEvent,
} from './types/slam';

type ViewportLayoutMode = 'SPLIT_GAZEBO_RVIZ' | 'GAZEBO_3D_ONLY' | 'RVIZ_2D_ONLY';

export default function App() {
  const [selectedPresetId, setSelectedPresetId] = useState<string>(
    MAP_PRESETS[0].id
  );
  const [viewportMode, setViewportMode] =
    useState<ViewportLayoutMode>('SPLIT_GAZEBO_RVIZ');

  const [config, setConfig] = useState<SimulationConfig>({
    lidarRange: 9,
    lidarRayCount: 96,
    speedHz: 8,
    infoGainWeight: 1.8,
    distanceWeight: 1.1,
    pruningAggression: 'STANDARD',
    safetyInflationRadius: 2.0,
    dynamicObstacleCount: 3,
  });

  const [layers, setLayers] = useState<LayerVisibility>({
    showGroundTruthGhost: true,
    showLidarRays: true,
    showCostmapInflation: true,
    showDwaRollouts: true,
    showDynamicObstacles: true,
    showFrontiers: true,
    showDeadEndTaboo: true,
    showPlannedPath: true,
    showBreadcrumbTrail: true,
    showGridLines: true,
  });

  const [activeBrush, setActiveBrush] = useState<MapBrushTool>('INSPECT');
  const [highlightedDeadEndId, setHighlightedDeadEndId] = useState<
    number | null
  >(null);

  // Simulation State
  const [grid, setGrid] = useState<CellState[][]>(() => {
    const init = createGridFromPreset(MAP_PRESETS[0]);
    return init.grid;
  });
  const [homeDock, setHomeDock] = useState<GridCoord>(
    () => MAP_PRESETS[0].homeDock
  );
  const [robotPose, setRobotPose] = useState<{
    x: number;
    y: number;
    theta: number;
  }>(() => ({
    x: MAP_PRESETS[0].homeDock.x,
    y: MAP_PRESETS[0].homeDock.y,
    theta: 0,
  }));

  const [isRunning, setIsRunning] = useState<boolean>(false);
  const [tickCount, setTickCount] = useState<number>(0);
  const [fsmState, setFsmState] = useState<FSMState>('IDLE');
  const [lidarRays, setLidarRays] = useState<LidarRay[]>([]);
  const [frontiers, setFrontiers] = useState<FrontierCluster[]>([]);
  const [selectedFrontier, setSelectedFrontier] =
    useState<FrontierCluster | null>(null);
  const [currentPath, setCurrentPath] = useState<GridCoord[]>([]);
  const [breadcrumbTrail, setBreadcrumbTrail] = useState<GridCoord[]>([]);
  const [deadEndRecords, setDeadEndRecords] = useState<DeadEndRecord[]>([]);
  const [dynamicObstacles, setDynamicObstacles] = useState<DynamicObstacle[]>(
    []
  );
  const [dwaRollouts, setDwaRollouts] = useState<DWARolloutArc[]>([]);
  const [sectorClearance, setSectorClearance] = useState<SectorClearance>({
    front: 9,
    frontLeft: 9,
    frontRight: 9,
    rear: 9,
    minDistance: 9,
    status: 'NOMINAL',
  });
  const [avoidanceCount, setAvoidanceCount] = useState<number>(0);
  const [events, setEvents] = useState<TelemetryEvent[]>([]);

  // Refs for synchronous access inside interval loop
  const stateRef = useRef({
    grid,
    homeDock,
    robotPose,
    tickCount,
    fsmState,
    currentPath,
    selectedFrontier,
    breadcrumbTrail,
    deadEndRecords,
    dynamicObstacles,
    config,
  });

  useEffect(() => {
    stateRef.current = {
      grid,
      homeDock,
      robotPose,
      tickCount,
      fsmState,
      currentPath,
      selectedFrontier,
      breadcrumbTrail,
      deadEndRecords,
      dynamicObstacles,
      config,
    };
  }, [
    grid,
    homeDock,
    robotPose,
    tickCount,
    fsmState,
    currentPath,
    selectedFrontier,
    breadcrumbTrail,
    deadEndRecords,
    dynamicObstacles,
    config,
  ]);

  const activePreset = useMemo(
    () =>
      MAP_PRESETS.find((p) => p.id === selectedPresetId) || MAP_PRESETS[0],
    [selectedPresetId]
  );

  // Initialize or Reset Gazebo Warehouse Environment
  const initializeSimulation = useCallback(
    (presetId: string, customConfig?: SimulationConfig) => {
      const preset =
        MAP_PRESETS.find((p) => p.id === presetId) || MAP_PRESETS[0];
      const activeCfg = customConfig || stateRef.current.config;
      const { grid: freshGrid, homeDock: dock } = createGridFromPreset(preset);

      freshGrid[dock.y][dock.x].visitCount = 1;
      const initObstacles = spawnDynamicObstacles(
        freshGrid,
        dock,
        activeCfg.dynamicObstacleCount
      );
      const scanRes = performLidarScan(
        freshGrid,
        { x: dock.x, y: dock.y, theta: 0 },
        activeCfg,
        0,
        initObstacles
      );
      const initialRecords: DeadEndRecord[] = [];
      detectAndLockDeadEnds(freshGrid, dock, initialRecords, activeCfg, 0);
      const initFrontiers = extractFrontiers(freshGrid, dock, activeCfg);

      setIsRunning(false);
      setGrid(freshGrid);
      setHomeDock(dock);
      setRobotPose({ x: dock.x, y: dock.y, theta: 0 });
      setTickCount(0);
      setFsmState('IDLE');
      setLidarRays(scanRes.rays);
      setSectorClearance(scanRes.sectorClearance);
      setFrontiers(initFrontiers);
      setSelectedFrontier(initFrontiers[0] || null);
      setCurrentPath([]);
      setBreadcrumbTrail([dock]);
      setDeadEndRecords(initialRecords);
      setDynamicObstacles(initObstacles);
      setDwaRollouts([]);
      setAvoidanceCount(0);
      setHighlightedDeadEndId(null);
      setEvents([
        {
          id: `init-${Date.now()}`,
          tick: 0,
          type: 'INFO',
          message: `Loaded Gazebo World: "${preset.name}"`,
          detail: `3D warehouse initialized with ${initObstacles.length} patrolling AGVs and zero preloaded RViz map.`,
        },
      ]);
    },
    []
  );

  useEffect(() => {
    initializeSimulation(MAP_PRESETS[0].id);
  }, [initializeSimulation]);

  // Core Single-Step Autonomous SLAM + DWA Obstacle Avoidance + Dead-End Pruning + Return-Home Engine
  const executeStep = useCallback(() => {
    const {
      grid: prevGrid,
      homeDock: dock,
      robotPose: pose,
      tickCount: prevTick,
      currentPath: prevPath,
      selectedFrontier: prevTarget,
      breadcrumbTrail: prevTrail,
      deadEndRecords: prevDeadEnds,
      dynamicObstacles: prevObstacles,
      config: cfg,
    } = stateRef.current;

    const nextTick = prevTick + 1;
    const nextGrid = cloneGrid(prevGrid);
    const nextDeadEnds = prevDeadEnds.map((r) => ({
      ...r,
      cells: [...r.cells],
    }));

    // 0. Step Moving Dynamic Obstacles along warehouse aisles
    const movedObstacles = stepDynamicObstacles(
      nextGrid,
      prevObstacles,
      pose,
      dock
    );

    // 1. Perform 360° LiDAR scan (separating transient dynamic hits from static walls)
    performLidarScan(nextGrid, pose, cfg, nextTick, movedObstacles);

    // 2. Run Topological Dead-End Pruning & Taboo Memory Locking
    const pruneBefore = detectAndLockDeadEnds(
      nextGrid,
      dock,
      nextDeadEnds,
      cfg,
      nextTick
    );

    // 3. Extract active non-taboo Frontiers
    let activeFrontiers = extractFrontiers(nextGrid, pose, cfg);

    let nextPose = { ...pose };
    let nextPath = [...prevPath];
    let nextTarget: FrontierCluster | null = prevTarget;
    let nextFsm: FSMState = 'EXPLORING_FRONTIERS';
    let latestRollouts: DWARolloutArc[] = [];
    const newEvents: TelemetryEvent[] = [];

    if (pruneBefore.newRecords.length > 0) {
      for (const rec of pruneBefore.newRecords) {
        newEvents.push({
          id: `de-${nextTick}-${rec.id}`,
          tick: nextTick,
          type: 'DEAD_END_LOCKED',
          message: `Sealed ${rec.label} (${rec.classification})`,
          detail: `Locked ${rec.cellCount} cells at bay/room entrance (${rec.throatCoord.x}, ${rec.throatCoord.y}) — never re-entering.`,
        });
      }
    }

    const detectedObstacles = movedObstacles.filter((o) => o.detectedByLidar);

    // 4. Global Frontier Exploration + Local DWA Obstacle Avoidance
    if (activeFrontiers.length > 0) {
      let needNewPlan = true;
      const pathBlockedByDynamic = nextPath
        .slice(1, 4)
        .some((pt) =>
          detectedObstacles.some(
            (o) => Math.hypot(o.x - pt.x, o.y - pt.y) <= 1.2
          )
        );

      if (
        !pathBlockedByDynamic &&
        nextPath.length > 1 &&
        nextTarget &&
        nextGrid[nextTarget.vantagePoint.y]?.[nextTarget.vantagePoint.x]
          ?.isFrontier &&
        !nextGrid[nextTarget.vantagePoint.y]?.[nextTarget.vantagePoint.x]
          ?.isDeadEndTaboo
      ) {
        needNewPlan = false;
      }

      if (needNewPlan) {
        nextPath = [];
        nextTarget = null;

        const scoredCandidates = activeFrontiers
          .map((cluster) => {
            const dx = cluster.vantagePoint.x - pose.x;
            const dy = cluster.vantagePoint.y - pose.y;
            const angleToTarget = Math.atan2(dy, dx);
            const angleDiff = Math.abs(
              Math.atan2(
                Math.sin(angleToTarget - pose.theta),
                Math.cos(angleToTarget - pose.theta)
              )
            );
            const headingBonus = (Math.PI - angleDiff) * 1.2;
            const hysteresisBonus =
              prevTarget &&
              Math.hypot(
                cluster.vantagePoint.x - prevTarget.vantagePoint.x,
                cluster.vantagePoint.y - prevTarget.vantagePoint.y
              ) <= 4
                ? 6.0
                : 0;
            return {
              cluster,
              score: cluster.utilityScore + headingBonus + hysteresisBonus,
            };
          })
          .sort((a, b) => b.score - a.score);

        for (const item of scoredCandidates) {
          const cand = item.cluster;
          let path = findAStarPath(
            nextGrid,
            pose,
            cand.vantagePoint,
            true,
            detectedObstacles
          );
          if (!path) {
            for (const altCell of cand.cells) {
              path = findAStarPath(
                nextGrid,
                pose,
                altCell,
                true,
                detectedObstacles
              );
              if (path) {
                cand.vantagePoint = altCell;
                break;
              }
            }
          }
          if (!path) {
            path = findAStarPath(nextGrid, pose, cand.vantagePoint, true, []);
          }
          if (path && path.length > 1) {
            nextPath = path;
            nextTarget = {
              ...cand,
              pathDistance: path.length - 1,
            };
            break;
          }
        }
      }

      if (nextPath.length > 1) {
        const dwaRes = computeDwaRolloutsAndReactiveStep(
          nextGrid,
          pose,
          nextPath,
          detectedObstacles
        );
        latestRollouts = dwaRes.rollouts;

        const stepCoord = dwaRes.nextStep;
        const dx = stepCoord.x - pose.x;
        const dy = stepCoord.y - pose.y;
        nextPose = {
          x: stepCoord.x,
          y: stepCoord.y,
          theta: dx === 0 && dy === 0 ? pose.theta : Math.atan2(dy, dx),
        };

        if (
          stepCoord.x === nextPath[1].x &&
          stepCoord.y === nextPath[1].y
        ) {
          nextPath = nextPath.slice(1);
        } else {
          nextPath = [stepCoord, ...nextPath.slice(1)];
        }

        if (dwaRes.evadingObstacle) {
          nextFsm = 'AVOIDING_OBSTACLE';
          setAvoidanceCount((c) => c + 1);
          if (stateRef.current.fsmState !== 'AVOIDING_OBSTACLE') {
            newEvents.push({
              id: `avoid-${nextTick}`,
              tick: nextTick,
              type: 'OBSTACLE_AVOIDED',
              message: 'DWA Reactive Evasion — Sidestepping Warehouse AGV',
              detail: `3D LiDAR detected moving AGV in aisle. Selected collision-free DWA arc to (${stepCoord.x}, ${stepCoord.y}).`,
            });
          }
        }
      } else {
        activeFrontiers = [];
      }
    }

    // 5. If no reachable frontiers remain -> Transition to RETURNING_HOME
    if (activeFrontiers.length === 0) {
      nextTarget = null;
      if (pose.x === dock.x && pose.y === dock.y) {
        nextFsm = 'DOCKED_COMPLETE';
        nextPath = [dock];
        setIsRunning(false);
        newEvents.push({
          id: `done-${nextTick}`,
          tick: nextTick,
          type: 'COMPLETE',
          message: 'Mission Complete — AMR Docked at Warehouse Origin',
          detail: `100% of reachable facility mapped and ${nextDeadEnds.length} dead-end bays/rooms permanently memorized.`,
        });
      } else {
        nextFsm = 'RETURNING_HOME';
        if (stateRef.current.fsmState !== 'RETURNING_HOME') {
          newEvents.push({
            id: `ret-${nextTick}`,
            tick: nextTick,
            type: 'RETURN_HOME',
            message: 'All Reachable Frontiers Exhausted — Returning to Dock',
            detail: `Computing inflation-safe A* return trajectory to Origin Dock (${dock.x}, ${dock.y}).`,
          });
        }

        const homePath =
          findAStarPath(nextGrid, pose, dock, true, detectedObstacles) ||
          findAStarPath(nextGrid, pose, dock, false, []);

        if (homePath && homePath.length > 1) {
          const dwaHome = computeDwaRolloutsAndReactiveStep(
            nextGrid,
            pose,
            homePath,
            detectedObstacles
          );
          latestRollouts = dwaHome.rollouts;

          const stepCoord = dwaHome.nextStep;
          const dx = stepCoord.x - pose.x;
          const dy = stepCoord.y - pose.y;
          nextPose = {
            x: stepCoord.x,
            y: stepCoord.y,
            theta: dx === 0 && dy === 0 ? pose.theta : Math.atan2(dy, dx),
          };
          nextPath = homePath.slice(1);

          if (nextPose.x === dock.x && nextPose.y === dock.y) {
            nextFsm = 'DOCKED_COMPLETE';
            setIsRunning(false);
            newEvents.push({
              id: `done-${nextTick}`,
              tick: nextTick,
              type: 'COMPLETE',
              message: 'Mission Complete — AMR Docked at Warehouse Origin',
              detail: `Autonomous Gazebo SLAM exploration, DWA obstacle avoidance, and A* return finished at tick T+${nextTick}.`,
            });
          }
        } else {
          nextFsm = 'NO_REACHABLE_FRONTIERS';
          setIsRunning(false);
        }
      }
    }

    // 6. Post-move LiDAR Scan & Immediate Dead-End Pruning at the new pose
    nextGrid[nextPose.y][nextPose.x].visitCount += 1;
    const postScan = performLidarScan(
      nextGrid,
      nextPose,
      cfg,
      nextTick,
      movedObstacles
    );
    const pruneAfter = detectAndLockDeadEnds(
      nextGrid,
      dock,
      nextDeadEnds,
      cfg,
      nextTick
    );

    if (pruneAfter.newRecords.length > 0) {
      if (nextFsm === 'EXPLORING_FRONTIERS') {
        nextFsm = 'PRUNING_DEAD_END';
      }
      for (const rec of pruneAfter.newRecords) {
        newEvents.push({
          id: `de-post-${nextTick}-${rec.id}`,
          tick: nextTick,
          type: 'DEAD_END_LOCKED',
          message: `Sealed ${rec.label} (${rec.classification})`,
          detail: `Locked ${rec.cellCount} cells at bay/room entrance (${rec.throatCoord.x}, ${rec.throatCoord.y}) — AMR will never re-enter.`,
        });
      }
    }

    const updatedFrontiers = extractFrontiers(nextGrid, nextPose, cfg);

    setGrid(nextGrid);
    setRobotPose(nextPose);
    setTickCount(nextTick);
    setFsmState(nextFsm);
    setLidarRays(postScan.rays);
    setSectorClearance(postScan.sectorClearance);
    setFrontiers(updatedFrontiers);
    setSelectedFrontier(nextTarget);
    setCurrentPath(nextPath);
    setBreadcrumbTrail([...prevTrail, { x: nextPose.x, y: nextPose.y }]);
    setDeadEndRecords(nextDeadEnds);
    setDynamicObstacles(movedObstacles);
    setDwaRollouts(latestRollouts);
    if (newEvents.length > 0) {
      setEvents((prev) => [...newEvents, ...prev].slice(0, 40));
    }
  }, []);

  // Simulation Loop Timer
  useEffect(() => {
    if (!isRunning) return;
    const intervalMs = Math.max(45, Math.round(1000 / config.speedHz));
    const timer = window.setInterval(() => {
      executeStep();
    }, intervalMs);
    return () => window.clearInterval(timer);
  }, [isRunning, config.speedHz, executeStep]);

  // Interactive Canvas Map Editing (Draw Wall/Rack, Erase, Spawn Moving AGV, Set Home Dock)
  const handleCellInteract = (x: number, y: number, isDrag: boolean) => {
    if (activeBrush === 'INSPECT') {
      const cell = grid[y]?.[x];
      if (cell && cell.deadEndClusterId !== null) {
        setHighlightedDeadEndId((prev) =>
          prev === cell.deadEndClusterId ? null : cell.deadEndClusterId
        );
      }
      return;
    }

    if (x <= 0 || y <= 0 || x >= grid[0].length - 1 || y >= grid.length - 1) {
      return;
    }
    if (x === robotPose.x && y === robotPose.y) return;

    if (activeBrush === 'SPAWN_DYNAMIC_OBSTACLE' && !isDrag) {
      if (grid[y][x].groundTruth === 'WALL') return;
      setDynamicObstacles((prev) => {
        const existingIdx = prev.findIndex((o) => o.x === x && o.y === y);
        if (existingIdx >= 0) {
          return prev.filter((_, idx) => idx !== existingIdx);
        }
        const nextId = prev.reduce((m, o) => Math.max(m, o.id), 0) + 1;
        return [
          ...prev,
          {
            id: nextId,
            label: `AGV-${String(nextId).padStart(2, '0')}`,
            x,
            y,
            vx: 1,
            vy: 0,
            detectedByLidar: true,
            lastSeenTick: tickCount,
          },
        ];
      });
      return;
    }

    if (activeBrush === 'SET_HOME_DOCK' && !isDrag) {
      const nextGrid = cloneGrid(grid);
      nextGrid[y][x].groundTruth = 'EMPTY';
      setGrid(nextGrid);
      setHomeDock({ x, y });
      return;
    }

    if (activeBrush === 'DRAW_WALL' || activeBrush === 'ERASE_WALL') {
      if (x === homeDock.x && y === homeDock.y) return;
      const targetTruth = activeBrush === 'DRAW_WALL' ? 'WALL' : 'EMPTY';
      if (grid[y][x].groundTruth === targetTruth) return;

      const nextGrid = cloneGrid(grid);
      nextGrid[y][x].groundTruth = targetTruth;
      nextGrid[y][x].belief = 'UNKNOWN';
      nextGrid[y][x].isDeadEndTaboo = false;
      nextGrid[y][x].deadEndClusterId = null;
      setGrid(nextGrid);
    }
  };

  // Compute live quantitative telemetry metrics
  const metrics = useMemo(() => {
    let totalNavigableGroundTruth = 0;
    let mappedFreeCount = 0;
    let mappedWallCount = 0;
    let lockedTabooCells = 0;

    for (let y = 0; y < grid.length; y++) {
      for (let x = 0; x < grid[0].length; x++) {
        const c = grid[y][x];
        if (c.groundTruth === 'EMPTY') totalNavigableGroundTruth++;
        if (c.belief === 'FREE') mappedFreeCount++;
        if (c.belief === 'OCCUPIED') mappedWallCount++;
        if (c.isDeadEndTaboo) lockedTabooCells++;
      }
    }

    const coveragePct =
      totalNavigableGroundTruth > 0
        ? Math.min(100, (mappedFreeCount / totalNavigableGroundTruth) * 100)
        : 0;

    return {
      coveragePct,
      mappedFreeCount,
      mappedWallCount,
      lockedTabooCells,
      frontierCellCount: frontiers.reduce((s, f) => s + f.cells.length, 0),
    };
  }, [grid, frontiers]);

  const handlePrimaryAction = () => {
    if (
      fsmState === 'DOCKED_COMPLETE' ||
      fsmState === 'NO_REACHABLE_FRONTIERS'
    ) {
      initializeSimulation(selectedPresetId);
      setTimeout(() => setIsRunning(true), 60);
    } else {
      setIsRunning((prev) => !prev);
      if (fsmState === 'IDLE') {
        setFsmState('EXPLORING_FRONTIERS');
      }
    }
  };

  return (
    <div className="min-h-screen bg-[#0B0E17] text-slate-100 flex flex-col">
      {/* Top Bar Contract: 3 Zones separated by gap-8 */}
      <header className="flex items-center justify-between gap-8 px-6 py-3.5 bg-[#0E1320] border-b border-[#1E293B]">
        {/* Zone 1: Single text element wordmark */}
        <a
          href="#stage"
          className="text-lg font-bold tracking-tight text-slate-100 font-display whitespace-nowrap shrink-0"
        >
          AutoSLAM Gazebo & RViz2
        </a>

        {/* Zone 2: 4 concise single-line navigation links */}
        <nav className="hidden md:flex items-center gap-6 text-sm font-medium text-slate-400">
          <a
            href="#stage"
            className="hover:text-slate-100 transition-colors whitespace-nowrap shrink-0"
          >
            Gazebo & RViz Stage
          </a>
          <a
            href="#dead-end-memory"
            className="hover:text-slate-100 transition-colors whitespace-nowrap shrink-0"
          >
            Dead-End Memory
          </a>
          <a
            href="#algorithm-pipeline"
            className="hover:text-slate-100 transition-colors whitespace-nowrap shrink-0"
          >
            ROS 2 & Gazebo Code
          </a>
          <a
            href="#telemetry-log"
            className="hover:text-slate-100 transition-colors whitespace-nowrap shrink-0"
          >
            Mission Telemetry
          </a>
        </nav>

        {/* Zone 3: 1 primary action button */}
        <div className="flex items-center gap-3 shrink-0">
          <button
            type="button"
            onClick={handlePrimaryAction}
            className={`px-4 py-2 text-xs font-semibold rounded-lg transition-colors whitespace-nowrap shrink-0 ${
              isRunning
                ? 'bg-amber-500 text-slate-950 hover:bg-amber-400'
                : 'bg-cyan-500 text-slate-950 hover:bg-cyan-400'
            }`}
          >
            {isRunning
              ? 'Pause Simulation'
              : fsmState === 'DOCKED_COMPLETE'
              ? 'Restart Mission'
              : 'Start Exploration'}
          </button>
        </div>
      </header>

      {/* Main 1600px Asymmetric Robotics Workstation */}
      <main className="flex-1 max-w-[1640px] w-full mx-auto px-4 sm:px-6 py-6 space-y-6">
        {/* Top Quantitative Telemetry Strip */}
        <section
          id="stage"
          className="grid grid-cols-2 md:grid-cols-5 gap-3 bg-[#111827] border border-[#1E293B] rounded-lg p-4"
        >
          <div>
            <div className="text-xs text-slate-400">AMR Intelligence State</div>
            <div className="mt-1 font-mono text-sm font-semibold tabular-nums">
              {fsmState === 'IDLE' && (
                <span className="text-slate-300">○ STANDBY AT DOCK</span>
              )}
              {fsmState === 'EXPLORING_FRONTIERS' && (
                <span className="text-cyan-400">● EXPLORING FRONTIERS</span>
              )}
              {fsmState === 'AVOIDING_OBSTACLE' && (
                <span className="text-orange-400">◆ DWA OBSTACLE EVASION</span>
              )}
              {fsmState === 'PRUNING_DEAD_END' && (
                <span className="text-rose-400">▲ LOCKING DEAD-END TABOO</span>
              )}
              {fsmState === 'RETURNING_HOME' && (
                <span className="text-emerald-400">◆ A* RETURNING TO DOCK</span>
              )}
              {fsmState === 'DOCKED_COMPLETE' && (
                <span className="text-emerald-400">✔ DOCKED (COMPLETE)</span>
              )}
              {fsmState === 'NO_REACHABLE_FRONTIERS' && (
                <span className="text-amber-400">✖ PATH BLOCKED</span>
              )}
            </div>
            <div className="text-xs font-mono text-slate-500 mt-0.5 tabular-nums">
              Elapsed Ticks: T+{tickCount}
            </div>
          </div>

          <div>
            <div className="text-xs text-slate-400">SLAM Coverage (Zero-Prior)</div>
            <div className="mt-1 flex items-baseline gap-1.5">
              <span className="text-2xl font-bold font-mono tabular-nums text-slate-100">
                {metrics.coveragePct.toFixed(1)}
              </span>
              <span className="text-xs font-mono text-slate-400">%</span>
            </div>
            <div className="text-xs font-mono text-slate-500 tabular-nums">
              {metrics.mappedFreeCount} free · {metrics.mappedWallCount} rack/wall cells
            </div>
          </div>

          <div>
            <div className="text-xs text-slate-400">DWA Obstacle Shield</div>
            <div className="mt-1 flex items-baseline gap-1.5">
              <span className="text-2xl font-bold font-mono tabular-nums text-orange-400">
                {avoidanceCount}
              </span>
              <span className="text-xs font-mono text-slate-400">EVASIONS</span>
            </div>
            <div className="text-xs font-mono text-slate-500 tabular-nums">
              {dynamicObstacles.length} 3D moving AGVs · 0 collisions
            </div>
          </div>

          <div>
            <div className="text-xs text-slate-400">Memorized Dead-End Bays</div>
            <div className="mt-1 flex items-baseline gap-1.5">
              <span className="text-2xl font-bold font-mono tabular-nums text-rose-400">
                {deadEndRecords.length}
              </span>
              <span className="text-xs font-mono text-slate-400">ZONES LOCKED</span>
            </div>
            <div className="text-xs font-mono text-slate-500 tabular-nums">
              {metrics.lockedTabooCells} taboo cells permanently pruned
            </div>
          </div>

          <div className="col-span-2 md:col-span-1">
            <div className="text-xs text-slate-400">Active Frontiers</div>
            <div className="mt-1 flex items-baseline gap-1.5">
              <span className="text-2xl font-bold font-mono tabular-nums text-amber-400">
                {frontiers.length}
              </span>
              <span className="text-xs font-mono text-slate-400">CLUSTERS</span>
            </div>
            <div className="text-xs font-mono text-slate-500 tabular-nums">
              Target:{' '}
              {selectedFrontier
                ? `(${selectedFrontier.vantagePoint.x}, ${selectedFrontier.vantagePoint.y})`
                : fsmState === 'RETURNING_HOME' || fsmState === 'DOCKED_COMPLETE'
                ? `Dock (${homeDock.x}, ${homeDock.y})`
                : 'Scanning'}
            </div>
          </div>
        </section>

        {/* Two-Zone Sandbox Layout: Left Control Deck (340px) + Right Split Gazebo/RViz Stage (Flex-1) */}
        <div className="flex flex-col lg:flex-row items-start gap-6">
          {/* Left Control & Calibration Column */}
          <aside className="w-full lg:w-[340px] shrink-0 space-y-4">
            {/* 1. Playback & Execution Controller */}
            <div className="bg-[#111827] border border-[#1E293B] rounded-lg p-4 space-y-4">
              <div className="flex items-center justify-between">
                <h2 className="text-sm font-semibold text-slate-100">
                  Simulation Execution
                </h2>
                <span className="text-xs font-mono text-slate-400 tabular-nums">
                  {config.speedHz} Hz
                </span>
              </div>

              <div className="grid grid-cols-3 gap-2">
                <button
                  type="button"
                  onClick={handlePrimaryAction}
                  className={`flex items-center justify-center gap-1.5 py-2 px-3 rounded text-xs font-semibold transition-colors whitespace-nowrap shrink-0 ${
                    isRunning
                      ? 'bg-amber-500 text-slate-950 hover:bg-amber-400'
                      : 'bg-cyan-500 text-slate-950 hover:bg-cyan-400'
                  }`}
                >
                  {isRunning ? (
                    <>
                      <Pause className="w-3.5 h-3.5" />
                      <span>Pause</span>
                    </>
                  ) : (
                    <>
                      <Play className="w-3.5 h-3.5" />
                      <span>
                        {fsmState === 'DOCKED_COMPLETE' ? 'Replay' : 'Run'}
                      </span>
                    </>
                  )}
                </button>

                <button
                  type="button"
                  disabled={isRunning || fsmState === 'DOCKED_COMPLETE'}
                  onClick={executeStep}
                  className="flex items-center justify-center gap-1.5 py-2 px-3 rounded text-xs font-medium bg-slate-800 text-slate-200 hover:bg-slate-700 disabled:opacity-40 disabled:pointer-events-none border border-slate-700 transition-colors whitespace-nowrap shrink-0"
                >
                  <SkipForward className="w-3.5 h-3.5" />
                  <span>Step +1</span>
                </button>

                <button
                  type="button"
                  onClick={() => initializeSimulation(selectedPresetId)}
                  className="flex items-center justify-center gap-1.5 py-2 px-3 rounded text-xs font-medium bg-slate-800 text-slate-200 hover:bg-slate-700 border border-slate-700 transition-colors whitespace-nowrap shrink-0"
                >
                  <RotateCcw className="w-3.5 h-3.5" />
                  <span>Reset</span>
                </button>
              </div>

              <div>
                <div className="flex justify-between text-xs text-slate-300 mb-1.5">
                  <span>Simulation Tick Rate</span>
                  <span className="font-mono tabular-nums text-cyan-400">
                    {config.speedHz} ticks/sec
                  </span>
                </div>
                <input
                  type="range"
                  min={1}
                  max={18}
                  step={1}
                  value={config.speedHz}
                  onChange={(e) =>
                    setConfig((c) => ({
                      ...c,
                      speedHz: Number(e.target.value),
                    }))
                  }
                  className="w-full accent-cyan-400 cursor-pointer"
                />
              </div>
            </div>

            {/* 2. Gazebo Industrial World Selector */}
            <div className="bg-[#111827] border border-[#1E293B] rounded-lg p-4 space-y-3">
              <div className="flex items-center gap-2">
                <Compass className="w-4 h-4 text-cyan-400" />
                <h2 className="text-sm font-semibold text-slate-100">
                  Gazebo 3D Facility Worlds
                </h2>
              </div>

              <div className="space-y-2">
                {MAP_PRESETS.map((preset) => {
                  const active = preset.id === selectedPresetId;
                  return (
                    <button
                      key={preset.id}
                      type="button"
                      onClick={() => {
                        setSelectedPresetId(preset.id);
                        initializeSimulation(preset.id);
                      }}
                      className={`w-full text-left p-2.5 rounded border transition-colors ${
                        active
                          ? 'bg-cyan-950/35 border-cyan-500/70 text-slate-100'
                          : 'bg-[#0B0F1A] border-[#1E293B] text-slate-300 hover:border-slate-700'
                      }`}
                    >
                      <div className="flex items-center justify-between gap-2">
                        <span className="text-xs font-semibold">
                          {preset.name}
                        </span>
                      </div>
                      <div className="text-[11px] font-mono text-cyan-400/90 mt-0.5">
                        {preset.subtitle}
                      </div>
                      <p className="text-[11px] text-slate-400 mt-1 leading-snug">
                        {preset.description}
                      </p>
                    </button>
                  );
                })}
              </div>
            </div>

            {/* 3. Interactive RViz2 & Gazebo World Editor Brush */}
            <div className="bg-[#111827] border border-[#1E293B] rounded-lg p-4 space-y-3">
              <div className="flex items-center justify-between">
                <h2 className="text-sm font-semibold text-slate-100">
                  World & Rack Builder Tool
                </h2>
                <span className="text-[11px] text-slate-400">
                  Click on 2D Map
                </span>
              </div>

              <div className="grid grid-cols-2 gap-1.5 p-1 bg-[#0B0F1A] rounded-lg border border-[#1E293B]">
                <button
                  type="button"
                  onClick={() => setActiveBrush('INSPECT')}
                  className={`flex items-center justify-center gap-1.5 py-1.5 px-2.5 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    activeBrush === 'INSPECT'
                      ? 'bg-slate-800 text-cyan-300 shadow-sm'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Eye className="w-3.5 h-3.5" />
                  <span>Inspect Cell</span>
                </button>

                <button
                  type="button"
                  onClick={() => setActiveBrush('SPAWN_DYNAMIC_OBSTACLE')}
                  className={`flex items-center justify-center gap-1.5 py-1.5 px-2.5 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    activeBrush === 'SPAWN_DYNAMIC_OBSTACLE'
                      ? 'bg-orange-600 text-white shadow-sm'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Bot className="w-3.5 h-3.5" />
                  <span>Add 3D AGV</span>
                </button>

                <button
                  type="button"
                  onClick={() => setActiveBrush('DRAW_WALL')}
                  className={`flex items-center justify-center gap-1.5 py-1.5 px-2.5 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    activeBrush === 'DRAW_WALL'
                      ? 'bg-rose-600 text-white shadow-sm'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <SquarePen className="w-3.5 h-3.5" />
                  <span>Add 3D Rack</span>
                </button>

                <button
                  type="button"
                  onClick={() => setActiveBrush('ERASE_WALL')}
                  className={`flex items-center justify-center gap-1.5 py-1.5 px-2.5 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    activeBrush === 'ERASE_WALL'
                      ? 'bg-cyan-600 text-white shadow-sm'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Eraser className="w-3.5 h-3.5" />
                  <span>Clear Rack</span>
                </button>

                <button
                  type="button"
                  onClick={() => setActiveBrush('SET_HOME_DOCK')}
                  className={`col-span-2 flex items-center justify-center gap-1.5 py-1.5 px-2.5 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    activeBrush === 'SET_HOME_DOCK'
                      ? 'bg-emerald-600 text-white shadow-sm'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Home className="w-3.5 h-3.5" />
                  <span>Move Origin Charging Dock</span>
                </button>
              </div>
            </div>

            {/* 4. LiDAR, Costmap Inflation & DWA Calibration */}
            <div className="bg-[#111827] border border-[#1E293B] rounded-lg p-4 space-y-4">
              <div className="flex items-center gap-2">
                <Sliders className="w-4 h-4 text-cyan-400" />
                <h2 className="text-sm font-semibold text-slate-100">
                  DWA, Costmap & SLAM Parameters
                </h2>
              </div>

              <div>
                <div className="flex justify-between text-xs text-slate-300 mb-1">
                  <span>Patrolling 3D Warehouse AGVs</span>
                  <span className="font-mono tabular-nums text-orange-400">
                    {config.dynamicObstacleCount} AGVs
                  </span>
                </div>
                <input
                  type="range"
                  min={0}
                  max={6}
                  step={1}
                  value={config.dynamicObstacleCount}
                  onChange={(e) => {
                    const count = Number(e.target.value);
                    const nextCfg = { ...config, dynamicObstacleCount: count };
                    setConfig(nextCfg);
                    setDynamicObstacles(
                      spawnDynamicObstacles(grid, homeDock, count)
                    );
                  }}
                  className="w-full accent-orange-400 cursor-pointer"
                />
              </div>

              <div>
                <div className="flex justify-between text-xs text-slate-300 mb-1">
                  <span>Costmap Safety Inflation Radius</span>
                  <span className="font-mono tabular-nums text-indigo-400">
                    {config.safetyInflationRadius.toFixed(1)} cells
                  </span>
                </div>
                <input
                  type="range"
                  min={1.0}
                  max={3.5}
                  step={0.2}
                  value={config.safetyInflationRadius}
                  onChange={(e) =>
                    setConfig((c) => ({
                      ...c,
                      safetyInflationRadius: Number(e.target.value),
                    }))
                  }
                  className="w-full accent-indigo-400 cursor-pointer"
                />
              </div>

              <div>
                <div className="flex justify-between text-xs text-slate-300 mb-1">
                  <span>360° LiDAR Max Range</span>
                  <span className="font-mono tabular-nums text-cyan-400">
                    {config.lidarRange} cells
                  </span>
                </div>
                <input
                  type="range"
                  min={6}
                  max={14}
                  step={1}
                  value={config.lidarRange}
                  onChange={(e) =>
                    setConfig((c) => ({
                      ...c,
                      lidarRange: Number(e.target.value),
                    }))
                  }
                  className="w-full accent-cyan-400 cursor-pointer"
                />
              </div>

              <div>
                <div className="text-xs text-slate-300 mb-1.5">
                  Dead-End Bay / Room Throat Sealing
                </div>
                <div className="grid grid-cols-3 gap-1 p-1 bg-[#0B0F1A] rounded border border-[#1E293B]">
                  {(
                    ['CONSERVATIVE', 'STANDARD', 'AGGRESSIVE'] as const
                  ).map((mode) => (
                    <button
                      key={mode}
                      type="button"
                      onClick={() =>
                        setConfig((c) => ({ ...c, pruningAggression: mode }))
                      }
                      className={`py-1.5 px-2 rounded text-[11px] font-medium transition-colors whitespace-nowrap shrink-0 ${
                        config.pruningAggression === mode
                          ? 'bg-rose-600 text-white'
                          : 'text-slate-400 hover:text-slate-200'
                      }`}
                    >
                      {mode === 'CONSERVATIVE'
                        ? 'Doorway (3c)'
                        : mode === 'STANDARD'
                        ? 'Aisle (6c)'
                        : 'Wide Bay (9c)'}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          </aside>

          {/* Right Main Stage: Viewport Mode Switcher + Gazebo 3D & RViz 2D Split View */}
          <div className="flex-1 min-w-0 space-y-5 w-full">
            {/* Viewport Layout & Layer Toolbar */}
            <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 bg-[#111827] border border-[#1E293B] rounded-lg">
              {/* Viewport Split Switcher */}
              <div className="flex items-center gap-1 p-0.5 bg-[#090D16] border border-[#1E293B] rounded-md">
                <button
                  type="button"
                  onClick={() => setViewportMode('SPLIT_GAZEBO_RVIZ')}
                  className={`flex items-center gap-1.5 px-3 py-1 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    viewportMode === 'SPLIT_GAZEBO_RVIZ'
                      ? 'bg-cyan-600 text-white'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Columns2 className="w-3.5 h-3.5" />
                  <span>Split: Gazebo 3D + RViz 2D</span>
                </button>
                <button
                  type="button"
                  onClick={() => setViewportMode('GAZEBO_3D_ONLY')}
                  className={`flex items-center gap-1.5 px-3 py-1 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    viewportMode === 'GAZEBO_3D_ONLY'
                      ? 'bg-cyan-600 text-white'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <Box className="w-3.5 h-3.5" />
                  <span>Gazebo 3D Focus</span>
                </button>
                <button
                  type="button"
                  onClick={() => setViewportMode('RVIZ_2D_ONLY')}
                  className={`flex items-center gap-1.5 px-3 py-1 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 ${
                    viewportMode === 'RVIZ_2D_ONLY'
                      ? 'bg-cyan-600 text-white'
                      : 'text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <MapIcon className="w-3.5 h-3.5" />
                  <span>RViz 2D Focus</span>
                </button>
              </div>

              {/* Layer Toggles */}
              <div className="flex flex-wrap items-center gap-1.5">
                <Layers className="w-4 h-4 text-cyan-400 mr-1 hidden sm:inline" />
                {[
                  {
                    key: 'showLidarRays' as const,
                    label: '360° LiDAR',
                  },
                  {
                    key: 'showCostmapInflation' as const,
                    label: 'Inflation',
                  },
                  {
                    key: 'showDwaRollouts' as const,
                    label: 'DWA Arcs',
                  },
                  {
                    key: 'showDynamicObstacles' as const,
                    label: '3D AGVs',
                  },
                  {
                    key: 'showFrontiers' as const,
                    label: 'Frontiers',
                  },
                  {
                    key: 'showDeadEndTaboo' as const,
                    label: 'Locked Dead-Ends',
                  },
                  {
                    key: 'showPlannedPath' as const,
                    label: 'A* Path',
                  },
                ].map((item) => {
                  const active = layers[item.key];
                  return (
                    <button
                      key={item.key}
                      type="button"
                      onClick={() =>
                        setLayers((prev) => ({
                          ...prev,
                          [item.key]: !prev[item.key],
                        }))
                      }
                      className={`px-2.5 py-1 rounded text-xs font-medium transition-colors whitespace-nowrap shrink-0 border ${
                        active
                          ? 'bg-slate-800 text-cyan-300 border-cyan-500/50'
                          : 'bg-[#0B0F1A] text-slate-500 border-slate-800 hover:text-slate-300'
                      }`}
                    >
                      {item.label}
                    </button>
                  );
                })}
              </div>
            </div>

            {/* Synchronized Side-by-Side Split View: Gazebo 3D World + RViz2 2D SLAM Map */}
            <div
              className={
                viewportMode === 'SPLIT_GAZEBO_RVIZ'
                  ? 'grid grid-cols-1 2xl:grid-cols-2 gap-5 items-stretch'
                  : 'space-y-5'
              }
            >
              {(viewportMode === 'SPLIT_GAZEBO_RVIZ' ||
                viewportMode === 'GAZEBO_3D_ONLY') && (
                <Suspense
                  fallback={
                    <div className="w-full h-[502px] lg:h-[562px] bg-[#090D16] border border-[#1E293B] rounded-lg flex items-center justify-center text-xs font-mono text-slate-400">
                      Initializing 3D Gazebo Warehouse Viewport...
                    </div>
                  }
                >
                  <GazeboViewport3D
                    grid={grid}
                    robotPose={robotPose}
                    homeDock={homeDock}
                    lidarRays={lidarRays}
                    frontiers={frontiers}
                    selectedFrontier={selectedFrontier}
                    currentPath={currentPath}
                    deadEndRecords={deadEndRecords}
                    dynamicObstacles={dynamicObstacles}
                    dwaRollouts={dwaRollouts}
                    fsmState={fsmState}
                    layers={layers}
                    presetName={activePreset.name}
                  />
                </Suspense>
              )}

              {(viewportMode === 'SPLIT_GAZEBO_RVIZ' ||
                viewportMode === 'RVIZ_2D_ONLY') && (
                <RvizCanvas
                  grid={grid}
                  robotPose={robotPose}
                  homeDock={homeDock}
                  lidarRays={lidarRays}
                  frontiers={frontiers}
                  selectedFrontier={selectedFrontier}
                  currentPath={currentPath}
                  breadcrumbTrail={breadcrumbTrail}
                  deadEndRecords={deadEndRecords}
                  dynamicObstacles={dynamicObstacles}
                  dwaRollouts={dwaRollouts}
                  sectorClearance={sectorClearance}
                  highlightedDeadEndId={highlightedDeadEndId}
                  fsmState={fsmState}
                  layers={layers}
                  activeBrush={activeBrush}
                  onCellInteract={handleCellInteract}
                />
              )}
            </div>

            {/* Dead-End Taboo Memory Ledger & Live Mission Event Log */}
            <div
              id="dead-end-memory"
              className="grid grid-cols-1 xl:grid-cols-3 gap-6"
            >
              <div className="xl:col-span-2">
                <DeadEndLedger
                  records={deadEndRecords}
                  highlightedId={highlightedDeadEndId}
                  onSelectDeadEnd={setHighlightedDeadEndId}
                  totalLockedCells={metrics.lockedTabooCells}
                />
              </div>

              {/* Live Telemetry Event Log */}
              <div
                id="telemetry-log"
                className="bg-[#111827] border border-[#1E293B] rounded-lg overflow-hidden flex flex-col"
              >
                <div className="px-4 py-3 border-b border-[#1E293B] bg-[#0E1422] flex items-center justify-between">
                  <div>
                    <h3 className="text-sm font-semibold text-slate-100">
                      Gazebo & RViz2 Event Log
                    </h3>
                    <p className="text-xs text-slate-400 mt-0.5">
                      Real-time AGV evasions & dead-end bay locks
                    </p>
                  </div>
                  <ShieldAlert className="w-4 h-4 text-slate-400" />
                </div>

                <div className="p-3 space-y-2.5 max-h-72 overflow-y-auto text-xs">
                  {events.map((ev) => (
                    <div
                      key={ev.id}
                      className="p-2.5 rounded bg-[#0B0F1A] border border-[#1E293B]"
                    >
                      <div className="flex items-center justify-between gap-2 font-mono">
                        <span
                          className={
                            ev.type === 'DEAD_END_LOCKED'
                              ? 'text-rose-400 font-semibold'
                              : ev.type === 'OBSTACLE_AVOIDED'
                              ? 'text-orange-400 font-semibold'
                              : ev.type === 'RETURN_HOME' ||
                                ev.type === 'COMPLETE'
                              ? 'text-emerald-400 font-semibold'
                              : 'text-cyan-400 font-semibold'
                          }
                        >
                          {ev.message}
                        </span>
                        <span className="text-slate-500 shrink-0 tabular-nums">
                          T+{ev.tick}
                        </span>
                      </div>
                      <p className="text-slate-400 mt-1 leading-relaxed">
                        {ev.detail}
                      </p>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Bottom Section: Mathematical Pipeline & ROS 2 + Gazebo SDF Export */}
        <section id="algorithm-pipeline" className="pt-4">
          <AlgorithmInspector
            fsmState={fsmState}
            config={config}
            coveragePct={metrics.coveragePct}
            frontierCount={frontiers.length}
            lockedCellsCount={metrics.lockedTabooCells}
            deadEndZoneCount={deadEndRecords.length}
          />
        </section>
      </main>

      {/* Quiet Footer */}
      <footer className="mt-12 border-t border-[#1E293B] py-4 px-6 text-xs text-slate-500 flex flex-wrap items-center justify-between gap-4">
        <span>
          AutoSLAM Gazebo & RViz2 Workstation · 3D Industrial Warehouse Simulator & Zero-Prior-Map Dead-End Pruning Explorer
        </span>
        <span className="font-mono">
          Three.js WebGL Gazebo Stage + RViz2 Occupancy Grid · Inflation-Aware A* & DWA Local Planner
        </span>
      </footer>
    </div>
  );
}
