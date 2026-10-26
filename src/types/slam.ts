export type GroundTruthType = 'EMPTY' | 'WALL';

export type BeliefState = 'UNKNOWN' | 'FREE' | 'OCCUPIED';

export type FSMState =
  | 'IDLE'
  | 'EXPLORING_FRONTIERS'
  | 'AVOIDING_OBSTACLE'
  | 'PRUNING_DEAD_END'
  | 'RETURNING_HOME'
  | 'DOCKED_COMPLETE'
  | 'NO_REACHABLE_FRONTIERS';

export interface GridCoord {
  x: number;
  y: number;
}

export interface CellState {
  x: number;
  y: number;
  groundTruth: GroundTruthType;
  belief: BeliefState;
  logOdds: number; // Continuous [-5.0, +5.0]
  inflationCost: number; // [0.0, 1.0] repulsive safety costmap buffer near walls
  isFrontier: boolean;
  isDeadEndTaboo: boolean; // Permanently locked dead-end cell (never re-scanned or targeted)
  deadEndClusterId: number | null;
  visitCount: number;
  lastScannedTick: number | null;
}

export interface LidarRay {
  angle: number;
  endX: number;
  endY: number;
  hitOccupied: boolean;
  hitDynamicObstacle?: boolean;
  distance: number;
}

export interface DynamicObstacle {
  id: number;
  label: string;
  x: number;
  y: number;
  vx: number;
  vy: number;
  detectedByLidar: boolean;
  lastSeenTick: number;
}

export interface DWARolloutArc {
  id: number;
  v: number;
  omega: number;
  points: Array<{ x: number; y: number }>;
  score: number;
  isCollision: boolean;
  isSelected: boolean;
}

export interface SectorClearance {
  front: number;
  frontLeft: number;
  frontRight: number;
  rear: number;
  minDistance: number;
  status: 'NOMINAL' | 'DWA_EVASION' | 'CRITICAL_BACKOFF';
}

export interface FrontierCluster {
  id: number;
  cells: GridCoord[];
  centroid: GridCoord;
  vantagePoint: GridCoord; // Nearest reachable non-taboo FREE cell
  infoGain: number; // Number of adjacent UNKNOWN cells in sensor radius
  pathDistance: number; // A* path length from current robot pose
  utilityScore: number; // Combined alpha*InfoGain - beta*Distance
}

export interface DeadEndRecord {
  id: number;
  label: string;
  classification: 'Cul-de-Sac Corridor' | 'Enclosed Alcove' | 'Blind Bay Pocket' | 'Deep Branch Trap';
  throatCoord: GridCoord;
  tipCoord: GridCoord;
  cells: GridCoord[];
  cellCount: number;
  lockedAtTick: number;
  savedRevisitCost: number; // Estimated redundant scan steps prevented
}

export interface TelemetryEvent {
  id: string;
  tick: number;
  type:
    | 'SCAN'
    | 'FRONTIER'
    | 'DEAD_END_LOCKED'
    | 'OBSTACLE_AVOIDED'
    | 'RETURN_HOME'
    | 'COMPLETE'
    | 'INFO';
  message: string;
  detail: string;
}

export interface SimulationConfig {
  lidarRange: number; // in grid cells (e.g. 8)
  lidarRayCount: number; // e.g. 90 rays for 360 deg
  speedHz: number; // simulation ticks per second (1 - 18)
  infoGainWeight: number; // alpha (e.g. 1.8)
  distanceWeight: number; // beta (e.g. 1.1)
  pruningAggression: 'CONSERVATIVE' | 'STANDARD' | 'AGGRESSIVE'; // max throat width for pocket sealing
  safetyInflationRadius: number; // costmap wall clearance radius in cells (e.g. 2.2)
  dynamicObstacleCount: number; // number of moving dynamic obstacles (0 - 6)
}

export interface LayerVisibility {
  showGroundTruthGhost: boolean;
  showLidarRays: boolean;
  showCostmapInflation: boolean;
  showDwaRollouts: boolean;
  showDynamicObstacles: boolean;
  showFrontiers: boolean;
  showDeadEndTaboo: boolean;
  showPlannedPath: boolean;
  showBreadcrumbTrail: boolean;
  showGridLines: boolean;
}

export type MapBrushTool =
  | 'INSPECT'
  | 'DRAW_WALL'
  | 'ERASE_WALL'
  | 'SPAWN_DYNAMIC_OBSTACLE'
  | 'SET_HOME_DOCK';

export interface MapPreset {
  id: string;
  name: string;
  subtitle: string;
  description: string;
  width: number;
  height: number;
  homeDock: GridCoord;
  asciiLayout: string[];
}
