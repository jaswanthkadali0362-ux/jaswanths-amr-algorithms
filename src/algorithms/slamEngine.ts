import {
  CellState,
  DeadEndRecord,
  DWARolloutArc,
  DynamicObstacle,
  FrontierCluster,
  GridCoord,
  LidarRay,
  SectorClearance,
  SimulationConfig,
} from '../types/slam';

const DIRS_4: GridCoord[] = [
  { x: 1, y: 0 },
  { x: -1, y: 0 },
  { x: 0, y: 1 },
  { x: 0, y: -1 },
];

const DIRS_8: GridCoord[] = [
  { x: 1, y: 0 },
  { x: -1, y: 0 },
  { x: 0, y: 1 },
  { x: 0, y: -1 },
  { x: 1, y: 1 },
  { x: 1, y: -1 },
  { x: -1, y: 1 },
  { x: -1, y: -1 },
];

export function inBounds(grid: CellState[][], x: number, y: number): boolean {
  return y >= 0 && y < grid.length && x >= 0 && x < grid[0].length;
}

/**
 * 0. Spawn & Step Moving Dynamic Obstacles (Patrolling AGVs / Workers)
 * Spawns dynamic obstacles in wide navigable corridors (so they patrol main aisles)
 */
export function spawnDynamicObstacles(
  grid: CellState[][],
  homeDock: GridCoord,
  count: number
): DynamicObstacle[] {
  if (count <= 0) return [];
  const height = grid.length;
  const width = grid[0].length;
  const candidates: GridCoord[] = [];

  for (let y = 2; y < height - 2; y++) {
    for (let x = 2; x < width - 2; x++) {
      if (grid[y][x].groundTruth !== 'EMPTY') continue;
      if (Math.hypot(x - homeDock.x, y - homeDock.y) < 7) continue;

      // Prefer wide corridor cells (all 8 neighbors empty) so patrol obstacles have room to pass
      let emptyNeighbors = 0;
      for (const d of DIRS_8) {
        if (grid[y + d.y][x + d.x].groundTruth === 'EMPTY') {
          emptyNeighbors++;
        }
      }
      if (emptyNeighbors >= 6) {
        candidates.push({ x, y });
      }
    }
  }

  const obstacles: DynamicObstacle[] = [];
  const step = Math.max(1, Math.floor(candidates.length / (count + 1)));

  for (let i = 0; i < count && i * step < candidates.length; i++) {
    const pt = candidates[(i + 1) * step - 1] || candidates[i];
    if (!pt) continue;
    const dir = DIRS_4[i % DIRS_4.length];
    obstacles.push({
      id: i + 1,
      label: `AGV-${String(i + 1).padStart(2, '0')}`,
      x: pt.x,
      y: pt.y,
      vx: dir.x,
      vy: dir.y,
      detectedByLidar: false,
      lastSeenTick: -1,
    });
  }

  return obstacles;
}

export function stepDynamicObstacles(
  grid: CellState[][],
  obstacles: DynamicObstacle[],
  robotPose: GridCoord,
  homeDock: GridCoord
): DynamicObstacle[] {
  return obstacles.map((obs) => {
    const tryMove = (vx: number, vy: number): boolean => {
      const nx = obs.x + vx;
      const ny = obs.y + vy;
      if (!inBounds(grid, nx, ny)) return false;
      if (grid[ny][nx].groundTruth === 'WALL') return false;
      if (nx === robotPose.x && ny === robotPose.y) return false;
      if (nx === homeDock.x && ny === homeDock.y) return false;
      // Avoid stacking two dynamic obstacles on the same cell
      if (obstacles.some((o) => o.id !== obs.id && o.x === nx && o.y === ny)) {
        return false;
      }
      return true;
    };

    // Continue along current velocity vector if clear
    if (tryMove(obs.vx, obs.vy)) {
      return {
        ...obs,
        x: obs.x + obs.vx,
        y: obs.y + obs.vy,
      };
    }

    // Otherwise reverse or pick an open cardinal direction
    if (tryMove(-obs.vx, -obs.vy)) {
      return {
        ...obs,
        vx: -obs.vx,
        vy: -obs.vy,
        x: obs.x - obs.vx,
        y: obs.y - obs.vy,
      };
    }

    for (const d of DIRS_4) {
      if (tryMove(d.x, d.y)) {
        return {
          ...obs,
          vx: d.x,
          vy: d.y,
          x: obs.x + d.x,
          y: obs.y + d.y,
        };
      }
    }

    return obs;
  });
}

/**
 * 1. 360-Degree 2D LiDAR Raycasting with Dynamic Obstacle Separation & Sector Shield
 * - Updates static wall beliefs in log-odds occupancy grid.
 * - Detects moving dynamic obstacles without permanently corrupting the static wall map,
 *   so moving obstacles NEVER trigger false dead-end locks!
 */
export function performLidarScan(
  grid: CellState[][],
  pose: { x: number; y: number; theta?: number },
  config: SimulationConfig,
  currentTick: number,
  dynamicObstacles: DynamicObstacle[] = []
): {
  rays: LidarRay[];
  newlyRevealedCount: number;
  sectorClearance: SectorClearance;
} {
  const rays: LidarRay[] = [];
  let newlyRevealedCount = 0;

  // Reset lidar detection flag for this scan
  for (const obs of dynamicObstacles) {
    obs.detectedByLidar = false;
  }

  if (inBounds(grid, pose.x, pose.y)) {
    const startCell = grid[pose.y][pose.x];
    if (startCell.belief === 'UNKNOWN') newlyRevealedCount++;
    startCell.belief = 'FREE';
    startCell.logOdds = Math.max(-5.0, startCell.logOdds - 1.5);
    startCell.lastScannedTick = currentTick;
  }

  const numRays = config.lidarRayCount;
  const maxRange = config.lidarRange;

  for (let i = 0; i < numRays; i++) {
    const angle = (i * 2 * Math.PI) / numRays;
    const targetX = pose.x + Math.cos(angle) * maxRange;
    const targetY = pose.y + Math.sin(angle) * maxRange;

    const steps = Math.ceil(maxRange * 3.5);
    let endX = pose.x;
    let endY = pose.y;
    let hitOccupied = false;
    let hitDynamicObstacle = false;

    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      const rx = pose.x + (targetX - pose.x) * t;
      const ry = pose.y + (targetY - pose.y) * t;
      const gx = Math.round(rx);
      const gy = Math.round(ry);

      if (!inBounds(grid, gx, gy)) {
        hitOccupied = true;
        break;
      }

      // Check if beam intersects a moving dynamic obstacle first
      const hitDyn = dynamicObstacles.find((o) => o.x === gx && o.y === gy);
      if (hitDyn && !(gx === pose.x && gy === pose.y)) {
        hitDyn.detectedByLidar = true;
        hitDyn.lastSeenTick = currentTick;
        // Mark the underlying floor cell as FREE (since a moving obstacle is standing on navigable floor!)
        const cell = grid[gy][gx];
        if (cell.groundTruth === 'EMPTY') {
          if (cell.belief === 'UNKNOWN') newlyRevealedCount++;
          cell.belief = 'FREE';
          cell.logOdds = Math.max(-5.0, cell.logOdds - 1.0);
        }
        endX = gx;
        endY = gy;
        hitOccupied = true;
        hitDynamicObstacle = true;
        break;
      }

      const cell = grid[gy][gx];
      cell.lastScannedTick = currentTick;

      if (cell.groundTruth === 'WALL') {
        if (cell.belief === 'UNKNOWN') newlyRevealedCount++;
        cell.belief = 'OCCUPIED';
        cell.logOdds = Math.min(5.0, cell.logOdds + 2.2);
        endX = gx;
        endY = gy;
        hitOccupied = true;
        break;
      } else {
        if (cell.belief === 'UNKNOWN') newlyRevealedCount++;
        cell.belief = 'FREE';
        cell.logOdds = Math.max(-5.0, cell.logOdds - 1.2);
        endX = rx;
        endY = ry;
      }
    }

    const dx = endX - pose.x;
    const dy = endY - pose.y;
    rays.push({
      angle,
      endX,
      endY,
      hitOccupied,
      hitDynamicObstacle,
      distance: Math.hypot(dx, dy),
    });
  }

  // Update Costmap Inflation around all discovered OCCUPIED walls
  updateCostmapInflation(grid, config.safetyInflationRadius);

  // Compute 5-Sector 360° Proximity Shield relative to robot heading
  const heading = pose.theta ?? 0;
  let front = maxRange;
  let frontLeft = maxRange;
  let frontRight = maxRange;
  let rear = maxRange;

  for (const ray of rays) {
    const relAngle = Math.atan2(
      Math.sin(ray.angle - heading),
      Math.cos(ray.angle - heading)
    );
    const deg = (relAngle * 180) / Math.PI;
    if (deg >= -30 && deg <= 30) {
      front = Math.min(front, ray.distance);
    } else if (deg > 30 && deg <= 105) {
      frontLeft = Math.min(frontLeft, ray.distance);
    } else if (deg < -30 && deg >= -105) {
      frontRight = Math.min(frontRight, ray.distance);
    } else {
      rear = Math.min(rear, ray.distance);
    }
  }

  const minDistance = Math.min(front, frontLeft, frontRight, rear);
  const nearDynamic = dynamicObstacles.some(
    (o) =>
      o.detectedByLidar && Math.hypot(o.x - pose.x, o.y - pose.y) <= 2.8
  );

  const status: SectorClearance['status'] =
    minDistance <= 1.15
      ? 'CRITICAL_BACKOFF'
      : nearDynamic || front <= 2.1
      ? 'DWA_EVASION'
      : 'NOMINAL';

  return {
    rays,
    newlyRevealedCount,
    sectorClearance: {
      front,
      frontLeft,
      frontRight,
      rear,
      minDistance,
      status,
    },
  };
}

/**
 * 1B. Exponential Safety Costmap Inflation
 * Computes repulsive potential field around discovered OCCUPIED wall cells
 */
export function updateCostmapInflation(
  grid: CellState[][],
  safetyRadius: number
): void {
  const height = grid.length;
  const width = grid[0].length;
  const rCeil = Math.ceil(safetyRadius);

  // Reset inflation costs
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      grid[y][x].inflationCost = grid[y][x].belief === 'OCCUPIED' ? 1.0 : 0.0;
    }
  }

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      if (grid[y][x].belief !== 'OCCUPIED') continue;

      for (let dy = -rCeil; dy <= rCeil; dy++) {
        for (let dx = -rCeil; dx <= rCeil; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (!inBounds(grid, nx, ny)) continue;
          if (grid[ny][nx].belief !== 'FREE') continue;

          const dist = Math.hypot(dx, dy);
          if (dist <= safetyRadius && dist > 0) {
            // Smooth exponential decay from wall (dist=1 -> ~0.55, dist=2.2 -> ~0.12)
            const cost = Math.exp(-0.75 * dist);
            if (cost > grid[ny][nx].inflationCost) {
              grid[ny][nx].inflationCost = cost;
            }
          }
        }
      }
    }
  }
}

function hasUnknownNeighbor8(grid: CellState[][], x: number, y: number): boolean {
  for (const d of DIRS_8) {
    const nx = x + d.x;
    const ny = y + d.y;
    if (inBounds(grid, nx, ny) && grid[ny][nx].belief === 'UNKNOWN') {
      return true;
    }
  }
  return false;
}

/**
 * 2. Topological & Morphological Dead-End Pruning Engine
 * Identifies both 1-cell cul-de-sacs AND multi-cell wide blind bays/corridors
 * that have been 100% scanned (zero UNKNOWN cells remaining inside the pocket)
 * and permanently locks them as Taboo (`isDeadEndTaboo = true`).
 */
export function detectAndLockDeadEnds(
  grid: CellState[][],
  homeDock: GridCoord,
  existingRecords: DeadEndRecord[],
  config: SimulationConfig,
  currentTick: number
): { newRecords: DeadEndRecord[]; newlyLockedCells: number } {
  const height = grid.length;
  const width = grid[0].length;
  const newlyLockedCoords: GridCoord[] = [];
  const NewlyLockedSet = new Set<string>();
  const key = (x: number, y: number) => `${x},${y}`;

  // Pass A: Iterative Degree-1 Cellular Peeling for narrow 1-cell corridors & alcoves
  let changed = true;
  while (changed) {
    changed = false;
    for (let y = 1; y < height - 1; y++) {
      for (let x = 1; x < width - 1; x++) {
        const cell = grid[y][x];
        if (cell.belief !== 'FREE' || cell.isDeadEndTaboo) continue;
        if (x === homeDock.x && y === homeDock.y) continue;
        if (hasUnknownNeighbor8(grid, x, y)) continue;

        let openNeighbors = 0;
        for (const d of DIRS_4) {
          const nx = x + d.x;
          const ny = y + d.y;
          if (!inBounds(grid, nx, ny)) continue;
          const nCell = grid[ny][nx];
          if (nCell.belief !== 'OCCUPIED' && !nCell.isDeadEndTaboo) {
            openNeighbors++;
          }
        }

        if (openNeighbors <= 1) {
          cell.isDeadEndTaboo = true;
          newlyLockedCoords.push({ x, y });
          NewlyLockedSet.add(key(x, y));
          changed = true;
        }
      }
    }
  }

  // Pass B: Topological Throat Cut for multi-cell wide corridors, rooms, and warehouse bays
  const maxThroatWidth =
    config.pruningAggression === 'CONSERVATIVE'
      ? 3
      : config.pruningAggression === 'STANDARD'
      ? 6
      : 9;

  const candidateThroats: GridCoord[][] = [];

  for (let y = 1; y < height - 1; y++) {
    let x = 1;
    while (x < width - 1) {
      if (grid[y][x].belief === 'FREE' && !grid[y][x].isDeadEndTaboo) {
        const startX = x;
        const segment: GridCoord[] = [];
        while (
          x < width - 1 &&
          grid[y][x].belief === 'FREE' &&
          !grid[y][x].isDeadEndTaboo
        ) {
          segment.push({ x, y });
          x++;
        }
        const leftBlocked =
          grid[y][startX - 1].belief === 'OCCUPIED' ||
          grid[y][startX - 1].isDeadEndTaboo;
        const rightBlocked =
          grid[y][x].belief === 'OCCUPIED' || grid[y][x].isDeadEndTaboo;

        if (
          leftBlocked &&
          rightBlocked &&
          segment.length >= 1 &&
          segment.length <= maxThroatWidth
        ) {
          candidateThroats.push(segment);
        }
      } else {
        x++;
      }
    }
  }

  for (let x = 1; x < width - 1; x++) {
    let y = 1;
    while (y < height - 1) {
      if (grid[y][x].belief === 'FREE' && !grid[y][x].isDeadEndTaboo) {
        const startY = y;
        const segment: GridCoord[] = [];
        while (
          y < height - 1 &&
          grid[y][x].belief === 'FREE' &&
          !grid[y][x].isDeadEndTaboo
        ) {
          segment.push({ x, y });
          y++;
        }
        const topBlocked =
          grid[startY - 1][x].belief === 'OCCUPIED' ||
          grid[startY - 1][x].isDeadEndTaboo;
        const bottomBlocked =
          grid[y][x].belief === 'OCCUPIED' || grid[y][x].isDeadEndTaboo;

        if (
          topBlocked &&
          bottomBlocked &&
          segment.length >= 1 &&
          segment.length <= maxThroatWidth
        ) {
          candidateThroats.push(segment);
        }
      } else {
        y++;
      }
    }
  }

  for (const throat of candidateThroats) {
    let throatValid = true;
    const throatKeys = new Set<string>();
    for (const c of throat) {
      if (
        grid[c.y][c.x].isDeadEndTaboo ||
        (c.x === homeDock.x && c.y === homeDock.y) ||
        hasUnknownNeighbor8(grid, c.x, c.y)
      ) {
        throatValid = false;
        break;
      }
      throatKeys.add(key(c.x, c.y));
    }
    if (!throatValid) continue;

    const seeds: GridCoord[] = [];
    for (const c of throat) {
      for (const d of DIRS_4) {
        const nx = c.x + d.x;
        const ny = c.y + d.y;
        if (
          inBounds(grid, nx, ny) &&
          grid[ny][nx].belief === 'FREE' &&
          !grid[ny][nx].isDeadEndTaboo &&
          !throatKeys.has(key(nx, ny))
        ) {
          seeds.push({ x: nx, y: ny });
        }
      }
    }

    const visitedGlobal = new Set<string>();
    for (const seed of seeds) {
      const sk = key(seed.x, seed.y);
      if (visitedGlobal.has(sk) || grid[seed.y][seed.x].isDeadEndTaboo) continue;

      const queue: GridCoord[] = [seed];
      const component: GridCoord[] = [];
      const compVisited = new Set<string>([sk]);
      let touchesUnknown = false;
      let containsHome = false;
      let exceededMaxPocketSize = false;

      while (queue.length > 0) {
        const curr = queue.shift()!;
        component.push(curr);
        visitedGlobal.add(key(curr.x, curr.y));

        if (curr.x === homeDock.x && curr.y === homeDock.y) {
          containsHome = true;
        }
        if (hasUnknownNeighbor8(grid, curr.x, curr.y)) {
          touchesUnknown = true;
        }
        if (component.length > 160) {
          exceededMaxPocketSize = true;
          break;
        }

        for (const d of DIRS_4) {
          const nx = curr.x + d.x;
          const ny = curr.y + d.y;
          const nk = key(nx, ny);
          if (!inBounds(grid, nx, ny) || compVisited.has(nk) || throatKeys.has(nk)) {
            continue;
          }
          const nCell = grid[ny][nx];
          if (nCell.belief === 'UNKNOWN') {
            touchesUnknown = true;
          } else if (nCell.belief === 'FREE' && !nCell.isDeadEndTaboo) {
            compVisited.add(nk);
            queue.push({ x: nx, y: ny });
          }
        }
      }

      if (!touchesUnknown && !containsHome && !exceededMaxPocketSize && component.length > 0) {
        const toLock = [...component, ...throat];
        for (const pt of toLock) {
          const cell = grid[pt.y][pt.x];
          if (!cell.isDeadEndTaboo) {
            cell.isDeadEndTaboo = true;
            const pk = key(pt.x, pt.y);
            if (!NewlyLockedSet.has(pk)) {
              NewlyLockedSet.add(pk);
              newlyLockedCoords.push(pt);
            }
          }
        }
      }
    }
  }

  const newRecords: DeadEndRecord[] = [];
  let nextClusterId =
    existingRecords.reduce((max, r) => Math.max(max, r.id), 0) + 1;

  const unassignedVisited = new Set<string>();
  for (const pt of newlyLockedCoords) {
    const cell = grid[pt.y][pt.x];
    const pk = key(pt.x, pt.y);
    if (cell.deadEndClusterId !== null || unassignedVisited.has(pk)) continue;

    const clusterCells: GridCoord[] = [];
    const q: GridCoord[] = [pt];
    unassignedVisited.add(pk);
    let adjacentExistingClusterId: number | null = null;

    while (q.length > 0) {
      const curr = q.shift()!;
      clusterCells.push(curr);

      for (const d of DIRS_8) {
        const nx = curr.x + d.x;
        const ny = curr.y + d.y;
        if (!inBounds(grid, nx, ny)) continue;
        const nCell = grid[ny][nx];
        if (nCell.isDeadEndTaboo) {
          if (nCell.deadEndClusterId !== null) {
            adjacentExistingClusterId = nCell.deadEndClusterId;
          } else {
            const nk = key(nx, ny);
            if (!unassignedVisited.has(nk)) {
              unassignedVisited.add(nk);
              q.push({ x: nx, y: ny });
            }
          }
        }
      }
    }

    if (adjacentExistingClusterId !== null) {
      const existingRec = existingRecords.find(
        (r) => r.id === adjacentExistingClusterId
      );
      if (existingRec) {
        for (const c of clusterCells) {
          grid[c.y][c.x].deadEndClusterId = existingRec.id;
          existingRec.cells.push(c);
        }
        existingRec.cellCount = existingRec.cells.length;
        existingRec.throatCoord = findThroatCoord(grid, existingRec.cells);
        existingRec.savedRevisitCost = Math.round(existingRec.cellCount * 2.4);
        existingRec.classification = classifyDeadEnd(existingRec.cellCount);
        continue;
      }
    }

    const clusterId = nextClusterId++;
    for (const c of clusterCells) {
      grid[c.y][c.x].deadEndClusterId = clusterId;
    }

    const throatCoord = findThroatCoord(grid, clusterCells);
    const tipCoord = findTipCoord(throatCoord, clusterCells);
    const classification = classifyDeadEnd(clusterCells.length);

    const record: DeadEndRecord = {
      id: clusterId,
      label: `Dead-End Zone #${String(clusterId).padStart(2, '0')}`,
      classification,
      throatCoord,
      tipCoord,
      cells: clusterCells,
      cellCount: clusterCells.length,
      lockedAtTick: currentTick,
      savedRevisitCost: Math.round(clusterCells.length * 2.4),
    };
    existingRecords.push(record);
    newRecords.push(record);
  }

  return { newRecords, newlyLockedCells: newlyLockedCoords.length };
}

function classifyDeadEnd(
  cellCount: number
): DeadEndRecord['classification'] {
  if (cellCount <= 5) return 'Enclosed Alcove';
  if (cellCount <= 16) return 'Cul-de-Sac Corridor';
  if (cellCount <= 35) return 'Blind Bay Pocket';
  return 'Deep Branch Trap';
}

function findThroatCoord(grid: CellState[][], cells: GridCoord[]): GridCoord {
  for (const c of cells) {
    for (const d of DIRS_4) {
      const nx = c.x + d.x;
      const ny = c.y + d.y;
      if (
        inBounds(grid, nx, ny) &&
        grid[ny][nx].belief === 'FREE' &&
        !grid[ny][nx].isDeadEndTaboo
      ) {
        return c;
      }
    }
  }
  return cells[cells.length - 1] || { x: 0, y: 0 };
}

function findTipCoord(throat: GridCoord, cells: GridCoord[]): GridCoord {
  let best = cells[0] || throat;
  let maxDist = -1;
  for (const c of cells) {
    const d = Math.hypot(c.x - throat.x, c.y - throat.y);
    if (d > maxDist) {
      maxDist = d;
      best = c;
    }
  }
  return best;
}

/**
 * 3. Frontier Extraction & Clustering
 */
export function extractFrontiers(
  grid: CellState[][],
  robotPose: GridCoord,
  config: SimulationConfig
): FrontierCluster[] {
  const height = grid.length;
  const width = grid[0].length;

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      grid[y][x].isFrontier = false;
    }
  }

  const frontierCoords: GridCoord[] = [];
  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      const cell = grid[y][x];
      if (cell.belief !== 'FREE' || cell.isDeadEndTaboo) continue;

      let touchesUnknown = false;
      for (const d of DIRS_4) {
        const nx = x + d.x;
        const ny = y + d.y;
        if (inBounds(grid, nx, ny) && grid[ny][nx].belief === 'UNKNOWN') {
          touchesUnknown = true;
          break;
        }
      }

      if (touchesUnknown) {
        cell.isFrontier = true;
        frontierCoords.push({ x, y });
      }
    }
  }

  const visited = new Set<string>();
  const key = (x: number, y: number) => `${x},${y}`;
  const clusters: FrontierCluster[] = [];
  let clusterId = 1;

  for (const start of frontierCoords) {
    const sk = key(start.x, start.y);
    if (visited.has(sk)) continue;

    const cells: GridCoord[] = [];
    const q: GridCoord[] = [start];
    visited.add(sk);

    while (q.length > 0) {
      const curr = q.shift()!;
      cells.push(curr);

      for (const d of DIRS_8) {
        const nx = curr.x + d.x;
        const ny = curr.y + d.y;
        const nk = key(nx, ny);
        if (
          inBounds(grid, nx, ny) &&
          !visited.has(nk) &&
          grid[ny][nx].isFrontier
        ) {
          visited.add(nk);
          q.push({ x: nx, y: ny });
        }
      }
    }

    const sumX = cells.reduce((acc, c) => acc + c.x, 0);
    const sumY = cells.reduce((acc, c) => acc + c.y, 0);
    const cx = sumX / cells.length;
    const cy = sumY / cells.length;

    let vantagePoint = cells[0];
    let bestDistToCentroid = Infinity;
    for (const c of cells) {
      const inflationPenalty = grid[c.y][c.x].inflationCost * 2.0;
      const d = Math.hypot(c.x - cx, c.y - cy) + inflationPenalty;
      if (d < bestDistToCentroid) {
        bestDistToCentroid = d;
        vantagePoint = c;
      }
    }

    let infoGain = 0;
    const r = 4;
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        const nx = vantagePoint.x + dx;
        const ny = vantagePoint.y + dy;
        if (
          inBounds(grid, nx, ny) &&
          dx * dx + dy * dy <= r * r &&
          grid[ny][nx].belief === 'UNKNOWN'
        ) {
          infoGain++;
        }
      }
    }

    const euclideanDist = Math.hypot(
      vantagePoint.x - robotPose.x,
      vantagePoint.y - robotPose.y
    );

    clusters.push({
      id: clusterId++,
      cells,
      centroid: { x: Math.round(cx), y: Math.round(cy) },
      vantagePoint,
      infoGain: infoGain + cells.length * 2,
      pathDistance: euclideanDist,
      utilityScore:
        config.infoGainWeight * (infoGain + cells.length * 2) -
        config.distanceWeight * euclideanDist,
    });
  }

  clusters.sort((a, b) => b.utilityScore - a.utilityScore);
  return clusters;
}

/**
 * 4. Inflation & Dynamic-Obstacle-Aware A* Path Planner
 * Incorporates continuous wall `inflationCost` and repulsive cost around detected dynamic obstacles.
 */
export function findAStarPath(
  grid: CellState[][],
  start: GridCoord,
  goal: GridCoord,
  allowTabooExitOnly = true,
  dynamicObstacles: DynamicObstacle[] = []
): GridCoord[] | null {
  if (start.x === goal.x && start.y === goal.y) return [start];

  const key = (x: number, y: number) => `${x},${y}`;
  const openSet: GridCoord[] = [start];
  const openKeys = new Set<string>([key(start.x, start.y)]);
  const closedKeys = new Set<string>();

  const gScore = new Map<string, number>();
  const fScore = new Map<string, number>();
  const cameFrom = new Map<string, GridCoord>();

  const startKey = key(start.x, start.y);
  gScore.set(startKey, 0);
  fScore.set(startKey, Math.hypot(goal.x - start.x, goal.y - start.y));

  const startIsTaboo = grid[start.y]?.[start.x]?.isDeadEndTaboo ?? false;

  while (openSet.length > 0) {
    let bestIdx = 0;
    let bestF = fScore.get(key(openSet[0].x, openSet[0].y)) ?? Infinity;
    for (let i = 1; i < openSet.length; i++) {
      const f = fScore.get(key(openSet[i].x, openSet[i].y)) ?? Infinity;
      if (f < bestF) {
        bestF = f;
        bestIdx = i;
      }
    }

    const current = openSet.splice(bestIdx, 1)[0];
    const ck = key(current.x, current.y);
    openKeys.delete(ck);

    if (current.x === goal.x && current.y === goal.y) {
      const path: GridCoord[] = [current];
      let currKey = ck;
      while (cameFrom.has(currKey)) {
        const prev = cameFrom.get(currKey)!;
        path.push(prev);
        currKey = key(prev.x, prev.y);
      }
      path.reverse();
      return path;
    }

    closedKeys.add(ck);
    const currentIsTaboo = grid[current.y][current.x].isDeadEndTaboo;

    for (const d of DIRS_8) {
      const nx = current.x + d.x;
      const ny = current.y + d.y;
      if (!inBounds(grid, nx, ny)) continue;

      const nk = key(nx, ny);
      if (closedKeys.has(nk)) continue;

      const nCell = grid[ny][nx];
      if (nCell.belief !== 'FREE') continue;

      // Never step directly onto a moving dynamic obstacle
      const directHitDynamic = dynamicObstacles.some(
        (o) => o.x === nx && o.y === ny && !(nx === goal.x && ny === goal.y)
      );
      if (directHitDynamic) continue;

      // Prevent diagonal corner-cutting through walls
      if (d.x !== 0 && d.y !== 0) {
        const side1 = grid[current.y][nx];
        const side2 = grid[ny][current.x];
        if (side1.belief !== 'FREE' || side2.belief !== 'FREE') continue;
      }

      if (nCell.isDeadEndTaboo) {
        if (!allowTabooExitOnly || (!startIsTaboo && !currentIsTaboo)) {
          continue;
        }
      }

      const stepCost = d.x !== 0 && d.y !== 0 ? Math.SQRT2 : 1.0;
      const inflationPenalty = nCell.inflationCost * 2.2;
      const tabooPenalty = nCell.isDeadEndTaboo ? 60.0 : 0;
      const revisitPenalty = Math.min(1.2, nCell.visitCount * 0.08);

      // Repulsive buffer around detected moving obstacles
      let dynamicProximityCost = 0;
      for (const obs of dynamicObstacles) {
        const dObs = Math.hypot(nx - obs.x, ny - obs.y);
        if (dObs <= 2.2) {
          dynamicProximityCost += (2.5 - dObs) * 4.5;
        }
      }

      const tentativeG =
        (gScore.get(ck) ?? Infinity) +
        stepCost +
        inflationPenalty +
        tabooPenalty +
        revisitPenalty +
        dynamicProximityCost;

      if (tentativeG < (gScore.get(nk) ?? Infinity)) {
        cameFrom.set(nk, current);
        gScore.set(nk, tentativeG);
        const h = Math.hypot(goal.x - nx, goal.y - ny);
        fScore.set(nk, tentativeG + h);
        if (!openKeys.has(nk)) {
          openSet.push({ x: nx, y: ny });
          openKeys.add(nk);
        }
      }
    }
  }

  return null;
}

/**
 * 5. Dynamic Window Approach (DWA) Local Planner & Reactive Obstacle Avoidance
 * Generates candidate (v, omega) velocity rollout arcs, scores them against the
 * A* lookahead waypoint, static wall inflation costmap, and moving dynamic obstacles,
 * and selects the safest collision-free local step.
 */
export function computeDwaRolloutsAndReactiveStep(
  grid: CellState[][],
  pose: { x: number; y: number; theta: number },
  plannedPath: GridCoord[],
  dynamicObstacles: DynamicObstacle[]
): {
  rollouts: DWARolloutArc[];
  nextStep: GridCoord;
  evadingObstacle: boolean;
} {
  const lookahead =
    plannedPath[Math.min(3, plannedPath.length - 1)] ||
    plannedPath[1] || { x: pose.x, y: pose.y };

  const desiredHeading = Math.atan2(
    lookahead.y - pose.y,
    lookahead.x - pose.x
  );

  // Generate 11 DWA candidate rollout arcs spanning [-1.25 rad, +1.25 rad] around desired heading
  const omegaSamples = [
    -1.25, -1.0, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0, 1.25,
  ];
  const rollouts: DWARolloutArc[] = [];
  let bestArcIdx = -1;
  let bestArcScore = -Infinity;

  for (let i = 0; i < omegaSamples.length; i++) {
    const omega = omegaSamples[i];
    const v = 1.0 - Math.abs(omega) * 0.18;
    const points: Array<{ x: number; y: number }> = [{ x: pose.x, y: pose.y }];
    let currX = pose.x;
    let currY = pose.y;
    let currTheta = desiredHeading + omega * 0.45;
    let isCollision = false;
    let minObsClearance = 5.0;
    let sumInflation = 0;

    // Rollout 5 micro-steps forward (~3.0 cells horizon)
    for (let s = 1; s <= 5; s++) {
      currTheta += omega * 0.18;
      currX += Math.cos(currTheta) * 0.58 * v;
      currY += Math.sin(currTheta) * 0.58 * v;
      points.push({ x: currX, y: currY });

      const gx = Math.round(currX);
      const gy = Math.round(currY);

      if (!inBounds(grid, gx, gy) || grid[gy][gx].belief === 'OCCUPIED') {
        isCollision = true;
        break;
      }

      sumInflation += grid[gy][gx].inflationCost;

      for (const obs of dynamicObstacles) {
        const d = Math.hypot(currX - obs.x, currY - obs.y);
        minObsClearance = Math.min(minObsClearance, d);
        if (d < 0.85) {
          isCollision = true;
        }
      }
      if (isCollision) break;
    }

    const endPt = points[points.length - 1];
    const distToLookahead = Math.hypot(
      endPt.x - lookahead.x,
      endPt.y - lookahead.y
    );
    const headingAlign = 1.0 - Math.abs(omega) / 1.5;

    const score = isCollision
      ? -1000
      : 3.5 * headingAlign -
        2.2 * distToLookahead +
        1.8 * minObsClearance -
        1.6 * sumInflation;

    if (!isCollision && score > bestArcScore) {
      bestArcScore = score;
      bestArcIdx = i;
    }

    rollouts.push({
      id: i,
      v,
      omega,
      points,
      score,
      isCollision,
      isSelected: false,
    });
  }

  if (bestArcIdx >= 0) {
    rollouts[bestArcIdx].isSelected = true;
  }

  // Decide the discrete grid step:
  // If plannedPath[1] is clear of dynamic obstacles, follow it;
  // otherwise pick the 8-connected neighbor maximizing DWA clearance + progress toward lookahead!
  const defaultNext = plannedPath[1] || { x: pose.x, y: pose.y };
  const blockedByDynamic = dynamicObstacles.some(
    (o) =>
      Math.hypot(o.x - defaultNext.x, o.y - defaultNext.y) <= 1.1 ||
      Math.hypot(o.x + o.vx - defaultNext.x, o.y + o.vy - defaultNext.y) <= 0.9
  );

  if (!blockedByDynamic && plannedPath.length > 1) {
    const closeDynamic = dynamicObstacles.some(
      (o) => Math.hypot(o.x - pose.x, o.y - pose.y) <= 2.5
    );
    return {
      rollouts,
      nextStep: defaultNext,
      evadingObstacle: closeDynamic,
    };
  }

  // Evasive DWA Neighbor Selection around moving obstacle
  let bestNeighbor: GridCoord = { x: pose.x, y: pose.y };
  let bestStepScore = -Infinity;

  for (const d of DIRS_8) {
    const nx = pose.x + d.x;
    const ny = pose.y + d.y;
    if (!inBounds(grid, nx, ny)) continue;
    const cell = grid[ny][nx];
    if (cell.belief !== 'FREE') continue;

    // Prevent diagonal corner cutting
    if (d.x !== 0 && d.y !== 0) {
      if (
        grid[pose.y][nx].belief !== 'FREE' ||
        grid[ny][pose.x].belief !== 'FREE'
      ) {
        continue;
      }
    }

    let minDynDist = 10;
    for (const obs of dynamicObstacles) {
      const distNow = Math.hypot(nx - obs.x, ny - obs.y);
      const distPred = Math.hypot(nx - (obs.x + obs.vx), ny - (obs.y + obs.vy));
      minDynDist = Math.min(minDynDist, distNow, distPred);
    }
    if (minDynDist < 1.05) continue;

    const distToGoal = Math.hypot(nx - lookahead.x, ny - lookahead.y);
    const tabooCost = cell.isDeadEndTaboo ? 25.0 : 0;
    const stepScore =
      4.0 * minDynDist -
      2.5 * distToGoal -
      3.0 * cell.inflationCost -
      tabooCost;

    if (stepScore > bestStepScore) {
      bestStepScore = stepScore;
      bestNeighbor = { x: nx, y: ny };
    }
  }

  return {
    rollouts,
    nextStep: bestNeighbor,
    evadingObstacle: true,
  };
}
