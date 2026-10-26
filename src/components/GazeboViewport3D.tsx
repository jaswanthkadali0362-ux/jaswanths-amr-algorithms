import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Camera, Compass, Eye, Maximize2 } from 'lucide-react';
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
} from '../types/slam';

export type GazeboCameraMode = 'ORBIT_ISO' | 'CHASE_CAM' | 'TOP_DOWN';

interface GazeboViewport3DProps {
  grid: CellState[][];
  robotPose: { x: number; y: number; theta: number };
  homeDock: GridCoord;
  lidarRays: LidarRay[];
  frontiers: FrontierCluster[];
  selectedFrontier: FrontierCluster | null;
  currentPath: GridCoord[];
  deadEndRecords: DeadEndRecord[];
  dynamicObstacles: DynamicObstacle[];
  dwaRollouts: DWARolloutArc[];
  fsmState: FSMState;
  layers: LayerVisibility;
  presetName: string;
}

// Pre-allocated module-scope math objects (Zero GC in render loop)
const _dummyMatrix = new THREE.Matrix4();
const _dummyPos = new THREE.Vector3();
const _dummyQuat = new THREE.Quaternion();
const _dummyScale = new THREE.Vector3(1, 1, 1);
const _dummyColor = new THREE.Color();
const _targetCamPos = new THREE.Vector3();
const _targetLookAt = new THREE.Vector3();

function createConcreteFloorTexture(maxAnisotropy: number): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 512;
  const ctx = canvas.getContext('2d')!;

  // Industrial sealed concrete slab base
  ctx.fillStyle = '#18202C';
  ctx.fillRect(0, 0, 512, 512);

  // Subtle aggregate speckle
  for (let i = 0; i < 1800; i++) {
    const x = (i * 73) % 512;
    const y = (i * 137) % 512;
    ctx.fillStyle = i % 2 === 0 ? 'rgba(255,255,255,0.025)' : 'rgba(0,0,0,0.04)';
    ctx.fillRect(x, y, 2, 2);
  }

  // Gazebo 1-cell calibration grid lines
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.12)';
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, 512, 512);

  // Sub-grid crosshairs
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.06)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(256, 0);
  ctx.lineTo(256, 512);
  ctx.moveTo(0, 256);
  ctx.lineTo(512, 256);
  ctx.stroke();

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.anisotropy = maxAnisotropy;
  return tex;
}

/**
 * Builds an 8-part composite 3D Industrial AMR Group (grounded at y=0)
 */
function createCompositeAMRModel(): {
  group: THREE.Group;
  turret: THREE.Object3D;
  ledRingMat: THREE.MeshStandardMaterial;
  wheels: THREE.Object3D[];
} {
  const group = new THREE.Group();
  const wheels: THREE.Object3D[] = [];

  // 1. Primary Lower Chassis Hull (Beveled Extrude)
  const hullShape = new THREE.Shape();
  const w = 0.36;
  const l = 0.44;
  const b = 0.08;
  hullShape.moveTo(-l + b, -w);
  hullShape.lineTo(l - b, -w);
  hullShape.lineTo(l, -w + b);
  hullShape.lineTo(l, w - b);
  hullShape.lineTo(l - b, w);
  hullShape.lineTo(-l + b, w);
  hullShape.lineTo(-l, w - b);
  hullShape.lineTo(-l, -w + b);
  hullShape.closePath();

  const hullGeo = new THREE.ExtrudeGeometry(hullShape, {
    depth: 0.22,
    bevelEnabled: true,
    bevelThickness: 0.03,
    bevelSize: 0.03,
    bevelSegments: 2,
  });
  hullGeo.rotateX(-Math.PI / 2);
  const hullMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    metalness: 0.65,
    roughness: 0.35,
  });
  const hullMesh = new THREE.Mesh(hullGeo, hullMat);
  hullMesh.position.y = 0.08;
  hullMesh.castShadow = true;
  hullMesh.receiveShadow = true;
  group.add(hullMesh);

  // 2. Upper Brushed Aluminum Deck Plate (Contrast roughness/metalness)
  const deckGeo = new THREE.BoxGeometry(0.72, 0.05, 0.58);
  const deckMat = new THREE.MeshStandardMaterial({
    color: 0x94a3b8,
    metalness: 0.85,
    roughness: 0.18,
  });
  const deckMesh = new THREE.Mesh(deckGeo, deckMat);
  deckMesh.position.y = 0.34;
  deckMesh.castShadow = true;
  group.add(deckMesh);

  // 3. Status LED Perimeter Bumper Ring
  const ledRingGeo = new THREE.BoxGeometry(0.92, 0.04, 0.76);
  const ledRingMat = new THREE.MeshStandardMaterial({
    color: 0x06b6d4,
    emissive: 0x06b6d4,
    emissiveIntensity: 0.85,
    roughness: 0.2,
    metalness: 0.3,
  });
  const ledRingMesh = new THREE.Mesh(ledRingGeo, ledRingMat);
  ledRingMesh.position.y = 0.16;
  group.add(ledRingMesh);

  // 4 & 5. Left & Right Differential Drive Wheels
  const wheelGeo = new THREE.CylinderGeometry(0.14, 0.14, 0.08, 20);
  wheelGeo.rotateX(Math.PI / 2);
  const wheelMat = new THREE.MeshStandardMaterial({
    color: 0x0f172a,
    roughness: 0.88,
    metalness: 0.1,
  });
  const leftWheel = new THREE.Mesh(wheelGeo, wheelMat);
  leftWheel.position.set(0, 0.14, 0.36);
  const rightWheel = new THREE.Mesh(wheelGeo, wheelMat);
  rightWheel.position.set(0, 0.14, -0.36);
  group.add(leftWheel, rightWheel);
  wheels.push(leftWheel, rightWheel);

  // 6. Sensor Mast Riser
  const mastGeo = new THREE.CylinderGeometry(0.12, 0.16, 0.16, 16);
  const mastMat = new THREE.MeshStandardMaterial({
    color: 0x334155,
    metalness: 0.5,
    roughness: 0.45,
  });
  const mastMesh = new THREE.Mesh(mastGeo, mastMat);
  mastMesh.position.set(0.05, 0.44, 0);
  group.add(mastMesh);

  // 7. Spinning 360° LiDAR Turret Puck
  const turret = new THREE.Group();
  turret.position.set(0.05, 0.56, 0);

  const puckGeo = new THREE.CylinderGeometry(0.13, 0.13, 0.11, 24);
  const puckMat = new THREE.MeshStandardMaterial({
    color: 0x0f172a,
    metalness: 0.8,
    roughness: 0.2,
  });
  const puckMesh = new THREE.Mesh(puckGeo, puckMat);

  const lensGeo = new THREE.CylinderGeometry(0.135, 0.135, 0.04, 24);
  const lensMat = new THREE.MeshStandardMaterial({
    color: 0x22d3ee,
    emissive: 0x0891b2,
    emissiveIntensity: 0.9,
    roughness: 0.15,
    metalness: 0.9,
  });
  const lensMesh = new THREE.Mesh(lensGeo, lensMat);

  // Turret rotary sweep indicator bar
  const barGeo = new THREE.BoxGeometry(0.24, 0.02, 0.04);
  const barMesh = new THREE.Mesh(barGeo, lensMat);
  barMesh.position.y = 0.06;

  turret.add(puckMesh, lensMesh, barMesh);
  group.add(turret);

  // 8. Front Optical Depth Camera Visor (+X forward direction)
  const visorGeo = new THREE.BoxGeometry(0.06, 0.06, 0.28);
  const visorMesh = new THREE.Mesh(visorGeo, lensMat);
  visorMesh.position.set(0.43, 0.26, 0);
  group.add(visorMesh);

  return { group, turret, ledRingMat, wheels };
}

/**
 * Builds a 5-part 3D Patrolling Warehouse AGV Obstacle Group
 */
function createDynamicAGVModel(): THREE.Group {
  const g = new THREE.Group();

  const bodyGeo = new THREE.BoxGeometry(0.74, 0.28, 0.62);
  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0xea580c,
    metalness: 0.45,
    roughness: 0.35,
  });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.position.y = 0.2;
  body.castShadow = true;
  g.add(body);

  const bumperGeo = new THREE.BoxGeometry(0.8, 0.08, 0.68);
  const bumperMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    roughness: 0.75,
    metalness: 0.2,
  });
  const bumper = new THREE.Mesh(bumperGeo, bumperMat);
  bumper.position.y = 0.09;
  g.add(bumper);

  const rackGeo = new THREE.BoxGeometry(0.56, 0.22, 0.48);
  const rackMat = new THREE.MeshStandardMaterial({
    color: 0xd97706,
    roughness: 0.6,
    metalness: 0.15,
  });
  const cargo = new THREE.Mesh(rackGeo, rackMat);
  cargo.position.y = 0.45;
  cargo.castShadow = true;
  g.add(cargo);

  const beaconGeo = new THREE.CylinderGeometry(0.06, 0.06, 0.12, 12);
  const beaconMat = new THREE.MeshStandardMaterial({
    color: 0xfbbf24,
    emissive: 0xf59e0b,
    emissiveIntensity: 0.9,
    roughness: 0.2,
  });
  const beacon = new THREE.Mesh(beaconGeo, beaconMat);
  beacon.position.set(0.18, 0.62, 0);
  g.add(beacon);

  return g;
}

export const GazeboViewport3D: React.FC<GazeboViewport3DProps> = ({
  grid,
  robotPose,
  homeDock,
  lidarRays,
  frontiers,
  selectedFrontier,
  currentPath,
  deadEndRecords,
  dynamicObstacles,
  dwaRollouts,
  fsmState,
  layers,
  presetName,
}) => {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const [cameraMode, setCameraMode] = useState<GazeboCameraMode>('ORBIT_ISO');

  const rows = grid.length;
  const cols = grid[0]?.length || 44;

  // Store latest props in ref so continuous Three.js loop always reads fresh state without re-creating WebGLRenderer
  const latestPropsRef = useRef({
    grid,
    robotPose,
    homeDock,
    lidarRays,
    frontiers,
    selectedFrontier,
    currentPath,
    deadEndRecords,
    dynamicObstacles,
    dwaRollouts,
    fsmState,
    layers,
    cameraMode,
  });

  useEffect(() => {
    latestPropsRef.current = {
      grid,
      robotPose,
      homeDock,
      lidarRays,
      frontiers,
      selectedFrontier,
      currentPath,
      deadEndRecords,
      dynamicObstacles,
      dwaRollouts,
      fsmState,
      layers,
      cameraMode,
    };
  }, [
    grid,
    robotPose,
    homeDock,
    lidarRays,
    frontiers,
    selectedFrontier,
    currentPath,
    deadEndRecords,
    dynamicObstacles,
    dwaRollouts,
    fsmState,
    layers,
    cameraMode,
  ]);

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const width = container.clientWidth || 720;
    const height = container.clientHeight || 480;

    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({
        antialias: false,
        powerPreference: 'high-performance',
      });
    } catch {
      return;
    }

    // Detect software WebGL renderer from the active context without creating a second context
    const isSoftwareGl = (() => {
      try {
        const gl = renderer.getContext();
        if (!gl) return true;
        const ext = gl.getExtension('WEBGL_debug_renderer_info');
        const param = ext ? ext.UNMASKED_RENDERER_WEBGL : gl.RENDERER;
        const gpu = String(gl.getParameter(param));
        return /SwiftShader|llvmpipe|softpipe|Software|Basic Render/i.test(gpu);
      } catch {
        return true;
      }
    })();

    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    renderer.setPixelRatio(
      isSoftwareGl ? 1 : Math.min(window.devicePixelRatio, 2)
    );
    renderer.setSize(width, height);
    renderer.shadowMap.enabled = !isSoftwareGl;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.innerHTML = '';
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f19);
    scene.fog = new THREE.FogExp2(0x0b0f19, 0.012);

    // Camera & OrbitControls
    const camera = new THREE.PerspectiveCamera(46, width / height, 0.2, 160);
    const centerX = cols / 2;
    const centerZ = rows / 2;
    camera.position.set(centerX - 8, 28, centerZ + 24);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(centerX, 0, centerZ);
    controls.enableDamping = !isSoftwareGl;
    controls.dampingFactor = 0.08;
    controls.maxPolarAngle = Math.PI / 2 - 0.04;
    controls.minDistance = 4;
    controls.maxDistance = 68;
    controls.update();

    // Balanced 3-Point Industrial Warehouse Lighting
    const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.95);
    hemiLight.position.set(centerX, 30, centerZ);
    scene.add(hemiLight);

    const keyLight = new THREE.DirectionalLight(0xfff7ed, 1.55);
    keyLight.position.set(centerX + 14, 32, centerZ + 12);
    keyLight.target.position.set(centerX, 0, centerZ);
    if (!isSoftwareGl) {
      keyLight.castShadow = true;
      keyLight.shadow.mapSize.width = 1024;
      keyLight.shadow.mapSize.height = 1024;
      const d = 28;
      keyLight.shadow.camera.left = -d;
      keyLight.shadow.camera.right = d;
      keyLight.shadow.camera.top = d;
      keyLight.shadow.camera.bottom = -d;
      keyLight.shadow.camera.near = 2;
      keyLight.shadow.camera.far = 65;
      keyLight.shadow.bias = -0.0005;
      keyLight.shadow.normalBias = 0.02;
    }
    scene.add(keyLight);
    scene.add(keyLight.target);

    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.45);
    rimLight.position.set(centerX - 18, 20, centerZ - 16);
    scene.add(rimLight);

    // 5. Industrial Concrete Warehouse Floor
    const floorTex = createConcreteFloorTexture(
      renderer.capabilities.getMaxAnisotropy()
    );
    floorTex.repeat.set(cols / 2, rows / 2);
    const floorGeo = new THREE.PlaneGeometry(cols + 16, rows + 16);
    floorGeo.rotateX(-Math.PI / 2);
    const floorMat = new THREE.MeshStandardMaterial({
      map: floorTex,
      roughness: 0.78,
      metalness: 0.12,
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.position.set(centerX - 0.5, 0, centerZ - 0.5);
    floorMesh.receiveShadow = true;
    scene.add(floorMesh);

    // 6. InstancedMesh for Warehouse Walls, Pallet Racking Bays & Cargo Boxes
    const maxCells = rows * cols;

    // Outer & Structural Walls
    const wallGeo = new THREE.BoxGeometry(0.98, 2.1, 0.98);
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.55,
      metalness: 0.35,
    });
    const wallInstanced = new THREE.InstancedMesh(wallGeo, wallMat, maxCells);
    wallInstanced.castShadow = !isSoftwareGl;
    wallInstanced.receiveShadow = true;
    wallInstanced.frustumCulled = false;
    scene.add(wallInstanced);

    // Industrial Pallet Rack Steel Uprights + Orange Beams for interior obstacles
    const rackFrameGeo = new THREE.BoxGeometry(0.92, 2.25, 0.92);
    const rackFrameMat = new THREE.MeshStandardMaterial({
      color: 0x1d4ed8,
      roughness: 0.4,
      metalness: 0.65,
    });
    const rackInstanced = new THREE.InstancedMesh(
      rackFrameGeo,
      rackFrameMat,
      maxCells
    );
    rackInstanced.castShadow = !isSoftwareGl;
    rackInstanced.receiveShadow = true;
    rackInstanced.frustumCulled = false;
    scene.add(rackInstanced);

    // Pallet Wooden/Cardboard Boxes sitting atop interior racks
    const boxGeo = new THREE.BoxGeometry(0.76, 0.55, 0.76);
    const boxMat = new THREE.MeshStandardMaterial({
      color: 0xd97706,
      roughness: 0.75,
      metalness: 0.1,
    });
    const boxInstanced = new THREE.InstancedMesh(boxGeo, boxMat, maxCells);
    boxInstanced.frustumCulled = false;
    scene.add(boxInstanced);

    // 7. InstancedMesh for Holographic Floor Overlays (Taboo Locked Dead-Ends + Frontiers + Mapped Floor)
    const tileGeo = new THREE.PlaneGeometry(0.92, 0.92);
    tileGeo.rotateX(-Math.PI / 2);

    const tabooTileMat = new THREE.MeshBasicMaterial({
      color: 0xf43f5e,
      transparent: true,
      opacity: 0.42,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -1,
      polygonOffsetUnits: -1,
    });
    const tabooInstanced = new THREE.InstancedMesh(
      tileGeo,
      tabooTileMat,
      maxCells
    );
    tabooInstanced.frustumCulled = false;
    scene.add(tabooInstanced);

    const frontierTileMat = new THREE.MeshBasicMaterial({
      color: 0xf59e0b,
      transparent: true,
      opacity: 0.55,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });
    const frontierInstanced = new THREE.InstancedMesh(
      tileGeo,
      frontierTileMat,
      maxCells
    );
    frontierInstanced.frustumCulled = false;
    scene.add(frontierInstanced);

    // 8. Home Charging Dock Station Pad
    const dockGroup = new THREE.Group();
    const dockPadGeo = new THREE.BoxGeometry(1.5, 0.05, 1.5);
    const dockPadMat = new THREE.MeshStandardMaterial({
      color: 0x059669,
      emissive: 0x10b981,
      emissiveIntensity: 0.45,
      roughness: 0.3,
      metalness: 0.6,
    });
    const dockPad = new THREE.Mesh(dockPadGeo, dockPadMat);
    dockPad.position.y = 0.025;
    const dockPostGeo = new THREE.BoxGeometry(0.25, 0.75, 0.85);
    const dockPost = new THREE.Mesh(dockPostGeo, dockPadMat);
    dockPost.position.set(-0.65, 0.38, 0);
    dockGroup.add(dockPad, dockPost);
    scene.add(dockGroup);

    // 9. Composite 8-Part 3D AMR Model
    const amr = createCompositeAMRModel();
    const initPose = latestPropsRef.current.robotPose;
    amr.group.position.set(initPose.x, 0, initPose.y);
    amr.group.rotation.y = -initPose.theta;
    scene.add(amr.group);

    // 10. Pool of 6 Composite 3D Dynamic AGV Obstacles
    const agvPool: THREE.Group[] = [];
    for (let i = 0; i < 8; i++) {
      const agv = createDynamicAGVModel();
      agv.visible = false;
      scene.add(agv);
      agvPool.push(agv);
    }

    // 11. 360° 3D LiDAR Laser Rays LineSegments
    const maxRays = 180;
    const lidarPositions = new Float32Array(maxRays * 2 * 3);
    const lidarGeo = new THREE.BufferGeometry();
    lidarGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(lidarPositions, 3)
    );
    const lidarMat = new THREE.LineBasicMaterial({
      color: 0x22d3ee,
      transparent: true,
      opacity: 0.38,
    });
    const lidarLines = new THREE.LineSegments(lidarGeo, lidarMat);
    lidarLines.frustumCulled = false;
    scene.add(lidarLines);

    // 12. 3D Global A* Planned Trajectory Line
    const maxPathPts = 200;
    const pathPositions = new Float32Array(maxPathPts * 3);
    const pathGeo = new THREE.BufferGeometry();
    pathGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(pathPositions, 3)
    );
    const pathMat = new THREE.LineBasicMaterial({
      color: 0x10b981,
      linewidth: 2,
    });
    const pathLine = new THREE.Line(pathGeo, pathMat);
    pathLine.frustumCulled = false;
    scene.add(pathLine);

    // Smoothly damped state trackers
    let smoothRobotX = initPose.x;
    let smoothRobotZ = initPose.y;
    let smoothRobotYaw = -initPose.theta;
    const smoothAgvPos: Array<{ x: number; z: number; yaw: number }> =
      Array.from({ length: 8 }, () => ({ x: 0, z: 0, yaw: 0 }));

    const clock = new THREE.Clock();
    let frameId = 0;
    let isDisposed = false;

    function afterDraw(cb: () => void) {
      const gl = renderer.getContext() as WebGL2RenderingContext;
      const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
      gl.flush();
      const poll = () => {
        if (isDisposed) return;
        const state = sync && gl.getSyncParameter(sync, gl.SYNC_STATUS);
        if (state === gl.UNSIGNALED) {
          frameId = window.setTimeout(poll, 5);
        } else {
          if (sync) gl.deleteSync(sync);
          frameId = window.setTimeout(cb, 80);
        }
      };
      frameId = window.setTimeout(poll, 5);
    }

    function updateSceneGeometry(delta: number) {
      const state = latestPropsRef.current;
      const g = state.grid;

      // Update Home Dock position
      dockGroup.position.set(state.homeDock.x, 0, state.homeDock.y);

      // Update Walls, Racks, Taboo & Frontier InstancedMeshes
      let wallIdx = 0;
      let rackIdx = 0;
      let boxIdx = 0;
      let tabooIdx = 0;
      let frontierIdx = 0;

      for (let y = 0; y < rows; y++) {
        for (let x = 0; x < cols; x++) {
          const cell = g[y][x];
          const isOuterBorder =
            x === 0 || y === 0 || x === cols - 1 || y === rows - 1;

          if (cell.groundTruth === 'WALL') {
            // Show walls if showGroundTruthGhost is on OR if the robot has already mapped them
            const visibleInGazebo =
              state.layers.showGroundTruthGhost || cell.belief === 'OCCUPIED';
            if (visibleInGazebo) {
              if (isOuterBorder) {
                _dummyPos.set(x, 1.05, y);
                _dummyQuat.identity();
                _dummyScale.set(1, 1, 1);
                _dummyMatrix.compose(_dummyPos, _dummyQuat, _dummyScale);
                wallInstanced.setMatrixAt(wallIdx++, _dummyMatrix);
              } else {
                // Interior warehouse pallet rack or room wall
                _dummyPos.set(x, 1.125, y);
                _dummyQuat.identity();
                _dummyScale.set(1, 1, 1);
                _dummyMatrix.compose(_dummyPos, _dummyQuat, _dummyScale);
                rackInstanced.setMatrixAt(rackIdx++, _dummyMatrix);

                // Top pallet cargo box
                _dummyPos.set(x, 2.52, y);
                _dummyMatrix.compose(_dummyPos, _dummyQuat, _dummyScale);
                boxInstanced.setMatrixAt(boxIdx++, _dummyMatrix);
              }
            }
          } else {
            // Floor Overlays: Taboo Locked Dead-End Cells
            if (state.layers.showDeadEndTaboo && cell.isDeadEndTaboo) {
              _dummyPos.set(x, 0.03, y);
              _dummyQuat.identity();
              _dummyScale.set(1, 1, 1);
              _dummyMatrix.compose(_dummyPos, _dummyQuat, _dummyScale);
              tabooInstanced.setMatrixAt(tabooIdx++, _dummyMatrix);
            }
            // Frontier Cells
            if (
              state.layers.showFrontiers &&
              cell.isFrontier &&
              !cell.isDeadEndTaboo
            ) {
              _dummyPos.set(x, 0.04, y);
              _dummyQuat.identity();
              _dummyScale.set(1, 1, 1);
              _dummyMatrix.compose(_dummyPos, _dummyQuat, _dummyScale);
              frontierInstanced.setMatrixAt(frontierIdx++, _dummyMatrix);
            }
          }
        }
      }

      wallInstanced.count = wallIdx;
      wallInstanced.instanceMatrix.needsUpdate = true;
      rackInstanced.count = rackIdx;
      rackInstanced.instanceMatrix.needsUpdate = true;
      boxInstanced.count = boxIdx;
      boxInstanced.instanceMatrix.needsUpdate = true;
      tabooInstanced.count = tabooIdx;
      tabooInstanced.instanceMatrix.needsUpdate = true;
      frontierInstanced.count = frontierIdx;
      frontierInstanced.instanceMatrix.needsUpdate = true;

      // Smoothly interpolate 3D AMR Pose
      const dampLambda = 12;
      smoothRobotX = THREE.MathUtils.damp(
        smoothRobotX,
        state.robotPose.x,
        dampLambda,
        delta
      );
      smoothRobotZ = THREE.MathUtils.damp(
        smoothRobotZ,
        state.robotPose.y,
        dampLambda,
        delta
      );

      const targetYaw = -state.robotPose.theta;
      const yawDiff = Math.atan2(
        Math.sin(targetYaw - smoothRobotYaw),
        Math.cos(targetYaw - smoothRobotYaw)
      );
      smoothRobotYaw += yawDiff * Math.min(1, delta * 10);

      amr.group.position.set(smoothRobotX, 0, smoothRobotZ);
      amr.group.rotation.y = smoothRobotYaw;

      // Spin 360° LiDAR turret continuously
      amr.turret.rotation.y += delta * 9.5;

      // Update AMR Status LED Bumper color based on FSM State
      if (state.fsmState === 'AVOIDING_OBSTACLE') {
        _dummyColor.setHex(0xfb923c);
      } else if (state.fsmState === 'PRUNING_DEAD_END') {
        _dummyColor.setHex(0xf43f5e);
      } else if (
        state.fsmState === 'RETURNING_HOME' ||
        state.fsmState === 'DOCKED_COMPLETE'
      ) {
        _dummyColor.setHex(0x10b981);
      } else {
        _dummyColor.setHex(0x06b6d4);
      }
      amr.ledRingMat.color.copy(_dummyColor);
      amr.ledRingMat.emissive.copy(_dummyColor);

      // Update 3D Moving AGV Obstacles
      const obsList = state.layers.showDynamicObstacles
        ? state.dynamicObstacles
        : [];
      for (let i = 0; i < agvPool.length; i++) {
        const mesh = agvPool[i];
        const obs = obsList[i];
        if (!obs) {
          mesh.visible = false;
          continue;
        }
        if (!mesh.visible) {
          smoothAgvPos[i].x = obs.x;
          smoothAgvPos[i].z = obs.y;
          mesh.visible = true;
        } else {
          smoothAgvPos[i].x = THREE.MathUtils.damp(
            smoothAgvPos[i].x,
            obs.x,
            10,
            delta
          );
          smoothAgvPos[i].z = THREE.MathUtils.damp(
            smoothAgvPos[i].z,
            obs.y,
            10,
            delta
          );
        }
        mesh.position.set(smoothAgvPos[i].x, 0, smoothAgvPos[i].z);
        mesh.rotation.y = -Math.atan2(obs.vy, obs.vx);
      }

      // Update 360° 3D LiDAR Rays
      if (state.layers.showLidarRays && state.lidarRays.length > 0) {
        lidarLines.visible = true;
        const count = Math.min(maxRays, state.lidarRays.length);
        for (let i = 0; i < count; i++) {
          const ray = state.lidarRays[i];
          const baseIdx = i * 6;
          lidarPositions[baseIdx] = smoothRobotX;
          lidarPositions[baseIdx + 1] = 0.56;
          lidarPositions[baseIdx + 2] = smoothRobotZ;

          lidarPositions[baseIdx + 3] = ray.endX;
          lidarPositions[baseIdx + 4] = ray.hitOccupied ? 0.56 : 0.12;
          lidarPositions[baseIdx + 5] = ray.endY;
        }
        lidarGeo.setDrawRange(0, count * 2);
        lidarGeo.attributes.position.needsUpdate = true;
      } else {
        lidarLines.visible = false;
      }

      // Update 3D Global A* Planned Path Ribbon
      if (state.layers.showPlannedPath && state.currentPath.length > 1) {
        pathLine.visible = true;
        const pCount = Math.min(maxPathPts, state.currentPath.length);
        for (let i = 0; i < pCount; i++) {
          const pt = state.currentPath[i];
          const idx = i * 3;
          pathPositions[idx] = pt.x;
          pathPositions[idx + 1] = 0.12;
          pathPositions[idx + 2] = pt.y;
        }
        pathGeo.setDrawRange(0, pCount);
        pathGeo.attributes.position.needsUpdate = true;
      } else {
        pathLine.visible = false;
      }

      // Camera Choreography (Orbit vs 3rd-Person Chase vs Top-Down Ortho-Perspective)
      if (state.cameraMode === 'CHASE_CAM') {
        controls.enabled = false;
        const backDist = 7.5;
        const camX =
          smoothRobotX - Math.cos(state.robotPose.theta) * backDist;
        const camZ =
          smoothRobotZ - Math.sin(state.robotPose.theta) * backDist;
        _targetCamPos.set(camX, 5.2, camZ);
        camera.position.lerp(_targetCamPos, Math.min(1, delta * 5));

        _targetLookAt.set(
          smoothRobotX + Math.cos(state.robotPose.theta) * 3.5,
          0.6,
          smoothRobotZ + Math.sin(state.robotPose.theta) * 3.5
        );
        controls.target.lerp(_targetLookAt, Math.min(1, delta * 7));
        camera.lookAt(controls.target);
      } else if (state.cameraMode === 'TOP_DOWN') {
        controls.enabled = true;
        _targetCamPos.set(centerX, 38, centerZ + 0.01);
        _targetLookAt.set(centerX, 0, centerZ);
        camera.position.lerp(_targetCamPos, Math.min(1, delta * 4));
        controls.target.lerp(_targetLookAt, Math.min(1, delta * 4));
        controls.update();
      } else {
        controls.enabled = true;
        controls.update();
      }
    }

    function tick() {
      if (isDisposed) return;
      const delta = Math.min(0.1, clock.getDelta());
      updateSceneGeometry(delta);
      renderer.render(scene, camera);
      if (isSoftwareGl) afterDraw(tick);
      else frameId = requestAnimationFrame(tick);
    }

    tick();

    const handleResize = () => {
      if (!container || isDisposed) return;
      const w = container.clientWidth || 720;
      const h = container.clientHeight || 480;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    return () => {
      isDisposed = true;
      window.removeEventListener('resize', handleResize);
      cancelAnimationFrame(frameId);
      clearTimeout(frameId);
      controls.dispose();
      floorTex.dispose();
      floorGeo.dispose();
      floorMat.dispose();
      wallGeo.dispose();
      wallMat.dispose();
      rackFrameGeo.dispose();
      rackFrameMat.dispose();
      boxGeo.dispose();
      boxMat.dispose();
      tileGeo.dispose();
      tabooTileMat.dispose();
      frontierTileMat.dispose();
      lidarGeo.dispose();
      lidarMat.dispose();
      pathGeo.dispose();
      pathMat.dispose();
      renderer.dispose();
    };
  }, [rows, cols]);

  return (
    <div className="relative w-full bg-[#090D16] border border-[#1E293B] rounded-lg overflow-hidden flex flex-col">
      {/* Top Gazebo 3D World Header Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 bg-[#111827] border-b border-[#1E293B] text-xs">
        <div className="flex items-center gap-2.5">
          <span className="font-semibold text-slate-100">
            Gazebo 3D Physical Environment
          </span>
          <span aria-hidden="true" className="text-slate-600">
            ·
          </span>
          <span className="font-mono text-cyan-400">{presetName}</span>
        </div>

        {/* Camera Preset Selector */}
        <div className="flex items-center gap-1 p-0.5 bg-[#090D16] border border-[#1E293B] rounded">
          <button
            type="button"
            onClick={() => setCameraMode('ORBIT_ISO')}
            className={`flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-medium transition-colors whitespace-nowrap shrink-0 ${
              cameraMode === 'ORBIT_ISO'
                ? 'bg-cyan-600 text-white'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Compass className="w-3 h-3" />
            <span>3D Orbit</span>
          </button>
          <button
            type="button"
            onClick={() => setCameraMode('CHASE_CAM')}
            className={`flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-medium transition-colors whitespace-nowrap shrink-0 ${
              cameraMode === 'CHASE_CAM'
                ? 'bg-cyan-600 text-white'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Camera className="w-3 h-3" />
            <span>AMR Chase Cam</span>
          </button>
          <button
            type="button"
            onClick={() => setCameraMode('TOP_DOWN')}
            className={`flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-medium transition-colors whitespace-nowrap shrink-0 ${
              cameraMode === 'TOP_DOWN'
                ? 'bg-cyan-600 text-white'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Maximize2 className="w-3 h-3" />
            <span>Top-Down</span>
          </button>
        </div>
      </div>

      {/* Dedicated Isolated WebGL Canvas Mount Container */}
      <div className="relative w-full h-[460px] lg:h-[520px] bg-[#0B0F19]">
        <div ref={mountRef} className="w-full h-full" />

        {/* Subtle Bottom-Center Unobtrusive Gazebo Camera Legend */}
        <div className="pointer-events-none absolute bottom-3 left-1/2 -translate-x-1/2 px-3 py-1.5 rounded bg-[#0F172A]/90 border border-slate-700/80 text-[11px] font-mono text-slate-300 flex items-center gap-3 whitespace-nowrap">
          <span className="inline-flex items-center gap-1 text-cyan-400">
            <Eye className="w-3 h-3" />
            <span>
              {cameraMode === 'ORBIT_ISO'
                ? 'Drag to Orbit · Scroll to Zoom · Right-Drag to Pan'
                : cameraMode === 'CHASE_CAM'
                ? 'Tracking AMR 3rd-Person Trajectory'
                : 'Top-Down Orthographic Warehouse View'}
            </span>
          </span>
          <span aria-hidden="true" className="text-slate-600">
            |
          </span>
          <span className="text-rose-400">
            Locked Dead-End Zones: {deadEndRecords.length}
          </span>
        </div>
      </div>
    </div>
  );
};

export default GazeboViewport3D;
