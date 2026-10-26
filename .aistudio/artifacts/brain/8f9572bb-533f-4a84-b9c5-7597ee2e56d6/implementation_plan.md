# Fix Dev Server Pre-Warming Crash & Optimize Three.js Startup

Resolves the `Pre-warming failed: dev server process died` error by optimizing Vite's dependency discovery and lazy-loading the Three.js 3D Gazebo viewport so the dev server starts and pre-warms instantaneously without memory spikes.

## User Review & Critical Decisions

> [!IMPORTANT]
> No functional features are removed—both the **3D Gazebo Industrial Warehouse Viewport** and the **2D RViz2 SLAM Map** remain active in your split view. This fix isolates the heavy Three.js WebGL shader/bundle initialization from Vite's initial server pre-warm phase.

- **Root Cause**: During container startup pre-warming, Vite simultaneously crawls, transforms, and pre-bundles `three` (`v0.186.1`) and its JSM addons (`OrbitControls`, `RoomEnvironment`) alongside the main React bundle, causing a memory/CPU spike that terminates the dev server process during pre-warm.
- **Resolution Strategy**:
  1. **Vite Pre-Warm & Dependency Optimization**: Configure `vite.config.ts` to cleanly handle `three` in `optimizeDeps` and disable unnecessary warmup overhead on port 3000.
  2. **Asynchronous Boundary (`React.lazy` + `Suspense`) for 3D Gazebo Stage**: Load the Three.js `GazeboViewport3D` module asynchronously after the main shell mounts, ensuring the initial HTTP pre-warm request completes in milliseconds.
  3. **Lightweight Lighting & Environment Setup**: Replace the heavy synchronous `PMREMGenerator` + `RoomEnvironment` cubemap shader bake on mount with direct studio multi-light PBR illumination (`HemisphereLight` + `DirectionalLight` key/rim lights), eliminating WebGL context stalls during startup.

---

## 1. Overview & Core Concept

- **What It Does**: Stabilizes the Vite development server on port 3000 so pre-warming and live reloads succeed reliably while keeping 60 FPS 3D Gazebo warehouse rendering and 2D RViz2 SLAM simulation performance.
- **Key Value**: Prevents container OOM / timeout crashes during dev server startup and speeds up initial page load.

---

## 2. User Experience & Visual Design

- **Seamless Split-View Loading**: The main workstation shell, telemetry ribbon, and 2D RViz2 SLAM canvas render immediately on first paint while the 3D Gazebo viewport mounts smoothly inside a styled dark-slate `Suspense` boundary.
- **Visual Fidelity Preserved**: All 3D warehouse racks, boxes, 8-part composite AMR, spinning 360° LiDAR laser fan, moving 3D AGVs, and locked dead-end floor overlays remain visually identical.

---

## 3. Key Product Decisions & Trade-Offs

- **Decision 1: Replace Synchronous `PMREMGenerator` Cubemap Bake with Direct 3-Point Studio Lighting**
  - *Chosen Approach*: Remove `PMREMGenerator.fromScene(new RoomEnvironment())` and instead use calibrated 3-point `HemisphereLight` + warm Key `DirectionalLight` + cool cyan Rim `DirectionalLight`.
  - *Why*: `PMREMGenerator` compiles and renders a multi-pass cubemap synchronously on the first frame, which can stall headless/pre-warming browser probes. Direct 3-point lighting is zero-latency and looks crisp on metallic and concrete warehouse surfaces.

---

## 4. Technical Architecture & Data Strategy

```
┌────────────────────────────────────────────────────────────────────────────┐
│                   VITE DEV SERVER PRE-WARM (PORT 3000)                     │
│  • Fast Initial Shell Load (App + 2D RViz2 SLAM Engine)                    │
│  • Async Dynamic Import Boundary (React.lazy -> GazeboViewport3D)          │
│  • Zero-Stall WebGL Context Init (No synchronous PMREM cubemap bake)       │
└────────────────────────────────────────────────────────────────────────────┘
```
