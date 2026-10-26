import React, { useMemo, useState } from 'react';
import {
  Check,
  Copy,
  Download,
  FileCode,
  FolderGit2,
  Sliders,
  Terminal,
} from 'lucide-react';
import {
  generateRos2PackageFiles,
  generateSelfExtractingInstallerSh,
  Ros2PackageConfig,
} from '../algorithms/ros2PackageGenerator';
import { FSMState, SimulationConfig } from '../types/slam';

interface AlgorithmInspectorProps {
  fsmState: FSMState;
  config: SimulationConfig;
  coveragePct: number;
  frontierCount: number;
  lockedCellsCount: number;
  deadEndZoneCount: number;
}

export const AlgorithmInspector: React.FC<AlgorithmInspectorProps> = ({
  fsmState,
  config,
  coveragePct,
  frontierCount,
  lockedCellsCount,
  deadEndZoneCount,
}) => {
  const [rosConfig, setRosConfig] = useState<Ros2PackageConfig>({
    packageName: 'amr_dead_end_explorer',
    rosDistro: 'humble',
    scanTopic: '/scan',
    mapTopic: '/map',
    cmdVelTopic: '/cmd_vel',
    baseFrame: 'base_link',
    odomFrame: 'odom',
    mapFrame: 'map',
    useNav2Action: true,
    gridResolution: 0.05,
    robotRadius: 0.22,
  });

  const generatedFiles = useMemo(
    () => generateRos2PackageFiles(config, rosConfig),
    [config, rosConfig]
  );

  const [activeFileId, setActiveFileId] = useState<string>('node_py');
  const [copiedId, setCopiedId] = useState<string | null>(null);

  const activeFile =
    generatedFiles.find((f) => f.id === activeFileId) || generatedFiles[0];

  const handleCopyFile = (content: string, id: string) => {
    navigator.clipboard.writeText(content);
    setCopiedId(id);
    setTimeout(() => setCopiedId(null), 2000);
  };

  const handleDownloadSingleFile = () => {
    const cleanFilename = activeFile.filename.includes('&')
      ? 'setup_and_package.py'
      : activeFile.filename;
    const blob = new Blob([activeFile.content], {
      type: 'text/plain;charset=utf-8',
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = cleanFilename;
    a.click();
    URL.revokeObjectURL(url);
  };

  const handleDownloadFullPackageInstaller = () => {
    const shContent = generateSelfExtractingInstallerSh(config, rosConfig);
    const blob = new Blob([shContent], { type: 'text/x-sh;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `install_${rosConfig.packageName}.sh`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const stages = [
    {
      num: '01',
      title: 'Zero-Map LiDAR Log-Odds Mapping',
      active:
        fsmState === 'EXPLORING_FRONTIERS' || fsmState === 'PRUNING_DEAD_END',
      formula:
        'L(mᵢ | z₁:ₜ) = L(mᵢ | z₁:ₜ₋₁) + log [ P(mᵢ | zₜ) / (1 − P(mᵢ | zₜ)) ]',
      summary: `Initializes belief grid M to 100% UNKNOWN (-1). At each tick, casts ${config.lidarRayCount} rays across 360° up to R = ${config.lidarRange} cells. Ray traversal decrements log-odds (FREE), while obstacle hits increment log-odds (OCCUPIED).`,
    },
    {
      num: '02',
      title: 'Topological Dead-End Pruning & Taboo Memory',
      active: fsmState === 'PRUNING_DEAD_END' || deadEndZoneCount > 0,
      formula:
        'Taboo(P) = True  iff  ∂P ∩ UNKNOWN = ∅  ∧  HomeDock ∉ P  ∧  |Throat(P)| ≤ W_max',
      summary: `Continuously inspects mapped corridors and multi-cell bays. The instant a branch has zero remaining UNKNOWN frontier cells, its entire pocket from the closed tip back to the junction throat is permanently locked (Taboo = True). Currently ${lockedCellsCount} cells across ${deadEndZoneCount} dead-end zones are permanently sealed.`,
    },
    {
      num: '03',
      title: 'DWA Obstacle Avoidance & Inflated Safety Costmap',
      active:
        fsmState === 'AVOIDING_OBSTACLE' || fsmState === 'EXPLORING_FRONTIERS',
      formula: `G(v, ω) = α·Heading(v, ω) + β·Clearance_LaserScan(v, ω) − γ·Cost_Inflation(r ≤ ${config.safetyInflationRadius.toFixed(1)}c)`,
      summary: `Maintains an exponential repulsive safety buffer around walls and evaluates 11 Dynamic Window (v, ω) velocity rollout arcs at every step. Moving dynamic obstacles detected by 360° LiDAR trigger immediate DWA evasive steering without falsely sealing corridors as dead ends (${frontierCount} active frontier clusters remaining).`,
    },
    {
      num: '04',
      title: 'Autonomous A* Return-to-Origin',
      active: fsmState === 'RETURNING_HOME' || fsmState === 'DOCKED_COMPLETE',
      formula:
        'Frontiers = ∅  ⇒  Transition(RETURNING_HOME)  →  min ∑ Cost_A*(x_AMR → x_Dock)',
      summary: `When the frontier queue empties (${coveragePct.toFixed(1)}% reachable coverage), the AMR switches to RETURNING_HOME and follows the shortest collision-free A* trajectory over known-free space directly back to its starting dock.`,
    },
  ];

  return (
    <div className="space-y-8">
      {/* Part 1: 4-Stage Mathematical Architecture */}
      <div className="space-y-4">
        <div className="flex flex-col md:flex-row md:items-center justify-between gap-4 pb-4 border-b border-[#1E293B]">
          <div>
            <h2 className="text-xl font-semibold text-slate-100">
              Autonomous SLAM & Topological Dead-End Pruning Pipeline
            </h2>
            <p className="text-sm text-slate-400 mt-1">
              Mathematical architecture governing zero-prior-map exploration, permanent cul-de-sac sealing, and return-to-dock navigation.
            </p>
          </div>
          <div className="font-mono text-xs text-slate-400">
            ACTIVE FSM STATE:{' '}
            <span className="text-cyan-400 font-semibold">{fsmState}</span>
          </div>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          {stages.map((st) => (
            <div
              key={st.num}
              className={`p-5 rounded-lg border transition-colors ${
                st.active
                  ? 'bg-[#111827] border-cyan-500/60'
                  : 'bg-[#0F1523] border-[#1E293B]'
              }`}
            >
              <div className="flex items-center justify-between gap-2">
                <h3 className="text-base font-semibold text-slate-100">
                  {st.num}. {st.title}
                </h3>
                <span
                  className={`text-xs font-mono ${
                    st.active ? 'text-cyan-400' : 'text-slate-500'
                  }`}
                >
                  {st.active ? '● ACTIVE STAGE' : '○ STANDBY'}
                </span>
              </div>

              <div className="mt-3 px-3 py-2 bg-[#090D16] border border-[#1E293B] rounded font-mono text-xs text-amber-300 overflow-x-auto">
                {st.formula}
              </div>

              <p className="mt-3 text-sm text-slate-300 leading-relaxed">
                {st.summary}
              </p>
            </div>
          ))}
        </div>
      </div>

      {/* Part 2: Complete ROS 2 Humble / Jazzy Production Code Workbench */}
      <div className="bg-[#111827] border border-[#1E293B] rounded-lg overflow-hidden">
        {/* Workbench Header Bar */}
        <div className="flex flex-wrap items-center justify-between gap-4 px-5 py-4 border-b border-[#1E293B] bg-[#0D1320]">
          <div>
            <div className="flex items-center gap-2">
              <FolderGit2 className="w-5 h-5 text-cyan-400" />
              <h2 className="text-lg font-semibold text-slate-100">
                ROS 2 ({rosConfig.rosDistro.toUpperCase()}) Autonomous SLAM & Nav2 Code Workbench
              </h2>
            </div>
            <p className="text-xs text-slate-400 mt-1">
              Production-ready Python rclpy node, SLAM Toolbox zero-prior-map YAML, Nav2 parameters, unified launch script, and RViz2 config synchronized with your simulator parameters.
            </p>
          </div>

          <div className="flex flex-wrap items-center gap-2.5">
            <button
              type="button"
              onClick={handleDownloadFullPackageInstaller}
              className="flex items-center gap-2 px-4 py-2 text-xs font-semibold bg-cyan-500 hover:bg-cyan-400 text-slate-950 rounded-lg transition-colors whitespace-nowrap shrink-0"
            >
              <Download className="w-4 h-4" />
              <span>Download Full ROS 2 Package (.sh Builder)</span>
            </button>
          </div>
        </div>

        {/* ROS 2 Frame, Topic & Stack Customizer */}
        <div className="p-4 bg-[#0B0F1A] border-b border-[#1E293B]">
          <div className="flex items-center gap-2 text-xs font-semibold text-slate-300 mb-3">
            <Sliders className="w-3.5 h-3.5 text-cyan-400" />
            <span>Customize ROS 2 Hardware Frames, Topics & Navigation Mode (Live-Updates Code Below)</span>
          </div>

          <div className="grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-6 gap-3 text-xs">
            <div>
              <label className="block text-slate-400 mb-1">ROS 2 Distro</label>
              <div className="grid grid-cols-2 gap-1 p-0.5 bg-[#111827] border border-[#1E293B] rounded">
                {(['humble', 'jazzy'] as const).map((d) => (
                  <button
                    key={d}
                    type="button"
                    onClick={() =>
                      setRosConfig((prev) => ({ ...prev, rosDistro: d }))
                    }
                    className={`py-1 px-2 rounded font-mono uppercase text-[11px] transition-colors whitespace-nowrap shrink-0 ${
                      rosConfig.rosDistro === d
                        ? 'bg-cyan-600 text-white font-semibold'
                        : 'text-slate-400 hover:text-slate-200'
                    }`}
                  >
                    {d}
                  </button>
                ))}
              </div>
            </div>

            <div>
              <label className="block text-slate-400 mb-1">LiDAR Scan Topic</label>
              <input
                type="text"
                value={rosConfig.scanTopic}
                onChange={(e) =>
                  setRosConfig((prev) => ({
                    ...prev,
                    scanTopic: e.target.value || '/scan',
                  }))
                }
                className="w-full px-2.5 py-1.5 bg-[#111827] border border-[#1E293B] rounded font-mono text-slate-100 focus:outline-none focus:border-cyan-500"
              />
            </div>

            <div>
              <label className="block text-slate-400 mb-1">SLAM Map Topic</label>
              <input
                type="text"
                value={rosConfig.mapTopic}
                onChange={(e) =>
                  setRosConfig((prev) => ({
                    ...prev,
                    mapTopic: e.target.value || '/map',
                  }))
                }
                className="w-full px-2.5 py-1.5 bg-[#111827] border border-[#1E293B] rounded font-mono text-slate-100 focus:outline-none focus:border-cyan-500"
              />
            </div>

            <div>
              <label className="block text-slate-400 mb-1">Robot Base Frame</label>
              <input
                type="text"
                value={rosConfig.baseFrame}
                onChange={(e) =>
                  setRosConfig((prev) => ({
                    ...prev,
                    baseFrame: e.target.value || 'base_link',
                  }))
                }
                className="w-full px-2.5 py-1.5 bg-[#111827] border border-[#1E293B] rounded font-mono text-slate-100 focus:outline-none focus:border-cyan-500"
              />
            </div>

            <div>
              <label className="block text-slate-400 mb-1">Robot Radius (m)</label>
              <input
                type="number"
                step={0.02}
                min={0.1}
                max={1.0}
                value={rosConfig.robotRadius}
                onChange={(e) =>
                  setRosConfig((prev) => ({
                    ...prev,
                    robotRadius: Math.max(0.1, Number(e.target.value) || 0.22),
                  }))
                }
                className="w-full px-2.5 py-1.5 bg-[#111827] border border-[#1E293B] rounded font-mono tabular-nums text-slate-100 focus:outline-none focus:border-cyan-500"
              />
            </div>

            <div>
              <label className="block text-slate-400 mb-1">Execution Mode</label>
              <button
                type="button"
                onClick={() =>
                  setRosConfig((prev) => ({
                    ...prev,
                    useNav2Action: !prev.useNav2Action,
                  }))
                }
                className={`w-full py-1.5 px-2.5 rounded border font-mono text-[11px] transition-colors whitespace-nowrap shrink-0 truncate ${
                  rosConfig.useNav2Action
                    ? 'bg-emerald-950/50 border-emerald-500/60 text-emerald-300'
                    : 'bg-amber-950/50 border-amber-500/60 text-amber-300'
                }`}
              >
                {rosConfig.useNav2Action
                  ? 'Nav2 ActionClient'
                  : 'Direct /cmd_vel'}
              </button>
            </div>
          </div>
        </div>

        {/* File Selector Tabs */}
        <div className="flex items-center gap-1.5 px-4 py-2.5 bg-[#0E1422] border-b border-[#1E293B] overflow-x-auto">
          {generatedFiles.map((file) => {
            const isSelected = file.id === activeFile.id;
            return (
              <button
                key={file.id}
                type="button"
                onClick={() => setActiveFileId(file.id)}
                className={`flex items-center gap-1.5 px-3 py-1.5 rounded-md text-xs font-mono transition-colors whitespace-nowrap shrink-0 ${
                  isSelected
                    ? 'bg-cyan-500/20 text-cyan-300 border border-cyan-500/50 font-semibold'
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800/60 border border-transparent'
                }`}
              >
                <FileCode className="w-3.5 h-3.5" />
                <span>{file.filename}</span>
              </button>
            );
          })}
        </div>

        {/* Active File Subheader & Actions */}
        <div className="flex flex-wrap items-center justify-between gap-4 px-4 py-3 bg-[#111827] border-b border-[#1E293B]">
          <div>
            <div className="flex items-center gap-2 text-xs font-mono text-cyan-400">
              <Terminal className="w-3.5 h-3.5" />
              <span>{activeFile.relativePath}</span>
            </div>
            <p className="text-xs text-slate-400 mt-0.5">
              {activeFile.description}
            </p>
          </div>

          <div className="flex items-center gap-2 shrink-0">
            <button
              type="button"
              onClick={() => handleCopyFile(activeFile.content, activeFile.id)}
              className="flex items-center gap-1.5 px-3 py-1.5 text-xs font-medium text-slate-200 bg-slate-800 hover:bg-slate-700 border border-slate-700 rounded transition-colors whitespace-nowrap shrink-0"
            >
              {copiedId === activeFile.id ? (
                <>
                  <Check className="w-3.5 h-3.5 text-emerald-400" />
                  <span>Copied {activeFile.filename}</span>
                </>
              ) : (
                <>
                  <Copy className="w-3.5 h-3.5 text-slate-400" />
                  <span>Copy File</span>
                </>
              )}
            </button>

            <button
              type="button"
              onClick={handleDownloadSingleFile}
              className="flex items-center gap-1.5 px-3 py-1.5 text-xs font-medium text-slate-200 bg-slate-800 hover:bg-slate-700 border border-slate-700 rounded transition-colors whitespace-nowrap shrink-0"
            >
              <Download className="w-3.5 h-3.5 text-cyan-400" />
              <span>Save File</span>
            </button>
          </div>
        </div>

        {/* Source Code Viewer */}
        <pre className="p-5 text-xs font-mono text-slate-200 overflow-x-auto max-h-[640px] overflow-y-auto leading-relaxed bg-[#080B12]">
          <code>{activeFile.content}</code>
        </pre>
      </div>
    </div>
  );
};
