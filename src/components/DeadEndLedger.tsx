import React from 'react';
import { Crosshair, ShieldAlert } from 'lucide-react';
import { DeadEndRecord } from '../types/slam';

interface DeadEndLedgerProps {
  records: DeadEndRecord[];
  highlightedId: number | null;
  onSelectDeadEnd: (id: number | null) => void;
  totalLockedCells: number;
}

export const DeadEndLedger: React.FC<DeadEndLedgerProps> = ({
  records,
  highlightedId,
  onSelectDeadEnd,
  totalLockedCells,
}) => {
  const totalSavedSteps = records.reduce((acc, r) => acc + r.savedRevisitCost, 0);

  return (
    <div className="bg-[#111827] border border-[#1E293B] rounded-lg overflow-hidden">
      <div className="flex flex-wrap items-center justify-between gap-4 px-4 py-3 border-b border-[#1E293B] bg-[#0E1422]">
        <div>
          <h3 className="text-sm font-semibold text-slate-100">
            Memorized Dead-End Taboo Ledger
          </h3>
          <p className="text-xs text-slate-400 mt-0.5">
            Permanently sealed cul-de-sacs and blind bays excluded from all future scanning and pathfinding
          </p>
        </div>
        <div className="flex items-center gap-4 text-xs font-mono tabular-nums text-slate-300">
          <span>LOCKED ZONES: {records.length}</span>
          <span aria-hidden="true" className="text-slate-600">
            ·
          </span>
          <span>PRUNED CELLS: {totalLockedCells}</span>
          <span aria-hidden="true" className="text-slate-600">
            ·
          </span>
          <span className="text-emerald-400">
            SAVED BACKTRACK STEPS: ~{totalSavedSteps}
          </span>
        </div>
      </div>

      {records.length === 0 ? (
        <div className="p-8 text-center">
          <ShieldAlert className="w-7 h-7 text-slate-500 mx-auto mb-2" />
          <p className="text-sm text-slate-300 font-medium">
            No dead-end pockets locked yet
          </p>
          <p className="text-xs text-slate-500 mt-1 max-w-md mx-auto">
            Start the exploration simulation. As the AMR scans inside corridors or blind bays and resolves all local unknown cells, the Topological Dead-End Pruner will automatically lock and log each zone here.
          </p>
        </div>
      ) : (
        <div className="overflow-x-auto max-h-72 overflow-y-auto">
          <table className="w-full text-left border-collapse text-xs">
            <thead>
              <tr className="border-b border-[#1E293B] text-slate-400 font-mono bg-[#0B0F1A]">
                <th className="py-2.5 px-4 font-medium">ZONE ID</th>
                <th className="py-2.5 px-4 font-medium">TOPOLOGY TYPE</th>
                <th className="py-2.5 px-4 font-medium">THROAT ENTRANCE</th>
                <th className="py-2.5 px-4 font-medium">TERMINAL TIP</th>
                <th className="py-2.5 px-4 font-medium text-right">LOCKED CELLS</th>
                <th className="py-2.5 px-4 font-medium text-right">SEALED AT TICK</th>
                <th className="py-2.5 px-4 font-medium text-right">INSPECT</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-[#1E293B] font-mono tabular-nums">
              {records.map((rec) => {
                const isSelected = highlightedId === rec.id;
                return (
                  <tr
                    key={rec.id}
                    onClick={() => onSelectDeadEnd(isSelected ? null : rec.id)}
                    className={`cursor-pointer transition-colors ${
                      isSelected
                        ? 'bg-rose-950/40 text-rose-200'
                        : 'hover:bg-slate-800/50 text-slate-200'
                    }`}
                  >
                    <td className="py-2.5 px-4 font-semibold text-rose-400">
                      DE-{String(rec.id).padStart(2, '0')}
                    </td>
                    <td className="py-2.5 px-4 font-sans text-slate-300">
                      {rec.classification}
                    </td>
                    <td className="py-2.5 px-4 text-slate-300">
                      ({rec.throatCoord.x}, {rec.throatCoord.y})
                    </td>
                    <td className="py-2.5 px-4 text-slate-400">
                      ({rec.tipCoord.x}, {rec.tipCoord.y})
                    </td>
                    <td className="py-2.5 px-4 text-right font-semibold">
                      {rec.cellCount} cells
                    </td>
                    <td className="py-2.5 px-4 text-right text-slate-400">
                      T+{rec.lockedAtTick}
                    </td>
                    <td className="py-2.5 px-4 text-right">
                      <button
                        type="button"
                        onClick={(e) => {
                          e.stopPropagation();
                          onSelectDeadEnd(isSelected ? null : rec.id);
                        }}
                        className={`inline-flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-sans font-medium transition-colors whitespace-nowrap shrink-0 ${
                          isSelected
                            ? 'bg-rose-600 text-white'
                            : 'bg-slate-800 text-slate-300 hover:bg-slate-700'
                        }`}
                      >
                        <Crosshair className="w-3 h-3" />
                        <span>{isSelected ? 'Highlighted' : 'Highlight'}</span>
                      </button>
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
};
