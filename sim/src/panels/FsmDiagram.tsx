// The 11 states of fsm.c with the active one highlighted.
const N: Record<string, [number, number]> = {
  BOOT: [50, 24], HOMING: [162, 24], IDLE: [278, 24], TEACH: [392, 24],
  READ_COLOUR: [278, 92], PICK: [392, 160], VERIFY: [278, 160], PLACE: [162, 160], RETURN: [162, 92],
  ERROR: [70, 214], ESTOP: [380, 214],
};
const BW = 84, BH = 24;
const E: [string, string][] = [
  ['BOOT', 'HOMING'], ['HOMING', 'IDLE'], ['IDLE', 'TEACH'], ['TEACH', 'IDLE'], ['IDLE', 'READ_COLOUR'],
  ['READ_COLOUR', 'PICK'], ['PICK', 'VERIFY'], ['VERIFY', 'PICK'], ['VERIFY', 'PLACE'], ['PLACE', 'RETURN'],
  ['RETURN', 'IDLE'], ['RETURN', 'HOMING'], ['ERROR', 'HOMING'], ['ESTOP', 'HOMING'],
];

export function FsmDiagram({ active }: { active?: string }) {
  return (
    <svg viewBox="0 0 442 244" className="fsm" role="img" aria-label={`State machine, active: ${active ?? 'none'}`}>
      <defs>
        <marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M0,0 L10,5 L0,10 z" fill="currentColor" />
        </marker>
      </defs>
      {E.map(([a, b], i) => {
        const [x1, y1] = N[a], [x2, y2] = N[b];
        const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy), ox = (-dy / len) * 3, oy = (dx / len) * 3;
        const t = Math.min(BW / 2 / Math.abs(dx || 1e-9), BH / 2 / Math.abs(dy || 1e-9)) + 0.02;   // clip at the box edge
        return <line key={i} x1={x1 + dx * t + ox} y1={y1 + dy * t + oy} x2={x2 - dx * t + ox} y2={y2 - dy * t + oy} className="edge" markerEnd="url(#arr)" />;
      })}
      {Object.entries(N).map(([s, [x, y]]) => (
        <g key={s} className={`node ${s === active ? 'active' : ''} ${s === 'ERROR' || s === 'ESTOP' ? 'alarm' : ''}`}>
          <rect x={x - BW / 2} y={y - BH / 2} width={BW} height={BH} rx={6} />
          <text x={x} y={y + 4} textAnchor="middle">{s}</text>
        </g>
      ))}
      <text x={221} y={240} textAnchor="middle" className="hint">any state → ESTOP · errors wait for RESET</text>
    </svg>
  );
}
