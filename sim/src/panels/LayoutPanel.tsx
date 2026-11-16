// Workcell layout editor (a draft): drag stations in the 3D view or type numbers, checked live with the
// firmware's own rules (core/stations.ts), then Send to arm. The arm validates again and has the last word.
import { useRef, useState } from 'react';
import { useArm } from '../store/useArm';
import { cadLayout, MAP_COLOURS, STATION_TYPES, nameOk, validate, type Layout, type StationType } from '../core/stations';
import { STATION } from '../core/config';

const FIELDS = ['x', 'y', 'z', 'roll', 'aboveDz'] as const;

function isLayout(v: unknown): v is Layout {
  const l = v as Layout;
  return !!l && Array.isArray(l.stations) && typeof l.map === 'object' &&
    l.stations.every(s => typeof s.name === 'string' && STATION_TYPES.includes(s.type) && FIELDS.every(k => Number.isFinite(s[k])));
}

export function LayoutPanel() {
  const { layout, edited, stErrors, selected, editLayout, select, loadLayout, sendLayout, connected } = useArm();
  const [msg, setMsg] = useState('');
  const [name, setName] = useState('REJECT');
  const [type, setType] = useState<StationType>('BIN');
  const file = useRef<HTMLInputElement>(null);
  const pickIdx = layout.stations.findIndex(s => s.type === 'PICK');

  const exportJson = () => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([JSON.stringify(layout, null, 2)], { type: 'application/json' }));
    a.download = 'workcell-layout.json'; a.click(); URL.revokeObjectURL(a.href);
  };
  const importJson = async (f: File) => {
    try {
      const v = JSON.parse(await f.text());
      if (!isLayout(v)) throw new Error('not a layout file');
      editLayout(() => v); setMsg(`imported ${f.name}`);
    } catch (e) { setMsg(`import failed: ${(e as Error).message}`); }
  };
  const add = () => {
    if (!nameOk(name) || layout.stations.some(s => s.name === name)) return setMsg(`name: 1–${STATION.nameLen} chars A-Z 0-9 _, unique`);
    editLayout(l => { l.stations.push({ name, type, x: 150, y: 150, z: type === 'PARK' ? 170 : 80, roll: 45, aboveDz: STATION.aboveDz }); return l; });
    select(name);
  };

  return (
    <section className="panel">
      <h2>Workcell layout {edited && <span className="warn-text small">· edited, not sent</span>}</h2>
      <div className="row">
        <button disabled={!connected} onClick={async () => { await loadLayout(); setMsg('loaded from arm'); }}>Load from arm</button>
        <button className="primary" disabled={!connected} onClick={async () => setMsg(await sendLayout())}>Send to arm</button>
        <button onClick={exportJson}>Export</button>
        <button onClick={() => file.current?.click()}>Import</button>
        <input ref={file} type="file" accept="application/json,.json" hidden onChange={e => { const f = e.target.files?.[0]; if (f) void importJson(f); e.target.value = ''; }} />
        <button className="ghost" onClick={() => editLayout(() => cadLayout())}>Factory layout</button>
      </div>
      <table className="poses">
        <thead><tr><th>Station</th>{FIELDS.map(k => <th key={k}>{k === 'aboveDz' ? 'above' : k}</th>)}<th>check</th><th /></tr></thead>
        <tbody>
          {layout.stations.map((s, i) => {
            const why = stErrors[s.name] ?? validate(s, s.aboveDz, s.type, layout.stations, i);
            return (
              <tr key={s.name} className={selected === s.name ? 'sel' : ''} onClick={() => select(s.name)}>
                <td title={s.type}>{s.name}<span className="muted small"> {s.type[0]}</span></td>
                {FIELDS.map(k => (
                  <td key={k}><input className="cell" type="number" aria-label={`${s.name} ${k}`} value={s[k]} step={k === 'roll' ? 5 : 1}
                    onChange={e => editLayout(l => { l.stations[i][k] = Number(e.target.value); return l; })} /></td>
                ))}
                <td className={why === 'OK' ? 'ok-text' : 'bad-text'}>{why === 'OK' ? '✓' : why}</td>
                <td><button disabled={i === 0 || i === pickIdx} aria-label={`delete ${s.name}`}
                  onClick={e => { e.stopPropagation(); editLayout(l => { l.stations.splice(i, 1); for (const c of MAP_COLOURS) if (l.map[c] === s.name) l.map[c] = null; return l; }); }}>×</button></td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <div className="row">
        <input aria-label="station name" value={name} maxLength={STATION.nameLen} size={8} onChange={e => setName(e.target.value.toUpperCase())} />
        <select aria-label="station type" value={type} onChange={e => setType(e.target.value as StationType)}>
          {STATION_TYPES.filter(t => t !== 'PICK').map(t => <option key={t}>{t}</option>)}
        </select>
        <button onClick={add}>Add</button>
        {MAP_COLOURS.map(c => (
          <label key={c} className="num">{c} →
            <select value={layout.map[c] ?? '-'} onChange={e => editLayout(l => { l.map[c] = e.target.value === '-' ? null : e.target.value; return l; })}>
              <option value="-">—</option>
              {layout.stations.filter(s => s.type === 'BIN').map(s => <option key={s.name}>{s.name}</option>)}
            </select>
            {stErrors[`map ${c}`] && <span className="bad-text">{stErrors[`map ${c}`]}</span>}
          </label>
        ))}
      </div>
      <div className="muted small">Drag markers on the baseplate (green = the selected station would be valid there). {msg}</div>
    </section>
  );
}
