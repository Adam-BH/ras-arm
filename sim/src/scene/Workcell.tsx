// The workcell to scale (mm): baseplate, arm, sensors, cubes, and the STATIONS from the layout
// (cradle, bins, park poses) as draggable markers. The baseplate is shaded green/red where the selected
// station would pass the firmware's validation (same rules: core/stations.ts).
// The scene group is rotated so the robot frame (x → cradle, y left, z up) sits in three.js' y-up world.
import { useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, type ThreeEvent } from '@react-three/fiber';
import { Html, OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { Arm } from './Arm';
import { Cube } from './Cube';
import { Sensors } from './Sensors';
import { CELL, GRIP_CLOSED_MM, STATION } from '../core/config';
import { MAP_COLOURS, validate, type Layout, type Station } from '../core/stations';
import { useArm, useVirtualFw } from '../store/useArm';

const BIN = 110, BIN_H = 60;
const PLATE = { x0: -220, x1: 340, y0: -320, y1: 320 }, CELL_MM = 20;   // reach shading grid
const MAP_TINT: Record<string, string> = { RED: '#7a2b2b', BLUE: '#25407a', UNKNOWN: '#5b3f8c' };
const yaw = (x: number, y: number) => Math.round(Math.atan2(y, x) * 180 / Math.PI);

/** green where the selected station (or a new bin) would be valid, red elsewhere */
function useReachTexture(layout: Layout, selected: string | null) {
  return useMemo(() => {
    const i = layout.stations.findIndex(s => s.name === selected);
    const s: Pick<Station, 'type' | 'z' | 'roll' | 'aboveDz'> = layout.stations[i] ?? { type: 'BIN', z: CELL.binA.z, roll: 0, aboveDz: STATION.aboveDz };
    const nx = (PLATE.x1 - PLATE.x0) / CELL_MM, ny = (PLATE.y1 - PLATE.y0) / CELL_MM, data = new Uint8Array(nx * ny * 4);
    for (let r = 0; r < ny; r++) for (let c = 0; c < nx; c++) {
      const x = PLATE.x0 + (c + 0.5) * CELL_MM, y = PLATE.y0 + (r + 0.5) * CELL_MM;
      const ok = validate({ x, y, z: s.z, roll: s.type === 'PARK' ? s.roll : yaw(x, y) }, s.aboveDz, s.type, layout.stations, i) === 'OK';
      data.set(ok ? [40, 170, 90, 80] : [190, 50, 50, 45], (r * nx + c) * 4);
    }
    const t = new THREE.DataTexture(data, nx, ny);
    t.magFilter = THREE.NearestFilter; t.needsUpdate = true;
    return t;
  }, [layout, selected]);
}

function Bin({ tint }: { tint: string }) {
  return (
    <>
      {[[0, BIN / 2, BIN, 4], [0, -BIN / 2, BIN, 4], [BIN / 2, 0, 4, BIN], [-BIN / 2, 0, 4, BIN]].map(([x, yy, w, d], i) => (
        <mesh key={i} position={[x, yy, BIN_H / 2]}>
          <boxGeometry args={[w, d, BIN_H]} />
          <meshStandardMaterial color={tint} transparent opacity={0.55} />
        </mesh>
      ))}
    </>
  );
}
function Cradle() {                                   // a U block holding the cube over the colour sensor
  return (
    <>
      {[-1, 1].map(s => (
        <mesh key={s} position={[0, s * 20, 8]}><boxGeometry args={[40, 8, 16]} /><meshStandardMaterial color="#5b6577" /></mesh>
      ))}
      <mesh position={[0, 0, 1]}><boxGeometry args={[14, 14, 2]} /><meshStandardMaterial color="#9b59b6" /></mesh>
    </>
  );
}

export function Workcell() {
  const tel = useArm(s => s.tel);
  const cubeColour = useArm(s => s.cubeColour);
  const { layout, stErrors, selected, select, editLayout } = useArm();
  const fw = useVirtualFw();
  const joints = tel?.joints ?? [0, 0, 60, 0, 120, 0, 45];
  const reach = useReachTexture(layout, selected);
  const group = useRef<THREE.Group>(null);
  const [drag, setDrag] = useState<string | null>(null);
  const pick = layout.stations.find(s => s.type === 'PICK') ?? { ...CELL.cube };

  useEffect(() => {                                   // release anywhere ends the drag
    if (!drag) return;
    const up = () => setDrag(null);
    window.addEventListener('pointerup', up);
    return () => window.removeEventListener('pointerup', up);
  }, [drag]);

  const onMove = (e: ThreeEvent<PointerEvent>) => {
    if (!drag || !group.current) return;
    const p = group.current.worldToLocal(e.point.clone());
    editLayout(l => {
      const s = l.stations.find(n => n.name === drag);
      if (s) { s.x = Math.round(p.x); s.y = Math.round(p.y); if (s.type !== 'PARK') s.roll = yaw(s.x, s.y); }   // fingers stay tangent (J6 = 0)
      return l;
    });
  };

  // where is the cube? virtual: the simulated world knows. USB: infer from state, IR and grip.
  let held: string | null = null, inCradle: string | null = null;
  if (fw) {
    const c = fw.world.cube;
    if (c?.where === 'gripper') held = c.colour; else if (c) inCradle = c.colour;
  } else if (tel) {
    const holding = ['PICK', 'VERIFY', 'PLACE'].includes(tel.state) && tel.joints[6] <= GRIP_CLOSED_MM + 1 && !tel.ir;
    if (holding) held = cubeColour;
    else if (tel.ir) inCradle = ['RED', 'BLUE'].includes(cubeColour) && tel.state !== 'IDLE' ? cubeColour : 'UNKNOWN';
  }

  return (
    <Canvas camera={{ position: [4.2, 3.6, 4.6], fov: 42 }} style={{ background: 'var(--scene-bg)' }}>
      <ambientLight intensity={0.55} />
      <directionalLight position={[4, 8, 3]} intensity={1.1} />
      <group ref={group} rotation={[-Math.PI / 2, 0, 0]} scale={0.01}>
        <mesh position={[60, 0, -5]}>
          <boxGeometry args={[560, 640, 10]} />
          <meshStandardMaterial color="#2a2f38" />
        </mesh>
        <gridHelper args={[600, 12, '#3d4552', '#323946']} rotation={[Math.PI / 2, 0, 0]} position={[60, 0, 0.5]} />
        <mesh position={[(PLATE.x0 + PLATE.x1) / 2, (PLATE.y0 + PLATE.y1) / 2, 0.8]} onPointerMove={onMove}>
          <planeGeometry args={[PLATE.x1 - PLATE.x0, PLATE.y1 - PLATE.y0]} />
          <meshBasicMaterial map={reach} transparent depthWrite={false} />
        </mesh>
        {layout.stations.map((s, i) => {
          const why = stErrors[s.name] ?? validate(s, s.aboveDz, s.type, layout.stations, i);
          const from = MAP_COLOURS.filter(c => layout.map[c] === s.name);
          const count = fw?.world.bins[s.name];
          return (
            <group key={s.name} position={[s.x, s.y, 0]}
              onPointerDown={e => { e.stopPropagation(); select(s.name); setDrag(s.name); }}>
              <group rotation={[0, 0, s.roll * Math.PI / 180]}>
                {s.type === 'BIN' && <Bin tint={from.length ? MAP_TINT[from[0]] : '#555c66'} />}
                {s.type === 'PICK' && <Cradle />}
              </group>
              {s.type === 'PARK' && (
                <mesh position={[0, 0, s.z]}><sphereGeometry args={[9, 16, 12]} /><meshStandardMaterial color="#e3a33b" /></mesh>
              )}
              <Html position={[0, 0, (s.type === 'PARK' ? s.z : BIN_H) + 20]} center
                style={{ color: why !== 'OK' ? '#ff6b6b' : selected === s.name ? '#e3a33b' : '#ddd', fontSize: 12, whiteSpace: 'nowrap', pointerEvents: 'none' }}>
                {s.name}{from.length ? ` ← ${from.join('/')}` : ''}{count !== undefined ? ` · ${count}` : ''}{why !== 'OK' ? ` ✕ ${why}` : ''}
              </Html>
            </group>
          );
        })}
        <Arm joints={joints} held={held} />
        <Sensors ir={tel?.ir ?? false} limits={tel?.limits ?? [false, false, false]} />
        {inCradle && <Cube colour={inCradle} x={pick.x} y={pick.y} />}
      </group>
      <OrbitControls target={[0.6, 0.9, 0]} makeDefault enabled={!drag} />
    </Canvas>
  );
}
