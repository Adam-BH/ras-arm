// The workcell to scale (mm): baseplate, cradle at r 220, bins at ±90°, arm, sensors, cubes.
// The scene group is rotated so the robot frame (x → cradle, y left, z up) sits in three.js' y-up world.
import { Canvas } from '@react-three/fiber';
import { Html, OrbitControls } from '@react-three/drei';
import { Arm } from './Arm';
import { Cube } from './Cube';
import { Sensors } from './Sensors';
import { VisionCamera } from './VisionCamera';
import { CELL, GRIP_CLOSED_MM } from '../core/config';
import { useArm, useVirtualFw } from '../store/useArm';

const BIN = 110, BIN_H = 60;

function Bin({ y, label, count, colour }: { y: number; label: string; count?: number; colour: string }) {
  return (
    <group position={[0, y, 0]}>
      {[[0, BIN / 2, BIN, 4], [0, -BIN / 2, BIN, 4], [BIN / 2, 0, 4, BIN], [-BIN / 2, 0, 4, BIN]].map(([x, yy, w, d], i) => (
        <mesh key={i} position={[x, yy, BIN_H / 2]}>
          <boxGeometry args={[w, d, BIN_H]} />
          <meshStandardMaterial color={colour} transparent opacity={0.55} />
        </mesh>
      ))}
      <Html position={[0, 0, BIN_H + 20]} center style={{ color: '#ddd', fontSize: 12, whiteSpace: 'nowrap', pointerEvents: 'none' }}>
        {label}{count !== undefined ? ` · ${count}` : ''}
      </Html>
    </group>
  );
}

export function Workcell() {
  const tel = useArm(s => s.tel);
  const cubeColour = useArm(s => s.cubeColour);
  const fw = useVirtualFw();
  const joints = tel?.joints ?? [0, 0, 60, 0, 120, 0, 45];

  // where is the cube? virtual: the simulated world knows. USB: infer from state, IR and grip.
  let held: string | null = null, inCradle: string | null = null;
  if (fw) {
    held = fw.world.held; inCradle = fw.world.cradle;
  } else if (tel) {
    const holding = ['PICK', 'VERIFY', 'PLACE'].includes(tel.state) && tel.joints[6] <= GRIP_CLOSED_MM + 1 && !tel.ir;
    if (holding) held = cubeColour;
    else if (tel.ir) inCradle = ['RED', 'BLUE'].includes(cubeColour) && tel.state !== 'IDLE' ? cubeColour : 'UNKNOWN';
  }

  return (
    <Canvas camera={{ position: [4.2, 3.6, 4.6], fov: 42 }} style={{ background: 'var(--scene-bg)' }}>
      <ambientLight intensity={0.55} />
      <directionalLight position={[4, 8, 3]} intensity={1.1} />
      <group rotation={[-Math.PI / 2, 0, 0]} scale={0.01}>
        <mesh position={[60, 0, -5]}>
          <boxGeometry args={[560, 640, 10]} />
          <meshStandardMaterial color="#2a2f38" />
        </mesh>
        <gridHelper args={[600, 12, '#3d4552', '#323946']} rotation={[Math.PI / 2, 0, 0]} position={[60, 0, 0.5]} />
        {/* cradle: a U block holding the cube over the colour sensor */}
        <group position={[CELL.cube.x, 0, 0]}>
          {[-1, 1].map(s => (
            <mesh key={s} position={[0, s * 20, 8]}>
              <boxGeometry args={[40, 8, 16]} />
              <meshStandardMaterial color="#5b6577" />
            </mesh>
          ))}
          <mesh position={[0, 0, 1]}><boxGeometry args={[14, 14, 2]} /><meshStandardMaterial color="#9b59b6" /></mesh>
        </group>
        <Bin y={CELL.binA.y} label="Bin A (red)" count={fw?.world.bins.A} colour="#7a2b2b" />
        <Bin y={CELL.binB.y} label="Bin B (blue)" count={fw?.world.bins.B} colour="#25407a" />
        <Arm joints={joints} held={held} />
        <Sensors ir={tel?.ir ?? false} limits={tel?.limits ?? [false, false, false]} />
        {inCradle && <Cube colour={inCradle} x={CELL.cube.x} y={0} />}
        {fw?.world.table.map((c, i) => <Cube key={i} colour={c.colour} x={c.x} y={c.y} yaw={c.yaw} />)}
        <VisionCamera />
      </group>
      <OrbitControls target={[0.6, 0.9, 0]} makeDefault />
    </Canvas>
  );
}
