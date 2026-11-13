// IR beam across the cradle (green = blocked, red = clear) and the 3 limit switches (lit when pressed).
import { Html } from '@react-three/drei';
import { CELL, H_SHOULDER } from '../core/config';

export function Sensors({ ir, limits }: { ir: boolean; limits: boolean[] }) {
  const { x } = CELL.cube, z = CELL.cubeSize / 2;
  const sw = [[-60, -70, 20], [0, -60, H_SHOULDER], [20, -60, H_SHOULDER + 40]];   // J1 base, J2 shoulder, J3 (motor on J2 axis)
  return (
    <group>
      <mesh position={[x, 0, z]} rotation={[0, 0, 0]}>
        <boxGeometry args={[2, 90, 2]} />
        <meshBasicMaterial color={ir ? '#2ecc71' : '#e74c3c'} />
      </mesh>
      <mesh position={[x, -48, z]}>
        <boxGeometry args={[14, 10, 18]} />
        <meshStandardMaterial color="#222" />
      </mesh>
      {sw.map(([sx, sy, sz], i) => (
        <group key={i} position={[sx, sy, sz]}>
          <mesh>
            <boxGeometry args={[12, 12, 12]} />
            <meshStandardMaterial color={limits[i] ? '#ffd23f' : '#444'} emissive={limits[i] ? '#ffb000' : '#000'} />
          </mesh>
          <Html distanceFactor={6} position={[0, -10, 12]} style={{ fontSize: 10, color: '#ccc', pointerEvents: 'none' }}>J{i + 1}</Html>
        </group>
      ))}
    </group>
  );
}
