import { CELL } from '../core/config';

export function Cube({ colour, x, y, z = CELL.cubeSize / 2, yaw = 0, opacity = 1 }: { colour: string; x: number; y: number; z?: number; yaw?: number; opacity?: number }) {
  const s = CELL.cubeSize;
  return (
    <mesh position={[x, y, z]} rotation={[0, 0, (yaw * Math.PI) / 180]}>
      <boxGeometry args={[s, s, s]} />
      <meshStandardMaterial color={colour === 'RED' ? '#d33' : colour === 'BLUE' ? '#2a6fdb' : '#aaa'} transparent={opacity < 1} opacity={opacity} />
    </mesh>
  );
}
