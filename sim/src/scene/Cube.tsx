import { CELL } from '../core/config';

export function Cube({ colour, x, y, z = CELL.cubeSize / 2, opacity = 1 }: { colour: string; x: number; y: number; z?: number; opacity?: number }) {
  const s = CELL.cubeSize;
  return (
    <mesh position={[x, y, z]}>
      <boxGeometry args={[s, s, s]} />
      <meshStandardMaterial color={colour === 'RED' ? '#d33' : colour === 'BLUE' ? '#2a6fdb' : colour === 'PURPLE' ? '#9b59b6' : '#aaa'} transparent={opacity < 1} opacity={opacity} />
    </mesh>
  );
}
