// The arm, driven by joint angles (deg; [6] = grip mm). Units: mm, z up (see Workcell).
// Chain: J1 yaw → shoulder (H) → J2 pitch → L1 → J3 pitch → J4 roll → L2 → J5 pitch → J6 roll → L3 + fingers.
import { L1, L2, L3, H_SHOULDER, CELL } from '../core/config';
import type { ReactNode } from 'react';

const R = Math.PI / 180;
const Link = ({ len, w, color }: { len: number; w: number; color: string }) => (
  <mesh position={[0, 0, len / 2]}>
    <boxGeometry args={[w, w, len]} />
    <meshStandardMaterial color={color} metalness={0.2} roughness={0.6} />
  </mesh>
);
const Joint = ({ r = 22, color = '#2b2f36' }: { r?: number; color?: string }) => (
  <mesh /* cylinder axis = y = the pitch axis */>
    <cylinderGeometry args={[r, r, 44, 24]} />
    <meshStandardMaterial color={color} />
  </mesh>
);
const Yaw = ({ deg, children }: { deg: number; children: ReactNode }) => <group rotation={[0, 0, deg * R]}>{children}</group>;
const Pitch = ({ deg, children }: { deg: number; children: ReactNode }) => <group rotation={[0, deg * R, 0]}>{children}</group>;

export function Arm({ joints, held }: { joints: number[]; held?: string | null }) {
  const [j1, j2, j3, j4, j5, j6, grip] = joints;
  const half = Math.max(grip, 0) / 2, s = CELL.cubeSize;
  return (
    <Yaw deg={j1}>
      <mesh position={[0, 0, H_SHOULDER / 2]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[55, 70, H_SHOULDER, 32]} />
        <meshStandardMaterial color="#3a4250" />
      </mesh>
      <group position={[0, 0, H_SHOULDER]}>
        <Joint r={30} />
        <Pitch deg={j2}>
          <Link len={L1} w={40} color="#e3a33b" />
          <group position={[0, 0, L1]}>
            <Joint />
            <Pitch deg={j3}>
              <Yaw deg={j4}>
                <Link len={L2} w={32} color="#e3a33b" />
                <group position={[0, 0, L2]}>
                  <Joint r={16} />
                  <Pitch deg={j5}>
                    <Yaw deg={-j6 /* tool points down: local +z is world −z */}>
                      <Link len={L3 - 30} w={26} color="#8a93a3" />
                      {[-1, 1].map(side => (
                        <mesh key={side} position={[0, side * (half + 4), L3 - 2]}>
                          <boxGeometry args={[14, 8, 44]} />
                          <meshStandardMaterial color="#c9ced6" />
                        </mesh>
                      ))}
                      {held && (
                        <mesh position={[0, 0, L3 + s / 2 - 1]}>
                          <boxGeometry args={[s, s, s]} />
                          <meshStandardMaterial color={held === 'RED' ? '#d33' : '#2a6fdb'} />
                        </mesh>
                      )}
                    </Yaw>
                  </Pitch>
                </group>
              </Yaw>
            </Pitch>
          </group>
        </Pitch>
      </group>
    </Yaw>
  );
}
