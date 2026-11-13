// The overhead camera of the 3D twin: renders the scene from above into an off-screen target and reads
// it back as an RGBA frame for the vision pipeline. Also draws the camera, its field of view and the
// pick zone (hidden while it takes a picture). Lives inside the Workcell's robot-frame group (mm, z up).
import { Html, Line } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { CAMERA, ZONE } from '../vision/config';
import type { Frame } from '../vision/detect';
import { applyH, cameraHomography } from '../vision/geometry';

// render targets hold linear colour (no tone mapping, no sRGB encoding): encode it like a camera would
const SRGB = new Uint8Array(256).map((_, i) => {
  const c = i / 255;
  return Math.round(255 * (c <= 0.0031308 ? 12.92 * c : 1.055 * c ** (1 / 2.4) - 0.055));
});
const toScene = (x: number, y: number, z: number) => new THREE.Vector3(x / 100, z / 100, -y / 100);   // robot mm → three.js world (Workcell group)

type Waiter = { resolve: (f: Frame) => void; reject: (e: Error) => void };
let grab: (() => Frame) | null = null;
const waiting: Waiter[] = [];

/** the next rendered frame of the overhead camera */
export function captureFrame(timeoutMs = 2000): Promise<Frame> {
  return new Promise((resolve, reject) => {
    if (!grab) return reject(new Error('camera: no 3D view'));
    const w: Waiter = { resolve, reject };
    waiting.push(w);
    setTimeout(() => {
      const i = waiting.indexOf(w);
      if (i >= 0) { waiting.splice(i, 1); reject(new Error('camera: no frame (tab in the background?)')); }
    }, timeoutMs);
  });
}

export function VisionCamera() {
  const gl = useThree(s => s.gl), scene = useThree(s => s.scene);
  const gizmo = useRef<THREE.Group>(null);
  const { rt, cam, buf } = useMemo(() => {
    const cam = new THREE.PerspectiveCamera(CAMERA.fovDeg, CAMERA.width / CAMERA.height, 0.1, 20);
    cam.position.copy(toScene(CAMERA.x, CAMERA.y, CAMERA.z));
    cam.up.set(1, 0, 0);                                        // image up = robot +x
    cam.lookAt(toScene(CAMERA.x, CAMERA.y, 0));
    cam.updateMatrixWorld();
    return { rt: new THREE.WebGLRenderTarget(CAMERA.width, CAMERA.height), cam, buf: new Uint8Array(CAMERA.width * CAMERA.height * 4) };
  }, []);

  useEffect(() => {
    grab = () => {
      const { width: w, height: h } = CAMERA;
      if (gizmo.current) gizmo.current.visible = false;
      gl.setRenderTarget(rt);
      gl.render(scene, cam);
      gl.readRenderTargetPixels(rt, 0, 0, w, h, buf);
      gl.setRenderTarget(null);
      if (gizmo.current) gizmo.current.visible = true;
      const data = new Uint8ClampedArray(w * h * 4);
      for (let v = 0; v < h; v++) {                              // GL rows go bottom-up
        const src = (h - 1 - v) * w * 4, dst = v * w * 4;
        for (let k = 0; k < w * 4; k += 4) {
          data[dst + k] = SRGB[buf[src + k]]; data[dst + k + 1] = SRGB[buf[src + k + 1]];
          data[dst + k + 2] = SRGB[buf[src + k + 2]]; data[dst + k + 3] = 255;
        }
      }
      return { width: w, height: h, data };
    };
    return () => { grab = null; rt.dispose(); };
  }, [gl, scene, cam, rt, buf]);

  // serve capture requests on the next frame, once the scene shows the latest telemetry
  useFrame(() => {
    if (!waiting.length || !grab) return;
    const f = grab();
    waiting.splice(0).forEach(w => w.resolve(f));
  });

  const { fov, zone } = useMemo(() => {
    const H0 = cameraHomography(CAMERA, 0), { width: w, height: h } = CAMERA;
    const fov = [[0, 0], [w, 0], [w, h], [0, h]].map(p => applyH(H0, p as [number, number]));
    const zone = [[ZONE.xMin, ZONE.yMin], [ZONE.xMax, ZONE.yMin], [ZONE.xMax, ZONE.yMax], [ZONE.xMin, ZONE.yMax], [ZONE.xMin, ZONE.yMin]];
    return { fov, zone };
  }, []);
  const eye: [number, number, number] = [CAMERA.x, CAMERA.y, CAMERA.z];

  return (
    <group ref={gizmo}>
      <mesh position={eye}>
        <boxGeometry args={[46, 60, 30]} />
        <meshStandardMaterial color="#20242b" />
      </mesh>
      <mesh position={[CAMERA.x, CAMERA.y, CAMERA.z - 18]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[11, 11, 10, 20]} />
        <meshStandardMaterial color="#59c3c3" emissive="#1d4f4f" />
      </mesh>
      {fov.map(([x, y], i) => <Line key={i} points={[eye, [x, y, 1]]} color="#59c3c3" lineWidth={1} transparent opacity={0.35} />)}
      <Line points={[...fov, fov[0]].map(([x, y]) => [x, y, 1] as [number, number, number])} color="#59c3c3" lineWidth={1} transparent opacity={0.5} />
      <Line points={zone.map(([x, y]) => [x, y, 1.5] as [number, number, number])} color="#e3a33b" lineWidth={1.5} dashed dashSize={8} gapSize={6} />
      <Html position={[CAMERA.x, CAMERA.y, CAMERA.z + 30]} center style={{ color: '#9fdede', fontSize: 11, whiteSpace: 'nowrap', pointerEvents: 'none' }}>
        overhead camera
      </Html>
    </group>
  );
}
