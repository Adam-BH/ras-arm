// Vision UI state: the live camera frame and what was found in it, the current sort run.
import { create } from 'zustand';
import { captureFrame } from '../scene/VisionCamera';
import { CAMERA, CUBE_PLANE_Z } from '../vision/config';
import { detect, type Detection, type Frame } from '../vision/detect';
import { cameraHomography } from '../vision/geometry';
import { VisionSorter, type SortStatus } from '../vision/sorter';
import { useArm } from './useArm';

// the virtual camera is calibrated from its own model; a webcam would get this from 4 clicked points
export const PIXEL_TO_TABLE = cameraHomography(CAMERA, CUBE_PLANE_Z);

type VisionState = {
  frame: Frame | null; detections: Detection[]; cameraError: string;
  sorter: VisionSorter | null; sort: SortStatus | null;
  look(): Promise<void>;
  startSort(): void;
  stopSort(): void;
};

export const useVision = create<VisionState>((set, get) => ({
  frame: null, detections: [], cameraError: '', sorter: null, sort: null,

  async look() {
    try {
      const frame = await captureFrame();
      set({ frame, detections: detect(frame, PIXEL_TO_TABLE), cameraError: '' });
    } catch (e) { set({ cameraError: (e as Error).message }); }
  },

  startSort() {
    const link = useArm.getState().link;
    if (!link || get().sorter) return;
    const sorter = new VisionSorter({
      send: cmd => useArm.getState().send(cmd),
      onEvent: cb => link.onEvent(cb),
      capture: async () => {
        const frame = await captureFrame();
        set({ frame, detections: detect(frame, PIXEL_TO_TABLE) });
        return frame;
      },
      pixelToTable: PIXEL_TO_TABLE,
      poses: () => useArm.getState().poses,
      wait: ms => new Promise(r => setTimeout(r, ms)),
    }, sort => set({ sort }));
    set({ sorter, sort: sorter.status });
    void sorter.run().finally(() => set({ sorter: null }));
  },

  stopSort() { get().sorter?.stop(); },
}));
