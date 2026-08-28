#!/usr/bin/env python3
"""Body keypoints UDP bridge for BODY_SKELETON_RETARGET (phase 1).

Streams shoulder / elbow / wrist / hip + confidence. Does NOT drive the robot.

Packet (CSV, one datagram per frame):
  ts_ms,valid,
  lsx,lsy,lsz,lsc, lex,ley,lez,lec, lwx,lwy,lwz,lwc,
  rsx,rsy,rsz,rsc, rex,rey,rez,rec, rwx,rwy,rwz,rwc,
  lhx,lhy,lhz,lhc, rhx,rhy,rhz,rhc

Adapters (select with --backend):
  hybrik     — HybrIK 29 joints (default; uses conda env hybrik)
  mediapipe  — MediaPipe Pose (optional; install mediapipe)

Controller code must only consume BodyPoseSample / this CSV — never backend names.
"""
from __future__ import annotations

import argparse
import abc
import os
import socket
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

_QUEST_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_QUEST_DIR)
_HYBRIK_ROOT = os.path.join(_REPO_ROOT, 'HybrIK')
_HYBRIK_SCRIPTS = os.path.join(_HYBRIK_ROOT, 'scripts')


@dataclass
class Joint3d:
    xyz: np.ndarray  # (3,)
    conf: float

    def fmt(self) -> str:
        p = self.xyz
        return f'{p[0]:.5f},{p[1]:.5f},{p[2]:.5f},{self.conf:.3f}'


@dataclass
class BodyPoseFrame:
    valid: bool
    left_shoulder: Joint3d
    left_elbow: Joint3d
    left_wrist: Joint3d
    right_shoulder: Joint3d
    right_elbow: Joint3d
    right_wrist: Joint3d
    left_hip: Joint3d
    right_hip: Joint3d
    # Optional 2D for drawing (image pixels). Keys: ls,le,lw,rs,re,rw,lh,rh
    pts2d: Optional[dict] = None


def _zero_joint() -> Joint3d:
    return Joint3d(np.zeros(3, dtype=np.float64), 0.0)


def empty_frame() -> BodyPoseFrame:
    z = _zero_joint()
    return BodyPoseFrame(False, z, z, z, z, z, z, z, z, None)


def pack_csv(ts_ms: int, frame: BodyPoseFrame) -> str:
    parts = [
        str(ts_ms),
        '1' if frame.valid else '0',
        frame.left_shoulder.fmt(),
        frame.left_elbow.fmt(),
        frame.left_wrist.fmt(),
        frame.right_shoulder.fmt(),
        frame.right_elbow.fmt(),
        frame.right_wrist.fmt(),
        frame.left_hip.fmt(),
        frame.right_hip.fmt(),
    ]
    return ','.join(parts)


class PoseAdapter(abc.ABC):
    """Uniform interface; backends swap without changing packet format."""

    @abc.abstractmethod
    def estimate(self, bgr: np.ndarray) -> BodyPoseFrame:
        raise NotImplementedError

    def close(self) -> None:
        pass


class HybrIKPoseAdapter(PoseAdapter):
    """HybrIK SMPL-29 joints → BodyPoseFrame (camera-relative XYZ)."""

    # pred_xyz_jts_29 indices (same as demo_webcam / SEW bridge)
    J_L_HIP, J_R_HIP = 1, 2
    J_L_SHOULDER, J_R_SHOULDER = 16, 17
    J_L_ELBOW, J_R_ELBOW = 18, 19
    J_L_WRIST, J_R_WRIST = 20, 21

    def __init__(self, opt):
        for p in (_HYBRIK_ROOT, _HYBRIK_SCRIPTS):
            if p not in sys.path:
                sys.path.insert(0, p)
        os.chdir(_HYBRIK_ROOT)

        from demo_webcam import (  # noqa: WPS433
            EMASmoother,
            UPPER_BODY,
            LOWER_BODY,
            build_models,
            detect_on_resized,
            draw_skeleton_2d,
            joints_in_skeleton,
            resolve_options,
            run_hybrik,
            track_box,
        )
        import torch

        self.torch = torch
        self.detect_on_resized = detect_on_resized
        self.run_hybrik = run_hybrik
        self.track_box = track_box
        self.draw_skeleton_2d = draw_skeleton_2d
        self.UPPER_BODY = UPPER_BODY
        self.LOWER_BODY = LOWER_BODY
        self.joints_vis = joints_in_skeleton(UPPER_BODY + LOWER_BODY)

        self.opt = resolve_options(opt)
        self.det_model, self.hybrik_model, self.transformation, self.device = build_models(self.opt)
        self.use_fp16 = bool(self.opt.fp16 and self.device.type == 'cuda')
        self.flip_test = not self.opt.no_flip

        if self.device.type == 'cuda':
            print('[body-pose-bridge] GPU warmup...')
            with torch.no_grad():
                dummy = torch.zeros(1, 3, 256, 256, device=self.device)
                _ = run_hybrik(self.hybrik_model, dummy, [0, 0, 256, 256], [128, 128],
                               self.device, False)
                torch.cuda.synchronize()

        smooth = float(getattr(self.opt, 'smooth', 0.28) or 0.0)
        self.xyz_smoother = EMASmoother(smooth) if smooth > 1e-6 else None
        self.box_smoother = EMASmoother(max(0.2, smooth)) if smooth > 1e-6 else None
        self.pts_smoother = EMASmoother(max(0.15, min(smooth, 0.35))) if smooth > 1e-6 else None
        self.prev_box = None
        self.last_raw_box = None
        self.miss_count = 0
        self.frame_idx = 0
        self.hold_frames = int(getattr(self.opt, 'hold_frames', 12))

    def _joint(self, xyz: np.ndarray, idx: int, conf: float) -> Joint3d:
        return Joint3d(np.asarray(xyz[idx], dtype=np.float64).copy(), float(conf))

    def estimate(self, bgr: np.ndarray) -> BodyPoseFrame:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pts = None
        with self.torch.no_grad():
            if self.last_raw_box is None or self.frame_idx % max(1, self.opt.det_interval) == 0:
                raw = self.detect_on_resized(
                    self.det_model, rgb, self.device, self.use_fp16,
                    max_side=self.opt.det_size, score_thr=self.opt.det_thr)
                tracked = self.track_box(raw, self.prev_box)
                if tracked is not None:
                    self.last_raw_box = tracked

            raw_bbox = self.last_raw_box
            if raw_bbox is not None:
                self.miss_count = 0
                if self.box_smoother is not None:
                    tight_bbox = self.box_smoother.update(raw_bbox).tolist()
                else:
                    tight_bbox = [float(x) for x in raw_bbox]
                self.prev_box = tight_bbox
            elif self.prev_box is not None and self.miss_count < self.hold_frames:
                self.miss_count += 1
                tight_bbox = self.prev_box
            else:
                tight_bbox = None
                self.prev_box = None
                self.last_raw_box = None
                self.miss_count = 0
                if self.xyz_smoother is not None:
                    self.xyz_smoother.reset()
                if self.box_smoother is not None:
                    self.box_smoother.reset()
                if self.pts_smoother is not None:
                    self.pts_smoother.reset()

            self.frame_idx += 1
            if tight_bbox is None:
                return empty_frame()

            pose_input, bbox, img_center = self.transformation.test_transform(rgb, tight_bbox)
            pose_input = pose_input.to(self.device, non_blocking=True)[None, :, :, :]
            pose_output = self.run_hybrik(
                self.hybrik_model, pose_input, bbox, img_center, self.device, self.flip_test)
            xyz = pose_output.pred_xyz_jts_29.float().reshape(29, 3).cpu().numpy()
            if self.xyz_smoother is not None:
                xyz = self.xyz_smoother.update(xyz)

            conf = 1.0
            frame = BodyPoseFrame(
                valid=True,
                left_shoulder=self._joint(xyz, self.J_L_SHOULDER, conf),
                left_elbow=self._joint(xyz, self.J_L_ELBOW, conf),
                left_wrist=self._joint(xyz, self.J_L_WRIST, conf),
                right_shoulder=self._joint(xyz, self.J_R_SHOULDER, conf),
                right_elbow=self._joint(xyz, self.J_R_ELBOW, conf),
                right_wrist=self._joint(xyz, self.J_R_WRIST, conf),
                left_hip=self._joint(xyz, self.J_L_HIP, conf),
                right_hip=self._joint(xyz, self.J_R_HIP, conf),
            )

            bbox_w = bbox[2] - bbox[0]
            uvd = pose_output.pred_uvd_jts.float().reshape(29, 3).cpu().numpy()
            pts = uvd[:, :2].copy()
            pts[:, 0] = pts[:, 0] * bbox_w + (bbox[0] + bbox[2]) / 2
            pts[:, 1] = pts[:, 1] * bbox_w + (bbox[1] + bbox[3]) / 2
            if self.pts_smoother is not None:
                pts = self.pts_smoother.update(pts)
            frame.pts2d = {
                'ls': pts[self.J_L_SHOULDER], 'le': pts[self.J_L_ELBOW], 'lw': pts[self.J_L_WRIST],
                'rs': pts[self.J_R_SHOULDER], 're': pts[self.J_R_ELBOW], 'rw': pts[self.J_R_WRIST],
                'lh': pts[self.J_L_HIP], 'rh': pts[self.J_R_HIP],
                '_all': pts,
            }
            return frame


class MediaPipePoseAdapter(PoseAdapter):
    """Optional MediaPipe backend (install mediapipe separately)."""

    # MediaPipe Pose landmark indices
    LS, LE, LW = 11, 13, 15
    RS, RE, RW = 12, 14, 16
    LH, RH = 23, 24

    def __init__(self):
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError(
                'mediapipe is not installed. Use --backend hybrik or: pip install mediapipe'
            ) from exc
        self.mp = mp
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    def _joint_from(self, lm, idx: int, w: int, h: int) -> Tuple[Joint3d, np.ndarray]:
        p = lm[idx]
        # world_landmarks are meters in a world frame if present; fall back to normalized image.
        xyz = np.array([p.x, p.y, p.z], dtype=np.float64)
        conf = float(getattr(p, 'visibility', 1.0) or 0.0)
        pt2 = np.array([p.x * w, p.y * h], dtype=np.float64)
        return Joint3d(xyz, conf), pt2

    def estimate(self, bgr: np.ndarray) -> BodyPoseFrame:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.pose.process(rgb)
        if res.pose_world_landmarks is None and res.pose_landmarks is None:
            return empty_frame()
        # Prefer world (metric-ish) landmarks when available.
        lm = (res.pose_world_landmarks or res.pose_landmarks).landmark
        img_lm = (res.pose_landmarks.landmark if res.pose_landmarks else lm)

        def both(idx: int):
            j3, _ = self._joint_from(lm, idx, w, h)
            _, p2 = self._joint_from(img_lm, idx, w, h)
            return j3, p2

        ls, ls2 = both(self.LS)
        le, le2 = both(self.LE)
        lw, lw2 = both(self.LW)
        rs, rs2 = both(self.RS)
        re, re2 = both(self.RE)
        rw, rw2 = both(self.RW)
        lh, lh2 = both(self.LH)
        rh, rh2 = both(self.RH)
        return BodyPoseFrame(
            valid=True,
            left_shoulder=ls, left_elbow=le, left_wrist=lw,
            right_shoulder=rs, right_elbow=re, right_wrist=rw,
            left_hip=lh, right_hip=rh,
            pts2d={'ls': ls2, 'le': le2, 'lw': lw2, 'rs': rs2, 're': re2, 'rw': rw2,
                   'lh': lh2, 'rh': rh2},
        )

    def close(self) -> None:
        self.pose.close()


def draw_keypoints(vis: np.ndarray, frame: BodyPoseFrame) -> None:
    if frame.pts2d is None:
        return
    if '_all' in frame.pts2d:
        # HybrIK full skeleton
        adapter_draw = getattr(draw_keypoints, '_hybrik_draw', None)
        # filled below in main when HybrIK loads
        if adapter_draw is not None:
            adapter_draw(vis, frame.pts2d['_all'])
            return
    pairs = [
        ('ls', 'le', (255, 128, 0)), ('le', 'lw', (255, 128, 0)),
        ('rs', 're', (0, 128, 255)), ('re', 'rw', (0, 128, 255)),
        ('lh', 'ls', (255, 128, 0)), ('rh', 'rs', (0, 128, 255)),
        ('lh', 'rh', (0, 230, 0)),
    ]
    for a, b, color in pairs:
        pa, pb = frame.pts2d.get(a), frame.pts2d.get(b)
        if pa is None or pb is None:
            continue
        cv2.line(vis, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])), color, 3, cv2.LINE_AA)
    for key, color in (('ls', (255, 128, 0)), ('le', (255, 128, 0)), ('lw', (255, 200, 100)),
                       ('rs', (0, 128, 255)), ('re', (0, 128, 255)), ('rw', (100, 200, 255)),
                       ('lh', (255, 128, 0)), ('rh', (0, 128, 255))):
        p = frame.pts2d.get(key)
        if p is None:
            continue
        cv2.circle(vis, (int(p[0]), int(p[1])), 6, color, -1, cv2.LINE_AA)


def open_camera(cam_id: int, width: Optional[int], height: Optional[int], force_res: bool):
    cap = cv2.VideoCapture(int(cam_id), cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(int(cam_id))
    if not cap.isOpened():
        raise RuntimeError(f'cannot open camera {cam_id}')
    if force_res:
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    ok, first = cap.read()
    if not ok:
        raise RuntimeError(f'camera {cam_id} read failed')
    mean = float(np.mean(first))
    print(f'[body-pose-bridge] cam /dev/video{cam_id}: {first.shape[1]}x{first.shape[0]}, mean={mean:.1f}')
    return cap, first


def parse_args():
    p = argparse.ArgumentParser(description='Body keypoints UDP bridge (phase 1)')
    p.add_argument('--backend', default='hybrik', choices=['hybrik', 'mediapipe'])
    p.add_argument('--cam', type=int, default=8)
    p.add_argument('--input', type=str, default='')
    p.add_argument('--gpu', type=int, default=0)
    p.add_argument('--fast', action='store_true')
    p.add_argument('--det', type=str, default='auto',
                   choices=['auto', 'yolo', 'mobile', 'resnet50'])
    p.add_argument('--fp16', action='store_true', default=None)
    p.add_argument('--no-fp16', action='store_true')
    p.add_argument('--det-interval', type=int, default=None)
    p.add_argument('--det-size', type=int, default=None)
    p.add_argument('--no-flip', action='store_true')
    p.add_argument('--det-thr', type=float, default=0.5)
    p.add_argument('--width', type=int, default=None)
    p.add_argument('--height', type=int, default=None)
    p.add_argument('--smooth', type=float, default=0.28)
    p.add_argument('--hold-frames', type=int, default=12)
    p.add_argument('--no-set-res', action='store_true')
    p.add_argument('--udp-host', type=str, default='127.0.0.1')
    p.add_argument('--udp-port', type=int, default=47952)
    p.add_argument('--print-hz', type=float, default=5.0)
    p.add_argument('--show', action='store_true')
    p.add_argument('--scale', type=float, default=1.5)
    p.add_argument('--preview-file', type=str, default='/tmp/body_pose_preview.jpg',
                   help='每帧写出预览图（无窗也可验收左右点）')
    # HybrIK cfg/ckpt filled by resolve_options when --fast
    p.add_argument('--cfg', type=str, default='')
    p.add_argument('--ckpt', type=str, default='')
    return p.parse_args()


def main():
    opt = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (opt.udp_host, opt.udp_port)
    tx_str = f'{opt.udp_host}:{opt.udp_port}'

    if opt.backend == 'hybrik':
        if not opt.fast:
            opt.fast = True  # phase-1 default: realtime
        if not opt.no_flip:
            # match SEW bridge: --no-flip for speed; keep configurable
            pass
        # Ensure HybrIK defaults exist for resolve_options
        if not opt.cfg:
            opt.cfg = 'configs/256x192_adam_lr1e-3-res34_smpl_3d_cam_2x_mix_w_pw3d.yaml'
        if not opt.ckpt:
            opt.ckpt = './pretrained_models/hybrik_res34.pth'
        adapter = HybrIKPoseAdapter(opt)
        # Wire HybrIK skeleton drawer
        from demo_webcam import UPPER_BODY, LOWER_BODY, draw_skeleton_2d, joints_in_skeleton

        jvis = joints_in_skeleton(UPPER_BODY + LOWER_BODY)

        def _hybrik_draw(vis, pts):
            draw_skeleton_2d(vis, pts, UPPER_BODY + LOWER_BODY, jvis)

        draw_keypoints._hybrik_draw = _hybrik_draw  # type: ignore[attr-defined]
    else:
        adapter = MediaPipePoseAdapter()

    if opt.input:
        cap = cv2.VideoCapture(opt.input)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open video {opt.input!r}')
        ok, first = cap.read()
        if not ok:
            raise RuntimeError(f'video read failed: {opt.input!r}')
        print(f'[body-pose-bridge] video {opt.input} {first.shape[1]}x{first.shape[0]}')
    else:
        cap, first = open_camera(opt.cam, opt.width, opt.height, force_res=not opt.no_set_res)

    print(f'[body-pose-bridge] backend={opt.backend}  UDP -> {tx_str}  show={int(opt.show)}')

    win = 'BodyPose bridge'
    close_streak = 0
    show_ready = 0
    if opt.show:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 960, 720)
        boot = first.copy()
        cv2.putText(boot, 'BodyPose starting...', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        if opt.scale != 1.0:
            boot = cv2.resize(boot, None, fx=opt.scale, fy=opt.scale)
        cv2.imshow(win, boot)
        cv2.waitKey(1)

    fps_ema = 0.0
    last_print = 0.0
    frame = first
    frame_idx = 0
    try:
        while True:
            if frame_idx > 0:
                ok, frame = cap.read()
                if not ok:
                    break
            t0 = time.time()
            body = adapter.estimate(frame)
            ts_ms = int(time.time() * 1000)
            msg = pack_csv(ts_ms, body)
            try:
                sock.sendto(msg.encode('ascii'), dst)
            except OSError as exc:
                print(f'[body-pose-bridge] UDP send failed: {exc}')

            dt = time.time() - t0
            fps_ema = 0.9 * fps_ema + 0.1 / max(dt, 1e-6) if fps_ema else 1.0 / max(dt, 1e-6)
            if opt.print_hz > 0 and (time.time() - last_print) >= 1.0 / opt.print_hz:
                last_print = time.time()
                conf_l = min(body.left_shoulder.conf, body.left_elbow.conf, body.left_wrist.conf)
                conf_r = min(body.right_shoulder.conf, body.right_elbow.conf, body.right_wrist.conf)
                print(f'[body-pose-bridge] fps={fps_ema:4.1f} valid={int(body.valid)} '
                      f'conf_L={conf_l:.2f} conf_R={conf_r:.2f}')

            if opt.show or opt.preview_file:
                vis = frame.copy()
                draw_keypoints(vis, body)
                cv2.putText(vis, f'BodyPose -> {tx_str}  fps={fps_ema:.1f}  [q/ESC]',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Left / right labels near shoulders when visible
                if body.pts2d is not None:
                    for key, label, color in (('ls', 'L', (255, 128, 0)),
                                              ('rs', 'R', (0, 128, 255))):
                        p = body.pts2d.get(key)
                        if p is not None:
                            cv2.putText(vis, label, (int(p[0]) + 8, int(p[1]) - 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                if opt.preview_file:
                    cv2.imwrite(opt.preview_file, vis)
                if opt.show:
                    out = vis
                    if opt.scale != 1.0:
                        out = cv2.resize(vis, None, fx=opt.scale, fy=opt.scale)
                    cv2.imshow(win, out)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), 27):
                        print('[body-pose-bridge] key quit')
                        break
                    show_ready += 1
                    if show_ready > 30:
                        try:
                            prop = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE)
                            if prop < 1:
                                close_streak += 1
                            else:
                                close_streak = 0
                        except cv2.error:
                            close_streak = 0
                        if close_streak >= 8:
                            print('[body-pose-bridge] window closed')
                            break
            frame_idx += 1
    except KeyboardInterrupt:
        print('\n[body-pose-bridge] Ctrl+C')
    finally:
        adapter.close()
        cap.release()
        sock.close()
        if opt.show:
            try:
                cv2.destroyWindow(win)
            except cv2.error:
                pass
            cv2.destroyAllWindows()
            for _ in range(4):
                cv2.waitKey(1)
        print('[body-pose-bridge] stopped.')


if __name__ == '__main__':
    main()
