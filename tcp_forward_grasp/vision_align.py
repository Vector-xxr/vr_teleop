"""Detection helpers: rightmost red handle on white paper.

Default target is the whole red skewer handle (beads + sprinkle barrel), not a
single round ball. Tracking uses the merged handle bbox; close trigger uses the
handle top edge (bead nearest the grapes).
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _contour_center(contour, bbox) -> tuple[float, float]:
    m = cv2.moments(contour)
    if abs(m.get("m00", 0.0)) > 1e-6:
        return float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"])
    x, y, w, h = bbox
    return float(x + 0.5 * w), float(y + 0.5 * h)


def _apply_named_roi(mask: np.ndarray, args, *, prefix: str) -> np.ndarray:
    h, w = mask.shape[:2]
    left = int(round(float(getattr(args, f"{prefix}_left", 0.0)) * w))
    right = int(round(float(getattr(args, f"{prefix}_right", 1.0)) * w))
    top = int(round(float(getattr(args, f"{prefix}_top", 0.0)) * h))
    bottom = int(round(float(getattr(args, f"{prefix}_bottom", 1.0)) * h))
    left = max(0, min(w, left))
    right = max(left + 1, min(w, right))
    top = max(0, min(h, top))
    bottom = max(top + 1, min(h, bottom))
    out = np.zeros_like(mask)
    out[top:bottom, left:right] = mask[top:bottom, left:right]
    return out


def _apply_roi(mask: np.ndarray, args) -> np.ndarray:
    return _apply_named_roi(mask, args, prefix="red_roi")


def _contour_features(contour):
    area = float(cv2.contourArea(contour))
    x, y, bw, bh = cv2.boundingRect(contour)
    bbox = (x, y, bw, bh)
    rect = cv2.minAreaRect(contour)
    (_cx, _cy), (rw, rh), angle = rect
    if rw < 1.0 or rh < 1.0:
        return area, bbox, 0.0, 0.0, 90.0, 0.0
    major = float(max(rw, rh))
    minor = float(max(1.0, min(rw, rh)))
    elongation = major / minor
    theta = float(angle if rw >= rh else angle + 90.0)
    theta = (theta + 180.0) % 180.0
    angle_from_vertical = abs(theta - 90.0)
    fill_ratio = area / float(max(1, bw * bh))
    return area, bbox, theta, elongation, angle_from_vertical, fill_ratio


def build_white_paper_mask(image_bgr: np.ndarray, args) -> tuple[np.ndarray, dict[str, Any] | None]:
    """Largest bright low-saturation blob = white paper region.

    Returns (mask_u8, info_or_none). Mask is dilated so red marks sitting on
    the paper still fall inside the allowed region.
    """
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    s_max = int(getattr(args, "white_s_max", 60))
    v_min = int(getattr(args, "white_v_min", 160))
    mask = ((saturation <= s_max) & (value >= v_min)).astype(np.uint8) * 255
    # Soft image ROI to ignore extreme borders / gripper if configured.
    mask = _apply_named_roi(mask, args, prefix="white_roi") if any(
        hasattr(args, k) for k in ("white_roi_left", "white_roi_right", "white_roi_top", "white_roi_bottom")
    ) else _apply_roi(mask, args)
    k = max(3, int(getattr(args, "morph_kernel", 5)))
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_frac = float(getattr(args, "white_min_area_frac", 0.08))
    min_area = max(500.0, min_frac * float(h * w))
    best = None
    best_area = 0.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area <= best_area:
            continue
        best = contour
        best_area = area
    if best is None:
        return np.zeros((h, w), dtype=np.uint8), None

    paper = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(paper, [best], -1, 255, thickness=-1)
    dilate_px = max(0, int(getattr(args, "white_dilate_px", 18)))
    if dilate_px > 0:
        dk = np.ones((dilate_px * 2 + 1, dilate_px * 2 + 1), np.uint8)
        paper = cv2.dilate(paper, dk)
    bbox = cv2.boundingRect(best)
    info = {
        "area_px": float(best_area),
        "bbox_xywh": tuple(int(v) for v in bbox),
        "white_s_max": s_max,
        "white_v_min": v_min,
        "dilate_px": dilate_px,
    }
    return paper, info


def list_red_ball_candidates(
    image_bgr: np.ndarray,
    args,
    *,
    paper_mask: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    """Return red blobs sorted by image-x (left -> right).

    If paper_mask is provided, only keep blobs whose center lies on the white paper.
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h_low = int(getattr(args, "red_h_low", 12))
    h_high = int(getattr(args, "red_h_high", 168))
    red_hue = (hue <= h_low) | (hue >= h_high)
    mask = (
        red_hue
        & (saturation >= int(getattr(args, "red_s_min", 70)))
        & (value >= int(getattr(args, "red_v_min", 25)))
    ).astype(np.uint8) * 255
    mask = _apply_roi(mask, args)
    if paper_mask is not None:
        mask = cv2.bitwise_and(mask, paper_mask)
    k = max(3, int(getattr(args, "morph_kernel", 5)))
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # Join nearby red beads / sprinkle barrel into one handle silhouette.
    if bool(getattr(args, "red_handle_mode", True)):
        close_px = max(k, int(getattr(args, "red_handle_close_px", 18)))
        if close_px % 2 == 0:
            close_px += 1
        hk = np.ones((close_px, close_px), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, hk)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = image_bgr.shape[:2]
    min_area = float(getattr(args, "red_min_area_px", 35.0))
    max_area = float(getattr(args, "red_max_area_px", 25000.0))
    items: list[dict[str, Any]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area > max_area:
            continue
        bbox = cv2.boundingRect(contour)
        ux, vy = _contour_center(contour, bbox)
        if paper_mask is not None:
            ix, iy = int(round(ux)), int(round(vy))
            if ix < 0 or iy < 0 or ix >= w or iy >= h or paper_mask[iy, ix] == 0:
                continue
        x, y, bw, bh = [int(v) for v in bbox]
        items.append(
            {
                "center_px": (float(ux), float(vy)),
                "area_px": area,
                "bbox_xywh": (x, y, bw, bh),
                "handle_top_y": float(y),
                "handle_bottom_y": float(y + bh),
                "image_size": (int(w), int(h)),
            }
        )
    items.sort(key=lambda d: d["center_px"][0])  # left -> right
    return items


def target_center_px(args, width: int, height: int) -> tuple[float, float]:
    raw = getattr(args, "target_center_px", None)
    if raw:
        try:
            if isinstance(raw, (list, tuple)) and len(raw) == 2:
                return float(raw[0]), float(raw[1])
            parts = [float(x.strip()) for x in str(raw).split(",") if x.strip()]
            if len(parts) == 2:
                return float(parts[0]), float(parts[1])
        except ValueError:
            pass
    return float(width) * 0.5, float(height) * 0.5


def _local_white_support(
    image_bgr: np.ndarray,
    ux: float,
    vy: float,
    args,
) -> float:
    """Fraction of a ring around (ux,vy) that looks like white paper."""
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    s_max = int(getattr(args, "white_s_max", 70))
    v_min = int(getattr(args, "white_v_min", 140))
    inner = max(2, int(getattr(args, "tray_white_inner_r", 6)))
    outer = max(inner + 2, int(getattr(args, "tray_white_outer_r", 22)))
    cx, cy = int(round(ux)), int(round(vy))
    y0, y1 = max(0, cy - outer), min(h, cy + outer + 1)
    x0, x1 = max(0, cx - outer), min(w, cx + outer + 1)
    if y1 <= y0 or x1 <= x0:
        return 0.0
    patch = hsv[y0:y1, x0:x1]
    yy, xx = np.ogrid[y0:y1, x0:x1]
    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
    ring = (dist2 >= inner * inner) & (dist2 <= outer * outer)
    if not np.any(ring):
        return 0.0
    white = (patch[:, :, 1] <= s_max) & (patch[:, :, 2] >= v_min)
    return float(np.count_nonzero(white & ring) / max(1, np.count_nonzero(ring)))


def list_tray_red_candidates(
    image_bgr: np.ndarray,
    args,
    *,
    near_gripper: bool = False,
) -> list[dict[str, Any]]:
    """Red blobs on the white tray above the black gripper / green crosshair.

    Filters:
      - red HSV blob
      - above the green target (gripper aim), so not the foam between fingers
        (relaxed when near_gripper=True for approach / enter-fingers)
      - local neighborhood is mostly white (tray surface)
        (relaxed when near_gripper=True)
      - roughly compact (ball-like), not long thin glare
    Sorted left -> right by image u; then keep uppermost tray row only
    (unless near_gripper, keep all in ROI and still prefer rightmost later).
    """
    h, w = image_bgr.shape[:2]
    cx, cy = target_center_px(args, w, h)
    if near_gripper:
        # Allow reds that have moved down into / past the green aim toward fingers.
        max_vy = float(h) * float(getattr(args, "approach_red_max_y_frac", 0.95))
        min_white = float(getattr(args, "approach_min_white_frac", 0.0))
    else:
        above_margin = float(getattr(args, "tray_above_target_margin_px", 30.0))
        max_vy = float(cy) - above_margin
        max_vy = min(max_vy, float(h) * float(getattr(args, "tray_max_y_frac", 0.70)))
        min_white = float(getattr(args, "tray_min_white_frac", 0.18))
    min_area = float(getattr(args, "red_min_area_px", 50.0))
    # Handles are elongated (beads + barrel); balls were ~2.8.
    max_aspect = float(
        getattr(
            args,
            "tray_max_bbox_aspect",
            6.0 if bool(getattr(args, "red_handle_mode", True)) else 2.8,
        )
    )

    # Temporarily widen bottom ROI while approaching fingers.
    saved_bottom = float(getattr(args, "red_roi_bottom", 1.0))
    if near_gripper:
        args.red_roi_bottom = float(getattr(args, "approach_red_roi_bottom", 0.98))
    try:
        raw = list_red_ball_candidates(image_bgr, args, paper_mask=None)
    finally:
        if near_gripper:
            args.red_roi_bottom = saved_bottom

    kept: list[dict[str, Any]] = []
    for item in raw:
        ux, vy = item["center_px"]
        if float(vy) > max_vy:
            continue
        if float(item["area_px"]) < min_area:
            continue
        x, y, bw, bh = item["bbox_xywh"]
        aspect = float(max(bw, bh)) / float(max(1, min(bw, bh)))
        if aspect > max_aspect:
            continue
        white_frac = _local_white_support(image_bgr, float(ux), float(vy), args)
        if white_frac < min_white:
            continue
        out = dict(item)
        out["white_frac"] = float(white_frac)
        out["above_target_px"] = float(cy - vy)
        out["bbox_aspect"] = float(aspect)
        out["handle_top_y"] = float(item.get("handle_top_y", y))
        out["handle_bottom_y"] = float(item.get("handle_bottom_y", y + bh))
        kept.append(out)

    if not kept:
        return kept
    if near_gripper:
        kept.sort(key=lambda d: d["center_px"][0])
        return kept
    # Uppermost horizontal tray row, then left->right.
    row_tol = float(getattr(args, "tray_row_y_tol_px", 70.0))
    y_min = min(float(c["center_px"][1]) for c in kept)
    row = [c for c in kept if float(c["center_px"][1]) <= y_min + row_tol]
    row.sort(key=lambda d: d["center_px"][0])
    return row if row else kept


def horizontal_only_error(record: dict[str, Any]) -> dict[str, Any]:
    """Keep left-right pixel error only (image u); zero up-down (image v)."""
    out = dict(record)
    err = list(record.get("error_px") or [0.0, 0.0])
    if len(err) < 2:
        err = [float(err[0]) if err else 0.0, 0.0]
    out["error_px"] = [float(err[0]), 0.0]
    out["align_mode"] = "horizontal_only"
    out["error_px_full"] = [float(err[0]), float(err[1])]
    return out


def _record_from_candidate(
    selected: dict[str, Any],
    *,
    cx: float,
    cy: float,
    w: int,
    h: int,
    candidates: list[dict[str, Any]],
    paper_mask: np.ndarray,
    paper_info: dict[str, Any] | None,
    selection: str,
    lock_center: tuple[float, float] | None = None,
) -> dict[str, Any]:
    ux, vy = selected["center_px"]
    x, y, bw, bh = [float(v) for v in selected["bbox_xywh"]]
    return {
        "detected": True,
        "selection": selection,
        "candidate_count": len(candidates),
        "center_px": [float(ux), float(vy)],
        "image_center_px": [float(w) * 0.5, float(h) * 0.5],
        "target_center_px": [float(cx), float(cy)],
        "error_px": [float(ux - cx), float(vy - cy)],
        "align_mode": "xy_full",
        "area_px": float(selected["area_px"]),
        "bbox_xywh": list(selected["bbox_xywh"]),
        # Handle top = bead nearest grapes (smaller image-y); bottom = tip end.
        "handle_top_y": float(selected.get("handle_top_y", y)),
        "handle_bottom_y": float(selected.get("handle_bottom_y", y + bh)),
        "white_frac": float(selected.get("white_frac", 0.0)),
        "white_paper": paper_info,
        "all_centers_left_to_right": [c["center_px"] for c in candidates],
        "paper_mask": paper_mask,
        "lock_center_px": list(lock_center) if lock_center is not None else [float(ux), float(vy)],
    }


class LockedRedTracker:
    """Pick rightmost once, then stick to the nearest blob (no rightmost re-pick).

    After arm motion the eye-in-hand view shifts a lot, so reacquire uses a
    larger jump budget and raw red blobs (not only tray-filtered ones).
    """

    def __init__(self, args) -> None:
        self.args = args
        self.locked = False
        self.lock_center: tuple[float, float] | None = None
        self.lock_area: float | None = None
        self.max_jump_px = float(getattr(args, "track_max_jump_px", 200.0))
        self.after_motion_jump_px = float(getattr(args, "track_after_motion_jump_px", 360.0))
        self.area_ratio_min = float(getattr(args, "track_area_ratio_min", 0.15))
        self.area_ratio_max = float(getattr(args, "track_area_ratio_max", 6.0))
        self._after_motion = False

    def reset(self) -> None:
        self.locked = False
        self.lock_center = None
        self.lock_area = None
        self._after_motion = False

    def notify_motion(self) -> None:
        """Call after the arm moves — next update may search farther."""
        self._after_motion = True

    def _tray_mask(self, image_bgr: np.ndarray) -> tuple[np.ndarray, dict[str, Any] | None, float, float, int, int]:
        h, w = image_bgr.shape[:2]
        cx, cy = target_center_px(self.args, w, h)
        paper_mask, paper_info = build_white_paper_mask(image_bgr, self.args)
        if paper_info is not None:
            cut_y = min(
                h,
                max(
                    0,
                    int(round(cy - float(getattr(self.args, "tray_above_target_margin_px", 35.0)))),
                ),
            )
            tray_mask = paper_mask.copy()
            tray_mask[cut_y:, :] = 0
        else:
            tray_mask = np.zeros((h, w), dtype=np.uint8)
        return tray_mask, paper_info, cx, cy, w, h

    def _pick_nearest(
        self,
        candidates: list[dict[str, Any]],
        *,
        max_jump: float,
        require_area: bool = True,
    ) -> dict[str, Any] | None:
        """Nearest to lock, but refuse large leftward hops when a righter blob is nearby.

        Eye-in-hand forward often drops the true rightmost briefly; pure nearest then
        sticks to a middle ball and never recovers — keep a rightward bias.
        """
        assert self.lock_center is not None
        lx, ly = self.lock_center
        left_slack = float(getattr(self.args, "track_left_slack_px", 55.0))
        scored: list[tuple[float, float, dict[str, Any]]] = []
        for c in candidates:
            ux, vy = float(c["center_px"][0]), float(c["center_px"][1])
            dist = float(np.hypot(ux - lx, vy - ly))
            if dist > max_jump:
                continue
            if require_area and self.lock_area and self.lock_area > 1.0:
                area = float(c["area_px"])
                ratio = area / self.lock_area
                if ratio < self.area_ratio_min or ratio > self.area_ratio_max:
                    continue
            scored.append((dist, ux, c))
        if not scored:
            return None
        scored.sort(key=lambda t: t[0])
        best_dist, best_x, best = scored[0]
        # If nearest jumped far left of lock, prefer a righter candidate still near lock.
        if best_x < lx - left_slack:
            rightish = [
                t
                for t in scored
                if t[1] >= lx - left_slack and t[0] <= max(best_dist * 1.6, best_dist + 40.0)
            ]
            if rightish:
                rightish.sort(key=lambda t: (t[0], -t[1]))
                best = rightish[0][2]
            else:
                # Still prefer the rightmost among near-enough options.
                near = [t for t in scored if t[0] <= max(best_dist * 1.8, 80.0)]
                if near:
                    best = max(near, key=lambda t: t[1])[2]
        return best

    def _resnap_to_rightmost(
        self,
        candidates: list[dict[str, Any]],
        selected: dict[str, Any],
    ) -> tuple[dict[str, Any], bool]:
        """If a clearly righter tray ball exists, switch lock to it.

        'Lock once + nearest' otherwise sticks to a middle ball after the first move
        when the true rightmost was briefly missing / newly appears farther right.
        """
        if not candidates or self.lock_center is None:
            return selected, False
        rightmost = candidates[-1]
        rx, ry = float(rightmost["center_px"][0]), float(rightmost["center_px"][1])
        sx, sy = float(selected["center_px"][0]), float(selected["center_px"][1])
        lx, ly = self.lock_center
        margin = float(getattr(self.args, "track_resnap_right_px", 40.0))
        y_band = float(getattr(self.args, "track_resnap_y_band_px", 90.0))
        # Rightmost is substantially righter than current pick / lock, similar row.
        if rx <= max(sx, lx) + margin:
            return selected, False
        if abs(ry - sy) > y_band and abs(ry - ly) > y_band:
            return selected, False
        return rightmost, True

    def _raw_red_candidates(self, image_bgr: np.ndarray) -> list[dict[str, Any]]:
        """All red blobs in a wide ROI — used only for locked reacquire."""
        saved = {
            "red_roi_bottom": float(getattr(self.args, "red_roi_bottom", 1.0)),
            "red_min_area_px": float(getattr(self.args, "red_min_area_px", 50.0)),
            "red_max_area_px": float(getattr(self.args, "red_max_area_px", 8000.0)),
        }
        self.args.red_roi_bottom = float(getattr(self.args, "approach_red_roi_bottom", 0.98))
        self.args.red_min_area_px = min(saved["red_min_area_px"], 25.0)
        self.args.red_max_area_px = max(saved["red_max_area_px"], 20000.0)
        try:
            items = list_red_ball_candidates(image_bgr, self.args, paper_mask=None)
        finally:
            for k, v in saved.items():
                setattr(self.args, k, v)
        # Allow elongated red handles; still drop extreme glare strips.
        max_aspect = float(
            getattr(
                self.args,
                "track_max_bbox_aspect",
                8.0 if bool(getattr(self.args, "red_handle_mode", True)) else 3.5,
            )
        )
        out: list[dict[str, Any]] = []
        for item in items:
            x, y, bw, bh = item["bbox_xywh"]
            aspect = float(max(bw, bh)) / float(max(1, min(bw, bh)))
            if aspect > max_aspect:
                continue
            item = dict(item)
            item["handle_top_y"] = float(item.get("handle_top_y", y))
            item["handle_bottom_y"] = float(item.get("handle_bottom_y", y + bh))
            out.append(item)
        return out

    def update(
        self,
        image_bgr: np.ndarray,
        *,
        near_gripper: bool = False,
    ) -> dict[str, Any]:
        tray_mask, paper_info, cx, cy, w, h = self._tray_mask(image_bgr)
        candidates = list_tray_red_candidates(
            image_bgr, self.args, near_gripper=near_gripper
        )
        jump = self.after_motion_jump_px if self._after_motion else self.max_jump_px

        if not self.locked:
            if not candidates:
                return {
                    "detected": False,
                    "selection": "await_rightmost_lock",
                    "candidate_count": 0,
                    "reason": "no_red_yet_for_lock",
                    "target_center_px": [float(cx), float(cy)],
                    "all_centers_left_to_right": [],
                    "paper_mask": tray_mask,
                    "white_paper": paper_info,
                    "locked": False,
                }
            selected = candidates[-1]  # rightmost once
            self.locked = True
            self.lock_center = (
                float(selected["center_px"][0]),
                float(selected["center_px"][1]),
            )
            self.lock_area = float(selected["area_px"])
            self._after_motion = False
            rec = _record_from_candidate(
                selected,
                cx=cx,
                cy=cy,
                w=w,
                h=h,
                candidates=candidates,
                paper_mask=tray_mask,
                paper_info=paper_info,
                selection="locked_rightmost_initial",
                lock_center=self.lock_center,
            )
            rec["locked"] = True
            rec["track_dist_px"] = 0.0
            return rec

        # Already locked: track nearest, but re-snap if a clearly righter ball appears.
        # (Common after first LR move: initial "rightmost" was middle; true rightmost pops in.)
        selected = self._pick_nearest(candidates, max_jump=jump) if candidates else None
        recover_src = "tray"
        if selected is None:
            # Eye-in-hand shift / strict tray filters: search raw reds nearby.
            raw = self._raw_red_candidates(image_bgr)
            selected = self._pick_nearest(raw, max_jump=jump, require_area=True)
            if selected is None:
                selected = self._pick_nearest(raw, max_jump=jump * 1.25, require_area=False)
            if selected is not None:
                candidates = raw
                recover_src = "raw_red"
        if selected is None and near_gripper:
            # Last resort: nearest raw red anywhere (still prefer lock area).
            raw = self._raw_red_candidates(image_bgr)
            selected = self._pick_nearest(
                raw, max_jump=max(jump, float(min(w, h)) * 0.75), require_area=False
            )
            if selected is not None:
                candidates = raw
                recover_src = "raw_red_wide"

        if selected is None:
            return {
                "detected": False,
                "selection": "locked_track_lost",
                "candidate_count": len(candidates),
                "reason": "no_candidate_near_lock",
                "target_center_px": [float(cx), float(cy)],
                "lock_center_px": list(self.lock_center) if self.lock_center else None,
                "all_centers_left_to_right": [c["center_px"] for c in candidates],
                "paper_mask": tray_mask,
                "white_paper": paper_info,
                "locked": True,
                "search_jump_px": jump,
            }

        resnapped = False
        if candidates:
            selected, resnapped = self._resnap_to_rightmost(candidates, selected)
            if resnapped:
                recover_src = "resnap_rightmost"

        ux, vy = float(selected["center_px"][0]), float(selected["center_px"][1])
        dist = float(np.hypot(ux - self.lock_center[0], vy - self.lock_center[1]))
        self.lock_center = (ux, vy)
        self.lock_area = float(selected["area_px"])
        self._after_motion = False
        selection = (
            "locked_resnap_rightmost"
            if resnapped
            else (
                "locked_track"
                if recover_src == "tray"
                else f"locked_reacquire_{recover_src}"
            )
        )
        rec = _record_from_candidate(
            selected,
            cx=cx,
            cy=cy,
            w=w,
            h=h,
            candidates=candidates,
            paper_mask=tray_mask,
            paper_info=paper_info,
            selection=selection,
            lock_center=self.lock_center,
        )
        rec["locked"] = True
        rec["track_dist_px"] = dist
        rec["recover_src"] = recover_src
        return rec


def detect_rightmost_red(
    image_bgr: np.ndarray,
    args,
    *,
    near_gripper: bool = False,
    lock_center_px: tuple[float, float] | list[float] | None = None,
) -> dict[str, Any] | None:
    """Rightmost red, or nearest-to-lock when lock_center_px is set.

    Prefer LockedRedTracker in the grasp loop so the initial rightmost choice
    stays sticky across frames.
    """
    tracker = LockedRedTracker(args)
    if lock_center_px is not None and len(lock_center_px) >= 2:
        tracker.locked = True
        tracker.lock_center = (float(lock_center_px[0]), float(lock_center_px[1]))
    return tracker.update(image_bgr, near_gripper=near_gripper)
