#!/usr/bin/env python3
import os
import math
import argparse
import yaml
import numpy as np
from PIL import Image

try:
    import cv2
    HAS_CV2 = True
except Exception:
    HAS_CV2 = False


# -----------------------------
# ROS map YAML + image handling
# -----------------------------
def load_map_yaml(yaml_path: str):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    # Resolve image path relative to YAML file
    img_path = data["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)

    res = float(data["resolution"])
    origin = data.get("origin", [0.0, 0.0, 0.0])
    origin_x, origin_y, origin_yaw = float(origin[0]), float(origin[1]), float(origin[2])
    negate = int(data.get("negate", 0))
    occ_th = float(data.get("occupied_thresh", 0.65))
    free_th = float(data.get("free_thresh", 0.196))
    mode = data.get("mode", "trinary")

    return {
        "yaml_path": yaml_path,
        "image_path": img_path,
        "resolution": res,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "origin_yaw": origin_yaw,
        "negate": negate,
        "occ_th": occ_th,
        "free_th": free_th,
        "mode": mode,
    }


def image_to_occupancy(img_gray: np.ndarray, negate: int, occ_th: float, free_th: float):
    """
    Convert grayscale map image to occupancy:
      100 = occupied, 0 = free, -1 = unknown

    ROS map_server convention (negate=0): black=occupied, white=free.
    """
    img = img_gray.astype(np.float32)

    if negate == 0:
        occ_prob = (255.0 - img) / 255.0
    else:
        occ_prob = img / 255.0

    occ = occ_prob > occ_th
    free = occ_prob < free_th
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[free] = 0
    grid[occ] = 100
    return grid


def load_occupancy_from_yaml(yaml_path: str):
    meta = load_map_yaml(yaml_path)
    im = Image.open(meta["image_path"]).convert("L")
    img = np.array(im)
    grid = image_to_occupancy(img, meta["negate"], meta["occ_th"], meta["free_th"])

    h, w = grid.shape
    meta["width"] = w
    meta["height"] = h
    return meta, grid


def resample_grid_to_resolution(grid: np.ndarray, res_from: float, res_to: float):
    """
    Resample grid to match target resolution (nearest-neighbor).
    Keeps the same metric extent.
    """
    if abs(res_from - res_to) < 1e-9:
        return grid

    h, w = grid.shape
    width_m = w * res_from
    height_m = h * res_from

    new_w = max(1, int(round(width_m / res_to)))
    new_h = max(1, int(round(height_m / res_to)))

    # tri-state -> image for NN resize
    # occupied=0, free=254, unknown=205
    img = np.full((h, w), 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0

    if HAS_CV2:
        img2 = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    else:
        img2 = np.array(Image.fromarray(img).resize((new_w, new_h), resample=Image.NEAREST))

    # back to occupancy with standard thresholds
    grid2 = image_to_occupancy(img2, negate=0, occ_th=0.65, free_th=0.196)
    return grid2


# -----------------------------
# Geometry helpers
# -----------------------------
def rotate_bound(arr: np.ndarray, angle_deg: float):
    """
    Rotate array with bounds so nothing gets clipped.
    Uses nearest-neighbor.
    """
    if abs(angle_deg) < 1e-9:
        return arr

    if HAS_CV2:
        h, w = arr.shape
        cX, cY = (w / 2.0), (h / 2.0)

        M = cv2.getRotationMatrix2D((cX, cY), angle_deg, 1.0)
        cos = abs(M[0, 0])
        sin = abs(M[0, 1])

        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))

        M[0, 2] += (nW / 2.0) - cX
        M[1, 2] += (nH / 2.0) - cY

        rotated = cv2.warpAffine(arr, M, (nW, nH), flags=cv2.INTER_NEAREST, borderValue=0)
        return rotated
    else:
        im = Image.fromarray(arr)
        rot = im.rotate(angle_deg, resample=Image.NEAREST, expand=True, fillcolor=0)
        return np.array(rot)


def downsample_max(mask: np.ndarray, factor: int):
    """
    Max-pool downsample for binary masks.
    """
    if factor <= 1:
        return mask
    h, w = mask.shape
    h2 = (h // factor) * factor
    w2 = (w // factor) * factor
    m = mask[:h2, :w2]
    m = m.reshape(h2 // factor, factor, w2 // factor, factor)
    return (m.max(axis=(1, 3)) > 0).astype(np.uint8)


def phase_correlation(a: np.ndarray, b: np.ndarray):
    """
    Phase correlation to estimate shift that should be applied to b to align with a.
    Returns (dx, dy) in pixels (x right, y down). Rounded later by caller.
    """
    a = a.astype(np.float32)
    b = b.astype(np.float32)

    # window to reduce edge effects
    ha, wa = a.shape
    wy = np.hanning(ha)
    wx = np.hanning(wa)
    window = (wy[:, None] * wx[None, :]).astype(np.float32)
    a *= window
    b *= window

    A = np.fft.fft2(a)
    B = np.fft.fft2(b)
    R = A * np.conj(B)
    denom = np.abs(R)
    R /= (denom + 1e-9)
    r = np.fft.ifft2(R)
    r_abs = np.abs(r)

    peak = np.unravel_index(np.argmax(r_abs), r_abs.shape)
    py, px = peak

    # convert to signed shift
    if px > wa // 2:
        px = px - wa
    if py > ha // 2:
        py = py - ha

    # This shift aligns b to a
    dx = float(px)
    dy = float(py)
    return dx, dy, float(r_abs[peak])


def overlap_score(occ1, free1, occ2, free2, tx, ty):
    """
    Score placing map2 (occ2/free2) at (tx,ty) relative to map1 (top-left).
    Higher is better.
    """
    h1, w1 = occ1.shape
    h2, w2 = occ2.shape

    x1a = max(0, tx)
    y1a = max(0, ty)
    x1b = min(w1, tx + w2)
    y1b = min(h1, ty + h2)

    if x1b <= x1a or y1b <= y1a:
        return -1e18

    x2a = x1a - tx
    y2a = y1a - ty
    x2b = x2a + (x1b - x1a)
    y2b = y2a + (y1b - y1a)

    o1 = occ1[y1a:y1b, x1a:x1b]
    f1 = free1[y1a:y1b, x1a:x1b]
    o2 = occ2[y2a:y2b, x2a:x2b]
    f2 = free2[y2a:y2b, x2a:x2b]

    # reward occupied agreement, penalize contradictions
    overlap_occ = np.sum((o1 & o2))
    conflict = np.sum((f1 & o2)) + np.sum((o1 & f2))

    # weights are empirical, safe-ish
    return float(3.0 * overlap_occ - 2.0 * conflict)


# -----------------------------
# Alignment search
# -----------------------------
def estimate_transform(map1_grid, map2_grid, angle_full=True):
    """
    Returns best (angle_deg, tx_px, ty_px) where tx,ty place rotated map2
    top-left into map1 pixel coordinates (x right, y down).
    """
    occ1 = (map1_grid == 100).astype(np.uint8)
    free1 = (map1_grid == 0).astype(np.uint8)
    occ2 = (map2_grid == 100).astype(np.uint8)
    free2 = (map2_grid == 0).astype(np.uint8)

    # pyramid factors (coarse -> fine)
    levels = [8, 4, 2, 1]

    best_angle = 0.0
    best_tx = 0
    best_ty = 0
    best_score = -1e18

    # initial angle sweep
    if angle_full:
        angle_candidates = np.arange(-180, 180, 15.0)
    else:
        angle_candidates = np.arange(-45, 45, 5.0)

    for factor in levels:
        o1 = downsample_max(occ1, factor)
        f1 = downsample_max(free1, factor)
        o2 = downsample_max(occ2, factor)
        f2 = downsample_max(free2, factor)

        # refine angle range around current best
        if factor != levels[0]:
            span = 15.0 if factor == 4 else (5.0 if factor == 2 else 2.0)
            step = 5.0 if factor == 4 else (1.0 if factor == 2 else 0.5)
            angle_candidates = np.arange(best_angle - span, best_angle + span + 1e-9, step)

        for ang in angle_candidates:
            ro2 = rotate_bound(o2, ang)
            rf2 = rotate_bound(f2, ang)

            h1, w1 = o1.shape
            h2, w2 = ro2.shape
            H = 2 * max(h1, h2)
            W = 2 * max(w1, w2)

            # center both into big canvases
            A = np.zeros((H, W), dtype=np.float32)
            B = np.zeros((H, W), dtype=np.float32)

            ay = H // 2 - h1 // 2
            ax = W // 2 - w1 // 2
            by = H // 2 - h2 // 2
            bx = W // 2 - w2 // 2

            A[ay:ay + h1, ax:ax + w1] = o1
            B[by:by + h2, bx:bx + w2] = ro2

            dx, dy, peak = phase_correlation(A, B)

            # dx,dy is shift to apply to B to align to A in the big canvas
            # compute top-left of rotated map2 relative to map1 top-left at this scale
            # (map1 top-left is at ax,ay; map2 top-left at bx,by)
            tx = int(round((bx + dx) - ax))
            ty = int(round((by + dy) - ay))

            # score in map1 pixel space at this scale
            score = overlap_score(o1.astype(bool), f1.astype(bool),
                                  ro2.astype(bool), rf2.astype(bool),
                                  tx, ty)

            # small tie-break using correlation peak
            score += 0.001 * peak

            if score > best_score:
                best_score = score
                best_angle = float(ang)
                best_tx = int(tx)
                best_ty = int(ty)

        # scale translation up to next finer level
        if factor != 1:
            # when factor halves, tx,ty roughly doubles
            best_tx *= 2
            best_ty *= 2

    return best_angle, best_tx, best_ty, best_score


# -----------------------------
# Merging
# -----------------------------
def fuse_grids(grid_a: np.ndarray, grid_b: np.ndarray):
    """
    Fuse two tri-state grids (-1/0/100) of same shape.
    Rule: occupied wins; else if any free and none occupied => free; else unknown.
    """
    out = np.full(grid_a.shape, -1, dtype=np.int8)

    a_occ = (grid_a == 100)
    b_occ = (grid_b == 100)
    a_free = (grid_a == 0)
    b_free = (grid_b == 0)

    out[a_occ | b_occ] = 100
    out[(~(a_occ | b_occ)) & (a_free | b_free)] = 0
    return out


def grid_to_ros_image(grid: np.ndarray):
    """
    Convert tri-state grid to grayscale image for map_server:
      occupied -> 0
      free     -> 254
      unknown  -> 205
    """
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0
    return img


def write_map(out_yaml_path: str, out_img_path: str, out_grid: np.ndarray, resolution: float, origin_x: float, origin_y: float):
    img = grid_to_ros_image(out_grid)
    Image.fromarray(img).save(out_img_path)

    data = {
        "image": os.path.basename(out_img_path),
        "resolution": float(resolution),
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    with open(out_yaml_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser(description="Stitch two ROS occupancy-grid maps (yaml+pgm/png) into one.")
    ap.add_argument("--map1", required=True, help="Path to map1.yaml (reference map)")
    ap.add_argument("--map2", required=True, help="Path to map2.yaml (to align and merge)")
    ap.add_argument("--out", required=True, help="Output stitched yaml path, e.g. stitched.yaml")
    ap.add_argument("--out_image", default=None, help="Output image path, e.g. stitched.pgm (default: alongside --out)")
    ap.add_argument("--full_rotation", action="store_true", help="Search full 360 deg for alignment (slower, more robust)")
    args = ap.parse_args()

    meta1, grid1 = load_occupancy_from_yaml(args.map1)
    meta2, grid2 = load_occupancy_from_yaml(args.map2)

    if abs(meta1["origin_yaw"]) > 1e-3 or abs(meta2["origin_yaw"]) > 1e-3:
        print("[WARN] Non-zero yaw in map origins detected. This script assumes yaw ~ 0 in YAML origin.")

    # Match resolutions by resampling map2 to map1 resolution (keeps metric extent)
    if abs(meta1["resolution"] - meta2["resolution"]) > 1e-9:
        print(f"[INFO] Resampling map2 from res={meta2['resolution']} to res={meta1['resolution']}")
        grid2 = resample_grid_to_resolution(grid2, meta2["resolution"], meta1["resolution"])

    # Estimate alignment in pixel space
    print("[INFO] Estimating transform (this can take a bit on big maps)...")
    angle_deg, tx, ty, score = estimate_transform(grid1, grid2, angle_full=args.full_rotation)
    print(f"[RESULT] best angle={angle_deg:.3f} deg, tx={tx} px, ty={ty} px, score={score:.3f}")

    # Rotate full-resolution grid2 (tri-state) via rotating masks and rebuilding
    occ2 = (grid2 == 100).astype(np.uint8)
    free2 = (grid2 == 0).astype(np.uint8)

    rocc2 = rotate_bound(occ2, angle_deg).astype(bool)
    rfree2 = rotate_bound(free2, angle_deg).astype(bool)

    # Build rotated tri-state grid2r
    grid2r = np.full(rocc2.shape, -1, dtype=np.int8)
    grid2r[rfree2] = 0
    grid2r[rocc2] = 100  # occupied wins

    h1, w1 = grid1.shape
    h2, w2 = grid2r.shape

    # Output bounds (in map1 pixel coords)
    minx = min(0, tx)
    miny = min(0, ty)
    maxx = max(w1, tx + w2)
    maxy = max(h1, ty + h2)

    W = int(math.ceil(maxx - minx))
    H = int(math.ceil(maxy - miny))

    ox1 = -minx
    oy1 = -miny
    ox2 = ox1 + tx
    oy2 = oy1 + ty

    out = np.full((H, W), -1, dtype=np.int8)

    # paste map1
    out[oy1:oy1 + h1, ox1:ox1 + w1] = grid1

    # paste map2 rotated into a temp canvas, then fuse
    temp = np.full((H, W), -1, dtype=np.int8)
    # compute overlap bounds safely
    x2a = max(0, ox2)
    y2a = max(0, oy2)
    x2b = min(W, ox2 + w2)
    y2b = min(H, oy2 + h2)
    if x2b > x2a and y2b > y2a:
        sx = x2a - ox2
        sy = y2a - oy2
        temp[y2a:y2b, x2a:x2b] = grid2r[sy:sy + (y2b - y2a), sx:sx + (x2b - x2a)]

    out = fuse_grids(out, temp)

    # Compute new YAML origin so map1 stays metrically consistent
    res = meta1["resolution"]
    origin_out_x = meta1["origin_x"] - ox1 * res

    # Y origin formula (see derivation): origin_out_y = origin1_y + (h1 - H + oy1) * res
    origin_out_y = meta1["origin_y"] + (h1 - H + oy1) * res

    out_yaml = args.out
    out_img = args.out_image
    if out_img is None:
        out_dir = os.path.dirname(os.path.abspath(out_yaml))
        base = os.path.splitext(os.path.basename(out_yaml))[0]
        out_img = os.path.join(out_dir, base + ".pgm")

    os.makedirs(os.path.dirname(os.path.abspath(out_yaml)), exist_ok=True)
    print(f"[INFO] Writing: {out_yaml} and {out_img}")
    write_map(out_yaml, out_img, out, res, origin_out_x, origin_out_y)
    print("[DONE]")

if __name__ == "__main__":
    main()
