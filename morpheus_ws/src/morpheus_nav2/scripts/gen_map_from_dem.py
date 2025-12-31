#!/usr/bin/env python3
"""Convert a Gazebo heightmap DEM image into a Nav2 occupancy grid (map.png + map.yaml).

Two modes:
  slope  (default) — marks pixels with high gradient (steep terrain) as obstacles.
                     Preferred: flat traversable areas stay free; cliff edges become walls.
  height            — marks pixels above a height threshold as obstacles.
                     Simpler but can block legitimate high-altitude flat terrain.
"""
import argparse
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def parse_heightmap_size(model_sdf):
    """Read the world-space <size> (X Y Z metres) from a Gazebo heightmap model.sdf.

    The <size> element gives the physical extent of the terrain mesh in world units.
    We need it to compute the metres-per-pixel resolution of the DEM image.
    We scan all elements for a 3-float text value rather than navigating the exact
    XML path, because the heightmap nesting varies across Gazebo versions.
    """
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    size_vals = None
    for tag in root.iter():
        if tag.tag.endswith('size'):
            tag.getparent() if hasattr(tag, "getparent") else None
        try:
            vals = list(map(float, tag.text.strip().split()))
            if len(vals) == 3:
                size_vals = vals
                break
        except Exception:
            continue
    if size_vals is None:
        raise RuntimeError(f"Cannot find <size> in {model_sdf}. Please check model.sdf heightmap <size>.")
    sx, sy, sz = size_vals
    return sx, sy, sz

def build_occupancy_from_dem(dem_path, mode='slope', thresh=20, ksize=5, inflate=2, invert=False):
    """Convert a DEM grayscale image to a binary occupancy image (white=obstacle, black=free).

    Slope mode uses the Sobel operator to approximate local terrain gradient.
    The Sobel kernel computes a finite-difference derivative of the image intensity,
    which is proportional to the terrain height gradient (i.e., slope steepness).
    Pixels where the gradient magnitude exceeds `thresh` are marked as obstacles.

    inflate: morphological dilation radius (pixels) added around all obstacles as
    a safety margin, so the rover's footprint clears detected edges.
    """
    gray = cv2.imread(dem_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(dem_path)

    if invert:
        gray = 255 - gray

    if mode == 'height':
        # Pixels brighter than thresh = high terrain = obstacle
        _, occ = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
    else:
        # Sobel: compute image gradient magnitude as a proxy for terrain slope
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=ksize)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=ksize)
        mag = cv2.magnitude(gx, gy)
        mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, occ = cv2.threshold(mag, thresh, 255, cv2.THRESH_BINARY)  # high gradient → white
        occ = 255 - occ  # invert to free=white, obstacle=black
        occ = 255 - occ  # re-invert to obstacle=white (ROS occupancy convention)

    if inflate > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*inflate+1, 2*inflate+1))
        occ = cv2.dilate(occ, kernel)

    return occ

def write_map_yaml(yaml_path, img_rel_path, resolution, origin, negate=0, occ_thresh=0.65, free_thresh=0.2):
    content = f"""image: {img_rel_path}
resolution: {resolution:.6f}
origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]
negate: {negate}
occupied_thresh: {occ_thresh}
free_thresh: {free_thresh}
"""
    with open(yaml_path, 'w') as f:
        f.write(content)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--terrain_model_sdf', required=True,
                    help='Path to worlds/marsyard2022_terrain/model.sdf')
    ap.add_argument('--dem_image', required=True,
                    help='Path to DEM image, e.g., worlds/marsyard2022_terrain/dem/mars_4096_hm.png')
    ap.add_argument('--out_dir', required=True,
                    help='Output dir for map.png and map.yaml, e.g., morpheus_nav2/config')
    ap.add_argument('--mode', default='slope', choices=['slope','height'])
    ap.add_argument('--thresh', type=float, default=20.0)
    ap.add_argument('--ksize', type=int, default=5)
    ap.add_argument('--inflate', type=int, default=2)
    ap.add_argument('--invert', action='store_true', help='Invert DEM grayscale first')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    map_png = os.path.join(args.out_dir, 'map.png')
    map_yaml = os.path.join(args.out_dir, 'map.yaml')

    # Step 1: read physical terrain size from SDF (metres)
    sx, sy, sz = parse_heightmap_size(args.terrain_model_sdf)
    # Step 2: derive metres-per-pixel from DEM image width vs physical X extent
    img = cv2.imread(args.dem_image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(args.dem_image)
    h, w = img.shape[:2]
    resolution = sx / float(w)   # assumes DEM X axis maps to image width

    # Step 3: build occupancy image (white=obstacle, black=free)
    occ = build_occupancy_from_dem(args.dem_image, mode=args.mode,
                                   thresh=args.thresh, ksize=args.ksize,
                                   inflate=args.inflate, invert=args.invert)
    cv2.imwrite(map_png, occ)

    # Step 4: origin = world position of the image's bottom-left corner.
    # Gazebo centres the terrain at (0,0), so the corner is at (-sx/2, -sy/2).
    origin = (-sx/2.0, -sy/2.0, 0.0)

    # Step 5: write map.yaml with negate=1.
    # build_occupancy_from_dem() outputs white=obstacle / black=free.
    # ROS map_server's default (negate=0) interprets white=free / black=occupied.
    # negate=1 flips the interpretation so the image is read correctly.
    # Without negate=1, ~99% of the map appears occupied and Nav2 cannot plan any path.
    write_map_yaml(map_yaml, 'map.png', resolution, origin,
                   negate=1, occ_thresh=0.65, free_thresh=0.2)

    print(f"[OK] map.png  -> {map_png}")
    print(f"[OK] map.yaml -> {map_yaml}")
    print(f"    resolution = {resolution:.6f} m/pixel, origin = {origin}")

if __name__ == '__main__':
    main()
