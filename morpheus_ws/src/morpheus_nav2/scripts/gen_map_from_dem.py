#!/usr/bin/env python3
import argparse
import os
import cv2
import numpy as np
import xml.etree.ElementTree as ET

def parse_heightmap_size(model_sdf):
    # 解析 Gazebo/IGN 高程图 <size> X Y Z
    # 常见路径：model.sdf -> <model>...<link>...<visual>...<geometry><heightmap><size>sx sy sz</size>
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    size_vals = None
    for tag in root.iter():
        if tag.tag.endswith('size'):
            # 父链为 heightmap?
            parent = tag.getparent() if hasattr(tag, "getparent") else None
        # 简单粗暴遍历：只要文本像 3 个浮点就当它是 size（本模型通常唯一）
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
    """
    mode='height'：按高度二值化；mode='slope'：按坡度边缘二值化（推荐）
    thresh：阈值（高度或梯度）；ksize：Sobel核；inflate：障碍膨胀像素
    invert：有些 DEM 黑低白高，可根据需要翻转
    """
    gray = cv2.imread(dem_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(dem_path)

    if invert:
        gray = 255 - gray

    if mode == 'height':
        # 低处视为可通行，高处为障碍（按需要调阈值）
        _, occ = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)  # 白=障碍, 黑=free
    else:
        # 坡度法：对地形的边缘/急剧变化当障碍，平坦区域可行
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=ksize)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=ksize)
        mag = cv2.magnitude(gx, gy)
        mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, occ = cv2.threshold(mag, thresh, 255, cv2.THRESH_BINARY)  # 白=障碍
        occ = 255 - occ  # 反转：白=free, 黑=障碍  -> 再翻转回 occupancy 规范
        occ = 255 - occ  # occupancy 里白(255)=occupied，黑(0)=free，我们最终要白=障碍
        # 先把 free=0, occ=255 的格式确认
    # 膨胀一点当作安全裕度
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

    # 1) 从 model.sdf 解析地形物理尺寸
    sx, sy, sz = parse_heightmap_size(args.terrain_model_sdf)  # 单位：米
    # 2) 根据 DEM 分辨率换算每像素米数
    img = cv2.imread(args.dem_image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(args.dem_image)
    h, w = img.shape[:2]
    resolution = sx / float(w)  # 假设 DEM X 对应图像宽度
    # 3) 生成 occupancy 图（白=障碍，黑=free）
    occ = build_occupancy_from_dem(args.dem_image, mode=args.mode,
                                   thresh=args.thresh, ksize=args.ksize,
                                   inflate=args.inflate, invert=args.invert)
    cv2.imwrite(map_png, occ)

    # 4) 设置 origin（地图图像左下角在世界坐标的 (-sx/2, -sy/2)）
    origin = (-sx/2.0, -sy/2.0, 0.0)

    # 5) 写 map.yaml
    write_map_yaml(map_yaml, 'map.png', resolution, origin,
                   negate=0, occ_thresh=0.65, free_thresh=0.2)

    print(f"[OK] map.png  -> {map_png}")
    print(f"[OK] map.yaml -> {map_yaml}")
    print(f"    resolution = {resolution:.6f} m/pixel, origin = {origin}")

if __name__ == '__main__':
    main()
