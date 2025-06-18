import numpy as np
import cv2
import struct

# --- PointCloud2のヘルパー ---

def _parse_pointcloud2_field_offset(fields):
    """PointCloud2のフィールドからx, y, z, intensityのオフセットを取得"""
    offsets = {}
    for field in fields:
        offsets[field.name] = field.offset
    return offsets

def _convert_pointcloud2_to_bgr(msg):
    """
    sensor_msgs/msg/PointCloud2 をトップダウンビューのBGR画像に変換する。
    """
    offsets = _parse_pointcloud2_field_offset(msg.fields)
    x_offset = offsets.get('x')
    y_offset = offsets.get('y')
    z_offset = offsets.get('z')

    if x_offset is None or y_offset is None or z_offset is None:
        return None

    points = []
    for i in range(msg.width * msg.height):
        base_offset = i * msg.point_step
        x = struct.unpack_from('<f', msg.data, base_offset + x_offset)[0]
        y = struct.unpack_from('<f', msg.data, base_offset + y_offset)[0]
        z = struct.unpack_from('<f', msg.data, base_offset + z_offset)[0]
        points.append([x, y, z])
    points = np.array(points)

    side_range, fwd_range = (-20, 20), (-20, 20)
    img_size = (512, 512)
    
    x_points, y_points, z_points = points[:, 0], points[:, 1], points[:, 2]
    f_filt = np.logical_and(x_points > fwd_range[0], x_points < fwd_range[1])
    s_filt = np.logical_and(y_points > side_range[0], y_points < side_range[1])
    filter = np.logical_and(f_filt, s_filt)
    points = points[filter]

    if len(points) == 0:
        return np.zeros((*img_size, 3), dtype=np.uint8)

    x_img = (-points[:, 1] * (img_size[0] - 1) / (side_range[1] - side_range[0])).astype(np.int32)
    y_img = (-points[:, 0] * (img_size[1] - 1) / (fwd_range[1] - fwd_range[0])).astype(np.int32)
    x_img -= int(np.floor(side_range[0] * (img_size[0] - 1) / (side_range[1] - side_range[0])))
    y_img += int(np.floor(fwd_range[1] * (img_size[1] - 1) / (fwd_range[1] - fwd_range[0])))
    
    z_points = points[:, 2]
    z_range = (-2.0, 3.0)
    z_points = np.clip(z_points, z_range[0], z_range[1])
    color_map = ((z_points - z_range[0]) / (z_range[1] - z_range[0]) * 255).astype(np.uint8)
    
    color_image = cv2.applyColorMap(color_map, cv2.COLORMAP_JET)

    image = np.zeros((*img_size, 3), dtype=np.uint8)
    
    image[y_img, x_img] = color_image.reshape(-1, 3)
    
    return image


# --- Image / CompressedImage の変換関数 ---

def _convert_image_msg_to_bgr(msg):
    """
    sensor_msgs/msg/Image をBGRのnumpy配列に変換する。
    """
    if msg.encoding == 'bgr8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'rgb8':
        image_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        return cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        image_mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(image_mono, cv2.COLOR_GRAY2BGR)
    # 他のエンコーディングは未対応
    return None

def _convert_compressed_image_to_bgr(msg):
    """
    sensor_msgs/msg/CompressedImage をBGRのnumpy配列に変換する。
    """
    np_arr = np.frombuffer(msg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def _convert_pointcloud2_to_numpy(msg):
    """
    sensor_msgs/msg/PointCloud2 を (N, 3) のXYZ座標配列、
    または (N, 4) の XYZ+Intensity 配列に変換する。
    """
    offsets = _parse_pointcloud2_field_offset(msg.fields)
    x_offset = offsets.get('x')
    y_offset = offsets.get('y')
    z_offset = offsets.get('z')
    intensity_offset = offsets.get('intensity')

    if x_offset is None or y_offset is None or z_offset is None:
        return None

    point_count = msg.width * msg.height
    point_step = msg.point_step
    data = msg.data
    
    has_intensity = intensity_offset is not None
    if has_intensity:
        points = np.zeros((point_count, 4), dtype=np.float32)
    else:
        points = np.zeros((point_count, 3), dtype=np.float32)

    for i in range(point_count):
        base_offset = i * point_step
        points[i, 0] = struct.unpack_from('<f', data, base_offset + x_offset)[0]
        points[i, 1] = struct.unpack_from('<f', data, base_offset + y_offset)[0]
        points[i, 2] = struct.unpack_from('<f', data, base_offset + z_offset)[0]
        if has_intensity:
            try:
                # 一般的な float32 の intensity を想定
                points[i, 3] = struct.unpack_from('<f', data, base_offset + intensity_offset)[0]
            except struct.error:
                # 他のデータ型 (例: uint8) の場合は要調整
                points[i, 3] = 0.0
                
    return points

# --- ディスパッチャ辞書 ---

IMAGE_THUMBNAIL_CONVERTERS = {
    'sensor_msgs/msg/Image': _convert_image_msg_to_bgr,
    'sensor_msgs/msg/CompressedImage': _convert_compressed_image_to_bgr,
    # 'sensor_msgs/msg/PointCloud2': _convert_pointcloud2_to_bgr,
}
POINTCLOUD_CONVERTERS = {
    'sensor_msgs/msg/PointCloud2': _convert_pointcloud2_to_numpy,
}