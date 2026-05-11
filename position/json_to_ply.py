#!/usr/bin/env python3
import json
import argparse
import os

def write_ply_frame(points, output_dir, frame_id):
    """
    将单帧的3D点写入PLY文件
    
    参数:
        points: 该帧的点列表
        output_dir: 输出目录
        frame_id: 帧ID
    """
    # 根据关键点索引分配颜色
    colors = [
        (255, 0, 0),     # 0: 红色 - 腕关节
        (255, 128, 0),   # 1: 橙色
        (255, 255, 0),   # 2: 黄色
        (128, 255, 0),   # 3: 浅绿色
        (0, 255, 0),     # 4: 绿色 - 拇指指尖
        (0, 255, 255),   # 5: 青色
        (0, 128, 255),   # 6: 浅蓝
        (0, 0, 255),     # 7: 蓝色
        (128, 0, 255),   # 8: 紫色
        (255, 0, 255),   # 9: 粉色
        (255, 0, 128),   # 10: 玫红
        (255, 128, 128), # 11: 浅红
        (128, 128, 128), # 12: 灰色
        (192, 192, 192), # 13: 浅灰
        (0, 128, 128),   # 14: 青绿
        (128, 128, 0),   # 15: 橄榄
        (255, 192, 0),   # 16: 金黄
        (128, 0, 128),   # 17: 深紫
        (0, 192, 192),   # 18: 天蓝
        (192, 0, 192),   # 19: 紫红
        (0, 255, 128),   # 20: 薄荷绿
    ]
    
    # 生成输出文件名
    ply_file = os.path.join(output_dir, f"frame_{frame_id:04d}.ply")
    
    with open(ply_file, 'w') as f:
        # PLY文件头
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        
        # 写入顶点数据
        for point in points:
            idx = point['point_idx'] % len(colors)
            r, g, b = colors[idx]
            f.write(f"{point['x']} {point['y']} {point['z']} {r} {g} {b}\n")
    
    return len(points)

def json_to_ply_per_frame(json_file, output_dir):
    """
    将JSON格式的3D关键点数据转换为PLY文件，每个帧单独保存
    
    参数:
        json_file: 输入的ndjson文件路径
        output_dir: 输出目录
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    total_frames = 0
    total_points = 0
    
    with open(json_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            try:
                data = json.loads(line)
                frame_id = data.get('frame_id', 0)
                
                keypoints3d = data.get('keypoints3d', [])
                frame_points = []
                
                for i, point in enumerate(keypoints3d):
                    # 跳过无效点（坐标为-1的点）
                    if point[0] == -1 or point[1] == -1 or point[2] == -1:
                        continue
                    
                    frame_points.append({
                        'x': point[0],
                        'y': point[1],
                        'z': point[2],
                        'point_idx': i
                    })
                
                # 写入该帧的PLY文件
                point_count = write_ply_frame(frame_points, output_dir, frame_id)
                total_frames += 1
                total_points += point_count
                
            except json.JSONDecodeError as e:
                print(f"解析JSON失败: {e}")
                continue
    
    print(f"转换完成！")
    print(f"处理帧数: {total_frames}")
    print(f"总顶点数: {total_points}")
    print(f"输出目录: {output_dir}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='将3D关键点JSON转换为PLY文件（每帧一个文件）')
    parser.add_argument('--input', required=True, help='输入的ndjson文件路径')
    parser.add_argument('--output-dir', required=True, help='输出目录（每个帧会保存为frame_XXXX.ply）')
    
    args = parser.parse_args()
    
    json_to_ply_per_frame(args.input, args.output_dir)