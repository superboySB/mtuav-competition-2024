import pymtmap

def main():
    # 创建Map实例，假设地图文件路径为 "path/to/map/file"
    map_file_path = "/home/sdk_for_user/map_client_sdk/for_py/voxel_map.bin"
    map_instance = pymtmap.Map(map_file_path)

    # 检查地图是否有效
    if map_instance.IsValid():
        print("Map is valid.")
        
        # 获取地图的边界信息
        print(f"Map boundaries: x({map_instance.min_x()} to {map_instance.max_x()}), "
              f"y({map_instance.min_y()} to {map_instance.max_y()}), "
              f"z({map_instance.min_z()} to {map_instance.max_z()})")
        
        # 查询特定坐标的体素信息
        # x, y, z = 146, 186, -34
        x, y, z = 430, 184, -10
        voxel = map_instance.Query(x, y, z)
        
        # 输出体素信息
        print(f"Voxel at ({x}, {y}, {z}):")
        print(f"  Distance: {voxel.distance}")
        print(f"  Current Height to Ground: {voxel.cur_height_to_ground}")
        print(f"  Height to Ground: {voxel.height_to_ground}")
        print(f"  Semantic: {voxel.semantic}")
    else:
        print("Map is not valid.")

if __name__ == "__main__":
    main()
