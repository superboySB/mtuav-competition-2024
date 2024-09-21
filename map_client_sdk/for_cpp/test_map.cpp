#include <iostream>
#include <string>
#include "libmap_client_sdk.go.h"

int main() {
    // 创建本地地图客户端
    std::string localMapPath = "voxel_map.bin";
    mtMapHandle mapHandle = MtMapCreateLocal(const_cast<char*>(localMapPath.c_str()));
    if (mapHandle == 0) {
        std::cerr << "Failed to create local map client" << std::endl;
        return 1;
    }

    // 查询地图范围
    int minX, maxX, minY, maxY, minZ, maxZ;
    if (MtMapRange(mapHandle, &minX, &maxX, &minY, &maxY, &minZ, &maxZ) == 0) {
        std::cerr << "Failed to query map range" << std::endl;
        return 1;
    }
    std::cout << "Map range: x(" << minX << " to " << maxX << "), "
              << "y(" << minY << " to " << maxY << "), "
              << "z(" << minZ << " to " << maxZ << ")" << std::endl;

    // 查询体素信息
    float x = 1.0, y = 2.0, z = 3.0;
    mtVoxel voxel;
    if (MtMapQuery(mapHandle, x, y, z, &voxel) == 0) {
        std::cerr << "Failed to query voxel" << std::endl;
        return 1;
    }
    std::cout << "Voxel at (" << x << ", " << y << ", " << z << "):" << std::endl;
    std::cout << "  Distance: " << voxel.distance << std::endl;
    std::cout << "  Current Height to Ground: " << voxel.cur_height_to_ground << std::endl;
    std::cout << "  Height to Ground: " << voxel.height_to_ground << std::endl;
    std::cout << "  Semantic: " << static_cast<int>(voxel.semantic) << std::endl;

    return 0;
}
