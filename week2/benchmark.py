# 对数据集中的点云，批量执行构建树和查找，评测几种方法的运行时间

import numpy as np
import time
import os
import struct
from scipy import spatial

import octree as octree
import kdtree as kdtree
from result_set import KNNResultSet, RadiusNNResultSet


def read_velodyne_bin(path):
    '''
    :param path:
    :return: homography matrix of the point cloud, N*3
    '''
    pc_list = []
    with open(path, 'rb') as f:
        content = f.read()
        pc_iter = struct.iter_unpack('ffff', content)
        for idx, point in enumerate(pc_iter):
            pc_list.append([point[0], point[1], point[2]])
    return np.asarray(pc_list, dtype=np.float32).T


def main():
    # 参数设置
    leaf_size = 32
    min_extent = 0.0001
    k = 8
    radius = 1

    # 数据集路径
    root_dir = './data/'
    cat = os.listdir(root_dir)
    iteration_num = len(cat)

    # 测试brute-force
    brute_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        query = db_np[0, :]

        begin_t = time.time()
        diff = np.linalg.norm(np.expand_dims(query, 0) - db_np, axis=1)
        nn_idx = np.argsort(diff)
        nn_dist = diff[nn_idx]
        brute_time_sum += time.time() - begin_t

    print("Brute-force: %.3f" % (brute_time_sum * 1000 / iteration_num))

    # 测试第三方库scipy.spatial.KDTree
    construction_time_sum = 0
    knn_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        begin_t = time.time()
        tree = spatial.KDTree(db_np, leaf_size)
        construction_time_sum += time.time() - begin_t

        query = db_np[0, :]

        begin_t = time.time()
        res = tree.query(query, k)
        knn_time_sum += time.time() - begin_t

    print("scipy.spatial.KDTree: build %.3f, knn %.3f" % (construction_time_sum * 1000 / iteration_num, knn_time_sum * 1000 / iteration_num))

    # 测试Kdtree
    construction_time_sum = 0
    knn_time_sum = 0
    radius_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        begin_t = time.time()
        root = kdtree.kdtree_construction(db_np, leaf_size)
        construction_time_sum += time.time() - begin_t

        query = db_np[0, :]

        begin_t = time.time()
        result_set = KNNResultSet(capacity=k)
        kdtree.kdtree_knn_search(root, db_np, result_set, query)
        knn_time_sum += time.time() - begin_t

        begin_t = time.time()
        result_set = RadiusNNResultSet(radius=radius)
        kdtree.kdtree_radius_search(root, db_np, result_set, query)
        radius_time_sum += time.time() - begin_t

    print("Kdtree: build %.3f, knn %.3f, radius %.3f" % (construction_time_sum * 1000 / iteration_num, knn_time_sum * 1000 / iteration_num, radius_time_sum * 1000 / iteration_num))

    # 测试Octree
    construction_time_sum = 0
    knn_time_sum = 0
    radius_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        begin_t = time.time()
        root = octree.octree_construction(db_np, leaf_size, min_extent)
        construction_time_sum += time.time() - begin_t

        query = db_np[0, :]

        begin_t = time.time()
        result_set = KNNResultSet(capacity=k)
        octree.octree_knn_search(root, db_np, result_set, query)
        knn_time_sum += time.time() - begin_t

        begin_t = time.time()
        result_set = RadiusNNResultSet(radius=radius)
        octree.octree_radius_search_fast(root, db_np, result_set, query)
        radius_time_sum += time.time() - begin_t

    print("Octree: build %.3f, knn %.3f, radius %.3f" % (construction_time_sum * 1000 / iteration_num, knn_time_sum * 1000 / iteration_num, radius_time_sum * 1000 / iteration_num))


if __name__ == '__main__':
    main()
