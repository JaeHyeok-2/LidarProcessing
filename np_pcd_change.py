# Module Import 
import open3d as o3d 
import numpy as np 
import pandas as pd 
import matplotlib as plt 
import pptk 


"""
Index
    1. Read Point Clouds 
    2. DownSampling(Voxel DownSampling)
        * Voxel내의 하나의 점만을 남겨 중복을 제거하는 방식
        * parameter : voxel_size 로, 해당 voxel_size의 정육면체 내에 하나의 Point만을 남김 
    3. Remove Outlier 
        3.1  Statistical Outlier Removal
            * neighbor이라는 단위로 그룹을 만든 후, 그룹내의 mean/std를 구한 후, threshold값을 정하여 거리의 편차가 큰 Point를 Outlier로 지정
            * Parameter : nb_neighbors, std_ratio 

        3.2  Radius Outlier Removal
            * sphere를 그리고, 구 내부에 포인트의 일정 개수 이하만 있으면 Outlier로 제거하는 방식; 즉 듬성 듬성 존재하는 점들을 제거하기 좋은 방법
    
    
            
    4. RANSAC(RAndom SAample Consensus)
        데이터를 랜덤하게 샘플링하여 사용하고자 하는 모델을 Fitting 후, Fitting 결과가 목표치(합의점, Consensus)에 도달하였는 지 확인하는 과정 
        RANSAC이란 1)데이터 샘픞링, 2) 모델 fitting, 3)목표치 도달 확인 이라는 3가지 과정을 반복적으로 수행하는 과정
        
        어떤 모형의 모델이던지 간에 모델 fitting 과정을 적용하면 되므로 RANSAC은 모델을 fitting하는 일종의 FrameWork라고 말하기도 합니다.
        
        * Outliers에 Robust한 모델을 만들 수 있음
        * Outliers에 Robust한 모델을 만드는 방법 중 매우 단순한 방법이므로 구현이 쉽고 응용하기가 쉬움
        * 어떤 모델을 사용하더라도 해당 기법을 적용할 수 있음
        
"""
# Transfom pcd to numpy  
pcd = o3d.io.read_point_cloud("data_object_velodyne/training/velodyne_pcd/000033.pcd")
pcd_np = np.asarray(pcd.points)


print(pcd_np)
print(f"Number of Point Clouds :  {len(pcd_np)}")
#  Transform numpy to pcd
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_np)

# v_1 = pptk.viewer(pcd.points)


# After DownSampling, 
def Difference_WO_DownSampling(pcd): 
    print(f"Points before DownSampling : {len(pcd.points)}")
    pcd = pcd.voxel_down_sample(voxel_size=0.3)
    print(f"Points After DownSamping : {len(pcd.points)}")
    return pcd 


DownSampling_pcd = Difference_WO_DownSampling(pcd) 
# v_2 = pptk.viewer(DownSampling_pcd.points)



# Outlier Removal

def statistic_outlier_removal(pcd):
    pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
    outlier_cloud.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    return pcd 
 

def radius_outlier_removal(pcd):
    pcd, inliers = pcd.remove_radius_outlier(nb_points=20, radius=0.5)
    print(inliers)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
    outlier_cloud.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    return pcd

statistical_removal_pcd = statistic_outlier_removal(DownSampling_pcd)
# radius_removal_pcd = radius_outlier_removal(DownSampling_pcd)