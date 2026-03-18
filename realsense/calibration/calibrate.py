#!/usr/bin/env python3
"""
簡易手眼標定腳本
收集對應點並計算變換矩陣
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import json
import subprocess
import time

class HandEyeCalibrator:
    def __init__(self):
        self.points_camera = []  # 相機座標系
        self.points_arm = []     # 手臂座標系
        
        # 相機內參
        self.fx = 909.1
        self.fy = 908.1
        self.cx = 644.8
        self.cy = 371.5
        
        # 手臂連桿長度 (m)
        self.d1 = 0.0612
        self.d3 = 0.2025
        self.d5 = 0.2
        self.d7 = 0.155
        self.arm_length = self.d3 + self.d5 + self.d7
        
    def pixel_to_camera(self, px, py, depth_m):
        """像素座標 + 深度 → 相機座標系"""
        x = (px - self.cx) * depth_m / self.fx
        y = (py - self.cy) * depth_m / self.fy
        z = depth_m
        return np.array([x, y, z])
    
    def joints_to_arm(self, j1, j2, j3, j4, j5, j6, j7):
        """關節角度 → 手臂座標系（簡化正向運動學）"""
        # 簡化：只用 J1 和 J4
        x = self.arm_length * np.sin(j1)
        y = 0
        z = -self.arm_length * np.cos(j1) + self.d1
        return np.array([x, y, z])
    
    def add_point(self, camera_xyz, arm_xyz):
        """添加一組對應點"""
        self.points_camera.append(camera_xyz)
        self.points_arm.append(arm_xyz)
        
    def compute_transform(self):
        """計算變換矩陣 (最小二乘法)"""
        if len(self.points_camera) < 4:
            return None
            
        Pc = np.array(self.points_camera)
        Pa = np.array(self.points_arm)
        
        # 用 SVD 求解 rigid transformation
        centroid_c = Pc.mean(axis=0)
        centroid_a = Pa.mean(axis=0)
        
        Pc_centered = Pc - centroid_c
        Pa_centered = Pa - centroid_a
        
        H = Pc_centered.T @ Pa_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
            
        t = centroid_a - R @ centroid_c
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        return T
    
    def save(self, filename):
        """保存標定結果"""
        T = self.compute_transform()
        data = {
            "points_camera": [p.tolist() for p in self.points_camera],
            "points_arm": [p.tolist() for p in self.points_arm],
            "transform": T.tolist() if T is not None else None
        }
        with open(filename, "w") as f:
            json.dump(data, f, indent=2)
        return T

print("手眼標定腳本已建立")
