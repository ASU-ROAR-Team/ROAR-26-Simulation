# 👁️ ROAR 2026 — GPU LiDAR Perception & 3D ArUco Detection

[![Package](https://img.shields.io/badge/Package-perception__module-E10600?logo=ros&logoColor=white)](file://${ROAR_WS}/src/navstack/perception/perception_module)
[![Hardware](https://img.shields.io/badge/Compute-CUDA_Accelerated-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/cuda-zone)

The `perception_module` package processes real-time sensor streams from the **16-beam 3D LiDAR** and the **ZED 2i Stereo Camera** to extract 3D obstacle bounding boxes and calculate 6-DOF poses of competition ArUco navigation markers.

---

## 🏗️ Perception Pipeline Architecture

```mermaid
flowchart TD
    classDef input fill:#1E293B,stroke:#38BDF8,stroke-width:1.5px,color:#F8FAFC;
    classDef process fill:#0F172A,stroke:#818CF8,stroke-width:1.5px,color:#F8FAFC;
    classDef output fill:#1E1B4B,stroke:#C084FC,stroke-width:2px,color:#F8FAFC;

    subgraph RAW ["Raw Sensor Ingestion"]
        L["/lidar/points (sensor_msgs/PointCloud2)"]
        C["/zed2i/left/image_rect_color (sensor_msgs/Image)"]
    end

    subgraph LIDAR_PROC ["GPU LiDAR Processing Pipeline"]
        L1["Voxel Grid Downsampling & PassThrough ROI Filtering"]
        L2["Ground Plane Extraction & Removal (RANSAC)"]
        L3["CUDA-Accelerated Euclidean Cluster Extraction (Kd-Tree)"]
        L4["Obstacle Centroid & Minimum Bounding Box Computation"]
    end

    subgraph VISION_PROC ["Computer Vision Pipeline"]
        C1["Dust & Glare Normalization"]
        C2["ArUco Dictionary 4x4 Corner Extraction"]
        C3["PnP 3D Pose Estimation via Camera Intrinsic Matrix"]
    end

    subgraph OUTPUTS ["Fused Perception Outputs"]
        O1["/perception/obstacle_markers (visualization_msgs/MarkerArray)"]
        O2["/perception/aruco_poses (geometry_msgs/PoseArray)"]
        O3["Local Costmap Obstacle Layer"]
    end

    L --> L1 --> L2 --> L3 --> L4 --> O1 & O3
    C --> C1 --> C2 --> C3 --> O2

    class L,C input;
    class L1,L2,L3,L4,C1,C2,C3 process;
    class O1,O2,O3 output;
```

---

## ⚡ Performance Specifications

- **LiDAR Processing Rate**: $\ge 15\,\text{Hz}$ with CUDA acceleration.
- **Minimum Detectable Rock Radius**: $0.15\,\text{m}$.
- **Maximum Detection Range**: $15.0\,\text{m}$ forward swath.
- **ArUco 3D Localization Accuracy**: $\pm 0.03\,\text{m}$ at $3.0\,\text{m}$ distance.
