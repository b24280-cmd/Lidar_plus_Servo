#!/usr/bin/env python3
#
# PointCleanNet — patch-based neural network outlier removal.
#
# Weights: ~/ros2_ws/pointcleannet_weights/PointCleanNetOutliers_model.pth
# Run:     ros2 run pointcloud_builder pointcleannet_node

import os
import struct

import numpy as np
import open3d as o3d

import torch
import torch.nn as nn
import torch.nn.functional as F

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

SCAN_PATH    = '/home/sam/Downloads/scan.ply'
WEIGHTS_PATH = os.path.expanduser(
    '~/ros2_ws/pointcleannet_weights/PointCleanNetOutliers_model.pth'
)

PATCH_SIZE       = 500    # must match trained model
BATCH_SIZE       = 256    # larger batch is faster on GPU
OUTLIER_FRACTION = 0.15   # top X% of outlier-probability scores are removed
MAX_POINTS       = 10000  # GPU handles 10k points in ~15 s


# ══════════════════════════════════════════════════════════════════════════════
#  Building blocks — match weight-file key names exactly
# ══════════════════════════════════════════════════════════════════════════════

class ResnetBlock1D(nn.Module):
    """
    Residual conv block.  Two-branch form (l0 + l1 → bn1 → l2 → bn2 + skip)
    is used when size_in != size_out; identity-skip form otherwise.
    last=True skips the final ReLU (used for the output layer).
    """
    def __init__(self, size_in, size_out, last=False):
        super().__init__()
        self.last      = last
        self.needs_proj = (size_in != size_out)
        self.l1 = nn.Conv1d(size_in, size_out, 1)
        self.l2 = nn.Conv1d(size_out, size_out, 1)
        self.bn1 = nn.BatchNorm1d(size_out)
        self.bn2 = nn.BatchNorm1d(size_out)
        if self.needs_proj:
            self.l0 = nn.Conv1d(size_in, size_out, 1)
            self.shortcut = nn.Sequential(
                nn.Conv1d(size_in, size_out, 1),
                nn.BatchNorm1d(size_out),
            )

    def forward(self, x):
        if self.needs_proj:
            residual = self.shortcut(x)
            h = F.relu(self.bn1(self.l0(x) + self.l1(x)))
        else:
            residual = x
            h = F.relu(self.bn1(self.l1(x)))
        out = self.bn2(self.l2(h) + residual)
        return out if self.last else F.relu(out)


class ResnetBlockFC(nn.Module):
    """FC (linear) residual block — same two-branch logic as ResnetBlock1D."""

    def __init__(self, size_in, size_out, last=False):
        super().__init__()
        self.last       = last
        self.needs_proj = (size_in != size_out)
        self.l1 = nn.Linear(size_in, size_out)
        self.l2 = nn.Linear(size_out, size_out)
        self.bn1 = nn.BatchNorm1d(size_out)
        self.bn2 = nn.BatchNorm1d(size_out)
        if self.needs_proj:
            self.l0 = nn.Linear(size_in, size_out)
            self.shortcut = nn.Sequential(
                nn.Linear(size_in, size_out),
                nn.BatchNorm1d(size_out),
            )

    def forward(self, x):
        if self.needs_proj:
            residual = self.shortcut(x)
            h = F.relu(self.bn1(self.l0(x) + self.l1(x)))
        else:
            residual = x
            h = F.relu(self.bn1(self.l1(x)))
        out = self.bn2(self.l2(h) + residual)
        return out if self.last else F.relu(out)


# ══════════════════════════════════════════════════════════════════════════════
#  Spatial Transformer Networks
# ══════════════════════════════════════════════════════════════════════════════

class STN_3D(nn.Module):
    """Predicts a 3-D rotation (as quaternion) to canonicalise each patch."""

    def __init__(self, num_points):
        super().__init__()
        self.b1   = ResnetBlock1D(3,    64)
        self.b2   = ResnetBlock1D(64,   128)
        self.b3   = ResnetBlock1D(128,  1024)
        self.mp1  = nn.MaxPool1d(num_points)
        self.bfc1 = ResnetBlockFC(1024, 512)
        self.bfc2 = ResnetBlockFC(512,  256)
        self.bfc3 = ResnetBlockFC(256,  4, last=True)   # quaternion (w,x,y,z)

    def forward(self, x):
        h = self.b3(self.b2(self.b1(x)))
        h = self.mp1(h).squeeze(-1)
        q = self.bfc3(self.bfc2(self.bfc1(h)))          # (B, 4)
        q = F.normalize(q, dim=1)
        return self._quat_to_rot(q)                     # (B, 3, 3)

    @staticmethod
    def _quat_to_rot(q):
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        return torch.stack([
            1 - 2*(y*y + z*z),  2*(x*y - z*w),       2*(x*z + y*w),
            2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w),
            2*(x*z - y*w),      2*(y*z + x*w),        1 - 2*(x*x + y*y),
        ], dim=1).view(-1, 3, 3)


class STN_64(nn.Module):
    """Predicts a 64×64 feature-space transform matrix."""

    def __init__(self, num_points):
        super().__init__()
        self.b1   = ResnetBlock1D(64,   64)
        self.b2   = ResnetBlock1D(64,   128)
        self.b3   = ResnetBlock1D(128,  1024)
        self.mp1  = nn.MaxPool1d(num_points)
        self.bfc1 = ResnetBlockFC(1024, 512)
        self.bfc2 = ResnetBlockFC(512,  256)
        self.bfc3 = ResnetBlockFC(256,  4096, last=True)   # 64×64

    def forward(self, x):
        h = self.b3(self.b2(self.b1(x)))
        h = self.mp1(h).squeeze(-1)
        T = self.bfc3(self.bfc2(self.bfc1(h))).view(-1, 64, 64)
        return T + torch.eye(64, device=x.device).unsqueeze(0)


# ══════════════════════════════════════════════════════════════════════════════
#  PCPNet feature extractor + outlier classifier
# ══════════════════════════════════════════════════════════════════════════════

class PCPNetFeat(nn.Module):

    def __init__(self, num_points=500):
        super().__init__()
        self.stn1 = STN_3D(num_points)
        self.b0a  = ResnetBlock1D(3,   64)
        self.b0b  = ResnetBlock1D(64,  64)
        self.stn2 = STN_64(num_points)
        self.b1   = ResnetBlock1D(64,  64)
        self.b2   = ResnetBlock1D(64,  128)
        self.b3   = ResnetBlock1D(128, 1024)
        self.mp1  = nn.MaxPool1d(num_points)

    def forward(self, x):
        x = torch.bmm(self.stn1(x), x)     # apply 3-D rotation  (B,3,K)
        x = self.b0b(self.b0a(x))          # → (B,64,K)
        x = torch.bmm(self.stn2(x), x)     # apply 64-D transform
        x = self.b3(self.b2(self.b1(x)))   # → (B,1024,K)
        return self.mp1(x).squeeze(-1)     # global max-pool → (B,1024)


class PCPNetOutliers(nn.Module):

    def __init__(self, num_points=500):
        super().__init__()
        self.feat = PCPNetFeat(num_points)
        self.b1   = ResnetBlockFC(1024, 512)
        self.b2   = ResnetBlockFC(512,  256)
        self.b3   = ResnetBlockFC(256,  1, last=True)   # raw logit

    def forward(self, x):
        h = self.feat(x)
        h = self.b2(self.b1(h))
        return torch.sigmoid(self.b3(h)).squeeze(1)    # (B,) in [0,1]


# ══════════════════════════════════════════════════════════════════════════════
#  Helpers
# ══════════════════════════════════════════════════════════════════════════════

def _pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]


XYZRGB_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


def _extract_all_patches(pts, k):
    """Batch k-NN extraction for ALL points at once (much faster than per-point loop)."""
    from sklearn.neighbors import NearestNeighbors
    nbrs = NearestNeighbors(n_neighbors=k, algorithm='kd_tree', n_jobs=-1).fit(pts)
    dists, indices = nbrs.kneighbors(pts)              # (n, k)
    patches = pts[indices] - pts[:, None, :]            # (n, k, 3)  centred
    scales  = dists.max(axis=1) + 1e-8                 # (n,)
    patches = patches / scales[:, None, None]           # normalise to unit sphere
    return patches.astype(np.float32)                  # (n, k, 3)


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ══════════════════════════════════════════════════════════════════════════════

class PointCleanNetNode(Node):

    def __init__(self):
        super().__init__('pointcleannet_node')
        self.clean_pub   = self.create_publisher(PointCloud2, '/pointcleannet/clean',    10)
        self.outlier_pub = self.create_publisher(PointCloud2, '/pointcleannet/outliers', 10)
        self._clean_pts, self._outlier_pts = self._run()
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing on /pointcleannet/clean and /pointcleannet/outliers'
        )

    def _load_model(self):
        if not os.path.exists(WEIGHTS_PATH):
            raise FileNotFoundError(
                f'Weights not found at {WEIGHTS_PATH}\n'
                'Run:  cp /tmp/outliersRemovalModel/PointCleanNetOutliers_model.pth '
                '~/ros2_ws/pointcleannet_weights/'
            )
        self.get_logger().info(f'Loading weights from {WEIGHTS_PATH} ...')
        model = PCPNetOutliers(num_points=PATCH_SIZE)
        state = torch.load(WEIGHTS_PATH, map_location='cpu', weights_only=True)
        model.load_state_dict(state)
        # Use train mode so BN uses batch statistics (the stored running stats
        # were calibrated on mesh point clouds, not LiDAR, causing value explosion
        # in eval mode).  No gradients computed thanks to torch.no_grad().
        model.train()
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Running on {device.upper()}')
        return model.to(device), device

    def _run(self):
        model, device = self._load_model()

        self.get_logger().info(f'Loading {SCAN_PATH} ...')
        pcd_full = o3d.io.read_point_cloud(SCAN_PATH)
        total    = len(pcd_full.points)
        self.get_logger().info(f'Loaded {total} points')

        if total > MAX_POINTS:
            pcd = pcd_full.random_down_sample(MAX_POINTS / total)
            self.get_logger().info(f'Subsampled to {len(pcd.points)} points')
        else:
            pcd = pcd_full

        pts = np.asarray(pcd.points, dtype=np.float32)
        n   = len(pts)

        self.get_logger().info(
            f'Building {n} patches  (k={PATCH_SIZE}) via batch k-NN ...'
        )
        all_patches = _extract_all_patches(pts, PATCH_SIZE)  # (n, K, 3)
        all_patches_t = torch.from_numpy(all_patches.transpose(0, 2, 1))  # (n, 3, K)

        self.get_logger().info(
            f'Running inference  (batch={BATCH_SIZE}, device={device}) ...'
        )
        outlier_prob = np.zeros(n, dtype=np.float32)

        with torch.no_grad():
            for start in range(0, n, BATCH_SIZE):
                end = min(start + BATCH_SIZE, n)
                x   = all_patches_t[start:end].to(device)
                outlier_prob[start:end] = model(x).cpu().numpy()

                if start % (BATCH_SIZE * 20) == 0:
                    self.get_logger().info(f'  {start}/{n} ...')

        # Percentile threshold: remove the top OUTLIER_FRACTION by outlier score.
        # Absolute probabilities are not calibrated to LiDAR data (model trained
        # on mesh clouds with high outlier ratios); ranking is still meaningful.
        threshold  = float(np.percentile(outlier_prob, (1.0 - OUTLIER_FRACTION) * 100))
        is_outlier = outlier_prob > threshold
        n_out      = is_outlier.sum()
        self.get_logger().info(
            f'Done — clean: {n - n_out}  outliers: {n_out} '
            f'({100.*n_out/n:.1f}%)  threshold={threshold:.4f}'
        )

        white = _pack_rgb(220, 220, 220)
        red   = _pack_rgb(230,  50,  50)

        def to_list(mask, color):
            return [[float(p[0]), float(p[1]), float(p[2]), color]
                    for p in pts[mask]]

        return to_list(~is_outlier, white), to_list(is_outlier, red)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        def hdr():
            h = Header()
            h.stamp    = stamp
            h.frame_id = 'map'
            return h

        self.clean_pub.publish(
            point_cloud2.create_cloud(hdr(), XYZRGB_FIELDS, self._clean_pts)
        )
        self.outlier_pub.publish(
            point_cloud2.create_cloud(hdr(), XYZRGB_FIELDS, self._outlier_pts)
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCleanNetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
