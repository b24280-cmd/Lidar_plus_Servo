#!/usr/bin/env python3

import open3d as o3d

SCAN_PATH = '/home/sam/Downloads/scan.ply'

# ── tunable parameters ────────────────────────────────────────────────────────
DISTANCE_THRESHOLD = 0.05   # metres — max distance from point to plane to count as inlier
MIN_POINTS         = 100    # planes with fewer inliers than this are discarded
MAX_PLANES         = 5      # stop after finding this many planes
NUM_ITERATIONS     = 1000   # RANSAC iterations per plane search
# ─────────────────────────────────────────────────────────────────────────────

COLOURS = [
    [1.0, 0.2, 0.2],   # red
    [0.2, 0.8, 0.2],   # green
    [0.2, 0.4, 1.0],   # blue
    [1.0, 0.9, 0.1],   # yellow
    [1.0, 0.2, 1.0],   # magenta
]


def main():

    print(f'Loading {SCAN_PATH} ...')
    pcd = o3d.io.read_point_cloud(SCAN_PATH)
    total = len(pcd.points)
    print(f'Loaded {total} points\n')

    remaining = pcd
    geometries = []

    for i in range(MAX_PLANES):

        if len(remaining.points) < MIN_POINTS:
            print(f'Fewer than {MIN_POINTS} points left — stopping.\n')
            break

        model, inliers = remaining.segment_plane(
            distance_threshold=DISTANCE_THRESHOLD,
            ransac_n=3,
            num_iterations=NUM_ITERATIONS,
        )

        if len(inliers) < MIN_POINTS:
            print(f'Plane {i + 1}: only {len(inliers)} inliers — stopping.\n')
            break

        a, b, c, d = model
        pct = 100.0 * len(inliers) / total
        print(
            f'Plane {i + 1}:  {len(inliers):>6} points ({pct:.1f}%)  '
            f'equation: {a:+.3f}x {b:+.3f}y {c:+.3f}z {d:+.3f} = 0'
        )

        plane_cloud = remaining.select_by_index(inliers)
        plane_cloud.paint_uniform_color(COLOURS[i % len(COLOURS)])
        geometries.append(plane_cloud)

        remaining = remaining.select_by_index(inliers, invert=True)

    # leftover non-planar points in grey
    remaining.paint_uniform_color([0.5, 0.5, 0.5])
    geometries.append(remaining)

    print(f'\nNon-planar points remaining: {len(remaining.points)}')
    print('\nOpening viewer  (drag to rotate, scroll to zoom, Q to quit) ...\n')

    o3d.visualization.draw_geometries(
        geometries,
        window_name='RANSAC Plane Detection',
        width=1280,
        height=720,
    )


if __name__ == '__main__':
    main()
