"""Example script: ManusSDK glove data with Open3D visualization.

Mirrors the visualization from manus_ros2/client_scripts/manus_data_viz.py
but uses the CFFI-based ManusDataPublisher directly (no ROS2).

Data is polled at DATA_RATE_HZ in a background thread. Rendering runs on the main
thread at whatever rate Open3D can sustain.

Usage:
    python -m manus_glove.run
"""

import logging
import threading
import time

import numpy as np
import open3d as o3d

from manus_glove import ManusDataPublisher

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DATA_RATE_HZ = 120
RENDER_RATE_HZ = 60


class GloveViz:
    """Open3D visualization for a single glove."""

    def __init__(self, glove_id: int):
        self.glove_id = glove_id
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window(window_name=f"Manus Glove {glove_id}", width=800, height=600)

        self.node_meshes: dict[int, o3d.geometry.TriangleMesh] = {}
        self.node_positions: dict[int, np.ndarray] = {}

        self.frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        self.line_set = o3d.geometry.LineSet()

        self.viz.add_geometry(self.frame_mesh)
        self.viz.add_geometry(self.line_set)


def update_glove_viz(glove_viz: GloveViz, glove_data: dict) -> None:
    """Update visualization with new glove data."""
    glove_viz.node_positions = {}

    for node in glove_data["raw_nodes"]:
        node_id = node["id"]
        pos = node["position"]
        position = np.array([pos[0], pos[1], pos[2]])

        glove_viz.node_positions[node_id] = position

        if node_id not in glove_viz.node_meshes:
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
            mesh.compute_vertex_normals()
            glove_viz.node_meshes[node_id] = mesh
            glove_viz.viz.add_geometry(mesh)

        mesh = glove_viz.node_meshes[node_id]
        mesh.translate(-np.asarray(mesh.get_center()), relative=True)
        mesh.translate(position, relative=False)
        glove_viz.viz.update_geometry(mesh)

    update_lines(glove_viz, glove_data)


def update_lines(glove_viz: GloveViz, glove_data: dict) -> None:
    """Update lines connecting child and parent nodes."""
    line_points = []
    line_indices = []

    for node in glove_data["raw_nodes"]:
        node_id = node["id"]
        parent_id = node.get("parentId")

        if parent_id is None:
            continue

        if parent_id in glove_viz.node_positions and node_id in glove_viz.node_positions:
            child_pos = glove_viz.node_positions[node_id]
            parent_pos = glove_viz.node_positions[parent_id]

            start_idx = len(line_points)
            line_points.append(parent_pos)
            line_points.append(child_pos)
            line_indices.append([start_idx, start_idx + 1])

    if line_points:
        glove_viz.line_set.points = o3d.utility.Vector3dVector(line_points)
        glove_viz.line_set.lines = o3d.utility.Vector2iVector(line_indices)
        glove_viz.line_set.paint_uniform_color([0, 0, 0])
        glove_viz.viz.update_geometry(glove_viz.line_set)


def data_poll_loop(
    pub: ManusDataPublisher,
    latest_data: dict[int, dict],
    data_lock: threading.Lock,
    stop_event: threading.Event,
):
    """Poll glove data at DATA_RATE_HZ in background thread."""
    interval = 1.0 / DATA_RATE_HZ
    last_log_time = time.monotonic()
    poll_counts: dict[int, int] = {}

    while not stop_event.is_set():
        tick = time.monotonic()

        glove_ids = pub.GetGloveIds()
        for glove_id in glove_ids:
            data = pub.GetGloveData(glove_id)
            if data is None:
                continue

            with data_lock:
                latest_data[glove_id] = data

            poll_counts[glove_id] = poll_counts.get(glove_id, 0) + 1

        # Periodic logging (mirrors C++ 10-second publish count log)
        now = time.monotonic()
        if now - last_log_time >= 10.0:
            for gid, count in poll_counts.items():
                logger.info(
                    "Glove ID: %d, polls in the last 10 seconds: %d (%.1f Hz)",
                    gid,
                    count,
                    count / 10.0,
                )
            poll_counts.clear()
            last_log_time = now

        elapsed = time.monotonic() - tick
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def main():
    glove_viz_map: dict[int, GloveViz] = {}

    # Shared between data thread and render loop
    latest_data: dict[int, dict] = {}
    data_lock = threading.Lock()
    stop_event = threading.Event()

    with ManusDataPublisher() as pub:
        logger.info("Connected. Waiting for glove data...")

        while pub.GetLandscape() is None:
            time.sleep(0.1)

        logger.info("Landscape received. Starting data + render loops.")

        # Start data polling thread
        data_thread = threading.Thread(
            target=data_poll_loop,
            args=(pub, latest_data, data_lock, stop_event),
            daemon=True,
        )
        data_thread.start()

        # Render loop on main thread
        try:
            while True:
                with data_lock:
                    snapshot = dict(latest_data)

                for glove_id, data in snapshot.items():
                    if glove_id not in glove_viz_map:
                        logger.info(
                            "New glove detected: id=%d side=%s",
                            glove_id,
                            data["side"],
                        )
                        glove_viz_map[glove_id] = GloveViz(glove_id)

                    update_glove_viz(glove_viz_map[glove_id], data)

                for gv in glove_viz_map.values():
                    gv.viz.poll_events()
                    gv.viz.update_renderer()

                time.sleep(1 / RENDER_RATE_HZ)
        finally:
            stop_event.set()
            data_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
