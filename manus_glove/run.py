"""Example script: ManusSDK glove data with Open3D visualization.

Mirrors the visualization from manus_ros2/client_scripts/manus_data_viz.py
but uses the CFFI-based ManusDataPublisher directly (no ROS2).

Data is polled at DATA_RATE_HZ in a background thread. Rendering runs on the main
thread at whatever rate Open3D can sustain.

Usage:
    python -m manus_glove.run [--data-rate-hz N] [--render-rate-hz N] [--viz-style {simple,enhanced}]
"""

import argparse
import logging
import threading
import time

import numpy as np
import open3d as o3d

from manus_glove import ManusDataPublisher

from . import common_viz

logger = logging.getLogger(__name__)

DATA_RATE_HZ = 120
RENDER_RATE_HZ = 60

OMITTED_NODES: set[int] = set()  # e.g. {5, 10, 15, 20} to match viz_21 behaviour


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


def update_glove_viz_enhanced(glove_viz: common_viz.GloveViz, glove_data: dict) -> None:
    """Update enhanced visualization with colored spheres, axes, and labels."""
    nodes = glove_data["raw_nodes"]
    kept = [n for n in nodes if n["id"] not in OMITTED_NODES]
    kept_ids = {n["id"] for n in kept}

    # Determine leaf nodes: IDs that are not anyone's parent
    parent_ids = {n["parentId"] for n in kept if n.get("parentId") is not None}
    leaf_ids = kept_ids - parent_ids

    # Build parent lookup for skipping omitted nodes
    parent_map = {n["id"]: n.get("parentId") for n in nodes}

    for idx, node in enumerate(sorted(kept, key=lambda n: n["id"])):
        node_id = node["id"]
        pos = np.array(node["position"])
        rot = np.array(node["rotation"])  # (x, y, z, w)
        glove_viz.update_node(node_id, pos, rot, is_leaf=(node_id in leaf_ids), node_index=idx)

    # Build skeleton connections (walk up parent chain to skip omitted nodes)
    connections = []
    for node_id in kept_ids:
        pid = parent_map.get(node_id)
        while pid is not None and pid in OMITTED_NODES:
            pid = parent_map.get(pid)
        if pid is not None and pid in kept_ids:
            connections.append((pid, node_id))

    glove_viz.update_skeleton(connections)
    glove_viz.update_axes()


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
    data_rate_hz: float = 120.0,
):
    """Poll glove data at data_rate_hz in background thread."""
    interval = 1.0 / data_rate_hz
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
    parser = argparse.ArgumentParser(description="Manus glove visualization")
    parser.add_argument(
        "--data-rate-hz",
        type=float,
        default=120.0,
        metavar="N",
        help="Data polling rate in Hz (default: 120.0)",
    )
    parser.add_argument(
        "--render-rate-hz",
        type=float,
        default=60.0,
        metavar="N",
        help="Render rate in Hz (default: 60.0)",
    )
    parser.add_argument(
        "--viz-style",
        choices=["simple", "enhanced"],
        default="enhanced",
        help="Visualization style (default: enhanced)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable DEBUG log messages",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.WARNING)

    glove_viz_map: dict[int, GloveViz] = {}

    # Shared between data thread and render loop
    latest_data: dict[int, dict] = {}
    data_lock = threading.Lock()
    stop_event = threading.Event()

    with ManusDataPublisher(debug=args.debug) as pub:
        logger.info("Connected. Waiting for glove data...")

        while pub.GetLandscape() is None:
            time.sleep(0.1)

        logger.info("Landscape received. Starting data + render loops.")

        # Start data polling thread
        data_thread = threading.Thread(
            target=data_poll_loop,
            args=(pub, latest_data, data_lock, stop_event, args.data_rate_hz),
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
                        if args.viz_style == "enhanced":
                            glove_viz_map[glove_id] = common_viz.GloveViz(glove_id, data["side"])
                        else:
                            glove_viz_map[glove_id] = GloveViz(glove_id)

                    if args.viz_style == "enhanced":
                        update_glove_viz_enhanced(glove_viz_map[glove_id], data)
                    else:
                        update_glove_viz(glove_viz_map[glove_id], data)

                for gv in glove_viz_map.values():
                    gv.viz.poll_events()
                    gv.viz.update_renderer()

                time.sleep(1 / args.render_rate_hz)
        finally:
            stop_event.set()
            data_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
