"""Semantic transform-tree validation and camera-edge resolution."""
from __future__ import annotations

from collections import deque
from typing import Sequence

from .config import FrameRef, TransformSpec
from .errors import TransformGraphError


def validate_transform_tree(transforms: Sequence[TransformSpec], camera_names: Sequence[str]) -> None:
    required = {FrameRef.imu(), FrameRef.lidar(), *(FrameRef.camera(name) for name in camera_names)}
    adjacency: dict[FrameRef, list[tuple[FrameRef, int]]] = {node: [] for node in required}
    seen_edges: set[frozenset[FrameRef]] = set()
    for index, transform in enumerate(transforms):
        for endpoint_name, endpoint in (("source", transform.source), ("target", transform.target)):
            if endpoint not in required:
                raise TransformGraphError(
                    "TRANSFORM_FRAME_UNKNOWN", f"static_transforms[{index}].{endpoint_name}",
                    "transform references a semantic frame absent from the configured sensors",
                    expected=sorted(node.label for node in required), actual=endpoint.label,
                )
        edge = frozenset((transform.source, transform.target))
        if edge in seen_edges:
            raise TransformGraphError(
                "TRANSFORM_EDGE_DUPLICATE", f"static_transforms[{index}]",
                "duplicate or inverse-duplicate transform edge creates ambiguity",
                actual=sorted(node.label for node in edge),
            )
        seen_edges.add(edge)
        adjacency[transform.source].append((transform.target, index))
        adjacency[transform.target].append((transform.source, index))

    visited: set[FrameRef] = set()
    parent: dict[FrameRef, FrameRef | None] = {FrameRef.imu(): None}
    queue = deque([FrameRef.imu()])
    while queue:
        node = queue.popleft()
        visited.add(node)
        for neighbor, index in adjacency[node]:
            if neighbor == parent.get(node):
                continue
            if neighbor in visited or neighbor in parent:
                raise TransformGraphError(
                    "TRANSFORM_GRAPH_CYCLE", f"static_transforms[{index}]",
                    "static transforms must form an acyclic tree rooted at IMU",
                    actual=[node.label, neighbor.label],
                )
            parent[neighbor] = node
            queue.append(neighbor)

    missing = sorted((required - visited), key=lambda node: node.label)
    if missing:
        first = missing[0]
        code = "TRANSFORM_PATH_MISSING" if first.role.value == "camera" else "TRANSFORM_GRAPH_DISCONNECTED"
        raise TransformGraphError(
            code, "static_transforms",
            f"no static transform path exists from {first.label} to IMU",
            actual=[node.label for node in missing],
        )
    if len(transforms) != len(required) - 1:
        raise TransformGraphError(
            "TRANSFORM_EDGE_COUNT_INVALID", "static_transforms",
            "a connected transform tree must contain exactly one fewer edge than frames",
            expected=len(required) - 1, actual=len(transforms),
        )


def calibration_edge_for_camera(
    transforms: Sequence[TransformSpec], camera_name: str
) -> tuple[int, TransformSpec]:
    """Return the first edge from a camera toward IMU in the unique validated tree."""
    camera = FrameRef.camera(camera_name)
    adjacency: dict[FrameRef, list[tuple[FrameRef, int]]] = {}
    for index, transform in enumerate(transforms):
        adjacency.setdefault(transform.source, []).append((transform.target, index))
        adjacency.setdefault(transform.target, []).append((transform.source, index))
    if camera not in adjacency:
        raise TransformGraphError(
            "TRANSFORM_PATH_MISSING", f"cameras['{camera_name}']",
            f"no static transform path exists from camera:{camera_name} to IMU",
        )
    queue = deque([(camera, None, None)])
    visited: set[FrameRef] = set()
    while queue:
        node, parent, first_edge = queue.popleft()
        if node == FrameRef.imu():
            assert first_edge is not None
            return first_edge, transforms[first_edge]
        visited.add(node)
        for neighbor, edge_index in adjacency.get(node, []):
            if neighbor != parent and neighbor not in visited:
                queue.append((neighbor, node, edge_index if first_edge is None else first_edge))
    raise TransformGraphError(
        "TRANSFORM_PATH_MISSING", f"cameras['{camera_name}']",
        f"no static transform path exists from camera:{camera_name} to IMU",
    )
