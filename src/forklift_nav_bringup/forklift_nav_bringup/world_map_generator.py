from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw, ImageFilter


COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
DEFAULT_RESOLUTION = 0.05
DEFAULT_PADDING = 0.20
DEFAULT_SPEED_VALUE = 100
DEFAULT_KEEPOUT_VALUE = 0
GENERATOR_VERSION = "world-map-generator-v3"
MIN_OBSTACLE_HEIGHT = 0.15
MAP_COMMENT = "# auto-generated from Gazebo world"
DEFAULT_OBSTACLE_EROSION_PIXELS = 2
IGNORED_MODEL_URI_FRAGMENTS = (
    "aws_robomaker_warehouse_Lamp_01",
    "aws_robomaker_warehouse_RoofB_01",
)


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class LocalFootprint:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float


@dataclass(frozen=True)
class LocalGeometry:
    polygons: tuple[tuple[tuple[float, float], ...], ...]
    min_z: float
    max_z: float


@dataclass(frozen=True)
class ArtifactPaths:
    map_png: str
    map_yaml: str
    keepout_png: str
    keepout_yaml: str
    speed_png: str
    speed_yaml: str
    manifest_json: str


def parse_pose(pose_text: str | None) -> Pose2D:
    if not pose_text:
        return Pose2D()

    values = [float(value) for value in pose_text.split()]
    while len(values) < 6:
        values.append(0.0)
    return Pose2D(x=values[0], y=values[1], z=values[2], yaw=values[5])


def compose_pose(parent: Pose2D, child: Pose2D) -> Pose2D:
    cos_yaw = math.cos(parent.yaw)
    sin_yaw = math.sin(parent.yaw)
    x = parent.x + child.x * cos_yaw - child.y * sin_yaw
    y = parent.y + child.x * sin_yaw + child.y * cos_yaw
    z = parent.z + child.z
    yaw = parent.yaw + child.yaw
    return Pose2D(x=x, y=y, z=z, yaw=yaw)


def rotate_point(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )


def footprint_to_polygon(footprint: LocalFootprint, pose: Pose2D) -> list[tuple[float, float]]:
    corners = [
        (footprint.min_x, footprint.min_y),
        (footprint.max_x, footprint.min_y),
        (footprint.max_x, footprint.max_y),
        (footprint.min_x, footprint.max_y),
    ]
    polygon = []
    for x, y in corners:
        rx, ry = rotate_point(x, y, pose.yaw)
        polygon.append((pose.x + rx, pose.y + ry))
    return polygon


def footprint_to_local_geometry(footprint: LocalFootprint) -> LocalGeometry:
    return LocalGeometry(
        polygons=(
            (
                (footprint.min_x, footprint.min_y),
                (footprint.max_x, footprint.min_y),
                (footprint.max_x, footprint.max_y),
                (footprint.min_x, footprint.max_y),
            ),
        ),
        min_z=footprint.min_z,
        max_z=footprint.max_z,
    )


def transform_polygon(polygon: Iterable[tuple[float, float]], pose: Pose2D) -> list[tuple[float, float]]:
    transformed = []
    for x, y in polygon:
        rx, ry = rotate_point(x, y, pose.yaw)
        transformed.append((pose.x + rx, pose.y + ry))
    return transformed


def polygon_bounds(polygon: Iterable[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    return min(xs), min(ys), max(xs), max(ys)


def resolve_model_dir(uri: str, search_roots: Iterable[Path]) -> Path | None:
    if not uri.startswith("model://"):
        return None

    model_name = uri[len("model://") :].split("/", 1)[0]
    for root in search_roots:
        candidate = root / model_name
        if candidate.is_dir():
            return candidate
    return None


def resolve_mesh_uri(uri: str, base_dir: Path, search_roots: Iterable[Path]) -> Path | None:
    if uri.startswith("model://"):
        model_dir = resolve_model_dir(uri, search_roots)
        if model_dir is None:
            return None
        relative = uri[len("model://") :].split("/", 1)
        if len(relative) == 1:
            return model_dir
        return model_dir / relative[1]

    if uri.startswith("file://"):
        return Path(uri[len("file://") :])

    return (base_dir / uri).resolve()


def read_collada_bounds(mesh_path: Path) -> LocalFootprint:
    root = ET.parse(mesh_path).getroot()
    asset = root.find("c:asset", COLLADA_NS)
    unit_scale = 1.0
    if asset is not None:
        unit = asset.find("c:unit", COLLADA_NS)
        if unit is not None and unit.attrib.get("meter"):
            unit_scale = float(unit.attrib["meter"])

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for float_array in root.findall(".//c:float_array", COLLADA_NS):
        array_id = float_array.attrib.get("id", "").upper()
        if "POSITION" not in array_id:
            continue

        values = [float(value) for value in (float_array.text or "").split()]
        for index in range(0, len(values), 3):
            x = values[index] * unit_scale
            y = values[index + 1] * unit_scale
            z = values[index + 2] * unit_scale
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)

    if not math.isfinite(min_x):
        raise RuntimeError(f"No POSITION array found in mesh {mesh_path}")

    return LocalFootprint(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        min_z=min_z,
        max_z=max_z,
    )


def _parse_collada_positions(root: ET.Element, unit_scale: float) -> dict[str, list[tuple[float, float, float]]]:
    positions: dict[str, list[tuple[float, float, float]]] = {}
    for source in root.findall(".//c:source", COLLADA_NS):
        float_array = source.find("c:float_array", COLLADA_NS)
        technique = source.find("c:technique_common/c:accessor", COLLADA_NS)
        if float_array is None or technique is None:
            continue

        stride = int(technique.attrib.get("stride", "3"))
        params = [param.attrib.get("name", "").upper() for param in technique.findall("c:param", COLLADA_NS)]
        if len(params) < 3 or params[:3] != ["X", "Y", "Z"]:
            continue

        values = [float(value) * unit_scale for value in (float_array.text or "").split()]
        source_points: list[tuple[float, float, float]] = []
        for index in range(0, len(values), stride):
            if index + 2 >= len(values):
                break
            source_points.append((values[index], values[index + 1], values[index + 2]))
        positions[f"#{source.attrib['id']}"] = source_points
    return positions


def _parse_collada_vertices(root: ET.Element) -> dict[str, str]:
    vertices_sources: dict[str, str] = {}
    for vertices in root.findall(".//c:vertices", COLLADA_NS):
        for input_tag in vertices.findall("c:input", COLLADA_NS):
            if input_tag.attrib.get("semantic") == "POSITION":
                vertices_sources[f"#{vertices.attrib['id']}"] = input_tag.attrib.get("source", "")
                break
    return vertices_sources


def _triangulate_indices(indices: list[int]) -> list[tuple[int, int, int]]:
    if len(indices) < 3:
        return []
    return [(indices[0], indices[index], indices[index + 1]) for index in range(1, len(indices) - 1)]


def read_collada_geometry(mesh_path: Path) -> LocalGeometry:
    root = ET.parse(mesh_path).getroot()
    asset = root.find("c:asset", COLLADA_NS)
    unit_scale = 1.0
    if asset is not None:
        unit = asset.find("c:unit", COLLADA_NS)
        if unit is not None and unit.attrib.get("meter"):
            unit_scale = float(unit.attrib["meter"])

    position_sources = _parse_collada_positions(root, unit_scale)
    vertices_sources = _parse_collada_vertices(root)

    polygons: list[tuple[tuple[float, float], ...]] = []
    min_z = float("inf")
    max_z = float("-inf")

    def resolve_points(source: str) -> list[tuple[float, float, float]]:
        resolved_source = vertices_sources.get(source, source)
        points = position_sources.get(resolved_source)
        if points is None:
            raise RuntimeError(f"Could not resolve COLLADA positions for source '{source}' in {mesh_path}")
        return points

    for triangles in root.findall(".//c:triangles", COLLADA_NS):
        inputs = triangles.findall("c:input", COLLADA_NS)
        if not inputs:
            continue
        vertex_input = next(
            (input_tag for input_tag in inputs if input_tag.attrib.get("semantic") in {"VERTEX", "POSITION"}),
            None,
        )
        if vertex_input is None:
            continue
        vertex_offset = int(vertex_input.attrib.get("offset", "0"))
        stride = max(int(input_tag.attrib.get("offset", "0")) for input_tag in inputs) + 1
        points = resolve_points(vertex_input.attrib.get("source", ""))
        raw_indices = [int(value) for value in (triangles.findtext("c:p", default="", namespaces=COLLADA_NS)).split()]
        for index in range(0, len(raw_indices), stride * 3):
            if index + stride * 3 > len(raw_indices):
                break
            polygon: list[tuple[float, float]] = []
            for vertex_index in range(3):
                point_index = raw_indices[index + vertex_index * stride + vertex_offset]
                x, y, z = points[point_index]
                polygon.append((x, y))
                min_z = min(min_z, z)
                max_z = max(max_z, z)
            polygons.append(tuple(polygon))

    for polylist in root.findall(".//c:polylist", COLLADA_NS):
        inputs = polylist.findall("c:input", COLLADA_NS)
        if not inputs:
            continue
        vertex_input = next(
            (input_tag for input_tag in inputs if input_tag.attrib.get("semantic") in {"VERTEX", "POSITION"}),
            None,
        )
        if vertex_input is None:
            continue
        vertex_offset = int(vertex_input.attrib.get("offset", "0"))
        stride = max(int(input_tag.attrib.get("offset", "0")) for input_tag in inputs) + 1
        points = resolve_points(vertex_input.attrib.get("source", ""))
        vcounts = [int(value) for value in (polylist.findtext("c:vcount", default="", namespaces=COLLADA_NS)).split()]
        raw_indices = [int(value) for value in (polylist.findtext("c:p", default="", namespaces=COLLADA_NS)).split()]
        cursor = 0
        for vertex_count in vcounts:
            polygon_indices: list[int] = []
            for _ in range(vertex_count):
                if cursor + vertex_offset >= len(raw_indices):
                    break
                polygon_indices.append(raw_indices[cursor + vertex_offset])
                cursor += stride
            for tri_indices in _triangulate_indices(polygon_indices):
                polygon: list[tuple[float, float]] = []
                for point_index in tri_indices:
                    x, y, z = points[point_index]
                    polygon.append((x, y))
                    min_z = min(min_z, z)
                    max_z = max(max_z, z)
                polygons.append(tuple(polygon))

    if not polygons:
        bounds = read_collada_bounds(mesh_path)
        return footprint_to_local_geometry(bounds)

    return LocalGeometry(polygons=tuple(polygons), min_z=min_z, max_z=max_z)


def scale_footprint(footprint: LocalFootprint, scale_xyz: tuple[float, float, float]) -> LocalFootprint:
    sx, sy, sz = scale_xyz
    xs = [footprint.min_x * sx, footprint.max_x * sx]
    ys = [footprint.min_y * sy, footprint.max_y * sy]
    zs = [footprint.min_z * sz, footprint.max_z * sz]
    return LocalFootprint(
        min_x=min(xs),
        max_x=max(xs),
        min_y=min(ys),
        max_y=max(ys),
        min_z=min(zs),
        max_z=max(zs),
    )


def scale_geometry(geometry: LocalGeometry, scale_xyz: tuple[float, float, float]) -> LocalGeometry:
    sx, sy, sz = scale_xyz
    scaled_polygons = tuple(
        tuple((x * sx, y * sy) for x, y in polygon)
        for polygon in geometry.polygons
    )
    zs = [geometry.min_z * sz, geometry.max_z * sz]
    return LocalGeometry(
        polygons=scaled_polygons,
        min_z=min(zs),
        max_z=max(zs),
    )


def parse_scale(scale_text: str | None) -> tuple[float, float, float]:
    if not scale_text:
        return (1.0, 1.0, 1.0)

    values = [float(value) for value in scale_text.split()]
    while len(values) < 3:
        values.append(1.0)
    return (values[0], values[1], values[2])


def should_skip_included_model(model_name: str, uri: str) -> bool:
    model_id = f"{model_name}::{uri}"
    return any(fragment in model_id for fragment in IGNORED_MODEL_URI_FRAGMENTS)


class ModelFootprintCache:
    def __init__(self, search_roots: Iterable[Path]) -> None:
        self.search_roots = [root.resolve() for root in search_roots if root.exists()]
        self._cache: dict[Path, tuple[list[tuple[LocalGeometry, Pose2D]], set[Path]]] = {}

    def get_shapes(self, model_dir: Path) -> tuple[list[tuple[LocalGeometry, Pose2D]], set[Path]]:
        model_dir = model_dir.resolve()
        cached = self._cache.get(model_dir)
        if cached is not None:
            return cached

        model_sdf = model_dir / "model.sdf"
        dependencies = {model_sdf}
        root = ET.parse(model_sdf).getroot()
        model = root.find("model") if root.tag == "sdf" else root
        if model is None:
            raise RuntimeError(f"Could not find <model> in {model_sdf}")

        model_pose = parse_pose(model.findtext("pose"))
        shapes = self._extract_model_shapes(
            model=model,
            base_pose=model_pose,
            base_dir=model_dir,
            dependencies=dependencies,
        )
        self._cache[model_dir] = (shapes, dependencies)
        return shapes, dependencies

    def _extract_model_shapes(
        self,
        *,
        model: ET.Element,
        base_pose: Pose2D,
        base_dir: Path,
        dependencies: set[Path],
    ) -> list[tuple[LocalGeometry, Pose2D]]:
        shapes: list[tuple[LocalGeometry, Pose2D]] = []
        for link in model.findall("link"):
            link_pose = compose_pose(base_pose, parse_pose(link.findtext("pose")))
            for collision in link.findall("collision"):
                collision_pose = compose_pose(link_pose, parse_pose(collision.findtext("pose")))
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                shapes.extend(
                    self._extract_geometry_shapes(
                        geometry=geometry,
                        pose=collision_pose,
                        base_dir=base_dir,
                        dependencies=dependencies,
                    )
                )
        return shapes

    def _extract_geometry_shapes(
        self,
        *,
        geometry: ET.Element,
        pose: Pose2D,
        base_dir: Path,
        dependencies: set[Path],
    ) -> list[tuple[LocalGeometry, Pose2D]]:
        if geometry.find("box") is not None:
            size_values = [float(value) for value in geometry.findtext("box/size", default="0 0 0").split()]
            while len(size_values) < 3:
                size_values.append(0.0)
            sx, sy, sz = size_values[:3]
            footprint = LocalFootprint(
                min_x=-sx / 2.0,
                max_x=sx / 2.0,
                min_y=-sy / 2.0,
                max_y=sy / 2.0,
                min_z=-sz / 2.0,
                max_z=sz / 2.0,
            )
            return [(footprint_to_local_geometry(footprint), pose)]

        if geometry.find("mesh") is not None:
            uri = geometry.findtext("mesh/uri", default="")
            mesh_path = resolve_mesh_uri(uri, base_dir, self.search_roots)
            if mesh_path is None or not mesh_path.exists():
                raise RuntimeError(f"Could not resolve mesh URI '{uri}' from {base_dir}")

            dependencies.add(mesh_path.resolve())
            mesh_geometry = read_collada_geometry(mesh_path)
            scale = parse_scale(geometry.findtext("mesh/scale"))
            mesh_geometry = scale_geometry(mesh_geometry, scale)
            return [(mesh_geometry, pose)]

        return []


def collect_world_shapes(world_path: Path, search_roots: Iterable[Path]) -> tuple[list[list[tuple[float, float]]], set[Path]]:
    world_path = world_path.resolve()
    dependencies = {world_path}
    model_cache = ModelFootprintCache(search_roots)
    root = ET.parse(world_path).getroot()
    world = root.find("world") if root.tag == "sdf" else root
    if world is None:
        raise RuntimeError(f"Could not find <world> in {world_path}")

    polygons: list[list[tuple[float, float]]] = []

    for model in world.findall("model"):
        model_pose = parse_pose(model.findtext("pose"))
        include = model.find("include")
        if include is not None:
            uri = include.findtext("uri", default="")
            if uri == "model://sun":
                continue
            if should_skip_included_model(model.attrib.get("name", ""), uri):
                continue
            model_dir = resolve_model_dir(uri, search_roots)
            if model_dir is None:
                raise RuntimeError(f"Could not resolve model URI '{uri}' from {world_path}")
            local_shapes, model_dependencies = model_cache.get_shapes(model_dir)
            dependencies.update(path.resolve() for path in model_dependencies)
            for local_geometry, local_pose in local_shapes:
                world_pose = compose_pose(model_pose, local_pose)
                if local_geometry.max_z + world_pose.z <= MIN_OBSTACLE_HEIGHT:
                    continue
                for polygon in local_geometry.polygons:
                    polygons.append(transform_polygon(polygon, world_pose))
            continue

        local_shapes = model_cache._extract_model_shapes(
            model=model,
            base_pose=model_pose,
            base_dir=world_path.parent,
            dependencies=dependencies,
        )
        for local_geometry, local_pose in local_shapes:
            if local_geometry.max_z + local_pose.z <= MIN_OBSTACLE_HEIGHT:
                continue
            for polygon in local_geometry.polygons:
                polygons.append(transform_polygon(polygon, local_pose))

    return polygons, dependencies


def compute_source_hash(dependencies: Iterable[Path]) -> str:
    hasher = hashlib.sha256()
    hasher.update(GENERATOR_VERSION.encode("utf-8"))
    for path in sorted({path.resolve() for path in dependencies}):
        hasher.update(str(path).encode("utf-8"))
        hasher.update(path.read_bytes())
    return hasher.hexdigest()


def compute_canvas(polygons: Iterable[list[tuple[float, float]]], resolution: float, padding: float) -> tuple[int, int, float, float]:
    min_x = min(point[0] for polygon in polygons for point in polygon)
    min_y = min(point[1] for polygon in polygons for point in polygon)
    max_x = max(point[0] for polygon in polygons for point in polygon)
    max_y = max(point[1] for polygon in polygons for point in polygon)

    min_x -= padding
    min_y -= padding
    max_x += padding
    max_y += padding

    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = int(math.ceil((max_x - origin_x) / resolution)) + 1
    height = int(math.ceil((max_y - origin_y) / resolution)) + 1
    return width, height, origin_x, origin_y


def world_to_pixel(x: float, y: float, origin_x: float, origin_y: float, resolution: float, height: int) -> tuple[int, int]:
    px = int(round((x - origin_x) / resolution))
    py = height - 1 - int(round((y - origin_y) / resolution))
    return px, py


def write_yaml(path: Path, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    text = (
        f"image: {image_name}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.6f}, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n\n"
        f"{MAP_COMMENT}\n"
    )
    path.write_text(text, encoding="utf-8")


def default_output_root() -> Path:
    artifact_root = Path(os.environ.get("ARTIFACT_ROOT", Path.home() / "ros2_forklift_warehouse_artifacts"))
    return artifact_root / "generated_maps"


def ensure_world_map(
    world_path: str | Path,
    *,
    output_root: str | Path | None = None,
    resolution: float = DEFAULT_RESOLUTION,
    padding: float = DEFAULT_PADDING,
    obstacle_erosion_pixels: int = DEFAULT_OBSTACLE_EROSION_PIXELS,
) -> ArtifactPaths:
    world_path = Path(world_path).expanduser().resolve()
    bringup_dir = world_path.parent.parent
    search_roots = [
        bringup_dir / "models",
        Path.cwd() / "src" / "forklift_nav_bringup" / "models",
        Path.cwd() / "src" / "third_party" / "aws-robomaker-small-warehouse-world" / "models",
    ]

    polygons, dependencies = collect_world_shapes(world_path, search_roots)
    if not polygons:
        raise RuntimeError(f"No obstacle geometry found in {world_path}")

    source_hash = compute_source_hash(dependencies)
    output_dir = Path(output_root or default_output_root()).expanduser().resolve() / world_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    map_png = output_dir / f"{world_path.stem}_map.png"
    map_yaml = output_dir / f"{world_path.stem}_map.yaml"
    keepout_png = output_dir / f"{world_path.stem}_keepout_mask.png"
    keepout_yaml = output_dir / f"{world_path.stem}_keepout_mask.yaml"
    speed_png = output_dir / f"{world_path.stem}_speed_mask.png"
    speed_yaml = output_dir / f"{world_path.stem}_speed_mask.yaml"
    manifest_json = output_dir / f"{world_path.stem}_manifest.json"

    if manifest_json.exists():
        manifest = json.loads(manifest_json.read_text(encoding="utf-8"))
        expected_files = [map_png, map_yaml, keepout_png, keepout_yaml, speed_png, speed_yaml]
        if manifest.get("source_hash") == source_hash and all(path.exists() for path in expected_files):
            return ArtifactPaths(
                map_png=str(map_png),
                map_yaml=str(map_yaml),
                keepout_png=str(keepout_png),
                keepout_yaml=str(keepout_yaml),
                speed_png=str(speed_png),
                speed_yaml=str(speed_yaml),
                manifest_json=str(manifest_json),
            )

    width, height, origin_x, origin_y = compute_canvas(polygons, resolution, padding)
    occupancy = Image.new("L", (width, height), 255)
    keepout = Image.new("L", (width, height), DEFAULT_KEEPOUT_VALUE)
    speed = Image.new("L", (width, height), DEFAULT_SPEED_VALUE)
    base_draw = ImageDraw.Draw(occupancy)

    for polygon in polygons:
        pixel_polygon = [
            world_to_pixel(x, y, origin_x, origin_y, resolution, height)
            for x, y in polygon
        ]
        base_draw.polygon(pixel_polygon, fill=0)

    for _ in range(max(0, obstacle_erosion_pixels)):
        occupancy = occupancy.filter(ImageFilter.MaxFilter(3))

    base_map = occupancy.convert("RGB")
    base_map.save(map_png)
    keepout.save(keepout_png)
    speed.save(speed_png)
    write_yaml(map_yaml, map_png.name, resolution, origin_x, origin_y)
    write_yaml(keepout_yaml, keepout_png.name, resolution, origin_x, origin_y)
    write_yaml(speed_yaml, speed_png.name, resolution, origin_x, origin_y)

    manifest = {
        "generator_version": GENERATOR_VERSION,
        "world_path": str(world_path),
        "source_hash": source_hash,
        "resolution": resolution,
        "padding": padding,
        "obstacle_erosion_pixels": obstacle_erosion_pixels,
        "width": width,
        "height": height,
        "origin": [origin_x, origin_y, 0.0],
        "dependencies": [str(path) for path in sorted(dependencies)],
    }
    manifest_json.write_text(json.dumps(manifest, indent=2, ensure_ascii=True), encoding="utf-8")

    return ArtifactPaths(
        map_png=str(map_png),
        map_yaml=str(map_yaml),
        keepout_png=str(keepout_png),
        keepout_yaml=str(keepout_yaml),
        speed_png=str(speed_png),
        speed_yaml=str(speed_yaml),
        manifest_json=str(manifest_json),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("world_path", help="Path to the Gazebo .world file")
    parser.add_argument("--output-root", default=str(default_output_root()))
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument("--padding", type=float, default=DEFAULT_PADDING)
    parser.add_argument(
        "--obstacle-erosion-pixels",
        type=int,
        default=DEFAULT_OBSTACLE_EROSION_PIXELS,
    )
    args = parser.parse_args()

    artifacts = ensure_world_map(
        args.world_path,
        output_root=args.output_root,
        resolution=args.resolution,
        padding=args.padding,
        obstacle_erosion_pixels=args.obstacle_erosion_pixels,
    )
    print(json.dumps(artifacts.__dict__, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
