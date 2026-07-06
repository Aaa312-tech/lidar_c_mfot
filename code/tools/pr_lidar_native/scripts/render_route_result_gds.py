import argparse
import inspect
import math
import sys
import warnings
from pathlib import Path

import yaml

warnings.filterwarnings("ignore", category=RuntimeWarning)


def load_yaml_compat(path):
    text = Path(path).read_text()
    try:
        return yaml.safe_load(text) or {}
    except yaml.YAMLError:
        try:
            return yaml.full_load(text) or {}
        except yaml.YAMLError:
            return yaml.unsafe_load(text) or {}


def clean_none_values(value):
    if isinstance(value, dict):
        return {
            key: clean_none_values(child)
            for key, child in value.items()
            if child is not None
        }
    if isinstance(value, list):
        return [clean_none_values(child) for child in value]
    return value


def make_lidar_render_config(lidar_src, lidar_data):
    from picroute.config.config import Config

    config = Config()
    for config_name in ("comp_LiDAR.yml", "default_config.yml"):
        config_path = lidar_src / "picroute" / "config" / config_name
        if config_path.exists():
            config.load(str(config_path), recursive=False)
            break

    dr_config = config.setdefault("dr", Config())
    if not isinstance(dr_config, Config):
        dr_config = Config(dr_config)
        config["dr"] = dr_config

    loss_comp = dr_config.setdefault("loss_comp", Config())
    if not isinstance(loss_comp, dict):
        loss_comp = Config()
        dr_config["loss_comp"] = loss_comp

    for macro_name in (lidar_data.get("library", {}) or {}).keys():
        loss_comp.setdefault(macro_name, 0.0)
    for instance in (lidar_data.get("instances", {}) or {}).values():
        settings = instance.get("settings", {}) or {}
        macro_type = settings.get("macro_type")
        if macro_type:
            loss_comp.setdefault(macro_type, 0.0)
    return config


def register_optional_gdsfactory_adapters():
    try:
        from gdsfactory_adapters import register_picbench_cells

        register_picbench_cells()
    except Exception:
        pass


def activate_gdsfactory_pdk(gf):
    try:
        gf.get_active_pdk()
        return
    except Exception:
        pass
    try:
        gf.gpdk.get_generic_pdk().activate()
        return
    except Exception:
        pass
    try:
        from gdsfactory.generic_tech import get_generic_pdk

        get_generic_pdk().activate()
    except Exception:
        pass


def sanitize_gdsfactory_layout_data(data):
    import gdsfactory as gf
    activate_gdsfactory_pdk(gf)
    register_optional_gdsfactory_adapters()

    def remove_module_keys(value):
        if isinstance(value, dict):
            value.pop("module", None)
            for child in value.values():
                remove_module_keys(child)
        elif isinstance(value, list):
            for child in value:
                remove_module_keys(child)

    remove_module_keys(data)

    for instance in (data.get("instances", {}) or {}).values():
        settings = instance.get("settings")
        if not isinstance(settings, dict):
            continue
        component = instance.get("component")
        if settings.get("cross_section") == "xs_sc":
            settings["cross_section"] = "strip"
        if isinstance(settings.get("cross_section"), dict):
            settings["cross_section"] = (
                "rib" if str(component).startswith("ring_") else "strip"
            )
        if isinstance(settings.get("pn_cross_section"), dict):
            settings.pop("pn_cross_section", None)
        if str(component).startswith("ring_") and isinstance(
            settings.get("heater_vias"), dict
        ):
            settings.pop("heater_vias", None)
        factory = gf.get_active_pdk().cells.get(component)
        if factory is None:
            continue
        signature = inspect.signature(factory)
        if any(
            parameter.kind == inspect.Parameter.VAR_KEYWORD
            for parameter in signature.parameters.values()
        ):
            continue
        allowed = {
            name
            for name, parameter in signature.parameters.items()
            if parameter.kind
            not in {
                inspect.Parameter.VAR_POSITIONAL,
                inspect.Parameter.VAR_KEYWORD,
            }
        }
        for key in list(settings.keys()):
            if key not in allowed:
                settings.pop(key, None)
    return data


def make_port(gf, data, layer, default_width):
    return gf.Port(
        name=data.get("name", "port"),
        center=tuple(data["center"]),
        orientation=float(data["orientation"]),
        width=float(data.get("width", default_width)),
        layer=layer,
    )


def port_data_with_gf_center(data, gf_port_by_db_name, default_width):
    db_name = data.get("name")
    if db_name not in gf_port_by_db_name:
        return data
    gf_data = dict(gf_port_by_db_name[db_name])
    # Keep the routed waveguide width. Some LiDAR PDK ports report grating widths
    # in db units, which triggers PortWidthMismatch in current gdsfactory/kfactory.
    gf_data["width"] = float(data.get("width", default_width))
    return gf_data


def route_path_port_data(data, default_width):
    port = dict(data)
    port["width"] = default_width
    return port


def crossing_candidate_from_port(data):
    base_orientation = {"o1": 180.0, "o2": 90.0, "o3": 0.0, "o4": 270.0}
    name = data.get("name")
    if name not in base_orientation:
        return None
    orientation = float(data.get("orientation", 0.0))
    x, y = [float(value) for value in data["center"]]
    theta = math.radians(orientation)
    center = (round(x - math.cos(theta) * 4.0, 6),
              round(y - math.sin(theta) * 4.0, 6))
    rotation = (orientation - base_orientation[name]) % 360.0
    return center, rotation


def infer_crossings_from_presplit_paths(paths_by_net):
    center_tolerance = 1e-5
    candidates = []
    for paths in paths_by_net.values():
        for path in paths:
            for key in ("start_port_data", "end_port_data"):
                candidate = crossing_candidate_from_port(path[key])
                if candidate is None:
                    continue
                center, rotation = candidate
                candidates.append((center, rotation))

    crossings = []
    used = [False] * len(candidates)
    for index, (center, rotation) in enumerate(candidates):
        if used[index]:
            continue
        cluster = [(center, rotation)]
        used[index] = True
        for other_index in range(index + 1, len(candidates)):
            if used[other_index]:
                continue
            other_center, other_rotation = candidates[other_index]
            if (
                abs(center[0] - other_center[0]) <= center_tolerance
                and abs(center[1] - other_center[1]) <= center_tolerance
                and abs((rotation - other_rotation + 180.0) % 360.0 - 180.0)
                <= 1e-6
            ):
                cluster.append((other_center, other_rotation))
                used[other_index] = True
        if len(cluster) < 2:
            continue
        # The port orientations encode the same crossing rotation from each side.
        avg_center = (
            round(sum(item[0][0] for item in cluster) / len(cluster), 6),
            round(sum(item[0][1] for item in cluster) / len(cluster), 6),
        )
        crossings.append((avg_center, round(cluster[0][1], 6)))
    return crossings


def simplify_collinear_points(points, eps=1e-9):
    if len(points) <= 2:
        return points
    simplified = [points[0]]
    for index in range(1, len(points) - 1):
        prev = simplified[-1]
        cur = points[index]
        nxt = points[index + 1]
        v1x = float(cur[0]) - float(prev[0])
        v1y = float(cur[1]) - float(prev[1])
        v2x = float(nxt[0]) - float(cur[0])
        v2y = float(nxt[1]) - float(cur[1])
        cross = v1x * v2y - v1y * v2x
        dot = v1x * v2x + v1y * v2y
        if abs(cross) <= eps and dot >= -eps:
            continue
        simplified.append(cur)
    simplified.append(points[-1])
    return simplified


def safe_route_bundle_sbend(
    route_bundle_sbend,
    component,
    ports1,
    ports2,
    *,
    allow_min_radius_violation=False,
    skip_invalid_access=False,
    invalid_accesses=None,
    label="",
):
    try:
        return route_bundle_sbend(component, ports1, ports2) or 0.0
    except ValueError as exc:
        if "min_bend_radius" not in str(exc):
            raise
        if skip_invalid_access:
            if invalid_accesses is not None:
                invalid_accesses.append(f"{label}: {exc}")
            return 0.0
        if not allow_min_radius_violation:
            raise
        try:
            return (
                route_bundle_sbend(
                    component,
                    ports1,
                    ports2,
                    allow_min_radius_violation=True,
                )
                or 0.0
            )
        except Exception as forced_exc:
            if invalid_accesses is not None:
                invalid_accesses.append(f"{label}: {forced_exc}")
            return 0.0


def generate_accessing_waveguide(
    gf,
    route_bundle_sbend,
    route,
    route_path,
    index,
    access_waveguide,
    width,
    layer,
    *,
    allow_min_radius_violation=False,
    skip_invalid_access=False,
    invalid_accesses=None,
    label="",
):
    port1 = route_path[index]
    orient = round(port1.orientation) % 360
    port2 = route.ports["o1"] if index == 1 else route.ports["o2"]

    if orient in {0, 90, 180, 270}:
        waveguide_length = safe_route_bundle_sbend(
            route_bundle_sbend,
            access_waveguide,
            [port1],
            [port2],
            allow_min_radius_violation=allow_min_radius_violation,
            skip_invalid_access=skip_invalid_access,
            invalid_accesses=invalid_accesses,
            label=label,
        )
        return None, waveguide_length

    x1, y1 = port1.dcenter
    x2, y2 = port2.dcenter
    dist = math.hypot(x2 - x1, y2 - y1)

    if orient in {45, 315}:
        fake_port1 = gf.Port(
            name="o1",
            orientation=0,
            center=(x1, y1),
            width=width,
            layer=layer,
        )
        fake_port2 = gf.Port(
            name="o2",
            orientation=180,
            center=(x1 + dist, y1),
            width=width,
            layer=layer,
        )
        rotation_angle = 45 if orient == 45 else -45
    elif orient in {135, 225}:
        fake_port1 = gf.Port(
            name="o1",
            orientation=180,
            center=(x1, y1),
            width=width,
            layer=layer,
        )
        fake_port2 = gf.Port(
            name="o2",
            orientation=0,
            center=(x1 - dist, y1),
            width=width,
            layer=layer,
        )
        rotation_angle = -45 if orient == 135 else 45
    else:
        waveguide_length = safe_route_bundle_sbend(
            route_bundle_sbend,
            access_waveguide,
            [port1],
            [port2],
            allow_min_radius_violation=allow_min_radius_violation,
            skip_invalid_access=skip_invalid_access,
            invalid_accesses=invalid_accesses,
            label=label,
        )
        return None, waveguide_length

    waveguide_length = safe_route_bundle_sbend(
        route_bundle_sbend,
        access_waveguide,
        [fake_port1],
        [fake_port2],
        allow_min_radius_violation=allow_min_radius_violation,
        skip_invalid_access=skip_invalid_access,
        invalid_accesses=invalid_accesses,
        label=label,
    )
    return (rotation_angle, x1, y1), waveguide_length


def split_line_string(line, distance, point_cls):
    if distance <= 0.0 or distance >= line.length:
        return None
    coords = list(line.coords)
    crossing_length_0 = 4.5
    crossing_length_45 = 4.5

    def split_with_clearance(first_point, second_point):
        first_distance = line.project(point_cls(first_point))
        second_distance = line.project(point_cls(second_point))
        eps = 1e-9
        first_part = [
            coord for coord in coords if line.project(point_cls(coord)) < first_distance - eps
        ]
        second_part = [
            coord for coord in coords if line.project(point_cls(coord)) > second_distance + eps
        ]
        return [first_part + [first_point], [second_point] + second_part]

    for i, _ in enumerate(coords):
        pd = line.project(point_cls(coords[i]))
        if pd <= distance:
            continue
        cp = line.interpolate(distance)
        x, y = coords[i]
        dx = cp.x - x
        dy = cp.y - y
        slope = 90 if dx == 0 else dy / dx
        if slope not in {float("inf"), float("-inf")}:
            slope = round(slope)
        else:
            slope = 90

        if slope == 0:
            if dx < 0:
                return (
                    split_with_clearance(
                        (cp.x - crossing_length_0, cp.y),
                        (cp.x + crossing_length_0, cp.y),
                    ),
                    ("o1", "o3"),
                    0,
                )
            return (
                split_with_clearance(
                    (cp.x + crossing_length_0, cp.y),
                    (cp.x - crossing_length_0, cp.y),
                ),
                ("o3", "o1"),
                0,
            )
        if slope == 90:
            if dy < 0:
                return (
                    split_with_clearance(
                        (cp.x, cp.y - crossing_length_0),
                        (cp.x, cp.y + crossing_length_0),
                    ),
                    ("o4", "o2"),
                    0,
                )
            return (
                split_with_clearance(
                    (cp.x, cp.y + crossing_length_0),
                    (cp.x, cp.y - crossing_length_0),
                ),
                ("o2", "o4"),
                0,
            )
        if slope == -1:
            if dy > 0:
                return (
                    split_with_clearance(
                        (cp.x - crossing_length_45, cp.y + crossing_length_45),
                        (cp.x + crossing_length_45, cp.y - crossing_length_45),
                    ),
                    ("o1", "o3"),
                    -45,
                )
            return (
                split_with_clearance(
                    (cp.x + crossing_length_45, cp.y - crossing_length_45),
                    (cp.x - crossing_length_45, cp.y + crossing_length_45),
                ),
                ("o3", "o1"),
                -45,
            )
        if slope == 1:
            if dy > 0:
                return (
                    split_with_clearance(
                        (cp.x + crossing_length_45, cp.y + crossing_length_45),
                        (cp.x - crossing_length_45, cp.y - crossing_length_45),
                    ),
                    ("o2", "o4"),
                    -45,
                )
            return (
                split_with_clearance(
                    (cp.x - crossing_length_45, cp.y - crossing_length_45),
                    (cp.x + crossing_length_45, cp.y + crossing_length_45),
                ),
                ("o4", "o2"),
                -45,
            )
    return None


def split_paths(line_string_cls, point_cls, net1_paths, net2_paths):
    for index1, subpath1 in enumerate(net1_paths):
        segment1 = line_string_cls(subpath1["points"])
        for index2, subpath2 in enumerate(net2_paths):
            segment2 = line_string_cls(subpath2["points"])
            intersection = segment1.intersection(segment2)
            if intersection.is_empty or not isinstance(intersection, point_cls):
                continue
            distance1 = segment1.project(intersection)
            distance2 = segment2.project(intersection)
            split1 = split_line_string(segment1, distance1, point_cls)
            split2 = split_line_string(segment2, distance2, point_cls)
            if split1 is None or split2 is None:
                continue
            segment1_parts, ports1, ori = split1
            segment2_parts, ports2, _ = split2
            return (
                [index1, segment1_parts, ports1],
                [index2, segment2_parts, ports2],
                [(intersection.x, intersection.y), ori],
            )
    return None


def add_crossing_component(gf, component, crossing):
    point, orientation = crossing
    crossing_component = gf.Component()
    ref = crossing_component << gf.components.crossing()
    ref.rotate(orientation)
    ref.move(point)
    crossing_component.add_ports(ref.ports)
    component.add_ref(crossing_component)
    return crossing_component.ports


def rewrite_legacy_layer44_vias(
    gds_path,
    *,
    layer=(44, 0),
    pitch_um=2.0,
    marker_size_um=0.7,
):
    """Restore the 5x5 layer-44 via arrays used by the reference LiDAR GDS."""
    import klayout.db as kdb

    layout = kdb.Layout()
    layout.read(str(gds_path))
    top = layout.top_cell()
    if top is None:
        return 0

    layer_index = layout.find_layer(*layer)
    if layer_index < 0:
        return 0

    dbu = float(layout.dbu)
    pitch = int(round(pitch_um / dbu))
    marker_size = int(round(marker_size_um / dbu))
    half_pitch = int(round((pitch_um / 2.0) / dbu))

    region = kdb.Region(top.begin_shapes_rec(layer_index))
    boxes = []
    for polygon in region.each():
        box = polygon.bbox()
        if box.width() == marker_size and box.height() == marker_size:
            boxes.append(box)

    if not boxes:
        return 0

    center_to_index = {
        ((box.left + box.right) // 2, (box.bottom + box.top) // 2): index
        for index, box in enumerate(boxes)
    }
    visited = set()
    clusters = []
    for center, index in center_to_index.items():
        if index in visited:
            continue
        stack = [center]
        cluster = []
        visited.add(index)
        while stack:
            current = stack.pop()
            current_index = center_to_index[current]
            cluster.append(current_index)
            cx, cy = current
            for neighbor in (
                (cx - pitch, cy),
                (cx + pitch, cy),
                (cx, cy - pitch),
                (cx, cy + pitch),
            ):
                neighbor_index = center_to_index.get(neighbor)
                if neighbor_index is None or neighbor_index in visited:
                    continue
                visited.add(neighbor_index)
                stack.append(neighbor)
        clusters.append(cluster)

    def is_regular_4x4(cluster):
        if len(cluster) != 16:
            return False
        llx = sorted({boxes[index].left for index in cluster})
        lly = sorted({boxes[index].bottom for index in cluster})
        if len(llx) != 4 or len(lly) != 4:
            return False
        if any(llx[i + 1] - llx[i] != pitch for i in range(3)):
            return False
        if any(lly[i + 1] - lly[i] != pitch for i in range(3)):
            return False
        expected = {(x, y) for x in llx for y in lly}
        actual = {(boxes[index].left, boxes[index].bottom) for index in cluster}
        return actual == expected

    rewritten_boxes = []
    converted_arrays = 0
    for cluster in clusters:
        if not is_regular_4x4(cluster):
            rewritten_boxes.extend(boxes[index] for index in cluster)
            continue
        min_x = min(boxes[index].left for index in cluster) - half_pitch
        min_y = min(boxes[index].bottom for index in cluster) - half_pitch
        for ix in range(5):
            for iy in range(5):
                left = min_x + ix * pitch
                bottom = min_y + iy * pitch
                rewritten_boxes.append(
                    kdb.Box(left, bottom, left + marker_size, bottom + marker_size)
                )
        converted_arrays += 1

    if converted_arrays == 0:
        return 0

    for cell in layout.each_cell():
        cell.shapes(layer_index).clear()
    top_shapes = top.shapes(layer_index)
    for box in rewritten_boxes:
        top_shapes.insert(box)
    layout.write(str(gds_path))
    return converted_arrays


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("lidar_src")
    parser.add_argument("route_result_yml")
    parser.add_argument("out_gds")
    parser.add_argument("--width", type=float, default=0.5)
    parser.add_argument("--layer", default="1,0")
    parser.add_argument("--no-access", action="store_true")
    parser.add_argument("--no-align", action="store_true")
    parser.add_argument("--no-crossing", action="store_true")
    parser.add_argument(
        "--ignore-presplit-paths",
        action="store_true",
        help=(
            "Ignore C++ post-processed paths and rebuild crossings from raw "
            "routed_path_um plus crossing_nets using the Python LiDAR split flow."
        ),
    )
    parser.add_argument(
        "--allow-min-radius-violation",
        action="store_true",
        help=(
            "Force gdsfactory bend_s generation even when the access S-bend "
            "violates radius_min. This preserves the old debug renderer behavior "
            "but does not match LiDAR/gdsfactory DRC semantics."
        ),
    )
    parser.add_argument(
        "--skip-invalid-access",
        action="store_true",
        help=(
            "Skip access S-bends that gdsfactory rejects for radius_min. The "
            "main route is still rendered, and skipped accesses are reported."
        ),
    )
    parser.add_argument(
        "--invalid-access-report",
        default=None,
        help="Optional text report listing skipped or forced-invalid access waveguides.",
    )
    parser.add_argument(
        "--skip-abnormal-nets",
        action="store_true",
        help=(
            "Skip nets listed in route_result.yml abnormal_nets. This is useful "
            "when comparing against Python LiDAR captures that popped those nets."
        ),
    )
    parser.add_argument(
        "--with-metadata",
        action="store_true",
        help=(
            "Keep gdsfactory/kfactory metadata properties in the GDS. Disabled "
            "by default because KLayout can reject properties containing inf."
        ),
    )
    parser.add_argument(
        "--legacy-layer44-vias",
        dest="legacy_layer44_vias",
        action="store_true",
        default=True,
        help="Rewrite current 4x4 layer-44 via arrays to the reference 5x5 convention.",
    )
    parser.add_argument(
        "--no-legacy-layer44-vias",
        dest="legacy_layer44_vias",
        action="store_false",
        help="Keep gdsfactory/kfactory's native layer-44 via arrays.",
    )
    parser.add_argument(
        "--base-lidar-yml",
        default=None,
        help="Optional LiDAR YAML benchmark to load as the device/layout base before adding routes.",
    )
    args = parser.parse_args()

    lidar_src = Path(args.lidar_src).resolve()
    sys.path.insert(0, str(lidar_src))

    import gdsfactory as gf
    import numpy as np
    from picroute.utils.route_bundle_sbend import route_bundle_sbend
    activate_gdsfactory_pdk(gf)
    register_optional_gdsfactory_adapters()

    if not hasattr(np, "asfarray"):
        np.asfarray = lambda values, *args, **kwargs: np.asarray(  # type: ignore[attr-defined]
            values, dtype=float, **kwargs
        )

    if not hasattr(gf.Component, "named_references"):
        gf.Component.named_references = property(  # type: ignore[attr-defined]
            lambda self: {inst.name: inst for inst in self.insts}
        )

    from picroute.utils.smooth import alignment, smooth
    from shapely.geometry import LineString, Point

    layer_tuple = tuple(int(part) for part in args.layer.split(","))
    layer = gf.get_layer(layer_tuple)
    data = yaml.safe_load(Path(args.route_result_yml).read_text())
    settings = data.get("settings", {})
    radius = float(settings.get("bend_radius", 5.0))
    out_gds = Path(args.out_gds).resolve()
    skipped_nets = set(data.get("abnormal_nets", []) or []) if args.skip_abnormal_nets else set()

    if args.base_lidar_yml:
        from picroute.database.schematic import CustomSchematic
        from picroute.database import schematic as schematic_mod

        original_to_yaml = schematic_mod.CustomNetlist.to_yaml

        def clean_generated_layout_yaml(self, filepath, *hook_args, **hook_kwargs):
            result = original_to_yaml(self, filepath, *hook_args, **hook_kwargs)
            layout_path = Path(filepath)
            try:
                layout_data = load_yaml_compat(layout_path)
                layout_path.write_text(
                    yaml.safe_dump(
                        sanitize_gdsfactory_layout_data(
                            clean_none_values(layout_data)
                        ),
                        sort_keys=False,
                    )
                )
            except Exception:
                pass
            return result

        schematic_mod.CustomNetlist.to_yaml = clean_generated_layout_yaml

        base_lidar_yml = Path(args.base_lidar_yml).resolve()
        cleaned_base_lidar_yml = out_gds.parent / f"{base_lidar_yml.stem}.render_clean.yml"
        base_lidar_data = clean_none_values(load_yaml_compat(base_lidar_yml))
        cleaned_base_lidar_yml.write_text(
            yaml.safe_dump(
                base_lidar_data,
                sort_keys=False,
            )
        )
        render_config = make_lidar_render_config(lidar_src, base_lidar_data)
        try:
            schematic = CustomSchematic(
                str(cleaned_base_lidar_yml),
                pdk=None,
                config=render_config,
            )
        except TypeError as exc:
            if "config" not in str(exc):
                raise
            schematic = CustomSchematic(str(cleaned_base_lidar_yml), pdk=None)
            try:
                schematic.config = render_config
            except Exception:
                pass
        schematic.load_gp()
        component = schematic.layout
        gf_port_by_db_name = {}
        for net in schematic.dbNets.values():
            for db_port in (net.NetPort1, net.NetPort2):
                gf_port = db_port.gf_port
                gf_port_by_db_name[db_port.port_name] = {
                    "name": db_port.port_name,
                    "center": [
                        float(gf_port.dcenter[0]),
                        float(gf_port.dcenter[1]),
                    ],
                    "orientation": float(gf_port.orientation),
                }
    else:
        component = gf.Component(f"{settings.get('design', 'picdb_lidar')}_routes")
        gf_port_by_db_name = {}
    route_count = 0
    access_count = 0
    crossing_count = 0
    total_length = 0.0
    route_paths = {}
    crossing_sets = {}
    invalid_accesses = []
    use_presplit_paths = not args.ignore_presplit_paths and any(
        bool(net.get("paths")) for net in data.get("nets", {}).values()
    )

    def add_short_sbend_route(net_name, net):
        nonlocal route_count, total_length
        start_port_data = port_data_with_gf_center(
            net.get("route_start_port", net.get("source_port", {})),
            gf_port_by_db_name,
            args.width,
        )
        end_port_data = port_data_with_gf_center(
            net.get("route_end_port", net.get("target_port", {})),
            gf_port_by_db_name,
            args.width,
        )
        if not start_port_data or not end_port_data:
            return
        short_component = gf.Component(name=f"{net_name}_short_sbend")
        start_port = make_port(gf, start_port_data, layer, args.width)
        end_port = make_port(gf, end_port_data, layer, args.width)
        total_length += safe_route_bundle_sbend(
            route_bundle_sbend,
            short_component,
            [start_port],
            [end_port],
            allow_min_radius_violation=args.allow_min_radius_violation,
            skip_invalid_access=args.skip_invalid_access,
            invalid_accesses=invalid_accesses,
            label=f"{net_name}/short_sbend",
        )
        component.add_ref(short_component)
        route_count += 1

    for net_name, net in data.get("nets", {}).items():
        if net_name in skipped_nets:
            continue
        if net.get("routed", False) and net.get("short_sbend", False):
            add_short_sbend_route(net_name, net)

    if use_presplit_paths:
        for net_name, net in data.get("nets", {}).items():
            if net_name in skipped_nets:
                continue
            if not net.get("routed", False):
                continue
            if net.get("short_sbend", False):
                continue
            for route_path in net.get("paths", []):
                points = route_path.get("points", [])
                if len(points) < 2:
                    continue
                points = simplify_collinear_points(points)
                start_port_data = route_path_port_data(
                    route_path["start_port"], args.width
                )
                end_port_data = route_path_port_data(
                    route_path["end_port"], args.width
                )
                start_port = make_port(gf, start_port_data, layer, args.width)
                end_port = make_port(gf, end_port_data, layer, args.width)
                route_paths.setdefault(net_name, []).append(
                    {
                        "points": points,
                        "start_port": start_port,
                        "end_port": end_port,
                        "start_port_data": start_port_data,
                        "end_port_data": end_port_data,
                    }
                )
        if not args.no_crossing:
            for crossing in infer_crossings_from_presplit_paths(route_paths):
                add_crossing_component(gf, component, crossing)
                crossing_count += 1
    else:
        for net_name, net in data.get("nets", {}).items():
            if net_name in skipped_nets:
                continue
            if not net.get("routed", False):
                continue
            if net.get("short_sbend", False):
                continue
            points = net.get("routed_path_um", [])
            if len(points) < 2:
                continue
            points = simplify_collinear_points(points)
            start_port_data = port_data_with_gf_center(
                net.get("route_start_port", net["source_port"]),
                gf_port_by_db_name,
                args.width,
            )
            end_port_data = port_data_with_gf_center(
                net.get("route_end_port", net["target_port"]),
                gf_port_by_db_name,
                args.width,
            )
            start_port = make_port(gf, start_port_data, layer, args.width)
            end_port = make_port(gf, end_port_data, layer, args.width)
            if not args.no_align:
                aligned_points = alignment(points, start_port, end_port, radius, bend=gf.path.euler)
                if aligned_points is not None:
                    points = aligned_points
            route_paths[net_name] = [
                {
                    "points": points,
                    "start_port": start_port,
                    "end_port": end_port,
                }
            ]
            crossing_sets[net_name] = set(
                net.get("raw_crossing_nets", net.get("crossing_nets", []))
            )

    if not use_presplit_paths and not args.no_crossing:
        for net_name in list(route_paths.keys()):
            for crossing_net_name in list(crossing_sets.get(net_name, set())):
                if crossing_net_name not in route_paths:
                    continue
                if net_name not in crossing_sets.get(crossing_net_name, set()):
                    continue
                crossing_sets[crossing_net_name].remove(net_name)
                split = split_paths(
                    LineString,
                    Point,
                    route_paths[net_name],
                    route_paths[crossing_net_name],
                )
                if split is None:
                    continue
                subpaths1, subpaths2, crossing = split
                crossing_ports = add_crossing_component(gf, component, crossing)
                crossing_count += 1

                index1, segment1_parts, ports1 = subpaths1
                index2, segment2_parts, ports2 = subpaths2
                subpath1 = route_paths[net_name][index1]
                subpath2 = route_paths[crossing_net_name][index2]

                route_paths[net_name][index1] = {
                    "points": segment1_parts[0],
                    "start_port": subpath1["start_port"],
                    "end_port": crossing_ports[ports1[0]],
                }
                route_paths[net_name].append(
                    {
                        "points": segment1_parts[1],
                        "start_port": crossing_ports[ports1[1]],
                        "end_port": subpath1["end_port"],
                    }
                )
                route_paths[crossing_net_name][index2] = {
                    "points": segment2_parts[0],
                    "start_port": subpath2["start_port"],
                    "end_port": crossing_ports[ports2[0]],
                }
                route_paths[crossing_net_name].append(
                    {
                        "points": segment2_parts[1],
                        "start_port": crossing_ports[ports2[1]],
                        "end_port": subpath2["end_port"],
                    }
                )

    for net_name, paths in route_paths.items():
        for path_index, route_path_entry in enumerate(paths):
            points = route_path_entry["points"]
            if len(points) < 2:
                continue
            try:
                path, _, _ = smooth(
                    points=points,
                    radius=radius - 1e-9,
                    bend=gf.path.euler,
                    use_eff=True,
                )
            except ValueError as exc:
                from gdsfactory import Path as GfPath

                print(
                    f"WARNING: smooth failed for {net_name}/{path_index}; "
                    f"using original polyline. Reason: {exc}",
                    file=sys.stderr,
                )
                path = GfPath(points)
            route = gf.path.extrude(path, width=args.width, layer=layer)
            final_component = gf.Component(name=f"{net_name}_{path_index}_route")
            if args.no_access:
                final_component.add_ref(route)
            else:
                route_path = [
                    points,
                    route_path_entry["start_port"],
                    route_path_entry["end_port"],
                ]
                access_waveguide1 = gf.Component()
                access_waveguide2 = gf.Component()
                rotation1, length1 = generate_accessing_waveguide(
                    gf,
                    route_bundle_sbend,
                    route,
                    route_path,
                    index=1,
                    access_waveguide=access_waveguide1,
                    width=args.width,
                    layer=layer,
                    allow_min_radius_violation=args.allow_min_radius_violation,
                    skip_invalid_access=args.skip_invalid_access,
                    invalid_accesses=invalid_accesses,
                    label=f"{net_name}/{path_index}/start",
                )
                rotation2, length2 = generate_accessing_waveguide(
                    gf,
                    route_bundle_sbend,
                    route,
                    route_path,
                    index=2,
                    access_waveguide=access_waveguide2,
                    width=args.width,
                    layer=layer,
                    allow_min_radius_violation=args.allow_min_radius_violation,
                    skip_invalid_access=args.skip_invalid_access,
                    invalid_accesses=invalid_accesses,
                    label=f"{net_name}/{path_index}/end",
                )
                ref1 = final_component << access_waveguide1
                ref2 = final_component << access_waveguide2
                if rotation1 is not None:
                    ref1.rotate(rotation1[0], (rotation1[1], rotation1[2]))
                if rotation2 is not None:
                    ref2.rotate(rotation2[0], (rotation2[1], rotation2[2]))
                final_component.add_ref(route)
                total_length += length1 + length2
                access_count += 2
            component.add_ref(final_component)
            route_count += 1
            total_length += route.info.model_extra.get("length", 0.0)

    out_gds.parent.mkdir(parents=True, exist_ok=True)
    component.write_gds(out_gds, with_metadata=bool(args.with_metadata))
    legacy_layer44_via_arrays = 0
    if args.legacy_layer44_vias:
        legacy_layer44_via_arrays = rewrite_legacy_layer44_vias(out_gds)
    if args.invalid_access_report:
        Path(args.invalid_access_report).write_text(
            "\n".join(invalid_accesses) + ("\n" if invalid_accesses else "")
        )
    print(f"routes={route_count}")
    print(f"access_waveguides={access_count}")
    print(f"invalid_access_waveguides={len(invalid_accesses)}")
    print(f"skipped_abnormal_nets={len(skipped_nets)}")
    print(f"crossings={crossing_count}")
    print(f"length={total_length:.6f}")
    print(f"legacy_layer44_via_arrays={legacy_layer44_via_arrays}")
    print(f"gds={out_gds}")


if __name__ == "__main__":
    main()
