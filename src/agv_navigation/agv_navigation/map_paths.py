"""Shared workspace and structured map path resolution helpers for agv_navigation.

Supports both structured map layouts (``<maps_root>/{yaml,pgm,png,json}/``) and
legacy flat layouts (``<maps_root>/``), across workspace root ``maps/``,
``src/agv_description/maps/``, and the installed ``agv_description`` share dir.
"""

from __future__ import annotations

import glob
import os
from typing import Iterable, List, Optional, Sequence

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # Allows standalone usage in offline ML venvs without ROS index
    get_package_share_directory = None  # type: ignore[assignment]

FORMAT_SUBDIRS = {
    '.yaml': 'yaml',
    '.yml': 'yaml',
    '.pgm': 'pgm',
    '.png': 'png',
    '.json': 'json',
}


def get_workspace_root() -> str:
    """Discover the AMR workspace root directory."""
    env_ws = os.environ.get('AMR_WS', '').strip()
    candidates: List[str] = [env_ws] if env_ws else []
    home = os.path.expanduser('~')
    candidates.extend([
        os.path.join(home, 'AMR', 'AMR-main'),
        os.path.join(home, 'AMR'),
    ])
    for cand in candidates:
        if cand and (
            os.path.exists(os.path.join(cand, 'src', 'agv_description'))
            or os.path.exists(os.path.join(cand, 'maps'))
        ):
            return cand
    return os.path.join(home, 'AMR', 'AMR-main')


def get_pkg_share_maps_dir() -> Optional[str]:
    """Return the installed ``agv_description/maps`` directory if available."""
    if get_package_share_directory is None:
        return None
    try:
        return os.path.join(get_package_share_directory('agv_description'), 'maps')
    except Exception:
        return None


def _build_map_roots(ws_root: Optional[str] = None, fallback_order: bool = False) -> List[str]:
    ws_base = ws_root or get_workspace_root()
    ws_hw_maps = os.path.join(ws_base, 'maps')
    ws_pkg_maps = os.path.join(ws_base, 'src', 'agv_description', 'maps')
    pkg_share_maps = get_pkg_share_maps_dir()

    if fallback_order:
        raw = [pkg_share_maps, ws_pkg_maps, ws_hw_maps]
    else:
        raw = [ws_hw_maps, ws_pkg_maps, pkg_share_maps]

    roots: List[str] = []
    for root in raw:
        if root and root not in roots:
            roots.append(root)
    return roots


def resolve_asset_in_root(map_root: str, filename: str) -> Optional[str]:
    """Locate ``filename`` under structured ``<map_root>/<fmt>/`` or flat ``<map_root>/``."""
    ext = os.path.splitext(filename)[1].lower()
    subdir = FORMAT_SUBDIRS.get(ext)
    candidates: List[str] = []
    if subdir:
        candidates.append(os.path.join(map_root, subdir, filename))
    candidates.append(os.path.join(map_root, filename))
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def _strip_suffixes(name: str, suffixes: Sequence[str]) -> str:
    clean = name.strip()
    for suf in suffixes:
        if clean.endswith(suf):
            return clean[: -len(suf)]
    return clean


def resolve_map_yaml(
    map_name: str = '',
    fallbacks: Iterable[str] = ('hospital_map', 'warehouse_map'),
    ws_root: Optional[str] = None,
) -> Optional[str]:
    """Resolve a map YAML file path across workspace and installed map directories."""
    ws_base = ws_root or get_workspace_root()
    raw_name = (map_name or '').strip()

    if raw_name:
        if os.path.isabs(raw_name) and os.path.exists(raw_name):
            return raw_name
        rel_cand = os.path.join(ws_base, raw_name)
        if not os.path.isabs(raw_name) and os.path.exists(rel_cand):
            return rel_cand

        stem = _strip_suffixes(os.path.basename(raw_name), ('.yaml', '.yml'))
        for root in _build_map_roots(ws_base, fallback_order=False):
            found = resolve_asset_in_root(root, f'{stem}.yaml')
            if found:
                return found

    fb_roots = _build_map_roots(ws_base, fallback_order=True)
    for root in fb_roots:
        for fb in fallbacks:
            fb_stem = _strip_suffixes(fb, ('.yaml', '.yml'))
            found = resolve_asset_in_root(root, f'{fb_stem}.yaml')
            if found:
                return found
    return None


def resolve_graph_json(
    map_name: str = '',
    fallbacks: Iterable[str] = ('warehouse_graph', 'warehouse_map_graph', 'warehouse_01_graph'),
    ws_root: Optional[str] = None,
) -> Optional[str]:
    """Resolve a topological roadmap JSON file across structured and flat map directories."""
    ws_base = ws_root or get_workspace_root()
    raw_name = (map_name or '').strip()

    if raw_name:
        if os.path.isabs(raw_name) and os.path.exists(raw_name):
            return raw_name
        rel_cand = os.path.join(ws_base, raw_name)
        if not os.path.isabs(raw_name) and os.path.exists(rel_cand):
            return rel_cand

        base = os.path.basename(raw_name)
        stem = _strip_suffixes(base, ('_graph.json', '.json', '.yaml', '.yml'))
        for root in _build_map_roots(ws_base, fallback_order=False):
            for fname in (f'{stem}_graph.json', f'{stem}.json'):
                found = resolve_asset_in_root(root, fname)
                if found:
                    return found

    fb_roots = _build_map_roots(ws_base, fallback_order=True)
    for root in fb_roots:
        for fb in fallbacks:
            fb_stem = _strip_suffixes(fb, ('.json',))
            found = resolve_asset_in_root(root, f'{fb_stem}.json')
            if found:
                return found
    return None


def resolve_places_json(map_name: str, ws_root: Optional[str] = None) -> Optional[str]:
    """Resolve a semantic places JSON file for ``map_name``."""
    ws_base = ws_root or get_workspace_root()
    stem = _strip_suffixes(
        os.path.basename((map_name or '').strip()),
        ('_places.json', '_graph.json', '.json', '.yaml', '.yml'),
    )
    if not stem:
        return None
    for root in _build_map_roots(ws_base, fallback_order=False):
        found = resolve_asset_in_root(root, f'{stem}_places.json')
        if found:
            return found
    return None


def resolve_map_image_from_yaml(yaml_path: str, yaml_meta: Optional[dict] = None) -> Optional[str]:
    """Resolve the occupancy image (.pgm/.png) referenced by ``yaml_path``."""
    img_field = str((yaml_meta or {}).get('image', '')).strip()
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    stem = os.path.splitext(os.path.basename(yaml_path))[0]

    candidates: List[str] = []
    if img_field:
        if os.path.isabs(img_field):
            candidates.append(img_field)
        else:
            candidates.append(os.path.normpath(os.path.join(yaml_dir, img_field)))
            if os.path.basename(yaml_dir) == 'yaml':
                base_dir = os.path.dirname(yaml_dir)
                candidates.append(os.path.join(base_dir, 'pgm', os.path.basename(img_field)))
                candidates.append(os.path.join(base_dir, 'png', os.path.basename(img_field)))

    if os.path.basename(yaml_dir) == 'yaml':
        base_dir = os.path.dirname(yaml_dir)
        candidates.append(os.path.join(base_dir, 'pgm', f'{stem}.pgm'))
        candidates.append(os.path.join(base_dir, 'png', f'{stem}.png'))
    candidates.append(os.path.join(yaml_dir, f'{stem}.pgm'))

    for cand in candidates:
        if os.path.exists(cand):
            return cand
    return None


def iter_map_yamls(ws_root: Optional[str] = None) -> List[str]:
    """Return all map YAML paths across workspace simulation and hardware map directories."""
    ws_base = ws_root or get_workspace_root()
    dirs = [
        os.path.join(ws_base, 'src', 'agv_description', 'maps', 'yaml'),
        os.path.join(ws_base, 'src', 'agv_description', 'maps'),
        os.path.join(ws_base, 'maps', 'yaml'),
        os.path.join(ws_base, 'maps'),
    ]
    found: List[str] = []
    seen = set()
    for d in dirs:
        for yf in sorted(glob.glob(os.path.join(d, '*.yaml'))):
            real = os.path.realpath(yf)
            if real not in seen:
                seen.add(real)
                found.append(yf)
    return found
