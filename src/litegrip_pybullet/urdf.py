"""Locate and normalise the LiteGrip URDF for PyBullet.

The upstream description lives in the ``litegrip-urdf`` repository and is a ROS 2
xacro: it uses ``$(arg …)`` substitutions, ``package://litegrip_urdf/meshes/…``
mesh URIs, a ``<ros2_control>`` block, and a massless ``base_footprint`` root
link.  PyBullet can load none of that as-is, so
:func:`resolve_urdf` rewrites the description into a plain URDF in a temporary
file and hands PyBullet that path:

* xacro arguments are expanded with their **declared defaults** (so the
  ``stroke`` etc. of the upstream file are used unchanged);
* ``package://…/meshes/`` URIs are pointed at the mesh directory on disk;
* the ``<ros2_control>`` block is dropped;
* a massless root link (``base_footprint``) and its joint are dropped, leaving
  ``base_link`` as the root.

Anything it cannot rewrite (``$(find …)``, ``<xacro:if>``, macros …) raises
:class:`UrdfError` instead of silently producing a wrong model.

Resolution order:

1. the ``urdf_path`` argument (or ``$LITEGRIP_URDF_PATH`` for a file,
   ``$LITEGRIP_URDF_DIR`` for a ``litegrip-urdf`` checkout);
2. the copy bundled with this package
   (``litegrip_pybullet/assets/litegrip_urdf``, synced from ``litegrip-urdf``);
3. a sibling ``litegrip-urdf`` checkout next to this repository.
"""

from __future__ import annotations

import atexit
import os
import re
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

__all__ = [
    "UrdfError",
    "bundle_dir",
    "default_urdf",
    "resolve_urdf",
    "normalize_urdf",
]


class UrdfError(RuntimeError):
    """The URDF could not be found or rewritten for PyBullet."""


_PACKAGE_ROOT = Path(__file__).resolve().parent
_BUNDLE_DIR = _PACKAGE_ROOT / "assets" / "litegrip_urdf"

#: Rewritten descriptions, keyed by (source file, mesh dir, mtime).  Kept for the
#: process lifetime and removed on exit; PyBullet only reads the file at load.
_CACHE: Dict[Tuple[str, str, float], str] = {}
_TEMP_FILES: List[str] = []


def _cleanup() -> None:
    for path in _TEMP_FILES:
        try:
            os.unlink(path)
        except OSError:
            pass


atexit.register(_cleanup)


# ── Locating the description ─────────────────────────────────────────────────

def bundle_dir() -> Path:
    """Directory of the copy bundled with this package (may not exist)."""
    return _BUNDLE_DIR


def default_urdf() -> Path:
    """Path of the bundled description file (may not exist)."""
    return _BUNDLE_DIR / "urdf" / "litegrip_urdf.urdf.xacro"


def _repo_root() -> Optional[Path]:
    """Repository root when running from a checkout, else ``None``."""
    try:
        return _PACKAGE_ROOT.parents[1]
    except IndexError:  # installed somewhere shallower than src/<pkg>
        return None


def _sibling_dir() -> Optional[Path]:
    """A ``litegrip-urdf`` checkout next to this repository, if any."""
    root = _repo_root()
    if root is None:
        return None
    sibling = root.parent / "litegrip-urdf"
    return sibling if (sibling / "urdf").is_dir() else None


def _dir_pair(directory: Path) -> Tuple[Path, Path]:
    """``dir`` → ``(description file, mesh dir)`` for a ``litegrip-urdf`` tree.

    Accepts either the repository root (holding ``urdf/`` and ``meshes/``) or
    the ``urdf/`` directory itself.
    """
    root = directory if (directory / "urdf").is_dir() else directory.parent
    urdf_dir = root / "urdf"
    meshes = root / "meshes"
    if not urdf_dir.is_dir():
        raise UrdfError(f"{directory} 下没有 urdf/ 目录，不是 litegrip-urdf 布局")
    for pattern in ("*.urdf.xacro", "*.urdf", "*.xacro"):
        found = sorted(urdf_dir.glob(pattern))
        if found:
            return found[0], meshes
    raise UrdfError(f"{urdf_dir} 下没有 .urdf / .urdf.xacro 文件")


def _pair_for_path(path: Path) -> Tuple[Path, Path]:
    """Resolve an explicit description file to ``(file, mesh dir)``."""
    path = path.expanduser()
    if not path.is_file():
        raise UrdfError(
            f"URDF 文件不存在：{path}\n"
            f"（可用 --urdf 指定，或设置 LITEGRIP_URDF_PATH / LITEGRIP_URDF_DIR）"
        )
    if path.parent.name == "urdf":
        return path, path.parent.parent / "meshes"
    return path, path.parent / "meshes"


def _source() -> Tuple[Path, Path]:
    """First available ``(description file, mesh dir)`` per the search order."""
    env_file = os.environ.get("LITEGRIP_URDF_PATH")
    if env_file:
        return _pair_for_path(Path(env_file))
    env_dir = os.environ.get("LITEGRIP_URDF_DIR")
    if env_dir:
        return _dir_pair(Path(env_dir).expanduser())
    if default_urdf().is_file():
        return default_urdf(), _BUNDLE_DIR / "meshes"
    sibling = _sibling_dir()
    if sibling is not None:
        return _dir_pair(sibling)
    raise UrdfError(
        "找不到 LiteGrip 的 URDF。按以下任一方式提供：\n"
        f"  1) 仓库自带资产（应当存在于 {default_urdf()}）；\n"
        "  2) 环境变量 LITEGRIP_URDF_PATH=/path/to/litegrip_urdf.urdf.xacro\n"
        "     或 LITEGRIP_URDF_DIR=/path/to/litegrip-urdf；\n"
        "  3) 把 litegrip-urdf 仓库克隆到本仓库的同级目录。"
    )


# ── Rewriting the description ────────────────────────────────────────────────

_COMMENT_RE = re.compile(r"<!--.*?-->", re.S)
_XACRO_ARG_DECL_RE = re.compile(r"<xacro:arg\b[^>]*/>")
_XACRO_ARG_NAME_RE = re.compile(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"\s*/>')
_ROS2_CONTROL_RE = re.compile(r"<ros2_control\b.*?</ros2_control>", re.S)
_PACKAGE_MESH_RE = re.compile(r"package://[^/\"]+/meshes/")


def normalize_urdf(text: str, mesh_dir: Path, source: str = "<memory>") -> str:
    """Rewrite a ``litegrip-urdf`` description into plain URDF text.

    Args:
        text: Contents of the upstream ``.urdf`` / ``.urdf.xacro`` file.
        mesh_dir: Directory holding the ``.STL`` meshes.
        source: Path used in error messages.

    Raises:
        UrdfError: on a document that is not a <robot>, on a xacro construct that
            cannot be expanded here, or on an unexpanded ``$(…)``.
    """
    if "<robot" not in text:
        raise UrdfError(f"{source} 不是 URDF（找不到 <robot> 元素）")

    text = _COMMENT_RE.sub("", text)

    # 1. Expand xacro arguments with their declared defaults.
    args = dict(_XACRO_ARG_NAME_RE.findall(text))
    text = _XACRO_ARG_DECL_RE.sub("", text)
    for name, value in args.items():
        text = text.replace(f"$(arg {name})", value)
    leftover = re.findall(r"\$\([^)]*\)", text)
    if leftover:
        raise UrdfError(
            f"{source} 里有无法展开的 xacro 表达式 {sorted(set(leftover))}；"
            f"请先用 xacro 生成普通 URDF（xacro {source} > out.urdf）"
        )
    unsupported = sorted(set(re.findall(r"<xacro:([A-Za-z_]+)", text)))
    if unsupported:
        raise UrdfError(
            f"{source} 使用了本工具不支持的 xacro 标签 {unsupported}；"
            f"请先用 xacro 生成普通 URDF（xacro {source} > out.urdf）"
        )

    # 2. Drop the ROS 2 control block and a massless dummy root link.
    text = _ROS2_CONTROL_RE.sub("", text)
    text = _strip_massless_root(text)

    # 3. Point mesh URIs at the meshes on disk.
    text = _PACKAGE_MESH_RE.sub(str(mesh_dir).rstrip("/") + "/", text)
    text = text.replace("file://", "")
    return text


_LINK_BLOCK_RE = re.compile(
    r'<link\s+name="([^"]+)"[^>]*?(?:/>|>.*?</link>)', re.S
)


def _link_blocks(text: str) -> List[Tuple[str, str]]:
    """All ``(name, block)`` pairs for the ``<link>`` elements of a URDF.

    Matches self-closing ``<link name="…"/>`` as well as full blocks — a
    self-closing link that is swept up into the *next* ``</link>`` would look
    like it owns that link's inertia.
    """
    return [(m.group(1), m.group(0)) for m in _LINK_BLOCK_RE.finditer(text)]


def _strip_massless_root(text: str) -> str:
    """Remove a massless root link (``base_footprint``) and the joint below it.

    PyBullet builds one rigid body per URDF tree and needs its root to carry
    mass.  ROS 2 descriptions habitually bolt a massless ``base_footprint`` on
    top of the real base so KDL has a proper tree root; dropping it makes
    ``base_link`` the root without changing any geometry.
    """
    blocks = dict(_link_blocks(text))
    joints = [m.group(0) for m in
              re.finditer(r"<joint\b.*?</joint>", text, re.S)]
    children = set()
    for joint in joints:
        child = re.search(r'<child\s+link="([^"]+)"', joint)
        if child:
            children.add(child.group(1))
    roots = [name for name in blocks if name not in children and name in blocks]
    if len(roots) != 1 or len(blocks) < 2:
        return text
    root = roots[0]
    if re.search(r"<inertial\b", blocks[root]):
        return text  # a root with real mass is part of the robot, keep it
    text = text.replace(blocks[root], "")
    for joint in joints:
        if re.search(r'<(?:parent|child)\s+link="%s"' % re.escape(root), joint):
            text = text.replace(joint, "")
    return text


def resolve_urdf(
    urdf_path: Optional[os.PathLike | str] = None,
    mesh_dir: Optional[os.PathLike | str] = None,
) -> str:
    """Return a path to a PyBullet-loadable URDF, rewriting it if needed.

    Args:
        urdf_path: Explicit ``.urdf`` / ``.urdf.xacro`` to use instead of the
            search order.  A path already normalised by this function is
            returned as-is (the rewrite is idempotent).
        mesh_dir: Overrides the mesh directory inferred from ``urdf_path``.

    Returns:
        Absolute path of the rewritten description.  It lives in a temporary
        file owned by this process and is deleted when the process exits.
    """
    if urdf_path is None:
        source, meshes = _source()
    else:
        source, meshes = _pair_for_path(Path(urdf_path))
    if mesh_dir is not None:
        meshes = Path(mesh_dir).expanduser()
    if not meshes.is_dir():
        raise UrdfError(
            f"网格目录不存在：{meshes}（URDF 里的 mesh 路径要指向它）"
        )

    key = (str(source), str(meshes), source.stat().st_mtime)
    cached = _CACHE.get(key)
    if cached is not None and Path(cached).is_file():
        return cached

    text = normalize_urdf(source.read_text(encoding="utf-8"), meshes, str(source))
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix=f"{source.stem}_")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        handle.write(text)
    _TEMP_FILES.append(path)
    _CACHE[key] = path
    return path
