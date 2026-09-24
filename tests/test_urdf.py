# -*- coding: utf-8 -*-
"""Turning the upstream ROS 2 xacro into something PyBullet can load."""

import pybullet as p
import pytest

from litegrip_pybullet import (
    N_FINGERS,
    STROKE_M,
    UrdfError,
    bundle_dir,
    default_urdf,
    normalize_urdf,
    resolve_urdf,
)

BUNDLED_MESHES = bundle_dir() / "meshes"


class TestBundledAssets:
    def test_description_is_bundled(self):
        assert default_urdf().is_file()

    def test_meshes_are_bundled(self):
        stls = sorted(BUNDLED_MESHES.glob("*.STL"))
        assert len(stls) == 3, [s.name for s in stls]
        assert all(s.stat().st_size > 1000 for s in stls)


class TestNormalize:
    """``normalize_urdf`` on synthetic documents — no upstream file needed."""

    MINIMAL = """\
<robot name="g">
  <xacro:arg name="stroke" default="0.042726"/>
  <link name="base_footprint"/>
  <link name="base_link"><inertial><mass value="1"/></inertial></link>
  <link name="finger"/>
  <joint name="bf_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="base_link"/><child link="finger"/>
    <limit lower="0" upper="$(arg stroke)"/>
  </joint>
  <ros2_control name="X" type="system"><hardware><plugin>mock</plugin></hardware>
  </ros2_control>
</robot>
"""

    def test_xacro_arg_is_expanded_with_its_default(self):
        out = normalize_urdf(self.MINIMAL, BUNDLED_MESHES)
        assert f'upper="{STROKE_M}"' in out
        assert "$(arg" not in out

    def test_ros2_control_block_is_dropped(self):
        out = normalize_urdf(self.MINIMAL, BUNDLED_MESHES)
        assert "ros2_control" not in out

    def test_massless_root_is_dropped(self):
        out = normalize_urdf(self.MINIMAL, BUNDLED_MESHES)
        assert "base_footprint" not in out
        # ...and the joint hanging off it goes with it
        assert "bf_joint" not in out
        assert 'name="base_link"' in out

    def test_root_with_real_mass_is_kept(self):
        text = self.MINIMAL.replace(
            '<link name="base_footprint"/>',
            '<link name="base_footprint"><inertial><mass value="0.1"/></inertial>'
            "</link>",
        )
        out = normalize_urdf(text, BUNDLED_MESHES)
        assert "base_footprint" in out

    def test_package_mesh_uris_are_rewritten(self):
        text = self.MINIMAL.replace(
            '<link name="finger"/>',
            '<link name="finger"><visual><geometry><mesh '
            'filename="package://litegrip_urdf/meshes/finger.STL"/></geometry>'
            "</visual></link>",
        )
        out = normalize_urdf(text, BUNDLED_MESHES, "<test>")
        assert "package://" not in out
        assert f"{BUNDLED_MESHES}/finger.STL" in out

    def test_comments_are_stripped(self):
        text = self.MINIMAL.replace('<link name="finger"/>',
                                    "<!-- <link name=\"ghost\"/> -->"
                                    '<link name="finger"/>')
        assert "ghost" not in normalize_urdf(text, BUNDLED_MESHES)

    def test_unexpandable_expression_is_an_error(self):
        text = self.MINIMAL.replace("$(arg stroke)", "$(find some_pkg)/x")
        with pytest.raises(UrdfError, match="xacro"):
            normalize_urdf(text, BUNDLED_MESHES, "bad.xacro")

    def test_unsupported_xacro_tag_is_an_error(self):
        text = self.MINIMAL.replace("</robot>", "<xacro:if value='1'/></robot>")
        with pytest.raises(UrdfError, match="xacro"):
            normalize_urdf(text, BUNDLED_MESHES, "bad.xacro")

    def test_non_urdf_text_is_an_error(self):
        with pytest.raises(UrdfError, match="URDF"):
            normalize_urdf("this is not a robot", BUNDLED_MESHES)


class TestResolve:
    def test_returns_a_loadable_file(self):
        path = resolve_urdf()
        assert path.endswith(".urdf")
        assert "package://" not in open(path, encoding="utf-8").read()
        assert "$(" not in open(path, encoding="utf-8").read()

    def test_is_cached_within_the_process(self):
        assert resolve_urdf() == resolve_urdf()

    def test_explicit_path_is_honoured(self):
        assert resolve_urdf(default_urdf()) == resolve_urdf()

    def test_env_var_selects_the_directory(self, monkeypatch):
        monkeypatch.setenv("LITEGRIP_URDF_DIR", str(bundle_dir()))
        assert resolve_urdf() == resolve_urdf()

    def test_env_var_selects_the_file(self, monkeypatch):
        monkeypatch.setenv("LITEGRIP_URDF_PATH", str(default_urdf()))
        assert resolve_urdf() == resolve_urdf()

    def test_missing_file_is_an_error(self):
        with pytest.raises(UrdfError):
            resolve_urdf("/nonexistent/gripper.urdf")

    def test_missing_mesh_dir_is_an_error(self):
        with pytest.raises(UrdfError, match="网格目录"):
            resolve_urdf(default_urdf(), mesh_dir="/nonexistent/meshes")


@pytest.fixture(scope="module")
def body():
    """One DIRECT client with the gripper loaded, shared by the load tests."""
    cid = p.connect(p.DIRECT)
    try:
        yield cid, p.loadURDF(resolve_urdf(), useFixedBase=True,
                              physicsClientId=cid)
    finally:
        p.disconnect(physicsClientId=cid)


class TestLoadsInPyBullet:
    """The real test: PyBullet accepts it and finds the two finger joints."""

    def test_two_prismatic_finger_joints(self, body):
        cid, body_id = body
        prismatic = [
            jid for jid in range(p.getNumJoints(body_id, physicsClientId=cid))
            if p.getJointInfo(body_id, jid, physicsClientId=cid)[2]
            == p.JOINT_PRISMATIC
        ]
        assert len(prismatic) == N_FINGERS

    def test_travel_limits_are_the_stroke(self, body):
        cid, body_id = body
        for jid in range(p.getNumJoints(body_id, physicsClientId=cid)):
            info = p.getJointInfo(body_id, jid, physicsClientId=cid)
            if info[2] != p.JOINT_PRISMATIC:
                continue
            assert info[8] == pytest.approx(0.0)         # lower
            assert info[9] == pytest.approx(STROKE_M)    # upper

    def test_fingers_slide_along_opposite_axes(self, body):
        """Equal joint values must move the two jaws symmetrically."""
        cid, body_id = body
        axes = [
            tuple(p.getJointInfo(body_id, jid, physicsClientId=cid)[13])
            for jid in range(p.getNumJoints(body_id, physicsClientId=cid))
            if p.getJointInfo(body_id, jid, physicsClientId=cid)[2]
            == p.JOINT_PRISMATIC
        ]
        assert len(axes) == N_FINGERS
        assert axes[0][0] * axes[1][0] < 0  # opposite signs on X

    def test_no_massless_root_warning(self, body):
        """The dummy ``base_footprint`` is gone, so no link lacks inertia."""
        cid, body_id = body
        assert p.getNumJoints(body_id, physicsClientId=cid) == N_FINGERS

    def test_meshes_actually_load(self, body):
        """Collision shapes come from the STLs; empty ones mean bad paths."""
        cid, body_id = body
        for jid in range(p.getNumJoints(body_id, physicsClientId=cid)):
            lo, hi = p.getAABB(body_id, jid, physicsClientId=cid)
            assert hi[0] - lo[0] > 0.01, f"link {jid} 的 mesh 没加载"
            assert hi[2] - lo[2] > 0.01, f"link {jid} 的 mesh 没加载"
