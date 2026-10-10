"""A backend that "imports" the file with simcompat's own parser, plus injectable importer bugs.

It proves that each check catches its class of bug without needing any engine. It is also the
smallest complete example of the Backend interface (see docs/ARCHITECTURE.md, "Adding a backend").
"""

import copy

import numpy as np

from simcompat.backends.base import (
    CAP_IMPORT,
    CAP_INERTIA,
    CAP_KINEMATICS,
    MIMIC_IGNORED,
    Backend,
    BackendLoadError,
    BodyView,
    EngineQueryError,
    JointView,
    LoadInfo,
    Option,
)
from simcompat.declared import parse_urdf


class FakeBackend(Backend):
    name = "fake"
    is_simulator = False
    capabilities = frozenset({CAP_IMPORT, CAP_INERTIA, CAP_KINEMATICS})
    options_spec = {
        "flip_axis": Option("joint whose axis is negated", kind="text"),
        "scale_mass": Option("link@factor: multiply a link's mass", kind="text"),
        "nan_mass": Option("link whose mass becomes NaN", kind="text"),
        "shift_origin": Option("joint whose origin moves 1 cm along x", kind="text"),
        "drop_limits": Option("joint whose limits are dropped", kind="text"),
        "drop_damping": Option("joint whose damping is dropped", kind="text"),
        "reverse_order": Option("report joints in reverse order"),
        "armature": Option("armature injected on every joint", kind="number"),
        "fail_load": Option("refuse to load the model"),
        "fail_frames": Option("fail to answer joint_frames()"),
    }

    @classmethod
    def availability(cls):
        return True, "test"

    def load(self, prepared, declared, settings):
        if self.option_enabled("fail_load"):
            raise BackendLoadError("the fake importer refuses this model", hint="(test hint)")
        model = copy.deepcopy(parse_urdf(prepared.path))
        options = self.options
        if "flip_axis" in options:
            model.joints[options["flip_axis"]].axis *= -1
        if "scale_mass" in options:
            link, factor = options["scale_mass"].split("@")
            model.links[link].inertial.mass *= float(factor)
        if "nan_mass" in options:
            model.links[options["nan_mass"]].inertial.mass = float("nan")
        if "shift_origin" in options:
            model.joints[options["shift_origin"]].xyz = model.joints[options["shift_origin"]].xyz + np.array(
                [0.01, 0, 0]
            )
        self.model = model
        return LoadInfo(version="test")

    def joint_order(self):
        order = self.model.movable_joints
        return order[::-1] if self.option_enabled("reverse_order") else order

    def default_base(self):
        return "fixed"

    def joint_views(self):
        views = {}
        for name in self.model.movable_joints:
            joint = self.model.joints[name]
            limited = joint.limited and self.options.get("drop_limits") != name
            views[name] = JointView(
                name=name,
                kind="prismatic" if joint.type == "prismatic" else "revolute",
                limited=limited,
                lower=joint.lower if limited else None,
                upper=joint.upper if limited else None,
                effort=joint.effort,
                velocity=joint.velocity,
                damping=0.0 if self.options.get("drop_damping") == name else (joint.damping or 0.0),
                friction=joint.friction or 0.0,
                armature=float(self.options.get("armature", 0.0)),
                mimic=MIMIC_IGNORED,
            )
        return views

    def bodies(self):
        return [
            BodyView(name, carrier, mass, com, inertia)
            for name, carrier, mass, com, inertia in self.model.link_inertias()
        ]

    def joint_frames(self, q):
        if self.option_enabled("fail_frames"):
            raise EngineQueryError("fake engine cannot compute frames")
        return self.model.joint_frames(q)
