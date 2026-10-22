"""Offline CAD mesh checks. No ROS, CAN, camera, or motor output."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

FINGERS = ("thumb", "index", "middle", "ring", "pinky")


class CollisionChecker:
    def __init__(self, urdf_path):
        self.path = Path(urdf_path).resolve()
        self.model = pin.buildModelFromUrdf(str(self.path))
        self.data = self.model.createData()
        self.geometry = pin.buildGeomFromUrdf(self.model, str(self.path), pin.GeometryType.COLLISION,
                                              package_dirs=[str(self.path.parent)])
        if len(self.geometry.geometryObjects) < 2:
            raise ValueError("Collision geometry unavailable; refusing an empty collision check")
        self.names = list(self.model.names)[1:]
        self.mimics = {j.get("name"): (j.find("mimic").get("joint"),
                                      float(j.find("mimic").get("multiplier", 1)),
                                      float(j.find("mimic").get("offset", 0)))
                       for j in ET.parse(self.path).getroot().findall("joint") if j.find("mimic") is not None}
        self.kinds = []
        for i, a in enumerate(self.geometry.geometryObjects):
            for j in range(i + 1, len(self.geometry.geometryObjects)):
                b = self.geometry.geometryObjects[j]
                fa, fb = self.finger(a.name), self.finger(b.name)
                kind = None
                if fa and fb and fa != fb:
                    kind = "cross_finger"
                elif a.name.startswith(("base_link", "wrist_base_link")) and self.distal_body(b.name) or \
                        b.name.startswith(("base_link", "wrist_base_link")) and self.distal_body(a.name):
                    kind = "palm_finger"
                if kind:
                    self.geometry.addCollisionPair(pin.CollisionPair(i, j))
                    self.kinds.append(kind)
        self.gdata = pin.GeometryData(self.geometry)

    @staticmethod
    def finger(name):
        return next((f for f in FINGERS if name.startswith(f + "_")), None)

    @staticmethod
    def distal_body(name):
        return any(part in name for part in ("_middle_", "_distal_"))

    def expand(self, named):
        named = dict(named)
        for name, (source, multiplier, offset) in self.mimics.items():
            if name not in named:
                named[name] = named[source] * multiplier + offset
        q = np.array([named[name] for name in self.names], dtype=float)
        if not np.isfinite(q).all():
            raise ValueError("Invalid collision-check configuration")
        return q

    def check(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.model.nq,) or not np.isfinite(q).all():
            raise ValueError("Expected all finite model joint positions")
        pin.computeCollisions(self.model, self.data, self.geometry, self.gdata, q, False)
        hits = []
        for pair, result, kind in zip(self.geometry.collisionPairs, self.gdata.collisionResults, self.kinds):
            if result.isCollision():
                hits.append({"a": self.geometry.geometryObjects[pair.first].name,
                             "b": self.geometry.geometryObjects[pair.second].name, "kind": kind})
        return hits

    def violations(self, q):
        return [{"joint": self.names[i], "angle_rad": float(q[i]),
                 "lower_rad": float(self.model.lowerPositionLimit[i]),
                 "upper_rad": float(self.model.upperPositionLimit[i])}
                for i in range(self.model.nq) if q[i] < self.model.lowerPositionLimit[i] - 1e-6 or
                q[i] > self.model.upperPositionLimit[i] + 1e-6]
