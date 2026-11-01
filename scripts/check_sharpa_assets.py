"""Validate Sharpa kinematics configs with cuRobo's URDF parser, without synthesis."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from curobo.cuda_robot_model.cuda_robot_generator import CudaRobotGeneratorConfig
from curobo.cuda_robot_model.urdf_kinematics_parser import UrdfKinematicsParser
from curobo.types.base import TensorDeviceType

REPO = Path(__file__).resolve().parents[1]


def main():
    reports = []
    for side in ("right", "left"):
        path = REPO / "src/curobo/content/configs/robot" / f"{side}_sharpa_wave_kinematics.yml"
        raw = yaml.safe_load(path.read_text())["robot_cfg"]["kinematics"]
        config = CudaRobotGeneratorConfig(**raw, tensor_args=TensorDeviceType(device="cpu"))
        urdf = Path(config.urdf_path).resolve()
        sys.path.insert(0, str(urdf.parent.parent / "scripts"))
        from validate_models import urdf_fk, require
        from joint_names import joint_names
        names = raw["cspace"]["joint_names"]
        require(names == list(joint_names(side)), "cuRobo cspace canonical order mismatch")
        require(not config.collision_link_names and config.collision_spheres is None,
                "Validation config must not imply calibrated collision coverage")
        parser = UrdfKinematicsParser(str(urdf), mesh_root=config.asset_root_path,
                                      build_scene_graph=True)
        original = ET.parse(urdf.parent.parent / "source/urdf" / urdf.name).getroot()
        moving = {j.get("name"): j for j in original.findall("joint") if j.get("type") != "fixed"}
        require(set(parser._robot.actuated_joint_names) == set(names), "Parser joint set mismatch")
        for joint in moving.values():
            params = parser.get_link_parameters(joint.find('child').get('link'))
            limit = joint.find('limit')
            np.testing.assert_allclose(params.joint_limits, [float(limit.get('lower')), float(limit.get('upper'))])
            velocity = float(limit.get('velocity'))
            np.testing.assert_allclose(params.joint_velocity_limits, [-velocity, velocity])
        limits = np.array([[float(moving[n].find('limit').get(k)) for k in ('lower', 'upper')] for n in names])
        rng = np.random.default_rng(1030)
        samples = np.vstack([np.zeros(22), limits[:, 0], limits[:, 1], rng.uniform(*limits.T, (16, 22))])
        max_error = 0.0
        for q in samples:
            values = dict(zip(names, q))
            parser._robot.update_cfg(values)
            for name, pose in urdf_fk(original, values).items():
                actual = parser._robot.get_transform(name, config.base_link)
                max_error = max(max_error, float(np.max(np.abs(actual-pose))))
        require(max_error < 1e-10, f"cuRobo parser FK mismatch: {max_error}")
        for mesh in ET.parse(urdf).findall('.//mesh'):
            require((Path(config.asset_root_path) / mesh.get('filename')).is_file(), "Missing mesh")
        reports.append(dict(side=side, joints=len(names), links=35, fk_samples=len(samples),
                            max_fk_matrix_error=max_error, collision_model="not configured",
                            scope="CPU URDF parser and config; CUDA kernels not exercised"))
        print(f"{side}: cuRobo config, 22 joint limits, 35 links and {len(samples)} FK poses passed")
    output = REPO / "docs/sharpa_validation.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
