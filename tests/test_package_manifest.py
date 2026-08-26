import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import os
import subprocess
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class PackageManifestTest(unittest.TestCase):
    def test_manifest_declares_portable_build_dependencies(self):
        root = ET.parse(REPOSITORY_ROOT / "package.xml").getroot()
        dependencies = {
            element.text.strip()
            for element in root
            if element.tag.endswith("depend") and element.text
        }

        unavailable_private_dependencies = {"scrubber_common", "agibot_robot"}
        self.assertTrue(
            dependencies.isdisjoint(unavailable_private_dependencies),
            "A clean robot cannot resolve private dependencies that the source does not use",
        )

        required_dependencies = {
            "ament_cmake_auto",
            "builtin_interfaces",
            "eigen",
            "libgoogle-glog-dev",
            "libgflags-dev",
            "libopencv-dev",
            "libpcl-all-dev",
            "libtbb-dev",
            "rosbag2_cpp",
            "yaml-cpp",
        }
        self.assertEqual(set(), required_dependencies - dependencies)

    def test_dependency_installer_requests_direct_native_dependencies(self):
        installer = REPOSITORY_ROOT / "scripts" / "install_dep.sh"
        self.assertTrue(os.access(installer, os.X_OK), "The documented ./scripts/install_dep.sh command must be executable")

        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            captured_arguments = temp_path / "sudo-arguments.txt"
            fake_sudo = temp_path / "sudo"
            fake_sudo.write_text(
                "#!/bin/sh\n"
                f"printf '%s\\n' \"$@\" > '{captured_arguments}'\n",
                encoding="utf-8",
            )
            fake_sudo.chmod(0o755)

            environment = os.environ.copy()
            environment["PATH"] = f"{temp_path}:{environment['PATH']}"
            subprocess.run(
                ["bash", str(installer)],
                check=True,
                env=environment,
            )

            requested_arguments = set(captured_arguments.read_text(encoding="utf-8").splitlines())

        required_packages = {
            "libeigen3-dev",
            "libgoogle-glog-dev",
            "libgflags-dev",
            "libopencv-dev",
            "libpcl-dev",
            "libtbb-dev",
            "libyaml-cpp-dev",
            "ros-jazzy-ament-cmake-auto",
            "ros-jazzy-pcl-conversions",
            "ros-jazzy-rosbag2-cpp",
        }
        self.assertEqual(set(), required_packages - requested_arguments)


if __name__ == "__main__":
    unittest.main()
