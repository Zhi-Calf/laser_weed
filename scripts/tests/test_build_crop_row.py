"""Test the build entry's scope/options/failure handling without building ROS."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


class BuildEntryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='crop-row-build-entry-')
        self.directory = Path(self.temp.name)
        self.calls = self.directory / 'calls.jsonl'
        self.env = dict(os.environ, ROS_DISTRO='humble',
                        PATH=f'{self.directory}:{os.environ["PATH"]}',
                        BUILD_TEST_CALLS=str(self.calls))
        colcon = self.directory / 'colcon'
        colcon.write_text(f'#!{sys.executable}\n'
                          'import json,os,sys\n'
                          'with open(os.environ["BUILD_TEST_CALLS"],"a") as f:\n'
                          ' f.write(json.dumps(sys.argv[1:])+"\\n")\n'
                          'sys.exit(int(os.environ.get("BUILD_TEST_EXIT","0")))\n')
        colcon.chmod(0o755)
        cmake = self.directory / 'cmake'
        cmake.write_text('#!/bin/sh\nexit 0\n')
        cmake.chmod(0o755)

    def tearDown(self):
        self.temp.cleanup()

    def run_entry(self, *args):
        result = subprocess.run(['bash', str(ROOT / 'scripts/build_crop_row.sh'), *args],
                                env=self.env, capture_output=True, text=True)
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()] if self.calls.exists() else []
        return result, calls

    def test_default_builds_sdk_and_observer_then_tests(self):
        result, calls = self.run_entry()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(calls), 3)
        self.assertIn(str(ROOT / 'row_nav_sdk'), calls[0])
        self.assertIn('--packages-up-to', calls[0])
        self.assertIn('crop_row_perception', calls[0])
        self.assertIn('-DRK_NAV_WITH_RKNN=OFF', calls[0])
        self.assertIn('row_nav_sdk', calls[1])
        self.assertIn('crop_row_perception', calls[1])
        self.assertNotIn('launch', sum(calls, []))

    def test_rknn_path_with_spaces_and_build_only(self):
        runtime = self.directory / 'runtime with spaces'
        runtime.mkdir()
        result, calls = self.run_entry('--with-rknn', '--rknn-root', str(runtime), '--build-only')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn('-DRK_NAV_WITH_RKNN=ON', calls[0])
        self.assertIn(f'-DRKNN_ROOT={runtime}', calls[0])

    def test_bad_arguments_never_build(self):
        result, calls = self.run_entry('/old/external/sdk')
        self.assertEqual(result.returncode, 2)
        self.assertFalse(calls)

    def test_missing_ros_never_builds(self):
        self.env.pop('ROS_DISTRO')
        result, calls = self.run_entry()
        self.assertEqual(result.returncode, 2)
        self.assertFalse(calls)

    def test_build_error_stops_before_tests(self):
        self.env['BUILD_TEST_EXIT'] = '7'
        result, calls = self.run_entry()
        self.assertEqual(result.returncode, 7)
        self.assertEqual(len(calls), 1)

    def test_package_graph_and_standalone_sdk(self):
        package = ET.parse(ROOT / 'row_nav_sdk/package.xml').getroot()
        self.assertEqual(package.findtext('name'), 'row_nav_sdk')
        self.assertEqual(package.findtext('export/build_type'), 'cmake')
        adapter = ET.parse(ROOT / 'crop_row_perception/package.xml').getroot()
        self.assertIn('row_nav_sdk', [x.text for x in adapter.findall('depend')])
        cmake = (ROOT / 'row_nav_sdk/CMakeLists.txt').read_text()
        self.assertNotIn('find_package(ament', cmake)
        self.assertNotIn('find_package(rclcpp', cmake)
        self.assertNotIn('../rk', cmake)
        sources = (ROOT / 'row_nav_sdk/cmake/RuntimeSources.cmake').read_text()
        for line in sources.splitlines():
            if '${ROW_NAV_SDK_SOURCE_DIR}/' in line:
                relative = line.strip().removesuffix(')').split('${ROW_NAV_SDK_SOURCE_DIR}/')[1]
                self.assertNotIn('..', Path(relative).parts)
                self.assertTrue((ROOT / 'row_nav_sdk' / relative).is_file(), relative)


if __name__ == '__main__':
    unittest.main()
