#!/usr/bin/env python3
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

PKG = 'morpheus_description'


@pytest.fixture(scope='module')
def urdf_xml():
    pkg_dir = get_package_share_directory(PKG)
    xacro_file = f'{pkg_dir}/urdf/robot.xacro'
    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f'xacro failed:\n{result.stderr}'
    return result.stdout


class TestUrdfParsing:
    def test_xacro_produces_valid_xml(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        assert root.tag == 'robot'

    def test_robot_name(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        assert root.attrib.get('name') == 'morpheus_rover'

    def test_has_links(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        links = root.findall('.//link')
        assert len(links) > 0, 'No <link> elements found'

    def test_has_joints(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        joints = root.findall('.//joint')
        assert len(joints) > 0, 'No <joint> elements found'

    def test_chassis_link_exists(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        names = [link.attrib.get('name') for link in root.findall('.//link')]
        assert 'chassis_link' in names

    def test_no_duplicate_link_names(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        names = [link.attrib['name'] for link in root.findall('.//link')]
        assert len(names) == len(set(names)), f'Duplicate links: {names}'

    def test_no_duplicate_joint_names(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        names = [j.attrib['name'] for j in root.findall('./joint')]
        assert len(names) == len(set(names)), f'Duplicate joints: {names}'

    def test_all_joints_reference_existing_links(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        link_names = {link.attrib['name'] for link in root.findall('.//link')}
        for joint in root.findall('.//joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is not None:
                assert parent.attrib['link'] in link_names, \
                    f"Joint {joint.attrib['name']} parent '{parent.attrib['link']}' not found"
            if child is not None:
                assert child.attrib['link'] in link_names, \
                    f"Joint {joint.attrib['name']} child '{child.attrib['link']}' not found"

    def test_has_ros2_control_tag(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        r2c = root.findall('.//ros2_control')
        if not r2c:
            r2c = root.findall('.//{http://ros.org/wiki/ros2_control}ros2_control')
        assert len(r2c) > 0 or 'ros2_control' in urdf_xml, \
            'No ros2_control hardware interface found'
