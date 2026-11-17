"""Check physics tensors, resources, control interfaces and tree integrity."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory


def test_description():
    share = Path(get_package_share_directory('sieglinde_sim'))
    root = ET.fromstring(xacro.process_file(str(share / 'urdf/tracker/tracker.urdf.xacro')).toxml())
    links = {link.get('name') for link in root.findall('link')}
    children = [j.find('child').get('link') for j in root.findall('joint')]
    assert len(children) == len(set(children))
    assert links - set(children) == {'base_link'}
    for mesh in root.findall('.//mesh'):
        package, path = mesh.get('filename').removeprefix('package://').split('/', 1)
        assert (Path(get_package_share_directory(package)) / path).is_file()
    for link in root.findall('link'):
        inertia = link.find('inertial/inertia')
        if inertia is None:
            assert link.get('name') in {'base_link', 'camera_link', 'camera_optical_frame', 'tool0'}
            continue
        a = {key: float(value) for key, value in inertia.attrib.items()}
        values = np.linalg.eigvalsh([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]])
        assert values[0] > 0, link.get('name')
        assert values[2] <= values[0] + values[1] + 1e-9, link.get('name')
    for joint in root.findall('joint'):
        if joint.find('mimic') is not None:
            control = root.find(f"ros2_control/joint[@name='{joint.get('name')}']")
            assert control is not None and control.find('command_interface') is None
