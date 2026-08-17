"""Static checks for required Gazebo observability plugins."""

from pathlib import Path
from xml.etree import ElementTree


WORKSPACE = Path(__file__).resolve().parents[3]
SIMULATION = WORKSPACE / 'src' / 'robot_simulation'


def test_robot_model_covers_every_physical_collision_with_contact_sensor():
    """Require one bumper sensor for each robot collision shape."""
    model = ElementTree.parse(
        SIMULATION / 'models' / 'turtlebot3_burger_low_lidar' / 'model.sdf'
    )
    root = model.getroot()
    collisions = {
        collision.attrib['name']
        for collision in root.findall('.//link/collision')
    }
    observed = {
        sensor.find('contact/collision').text
        for sensor in root.findall('.//sensor[@type="contact"]')
    }
    assert collisions == observed == {
        'base_collision',
        'lidar_sensor_collision',
        'wheel_left_collision',
        'wheel_right_collision',
        'caster_collision',
    }
    for sensor in root.findall('.//sensor[@type="contact"]'):
        assert sensor.find('always_on').text == 'true'
        assert float(sensor.find('update_rate').text) == 100.0
        assert sensor.find('plugin').attrib['filename'] == 'libgazebo_ros_bumper.so'


def test_world_loads_gazebo_state_plugin_at_expected_rate():
    """Require model-state ground truth in the actual bottle-world file."""
    world = ElementTree.parse(
        SIMULATION / 'worlds' / 'target_car_basic.world'
    ).getroot()
    plugin = world.find('.//world/plugin[@name="gazebo_ros_state"]')
    assert plugin is not None
    assert plugin.attrib['filename'] == 'libgazebo_ros_state.so'
    assert plugin.find('ros/namespace').text == '/gazebo'
    assert float(plugin.find('update_rate').text) == 30.0


def test_control_only_launch_is_in_perception_install_manifest():
    """Ensure the runner's split control launch is installed by setuptools."""
    setup_file = (
        WORKSPACE / 'src' / 'robot_perception' / 'setup.py'
    ).read_text(encoding='ascii')
    assert "'launch/obstacle_control.launch.py'" in setup_file
