"""Gazebo simulation of the BMW i8 kids ride-on with the real car's navigation stack.

Everything the real car provides on the Jetson side is replaced by a simulation
counterpart, everything else is the real configuration (config/ is a verbatim
copy of the car project's sim_export):

    real car                                     simulation
    ----------------------------------------------------------------------------
    ESP32 (/cmd_vel, /steer_cmd, /steer_state)   sim_esp.py            -> Gazebo
    cuVSLAM (/visual_slam/tracking/odometry,     sim_odom.py           (ground truth)
             odom -> base_link)
    IMX219 stereo + camera_info                  Gazebo cameras + sim_camera_info.py
    nvblox 2D slice for the costmaps             ObstacleLayer on the simulated depth cloud
    Nav2 + ackermann_cmd_node                    unchanged (the car's nav2.launch.py)

Usage:
    ros2 launch bitt_sim sim.launch.py scenario:=parking_lot
    ros2 launch bitt_sim sim.launch.py world:=/path/to/world.sdf gui:=false

Scenarios are looked up in $BITT_SCENARIOS_DIR (default: <workspace>/scenarios).
"""

import copy
import glob
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'bitt_sim'
MODEL = 'bmw_kitt'
ODOM = ('odom', '/visual_slam/tracking/odometry')
DEFAULT_SCENARIOS = '/workspaces/bmw_industries_two_thousand/scenarios'

# Sim-only physical values: the real car does not publish them (robot_config.yaml: "not_modelled").
BODY_MASS = 30.0       # kg, kids car + battery
WHEEL_MASS = 1.5       # kg
WHEEL_FRICTION = 0.9
DEPTH_RANGE = (0.4, 5.0)   # m, stereo depth is unusable beyond ~5 m (f*B = 34 px*m)
DEPTH_SIZE = (320, 180)    # downsampled: the costmap does not need 640x360 points
DEPTH_RATE_HZ = 5


def cfg(*parts):
    return os.path.join(get_package_share_directory(PKG), 'config', *parts)


def find_world(scenario, world):
    """Return the SDF file for the launch arguments."""
    if world:
        if not os.path.isfile(world):
            raise RuntimeError(f'world file not found: {world}')
        return os.path.abspath(world)
    root = os.environ.get('BITT_SCENARIOS_DIR', DEFAULT_SCENARIOS)
    candidates = [scenario, os.path.join(root, scenario),
                  os.path.join(root, scenario + '.sdf'),
                  os.path.join(root, 'i8_' + scenario + '.sdf')]
    candidates += sorted(glob.glob(os.path.join(root, scenario, '*.sdf')))
    candidates += sorted(glob.glob(os.path.join(root, '**', scenario + '.sdf'), recursive=True))
    for c in candidates:
        if os.path.isfile(c) and c.endswith('.sdf'):
            return os.path.abspath(c)
    available = sorted(os.path.basename(p) for p in glob.glob(os.path.join(root, '**', '*.sdf'),
                                                                recursive=True))
    raise RuntimeError(
        f"scenario '{scenario}' not found in {root} (set BITT_SCENARIOS_DIR or mount the "
        f"scenarios folder). Available: {available or 'none'}")


def world_name(path):
    with open(path, encoding='utf-8') as f:
        m = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']', f.read())
    if not m:
        raise RuntimeError(f'no <world name="..."> in {path}')
    return m.group(1)


def _inertial(mass, ixx, iyy, izz, xyz='0 0 0'):
    e = ET.Element('inertial')
    ET.SubElement(e, 'origin', xyz=xyz, rpy='0 0 0')
    ET.SubElement(e, 'mass', value=str(mass))
    ET.SubElement(e, 'inertia', ixx=str(ixx), iyy=str(iyy), izz=str(izz), ixy='0', ixz='0',
                  iyz='0')
    return e


def gazebo_urdf(urdf_text, vehicle, calib):
    """The car's URDF has only fixed joints. Add what Gazebo needs to drive it:
    inertials, driven rear wheels, steerable front wheels, the depth sensor and the plugins.
    The TF tree published on the ROS side stays the original (fixed) one."""
    root = ET.fromstring(urdf_text)
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    r = vehicle['wheel_radius']

    links['body'].append(_inertial(BODY_MASS, 4.2, 15.0, 16.5))
    # solid cylinder, axis along y
    i_axis = 0.5 * WHEEL_MASS * r * r
    i_perp = WHEEL_MASS * (3 * r * r + vehicle['wheel_width'] ** 2) / 12
    for name in ('rear_left', 'rear_right', 'front_left', 'front_right'):
        links[f'{name}_wheel'].append(_inertial(WHEEL_MASS, i_perp, i_axis, i_perp))

    # Wheel cylinders are modelled with their axis along y (visual rpy), so spin about +y.
    def spin(joint):
        joint.set('type', 'continuous')
        for a in joint.findall('axis'):
            joint.remove(a)
        ET.SubElement(joint, 'axis', xyz='0 1 0')

    spin(joints['rear_left_wheel_joint'])
    spin(joints['rear_right_wheel_joint'])

    limit = vehicle['max_steering_angle'] * 1.6   # hub limit; the plugin clamps the command
    for side in ('left', 'right'):
        wj = joints[f'front_{side}_wheel_joint']
        origin = wj.find('origin').get('xyz')
        hub = ET.SubElement(root, 'link', name=f'{side}_steer_hub')
        hub.append(_inertial(0.5, 0.001, 0.001, 0.001))
        sj = ET.SubElement(root, 'joint', name=f'{side}_steer_joint', type='revolute')
        ET.SubElement(sj, 'parent', link='base_link')
        ET.SubElement(sj, 'child', link=f'{side}_steer_hub')
        ET.SubElement(sj, 'origin', xyz=origin, rpy='0 0 0')
        ET.SubElement(sj, 'axis', xyz='0 0 1')
        ET.SubElement(sj, 'limit', lower=str(-limit), upper=str(limit), effort='50',
                      velocity='2.0')
        wj.find('parent').set('link', f'{side}_steer_hub')
        wj.find('origin').set('xyz', '0 0 0')
        spin(wj)

    # Camera models: ideal pinhole with the calibrated fx, fy, cx, cy (no distortion).
    for sensor in root.iter('sensor'):
        key = {'stereo_left': 'left', 'stereo_right': 'right'}.get(sensor.get('name'))
        if key is None:
            continue
        k = calib[key]['camera_matrix']['data']
        lens = ET.SubElement(sensor.find('camera'), 'lens')
        intr = ET.SubElement(lens, 'intrinsics')
        for tag, val in (('fx', k[0]), ('fy', k[4]), ('cx', k[2]), ('cy', k[5]), ('s', 0)):
            ET.SubElement(intr, tag).text = str(val)

    camera_link = next(g for g in root.findall('gazebo') if g.get('reference') == 'camera_link')
    depth = ET.SubElement(camera_link, 'sensor', name='front_depth', type='rgbd_camera')
    for tag, val in (('always_on', 'true'), ('update_rate', DEPTH_RATE_HZ),
                     ('topic', 'front_depth'), ('gz_frame_id', 'camera_link')):
        ET.SubElement(depth, tag).text = str(val)
    cam = ET.SubElement(depth, 'camera')
    ET.SubElement(cam, 'horizontal_fov').text = '1.05097786'
    image = ET.SubElement(cam, 'image')
    ET.SubElement(image, 'width').text = str(DEPTH_SIZE[0])
    ET.SubElement(image, 'height').text = str(DEPTH_SIZE[1])
    clip = ET.SubElement(cam, 'clip')
    ET.SubElement(clip, 'near').text = str(DEPTH_RANGE[0])
    ET.SubElement(clip, 'far').text = str(DEPTH_RANGE[1])

    # Friction and plugins
    for wheel in ('rear_left', 'rear_right', 'front_left', 'front_right'):
        g = ET.SubElement(root, 'gazebo', reference=f'{wheel}_wheel')
        ET.SubElement(g, 'mu1').text = str(WHEEL_FRICTION)
        ET.SubElement(g, 'mu2').text = str(WHEEL_FRICTION)

    plugins = ET.SubElement(root, 'gazebo')

    def plugin(filename, name, **tags):
        p = ET.SubElement(plugins, 'plugin', filename=filename, name=name)
        for tag, val in tags.items():
            ET.SubElement(p, tag).text = str(val)

    # The stock Ackermann drive mode swings the wheels as v nears 0, so only the steering is
    # taken from it and the wheel speeds come from sim_esp.py.
    plugin('gz-sim-ackermann-steering-system', 'gz::sim::systems::AckermannSteering',
           steering_only='true', left_steering_joint='left_steer_joint',
           right_steering_joint='right_steer_joint',
           steering_limit=vehicle['max_steering_angle'], wheel_base=vehicle['wheelbase'],
           wheel_separation=vehicle['track_width'], topic='/gz_steer_angle', steer_p_gain=20.0)
    for side in ('left', 'right'):
        plugin('gz-sim-joint-controller-system', 'gz::sim::systems::JointController',
               joint_name=f'rear_{side}_wheel_joint', topic=f'/gz_{side}_wheel_cmd')
    plugin('gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher',
           odom_frame='odom', robot_base_frame='base_link', odom_publish_frequency=50,
           odom_topic='gz_odom', tf_topic='gz_tf_truth')

    return ET.tostring(root, encoding='unicode')


def nav2_params(vehicle):
    """The car's nav2_params.yaml with the nvblox costmap layer replaced by a stock obstacle layer
    on the simulated depth cloud (nvblox needs a Jetson/CUDA and the real stereo depth chain)."""
    with open(cfg('nav2_params.yaml')) as f:
        p = yaml.safe_load(f)
    obstacle = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'observation_sources': 'front_depth',
        'front_depth': {
            'topic': '/front_depth/points',
            'data_type': 'PointCloud2',
            'marking': True,
            'clearing': True,
            # odom is at base_link height, the ground is wheel_radius below: ignore the ground
            'min_obstacle_height': -0.5 * vehicle['wheel_radius'],
            'max_obstacle_height': vehicle['height'] + 0.1,
            'obstacle_min_range': DEPTH_RANGE[0],
            'obstacle_max_range': DEPTH_RANGE[1] - 0.5,
            'raytrace_min_range': 0.0,
            'raytrace_max_range': DEPTH_RANGE[1],
        },
    }
    for name in ('local_costmap', 'global_costmap'):
        c = p[name][name]['ros__parameters']
        c.pop('nvblox_layer', None)
        c['obstacle_layer'] = copy.deepcopy(obstacle)
        c['plugins'] = ['obstacle_layer' if x == 'nvblox_layer' else x for x in c['plugins']]
    p['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = cfg(
        'behavior_trees', 'navigate_to_pose_forward_only.xml')
    out = os.path.join(tempfile.gettempdir(), 'bitt_sim_nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f)
    return out


def launch_setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    world = find_world(scenario, LaunchConfiguration('world').perform(context))
    wname = world_name(world)

    with open(cfg('robot_config.yaml')) as f:
        vehicle = yaml.safe_load(f)['vehicle']
    with open(cfg('bmw_kitt.urdf')) as f:
        urdf = f.read()
    calib = {}
    for side in ('left', 'right'):
        with open(cfg('calib', f'{side}.yaml')) as f:
            calib[side] = yaml.safe_load(f)
    sim_urdf = gazebo_urdf(urdf, vehicle, calib)
    params = nav2_params(vehicle)
    sim_time = {'use_sim_time': True}

    resource_path = os.pathsep.join(
        [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.path.dirname(world)])
    gz_env = {'GZ_SIM_RESOURCE_PATH': resource_path}

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
        additional_env=gz_env, output='screen')
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], additional_env=gz_env, output='screen',
                            condition=IfCondition(LaunchConfiguration('gui')))

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', wname, '-name', MODEL, '-string', sim_urdf,
                            '-x', '0', '-y', '0', '-z', str(vehicle['wheel_radius'] + 0.05)])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[sim_time],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/gz_steer_angle@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz_left_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz_right_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/gz_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/visual_slam/image_0@sensor_msgs/msg/Image[gz.msgs.Image',
            '/visual_slam/image_1@sensor_msgs/msg/Image[gz.msgs.Image',
            '/back_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ])

    def node(package, executable, name=None, parameters=(), remappings=()):
        # Nav2 executables get their executable name as node name (as in the car's nav2.launch.py);
        # the python scripts name themselves.
        if name is None and not executable.endswith('.py'):
            name = executable
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=list(parameters) + [sim_time],
                    remappings=list(remappings))

    nav2 = [
        node('nav2_controller', 'controller_server', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ODOM]),
        node('nav2_smoother', 'smoother_server', parameters=[params]),
        node('nav2_planner', 'planner_server', parameters=[params]),
        node('nav2_behaviors', 'behavior_server', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ODOM]),
        node('nav2_bt_navigator', 'bt_navigator', parameters=[params], remappings=[ODOM]),
        node('nav2_velocity_smoother', 'velocity_smoother', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ODOM]),
        node('nav2_lifecycle_manager', 'lifecycle_manager', name='lifecycle_manager_navigation',
             parameters=[params]),
        node(PKG, 'ackermann_cmd_node.py', parameters=[{
            'wheelbase': vehicle['wheelbase'],
            'max_steering_angle': vehicle['max_steering_angle'],
            'max_speed': vehicle['max_speed'],
            'max_reverse_speed': vehicle.get('max_reverse_speed', 0.0),
        }]),
    ]

    sim_nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': urdf}, sim_time]),
        node(PKG, 'sim_esp.py', parameters=[{
            'wheel_radius': vehicle['wheel_radius'],
            'max_steering_angle': vehicle['max_steering_angle'],
        }]),
        node(PKG, 'sim_odom.py'),
        node(PKG, 'sim_camera_info.py', parameters=[{
            'calib_dir': cfg('calib'),
        }]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('foxglove'))),
    ]

    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            gz_server, gz_gui, spawn, bridge, *sim_nodes, *nav2]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='parking_lot',
                              description='scenario name inside $BITT_SCENARIOS_DIR'),
        DeclareLaunchArgument('world', default_value='',
                              description='path to a world .sdf (overrides scenario)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='also open the Gazebo GUI (needs a display)'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='start foxglove_bridge on port 8765'),
        OpaqueFunction(function=launch_setup),
    ])
