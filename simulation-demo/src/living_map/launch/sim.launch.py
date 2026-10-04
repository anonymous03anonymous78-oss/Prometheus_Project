"""The Living Map v3.2 - full Phase 1 simulation (aligned with the technical report).

    ros2 launch living_map sim.launch.py scenario:=all speed:=2

scenario: nominal | writer_destroyed | beacon_knockout | false_positive | all
world:    auto | fuel | basic   (auto = SubT Fuel tiles if they are in the local Fuel cache,
                                 otherwise the offline fallback galleries)
camera:   auto | manual (auto = the director drives the Gazebo camera through the story)
gui:      true | false  (false = headless Gazebo server)
speed:    real-time factor asked from Gazebo (1 = real time, 2 = twice as fast if the PC can)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCENARIOS = ('nominal', 'writer_destroyed', 'beacon_knockout', 'false_positive', 'all')

# (executable, node name, extra parameters): two Writers, two Flix, three Outside Network Areas
NODES = [('writer_node', 'writer_a', {'robot': 'writer_a'}),
         ('writer_node', 'writer_b', {'robot': 'writer_b'}),
         ('executor_node', 'executor', {'robot': 'executor'}),
         ('flix_node', 'flix_a', {'robot': 'flix_a'}),
         ('flix_node', 'flix_b', {'robot': 'flix_b'}),
         ('beacon_network', 'beacon_network', {}),
         ('outside_network', 'ona_a', {'ona': 'A'}),
         ('outside_network', 'ona_b', {'ona': 'B'}),
         ('outside_network', 'ona_c', {'ona': 'C'}),
         ('command_post', 'command_post', {}),
         ('environment', 'environment', {}),
         ('gz_actions', 'gz_actions', {}),
         ('director_node', 'director', {}),
         ('dataflow_view', 'dataflow_view', {})]

# Fuel models the full world needs (downloaded once by setup.sh)
FUEL_MODELS = ('tunnel tile 1', 'tunnel tile 5', 'tunnel tile 6', 'rescue randy sitting')

# Gazebo Fortress <-> ROS 2 Humble bridge.  ]  = ROS -> Gazebo,  [  = Gazebo -> ROS
BRIDGE = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
for _r in ('writer_a', 'writer_b', 'executor'):
    BRIDGE += [f'/model/{_r}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
               f'/model/{_r}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
               f'/{_r}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
for _r in ('writer_a', 'writer_b', 'executor', 'flix_a', 'flix_b'):
    BRIDGE += [f'/model/{_r}/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
               f'/{_r}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU']
for _r in ('flix_a', 'flix_b'):
    BRIDGE += [f'/model/{_r}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
               f'/{_r}/tof_down@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
               f'/{_r}/tof@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']


def fuel_cached():
    """True if every Fuel model of the full world is in the local Fuel cache."""
    roots = [os.path.expanduser('~/.ignition/fuel'), os.path.expanduser('~/.gz/fuel')]
    found = set()
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, _ in os.walk(root):
            for d in dirnames:
                if d.lower() in FUEL_MODELS:
                    found.add(d.lower())
            if dirpath.count(os.sep) - root.count(os.sep) > 4:
                dirnames[:] = []
    return all(m in found for m in FUEL_MODELS)


def setup(context):
    share = get_package_share_directory('living_map')
    scenario = LaunchConfiguration('scenario').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('1', 'true', 'yes')
    world_arg = LaunchConfiguration('world').perform(context).lower()
    if scenario not in SCENARIOS:
        raise RuntimeError(f'unknown scenario {scenario!r}; use one of {SCENARIOS}')
    kind = world_arg if world_arg in ('fuel', 'basic') else ('fuel' if fuel_cached() else 'basic')
    world = os.path.join(share, 'worlds', 'mine.sdf' if kind == 'fuel' else 'mine_basic.sdf')
    gz_cmd = ['ign', 'gazebo', '-r', world] if gui else ['ign', 'gazebo', '-r', '-s', world]
    cam_auto = LaunchConfiguration('camera').perform(context).lower() != 'manual'
    params = [{'use_sim_time': True, 'scenario': scenario, 'world_kind': kind,
               'camera_auto': cam_auto}]
    models = os.path.join(share, 'models')
    res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models + (os.pathsep + res if res else '')),
        LogInfo(msg=f'[living_map] scenario = {scenario} | world = {kind} '
                    f'({"SubT Fuel tiles" if kind == "fuel" else "offline fallback galleries"}) '
                    f'| 2D window: http://localhost:8765'),
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             arguments=BRIDGE, output='screen'),
    ]
    if world_arg == 'auto' and kind == 'basic':
        actions.insert(1, LogInfo(msg='[living_map] SubT Fuel models not found in the local cache '
                                      '-> using the offline world. Run setup.sh with internet to '
                                      'download them.'))
    for exe, name, extra in NODES:
        actions.append(Node(package='living_map', executable=exe, name=name,
                            parameters=[dict(params[0], **extra)], output='screen', emulate_tty=True))
    speed = float(LaunchConfiguration('speed').perform(context) or 1.0)
    if abs(speed - 1.0) > 1e-3:
        # ask Gazebo for a faster real-time factor (it runs as fast as the PC allows)
        actions.append(TimerAction(period=12.0, actions=[ExecuteProcess(cmd=[
            'ign', 'service', '-s', '/world/living_map/set_physics', '--reqtype',
            'ignition.msgs.Physics', '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
            '--req', f'max_step_size: 0.004, real_time_factor: {speed}'], output='screen')]))
        actions.append(LogInfo(msg=f'[living_map] asking Gazebo for real-time factor {speed}'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal',
                              description='nominal | writer_destroyed | beacon_knockout | '
                                          'false_positive | all'),
        DeclareLaunchArgument('world', default_value='auto', description='auto | fuel | basic'),
        DeclareLaunchArgument('camera', default_value='auto', description='auto | manual'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('speed', default_value='1.0',
                              description='Gazebo real-time factor (1 = real time, 2 = 2x)'),
        OpaqueFunction(function=setup),
    ])
