from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=True)
    
    # Declare arguments
    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('namespace', default_value='orca')
    sl.declare_arg('ground_truth', default_value=True)
    sl.declare_arg('camera', default_value=True)
    sl.declare_arg('gazebo_world_name', default_value='none')

    # Declare initial pose
    sl.declare_gazebo_axes(x=1., y=0., z=1., roll=0., pitch=0., yaw=0.)

    # Handle GUI argument
    with sl.group(if_arg='gui'):
        sl.gz_launch(sl.find('orca_sim', 'demo_world.sdf'), "-r")
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch(sl.find('orca_sim', 'demo_world.sdf'), "-r -s")

    # Set Gazebo world name if provided
    if sl.arg('gazebo_world_name') != 'none':
        GazeboBridge.set_world_name(sl.arg('gazebo_world_name'))

    ns = sl.arg('namespace')

    # Include state publisher
    sl.include('orca_sim', 'state_publisher_launch.py',
               launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=ns):
        # URDF spawner to Gazebo
        sl.spawn_gz_model(ns, spawn_args=sl.gazebo_axes_args())

        # ROS-Gz bridges
        bridges = [GazeboBridge.clock(),
                   GazeboBridge('/ocean_current', '/current', 'geometry_msgs/Vector3', GazeboBridge.ros2gz)]

        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # Pose ground truth
        bridges.append(GazeboBridge(f'/model/{ns}/pose', 'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # Odometry
        bridges.append(GazeboBridge(f'/model/{ns}/odometry', 'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros))

        # IMU
        for imu in ('mpu', 'lsm'):
            bridges.append(GazeboBridge(f'{ns}/{imu}', imu, 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        # Camera
        if sl.arg('camera'):
            bridges.append(GazeboBridge(f'{ns}/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))

        # sonar (lidar)
        bridges.append(GazeboBridge(f'{ns}/sonar_scan', 'sonar_scan', 'sensor_msgs/LaserScan', GazeboBridge.gz2ros))
        bridges.append(GazeboBridge(f'{ns}/sonar_scan/points', 'sonar_cloud', 'sensor_msgs/PointCloud2', GazeboBridge.gz2ros))

        # current
        bridges.append(GazeboBridge('/ocean_current', '/ocean_current', 'geometry_msgs/Vector3', GazeboBridge.ros2gz))
        
        # Thrusters
        for thr in range(1, 9):
            thruster = f'thruster{thr}'
            gz_thr_topic = f'/{ns}/{thruster}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        sl.create_gz_bridge(bridges)

        # Ground truth to TF if requested
        if sl.arg('ground_truth'):
            sl.node('pose_to_tf', parameters={'child_frame': ns + '/base_link'})
        else:
            sl.node('pose_to_tf', parameters={'child_frame': ns + '/base_link_gt'})

    return sl.launch_description()

