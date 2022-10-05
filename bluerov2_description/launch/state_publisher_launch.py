from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('camera', False)
    sl.declare_arg('rviz', False)
    
    namespace=sl.arg('namespace')
    
    with sl.group(ns=namespace):

        # xacro parsing + change moving joints to fixed if no Gazebo here
        xacro_args = {'namespace': namespace, 'simulation': sl.sim_time, 'camera': sl.arg('camera')}
        sl.robot_state_publisher('bluerov2_description', 'bluerov2.xacro', xacro_args=xacro_args)
        
    with sl.group(if_arg='rviz'):
        sl.node('rviz2', arguments = ['-d', sl.find('bluerov2_description', 'bluerov2.rviz')])

    return sl.launch_description()
