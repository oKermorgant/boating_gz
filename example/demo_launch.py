from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('manual', True)

    # launch world and clock / current / wind bridges
    sl.include('boating_gz', 'world_launch.py',
            launch_arguments={'world_file': sl.find('boating_gz', 'sydney_regatta.sdf'),
                                'gz_args': ''})

    name = 'sailboat'

    # spawn sailboat
    with sl.group(ns = name):

        sl.robot_state_publisher('boating_gz', 'sailboat.xacro')
        sl.spawn_gz_model(name)

        # run bridges
        bridges = []

        # add bridges for this model
        bridges = []
        bridges.append((f'/world/sydney_regatta/model/{name}/joint_state',
                        'joint_states',
                        'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # main prop
        bridges.append((f'/{name}/thrust',
                        'thrust',
                        'std_msgs/Float64', GazeboBridge.ros2gz))
        # rudder and wing
        for joint in ('rudder','wing'):
            bridges.append((f'/{name}/{joint}/pos',
                        f'{joint}/cmd_pos',
                        'std_msgs/Float64', GazeboBridge.ros2gz))



        # TODO also in practice sensors, not used in this Gz demo


        sl.create_gz_bridge(bridges)


        # run sliders for manual actuation
        sl.node('slider_publisher', arguments = sl.find('boating_gz', 'manual.yaml'))



    return sl.launch_description()
