from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    # launch world
    sl.gz_launch(sl.declare_arg('world_file', sl.find('boating_gz', 'sydney_regatta.sdf')),
                 sl.declare_arg('gz_args', ''))

    # run bridges for clock and wind / water current
    bridges = [GazeboBridge.clock()]

    for gz, ros in (('/ocean_current', '/current'),
                    ('/model/wind/ocean_current', '/wind')):
        bridges.append((gz, ros, 'geometry_msgs/Vector3', GazeboBridge.ros2gz))

    sl.create_gz_bridge(bridges)

    # run current / wind slider
    sl.node('slider_publisher',
            arguments = sl.find('boating_gz', 'current.yaml'))
    return sl.launch_description()

