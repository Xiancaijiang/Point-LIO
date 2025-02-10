import os  # 导入操作系统模块  

from ament_index_python.packages import get_package_share_directory  # 导入获取包共享目录的函数  
from launch import LaunchDescription  # 导入LaunchDescription类，用于描述启动文件  
from launch.actions import DeclareLaunchArgument  # 导入声明启动参数的类  
from launch.conditions import IfCondition  # 导入条件类，用于控制节点的启动  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 导入替换类，用于处理启动配置  
from launch_ros.actions import Node  # 导入Node类，用于定义ROS节点  

def generate_launch_description():  
    # 将完全限定名称映射到相对名称，以便可以在节点前加上命名空间。  
    # 关于tf的情况，目前似乎没有更好的替代方案  
    # https://github.com/ros/geometry2/issues/32  
    # https://github.com/ros/robot_state_publisher/pull/30  
    # TODO(orduno) 用`PushNodeRemapping`替代  
    #              https://github.com/ros2/launch_ros/issues/56  
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]  # 定义tf和tf_static的重映射  

    namespace = LaunchConfiguration("namespace")  # 定义命名空间的启动配置  
    use_rviz = LaunchConfiguration("rviz")  # 定义是否启动RViz的启动配置  
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")  # 定义Point-LIO配置文件路径的启动配置  

    point_lio_dir = get_package_share_directory("point_lio")  # 获取point_lio包的共享目录  

    # 声明命名空间的启动参数  
    declare_namespace = DeclareLaunchArgument(  
        "namespace",  
        default_value="",  
        description="Namespace for the node",  # 参数描述  
    )  

    # 声明是否启动RViz的启动参数  
    declare_rviz = DeclareLaunchArgument(  
        "rviz", default_value="True", description="Flag to launch RViz."  # 参数描述  
    )  

    # 声明Point-LIO配置文件路径的启动参数  
    declare_point_lio_cfg_dir = DeclareLaunchArgument(  
        "point_lio_cfg_dir",  
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),  # 默认配置文件路径  
        description="Path to the Point-LIO config file",  # 参数描述  
    )  

    # 启动Point-LIO节点  
    start_point_lio_node = Node(  
        package="point_lio",  # 指定包名  
        executable="pointlio_mapping",  # 指定可执行文件  
        namespace=namespace,  # 使用命名空间  
        parameters=[point_lio_cfg_dir],  # 使用配置文件路径参数,即mid360.yaml
        remappings=remappings,  # 使用tf重映射  
        output="screen",  # 输出到屏幕  
    )  

    # 启动RViz节点，条件是use_rviz为True  
    start_rviz_node = Node(  
        condition=IfCondition(use_rviz),  # 如果use_rviz为True，则启动  
        package="rviz2",  # 指定包名  
        executable="rviz2",  # 指定可执行文件  
        namespace=namespace,  # 使用命名空间
        name="rviz",  # 节点名称  
        remappings=remappings,  # 使用tf重映射，即将/tf映射到/tf
        arguments=[  
            "-d",  
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),  # 指定RViz配置文件路径，即"loam_livox.rviz"
            ".rviz",  
        ],  
    )  

    ld = LaunchDescription()  # 创建LaunchDescription对象  

    # 添加各个启动参数和节点到LaunchDescription中  
    ld.add_action(declare_namespace)  # 添加命名空间参数  
    ld.add_action(declare_rviz)  # 添加RViz参数  
    ld.add_action(declare_point_lio_cfg_dir)  # 添加Point-LIO配置文件路径参数  
    ld.add_action(start_point_lio_node)  # 添加Point-LIO节点  
    ld.add_action(start_rviz_node)  # 添加RViz节点  

    return ld  # 返回LaunchDescription对象