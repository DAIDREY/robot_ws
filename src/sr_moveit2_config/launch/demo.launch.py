from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("seed_robot", package_name="sr_moveit2_config")
        .moveit_cpp(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)