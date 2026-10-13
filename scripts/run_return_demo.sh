# Run inside the robot's existing SSH shell:
#   source ~/run_return_demo.sh
# Settings live in ~/longer_return.yaml. Do not start another shell.
# A subshell keeps environment changes and failures out of the login shell.
(
    demo_config="$HOME/longer_return.yaml"
    if [ ! -f "$demo_config" ]; then
        echo "Missing settings: $demo_config"
        exit 1
    fi

    source /home/ubuntu/ros2_ws/.robotrc || exit 1
    source /home/ubuntu/ros2_ws/install/local_setup.zsh || exit 1
    export MACHINE_TYPE=JetRover_Mecanum

    sudo systemctl stop start_app_node.service button_scan.service || exit 1

    # Refuse to launch a second set of drivers after an SSH reconnect.
    demo_nodes=$(timeout 15s ros2 node list) || {
        echo "Could not check running nodes. Demo was not started."
        exit 1
    }
    if printf '%s\n' "$demo_nodes" | grep -Eq '^/(retrieve_object|odom_publisher|ros_robot_controller|controller_manager)$'; then
        echo "A mission or controller is already running."
        echo "Press Ctrl+C in its terminal before running this launcher again."
        exit 1
    fi

    echo "Using settings from $demo_config:"
    cat "$demo_config"
    echo "Demo starts automatically when ready. Ctrl+C stops the launch."
    ros2 launch jetrover_retrieve retrieve_object.launch.py \
        mission_mode:=return_test \
        config_file:="$demo_config" \
        auto_start:=true
)
