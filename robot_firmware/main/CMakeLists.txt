idf_component_register(
    SRCS
        "main.cpp"
        "common/robot_state.cpp"
        "drivers/drv_mpu6500.cpp"
        "drivers/drv_vl53l0x.cpp"
        "hal/hal_motor.cpp"
        "hal/hal_imu.cpp"
        "hal/hal_tof.cpp"
        "hal/hal_battery.cpp"
        "services/pid.cpp"
        "services/kinematic.cpp"
        "services/odometry.cpp"
        "services/motor_controller.cpp"
        "tasks/task_motor.cpp"
        "tasks/task_imu.cpp"
        "tasks/task_tof.cpp"
        "tasks/task_battery.cpp"

        # Comms (uncomment when ready)
        # "comms/comms.cpp"
        # "comms/publishers.cpp"
        # "comms/subscribers.cpp"
        # "tasks/task_comms.cpp"

    INCLUDE_DIRS
        "."
        "config"
        "common"
        "drivers"
        "hal"
        "services"
        "comms"
        "tasks"
)