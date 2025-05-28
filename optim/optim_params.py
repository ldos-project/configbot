SUPPORTED_STACKS = ['dummy', 'spot']

# list of essential nodes
ESSENTIAL_NODES = {
    'dummy': [
        'lidar_processor_node', 'fake_lidar_node', 'nav_node'
    ],
    'spot': [
        '/bev_node', '/enml', '/joystick', '/navigation', '/pointcloud_to_laserscan_highbeams', 
        '/rosout', '/spot/robofleet_client', '/spot/spot_ros', '/status_translator', '/sterling', 
        '/velodyne_nodelet_manager', '/velodyne_nodelet_manager_driver', '/velodyne_nodelet_manager_laserscan', 
        '/velodyne_nodelet_manager_transform', '/velodyne_tf_broadcaster', '/websocket'
    ]
}

# topics on which we do not place adaptors (for whatever reason)
# for the dummy stack, we are allowing adaptors on everything, but typically this list should
# inclide topics that: (i) have very infrequent messages, or (ii) where data loss is unacceptable.
ADAPTOR_BLACKLIST = {
    'dummy': [],
    'spot': [
        "topic_statistics", "spot/spot_ros", "set_", "velodyne", "joystick", "odometry", "goal", "status_translator", 
        "spot/camera", "enml/adaptor_sub/odom", "/adaptor_node/navigation/adaptor_sub/localization", "subscriptions", 
        "halt_robot"
    ]
}

# our approach will try to maximize the min( freq(OBJECTIVE_TOPICS)/relative_weights(OBJECTIVE_TOPICS) )
# we set them all to equal weights. but you could choose to do something different. For spot, we are trying
# to maximize the frequency at which optinal camera based tasks (e.g. object detection, image logging, etc)
# are performed. For the dummy stack, we are seeking to maximize the frequency of our object detection node. 
OBJECTIVE_TOPICS = {
    "dummy": {
        "topics": ["/detections"],
        "weights": [1]
    },
    "spot": {
        "topics": [
            '/camera/rgb/image_raw/compressed', '/gsam_output', 
            '/spot/camera/frontleft/image', '/spot/camera/frontright/image',
            '/spot/camera/left/image', 'spot/camera/right/image',
            'spot/camera/back/image'
        ],
        "weights": [
            1, 1,  
            1, 1, 
            1, 1, 
            1
        ]
    }
}

# we wanted our command velocity topic to always be > 35
CONSTRAINT_TOPIC = {
    "dummy": {
        "topic": '/nav/decision',
        'value': 50
    },
    "spot": {
        "topic": '/navigation/cmd_vel',
        "value": 35.00
    }
}

for stack in SUPPORTED_STACKS:
    assert stack in ESSENTIAL_NODES, f"ESSENTIAL_NODES missing for {robot}"
    assert stack in ADAPTOR_BLACKLIST, f"ADAPTOR_BLACKLIST missing for {robot}"
    assert stack in OBJECTIVE_TOPICS, f"OBJECTIVE_TOPICS missing for {robot}"
    assert stack in CONSTRAINT_TOPIC, f"CONSTRAINT_TOPIC missing for {robot}"
