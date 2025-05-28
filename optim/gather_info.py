import os
import argparse
import json
import rospy

# Set up argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("env_name", type=str, help="Name of the environment")
args = parser.parse_args()

env_name = args.env_name
save_path = f"results/envs/{env_name}.json"

# Ensure the save directory exists
os.makedirs(os.path.dirname(save_path), exist_ok=True)

# Check if the file already exists before proceeding
if not os.path.exists(save_path):
    master = rospy.get_master()
    pubs, subs, _ = master.getSystemState()[2]

    ros_nodes = {}
    
    for topicinfo in pubs:
        name, publishers = topicinfo
        for pub in publishers:
            print(f"Node {pub} publishes to {name}")
            if pub not in ros_nodes:
                ros_nodes[pub] = {"publishes_to": [], "subscribes_to": []}
            ros_nodes[pub]["publishes_to"].append(name)
    del publishers, pub

    for topicinfo in subs:
        name, subscribers = topicinfo
        for sub in subscribers:
            if sub not in ros_nodes:
                ros_nodes[sub] = {"publishes_to": [], "subscribes_to": []}
            ros_nodes[sub]["subscribes_to"].append(name)
    
    with open(save_path, "w") as f:
        json.dump(ros_nodes, f, indent=4)

    print(f"File saved at {save_path}")
else:
    print(f"File already exists at {save_path}, skipping save.")