import base64
import numpy as np
import os
import psutil
import signal
import sys
import subprocess
import time

import rosnode
import rospy 

from utils import move_pid_to_cgroup, change_cpumax_limit
from rospy.srv import AdaptorService
from optim_params import ADAPTOR_BLACKLIST, ESSENTIAL_NODES

CONFIGBOT_CGROUP_PATH = "/sys/fs/cgroup/system.slice/configbot_cgroup/"
assert os.path.exists(CONFIGBOT_CGROUP_PATH), "Please setup requisite cgroups"

class CgroupManager():
    def __init__(self, n_cpu=1, min_cpu=0.01, use_essential=False, robot=None):
        super().__init__() 
        # get a list of all ROS nodes
        self.min_cpu = min_cpu
        self.n_cpu = n_cpu
        self.use_essential = use_essential
        self.robot = robot

        self.all_nodes = rosnode.get_node_names()
        
        self.essential_nodes = ESSENTIAL_NODES[self.robot]
        missing_essential_nodes = set(self.essential_nodes) - set(self.all_nodes)
        
        if len(missing_essential_nodes) != 0:
            print(f"Some essential nodes seem to have not been launched: {missing_essential_nodes}")
            assert ('/sterling' in missing_essential_nodes) == ('/bev_node' in missing_essential_nodes)
            assert len(missing_essential_nodes) <= 2
        
        self.non_essential_nodes = list(set(self.all_nodes) - set(self.essential_nodes))
        print(f"Identified non-essential nodes: {self.non_essential_nodes}")
        
        self.configbot_cgroup_path = CONFIGBOT_CGROUP_PATH
        # for every ROS node, create a new cgroup 
        self.bounds = dict()
        for node in self.all_nodes:
            # create cgroup            
            node_cgroup_path = os.path.join(self.configbot_cgroup_path, self.serialize_node(node))
            os.makedirs(node_cgroup_path, exist_ok=True)
            
            # add this to bounds
            if (node in self.non_essential_nodes) or (self.use_essential):
                self.bounds[node] = (self.min_cpu, 1.0 * self.n_cpu)
                assert self.deserialize_node(self.serialize_node(node)) == node
            
            # identify PID of process and children PIDs
            node_info = subprocess.check_output(["rosnode", "info", node], text=True)
            pid_line = [line for line in node_info.splitlines() if "Pid" in line]
            pid = int(pid_line[0].split(":")[1].strip()) if pid_line else None
            assert pid is not None
            children = psutil.Process(pid).children(recursive=True)

            # move ROS node (and children) to cgroup
            pids_to_move = [pid] + [child.pid for child in children]
            for _pid in pids_to_move:
                move_pid_to_cgroup(_pid, node_cgroup_path)

        print(f"Identified {len(self.bounds)} apps.")
        print(self.bounds)
        self.cleanup()
    
    def serialize_node(self, nodename):
        return "cpu_" + base64.b16encode(bytes(nodename, 'utf-8')).decode('utf-8')

    def deserialize_node(self, nodename):
        return base64.b16decode(bytes(nodename.replace('cpu_', ''), 'utf-8')).decode('utf-8')

    def get_bounds(self):
        return self.bounds

    def get_pbounds(self):
        op = {}
        for key in self.bounds:
            op[self.serialize_node(key)] = self.bounds[key]
        return op

    def set_new_config(self, spec):
        # identify which configs are relevant to us
        filtered_spec = {key: value for key, value in spec.items() if key.startswith('cpu_')}
        total_changed = 0
        for appname, max_limit in filtered_spec.items():
            total_changed += self.set_cgroup_limit(appname, max_limit)
            
        return total_changed
    
    def set_cgroup_limit(self, appname, max_limit):
        if self.deserialize_node(appname) in self.bounds.keys(): 
            cgroup = os.path.join(self.configbot_cgroup_path, appname)
            change_cpumax_limit(cgroup, max_limit)
            return 1
        else:
            print(f"[cgroup_manager::WARN] {self.deserialize_node(appname)} not being set.")
            return 0

    def cleanup(self, *args):
        for appname in self.get_pbounds().keys():
            cgroup = os.path.join(self.configbot_cgroup_path, appname)
            change_cpumax_limit(cgroup, "max")

class ROSAdaptorManager():
    def __init__(self, use_essential=True, robot=None, strict_mode=True):
        self.use_essential = use_essential
        self.robot = robot
        self.essential_nodes = ESSENTIAL_NODES[self.robot]
        self.all_nodes = rosnode.get_node_names()
        self.non_essential_nodes = list(set(self.all_nodes) - set(self.essential_nodes))
        
        # check if we are using the custom ROS image
        master = rospy.get_master()
        _, _, services = master.getSystemState()[2]
        self.active_adaptor_endpoints = list(filter(lambda x: x.startswith("/adaptor_node/"), list(map(lambda x: x[0], services))))
        assert len(self.active_adaptor_endpoints) != 0, f"no /adaptor_node/ services found - are you sure you are using cusotm ROS?"
        
        self.essential_adaptors = []
        self.nonessential_adaptors = []
        self.all_adaptors = []
        self.used_adaptors = {}
        self.adaptor_blacklist = ADAPTOR_BLACKLIST

        # fail fast if anything is fishy
        for _adaptor in self.active_adaptor_endpoints:
            _blacklisted = sum(list(map(lambda x: x in _adaptor, self.adaptor_blacklist)))
            
            if not _blacklisted or not strict_mode:
                print("Whitelisted: ", _adaptor)
                assert "adaptor_sub" in _adaptor
                _node, _topic = _adaptor.split('/adaptor_sub')
                _node = _node.replace('/adaptor_node', '')
                
                self.all_adaptors.append(_adaptor)
                if _node in self.essential_nodes:
                    self.essential_adaptors.append(_adaptor)
                else:
                    assert _node in self.non_essential_nodes, f"Category of {_node} is UNK."
                    self.nonessential_adaptors.append(_adaptor)
            else:
                print("Blacklisted: ", _adaptor)
            
        print(f"Identified {len(self.essential_adaptors)} essential and {len(self.nonessential_adaptors)} nonessential in this config")

        self.bounds = {}
        
        # first setup throttle nodes (by setting em up in bounds)
        for _adaptor in self.all_adaptors:
            if self.use_essential or (_adaptor in self.nonessential_adaptors):
                self.bounds[_adaptor] = (0.0, 0.5)
                rospy.wait_for_service(_adaptor)
                self.used_adaptors[_adaptor] = rospy.ServiceProxy(_adaptor, AdaptorService)
                assert self.deserialize_adaptor(self.serialize_adaptor(_adaptor)) == _adaptor
        
        print("Initialize all used_adaptors with high frequency.")
        for _adaptor in self.used_adaptors:
            self.adjust_adaptor(_adaptor, 1)
            
        print(f"DONE. Initialized {len(self.used_adaptors)} adaptors.")

    def clear_adaptors(self):
        for key in self.used_adaptors:
            self.used_adaptors[key](0.0)
    
    def disable_adaptors(self):
        for key in self.used_adaptors:
            self.used_adaptors[key](-1.0)
    
    ######### ADAPTOR CREATION STUFFF #########
    def adjust_adaptor(self, adaptor_name, max_freq):
        self.used_adaptors[adaptor_name](100.0 * max_freq)

    ######### APPLYING NEW CONFIG #########
    def set_new_config(self, spec):
        filtered_spec = {self.deserialize_adaptor(key): value for key, value in spec.items() if key.startswith('adaptor_')}
        total_updated = 0

        for adaptor_name, max_freq in filtered_spec.items():
            if adaptor_name in self.used_adaptors.keys():
                self.adjust_adaptor(adaptor_name, max_freq)
                total_updated += 1
            else:
                print(f"[ros_adaptor_manager] [WARN] Not adjusting: {adaptor_name} since it doesn't exist")
        return total_updated
    
    ######### BOUNDS #########
    def get_bounds(self):
        return self.bounds

    def get_pbounds(self):
        op = {}
        for key in self.bounds:
            op[self.serialize_adaptor(key)] = self.bounds[key]
        return op    

    ######### UTILITY #########
    def serialize_adaptor(self, adap):
        return "adaptor_" + base64.b16encode(bytes(adap, 'utf-8')).decode('utf-8')

    def deserialize_adaptor(self, adap):
        adap = adap.replace('adaptor_', '')
        return base64.b16decode(bytes(adap, 'utf-8')).decode('utf-8')