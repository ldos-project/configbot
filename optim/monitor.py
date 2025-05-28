# basic imports
import json
import os
import rospy
import signal
import sys
import subprocess

# services
from configbot.srv import StatsService
from std_srvs.srv import Empty

from utils import move_pid_to_cgroup

class Monitor():
    def __init__(self, topic_list=['cmd_vel']):
        self.topic_list = topic_list        
        
        # launch the monitoring process here
        cmd=". /root/configbot/optim/devel/setup.sh; cd /root/configbot/optim/src/configbot; python3 scripts/topic_stats_node.py"
        for topic in topic_list:
            cmd += f" {topic}"
        print(f"[monitor] running: {cmd}")
        
        self.proc = subprocess.Popen(cmd, shell=True, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        move_pid_to_cgroup(self.proc.pid, "/sys/fs/cgroup/system.slice/agent/") # move the monitor node to the same cgroup as the agent
        
        # wait for services to be up
        try:
            rospy.wait_for_service('flush', timeout=2)
            rospy.wait_for_service('results', timeout=2)
        except rospy.exceptions.ROSException:
            self.check_health()

        self.flush_function = rospy.ServiceProxy('flush', Empty)
        self.result_function = rospy.ServiceProxy('results', StatsService)

    def check_health(self):
        if self.proc.poll() is not None:
            print(f"topic_stats_node exited unexpectedly")
            stdout, stderr = self.proc.communicate()
            print(f"[stdout] {stdout}")
            print(f"[stderr] {stderr}")
            sys.exit(-1)
        
    def start_monitor(self):
        self.check_health()
        self.flush_function()

    def end_monitor(self):
        self.check_health()
        resp = self.result_function(1).response_message
        return json.loads(resp)

    def __del__(self):
        if self.proc and self.proc.poll() is None:  # Check if still running
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait()