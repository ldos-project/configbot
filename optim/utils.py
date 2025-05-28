import math
import os
import subprocess

def move_cgroup_to_cgroup(cgroup_src, cgroup_dst):
    cgroup_src_procs = os.path.join(cgroup_src, "cgroup.procs")
    
    if not os.path.exists(cgroup_src_procs):
        print(f"[utils] Source cgroup.procs does not exist: {cgroup_src_procs}")
        return

    if not os.path.exists(cgroup_dst):
        print(f"[utils] Destination cgroup.procs does not exist: {cgroup_dst}")
    
    # Read all PIDs from the source cgroup
    with open(cgroup_src_procs, 'r') as f:
        pids = [int(pid) for pid in f.read().split()]
    
    # Move each PID to the destination cgroup
    for pid in pids:
        move_pid_to_cgroup(pid, cgroup_dst)

def move_pid_to_cgroup(pid, cgroup_path, verbose=False):
    # Construct the path to the cgroup's "cgroup.procs" file
    cgroup_procs_file = os.path.join(cgroup_path, "cgroup.procs")
    
    # Write the PID to the cgroup's "cgroup.procs" file
    with open(cgroup_procs_file, 'w') as f:
        f.write(str(pid))
    
    if(verbose):
        print(f"Process {pid} moved to cgroup: {cgroup_path}")

def change_cpumax_limit(cgroup, limit):
    if isinstance(limit, str):
        assert limit == "max"
        cmd_string = f"max 100000"
    else:
        cmd_string = f"{int(math.ceil(limit * 100000))} {100000}"
    
    f = open(os.path.join(cgroup, "cpu.max"), 'w')
    f.write(cmd_string)
    f.close()

def move_self_to_cgroup(cgroup_path):
    pid = os.getpid()
    move_pid_to_cgroup(pid, cgroup_path, verbose=True)

def setup_env():
    nproc = int(subprocess.check_output(["nproc"]).strip())
    move_self_to_cgroup("/sys/fs/cgroup/system.slice/agent/")
    return nproc