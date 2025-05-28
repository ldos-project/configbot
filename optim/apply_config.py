import argparse
import json
import os
import re
import time

from resource_manager import ROSAdaptorManager, CgroupManager
from monitor import Monitor
from utils import setup_env, move_cgroup_to_cgroup

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str)
    parser.add_argument('number', type=int)
    parser.add_argument('name', type=str)
    parser.add_argument('--all_topics', nargs="+", default=["/navigation/cmd_vel", "/camera/rgb/image_raw/compressed", "/gsam_output"], type=str)
    parser.add_argument('--unify_navs', action='store_const', const=True, default=False)
    parser.add_argument('--default', action='store_const', const=True, default=False)
    args = parser.parse_args()
    args.num_cpus = setup_env()

    print(f"[INFO] [INFO] [INFO] You are running on {args.num_cpus} cpus.")
    print(f"[INFO] [INFO] [INFO] Confirm this is what you intended.")
    input()
    
    args.result_file = os.path.join(os.getcwd(), "./results/profiling/", f"{args.name}_{args.num_cpus}cpus.log")
    assert not os.path.exists(args.result_file), f"Did you already run {args.result_file}?"
    
    result_dict = dict()
    
    if not args.default:
        # read the config
        args.file_path = os.path.join(os.getcwd(), "./results/optimization/", args.filename)
        assert os.path.exists(args.file_path), f"Couldn't find {args.file_path} :'-("
        
        with open(args.file_path, 'r') as f:
            config_raw = f.readlines()[args.number]
        
        # check if there are adaptors or cgroups or both in config
        assert "cpu_" in config_raw
        args.cgroups = True
        args.adaptors = "adaptor_" in config_raw
        config = json.loads(config_raw)
        print(f"You chose line={args.number}, which has target={config['target']} and constraint={config['constraint']}")
        
        # setup dict
        result_dict['target'] = config['target']
        result_dict['constraint'] = config['constraint']
        result_dict['config'] = config['params']
        
        del config['target'], config['constraint']    
        args.pbounds = dict()
        
        total_updated = 0
        if args.cgroups:
            args.cgroup_manager = CgroupManager(n_cpu=args.num_cpus, use_essential=True, robot='spot')
            args.pbounds.update(args.cgroup_manager.get_pbounds())
            total_updated += args.cgroup_manager.set_new_config(result_dict['config'])

        if args.unify_navs:
            assert args.cgroups
            nav_nodes = ["/navigation", "/sterling", "/bev_node"]
            print("[apply_config] UNIFYING NAVs")
            total_allocated_cpus = 0
            for _tmp_node in nav_nodes:
                _serialized_tmp_node = args.cgroup_manager.serialize_node(_tmp_node)
                if _serialized_tmp_node in result_dict['config']:
                    print(f"[apply_config] Found cpu.max={result_dict['config'][_serialized_tmp_node]}")
                    total_allocated_cpus += result_dict['config'][_serialized_tmp_node]
                else:
                    print(f"[apply_config] NOT FOUND: value for {_tmp_node} in current config. Skipping")
            
            nav_serialized = args.cgroup_manager.serialize_node("/navigation")
            nav_node_cgroup_path = os.path.join(args.cgroup_manager.configbot_cgroup_path, nav_serialized)
            print(f"Navigation command cgroup location: {nav_node_cgroup_path}")
            assert os.path.exists(nav_node_cgroup_path)
            
            print(f"[apply_config] Total CPUs allocated across {nav_nodes}: {total_allocated_cpus}")
            print(f"[apply_config] Changing NAV node allocation...")
            assert args.cgroup_manager.set_cgroup_limit(nav_serialized, total_allocated_cpus), f"Couldn't set modified total limit"
            print(f"[apply_config] Success!")            

            for _tmp_node in nav_nodes:
                if _tmp_node != "/navigation":
                    print(f"Moving all processes in {_tmp_node} to navigation cgroup")
                    _serialized_tmp_node = args.cgroup_manager.serialize_node(_tmp_node)
                    _node_cgroup_path = os.path.join(args.cgroup_manager.configbot_cgroup_path, _serialized_tmp_node)
                    move_cgroup_to_cgroup(_node_cgroup_path, nav_node_cgroup_path)

        args.adaptor_manager = ROSAdaptorManager(use_essential=True, robot='spot', strict_mode=False)
        if args.adaptors:
            args.pbounds.update(args.adaptor_manager.get_pbounds())
            total_updated += args.adaptor_manager.set_new_config(result_dict['config'])
            
        if len(result_dict['config']) != total_updated: 
            print(f"[WARN] Possible error. Only {total_updated} / {len(result_dict['config'])} updated.")
    
    args.monitor = Monitor(topic_list=args.all_topics)

    args.adaptor_manager.clear_adaptors()
    print("Set all adaptors to zero.")
    time.sleep(5)
    print("Disabling adaptors.")
    args.adaptor_manager.disable_adaptors()

    if args.adaptors:
        args.adaptor_manager.set_new_config(result_dict['config'])

    args.monitor.start_monitor()
    print("[apply_config] Monitoring started -- will sleep for 60s")
    time.sleep(60)
    profiling_results = args.monitor.end_monitor()

    result_dict['results'] = profiling_results
    with open(args.result_file, 'w') as f:
        json.dump(result_dict, f)

if __name__ == "__main__":
    main()