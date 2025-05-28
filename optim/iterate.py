import argparse
import json
import numpy as np
import os
import subprocess
import time

from utils import setup_env
from resource_manager import ROSAdaptorManager, CgroupManager
from monitor import Monitor
from optim_params import SUPPORTED_STACKS

from bayes_opt import BayesianOptimization
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events

from scipy.optimize import NonlinearConstraint

class OptimizationManager():
    def __init__(self, args):
        self.args = args
        assert self.args.cgroup or self.args.adaptor, f"Atleast one of adaptor/cgroup must be selected"
        self.n_cpus = setup_env()
        
        self.name = f"{self.args.name}_{self.n_cpus}cpus"
        if self.args.cgroup: 
            self.name = f"{self.name}_cgroup"
        
        if self.args.adaptor:
            self.name = f"{self.name}_adaptor"

        self.pbounds = dict()
        
        if self.args.cgroup:
            self.cgroup_manager = CgroupManager(n_cpu=self.n_cpus, use_essential=self.args.use_essential, robot=self.args.stack)
            self.pbounds.update(self.cgroup_manager.get_pbounds())
        
        if self.args.adaptor:
            self.adaptor_manager = ROSAdaptorManager(use_essential=self.args.use_essential, robot=self.args.stack)
            self.pbounds.update(self.adaptor_manager.get_pbounds())

        self.nonlinear_constraint = NonlinearConstraint(self.constraint_fn, CONSTRAINT_TOPIC[self.args.stack]['value'], np.inf)

        # set topic names
        self.objective_topics = OBJECTIVE_TOPICS[self.args.stack]
        self.constraint_topic = CONSTRAINT_TOPIC[self.args.stack]['topic']

        self.raw_log_file_path = os.path.join(os.getcwd(), "./results/optimization/", f"{self.name}-raw.log")
        assert not os.path.exists(self.raw_log_file_path)
        
        self.raw_log_file = open(self.raw_log_file_path, "w")
        self.monitor = Monitor(topic_list=[*self.objective_topics, self.constraint_topic])

    def observation(self, **kwargs):
        # set new config
        total_updated = 0
        
        if self.args.adaptor:
            total_updated += self.adaptor_manager.set_new_config(kwargs)
        if self.args.cgroup:
            total_updated += self.cgroup_manager.set_new_config(kwargs)
        
        assert len(kwargs) == total_updated, f"Fatal error. Only {total_updated} / {len(config)} updated."
        time.sleep(5) # just to let the system settle in

        # observe what happens
        self.monitor.start_monitor()
        time.sleep(5) # measurement window
        monitoring_data = self.monitor.end_monitor()
        self.raw_log_file.write(json.dumps(monitoring_data) + "\n")
        freqs = self.get_processed_monitoring(monitoring_data)

        self.constraint_value = freqs[self.constraint_topic]
        self.constraint_kwargs = kwargs
        config = dict()
        
        for key in kwargs.keys():
            if key.startswith('cpu'):
                config[self.cgroup_manager.deserialize_node(key)] = kwargs[key]
            else:
                config[self.adaptor_manager.deserialize_adaptor(key)] = kwargs[key]
        
        # iterate over all topics in self.objective_topic
        # and then find min frequency and return that
        min_freq = min(freqs[topic] for topic in self.objective_topics)
        return min_freq
    
    def get_processed_monitoring(self, stats):
        result_data = {}
        for topic, timestamps in stats.items():
            intervals = [b - a for a, b in zip(timestamps[:-1], timestamps[1:])]
            if len(intervals) > 0:
                avg_interval = sum(intervals) / len(intervals)
                result_data[topic] = 1 / avg_interval if avg_interval > 0 else 0
            else:
                result_data[topic] = 0
        return result_data

    def constraint_fn(self, **kwargs):
        assert kwargs == self.constraint_kwargs and self.constraint_value is not None
        return self.constraint_value

    def __del__(self):
        if hasattr(self, "raw_log_fille"):
            self.raw_log_file.close()

def main():
    parser = argparse.ArgumentParser(description="Run bayesian optimization.")
    
    parser.add_argument('--name', default=f"{time.time()}", help="What to name the output file.")
    parser.add_argument('--stack', default="dummy", type=str, help="What stack are you trying to optimize?")
    parser.add_argument('--randomness', type=int, default=1, help="Seed for randomness.")
    
    # Mutually exclusive group for --adaptor and --noadaptor
    adaptor_group = parser.add_mutually_exclusive_group()
    adaptor_group.add_argument("--use-essential", action="store_true", dest="use_essential", default=False, help="Allow agent to constrain essential apps as well.")
    adaptor_group.add_argument("--no-use-essential", action="store_false", dest="use_essential", help="Disallow agent to constrain essential apps as well.")

    # Mutually exclusive group for --adaptor and --noadaptor
    adaptor_group = parser.add_mutually_exclusive_group()
    adaptor_group.add_argument("--adaptor", action="store_true", default=False, help="Enable adaptor knobs (default: False).")
    adaptor_group.add_argument("--noadaptor", action="store_false", dest="adaptor", help="Disable adaptor knobs.")

    # Mutually exclusive group for --cpumax and --nocpumax
    cpumax_group = parser.add_mutually_exclusive_group()
    cpumax_group.add_argument("--cgroup", action="store_true", default=True, help="Enable cpumax knobs (default: True).")
    cpumax_group.add_argument("--nocgroup", action="store_false", dest="cgroup", help="Disable cpumax knobs.")
    args = parser.parse_args()

    assert args.stack in SUPPORTED_STACKS, f"Stack {args.stack} is not supported. Supported robots: {SUPPORTED_STACKS}"
    
    manager = OptimizationManager(args)
    
    optimizer = BayesianOptimization(
        f=manager.observation,
        constraint=manager.nonlinear_constraint,
        pbounds=manager.pbounds,
        verbose=2,
        random_state=args.randomness,
        allow_duplicate_points=True
    )
    
    logger = JSONLogger(path=os.path.join(os.getcwd(), f"./results/optimization/{manager.name}.log"))
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    optimizer.maximize(
        init_points=5,
        n_iter=3
    )

if __name__ == "__main__":
    main()