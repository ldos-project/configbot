import rospy
from std_srvs.srv import Empty, EmptyResponse
from configbot.srv import StatsService, StatsServiceRequest, StatsServiceResponse
import argparse
from collections import defaultdict
import time
import json

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('topics', nargs='+', help='List of topics to subscribe to')
    args, _ = parser.parse_known_args()
    return args.topics

class TopicStatistics:
    def __init__(self, topics):
        self.topics = topics
        self.stats = {topic: [] for topic in topics}
        self.subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, self.callback, callback_args=topic) for topic in topics]
        self.flush_service = rospy.Service('flush', Empty, self.flush)
        self.results_service = rospy.Service('results', StatsService, self.get_results)

    def callback(self, msg, topic):
        self.stats[topic].append(time.time())

    def flush(self, req):
        print("[topic_stats_node] received a flush req")
        self.stats = {topic: [] for topic in self.topics}
        return EmptyResponse()

    def get_results(self, req : StatsServiceRequest):
        print(f"[topic_stats_node] received get results request: {req.request_type}")
        return StatsServiceResponse(response_message = json.dumps(self.stats))

if __name__ == '__main__':
    rospy.init_node('topic_statistics')
    topics = parse_args()
    TopicStatistics(topics)
    rospy.spin()
