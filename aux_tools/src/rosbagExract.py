import rosbag
bag_file = '/home/xzh/ros1/BPws/my_bag.bag'
bag_data = rosbag.Bag(bag_file, "r")

info = bag_data.get_type_and_topic_info()
print(info)

topic_name1 = "/record/agent_0/ego_behavior_vis"
topic_name2 = "/record/agent_0/ego_vehicle_status"
topic_name3 = "/record/agent_0/forward_trajs"
topic_name4 = "/vis/agent_0/local_lanes_vis"

# 记录每个时间步的behavior
behavior_record = []
behavior_record_time = []
perception_data = bag_data.read_messages(topic_name1)
for topic, msg, t in perception_data:
    if msg is not None:
        behavior_record.append(msg.Behavior)
        behavior_record_time.append(t)

# 记录每个时间步的车辆状态，包括位置和其他的一些信息
ego_vehicle_position = []
ego_vehicle_status = [] # 0 angle,1 curvature,2 velocity,3 acceleration,4 steer
perception_data = bag_data.read_messages(topic_name2)
for topic, msg, t in perception_data:
    if msg is not None:
        ego_vehicle_position.append([msg.state.vec_position.x,msg.state.vec_position.y])
        ego_vehicle_status.append([msg.state.angle,msg.state.curvature,msg.state.velocity,msg.state.acceleration,msg.state.steer])

# 记录每个时间步所模拟的所有轨迹
forward_trajs_record = [] # 记录所有时刻的所有轨迹
perception_data = bag_data.read_messages(topic_name3)
for topic, msg, t in perception_data:
    if msg is not None:
        current_time_trajs = [] # 记录该时刻的所有轨迹
        # 遍历该时刻的每一条轨迹
        for traj in msg.forward_trajs_record:
            tra_record = [] # 记录该条轨迹上的所有点
            # 遍历该轨迹上的每一个点
            for point in traj.forward_trajs:
                tra_record.append([point.x, point.y])
            current_time_trajs.append(tra_record)
        forward_trajs_record.append(current_time_trajs)

Local_lanes_record = []
perception_data = bag_data.read_messages(topic_name4)
for topic, msg, t in perception_data:
    if msg is not None:
        Local_lanes_record_current = []
        for lane in msg.markers:
            Local_lane = []
            for point in lane.points:
                Local_lane.append([point.x, point.y])
            Local_lanes_record_current.append(Local_lane)
        Local_lanes_record.append(Local_lanes_record_current)
        
bag_data.close()

