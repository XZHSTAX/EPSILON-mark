import rosbag
from rich.console import Console
from rich.table import Table


bag_file = '/home/xzh/ros1/BPws/my_bag_debug1.bag'
bag_data = rosbag.Bag(bag_file, "r")

info = bag_data.get_type_and_topic_info()
print(info)

topic_name1 = "/record/agent_0/ego_behavior_vis"
topic_name2 = "/record/agent_0/ego_vehicle_status"
topic_name3 = "/record/agent_0/forward_trajs"
topic_name4 = "/vis/agent_0/local_lanes_vis"

topic_name6 = "/record/ssc/exec_traj"

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


ssc_traj = []
perception_data = bag_data.read_messages(topic_name6)
for topic, msg, t in perception_data:
    current_time_ssc_trajs = [] # 0 x,1 y,2 angle,3 velocity,4 acceleration,5 curvature,6 steer
    if msg is not None:
        for state in msg.StateSet:
            current_time_ssc_trajs.append([state.vec_position.x,state.vec_position.y,state.angle,state.velocity,state.acceleration,state.curvature,state.steer])
        ssc_traj.append(current_time_ssc_trajs)
            

PP_ctrl_signal = []
perception_data = bag_data.read_messages("/PP_ctrl_signal")
for topic, msg, t in perception_data:
    if msg is not None:
        PP_ctrl_signal.append([msg.vec_position.x,msg.vec_position.y,msg.velocity,msg.steer])


bag_data.close()


# 创建一个控制台对象
console = Console()

# 创建一个表格对象
table = Table(title="conclusion 1")

# 添加列名
# style: 设置该列的样式 no_wrap: 设置该列内容不换行 justify: 设置该列内容对齐方式
table.add_column("list name", style="cyan", no_wrap=True)
table.add_column("len", justify="right")

# 添加行数据
table.add_row("behavior_record", str(len(behavior_record)))
table.add_row("ego_vehicle_position", str(len(ego_vehicle_position)))
table.add_row("forward_trajs_record", str(len(forward_trajs_record)))

table.add_row("Local_lanes_record", str(len(Local_lanes_record)))
table.add_row("ssc_traj", str(len(ssc_traj)))
table.add_row("PP_ctrl_signal", str(len(PP_ctrl_signal)))


console.print(table)