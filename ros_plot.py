import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import rclpy.serialization
import rosidl_runtime_py.utilities as rosidl
from geometry_msgs.msg import Wrench
from spacecraft_msgs.msg import StateSpaceVector
import numpy as np
import transforms3d as tf

# Path to your rosbag2 .db3 file
db_file = '/home/user/rosbag2_2025_04_19-22_16_42/rosbag2_2025_04_19-22_16_42_0.db3'

# Connect to the SQLite database
conn = sqlite3.connect(db_file)
cursor = conn.cursor()

# Find topic ID for /spacecraft/ThrusterBodyWrench
topic_name = '/spacecraft/ThrusterBodyWrench'
cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
result = cursor.fetchone()
if result is None:
    raise ValueError(f"Topic {topic_name} not found in the bag.")
topic_id = result[0]

# Prepare deserializer
msg_type = rosidl.get_message('geometry_msgs/msg/Wrench')

# Fetch messages for the topic
cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
rows = cursor.fetchall()

# Deserialize and store values
timestamps = []
force_x = []
force_y = []
force_z = []
torque_x = []
torque_y = []
torque_z = []

for ts, data in rows:
    msg = rclpy.serialization.deserialize_message(data, msg_type)
    timestamps.append(ts * 1e-9)  # Convert from nanoseconds to seconds
    force_x.append(msg.force.x)
    force_y.append(msg.force.y)
    force_z.append(msg.force.z)
    torque_x.append(msg.torque.x)
    torque_y.append(msg.torque.y)
    torque_z.append(msg.torque.z)

conn.close()


df_wrench = pd.DataFrame({
    'time_sec': timestamps,
    'force_x': force_x,
    'force_y': force_y,
    'force_z': force_z,
    'torque_x': torque_x,
    'torque_y': torque_y,
    'torque_z': torque_z,
})


# # Plot force
# plt.figure()
# plt.plot(df['time_sec'].to_numpy(), df['force_x'].to_numpy(), label='Force X')
# plt.plot(df['time_sec'].to_numpy(), df['force_y'].to_numpy(), label='Force Y')
# plt.plot(df['time_sec'].to_numpy(), df['force_z'].to_numpy(), label='Force Z')
# plt.xlabel("Time (s)")
# plt.ylabel("Force (N)")
# plt.legend()
# plt.title("Thruster Body Force")
# plt.grid()

# # Plot torque
# plt.figure()
# plt.plot(df['time_sec'].to_numpy(), df['torque_x'].to_numpy(), label='Torque X')
# plt.plot(df['time_sec'].to_numpy(), df['torque_y'].to_numpy(), label='Torque Y')
# plt.plot(df['time_sec'].to_numpy(), df['torque_z'].to_numpy(), label='Torque Z')
# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.legend()
# plt.title("Thruster Body Torque")
# plt.grid()


# Connect to the SQLite database
conn = sqlite3.connect(db_file)
cursor = conn.cursor()

# Get topic ID
topic_name = '/spacecraft/state_space_vector'
cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
result = cursor.fetchone()
if result is None:
    raise ValueError(f"Topic {topic_name} not found in the bag.")
topic_id = result[0]

# Prepare deserializer
msg_type = rosidl.get_message('spacecraft_msgs/msg/StateSpaceVector')

# Get messages
cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
rows = cursor.fetchall()

ows = cursor.fetchall()

# Prepare storage
timestamps = []
positions = []
orientations_quat = []
orientations_euler = []
lin_vels = []
ang_vels = []

for ts, data in rows:
    msg = rclpy.serialization.deserialize_message(data, msg_type)
    state = msg.state_vector.data

    if len(state) != 13:
        continue  # skip malformed data

    timestamps.append(ts * 1e-9)  # ns to seconds
    positions.append([float(s) for s in state[0:3]])
    orientation = [float(s) for s in state[3:7]]
    orientations_quat.append(orientation)
    lin_vels.append([float(s) for s in state[7:10]])
    ang_vels.append([float(s) for s in state[10:13]])

    quat_wxyz = np.array([
            orientation[3],
            orientation[0],
            orientation[1],
            orientation[2],
        ])

    euler = tf.euler.quat2euler(quat_wxyz)
    orientations_euler.append(euler)

conn.close()


# Sanitize the DataFrame
for col in df_wrench.columns:
    df_wrench[col] = pd.to_numeric(df_wrench[col], errors='coerce')
df_wrench.dropna(inplace=True)

# Convert to DataFrame
df_state = pd.DataFrame({
    'time': timestamps,
    'pos_x': [p[0] for p in positions],
    'pos_y': [p[1] for p in positions],
    'pos_z': [p[2] for p in positions],
    'orientation_euler_x': [np.rad2deg(e[0]) for e in orientations_euler],
    'orientation_euler_y': [np.rad2deg(e[1]) for e in orientations_euler],
    'orientation_euler_z': [np.rad2deg(e[2]) for e in orientations_euler], 
    'quat_x': [q[0] for q in orientations_quat],
    'quat_y': [q[1] for q in orientations_quat],
    'quat_z': [q[2] for q in orientations_quat],
    'quat_w': [q[3] for q in orientations_quat],
    'angvel_x': [a[0] for a in ang_vels],
    'angvel_y': [a[1] for a in ang_vels],
    'angvel_z': [a[2] for a in ang_vels],
})

# Sanitize the DataFrame
for col in df_state.columns:
    df_state[col] = pd.to_numeric(df_state[col], errors='coerce')
df_state.dropna(inplace=True)

# Plot position
plt.figure()
plt.plot(np.array(df_state['time']), np.array(df_state['pos_x']), label='x')
plt.plot(np.array(df_state['time']), np.array(df_state['pos_y']), label='y')
plt.plot(np.array(df_state['time']), np.array(df_state['pos_z']), label='z')
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Spacecraft Position")
plt.legend()
plt.grid()

# Plot angular velocity
plt.figure()
plt.plot(np.array(df_state['time']), np.array(df_state['angvel_x']), label='wx')
plt.plot(np.array(df_state['time']), np.array(df_state['angvel_y']), label='wy')
plt.plot(np.array(df_state['time']), np.array(df_state['angvel_z']), label='wz')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity")
plt.legend()
plt.grid()

# Plot angular velocity
plt.figure()
plt.plot(np.array(df_state['time']), np.array(df_state['quat_x']), label='quat_x')
plt.plot(np.array(df_state['time']), np.array(df_state['quat_y']), label='quat_y')
plt.plot(np.array(df_state['time']), np.array(df_state['quat_z']), label='quat_z')
plt.plot(np.array(df_state['time']), np.array(df_state['quat_w']), label='quat_w')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Angular Velocity")
plt.legend()
plt.grid()

# Plot orientation
fig, ax1 = plt.subplots()

# --- Primary y-axis: Orientation Euler angles ---
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Euler angles (rad)")
ax1.plot(df_state['time'].to_numpy(), df_state['orientation_euler_x'].to_numpy(), label='Euler x', color='tab:blue')
ax1.plot(df_state['time'].to_numpy(), df_state['orientation_euler_y'].to_numpy(), label='Euler y', color='tab:orange')
ax1.plot(df_state['time'].to_numpy(), df_state['orientation_euler_z'].to_numpy(), label='Euler z', color='tab:green')
ax1.tick_params(axis='y')
ax1.legend(loc='upper left')

# --- Secondary y-axis: Torque ---
ax2 = ax1.twinx()  # Create second y-axis
ax2.set_ylabel("Torque (Nm)")
ax2.plot(df_wrench['time_sec'].to_numpy(), df_wrench['torque_x'].to_numpy(), label='Torque X', linestyle='--', color='tab:red')
ax2.plot(df_wrench['time_sec'].to_numpy(), df_wrench['torque_y'].to_numpy(), label='Torque Y', linestyle='--', color='tab:purple')
ax2.plot(df_wrench['time_sec'].to_numpy(), df_wrench['torque_z'].to_numpy(), label='Torque Z', linestyle='--', color='tab:brown')
ax2.tick_params(axis='y')
ax2.legend(loc='upper right')

plt.title("Euler Angles and Torque vs Time")
plt.grid()
plt.tight_layout()
plt.show()


