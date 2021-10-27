# cpp_pubsub

Run using:
ros2 run cpp_pubsub listener

To generate a pointstamp from a command line argument use:
ros2 topic pub --once /topic geometry_msgs/msg/PointStamped "{header: {frame_id: "base_link"}, point:{x: 0.0, y: 0.0, z: 1.8}}"

