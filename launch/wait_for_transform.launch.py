import launch
from launch import LaunchContext, LaunchDescription
from launch.event import Event
from launch_ros.actions import Node

def generate_launch_description():
  sender = Node(
      package='wait_for_transform',
      executable='sender',
      name='sender',
      parameters=[{'timeout_ms': 5000}, {'send_first_tf': 'before'}, {'send_second_tf': 'during'}, {'send_buffer_time_ms': 5}],
      remappings=[("points_in", "points_raw")]
  )
  receiver = Node(
      package='wait_for_transform',
      executable='receiver',
      name='receiver',
      parameters=[{'timeout_ms': 5000, 'method': 'lt', 'executor': 'single', 'request_time': 'current'}],
      remappings=[("points_in", "points_raw")]
  )

  def on_exit(event: Event, context: LaunchContext):
    print(event)
    print(context)
    pass

  shutdown_handler = launch.actions.RegisterEventHandler(
      launch.event_handlers.OnProcessExit(
          target_action=sender,
          on_exit=on_exit,
      )
  )
  return LaunchDescription([sender, receiver, shutdown_handler])