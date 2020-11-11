# `wait_for_transform`

The sender publishes transforms with timestamps 100 and 104. The transform with timestamp 102 is requested by the receiver, which should be obtainable from the transforms at 100 and 104.

These timestamps have no relation to wall time. The wall time behavior can be configured with parameters:
* `send_first_tf`: Whether the first transform (timestamp 100) is sent before, during or after the waiting period
* `send_second_tf`: Whether the second transform (timestamp 104) is sent before, during or after the waiting period
* `timeout`: How long the receiver should wait for the transform after it has received the trigger message