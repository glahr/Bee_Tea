# Bee_Tea
A behaviour tree implementation on ROS using Python. Can use ROS actionlib servers as leaf nodes.

See scripts/bt_example.py for an example of how to construct a BT and use a ROS action server at a leaf node.
An example run from `scripts/`:

Terminal 1:
```
$ ./bt_action_node.py test
```


Terminal 2:
```
$ ./bt_example.py
```

You should implement something that is not counting to 5 in the `act`, `_fail`, `_succeed`, `_preempt_cb`, and `_execute_cb`
methods of `BT_ActionNode` in `scripts/bt_action_node.py`.
