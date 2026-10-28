// micro-ROS interface: the ONLY file that knows about ROS.            OWNER: Razvi
// Topics are the same as mock_hw_node on the Pi (doc section 5), so the Pi cannot tell
// the difference between the simulation and this board.
//
//   subscribes  /turret/cmd      TurretCommand    (also the heartbeat)
//               /launcher/cmd    LauncherCommand
//   publishes   /turret/state    TurretState      50 Hz
//               /launcher/state  LauncherState    5 Hz + on change
//               /range/target    Range            15 Hz
//               /system/faults   Fault            on event
#pragma once

bool ros_create();   // create node, publishers, subscribers; false on failure
void ros_destroy();
void ros_spin();     // call often while connected
