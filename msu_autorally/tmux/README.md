# tmux #


## Wiki ##
Please see the msu_autorally_src wiki, link below, for more detailed installation and running instructions.
Wiki: https://github.com/gsimon2/msu_autorally_src/wiki

## Tmux Commands ##
To start the Evo-ROS instances on the VMs use the following command:
```tmuxp load start_evo_ros.yaml```

To kill the session once finished detach the tmux session (ctrl-b then d) and use the following command:
```tmux kill-session -t evo-ros-workers```

Ensure that no sessions are running with:
```tmux ls```

A Tmux cheatsheet can be found here:
https://gist.github.com/MohamedAlaa/2961058

Note that when working with many instances of Evo-ROS at once, it is often helpful to syncronize keystrokes across all panes. I have this setup on the SENS desktop bound to the ctrl-b then e (for sync on) and ctrl-b then E (for sync off). Please see the following link for more details:
https://stackoverflow.com/questions/27400246/send-a-keyboard-shortcut-to-all-of-the-panes-for-the-current-tmux-session



## File Discriptions ##
### start_evo_ros.yaml ###
Defines the nodes to be started when using Tmux to start the Evo-ROS instances on the robonode VMs.

### test_session.yaml ###
A small test session for nexting new behaviors while using Tmuxp.



