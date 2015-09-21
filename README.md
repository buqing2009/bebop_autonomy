# bebop_autonomy
ROS driver for Parrot Bebop drone

I add some function for bebop_autonomy, u can control your bebop by keyboard.
```
      't': take off
      'y': land
      'h': enter hovering mode
      'm': enter move mode
      'space bar': activate/deactivate emergency mode
```
In move mode:

```
      arrow_left: move/tilt left
      arrow_right: move/tilt right
      arrow_up: move/tilt forwards
      arrow_down: move/tilt backwards
      'q': move upwards
      'a': move downwards
      'z': yaw left/counterclockwise
      'x': yaw right/clockwise
      's': stop / send "zero" commands in every command. In ROS ardrone_autonomy this means entering the hovering mode	
```
