from math import atan2, degrees, radians

robot_pos = (1200, -1200)
target_pos = (1000, -1200) # snhould give us 0*

dx, dy = target_pos[0] - robot_pos[0], target_pos[1] - robot_pos[1]
print(dx, dy)
heading_to_target = degrees(atan2(dx, dy))
if dx < 0:
    heading_to_target = 360 + heading_to_target
print(heading_to_target)