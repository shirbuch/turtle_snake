
def print_turtle_pose(pose, message=None):
    print("----------------")
    if message:
        print(message)
    print("x:", pose.x)
    print("y:", pose.y)
    print("theta:", pose.theta)
    pose_degrees = pose.theta * 180/PI
    print("pose_degrees:", pose_degrees)
    print("----------------")

def print_observing_handling():
    print("------------ Observer ----------------")
    print(f"prev_pose: {prev_pose}")
    print(f"current_pose: {current_pose}")
    print("-------------------------------------")
    print(f"Distances: {distances}")
    print(f"Angles: {angles}")
    print("-------------------------------------")

def get_wall_facing(pose_degrees):
    ret = ""
    if facing_down(pose_degrees):
        ret += "down "
    if facing_up(pose_degrees):
        ret += "up "
    if facing_right(pose_degrees):
        ret += "right "
    if facing_left(pose_degrees):
        ret += "left "

    return ret
