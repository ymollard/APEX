from apex_playground.msg import SensorialState

def ros_to_dict(sensorial_state):
    assert isinstance(sensorial_state, SensorialState)

    m = sensorial_state
    d = {"hand": {"position": [m.hand.pose.position.x, m.hand.pose.position.y, m.hand.pose.position.z],
                  "orientation": [m.hand.pose.orientation.x, m.hand.pose.orientation.y, m.hand.pose.orientation.z, m.hand.pose.orientation.w]},
         "ball": {"angle": m.ball.angle,
                  "extended": m.ball.extended},
         "joystick_1": m.joystick_1.axes,
         "joystick_2": m.joystick_2.axes,
         "ergo": {"angle": m.ergo.angle,
                  "extended": m.ergo.extended},
         "sound": m.sound.data,
         "color": m.color.data}
    return d
