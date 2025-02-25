import math
from px4msgs.msg import Command
from rospy import Publisher, init_node, Subscriber, on_shutdown,Time, Rate
from std_msgs.msg import Bool

class ExampleTraj:
    def __init__(self):
        init_node('traj')
        self.pub = Publisher('/drone/poscmd', Command, queue_size=20)
        self.sub = Subscriber('/drone/allow_cmd', Bool, self.callback)
        self.allow_cmd = False

        self.ok = True
        on_shutdown(lambda: setattr(self, 'ok', False))
        start = False
        last = Time.now()
        circle = self.example_circle([0, 0], 3, 1)
        rate = Rate(100)
        while self.ok:
            if self.allow_cmd:
                if not start:
                    print("TRIGGERED")
                    start = True
                    last = Time.now()
                dt = Time.now() - last
                # dt*=2
                x, y, z, vx, vy, vz, yaw = circle(dt.to_sec())
                cmd = Command()
                cmd.type = Command.DESIRED_POS
                cmd.pos = [x, y, z]
                cmd.yaw = yaw
                cmd.vel = [vx, vy, vz]
                self.pub.publish(cmd)
            else:
                cmd = Command()
                cmd.type = Command.THRUST_BODYRATE
                cmd.u = [0, 0, 0, 0]
                self.pub.publish(cmd)
                start = False
            rate.sleep()
            

    def callback(self, msg:Bool):
        self.allow_cmd = msg.data

    @staticmethod
    def example_circle(initpos, radius, height):
        start = [radius, 0, 1]
        offset_x = initpos[0] - start[0]
        offset_y = initpos[1] - start[1]
        def circle(t):
            x = offset_x + radius * math.cos(t)
            y = offset_y + radius * math.sin(t)
            z = height
            vx = -radius * math.sin(t)
            vy = radius * math.cos(t)
            vz = 0
            yaw = 0
            return [x, y, z, vx, vy, vz, yaw]
        return circle
    
    @staticmethod
    def example_lemniscate(initpos, radius, height):
        start = [radius, 0, 1]
        offset_x = initpos[0] - start[0]
        offset_y = initpos[1] - start[1]
        def lemniscate(t):
            x = radius * math.cos(t)+  offset_x
            y = radius * math.sin(2*t)/2 + offset_y
            z = height
            sin = math.sin
            cos = math.cos
            vx = -radius * sin(t)
            vy = radius * cos(2*t)
            vz = 0
            # heading to the center
            yaw = 0
            return [x, y, z, vx, vy, vz, yaw]
        return lemniscate
    
if __name__ == '__main__':
    ExampleTraj()
    # Plot
    # from matplotlib import pyplot as plt
    # import numpy as np
    # loop = ExampleTraj.example_lemniscate([0, 0], 1, 1)
    # t = np.linspace(0, 2*np.pi, 100)
    # x = []
    # y = []
    # for i in t:
    #     x.append(loop(i)[0])
    #     y.append(loop(i)[1])
    # plt.plot(x, y)
    # plt.show()