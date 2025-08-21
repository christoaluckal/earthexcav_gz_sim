import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from deltacan.msg import DeltaCan
import numpy as np
from scipy.interpolate import CubicSpline

class LookUpOperation:
    def __init__(self):
        signal = np.arange(-1.0, 1.0, 0.1)
        self.bucket_spline = CubicSpline(signal, [self.bucket_cmd(x) for x in signal])
        self.arm_spline = CubicSpline(signal, [self.arm_cmd(x) for x in signal])
        self.boom_spline = CubicSpline(signal, [self.boom_cmd(x) for x in signal])
        pass

    def bucket_cmd(self,x):
        if x < -0.9: return -0.7464773197463939
        elif x < -0.8: return -0.7556077862660108
        elif x < -0.7: return -0.7475886335802124
        elif x < -0.6: return -0.6943159340316355
        elif x < -0.5: return -0.3478137666597351
        elif x < -0.45: return -0.11074819014084174
        elif x >= -0.45 and x <= 0.45: return 0.0
        elif x < 0.45: return -0.03170411937007261
        elif x < 0.5: return 0.07074540207098638
        elif x < 0.6: return 0.1531096691607231
        elif x < 0.7: return 0.494843938147395
        elif x < 0.8: return 0.9654822056154688
        elif x < 0.9: return 1.1323096288265486
        elif x < 1.0: return 1.124019310302267
        else: return 1.1221103605940879

    def arm_cmd(self, x):
        if x < -0.9: return -0.7297529607089939
        elif x < -0.8: return -0.7417820435603295
        elif x < -0.7: return -0.7455697543943114
        elif x < -0.6: return -0.6743000194405915
        elif x < -0.5: return -0.2602819510725163
        elif x < -0.45: return -0.07527410444846894
        elif x >= -0.45 and x <= 0.45: return 0.0
        elif x < 0.45: return -0.028214468531235375
        elif x < 0.5: return 0.023708225612123934
        elif x < 0.6: return 0.09856746947233368
        elif x < 0.7: return 0.38919978426931034
        elif x < 0.8: return 0.7416978037933635
        elif x < 0.9: return 0.7555933051518472
        elif x < 1.0: return 0.752780260055926
        else: return 0.744536923787804

    def boom_cmd(self, x):
        if x < -0.9: return 0.4463869246347824
        elif x < -0.8: return 0.4503544485398761
        elif x < -0.7: return 0.43847760993112805
        elif x < -0.6: return 0.3985477917802957
        elif x < -0.5: return 0.22969102031072908
        elif x < -0.45: return 0.10025171220496362
        elif x >= -0.45 and x <= 0.45: return 0.0
        elif x < 0.45: return 0.04753058433724343
        elif x < 0.5: return -0.06316787902224205
        elif x < 0.6: return -0.13641786070618744
        elif x < 0.7: return -0.2727715499858695
        elif x < 0.8: return -0.45252302846893266
        elif x < 0.9: return -0.5158417589198473
        elif x < 1.0: return -0.5363396786385148
        else: return -0.5367303013977662


class AbstractController(Node):
    def __init__(self):
        super().__init__('abstract_controller')

        self.w_ref = [0.0]*3
        self.B = 10
        self.phi = np.pi/2
        self.C = 5
        self.last_control = [0.0]*3
        self.last_msg_time = self.get_clock().now()
        self.inactivity_timeout = 1.0

        self.num_joints = 3
        self.last_u = [0.0] * self.num_joints
        self.start_times = [None] * self.num_joints

        self.lookup = LookUpOperation()

        self.subscription = self.create_subscription(
            DeltaCan,
            '/deltacan',
            self.listener_callback,
            10
        )

        self.velocity_pub = self.create_publisher(Float64MultiArray, '/manipulator_controller/commands', 10)
        self.inactivity_checker = self.create_timer(0.5, self.check_inactivity)

    def check_inactivity(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds * 1e-9
        if elapsed > self.inactivity_timeout:
            self.start_times = [None] * self.num_joints
            self.last_u = [0.0] * self.num_joints

    def compute_wi(self, i, t):
        if t >= 5.0 or self.start_times[i] is None:
            return 1
        return (1+np.sin(self.B * t + self.phi) * np.exp(-self.C * t))

    def listener_callback(self, msg:DeltaCan):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_msg_time = self.get_clock().now()
        u1 = msg.mboomcmd
        u2 = msg.marmcmd
        u3 = msg.mbucketcmd
        u = [u1, u2, u3]
        # self.w_ref[0] = self.lookup.boom_cmd(u1)
        # self.w_ref[1] = self.lookup.arm_cmd(u2)
        # self.w_ref[2] = self.lookup.bucket_cmd(u3)

        self.w_ref[0] = self.lookup.boom_spline(u1)
        self.w_ref[1] = self.lookup.arm_spline(u2)
        self.w_ref[2] = self.lookup.bucket_spline(u3)

        w_out = []
        for i in range(self.num_joints):
            if abs(u[i]) > 1e-3:
                if abs(self.last_u[i] - u[i]) > 1e-3:
                    self.start_times[i] = now
                t_elapsed = now - self.start_times[i] if self.start_times[i] is not None else 0.0
                scale = self.compute_wi(i, t_elapsed)
                w_i = self.w_ref[i] * scale
            else:
                self.start_times[i] = None 
                w_i = 0.0

            w_out.append(w_i)
            self.last_u[i] = u[i]
            

        msg_out = Float64MultiArray()
        msg_out.data = w_out
        self.velocity_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    controller = AbstractController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()