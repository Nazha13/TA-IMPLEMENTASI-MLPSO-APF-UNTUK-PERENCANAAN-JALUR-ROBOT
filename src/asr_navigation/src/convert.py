#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from main_controller.msg import ControllerData

class Converter:
    def __init__(self):
        rospy.init_node('msg_converter')
        
        rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.pub = rospy.Publisher('/robot/cmd_vel', ControllerData, queue_size=10)
        
        # ==========================================
        # SCALING
        # ==========================================
        # Berdasarkan program di comhardware, stm32 sudah terlanjur menerima satuan cm/s untuk linear dan deg/s untuk angular.

        self.LINEAR_SCALE  = 100.0 # konversi ke cm
        self.ANGULAR_SCALE = 180.0/3.14 # konversi ke deg
        
        # ==========================================
        # TUNING: DYNAMIC RANGE MODE
        # ==========================================
        # Linear: Tenaga minimum di 8 agar tidak stalling, batas atas 40
        self.MIN_LINEAR_POWER  = 8
        self.MAX_LINEAR_LIMIT  = 40
        
        # Angular: Tenaga minimum di 6 untuk rotasi halus, batas atas 20 untuk manuver cepat
        self.MIN_ANGULAR_POWER = 6
        self.MAX_ANGULAR_LIMIT = 20

    def callback(self, msg):
        custom_msg = ControllerData()
        # [Y, X, Theta]
        custom_msg.data = [0, 0, 0]

        # 1. CALCULATE RAW
        raw_x_fwd   = msg.linear.x * self.LINEAR_SCALE
        raw_y_side  = msg.linear.y * self.LINEAR_SCALE
        raw_z_turn  = msg.angular.z * self.ANGULAR_SCALE

        # 2. APPLY DEADBAND BOOST (Untuk melawan stiction motor)
        val_x = self.apply_deadband(raw_x_fwd, self.MIN_LINEAR_POWER)
        val_y = self.apply_deadband(raw_y_side, self.MIN_LINEAR_POWER)
        val_z = self.apply_deadband(raw_z_turn, self.MIN_ANGULAR_POWER)

        # 3. MAP TO ROBOT & CLAMP
        # Index 0 = Y (Translasi sumbu Y / Sideways)
        custom_msg.data[0] = max(min(val_y, self.MAX_LINEAR_LIMIT), -self.MAX_LINEAR_LIMIT)
        
        # Index 1 = X (Translasi sumbu X / Forward)
        custom_msg.data[1] = max(min(val_x, self.MAX_LINEAR_LIMIT), -self.MAX_LINEAR_LIMIT)
        
        # Index 2 = Angular (Rotasi sumbu Z)
        custom_msg.data[2] = max(min(val_z, self.MAX_ANGULAR_LIMIT), -self.MAX_ANGULAR_LIMIT)

        custom_msg.StatusControl = 1 
        self.pub.publish(custom_msg)

    def apply_deadband(self, val, min_cutoff):
        int_val = int(val)
        if int_val == 0: return 0
        
        if abs(int_val) < min_cutoff:
            return min_cutoff if int_val > 0 else -min_cutoff
        return int_val

if __name__ == '__main__':
    try:
        node = Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass