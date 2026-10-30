import numpy as np
np.set_printoptions(precision=3, suppress=True)


class HandController:
    def __init__(self, hand_chirality):
        self.max_joint_width = 0.15
        self.min_joint_width = 0.0
        self.num_motors = 1
        self.hand_chirality = hand_chirality
        
    def check_joint_width(self, joint_width):
        if joint_width < self.min_joint_width or joint_width > self.max_joint_width:
            return False
        else:
            return True
    
    def init_joints(self, calibrate: bool = False):
        pass
    
    def command_joint_angles(self, joint_width):
        # check if the width is within the limits, if not return error
        return self.check_joint_width(joint_width)
        
if __name__ == "__main__" :
    gc = HandController()
    gc.init_joints()
