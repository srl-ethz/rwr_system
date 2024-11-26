import numpy as np

 
finger_PIP_diameter = 12.4 #mm
finger_MCP_diameter = 11.5
finger_ABD_diameter = 13.5

thumb_MCP_diameter = 16.5
thumb_ABD_diameter = 14.6
thumb_PIP_diameter = 11.5
thumb_DIP_diameter = 12.5


wrist_joint_diameter = 25.0 # to optimize

def get_tendonlength(theta, joint_diameter):
   '''Input: joint angle of PIP joint in rad
      Output: total normal lengths of flexor tendon through PIP joint'''
   return (joint_diameter/2)*theta

def pose2wrist(theta):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [ get_tendonlength(theta, wrist_joint_diameter),
             get_tendonlength(theta, wrist_joint_diameter)]

def pose2tendon_length(muscle_name, theta_ABD, theta_MCP, theta_PIP, theta_DIP=None):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   
   if muscle_name == "thumb":
      return [ get_tendonlength(theta_ABD, thumb_MCP_diameter),
               get_tendonlength(theta_ABD, thumb_MCP_diameter),
               get_tendonlength(theta_MCP, thumb_ABD_diameter),
               get_tendonlength(theta_MCP, thumb_ABD_diameter),     
               get_tendonlength(theta_PIP, thumb_PIP_diameter),
               get_tendonlength(theta_PIP, thumb_PIP_diameter),
               get_tendonlength(theta_DIP, thumb_DIP_diameter),
               get_tendonlength(theta_DIP, thumb_DIP_diameter)]
   else:
      return [ get_tendonlength(theta_ABD, finger_ABD_diameter),
             get_tendonlength(theta_ABD, finger_ABD_diameter),
             get_tendonlength(theta_MCP, finger_MCP_diameter),
             get_tendonlength(theta_MCP, finger_MCP_diameter),
             get_tendonlength(theta_PIP, finger_PIP_diameter),
             get_tendonlength(theta_PIP, finger_PIP_diameter)]



# def tendonlength_flexor_PIP(theta_PIP):
#    '''Input: joint angle of PIP joint in rad
#       Output: total normal lengths of flexor tendon through PIP joint'''
#    return (finger_PIP_diameter/2)*theta_PIP

# def tendonlength_extensor_PIP(theta_PIP):
#    '''Input: joint angle of PIP joint in rad
#       Output: total normal lengths of extensor tendon through PIP joint'''
#    return (finger_PIP_diameter/2)*theta_PIP

# def tendonlength_flexor_MCP(theta_MCP):
#    '''Input: joint angle of MCP joint in rad
#       Output: total normal lengths of flexor tendon through MCP joint'''
#    return (finger_MCP_diameter/2)*theta_MCP

# def tendonlength_extensor_MCP(theta_MCP):
#    '''Input: joint angle of MCP joint in rad
#       Output: total normal lengths of extensor tendon through MCP joint'''
#    return (finger_MCP_diameter/2)*theta_MCP

# def tendonlength_flexor_ABD(theta_ABD):
#    # As flexor for ABD we consider the left abduction motion while looking at the palm
#    '''Input: joint angle of ABD joint in rad
#       Output: total normal lengths of flexor tendon through ABD joint'''
#    return (finger_ABD_diameter/2)*theta_ABD

# def tendonlength_extensor_ABD(theta_ABD):
#    '''Input: joint angle of ABD joint in rad
#       Output: total normal lengths of extensor tendon through ABD joint'''
#    return (finger_ABD_diameter/2)*theta_ABD



# def pose2tendon_finger(theta_ABD, theta_MCP, theta_PIP):
#    '''Input: controllable joint angles
#       Output: array of tendon lengths for given joint angles'''
#    return [ tendonlength_flexor_ABD(theta_ABD),
#             tendonlength_extensor_ABD(theta_ABD),
#             tendonlength_flexor_MCP(theta_MCP), 
#             tendonlength_extensor_MCP(theta_MCP),
#             tendonlength_flexor_PIP(theta_PIP),
#             tendonlength_extensor_PIP(theta_PIP),
#             ]

