import numpy as np

 
finger_PIP_diameter = 15 #mm
finger_MCP_diameter = 15
finger_ABD_diameter = 18

def tendonlength_flexor_PIP(theta_PIP):
   '''Input: joint angle of PIP joint in rad
      Output: total normal lengths of flexor tendon through PIP joint'''
   return (finger_PIP_diameter/2)*theta_PIP

def tendonlength_extensor_PIP(theta_PIP):
   '''Input: joint angle of PIP joint in rad
      Output: total normal lengths of extensor tendon through PIP joint'''
   return (finger_PIP_diameter/2)*theta_PIP

def tendonlength_flexor_MCP(theta_MCP):
   '''Input: joint angle of MCP joint in rad
      Output: total normal lengths of flexor tendon through MCP joint'''
   return (finger_MCP_diameter/2)*theta_MCP

def tendonlength_extensor_MCP(theta_MCP):
   '''Input: joint angle of MCP joint in rad
      Output: total normal lengths of extensor tendon through MCP joint'''
   return (finger_MCP_diameter/2)*theta_MCP

def tendonlength_flexor_ABD(theta_ABD):
   # As flexor for ABD we consider the left abduction motion while looking at the palm
   '''Input: joint angle of ABD joint in rad
      Output: total normal lengths of flexor tendon through ABD joint'''
   return (finger_ABD_diameter/2)*theta_ABD

def tendonlength_extensor_ABD(theta_ABD):
   '''Input: joint angle of ABD joint in rad
      Output: total normal lengths of extensor tendon through ABD joint'''
   return (finger_ABD_diameter/2)*theta_ABD

def pose2tendon_finger(theta_PIP, theta_MCP, theta_ABD):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   return [tendonlength_flexor_PIP(theta_PIP),
            tendonlength_extensor_PIP(theta_PIP),
            tendonlength_flexor_MCP(theta_MCP), 
            tendonlength_extensor_MCP(theta_MCP),
            tendonlength_flexor_ABD(theta_ABD),
            tendonlength_extensor_ABD(theta_ABD)]
