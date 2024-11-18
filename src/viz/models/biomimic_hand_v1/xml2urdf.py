import mujoco_py
from lxml import etree

# Load the MuJoCo model
model = mujoco_py.load_model_from_path('robot.xml')
sim = mujoco_py.MjSim(model)

# Create an empty URDF structure
robot_urdf = etree.Element("robot", name="robot")

# Iterate through MuJoCo bodies and joints and create URDF elements
for i in range(model.nbody):
    body_name = model.body_names[i]
    body = etree.SubElement(robot_urdf, "link", name=body_name)

    # Visual and inertial properties can be added here
    visual = etree.SubElement(body, "visual")
    geom = etree.SubElement(visual, "geometry")
    geom_type = model.geom_type[i]  # This will give you the type of geom
    # Add geometry based on geom_type (sphere, box, etc.)

    # Inertial properties can be added if needed
    inertial = etree.SubElement(body, "inertial")
    mass = model.body_mass[i]
    inertia = model.body_inertia[i]
    inertial.set("mass", str(mass))
    # Add inertia matrix here

# Save to URDF file
tree = etree.ElementTree(robot_urdf)
tree.write("robot.urdf", pretty_print=True)

