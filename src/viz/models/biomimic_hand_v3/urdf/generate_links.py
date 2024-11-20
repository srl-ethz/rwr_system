#!/usr/bin/env python3
import xml.etree.ElementTree as ET

def extract_links(urdf_file):
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # List to hold link names
    links = []

    # Iterate over all 'link' elements in the URDF
    for link in root.findall('.//link'):
        link_name = link.get('name')
        if link_name:
            links.append(link_name)

    # Sort links alphabetically
    links.sort()
    return links

def generate_rviz_config(links):
    # Define the header for the .rviz configuration
    rviz_config = '''Links:
  All Links Enabled: true
  Expand Joint Details: false
  Expand Link Details: false
  Expand Tree: false
  Link Tree Style: Links in Alphabetic Order
'''

    # Add each link to the configuration in the format expected by RViz
    for link in links:
        rviz_config += f'  {link}:\n'
        rviz_config += '    Alpha: 1\n'
        rviz_config += '    Show Axes: false\n'
        rviz_config += '    Show Trail: false\n'

    return rviz_config

def save_rviz_config(config, output_file):
    # Save the generated RViz config to a file
    with open(output_file, 'w') as f:
        f.write(config)
    print(f"RViz configuration saved to {output_file}")

def main():
    # Input URDF file and output RViz config file
    urdf_file = 'biomimic_hand.urdf'  # Replace with your URDF file path
    output_file = 'output_config.rviz'  # The RViz config output file

    # Extract links from the URDF
    links = extract_links(urdf_file)

    # Generate RViz configuration from the links
    rviz_config = generate_rviz_config(links)

    # Save the generated config to a file
    save_rviz_config(rviz_config, output_file)

if __name__ == '__main__':
    main()

