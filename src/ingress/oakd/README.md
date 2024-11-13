# Oak-D Camera Ingress

This module handles the ingress for the Oak-D camera. 

Refer to the [official documentation](https://docs.luxonis.com/software/depthai/hello-world/) for more details.

- **oakd_ingress.py**: Contains the driver to retrieve RGB and depth images from the cameras.
- **oakd_node.py**: Publishes the images to the topic.

**IMPORTANT**: Check the IDs of the cameras in `oakd_cams.yaml`. Each camera has an ID printed on top of it. Ensure that the ID matches the one specified in the configuration file.
