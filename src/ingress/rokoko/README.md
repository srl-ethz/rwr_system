# Rokoko Gloves Ingress

This module handles the ingress for Rokoko gloves. The data is streamed in JSON format. For more details, refer to the [official documentation](https://support.rokoko.com/hc/en-us/articles/4410416376977-Custom-Streaming).

`RokokoTracker` manages all operations, and you can access the raw keypoints through the `self.keypoint_positions` object or by using the `get_keypoints_position` method. The node publishes data to the `/ingress/mano` and `/ingress/wrist` topics.

