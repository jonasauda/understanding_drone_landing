# Understanding Drone Landing on the Human Body

## Jonas Auda, Martin Weigel, Jessica R. Cauchard and Stefan Schneegass

## Contact
jonas.auda@uni-due.de

## Abstract
We envision the human body as a platform for fast take-off and landing of drones
in entertainment and professional uses such as medical emergencies, rescue missions,
or supporting police units. This new interaction modality challenges our knowledge
of human-drone experiences, in which interaction usually occurs at a distance from
the body. This work explores important factors for understanding the interplay between
drones and humans. We first investigated the suitability of various body locations
for landing in an online study (N = 159). Our results, presented as body maps, show
that the hand and upper back are particularly well-suited body locations. We further
tested these findings in a follow-up study (N = 12), in which participants experienced
drones landing on their bodies through carefully designed and pre-recorded 360° videos.
This immersion into the landing scenarios helped us to identify common themes and
research approaches for different body parts. Taken together, the findings provide
first insights into location preferences and themes for drones landing on the human
body.


## Citation/BibTeX

```
@inproceedings{10.1145/3447526.3472031,
author = {Auda, Jonas and Weigel, Martin and Cauchard, Jessica R. and Schneegass, Stefan},
title = {Understanding Drone Landing on the Human Body},
year = {2021},
isbn = {9781450383288},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3447526.3472031},
doi = {10.1145/3447526.3472031},
abstract = { We envision the human body as a platform for fast take-off and landing of drones
in entertainment and professional uses such as medical emergencies, rescue missions,
or supporting police units. This new interaction modality challenges our knowledge
of human-drone experiences, in which interaction usually occurs at a distance from
the body. This work explores important factors for understanding the interplay between
drones and humans. We first investigated the suitability of various body locations
for landing in an online study (N = 159). Our results, presented as body maps, show
that the hand and upper back are particularly well-suited body locations. We further
tested these findings in a follow-up study (N = 12), in which participants experienced
drones landing on their bodies through carefully designed and pre-recorded 360°videos.
This immersion into the landing scenarios helped us to identify common themes and
research approaches for different body parts. Taken together, the findings provide
first insights into location preferences and themes for drones landing on the human
body.},
booktitle = {Proceedings of the 23rd International Conference on Mobile Human-Computer Interaction},
articleno = {23},
numpages = {13},
keywords = {Virtual Reality Study, Human-Drone Interaction, Drone, Mechanical Turk, On-Body Landing, UAV, Micro Air Vehicle},
location = {Toulouse &amp; Virtual, France},
series = {MobileHCI '21}
}

```
# Documentation

## Requirements

This project relys on a Optical tracking system. Therefore, to run this project you need to stream tracking data (position and rotation) of the drones to this framework.

We use VinteR (https://github.com/jonasauda/VinteR) to stream our data from OptiTrack Motive to our application. You can implement your own adapter to receive tracking data in vinter_receiver.py


## Configuration

You can configure drones and flightpaths in this framework.

1. PID values can be configured in [drones.json](drone_control/drones.json).
2. Currently, we do not have our own PID controller. We suggest to use this controller: [PID Controller](https://pypi.org/project/simple-pid/).
3. Each drone has a flight path attribute that specifies a set of waypoints that the drone should follow.
4. Here is a example of a flight path [path_back.json](drone_control/flight_paths/crazy/path_back.json).
5. The app can be started from [proxy_drone_app.py](drone_control/proxy_drone_app.py)
6. In [drone.py](drone_control/drones/drone.py) you can integrate your tracking system. You need to provide a centroid (position (x,y,z)) and a rotation (quaternion (x,y,z,w) or in Euler angles).
7. This data is then retrieved by the "control_drone" loop to steer the drone.

If you have any questions, feel free to write an email to: jonas.auda@uni-due.de

# License
MIT
