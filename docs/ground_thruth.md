## GROUND TRUTH GENERATION

We have added support to generate ground truth data for computer vision algorithms (e.g. collision avoidance). Below we present some result of our solution, you can execute an example by running the following commands:
```bash
        roslaunch usv_sim airboat_segmentation_2.launch parse:=true
        roslaunch usv_sim airboat_segmentation_2.launch parse:=false
```
As soon as both UWSIM show up, press **c** on your keyboard in each UWSim window, so the boat camera will be rendered.

<p align="center">
  <img src="./images/ground_thruth.png" width="800" alt="Ground Truth generation"/>
</p>

