
# How disturbances can affect the vehicles

One of the key points of usv_sim is to simulate environmental disturbances that negativelly affect the vehicle pose, navigation and control algorithms. The implemented disturbances models can affect vehicles in different ways:
- **NONE:** vehicle will not be affected by the disturbance;
- **GLOBAL:** vehicle will be affected by a global disturbance. It will not change in time and in space (same value to all over the place);
- **LOCAL:** vehicle will be affected by a local disturbance. It can change in time and in space. Disturbance values will acquired from wind_current (OpenFoam Simulation) and water_current (HEC-RAS Simulation).

## CONFIGURATION

You can configure how the wind current and how the water current will affect each vehicle. Thus you can define on `windType` and `waterType` parameter one of the following options: `none`, `global`, `local`.

Below it is presented the portion of a launch file that it responsible to configure an airboat in the simulation. In this case, the `windType` and `waterType` was configurated with value `local`. 
```
        <include file="$(find usv_sim)/launch/models/spawn_airboat_validation.launch">
                <arg name="gui" value="$(arg gui)"/>
                <arg name="spawnGazebo" value="$(arg spawnGazebo)"/>
                <arg name="namespace" value="$(arg namespace)"/>
                <arg name="windType" value="local"/>
                <arg name="waterType" value="local"/>
        </include>
```

Below, it is present another example, where a differential boat (named `diffboat1`). It was configurated such that the wind current is set to `global` (i.e. will not change in space and time), and the water current was defined to `none`.  This way, the vehicle will only affected by wind.
```
        <include file="$(find usv_sim)/launch/models/spawn_diffboat_validation.launch">
                <arg name="gui" value="$(arg gui)"/>
                <arg name="spawnGazebo" value="$(arg spawnGazebo)"/>
                <arg name="namespace" value="diffboat1"/>
                <arg name="windType" value="global"/>
                <arg name="waterType" value="none"/>
        </include>
```


## VIDEOS

The following videos show how bad the environmental disturbances can affect the boat navegability.

<p align="center">Differential boats sailing upstream - Global vs Local disturbances</p> 
<p align="center">
        <a href="http://www.youtube.com/watch?feature=player_embedded&v=JEhY3h-BKGQ" target="_blank">
                <img src="./images/diffboat_scenario.png" alt="Differential boats sailing upstream" width="423" height="271" border="10" />
        </a>
</p>
        


### Scenario 1

[![Airboat Scenario 1](https://img.youtube.com/vi/jvgcgIXkRtQ/0.jpg)](https://www.youtube.com/watch?v=jvgcgIXkRtQ)
[![Diff boat Scenario 1](https://img.youtube.com/vi/u-JnylVnD9I/0.jpg)](https://www.youtube.com/watch?v=u-JnylVnD9I)
[![Rudder boat Scenario 1](https://img.youtube.com/vi/QlenP-I_Oms/0.jpg)](https://www.youtube.com/watch?v=QlenP-I_Oms)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=jvgcgIXkRtQ" target="_blank">
 <iframe width="400" height="250" src="https://www.youtube.com/embed/jvgcgIXkRtQ"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=u-JnylVnD9I" target="_blank">
 <iframe width="400" height="250" src="https://www.youtube.com/embed/u-JnylVnD9I"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=QlenP-I_Oms" target="_blank">
 <iframe width="400" height="250" src="https://www.youtube.com/embed/QlenP-I_Oms"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>

### Scenario 2

[![Airboat Scenario 2](https://img.youtube.com/vi/aAN48eRpTSw/0.jpg)](https://www.youtube.com/watch?v=aAN48eRpTSw)
[![Diff boat Scenario 2](https://img.youtube.com/vi/pb13mWNcg74/0.jpg)](https://www.youtube.com/watch?v=pb13mWNcg74)
[![Rudder boat Scenario 2](https://img.youtube.com/vi/sTJ3DfIYY_M/0.jpg)](https://www.youtube.com/watch?v=sTJ3DfIYY_M)


<a href="http://www.youtube.com/watch?feature=player_embedded&v=aAN48eRpTSw" target="_blank">
  <iframe width="400" height="250" src="https://www.youtube.com/embed/aAN48eRpTSw"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=pb13mWNcg74" target="_blank">
  <iframe width="400" height="250" src="https://www.youtube.com/embed/pb13mWNcg74"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=sTJ3DfIYY_M" target="_blank">
  <iframe width="400" height="250" src="https://www.youtube.com/embed/sTJ3DfIYY_M"  frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</a>

