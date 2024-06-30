## Supported Boats

Describe the available boats and how they were built.

Figures: show each boat

.. image:: ./images/airboat.png
    :align: center

.. image:: ./images/rudder.png
    :align: center


.. image:: ./images/diff.png
    :align: center


.. image:: ./images/sailing.png
    :align: center

.. image:: ./images/wam.png
    :align: center


.. WARNING::

  @ To be done by Marcelo



## Boat Models

There are 4 boat models preconfigured in package usv_sim:
- airboat: composed by one thruster above the hull. This model has greater advantaged to navigate on shallow waters.
- differential boat: two thruster under water surface. This model has the simplest maneuverability.
- rudder boat: one thruster and one rudder. One of the most common configuration presented in boats.
- sailboat: one sail and one rudder.

<p align="center">
  <img src="./docs/source/images/boats.png" width="800" alt="4 boat models"/>
</p>

The hull of all models above has been subdivided in 6 parts (see image above), so waves affects buoyancy of model in such way that boats present more realistic movement. If you want greater realism, you can subdivided the hull in more parts. To do that, you have to use geometric tools like Blender to model each part of hull. After that, you should configure links and joints in xacro files (like usv_sim/xacro/boat_subdivided4.xacro). As gazebo simulator combine fixed joints, you should define the joints of hull as of type revolution, but with zero value to upper and lower limits. 

<p align="center">
  <img src="./docs/source/images/boat_hull.png" width="800" alt="Boat hull subdivision"/>
</p>
