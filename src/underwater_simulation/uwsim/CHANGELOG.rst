^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uwsim
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.1 (2015-12-14)
------------------
* Can use package:// to resolve ROS package location for meshes
* Can use package:// to pull a urdf from a ROS package
* Fixed transform publishing to fit the new robotpublisher interface
* Contributors: Bence Magyar, perezsolerj

1.4.0 (2015-09-23)
------------------
* Initial support for markers and interactive markers issue #1
* MultibeamSensors now use multiple cameras when the FOV is high in order to avoid eyefish distortions and allow FOV >180º.
* Added xacro support for scene building, some xacro macros created as example
* Added lightRate configuration option, shader projection is no longer affected by lighting
* Support for buried objects and dredging
* Added ARMask to Augmented reality elements such as trails, axis, rangeSensor debug, pointclouds and multibeamSensor debug so they are not showed on virtual cameras
* Added debug visible tag to multibeam sensors (similar to range sensor)
* Added gaussian random noise to camera output through shaders
* Underwater particles are no longer present on multibeamSensor, although it can be activated through XML
* Added ROSPointCloudLoader interface that allows to visualize 3D pointclouds on demand.
* Major update on shader management:
* added timeWindow and restarter (y key) to trajectory trail.
* Bugs and fixes.
* Contributors: David Fornas, Miguel Ribeiro, perezsolerj
