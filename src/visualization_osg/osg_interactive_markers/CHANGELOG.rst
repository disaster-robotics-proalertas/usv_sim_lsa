^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package osg_interactive_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2015-09-17)
------------------
* Changed return type to object, as osg was internally creating a copy (temporary reference return)
* Now mesh interactive markers autoscale to mesh size.
* Contributors: perezsolerj

1.0.1 (2015-06-11)
------------------
* catkin_make_isolated compatibility
* catkinizing package
* osg_interactive_markers: fixed bug that published world tranform feedback instead of local transform
* IM: Fixed bug that removed all the scene when deleting IM
* IM: delete markers scenegraph in destructors
* IM: Fixes for fuerte
* IM: fixed wall_dt and ros_dt in demo
* IM: fix for getting fixed_frame from frameManager
* osg_interactive_markers changed example name
* Contributors: marioprats@gmail.com, perezsolerj
