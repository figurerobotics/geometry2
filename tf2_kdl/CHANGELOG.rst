^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_kdl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2013-06-27)
------------------
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* passing unit tests

0.3.6 (2013-03-03)
------------------
* fix compilation under Oneiric

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2
* fixed missing include export & tf2_ros dependecy

0.3.1 (2013-02-14)
------------------
* fixing version number in tf2_kdl
* catkinized tf2_kdl

0.3.0 (2013-02-13)
------------------
* fixing groovy-devel
* removing bullet and kdl related packages
* catkinizing geometry-experimental
* catkinizing tf2_kdl
* fix for kdl rotaiton constrition
* add twist, wrench and pose conversion to kdl, fix message to message conversion by adding specific conversion functions
* merge tf2_cpp and tf2_py into tf2_ros
* Got transform with types working in python
* A working first version of transforming and converting between different types
* Moving from camelCase to undescores to be in line with python style guides
* kdl unittest passing
* whitespace test
* add support for PointStamped geometry_msgs
* Fixing script
* set transform for test
* add advanced api
* working to transform kdl objects with dummy buffer_core
* plugin for py kdl
* add regression tests for geometry_msgs point, vector and pose
* add frame unit tests to kdl and bullet
* add first regression tests for kdl and bullet tf
* add bullet transforms, and create tests for bullet and kdl
* transform for vector3stamped message
* move implementation into library
* add advanced api
* compiling again with new design
* renaming classes
* compiling now
* almost compiling version of template code
* add test to start compiling