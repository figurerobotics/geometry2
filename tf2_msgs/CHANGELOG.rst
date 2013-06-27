^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2013-06-27)
------------------
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_

0.3.6 (2013-03-03)
------------------

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

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* removing packages with missing deps
* adding include folder
* adding tf2_msgs/srv/FrameGraph.srv
* catkin fixes
* catkinizing geometry-experimental
* catkinizing tf2_msgs
* Adding ROS service interface to cpp Buffer
* fix tf messages dependency and name
* add python transform listener
* Compiling version of the buffer server
* Compiling version of the buffer client
* Adding a message that encapsulates errors that can be returned by tf
* A fully specified version of the LookupTransform.action
* Commiting so I can merge
* Adding action for LookupTransform
* Updating CMake to call genaction
* Moving tfMessage to TFMessage to adhere to naming conventions
* Copying tfMessage from tf to new tf2_msgs package
* Creating a package for new tf messages