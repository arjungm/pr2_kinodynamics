pr2_kinodynamics
================

public snapshot of the kindoynamics utilities for the pr2 arm

Requirements:

* Boost v1.53 or greater
* ROS Fuerte

Problems:

* ROS Fuerte is built against Boost 1.46

Workaround:

* pr2_dynamics is manually built against the latest version of Boost
* pr2_trajectory is built against the ROS version of Boost

Explanation:

* Odeint and Shared Pointers are header-only
