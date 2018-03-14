=============
Dubins-Curves
=============

About
=====

This software finds the shortest paths between configurations for the Dubins'
car [Dubins51]_, the forward only car-like vehicle with a constrained turning
radius. A good description of the equations and basic strategies for doing this
are described in section 15.3.1 `"Dubins Curves"
<http://planning.cs.uiuc.edu/node821.html>`_ of the book "Planning Algorithms"
[LaValle06]_.

The approach used to find paths is based on the algebraic solutions published
in [Shkel01]_. However, rather than using angular symmetries to improve
performance, the simpler approach to test all possible solutions is used here. 

|build_status| |coverage| |license|

Usage
=====

The recommended approach is to add `dubins.c` and `dubins.h` to your project
and compile with an appropriate build system.  

The repository includes a basic cmake example that demonstrates how to build
and test the library.

Examples
========

The following code snippet demonstrates how to generate intermediate points
along the shortest path between a pair of configuration (x, y, theta).

.. code-block:: c

    #include "dubins.h"
    #include <stdio.h>

    int printConfiguration(double q[3], double x, void* user_data) {
        printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
        return 0;
    }

    int main()
    {
        double q0[] = { 0,0,0 };
        double q1[] = { 4,4,3.142 };
        double turning_radius = 1.0;
        DubinsPath path;
        dubins_shortest_path( &path, q0, q1, turning_radius);
        dubins_path_sample_many( &path, 0.1, printConfiguration, NULL);
        return 0;
    }

The following image shows some example paths, and the heading of the vehicle at
each of the intermediate configurations.

.. image:: ./docs/images/samples.png

Other Version
=============

* There is a MATLAB Mex wrapper of this code on the `MathWorks FileExchange <http://www.mathworks.com.au/matlabcentral/fileexchange/40655-dubins-curve-mex>`_
* There is a Python wrapper of this code available on `GitHub <https://github.com/AndrewWalker/pydubins>`_ and on `PyPI <https://pypi.python.org/pypi/dubins/>`_

Citing
======

If you would like to cite this library in a paper or presentation, the following is recommended:

.. code-block:: bibtex

    @Misc{DubinsCurves,
      author = {Andrew Walker},
      title  = {Dubins-Curves: an open implementation of shortest paths for the forward only car},
      year   = {2008--},
      url    = "https://github.com/AndrewWalker/Dubins-Curves"
    }

Contributors
============

The Dubin's curves library was completed as one small part of [Walker11]_. New
contributions or bug fixes are welcome.

Key contributors to the project include: 

* Francis Valentinis
* Royce Smart - who tested early versions of this code while writing up [Smart08]_.
* Scott Teuscher - who wrote the MATLAB Mex wrapper

License
=======

MIT License. See `LICENSE.txt <LICENSE.txt>`_ for details.

References
==========

.. [Dubins51] Dubins, L.E. (July 1957). "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents". American Journal of Mathematics 79 (3): 497–516
.. [LaValle06] LaValle, S. M. (2006). "Planning Algorithms". Cambridge University Press
.. [Shkel01] Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins set". Robotics and Autonomous Systems 34 (2001) 179–202
.. [Walker11] Walker, A. (2011). "Hard Real-Time Motion Planning for Autonomous Vehicles", PhD thesis, Swinburne University.
.. [Smart08] Royce, S. (2008). "Evolutionary Control of Autonomous Underwater Vehicles". PhD thesis, RMIT

.. |build_status| image:: https://secure.travis-ci.org/AndrewWalker/Dubins-Curves.png?branch=master
   :target: https://travis-ci.org/AndrewWalker/Dubins-Curves
   :alt: Current build status

.. |coverage| image:: https://codecov.io/gh/AndrewWalker/Dubins-Curves/branch/master/graph/badge.svg
   :target: https://codecov.io/gh/AndrewWalker/Dubins-Curves
   :alt: Code coverage shield

.. |license| image:: https://img.shields.io/badge/License-MIT-blue.svg
   :target: http://opensource.org/licenses/MIT
   :alt: license shield
