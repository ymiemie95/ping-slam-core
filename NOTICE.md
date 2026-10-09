# Third-party and package notices

This repository is a workspace containing components with different licenses;
it is not covered by one blanket license.

- `src/sonar_slam_frontend/` is BSD-3-Clause and contains derived BSD code
  documented in its `NOTICE` file.
- `src/sonar_slam_backend/` is derived from slam_toolbox and contains LGPL-2.1 back-end code and a separately
  licensed Karto SDK (LGPL-3.0), with some retained upstream CCPL headers;
  its `NOTICE` file records the attribution and modified files.
- `src/datasetMessageDefinitions/` contains ROS message packages with their
  own package metadata and upstream licenses.

Keep the package-level license and notice files when redistributing any part of
the workspace. Check the upstream license before combining these packages into
a different product.
