# rostest_example
Example of rostest/unittest integration in Python using a wall detection algorithm.

After downloading this package, putting it into `catkin_ws/src`, and `catkin_make`-ing, you can test it out by running `rostest rostest_example test-wall-detector.launch`.

This set of test cases tests out the node defined in `src/wall_detector.py`, which takes in a custom `ScanPoints.msg` message (which consists of an x and y value array) and tries to fit a line to these points. Then, it puts the slope and y-intercept of this line into a custom `Line.msg` message and publishes it.

To conduct these tests, test nodes are defined in `src/wall_detector_test.py`. ROS's testing functionalities work with the `unittest` Python library, and so anything that works with that applies here. A good starting video on the library is [here](https://www.youtube.com/watch?v=6tNS--WetLI). Documentation is [here](https://docs.python.org/3/library/unittest.html).

The general idea of how this library works is that you create a test class that subclasses `unittest.TestCase`. Then, you create class methods, where each one's name starts with `test_` and represents a single test case. Finally, you write code to actually run a test in each of these methods. This usually entails running `self.assert*` statements (once again, see documentation [here](https://docs.python.org/3/library/unittest.html)). If the asserts return true, then the test case passes. Else, it fails. Any additional errors in the test case would also result in failure.

As for how this works with ROS, in each test case, one can define nodes that publish/subscribe to topics that your node of interest connects to. In this example, the test nodes each have a ground truth line that they get some points from. Then, they publish these points, subsequently waiting for `wall_detector` to try to figure out the ground truth line's slope and intercept values. They receive these points via a subscriber. Finally, they assert that the received slope and intercept are similar to the actual ones.

To actually run these nodes, we put everything into a launch file called `test-wall-detector.launch`. Note that the test node is labelled with a `test` flag. `rostest` works the same way as `roslaunch`, just that it also runs the nodes with this flag. In fact, one can run `roslaunch rostest_example test-wall-detector.launch` -- it just ignores the test nodes! You can try this yourself -- run that command and then check `rqt_graph`, and `wall_detector` should be the only node there!

Finally, note that one has to add a line to `package.yml` to get `rostest` to work with a package (the `test_depend` part). For more information about actually creating tests, see [here](http://wiki.ros.org/rostest/Writing). For more examples of rostest in action, see [here](https://github.com/mit-rss/rostest_example).
