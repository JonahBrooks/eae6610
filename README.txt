To run my code, simply clone this repo into the root openFrameworks folder (so the .git folder is next to a folder titled of_v0.11.2_vs2017_release that contains all the openFrameworks library), then open apps/myApps/brooks_HW2/brooks_HW2.sln in Visual Studio 2019 and click play. You may get a lot of warnings from the openFrameworks code if it was not set up to use visual studio 2019, but it should still work.

Most of the relevant code for this assignment is located in src/ai_pathfinding.cc and in src/hw2_app.cc. I'm especially proud of the wall creation and graph modification code found in src/hw2_app.cc's MakeWallAndEditEdges function.

Once the game is running, click anywhere on the screen (aside from the button in the top left) to put down a waypoint (green dot). Yellow dots will appear showing the path the Boid will follow to reach that point. If you click the button, the game will halt for a while to process the two graphs from part 1 of the assignment and output analysis data to the terminal.


The following is the README from Assignment 1:

To run my code, simply clone this repo into the root openFrameworks folder (so the .git folder is next to a folder titled of_v0.11.2_vs2017_release that contains all the openFrameworks library), then open apps/myApps/brooks_HW1/brooks_HW1.sln in Visual Studio 2019 and click play. You may get a lot of warnings from the openFrameworks code if it was not set up to use visual studio 2019, but it should still work.

Once the app is running, click the buttons along the top of the screen to toggle which demo is being run.

Most of the relevant code for this assignment is located in src/ai_behaviors.cc. Specifically:

For part one (Kinematic Motion) see KinematicSeek() in src/ai_behaviors.cc for the core code of the algorithm, and SetupKinematicDemo() and UpdateKinematicDemo() in src/hw1_app.cc for usage.

For part two (Seek Steering Behaviors) see DynamicSeek(), DynamicArrive(), and LookWhereYouAreGoing() in src/ai_behaviors.cc for the core code of the algorithms, and SetupSeekDemo(), SetupSeekArriveDemo(), UpdateSeekDemo(), and UpdateSeekArriveDemo() in src/hw1_app.cc for usage.

For part three (Wander Steering Behaviors) see DynamicWander() and DynamicWander2() in src/ai_behaviors.cc for the core code of the algorithm, and SetupWanderDemo(), SetupWander2Demo(), UpdateWanderDemo(), and UpdateWander2Demo() in src/hw1_app.cc for usage.

For part four (Flocking Behavior and Blending/Arbitration) see Flocking(), DynamicSeparate(), DynamicArrive(), and VelocityMatch() for the core code of the algorithm, and SetupFlockingDemo() and UpdateFlockingDemo() for usage.

