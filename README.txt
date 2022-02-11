To run my code, simply clone this repo into the root openFrameworks folder (so the .git folder is next to a folder titled of_v0.11.2_vs2017_release that contains all the openFrameworks library), then open apps/myApps/brooks_HW1/brooks_HW1.sln in Visual Studio 2019 and click play. You may get a lot of warnings from the openFrameworks code if it was not set up to use visual studio 2019, but it should still work.

Once the app is running, click the buttons along the top of the screen to toggle which demo is being run.

Most of the relevant code for this assignment is located in src/ai_behaviors.cc. Specifically:

For part one (Kinematic Motion) see KinematicSeek() in src/ai_behaviors.cc for the core code of the algorithm, and SetupKinematicDemo() and UpdateKinematicDemo() in src/hw1_app.cc for usage.

For part two (Seek Steering Behaviors) see DynamicSeek(), DynamicArrive(), and LookWhereYouAreGoing() in src/ai_behaviors.cc for the core code of the algorithms, and SetupSeekDemo(), SetupSeekArriveDemo(), UpdateSeekDemo(), and UpdateSeekArriveDemo() in src/hw1_app.cc for usage.

For part three (Wander Steering Behaviors) see DynamicWander() and DynamicWander2() in src/ai_behaviors.cc for the core code of the algorithm, and SetupWanderDemo(), SetupWander2Demo(), UpdateWanderDemo(), and UpdateWander2Demo() in src/hw1_app.cc for usage.

For part four (Flocking Behavior and Blending/Arbitration) see Flocking(), DynamicSeparate(), DynamicArrive(), and VelocityMatch() for the core code of the algorithm, and SetupFlockingDemo() and UpdateFlockingDemo() for usage.
