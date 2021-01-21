![](AI-car-demo1.gif)


![](AI-car-demo2.gif)


Deep Reinforcement learning project which features a simple obstacle avoiding AI car made with [Unity ML Agents](https://github.com/Unity-Technologies/ml-agents).
The Agent controls acceleration and steering (both continuous). Observations include the normalized local position, velocity, angular velocity, steering angle, torque and the dot product of the Agent's forward direction and the direction to the next waypoint. The Agent was trained with PPO.

The track is procedurally generated every episode using Catmull-Rom splines, based on https://www.gamasutra.com/blogs/GustavoMaciel/20131229/207833/Generating_Procedural_Racetracks.php

The car model is made using blender.

Unity version used : 2019.4.17f1

ML-Agents version used : 1.7.2, release 12




External assets used in this project :

Arcade car physics by saarg

https://assetstore.unity.com/packages/tools/physics/arcade-car-physics-119484

Free HDR Sky by ProAssets

https://assetstore.unity.com/packages/2d/textures-materials/sky/free-hdr-sky-61217





Limitations at the moment :

- The procedural track may rarely intersect with itself.
- The track is 2D i.e. the Y location of the track is 0 everywhere.
- The Agent does not perform well at sharp turns especially with obstacles at those turns.
- The Agent stops moving if it collides with an obstacle head on.

Instructions to train in Google Colab :
- Make a x64 Linux server build in Unity
- Upload the build folder to Google Drive
- Go to Google Colab and use [this Jupyter Notebook](UnityProjects_MLAgents.ipynb)
