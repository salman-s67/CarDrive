This repo is part of a racing game project. It was developed in Unity 6 HDRP, version 6000.0.36f1.

GitHub did not allow a few files to be uploaded due to large file size. Please download these two files: 
https://studio67.io/wp-content/uploads/2025/08/Assets.zip, which contains Assets/Models/Bogota.fbx.
https://studio67.io/wp-content/uploads/2025/08/Assets2.zip, which contains Assets/Textures/Bogota and AMCGremlin.

In this project, ScriptableObjects are used to set up vehicles. The only vehicle model that was made was that of an AMC Gremlin using Blender. In the SO folder, you'll see AMCGremlin, AMCGremlinFast and SubaruWRX. The Gremlin had only 2 gears so it wasn't fun to drive, and therefore, values were copied from the WRX to create a GremlinFast version, which didn't exist in the real world.

In prefabs/AMCGremlinFast, you can enable IsAI to enable the auto-drive mode. It uses whiskers and a scout to look at the environment to make turning decisions. This technique was discussed in AI for Games by Ian Millington.

The physics for the car in non-AI mode comes from the book Race Car Design by Derek Seward, but some items, such as roll and tilt forces have not been computed, and are instead faked to reduce computational load on the system. If desired, the book's information can be used to implement these functions as well.

The BogotaMountainScene was created in Blender, and is based on real topographical information from Google for highway 40 from Villavicencio to Bogota. Sections of the track are floating above the surface; this is because the techniques being used to cut the road through the surrounding topography were still being experimented on at the time the included fbx file was exported.


I hope the code is of use to you; you may use it in your projects freely.
