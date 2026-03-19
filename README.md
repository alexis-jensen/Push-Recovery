# Physical Simulation of Balance Recovery after a Push




This code repo is to support the paper _**Physical Simulation of Balance Recovery after a Push
**_.
[[Paper Link](https://dl.acm.org/doi/10.1145/3623264.3624448)]
[[Youtube](https://youtu.be/vbad-yy8gxs)]

This paper has been accepted by *MIG '23: Proceedings of the 16th ACM SIGGRAPH Conference on Motion, Interaction and Games*

![](docs/teaser.png)

_**Abstract**_ --Our goal is to simulate how humans recover balance after external perturbation, e.g., being pushed. While different strategies can be adopted to achieve balance recovery, we particularly aim at replicating how humans combine the control of their support area with the control of their body movement to regain balance when it is necessary. We develop a physics-based approach to simulate balance recovery, with two main contributions to achieve our goal: a foot control technique to adjust the shape of a character's support zone to the motion of its center of mass (CoM), and the dynamic control of the CoM to maintain its vertical projection in this same zone. We also calibrate the simulation by optimisation, before validating our results against experimental data.

## Dependencies (from the [[Bullet](https://github.com/bulletphysics/bullet3)] repository)


A C++ compiler for C++ 2003. The library is tested on Windows, Linux, Mac OSX, iOS, Android,
but should likely work on any platform with C++ compiler. 

Visual Studio (Downloadable [[here](https://visualstudio.microsoft.com/fr/)] )

## Code Usage (Windows)

Click on build_visual_studio_vr_pybullet_double.bat and open build3/vs2010/0_Bullet3Solution.sln with Visual Studio.

When asked, convert the projects to a newer version of Visual Studio.
If you installed Python in the C:\ root directory, the batch file should find it automatically.
Otherwise, edit this batch file to choose where Python include/lib directories are located.

Add the **PushRecovery** folder's content to the build.

Building the project will create an executable that will automatically run once building is complete.

## Citation
    @inproceedings{10.1145/3623264.3624448,
    author = {Jensen, Alexis and Chatagnon, Thomas and Khoshsiyar, Niloofar and Reda, Daniele and Van De Panne, Michiel and Pontonnier, Charles and Pettr\'{e}, Julien},
    title = {Physical Simulation of Balance Recovery after a Push},
    year = {2023},
    isbn = {9798400703935},
    publisher = {Association for Computing Machinery},
    address = {New York, NY, USA},
    url = {https://doi.org/10.1145/3623264.3624448},
    doi = {10.1145/3623264.3624448},
    articleno = {23},
    numpages = {11},
    series = {MIG '23}
    }