# OPW ROE
ROE is Robot On Extender. Nothing more. Let's move on.

This is almost an exact copy of the [OPW](../opw/) interface package but made specifically for the case where a 6 DOF robotic arm (specifically the industrial arms that OPW is made for) is mounted on a prismatic/linear position, also called an Extender. The specific modifications are detailed in the next section.

## Modifications
1. ### Addition of `roe` to all the namespaces and file names
   All the files and namespaces therein, have an `roe` designation added in the middle. This is so the builder can differentiate.
   Also added the new solver to the CMakeLists.txt for the tesseract_kinematics package.
2. ### Parameters to specify the extender joint
   Specifically these parameters were added:
   - Base link transform : The transform describing the relation between extender frame (`y_prismatic`) and the positioner(`base_link`). This is useful for specifying configurations where the robot is mounted in a non-standard orientation (and even position, of course). Everything is in SI units (m).
   - Extender minimum : The minimum value of the extender position, in m. Typically, this is set to 0 m.
   - Extender maximum : The maximum value of the extender position, in m. Typically, this is a few centimeters short of the size of the gantry/positioner.
   - Extender sampling minimum deviation : This parameter defines the amount to add to the y coordinate of the inverse kinematics target, to obtain the minimum value in the sampling range. Typically, a negative value, in m.
   - Extender sampling maximum deviation : Same as the previous paramter but to obtain the maximum value in the sampling range. Typically, a positive value, in m.
   - Extender step : Defines the sampling frequency in m.
3. ### Extender joint sampling for inverse kinematics solutions
   The solver has been modified to sample the extender position and then apply the OPW solver for each extender sample. 