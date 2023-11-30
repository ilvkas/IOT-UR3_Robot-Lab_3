# IOT-UR3_Robot-Lab_3
<details> 
  <summary>Task </summary>
You are given a workcell consisting of two UR3 robot arms, and a conveyor belt in between them, that are working side-by-side. The task is to design and implement a sequencing algorithm for picking and sorting blocks. Two types of block are to be handled: (i) cylinders, and (ii) cubes.


Home locations: You must designate one robot whitespace as the final home of the cylinders, and the other whitespace and the final home for the cubes. You should designate an edge of each whitespace, as the actual place that the homed blocks shall be placed. The human user of your system may add these blocks at random in the white workspaces of either robot. The human is not allowed to place any blocks on the conveyor belt. The human may add blocks while your system is in operation. If the human removes any block at all, that block has to be from the home location.

Your system should sort all of the cylinders into the home edge designated for cylinders, and likewise must sort all of the cubes into the home edge for the cubes. You should sequence your robot arm movements and conveyor belt operations in such a way that your system sorts blocks in the shortest possible time. We do not mean that the conveyor belt must be running at top speed ! What we mean is that your sequencing must be such that no unnecessary idling takes place.

Nudge allowed: The camera does not find the locations of blocks very accurately. Therefore, you are allowed to use your hand to give a slight nudge to any block if it looks like the robot arm is moving a couple of centimetres off the correct location of that block. Your nudge cannot be more than 3 cm long!
</details>

# Installation
- Python Version 3.7.3 is required.
- requirements.txt contains all the required packages to run the program.
- Gripper.py & Gripper.script need to be in the same folder as your script.
