
### Robot Co-optimization using planar linkage
The base code implements the planar linkage kinematics in `linkage.py`, and the linkage can be installed on a robot and simulate to test the robot performance using `linkage_physics.py`.  Finally, we implemented a basic linkage optimizer using simulated annealing in `optimizer_anneal.py`.

#### Install

The code has been tested on Python 3.6 and 3.12. 

To run the code, you first need to install Anaconda, which can be downloaded from https://docs.anaconda.com/anaconda/install/

Once Anaconda is installed, you can open the anaconda and type the following commands in order:

1. now, you can create your conda environment, named "box2d-py"
   
`conda create -n box2d-py`

2. remember every time you activate the environment, before you use it.
   
`conda activate box2d-py`

3. once you activate the environment for the first time, you need to install the following packages.

`conda install conda-forge::box2d-py` (box2d-py (https://github.com/pybox2d/pybox2d). Fortunately, you can install it via Anaconda.)
   
`pip install pygame`

`pip install numpy`

`pip install vapory`

4. run the Python scripts one by one
    
`python linkage.py`

`python optimizer_anneal.py`

`python linkage_physics.py`

### About linkage_physics.py

To run linkage_physics.py, you need the 'best.pickle' file that contains the design from optimizer_anneal.py. That said, before you run optimizer_anneal.py, you do not have it yet. 

If you want to simply test the linkage_physics.py, instead, you can replace 

`robot=create_robot('best.pickle', sep=5.)`

with 

`link=Linkage.createSimple()`

`robot=create_robot(link, sep=5.) `

Once you run linkage_physics.py, you can control the robots via

press 'SPACE' to turn on/off simulation

press 'RCTRL' to turn on/off robot motor


### Installing Povray on Windows

On windows, the default povray does not work. We need a special version of povray that is independently compiled called MegaPov.
I have included a version of this MegaPov in the project, i.e., the megapov.rar. To enable povray on windows, unzip this folder to any directory.
Let's say you have a povray.exe located in "C:\Users\Admin\AppData\Roaming\POV-Ray\bin".
After this step, you need to add this to your system path as follows:
![alt text](https://github.com/dyingbrain/RoboCoOpt/blob/main/sys.png)

#### Note that you might need to restart the system to let the setting enabled.


Now you can press "LSHIFT" in the window to have the povray render a nice image for you, an example is as follows:
![alt text](https://github.com/dyingbrain/RoboCoOpt/blob/main/frm.png)

### Record your robot walking animation

To help you share the result you are most proud of, when you call function `main_linkage_physics`, you can pass the parameter `recordTime=10` to record a 10-second animation as follows:
![alt text](https://github.com/dyingbrain/RoboCoOpt/blob/main/record.gif)
