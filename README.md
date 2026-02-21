# Project Description
This project is a simulation environment for a 3DOF-per-leg hexapod named TIFFANY. The idea is to improve movements and robot adaptability in complex environment settings.

# Modeling and Tools
The project uses onshape-to-robot to convert CAD assemblies into URDF files with collision and inertial data.
PyBullet serves as the primary physics engine for tests.

# Future Development
This project is structured to transition from gait control to machine learning. Future versions will implement Gymnasium to apply Reinforcement Learning. 


# Cloning the Repository
```bash
git clone https://github.com/SENAI4LIFE/VIRTUALHEXAPOD.git
```

# Create virtual environment
```bash
python3 -m venv TIFFANY
```

# Activate environment
```bash
source TIFFANY/bin/activate
```

# Navigate to project directory
```bash
cd ~/VIRTUALHEXAPOD/HEXAPOD/COMPLETE
```

# Install dependencies
```bash
pip install --upgrade pip
pip install pybullet
pip install numpy
```

# Start Simulation
```bash
python3 main.py
```

# Controls
Arrows: Walk / Rotate

1 / 2 / 3: Switch Gaits (Tripod, Wave, Ripple)

E: Power On / Initialize

R: Power Off / Shutdown


#train
```bash
pip install stable-baselines3
pip install shimmy
pip install tensorboard
pip install gymnasium
```
