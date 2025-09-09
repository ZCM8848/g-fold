# G-FOLD Python Generator

This is the Python implementation of the G-FOLD algorithm with code generation capabilities. The problem is described using CVXPY/Python and C/C++ code is generated using CVXPYGen.

## Installation

### Option 1: Install from source

#### Prerequisites

You need to install [Rust](https://www.rust-lang.org/tools/install) and [Eigen](https://github.com/oxfordcontrol/Clarabel.cpp#installation) for the code generation feature.

Clone the repository:

```bash
git clone https://github.com/samutoljamo/g-fold.git
cd g-fold/generator
```

Install the package in development mode:

```bash
pip install -e .
```

On WSL, make sure you've installed Tkinter (version depends on the python version you're using):
```bash
sudo apt-get install python3.12-tk
```

### Option 2: Install from PyPI

```bash
pip install gfold
```

## Usage

### As a command-line tool

After installation, you can run G-FOLD from the command line:

```bash
# Solve the example problem with 100 steps and display graphs
gfold -n 100

# Generate C++ code NOTE: not supported on windows
gfold -g -n 100 -o output_directory

# Save the plot to a file without displaying it
gfold -n 100 --save-plot --no-plot
```

### As a Python library

```python
from gfold import GFoldSolver
from gfold.visualization import plot_results

# Create a solver with 100 steps
solver = GFoldSolver()

# Solve the problem
solution = solver.solve(verbose=True)
print(f"Final mass: {solution['final_mass']:.2f} kg")

# Plot the results
plot_results(solution, save_path="gfold_plot.png")

# Generate C++ code
solver.generate_code(code_dir="generated_code")
```

### After code generateon
see *[Here](https://github.com/cvxgrp/cvxpygen?tab=readme-ov-file#2-solve--compare)* or below:
```python
from math import log
from random import uniform
from compiled_solvers.tower_catch.cpg_solver import cpg_solve
from solver import GFoldSolver, config, visualization
from numpy import array
import time

t0 = time.time()
vessel_config = config.GFoldConfig()
g = 9.80665

# Falcon 9 specific parameters
m_dry = 22.2 * 1e3      # dry mass kg
m_fuel = 13.4 * 1e3     # fuel mass kg
T_max = 845000    # maximum thrust N
Isp = 300               # specific impulse s
throt = [0.4, 1.0]      # throttle range
    
# Initial conditions (coordinate system: x-downrange, y-crossrange, z-altitude)
vessel_config.spacecraft.initial_position = array([5, 10, 2000])  # r_ from Falcon 9 data 
vessel_config.spacecraft.initial_velocity = array([2, -10, -150])  # v0 from Falcon 9 data 
    
# Mass and fuel parameters
vessel_config.spacecraft.fuel_consumption = 1 / (Isp * g)  # alpha parameter
vessel_config.spacecraft.wet_mass = m_dry + m_fuel  # total wet mass
vessel_config.spacecraft.fuel = m_fuel  # fuel mass
    
# Thrust parameters
vessel_config.spacecraft.min_thrust_pct = throt[0]  # minimum throttle
vessel_config.spacecraft.max_thrust_pct = throt[1]  # maximum throttle
vessel_config.spacecraft.real_max_thrust = T_max  # maximum thrust
    
# Target conditions
vessel_config.spacecraft.target_position = array([0, 0, 23.58])  # rf from Falcon 9 data
vessel_config.spacecraft.target_velocity = array([0, 0, 0])  # vf from Falcon 9 data
vessel_config.spacecraft.target_direction = array([0, 0, 1])  # uf from Falcon 9 data
    
# Environment parameters
vessel_config.environment.gravity = array([0, 0, -g])  # gravity vector
vessel_config.environment.glide_slope_angle = 1  # y_gs in degrees
vessel_config.environment.max_angle = 10  # p_cs constraint
    
# Solver parameters
vessel_config.solver.n = 100  # N nodes in discretization

solver = GFoldSolver(vessel_config)
problem = solver.problem
problem.register_solve('CPG', cpg_solve)

# solve problem with C code via python wrapper
result = problem.solve(method='CPG')
t1 = time.time()
# result_direct = solver.solve()
print('\nCVXPYgen\nSolve time: %.3f ms\n' % (1000 * (t1 - t0)))

while True:
    t0 = time.time()
    position = array([uniform(-200, 200), uniform(-200, 200), uniform(1000, 3000)])
    velocity = array([uniform(-10, 10), uniform(-10, 10), uniform(-200, -100)])
    # print(position, velocity)
    m_fuel = 13.4e3 - uniform(0, 13.4e3)
    solver.update_parameter('initial_position', position)
    solver.update_parameter('initial_velocity', velocity)
    solver.update_parameter('log_mass', log(m_fuel + m_dry))
    solver.update_parameter('log_dry_mass', log(m_dry))
    solution = problem.solve(method='CPG')
    print(f'solved in {time.time() - t0}s')
    result = {}
    result['positions'] = problem.solution.primal_vars[1][:, :3]
    result['velocities'] = problem.solution.primal_vars[1][:, 3:]
    # note that in some occations, using this method to get results from primal_vars may fail. Then you should try to find key by shape.
```
### Example scripts

Check the `examples` directory for more usage examples:

- `simple_example.py`: Basic usage of the solver and visualization

## Development

To set up the development environment:

```bash
cd g-fold/generator
pip install -e .
```
