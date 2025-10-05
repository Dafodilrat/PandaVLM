import os
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim. Use headless mode if you do not want a UI.
simulation_app = SimulationApp({"headless": False})

import omni.usd

# The absolute path to your USD file
usd_path = os.path.join("Isaac_panda","Arm","env.usda")

# Wait for the Isaac Sim application to fully load and initialize.
# The app.update() call is crucial for ensuring the application is ready.

print(f"Opening stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)

# You can add more code here to interact with the stage

# Run the simulation loop
try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    pass

# Close the application cleanly when finished
simulation_app.close()