import time

import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description

# Load a practice robot (this will be replaced by your own URDF later)
urdf = load_robot_description("panda_description")

# Start the viewer
server = viser.ViserServer()
ViserUrdf(server, urdf_or_path=urdf)

print("Open http://localhost:8080 in your browser")
print("Press Ctrl+C here to stop")

# Keep the program running
while True:
    time.sleep(1)