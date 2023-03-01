import json

# Define the ifm3d objects for the communication
from ifm3dpy.device import O3R
o3r = O3R()

# Get the current configuration
config = o3r.get()

# Print a little part from the config to verify the configuration
print(json.dumps(config['device']['swVersion'], indent=4))
# Note: this assumes that a camera is plugged into port 1
print(config['ports']['port1']['state'])

# Let's change the name of the device
o3r.set({'device':{'info':{'name':'great_o3r'}}})
o3r.save_init()

# Double check the configuration
config = o3r.get()
print(config['device']['info']['name'])
