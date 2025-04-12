import json
import numpy as np

home = np.array([[0.], [0.], [0.], [0.], [.0], [0.]])
np.save('/home/aaronbuergenthal/vsCode/6axisRobot/data/known_pos.npy', home)
positions = np.load('/home/aaronbuergenthal/vsCode/6axisRobot/data/known_pos.npy').astype(float)
print(positions)

# Define a dictionary with a single key storing multiple arrays
arrays_dict = {
    "arrays": [
        [0]
    ]
}

# Define the file path
jsonFname = "routines.json"
json_file_path = f"/home/aaronbuergenthal/vsCode/6axisRobot/data/{jsonFname}"

# Save the dictionary to a JSON file
with open(json_file_path, "w") as json_file:
    json.dump(arrays_dict, json_file, indent=4)  # Pretty-print with indentation

with open(json_file_path, "r") as file:
    data = json.load(file)
print(data)