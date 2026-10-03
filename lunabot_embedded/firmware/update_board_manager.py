import os

import yaml

if __name__ == "__main__":
    # Step 4: Make the .arduino directory if it doesn't exist
    yaml_file_path = os.path.join(
        os.path.expanduser("~"), ".arduino15/arduino-cli.yaml"
    )

    # Check if pyyaml is installed and import it
    try:
        from yaml import SafeDumper, SafeLoader
    except ImportError as e:
        print("Please install pyyaml to run this script.")
        raise e

    # Load the current YAML configuration
    with open(yaml_file_path) as yaml_file:
        config_data = yaml.load(yaml_file, Loader=SafeLoader)

    # Define the URL to add
    url_to_add = "https://www.pjrc.com/teensy/td_156/package_teensy_index.json"

    # Add the URL if it's not already in the list
    if "board_manager" not in config_data:
        config_data["board_manager"] = {"additional_urls": []}

    if url_to_add not in config_data["board_manager"].get("additional_urls", []):
        config_data["board_manager"]["additional_urls"].append(url_to_add)

    # Write the updated configuration back to the YAML file
    with open(yaml_file_path, "w") as yaml_file:
        yaml.dump(config_data, yaml_file, Dumper=SafeDumper)
