def load_config_data():
    import json
    import os
    import re

    cwd_path = re.match(".*(?<=HallerSim)", __file__)[0]
    path = os.path.join(cwd_path, 'src', 'haller_sim', 'auvConfig', 'simulationConfig.json')
    with open(path, 'r') as file:
        return json.load(file)


def parse_camel_to_snake_case(string: str) -> str:
    import re
    return re.sub(r'(?<!^)(?=[A-Z])', '_', string).lower()
