def load_config_data():
    import json
    import os
    import re

    cwd_path = re.match(".*HallerSim/(?=devel)", __file__)[0]
    path = os.path.join(cwd_path, 'src', 'haller_sim', 'auvConfig', 'auvConfig.json')
    with open(path, 'r') as file:
        return json.load(file)['simulation']