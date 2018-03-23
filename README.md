# secure_carla
Secure CARLA

1. Config:
    - Dependency: configparser module.
        pip install configparser
    - Modify config_attack.ini to add/delete parameters and groups as needed
    - To use: import parse_config_attack
    - config_params = parse_config(config_attack_ini) will load groups in init file into config_params, a dict of dicts, for example: config_params['sensors_default'] = {'accel_mean':0, 'accel_var':1, accel_offset', ...}
