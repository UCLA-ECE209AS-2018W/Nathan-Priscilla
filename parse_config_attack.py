from __future__ import print_function
import configparser
import sys
import os.path


def parse_config(config_file):

    config_params = {}
    
    # Initialize config parser object
    config = configparser.ConfigParser()

    if not os.path.isfile(config_file):
        print('Cannot find', config_file+'. ', 'Exiting now.')
        exit(1)
    else:
        # Read and parse config file
        config.read(config_file)
        
        for config_key in config.items():
            if config_key[0] == 'DEFAULT':
                continue
            config_params[config_key[0]] = dict(config.items(config_key[0]))
            config_params[config_key[0]] = {key:float(val) for key, val in config_params[config_key[0]].items()}
        
        # Print configuration parameters loaded
        print('Finished loading configs:')
        for key in config_params.keys():
            print('---',key,'---')
            print(config_params[key])

    return config_params

#if __name__ == "__main__":
#    if len(sys.argv) < 1:
#        print('No config file provided. Usage: python parse_config_attack.py <name_of_config_file>')
#        exit(1)
#    else:
#        config_file = sys.argv[1]
#        config_params = parse_config(config_file) 
#        #print(config_params)
