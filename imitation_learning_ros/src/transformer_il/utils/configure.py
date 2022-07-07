from configparser import ConfigParser
import yaml

def read_yaml(path):
    file = open(path, 'r', encoding='utf-8')
    string = file.read()
    dict = yaml.safe_load(string)

    return dict


def create_ini_config():
    #Get the configparser object
    config_object = ConfigParser()


    #Assume we need 2 sections in the config file, let's call them USERINFO and SERVERCONFIG
    config_object["ENC_INFO"] = {
        "env": "insert",
        "K": 10,
        "pct_traj":1,
        "batch_size": 64,
        "model_type": 'dt',
        "embed_dim": 128,
        "n_layer": 3,
        "n_head": 1,
        "activation_function": "relu",
        "dropout": 0.1,
        "learning_rate": 1e-4,
        "weight_decay": 1e-4,
        "warmup_steps": 10000,
        "num_eval_episodes": 100,
        "max_iters": 10,
        "num_steps_per_iter": 10000,
        "device": "cpu",
        "log_to_wandb": False,
    }

    config_object["DEC_INFO"] = {
        "env": "insert",
        "dataset": 'medium',
        "mode": 'normal',
        "K": 10,
        "pct_traj":1,
        "batch_size": 64,
        "model_type": 'dt',
        "embed_dim": 128,
        "n_layer": 3,
        "n_head": 1,
        "activation_function": "relu",
        "dropout": 0.1,
        "learning_rate": 1e-4,
        "weight_decay": 1e-4,
        "warmup_steps": 10000,
        "num_eval_episodes": 100,
        "max_iters": 10,
        "num_steps_per_iter": 10000,
        "device": "cpu",
        "log_to_wandb": False,
    }

    #Write the above sections to config.ini file
    with open('configs/config.ini', 'w') as conf:
        config_object.write(conf)