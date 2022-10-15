# -*- coding: utf-8 -*-
import os
import yaml
import argparse
def dict2namespace(config):
    #声明命名空间
    namespace = argparse.Namespace()
    for key, value in config.items():
        if isinstance(value, dict):
            new_value = dict2namespace(value)
        else:
            new_value = value
        #将参数对添加到命名空间中
        setattr(namespace, key, new_value)
    return namespace

def load_argparse():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg',type = str,default=r"myArgs.yaml",help="...") 
    args = parser.parse_args()
    filepath = os.path.join(os.getcwd(), args.cfg)

    with open(filepath, 'r',encoding='utf-8') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    configs = dict2namespace({**config, **vars(args)})

    return configs