import pickle
import pprint
import numpy as np

with open('transforms.pkl', 'rb') as f:
    transform_dict = pickle.load(f)
    for key, value in transform_dict.items():
        print(f"Key: {key}")
        print(f"Value: {value}")
        print()
