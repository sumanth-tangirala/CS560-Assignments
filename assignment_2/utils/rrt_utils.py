import numpy as np
from constants import ALPHA
from utils import angular_diff


class RRTNode:
    def __init__(self, config, parent=None):
        self.config = config
        self.position = config[:2]
        self.orientation = config[2]
        self.parent = parent

        self.children = {}

    def __str__(self) -> str:
        return f'{self.config}'
    
    def __repr__(self) -> str:
        return f'{self.config}'

    def add_child(self, child, action, trajectory):
        self.children[child] = [action, trajectory]

    def compute_distance(self, other_config, individual=False, alpha = ALPHA):
        pos_dist = np.linalg.norm(self.position - other_config[:2])
        angle_diff = np.abs(angular_diff(self.orientation, other_config[2]))
        
        if individual:
            return pos_dist, angle_diff
        else:
            return (ALPHA * pos_dist) + ((1 - ALPHA) * angle_diff)