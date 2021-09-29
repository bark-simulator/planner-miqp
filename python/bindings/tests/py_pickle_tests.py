
try:
    import debug_settings
except:
    pass

import unittest
import pickle
import numpy as np

from bark.core.world.agent import *
from bark.core.models.behavior import *
from bark.core.world import *
from bark.core.world.map import *
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.core.world.goal_definition import *
from bark.core.world.evaluation.ltl import *
from bark.runtime.commons.parameters import ParameterServer

from bark.core.models.behavior import BehaviorMiqpAgent

def pickle_unpickle(object):
    with open('temp.pickle','wb') as f:
        pickle.dump(object,f)
    object = None
    with open( 'temp.pickle', "rb" ) as f:
        object = pickle.load(f)
    return object


class PickleTests(unittest.TestCase):

    def test_behavior_safety(self):
      params = ParameterServer()
      pickle_unpickle(BehaviorMiqpAgent(params))


if __name__ == '__main__':
    unittest.main()