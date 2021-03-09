import numpy as np

import os
# set the current working directory to the deployed package folder. This is required by isaac.
os.chdir("/home/davis/deploy/davis/rm_isaac_bridge-pkg")
os.getcwd()

from packages.pyalice import Composite

class Command(object):
    __slots__ = ('action', 'payload', 'response')
    def __init__(self, action, payload):
        self.action = action
        self.payload = payload
        self.response = None
        
class CompositeArray(object):
    __slots__ = ('_measure', '_entities', '_quantities', '_values')
    def __init__(self, entities, measure, values):
        self._measure = measure
        self._entities = entities
        self._quantities = [[x, self._measure, 1] for x in self._entities]
        
        if type(values) != np.ndarray:
            values = np.array(values, dtype=np.float64)
        self._values = values
        
    @property
    def composite(self):
        return Composite.create_composite_message(self._quantities, self._values)
    
    @composite.setter
    def composite(self, composite):
        values = Composite.parse_composite_message(composite, self._quantities)
        if len(self._values) != len(values):
            raise ValueError('Size of state does not match number of values')
        self._values = values
        
    @property
    def values(self):
        return self._values
    
    @values.setter
    def values(self, values):
        if type(values) != np.ndarray:
            values = np.array(values, dtype=np.float64)
        self._values = values
        
class CompositeValue(object):
    __slots__ = ('_measure', '_entity', '_quantity', '_value')
    def __init__(self, entity, measure, value):
        self._measure = measure
        self._entity = entity
        self._quantity = [[self._entity, self._measure, 1]]
        
        self._value = np.array(value, dtype=np.float64)
        
    @property
    def composite(self):
        return Composite.create_composite_message(self._quantity, self._value)
    
    @composite.setter
    def composite(self, composite):
        value = Composite.parse_composite_message(composite, self._quantity)
        if len(values) != 1:
            raise ValueError('Size of state does not match number of values')
        self._value = value
        
    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, value):
        value = np.array(value, dtype=np.float64)
        self._value = value