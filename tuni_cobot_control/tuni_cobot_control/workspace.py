from owlready2 import *


class Workspace(Thing):

    def is_empty(self):
        return False if len(self.contains) > 0 else True

    def test_contains(self):
        print("Contains")
        return self.contains

    def print_status(self):
        meta = ['namespace', 'storid']
        for key in self.__dict__:
            if not key in meta:
                print("{} - {}".format(key, self.__dict__[key]))
