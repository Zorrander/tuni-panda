class Triple:

    def __init__(self, *args):
        if len(args) == 3:
            self.subject = args[0]
            self.predicate = args[1]
            self.object = args[2]
        else:
            print("ERROR: Cannot create triple with args {}".format(args))

    def __eq__(self, other):
        result = True if (self.subject == other.subject and self.predicate == other.predicate and self.object == other.object) else False
        return result

    def __str__(self):
        return "---\n{} \n{} \n{}\n---".format(self.subject, self.predicate, self.object)


class Collection:

    def __init__(self, list_triples):
        self.triples = [Triple(subject, predicate, object) for subject, predicate, object in list_triples]

class ActionCollection(Collection):

     def has_type(self):
         ''' Indicate whether the action command the arm or the gripper '''
         pass

class ObjectCollection(Collection):

    def has_location(self):
        pass

    def has_width(self):
        pass
