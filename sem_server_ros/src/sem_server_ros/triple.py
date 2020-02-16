class Triple:

    def __init__(self, *args):
        if len(args) == 3:
            self.subject = args[0]
            self.predicate = args[1]
            self.object = args[2]
        elif len(args) == 1:
            self.subject = args[0].subject.value
            self.predicate = args[0].predicate.value
            self.object = args[0].object.value
        else:
            print("ERROR: Cannot create triple with args {}".format(args))

    def __eq__(self, other):
        result = True if (self.subject == other.subject and self.predicate == other.predicate and self.object == other.object) else False
        return result

    def __str__(self):
        return "{} \n{} \n{}".format(self.subject, self.predicate, self.object)
