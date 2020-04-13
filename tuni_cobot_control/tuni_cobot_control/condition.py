from owlready2 import *

class ReceivedCommand(Thing):
    def evaluate(self, world, robot):
        cmd = world.search_one(type = world['http://onto-server-tuni.herokuapp.com/Panda#Command'])
        if cmd:
            method = world.search_one(is_a = world['http://onto-server-tuni.herokuapp.com/Panda#CommandMethod'])
            if cmd.has_action=="give":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#HandoverTask']
            elif cmd.has_action=="release":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#ReleaseTask']
            elif cmd.has_action=="grasp":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#GraspTask']
            elif cmd.has_action=="reach":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#ReachTask']
            elif cmd.has_action=="pick":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#PickTask']
            destroy_entity(cmd)
            return True
        else:
            return False

class IsWaitingForSomething(Thing):
    def evaluate(self, world, robot):
        return True if ('isWaitingForSomething' in robot.__dict__ and robot.isHoldingSomething == True) else False

class IsHoldingSomething(Thing):
    def evaluate(self, world, robot):
        print(robot.name)
        return True if ('isHoldingSomething' in robot.__dict__ and robot.isHoldingSomething == True) else False

class IsNotHoldingSomething(Thing):
    def evaluate(self, world,robot):
        return True if (not 'isHoldingSomething' in robot.__dict__ or robot.isHoldingSomething == False) else False

class IsCapableOfReaching(Thing):
    def evaluate(self, world, robot):
        result = True if ('isCapableOfReaching' in robot.__dict__ and robot.isCapableOfReaching == True) else False
        print(result)
        return result

class IsNotCapableOfReaching(Thing):
    def evaluate(self, world,robot):
        return True if (not 'isCapableOfReaching' in robot.__dict__ or robot.isCapableOfReaching == False) else False

class IsReadyToBeTaken(Thing):
    def evaluate(self, world,robot):
        return False

class IsNotReadyToBeTaken(Thing):
    def evaluate(self, world,robot):
        pass
