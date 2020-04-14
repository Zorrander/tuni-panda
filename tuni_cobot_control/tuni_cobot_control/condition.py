from owlready2 import *
from tuni_cobot_control.workspace import Workspace


class AnchoringError(Exception):
   def __init__(self, object):
      self.object = object

class SymbolGroundingError(Exception):
   def __init__(self, action_symbol):
      self.action_symbol = action_symbol

class ReceivedCommand(Thing):
    def evaluate(self, world, robot, workspace):
        cmd = world.search_one(type = world['http://onto-server-tuni.herokuapp.com/Panda#Command'])
        if cmd:
            method = world.search_one(is_a = world['http://onto-server-tuni.herokuapp.com/Panda#CommandMethod'])
            objects = world.search(is_a = world['http://onto-server-tuni.herokuapp.com/Panda#Object'])
            if cmd.has_target:
                anchored = []
                for object in objects:
                    if object.is_called == cmd.has_target:
                        anchored.append(object)
                if anchored:
                    method.actsOn = anchored[0]
                    print("anchoring {}".format(anchored[0]))
                else:
                    #raise AnchoringError(cmd.has_target)
                    return False
            if cmd.has_action=="give":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#HandoverTask']
            elif cmd.has_action=="clean":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#PackagingTask']
            elif cmd.has_action=="release":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#ReleaseTask']
            elif cmd.has_action=="grasp":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#GraspTask']
            elif cmd.has_action=="reach":
                print("will reach")
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#ReachTask']
            elif cmd.has_action=="pick":
                method.hasSubtask = world['http://onto-server-tuni.herokuapp.com/Panda#PickTask']
            destroy_entity(cmd)
            return True
        else:
            #raise SymbolGroundingError(cmd.has_action)
            return False

class EmptyWorkspace(Thing):
    def evaluate(self, world, robot, workspace):
        return workspace.is_empty()

class IsWaitingForSomething(Thing):
    def evaluate(self, world, robot, workspace):
        return True if ('isWaitingForSomething' in robot.__dict__ and robot.isHoldingSomething == True) else False

class IsHoldingSomething(Thing):
    def evaluate(self, world, robot, workspace):
        return True if ('isHoldingSomething' in robot.__dict__ and robot.isHoldingSomething == True) else False

class IsNotHoldingSomething(Thing):
    def evaluate(self, world,robot, workspace):
        return True if (not 'isHoldingSomething' in robot.__dict__ or robot.isHoldingSomething == False) else False

class IsCapableOfReaching(Thing):
    def evaluate(self, world, robot, workspace):
        result = True if ('isCapableOfReaching' in robot.__dict__ and robot.isCapableOfReaching == True) else False
        return result

class IsNotCapableOfReaching(Thing):
    def evaluate(self, world,robot, workspace):
        return True if (not 'isCapableOfReaching' in robot.__dict__ or robot.isCapableOfReaching == False) else False

class IsReadyToBeTaken(Thing):
    def evaluate(self, world,robot, workspace):
        return False

class IsNotReadyToBeTaken(Thing):
    def evaluate(self, world,robot, workspace):
        pass
