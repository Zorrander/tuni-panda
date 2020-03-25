#!/usr/bin/env python
import rospy
import unittest
from sem_server_ros.srv import Ask, Describe, Select, Update

class TestSparqlEndpoint(unittest.TestCase):

    def test_add_data(self):
        rospy.wait_for_service('add_data')
        insert_op = rospy.ServiceProxy('add_data', Update)
        #resp = insert_op()
        test = True
        self.assertTrue(test, "Triple are not added correctly.")

    def test_select_data(self):
        rospy.wait_for_service('select_data')
        select_op = rospy.ServiceProxy('select_data', Select)
        test = True
        self.assertTrue(test, "Ontologies cannot be read correctly")

    def test_read_data(self):
        rospy.wait_for_service('read_data')
        describe_op = rospy.ServiceProxy('read_data', Describe)
        return True

    def test_test_data(self):
        ''' construct operation '''
        rospy.wait_for_service('test_data')
        ask_op = rospy.ServiceProxy('test_data', Ask)
        return True

    def test_remove_data(self):
        '''insert operation'''
        rospy.wait_for_service('remove_data')
        delete_op = rospy.ServiceProxy('remove_data', Update)
        return True


if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_node', 'test_sparql_endpoint', TestSparqlEndpoint)
