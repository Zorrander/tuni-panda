#!/usr/bin/env python
import rospy
import rostest
import unittest
from sem_server_ros.srv import CreateSem, ReadSem, UpdateSem, DeleteSem
from sem_server_ros.msg import URI, Triple

rdf = "http://www.w3.org/1999/02/22-rdf-syntax-ns"
owl = "http://www.w3.org/2002/07/owl"
cogtuni = "http://cognitive.robotics.tut"

class CRUDServices(unittest.TestCase):
    ''' Create, Read, Update, Delete '''

    def test_create_sem(self):
        rospy.wait_for_service('create_sem')
        create_sem = rospy.ServiceProxy('create_sem', CreateSem)
        resp = create_sem(URI(cogtuni, "TestClass"), URI(rdf, "type"), URI(owl, "Class"))
        created_triple = Triple(resp.feedback.subject.value, resp.feedback.predicate.value, resp.feedback.object.value)
        test_triple = Triple("TestClass", "type", "Class")
        test = True if test_triple == created_triple else False
        self.assertTrue(test, "Triple are not added correctly.")

    def test_read_sem(self):
        rospy.wait_for_service('read_sem')
        read_sem = rospy.ServiceProxy('read_sem', ReadSem)
        subject = "FrankaDictionary"
        resp = read_sem(subject)
        test = False
        for triple in resp.result:
            if Triple("FrankaDictionary", "type", "NamedIndividual") ==  Triple(triple):
                test = True
        self.assertTrue(test, "Ontologies cannot be read correctly")

    '''
    def test_update_sem(self):
        rospy.wait_for_service('update_sem')
        update_sem = rospy.ServiceProxy('update_sem', UpdateSem)
        subject = "FrankaDictionary"
        resp = update_sem(subject_uri)
        return True

    def test_delete_sem(self):
        rospy.wait_for_service('delete_sem')
        update_sem = rospy.ServiceProxy('delete_sem', DeleteSem)
        resp = update_sem()
        return True
    '''


if __name__ == '__main__':
    unittest.main()
