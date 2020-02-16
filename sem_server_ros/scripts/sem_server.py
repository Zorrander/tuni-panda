#!/usr/bin/env python

import rospy
from sem_server_ros.ontology import JenaSempy
from sem_server_ros.srv import CreateSem, ReadSem, UpdateSem, DeleteSem
from sem_server_ros.srv import CreateSemResponse, ReadSemResponse, UpdateSemResponse, DeleteSemResponse
from sem_server_ros.msg import Triple, URI

ontology_server = JenaSempy()

def create_semantics(req):
    print("Creating {} {} {}".format(req.subject, req.predicate, req.object))
    new_triple = ontology_server.create(req.subject, req.predicate, req.object)
    result = CreateSemResponse()
    result.feedback = create_triple(req.subject.value, new_triple[0][0], new_triple[0][1])
    return result

def read_semantics(req):
    print("Reading.....")
    onto = ontology_server.read(req.subject)
    triples = [create_triple(req.subject, predicate, object) for predicate, object in onto]
    response = ReadSemResponse(triples)
    return response

def update_semantics(req):
    pass

def delete_semantics(req):
    pass


def create_triple(subject, predicate, object):
    result = Triple()
    result.subject   = create_uri(subject)
    result.predicate = create_uri(predicate)
    result.object    = create_uri(object)
    return result

def create_uri(value):
    result = URI()
    result.value = value
    return result


if __name__ == "__main__":
    rospy.init_node('sem_server_node')
    s = rospy.Service('create_sem', CreateSem, create_semantics)
    s = rospy.Service('read_sem', ReadSem, read_semantics)
    s = rospy.Service('update_sem', UpdateSem, update_semantics)
    s = rospy.Service('delete_sem', DeleteSem, delete_semantics)
    print "Ready to interact with the knowledge base."
    rospy.spin()
