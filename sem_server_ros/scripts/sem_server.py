#!/usr/bin/env python

import rospy

from sem_server_ros.srv import Select, Describe, Ask, Update, CreateInstance, GenerateInstanceName
from sem_server_ros.srv import SelectResponse, DescribeResponse, AskResponse, UpdateResponse, CreateInstanceResponse, GenerateInstanceNameResponse
from sem_server_ros.server_com import ROSFusekiServer
from sem_server_ros.msg import Triple

endpoint = ROSFusekiServer()

def generate_instance_uri(req):
    uri = endpoint.generate_instance_uri(req.class_name)
    return GenerateInstanceNameResponse(uri)

def create_instance(req):
    result = endpoint.create_instance(req.class_name)
    return CreateInstanceResponse(result)

def select_data(req):
    ''' Returns variables bound in a query pattern match
    sem_server_ros/Triple where
    ---
    sem_server_ros/Triple[] match

    '''
    subject =   "ns:"+req.where.subject if req.where.subject else "?s"
    predicate = "rdf:"+req.where.predicate if req.where.predicate else "?p"
    object =    "ns:"+req.where.object if req.where.object else "?o"

    list_triples = endpoint.select_data(Triple(subject, predicate, object))

    resp=[]
    for triple in list_triples:
            new_triple=Triple()
            new_triple.subject= triple["s"]["value"].encode("ascii") if 's' in triple else ""
            new_triple.predicate = triple["o"]["value"].encode("ascii") if 'o' in triple else ""
            new_triple.object = triple["p"]["value"].encode("ascii") if 'p' in triple else ""
            resp.append(new_triple)
    return SelectResponse(resp)


def add_data(req):
    ''' DELETE or INSERT operations
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    subject =   "ns:"+req.update_triple.subject if req.update_triple.subject else "?s"
    predicate = "ns:"+req.update_triple.predicate if req.update_triple.predicate else "?p"
    object =    "ns:"+req.update_triple.object if req.update_triple.object else "?o"
    success = endpoint.add_data(Triple(subject, predicate, object))
    return UpdateResponse(True)

def remove_data(req):
    '''
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    subject =   "ns:"+req.update_triple.subject if req.update_triple.subject else "?s"
    predicate = "ns:"+req.update_triple.predicate if req.update_triple.predicate else "?p"
    object =    "ns:"+req.update_triple.object if req.update_triple.object else "?o"
    success = endpoint.remove_data(Triple(subject, predicate, object))
    return UpdateResponse(success)

def test_data(req):
    ''' Returns a boolean indicating whether a query pattern matches or not.
    sem_server_ros/Triple triple
    ---
    bool veracity
    '''
    subject =   "ns:"+req.triple.subject if req.triple.subject else "?s"
    predicate = "ns:"+req.triple.predicate if req.triple.predicate else "?p"
    object =    "ns:"+req.triple.object if req.triple.object else "?o"
    print(Triple(subject, predicate, object))
    found = endpoint.test_data(Triple(subject, predicate, object))
    return AskResponse(found)

def read_data(req):
    '''  Returns an RDF graph that describes the resources found.
    sem_server_ros/Triple where
    ---
    sem_server_ros/Triple[] rdf_description
    '''
    subject =   "ns:"+req.where.subject if req.where.subject else "?s"
    predicate = "ns:"+req.where.predicate if req.where.predicate else "?p"
    object =    "ns:"+req.where.object if req.where.object else "?o"
    list_triples = endpoint.read_data(Triple(subject, predicate, object))
    resp = [Triple(subject, predicate, object) for subject, predicate, object in list_triples]
    return DescribeResponse(resp)

if __name__ == "__main__":
    rospy.init_node('sem_server_node')
    s = rospy.Service('add_data',    Update,   add_data)
    s = rospy.Service('select_data', Select,   select_data)
    s = rospy.Service('read_data',   Describe, read_data)
    s = rospy.Service('test_data',   Ask,      test_data)
    s = rospy.Service('remove_data', Update,   remove_data)
    s = rospy.Service('create_instance',   CreateInstance,      create_instance)
    s = rospy.Service('generate_instance_uri', GenerateInstanceName,   generate_instance_uri)
    rospy.spin()
