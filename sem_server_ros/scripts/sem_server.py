#!/usr/bin/env python

import rospy
from jena_com.communication import FusekiServer
from sem_server_ros.srv import Select, Describe, Ask, Update
from sem_server_ros.srv import SelectResponse, DescribeResponse, AskResponse, UpdateResponse
from sem_server_ros.msg import Triple

jena_fuseki_server = FusekiServer()

def select_data(req):
    ''' Returns variables bound in a query pattern match
    sem_server_ros/Triple where
    ---
    sem_server_ros/Triple[] match

    '''
    try:
        subject =   "ns:"+req.where.subject if req.where.subject else "?s"
        predicate = "rdf:"+req.where.predicate if req.where.predicate else "?p"
        object =    "ns:"+req.where.object if req.where.object else "?o"

        query = """
            PREFIX ns: <http://www.example.org/ns#>
            PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            SELECT *
            WHERE {
                """ + subject + """ """ + predicate + """ """ + object + """ .
            }
        """
        list_triples = jena_fuseki_server.select_operation(query)
        resp=[]
        for triple in list_triples:
                new_triple=Triple()
                new_triple.subject= triple["s"]["value"].encode("ascii") if 's' in triple else ""
                new_triple.predicate = triple["o"]["value"].encode("ascii") if 'o' in triple else ""
                new_triple.object = triple["p"]["value"].encode("ascii") if 'p' in triple else ""
                resp.append(new_triple)
        print(SelectResponse(resp))
        return SelectResponse(resp)
    except Exception as e:
        pass

def add_data(req):
    ''' DELETE or INSERT operations
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    try:
        subject =   "ns:"+req.update_triple.subject if req.update_triple.subject else "?s"
        predicate = "ns:"+req.update_triple.predicate if req.update_triple.predicate else "?p"
        object =    "ns:"+req.update_triple.object if req.update_triple.object else "?o"
        query ="""
            PREFIX ns: <http://www.example.org/ns#>
            INSERT DATA {
                """ + subject + """ """ + predicate + """ """ + object + """ .
            }
        """
        jena_fuseki_server.update_operation(query)
        return UpdateResponse(True)
    except Exception as e:
        pass

def remove_data(req):
    '''
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    try:
        subject =   "ns:"+req.update_triple.subject if req.update_triple.subject else "?s"
        predicate = "ns:"+req.update_triple.predicate if req.update_triple.predicate else "?p"
        object =    "ns:"+req.update_triple.object if req.update_triple.object else "?o"
        query ="""
            PREFIX ns: <http://www.example.org/ns#>
            DELETE DATA {
                """ + subject + """ """ + predicate + """ """ + object + """ .
            }
        """
        jena_fuseki_server.update_operation(query)
        return UpdateResponse(True)
    except Exception as e:
        pass

def test_data(req):
    ''' Returns a boolean indicating whether a query pattern matches or not.
    sem_server_ros/Triple triple
    ---
    bool veracity
    '''
    subject =   "ns:"+req.triple.subject if req.triple.subject else "?s"
    predicate = "ns:"+req.triple.predicate if req.triple.predicate else "?p"
    object =    "ns:"+req.triple.object if req.triple.object else "?o"
    try:
        query = """
            PREFIX ns: <http://www.example.org/ns#>
            ASK {
              """ + subject + """ """ + predicate + """ """ + object + """ .
            }
        """
        found = jena_fuseki_server.ask_operation(query)
        return AskResponse(found)
    except Exception as e:
        pass

def read_data(req):
    '''  Returns an RDF graph that describes the resources found.
    sem_server_ros/Triple where
    ---
    sem_server_ros/Triple[] rdf_description
    '''
    subject =   "ns:"+req.where.subject if req.where.subject else "?s"
    predicate = "ns:"+req.where.predicate if req.where.predicate else "?p"
    object =    "ns:"+req.where.object if req.where.object else "?o"
    describe = []
    if subject == "?s":
        describe.append(subject)
    if predicate == "?p":
        describe.append(predicate)
    if object == "?o":
        describe.append(object)

    try:
        query = """
            PREFIX ns: <http://www.example.org/ns#>
            DESCRIBE """ + " ".join(describe) + """
            WHERE {
                """ + subject + """ """ + predicate + """ """ + object + """ .
            }
        """
        list_triples = jena_fuseki_server.describe_operation(query)
        resp = [Triple(subject, predicate, object) for subject, predicate, object in list_triples]
        return DescribeResponse(resp)
    except Exception as e:
        pass


if __name__ == "__main__":
    rospy.init_node('sem_server_node')
    s = rospy.Service('add_data',    Update,   add_data)
    s = rospy.Service('select_data', Select,   select_data)
    s = rospy.Service('read_data',   Describe, read_data)
    s = rospy.Service('test_data',   Ask,      test_data)
    s = rospy.Service('remove_data', Update,   remove_data)
    print "Ready to interact with the knowledge base."
    rospy.spin()
