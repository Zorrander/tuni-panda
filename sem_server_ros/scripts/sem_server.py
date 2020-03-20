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
        pass
        # list_triples = self.jena_fuseki_server.select_operation()
    except Exception as e:
        pass

def add_data(req):
    ''' DELETE or INSERT operations
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    try:
        pass
        # self.jena_fuseki_server.update_operation()
    except Exception as e:
        pass

def remove_data(req):
    '''
    sem_server_ros/Triple update_triple
    ---
    bool success
    '''
    try:
        pass
        # self.jena_fuseki_server.update_operation()
    except Exception as e:
        pass

def test_data(req):
    ''' Returns a boolean indicating whether a query pattern matches or not.
    sem_server_ros/Triple triple
    ---
    bool veracity
    '''
    print(req)
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
        print(query)
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
    # s = rospy.Service('add_data',    Update,   add_data)
    # s = rospy.Service('select_data', Select,   select_data)
    s = rospy.Service('read_data',   Describe, read_data)
    s = rospy.Service('test_data',   Ask,      test_data)
    # s = rospy.Service('remove_data', Update,   remove_data)
    print "Ready to interact with the knowledge base."
    rospy.spin()
