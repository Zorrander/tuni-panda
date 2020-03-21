import json
from rdflib import URIRef, BNode, Literal, Namespace, RDF
from jena_com.communication import FusekiServer
from jena_com import queries as qry

class Knowledge:

    def __init__(self):
        self.cogtuni_ns = Namespace("http://cognitive.robotics.tut#")
        self.server = Server()
        self.shape_property = self.cogtuni_ns["hasShape"]
        self.size_property  = self.cogtuni_ns["hasSize"]
        self.link_property  = self.cogtuni_ns["isLinkedTo"]

        self.size_values  = [self.cogtuni_ns["big"], self.cogtuni_ns["small"]]
        self.shape_values = [self.cogtuni_ns["circle"], self.cogtuni_ns["square"]]

    def add_object(self, json_data):
        ''' Receives JSON data from the webapp and translate in terms of rdf triples as an assembly part. '''
        list_triples = []
        parsed_json = (json.loads(json_data))
        for object in parsed_json["parts"]:
            object_name = object["name"]
            for key, value in object.items():
                if key != "name":
                        key_ns = self.server.find_namespace(key)
                        value_ns = self.server.find_namespace(value)
                        key_ns = Namespace(key_ns+"#") if key_ns else self.cogtuni_ns
                        value_ns = Namespace(value_ns+"#") if value_ns else self.cogtuni_ns
                        list_triples.append( (self.cogtuni_ns[object_name], key_ns[key], value_ns[value]) )
        for triple in list_triples:
            self.server.create(triple[0], triple[1], triple[2])
        return list_triples

    def retrieve_assembly_steps(self, skill):
        ''' Given the name of a skill retrieves the list of steps and constraints associated with it. '''
        steps = self.server.query(qry.select_assembly_steps(skill))
        return steps

    def retrieve_links(self, step_name):
        ''' Given a list of parts and links, isolate independant groups of tasks. '''
        constraints = [x[0].toPython() for x in self.server.query(qry.select_links(step_name))]
        return constraints

    def retrieve_type(self, part):
        '''  Given a list of parts and links, identifies a sequence order. '''
        type = [x[0].toPython() for x in self.server.query(qry.select_type(part))]
        type = [x for x in type if x != 'NamedIndividual']
        if len(type) == 1:
            type = type[0]
        return type

    def deduce_assembly_logic(self, assembly):
        ''' Given objects decides which one is inserted in which '''
        if len(assembly) == 2:  # Just insert peg in hole
            print("assembly: {}".format(assembly))
            type1 = self.retrieve_type(assembly[0])
            type2 = self.retrieve_type(assembly[1])
            if type1 == 'Peg' and type2 == 'Hole':
                return (assembly[0], assembly[1])
            elif type2 == 'Peg' and type1 == 'Hole':
                return (assembly[1], assembly[0])
            else:
                return((assembly[0], assembly[1]),None)
        else:
            hole = None
            for part in assembly:
                if self.retrieve_type(part) == 'Hole':
                    hole = part
                    break
            edges = []
            for part in assembly:
                if part != hole:
                    edges.append((part, hole))
            return edges
