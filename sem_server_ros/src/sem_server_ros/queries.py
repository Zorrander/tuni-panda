"""
Define a set of queries to retrieve information from an RDF Triple store.
"""

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

def add_limit(query, limit=25):
    return " LIMIT " + str(limit)

''' Select every triple in the store '''
def select_all():
    query = """
    SELECT ?subject ?predicate ?object
    WHERE {
        ?subject ?predicate ?object
    }
    """
    #query += add_limit(query)
    return query

def select_skill(action, target):
    query = """
        SELECT ?skill
        WHERE {
            ?entity rdfs:subClassOf      cogtuni:Utterance ;
                    cogtuni:isInvokedBy  ?sl1;
                    cogtuni:isFollowedBy ?sl2;
                    cogtuni:activates    ?skill.
            ?sl1 cogtuni:hasValue """+ "'" + action + "'" + """.
            ?sl2 cogtuni:hasValue """ + "'" + target + "'" + """.
        }
    """
    return query

def select_assembly_steps(skill):
    query = """
        SELECT ?s
        WHERE {
        ?step rdf:type cogrobtut:AssembleTask .
        bind( strafter(str(?step), "#") as ?s) . }
    """
    return query

def select_links(step_name):
    query = """
        SELECT ?part
        WHERE {
        OPTIONAL{cogrobtut:""" + step_name + """ cogrobtut:actsOn ?ps. }
        bind( strafter(str(?ps), "#") as ?part) .
        }
    """
    return query

def select_type(part):
    query = """
        SELECT ?type
        WHERE {
            cogrobtut:""" + part + """ rdf:type ?t .
            BIND( strafter(str(?t), "#") as ?type) .
        }
    """
    return query
