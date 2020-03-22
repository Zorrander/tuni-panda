"""
Define a set of queries to retrieve information from an RDF Triple store.
"""


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
