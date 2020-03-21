from jena_com.communication import FusekiServer


class ROSFusekiServer:

    def __init__(self):
        self.jena_fuseki_server = FusekiServer()

    def select_data(self, triple):
        ''' Returns variables bound in a query pattern match   '''
        try:
            query = """
                PREFIX ns: <http://www.example.org/ns#>
                PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
                SELECT *
                WHERE {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            return self.jena_fuseki_server.select_operation(query)
        except Exception as e:
            pass

    def add_data(self, triple):
        try:
            query ="""
                PREFIX ns: <http://www.example.org/ns#>
                INSERT DATA {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            return self.jena_fuseki_server.update_operation(query)
        except Exception as e:
            pass

    def remove_data(self, triple):
        try:
            query ="""
                PREFIX ns: <http://www.example.org/ns#>
                DELETE DATA {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            return self.jena_fuseki_server.update_operation(query)
        except Exception as e:
            pass

    def test_data(self, triple):
        try:
            print(triple)
            query = """
                PREFIX ns: <http://www.example.org/ns#>
                ASK {
                  """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """

            return self.jena_fuseki_server.ask_operation(query)
        except Exception as e:
            pass

    def read_data(self, triple):
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
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            return self.jena_fuseki_server.describe_operation(query)
        except Exception as e:
            pass
