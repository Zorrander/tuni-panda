from jena_com.communication import FusekiServer


class ROSFusekiServer:

    def __init__(self):
        self.jena_fuseki_server = FusekiServer()

    @staticmethod
    def concatenate_prefix(query):
        return """
                    PREFIX cogrob: <http://onto-server-tuni.herokuapp.com/Panda#>
                    PREFIX rdf:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
                    PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
                    PREFIX owl:  <http://www.w3.org/2002/07/owl#>
                    PREFIX xsd:  <http://www.w3.org/2001/XMLSchema#>
                """ + query

    def generate_instance_uri(self, class_name):
        try:
            return self.jena_fuseki_server.generate_instance_uri(class_name)
        except Exception as e:
            pass


    def create_instance(self, class_name):
        try:
            return self.jena_fuseki_server.create_instance(class_name)
        except Exception as e:
            pass

    def select_data(self, triple):
        ''' Returns variables bound in a query pattern match   '''
        try:
            query = """
                SELECT *
                WHERE {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            query = self.concatenate_prefix(query)
            return self.jena_fuseki_server.select_operation(query)
        except Exception as e:
            pass

    def add_data(self, triple):
        try:
            query ="""
                INSERT DATA {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            query = self.concatenate_prefix(query)
            return self.jena_fuseki_server.update_operation(query)
        except Exception as e:
            pass

    def remove_data(self, triple):
        try:
            query ="""
                DELETE DATA {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            query = self.concatenate_prefix(query)
            return self.jena_fuseki_server.update_operation(query)
        except Exception as e:
            pass

    def test_data(self, triple):
        try:
            query = """
                ASK {
                  """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            query = self.concatenate_prefix(query)
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
                DESCRIBE """ + " ".join(describe) + """
                WHERE {
                    """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                }
            """
            query = self.concatenate_prefix(query)
            return self.jena_fuseki_server.describe_operation(query)
        except Exception as e:
            pass
