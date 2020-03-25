from jena_com.communication import FusekiServer


class ROSFusekiServer:

    def __init__(self):
        self.jena_fuseki_server = FusekiServer()

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
            return self.jena_fuseki_server.describe_operation(query)
        except Exception as e:
            pass
