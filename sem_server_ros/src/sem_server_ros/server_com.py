from jena_com.communication import FusekiServer


class FusekiEndpoint:

    def __init__(self):
        self.jena_fuseki_server = FusekiServer()

    def generate_instance_uri(self, class_name):
        try:
            return self.jena_fuseki_server.generate_instance_uri(class_name)
        except Exception as e:
            pass

    def create_instance(self, class_name):
        try:
            name = self.generate_instance_uri(class_name)
            self.jena_fuseki_server.create_instance(class_name)
            return name
        except Exception as e:
            pass

    def select_data(self, *args):
        ''' Returns variables bound in a query pattern match   '''
        try:
            if len(args) == 1:
                query = args[0]
            else:
                query = """
                    SELECT *
                    WHERE {
                        """ +  args[0] + """ """ +  args[1] + """ """ + args[2] + """ .
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
            print(e)

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

    def test_data(self, *args):
        try:
            if len(args) == 1:
                query = args[0]
            else:
                query = """
                    ASK {
                        """ +  args[0] + """ """ +  args[1] + """ """ + args[2] + """ .
                    }
                """
            return self.jena_fuseki_server.ask_operation(query)
        except Exception as e:
            pass

    def read_data(self, *args):
        try:
            if len(args) == 1:
                query = args[0]
            else:
                describe = []
                if triple.subject == "?s":
                    describe.append(subject)
                if triple.predicate == "?p":
                    describe.append(predicate)
                if triple.object == "?o":
                    describe.append(object)

                query = """
                    DESCRIBE """ + " ".join(describe) + """
                    WHERE {
                        """ + triple.subject + """ """ + triple.predicate + """ """ + triple.object + """ .
                    }
                """
            return self.jena_fuseki_server.describe_operation(query)
        except Exception as e:
            pass
