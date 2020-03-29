import os

class QueryTemplateEngine:

    def __init__(self, file):
        mod = os.path.dirname(file)
        src_dir = os.path.dirname(mod)
        package = os.path.dirname(src_dir)
        self.query_folder = os.path.join(package, "sparql")

    def load_template(self, file):
        with open(os.path.join(self.query_folder, file), 'r') as template:
            return template.read()

    @staticmethod
    def generate(template, parameter):
        return template.replace("*", parameter)
