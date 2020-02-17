from jena_com.communication import Server
from sem_server_ros.msg import Triple, URI

class JenaSempy:

    def __init__(self):
        self.wrapper = Server()

    def fetch_all(self):
        return self.wrapper.fetch_all()

    def create(self, subject, predicate, object):
        self.wrapper.create(subject.namespace+"#"+subject.value , object.namespace+"#"+object.value, predicate.namespace+"#"+predicate.value)
        return self.wrapper.read(subject)

    def read(self, subject_uri):
        query = self.wrapper.read(subject_uri)
        return self.wrapper.read(subject_uri)
