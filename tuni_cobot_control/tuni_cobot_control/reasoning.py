from owlready2 import *

onto = get_ontology("file:///home/alex/handover.owl").load()

with onto:
    class Command(Thing):pass
    class has_action(Command >> str, FunctionalProperty):pass
    class ReceivedHandoverCommand(Thing):pass
    class isMet(ReceivedHandoverCommand >> bool, FunctionalProperty):pass

    rule = Imp()
    rule.set_as_rule("""Command(?c), has_action(?c, 'give'), ReceivedHandoverCommand(?cond) -> isMet(?cond, true)""")
