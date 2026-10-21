"""Independent, idempotent demand tokens for static and dynamic consumers."""
from rsim.core.demand import DemandSnapshot


class DemandTracker:
    """Mutated only on the owning event loop; snapshots are safe to hand to threads."""
    def __init__(self):
        self.generation = 0
        self._tokens = {}

    def acquire(self, port, token):
        port = port._resolved()
        tokens = self._tokens.setdefault(port, set())
        if token not in tokens:
            tokens.add(token)
            self.generation += 1

    def release(self, port, token):
        port = port._resolved()
        tokens = self._tokens.get(port, set())
        if token in tokens:
            tokens.remove(token)
            self.generation += 1
            if not tokens:
                self._tokens.pop(port, None)

    def snapshot(self, component):
        ports = (*component.outputs.values(), *component.sinks.values())
        return DemandSnapshot(self.generation, frozenset(port.name for port in ports
                                                        if port._resolved() in self._tokens))

    def count(self, port):
        return len(self._tokens.get(port._resolved(), ()))

    def clear(self):
        if self._tokens:
            self._tokens.clear()
            self.generation += 1
