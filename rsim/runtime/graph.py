"""Resolve component graphs and own their lifecycle."""
import asyncio
import uuid
from rsim.core.component import Component, Reference
from rsim.core.errors import ComponentError, CleanupFailed, PlatformUnavailable, StartupFailed
from rsim.platforms.errors import CapabilityUnavailable
from .options import RuntimeOptions
from .supervision import Lifecycle, State
from .demand import DemandTracker

class Runtime:
    """Own Components required by requested Components, Signals or sinks.

    Ownership DAGs determine lifetime order. Signal producers are discovered
    independently, so data feedback does not create an ownership cycle.
    """
    def __init__(self, *roots, placement=None, options=None, _root_ports=True):
        self.options = options or RuntimeOptions()
        self.lifecycle = Lifecycle('runtime')
        self.demand = DemandTracker()
        self._cleanup_task = None
        self._close_pending = {}
        self.accepting = False
        self.roots = self._original_roots = roots
        self.placement = placement or {}
        self._order, self._started, self._aliases = [], [], []
        self._original_dependencies = {}
        self._active = False
        self._binding_plan = None
        self._command_claims = {}
        self._root_ports = _root_ports
        self.demanded_ports = {}

    def _release_bindings(self):
        for alias in self._aliases:
            alias._canonical = None
        self._aliases.clear()
        for component, dependencies in self._original_dependencies.items():
            component.dependencies = dependencies
            component._runtime_peers = ()
        self._original_dependencies.clear()
        self._command_claims.clear()
        self.roots = self._original_roots

    def _resolve(self):
        keyed, components, data_edges = {}, {}, {}
        self._order = []
        self._command_claims.clear()

        def visit(component):
            if not isinstance(component, Component):
                raise TypeError("Runtime roots must be Components, Signals or CommandSinks")
            if component._binding is not None:
                return visit(component._binding)
            if component._canonical is not None and component in self._aliases:
                return visit(component._canonical)
            if component.key is not None:
                prior = keyed.setdefault(component.key, component)
                if prior.configuration() != component.configuration():
                    raise ValueError(f"conflicting source configuration: {component.key}")
                if prior is not component:
                    if component._runtime is not None or (component._canonical is not None
                                                          and component not in self._aliases):
                        raise ComponentError("component alias already belongs to an active runtime")
                    if component not in self._aliases:
                        self._aliases.append(component)
                    component._canonical = prior
                component = prior
            if component in components:
                return component
            if component._runtime is not None:
                raise ComponentError("component already belongs to an active runtime")
            components[component] = None
            self._original_dependencies.setdefault(component, component.dependencies)
            component.dependencies = tuple(visit(child) for child in component.dependencies)
            data_edges[component] = tuple(visit(port.producer) for port in
                                          component.inputs + component.command_targets)
            return component

        root_components = []
        for root in self.roots:
            owner = root if isinstance(root, Component) else getattr(root, "producer", None)
            root_components.append(visit(owner))

        visiting, visited = set(), set()
        def check_ownership(component):
            if component in visiting:
                raise ValueError("cyclic ownership dependency; use Signal inputs for feedback")
            if component in visited:
                return
            visiting.add(component)
            for child in component.dependencies:
                check_ownership(child)
            visiting.remove(component)
            visited.add(component)
        for component in components:
            check_ownership(component)

        visiting, visited = set(), set()
        def order(component):
            if component in visited or component in visiting:
                return
            visiting.add(component)
            for child in component.dependencies + data_edges[component]:
                order(child)
            visiting.remove(component)
            visited.add(component)
            self._order.append(component)
        for component in root_components:
            order(component)

        # Data cycles can contradict preferred input-first order. Hard resource
        # dependencies always take precedence; ready data producers are a tie
        # breaker, never a reason to open an owner before its dependency.
        remaining, self._order = self._order, []
        started = set()
        while remaining:
            ready = [item for item in remaining if set(item.dependencies) <= started]
            component = next((item for item in ready if set(data_edges[item]) <= started), ready[0])
            self._order.append(component)
            started.add(component)
            remaining.remove(component)

        for component in self._order:
            peers = component.dependencies + data_edges[component]
            if isinstance(component, Reference):
                target = component.target._canonical or component.target
                if target not in components or isinstance(target, Reference):
                    raise ComponentError("Reference target must be an owned component in this Runtime")
                peers += (target,)
            component._runtime_peers = tuple(peer for peer in peers if peer is not component)
            for sink in component.command_targets:
                sink = sink._resolved()
                if sink in self._command_claims and self._command_claims[sink] is not component:
                    raise ValueError("multiple command writers require an explicit CommandMux")
                self._command_claims[sink] = component

        # Destination is a logical consumer, or None for the calling process.
        # Ownership edges activate resources but never request their outputs.
        self.demanded_ports = {}
        def demand(port, destination):
            self.demanded_ports.setdefault(port._resolved(), set()).add(destination)
        if self._root_ports:
            for root in self.roots:
                if isinstance(root, Component):
                    owner = root._binding or root._canonical or root
                    for output in owner.outputs.values():
                        demand(output, None)
                else:
                    demand(root, None)
        for component in self._order:
            for port in component.inputs + component.command_targets:
                demand(port, component)

    async def __aenter__(self):
        if self._active or self._started:
            raise ComponentError("runtime already active")
        self._cleanup_task = None
        self.lifecycle.failure = None
        self.lifecycle.transition(State.STARTING)
        try:
            self.options = self.options.resolved()
            deadline = asyncio.get_running_loop().time() + self.options.startup_timeout_s
            self._resolve()
            if self.placement or any(item.placement is not None for item in self._order):
                from .deployment import BindingPlan
                self._binding_plan = BindingPlan(self)
                self._binding_plan.prepare()
                self._resolve()
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError('graph preparation exceeded startup budget')
        except BaseException as error:
            if self._binding_plan is not None:
                self._binding_plan.release()
                self._binding_plan = None
            self._release_bindings()
            self.lifecycle.transition(State.FAILED, error)
            if isinstance(error, CapabilityUnavailable):
                raise PlatformUnavailable(str(error), phase=error.operation) from error
            if isinstance(error, TimeoutError):
                raise StartupFailed('runtime startup timed out', phase='preparing') from error
            raise
        self._active = True
        self.accepting = True
        self.demand.clear()
        for port, consumers in self.demanded_ports.items():
            for consumer in consumers:
                self.demand.acquire(port, ('static', consumer))
        try:
            # All outputs become readable before open callbacks run, so seeded
            # feedback and consumers waiting on later producers are possible.
            for component in self._order:
                component._runtime = self
                component._closed = False
                component._closing = False
                component._failure = None
                component._instance_id = uuid.uuid4().hex
                component._ready.clear()
                component._failed.clear()
                for output in component.outputs.values():
                    output._history.clear()
                    output.history_evictions = 0
                    output._publication_sequence = 0
                    output._port_error = None
            for component in self._order:
                self._started.append(component)
                async with asyncio.timeout_at(deadline):
                    await component.open()
                    for sink in component.sinks.values():
                        await sink._activate()
                component._ready.set()
                component._tasks.add(asyncio.create_task(
                    component._watchdog(), name=f"{type(component).__name__}:WatchDog"))
        except BaseException as error:
            self.lifecycle.transition(State.FAILED, error)
            try:
                await self.aclose()
            except Exception as cleanup:
                error.add_note(f'cleanup also failed: {cleanup}')
            if isinstance(error, CapabilityUnavailable):
                raise PlatformUnavailable(str(error), phase=error.operation) from error
            if isinstance(error, TimeoutError):
                raise StartupFailed('runtime startup timed out', phase='initializing') from error
            raise
        self.lifecycle.transition(State.RUNNING)
        return self

    async def wait(self):
        if not self._active:
            raise ComponentError("runtime is not active")
        pending = [asyncio.create_task(component.wait()) for component in self._order]
        if not pending:
            raise ComponentError("runtime has no components")
        try:
            done, _ = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
            try:
                await next(iter(done))
            except Exception as error:
                self.lifecycle.transition(State.FAILED, error)
                raise
        finally:
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)

    def diagnostics(self):
        from .diagnostics import runtime_report
        return runtime_report(self)

    async def aclose(self):
        if self._cleanup_task is None or (self._cleanup_task.done() and self._started):
            self._cleanup_task = asyncio.create_task(self._close(), name='rsim-runtime-close')
        try:
            await asyncio.shield(self._cleanup_task)
        except asyncio.CancelledError:
            # Resource cleanup completes independently; never swallow cancellation.
            try:
                await self._cleanup_task
            except Exception as error:
                self.lifecycle.cleanup.append(dict(error=str(error), operation='cancelled-close'))
            raise

    async def _close(self):
        if not self._started and not self._active:
            if self.lifecycle.state == State.NEW:
                self.lifecycle.transition(State.CLOSED)
            return
        self.lifecycle.transition(State.STOPPING)
        errors = []
        unfinished = []
        # Public writes stop immediately; owners can still send their fallback
        # through dependencies until those dependencies reach teardown.
        self.accepting = False
        for component in reversed(self._started):
            component._closing = True
            component._ready.set()
            component._failed.set()
            for sink in component.sinks.values():
                try:
                    async with asyncio.timeout(self.options.shutdown.grace_s):
                        await sink._deactivate()
                except Exception as error:
                    errors.append(error)
            for task in component._tasks:
                task.cancel()
            if component._tasks:
                done, pending = await asyncio.wait(component._tasks, timeout=self.options.shutdown.grace_s)
                await asyncio.gather(*done, return_exceptions=True)
                component._tasks = pending
            component._fail_requests()
            component._services.clear()
            closed = False
            try:
                task = self._close_pending.get(component)
                if task is None:
                    task = asyncio.create_task(component.close(), name=f'{type(component).__name__}:close')
                    self._close_pending[component] = task
                budget = self.options.shutdown.grace_s + self.options.shutdown.kill_wait_s + 3
                done, _ = await asyncio.wait((task,), timeout=budget)
                if not done or component._tasks:
                    unfinished.append(component)
                    raise CleanupFailed(f'{type(component).__name__} still has running resources; '
                                        'use ProcessPlacement for forcible native cleanup', phase='close',
                                        instance_id=component._instance_id)
                await task
                closed = True
                self._close_pending.pop(component, None)
            except Exception as error:
                errors.append(error)
                if component not in unfinished:
                    unfinished.append(component)
                if task.done():
                    self._close_pending.pop(component, None)
            finally:
                if closed or component not in unfinished:
                    component._closed = True
                    component._runtime = None
                for output in component.outputs.values():
                    await output._notify()
                    output._history.clear()
        # An open callback may fail before later components are started.
        for component in self._order:
            if component in unfinished:
                continue
            component._closed = True
            component._runtime = None
            component._failed.set()
            for output in component.outputs.values():
                await output._notify()
        self._started = list(reversed(unfinished))
        self.demand.clear()
        self._active = False
        if self._binding_plan is not None and not unfinished:
            self._binding_plan.release()
            self._binding_plan = None
        if not unfinished:
            self._release_bindings()
        if errors:
            self.lifecycle.cleanup.extend(dict(operation='close', error=f'{type(error).__name__}: {error}')
                                          for error in errors)
            self.lifecycle.transition(State.FAILED, errors[0])
            raise ExceptionGroup("component cleanup failed", errors)
        self.lifecycle.transition(State.FAILED if self.lifecycle.failure else State.CLOSED)

    async def __aexit__(self, exc_type, exc, tb):
        cleanup = asyncio.create_task(self.aclose())
        try:
            await asyncio.shield(cleanup)
        except asyncio.CancelledError:
            await cleanup
            raise
        except Exception as error:
            if isinstance(exc, asyncio.CancelledError):
                exc.add_note(f'Runtime cleanup also failed: {error}')
                raise exc.with_traceback(tb)
            if exc is not None:
                raise BaseExceptionGroup('Runtime body and cleanup failed', [exc, error]) from None
            raise
