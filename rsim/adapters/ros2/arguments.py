"""Native driver arguments; importing this module does not import ROS."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
from pathlib import Path


def parameter_text(value):
    """JSON scalars/arrays are YAML too, and preserve quoted string values."""
    if isinstance(value, (list, tuple)):
        if value and (type(value[0]) not in (bool, int, float, str)
                        or any(type(item) is not type(value[0])
                                for item in value)):
            raise TypeError(
                "ROS parameter arrays must contain one scalar type"
                )
        for item in value:
            parameter_text(item)
    elif type(value) not in (bool, int, float, str):
        raise TypeError(
            "ROS parameters must be bool, int, float, str or homogeneous arrays"
            )
    if type(value) is int and not -(2**63) <= value < 2**63:
        raise ValueError("ROS integer parameters must fit in int64")
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


class RosArguments:

    def __init__(self, parameters=None, ros_args=None, remappings=None):
        self.parameters = deepcopy(dict(parameters or {}))
        for name, value in self.parameters.items():
            if not isinstance(name, str) or not name or ':=' in name:
                raise ValueError(
                    "ROS parameter names must be nonempty strings without ':='"
                    )
            parameter_text(value)
        if isinstance(ros_args, (str, bytes)):
            raise TypeError(
                "ros_args must be a sequence of argv strings, not a shell command"
                )
        args = list(ros_args or ())
        if any(not isinstance(arg, str) for arg in args):
            raise TypeError("ros_args must contain strings")
        if args and args[0] != "--ros-args":
            args.insert(0, "--ros-args")
        self.files = {}
        in_ros = False
        for index, arg in enumerate(args):
            if arg == "--ros-args":
                in_ros = True
            elif arg == "--":
                in_ros = False
            elif in_ros and arg == "--params-file":
                if index + 1 == len(args):
                    raise ValueError("--params-file requires a path")
                path = Path(args[index + 1]
                           ).expanduser().resolve(strict=True)
                args[index + 1] = str(path)
                self.files[str(path)] = hashlib.sha256(path.read_bytes()
                                                      ).hexdigest()
        self.ros_args = tuple(args)
        self.remappings = dict(remappings or {})

    def arguments(self):
        # A factory describes a fixed recipe even when it starts much later.
        for path, digest in self.files.items():
            if hashlib.sha256(Path(path).read_bytes()
                             ).hexdigest() != digest:
                raise ValueError(
                    f"ROS params file changed; recreate the provider: {path}"
                    )
        args = ["--ros-args"]
        for name, value in self.parameters.items():
            args.extend(("-p", f"{name}:={parameter_text(value)}"))
        args.append("--")
        args.extend(self.ros_args)
        if self.ros_args and self.ros_args[-1] != "--":
            args.append("--")
        # ROS parameters use the last assignment, remaps the first match.
        # Put fallback remaps after user arguments so both override correctly.
        args.append("--ros-args")
        for name, value in self.remappings.items():
            args.extend(("-r", f"{name}:={value}"))
        return args

    def signature(self):
        return json.dumps({
            "parameters": self.parameters,
            "ros_args": self.ros_args,
            "remappings": self.remappings,
            "files": self.files
            },
                            sort_keys=True,
                            ensure_ascii=False)

    @contextmanager
    def resolve(self, node_name, namespace="/"):
        """Use rcl itself for YAML, node-scoped overrides and topic remapping.

        The temporary node has no parameter services and is destroyed before
        returning a provider. No ROS handles enter its serialized factory.
        """
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions

        context, node = Context(), None
        try:
            rclpy.init(
                args=[],
                context=context,
                signal_handler_options=SignalHandlerOptions.NO
                )
            node = rclpy.create_node(
                node_name,
                namespace=namespace,
                context=context,
                cli_args=self.arguments(),
                use_global_arguments=False,
                enable_rosout=False,
                start_parameter_services=False,
                automatically_declare_parameters_from_overrides=True
                )
            yield node
        finally:
            if node is not None:
                node.destroy_node()
            context.try_shutdown()


def effective_settings(options, node_name, namespace, topics):
    """Return effective values and resolved subscriptions without retaining ROS."""
    if options.ros_args:
        with options.resolve(node_name, namespace) as node:
            values = {
                name: node.get_parameter(name).value
                for name in node.list_parameters([], depth=0).names
                }
            resolved = {
                name: node.resolve_topic_name(topic)
                for name, topic in topics(values).items()
                }
    else:
        values = deepcopy(options.parameters)
        name = options.remappings.get("__node", node_name)
        ns = options.remappings.get("__ns", namespace).rstrip("/")
        resolved = {}
        for stream, topic in topics(values).items():
            if topic.startswith("~/"):
                topic = f"{ns}/{name}/{topic[2:]}"
            elif not topic.startswith("/"):
                topic = f"{ns}/{topic}"
            resolved[stream] = topic
    return values, resolved
