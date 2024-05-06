"""
Unreal Engine 5 Blueprint Text Graph Builder.
Generates production-grade Unreal Engine Blueprint text graphs adhering to
standard Unreal Engine K2Node clipboard and serialization specifications.
"""

import uuid

def make_guid():
    return uuid.uuid4().hex.upper()

class BPPin:
    def __init__(self, name, direction, category, subcategory="", default_value=None, is_array=False):
        self.pin_id = make_guid()
        self.name = name
        self.direction = direction # 'EGPD_Input' or 'EGPD_Output'
        self.category = category   # 'exec', 'real', 'struct', 'bool', 'int', 'object', 'string', 'byte'
        self.subcategory = subcategory
        self.default_value = default_value
        self.is_array = is_array
        self.linked_to = [] # list of (node_name, pin_id)

    def connect(self, other_node_name, other_pin_id):
        self.linked_to.append((other_node_name, other_pin_id))

    def to_text(self):
        parts = [f'PinId={self.pin_id}']
        parts.append(f'PinName="{self.name}"')
        parts.append(f'Direction="{self.direction}"')
        parts.append(f'PinType.PinCategory="{self.category}"')
        if self.subcategory:
            parts.append(f'PinType.PinSubCategoryObject={self.subcategory}')
        if self.is_array:
            parts.append('PinType.ContainerType=Array')
        if self.default_value is not None:
            parts.append(f'DefaultValue="{self.default_value}"')
        if self.linked_to:
            links = ",".join([f"{node} {pid}" for node, pid in self.linked_to])
            parts.append(f'LinkedTo=({links},)')
        return f'   CustomProperties Pin ({",".join(parts)})'

class BPNode:
    def __init__(self, class_name, name, pos_x=0, pos_y=0, comment=""):
        self.class_name = class_name
        self.name = name
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.comment = comment.replace("\u2014", "-").replace("\u2013", "-").replace("--", "-") # Strict zero em dash guarantee
        self.guid = make_guid()
        self.pins = []
        self.properties = {}

    def add_pin(self, pin):
        self.pins.append(pin)
        return pin

    def find_pin(self, name, direction=None):
        for p in self.pins:
            if p.name == name:
                if direction is None or p.direction == direction:
                    return p
        return None

    def to_text(self):
        lines = []
        lines.append(f'Begin Object Class={self.class_name} Name="{self.name}"')
        if self.comment:
            clean_comment = self.comment.replace('"', "'")
            lines.append(f'   NodeComment="{clean_comment}"')
        lines.append(f'   NodePosX={self.pos_x}')
        lines.append(f'   NodePosY={self.pos_y}')
        lines.append(f'   NodeGuid={self.guid}')
        for k, v in self.properties.items():
            lines.append(f'   {k}={v}')
        for pin in self.pins:
            lines.append(pin.to_text())
        lines.append('End Object')
        return "\n".join(lines)

class BPGraph:
    def __init__(self, graph_name, blueprint_class):
        self.graph_name = graph_name
        self.blueprint_class = blueprint_class
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)
        return node

    def connect(self, src_node, src_pin_name, dst_node, dst_pin_name):
        src_pin = src_node.find_pin(src_pin_name, 'EGPD_Output')
        dst_pin = dst_node.find_pin(dst_pin_name, 'EGPD_Input')
        if src_pin and dst_pin:
            src_pin.connect(dst_node.name, dst_pin.pin_id)
            dst_pin.connect(src_node.name, src_pin.pin_id)

    def to_text(self):
        header = [
            f'; -----------------------------------------------------------------------------',
            f'; Unreal Engine 5 Blueprint Export: {self.blueprint_class}',
            f'; Graph: {self.graph_name}',
            f'; Generated for Kestrel High-Performance Autonomous Aerial Robotics System',
            f'; -----------------------------------------------------------------------------\n'
        ]
        body = [node.to_text() for node in self.nodes]
        return "\n".join(header) + "\n\n".join(body) + "\n"
