#include "BlueprintAST.h"

namespace Kestrel::VM {

const ASTPin* ASTNode::FindPin(const std::string& pinName, EPinDirection dir) const {
    for (const auto& pin : pins) {
        if (pin.name == pinName) {
            if (dir == EPinDirection::Unknown || pin.direction == dir) {
                return &pin;
            }
        }
    }
    return nullptr;
}

ASTPin* ASTNode::FindPinMutable(const std::string& pinName, EPinDirection dir) {
    for (auto& pin : pins) {
        if (pin.name == pinName) {
            if (dir == EPinDirection::Unknown || pin.direction == dir) {
                return &pin;
            }
        }
    }
    return nullptr;
}

std::vector<const ASTPin*> ASTNode::GetExecOutputs() const {
    std::vector<const ASTPin*> execs;
    for (const auto& pin : pins) {
        if (pin.direction == EPinDirection::Output && pin.category == EPinCategory::Exec) {
            execs.push_back(&pin);
        }
    }
    return execs;
}

void BlueprintGraphAST::AddNode(std::unique_ptr<ASTNode> node) {
    if (!node) return;
    std::string name = node->name;
    nodeLookup[name] = node.get();
    nodes.push_back(std::move(node));
}

const ASTNode* BlueprintGraphAST::FindNode(const std::string& nodeName) const {
    auto it = nodeLookup.find(nodeName);
    if (it != nodeLookup.end()) {
        return it->second;
    }
    return nullptr;
}

ASTNode* BlueprintGraphAST::FindNodeMutable(const std::string& nodeName) {
    auto it = nodeLookup.find(nodeName);
    if (it != nodeLookup.end()) {
        return it->second;
    }
    return nullptr;
}

size_t BlueprintGraphAST::CountTotalPins() const {
    size_t count = 0;
    for (const auto& n : nodes) {
        count += n->pins.size();
    }
    return count;
}

size_t BlueprintGraphAST::CountTotalConnections() const {
    size_t count = 0;
    for (const auto& n : nodes) {
        for (const auto& p : n->pins) {
            count += p.linkedTo.size();
        }
    }
    // Each link connects two pins, so divide by 2
    return count / 2;
}

std::vector<const ASTNode*> BlueprintGraphAST::FindNodesByClass(const std::string& classSubstr) const {
    std::vector<const ASTNode*> matches;
    for (const auto& n : nodes) {
        if (n->className.find(classSubstr) != std::string::npos) {
            matches.push_back(n.get());
        }
    }
    return matches;
}

} // namespace Kestrel::VM
