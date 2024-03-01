#include "BlueprintValidator.h"
#include <sstream>

namespace Kestrel::VM {

ValidationReport BlueprintValidator::ValidateGraph(const BlueprintGraphAST& ast) {
    ValidationReport report;
    report.totalNodesChecked = ast.GetNodeCount();
    report.totalPinsChecked = ast.CountTotalPins();
    report.totalWiresChecked = ast.CountTotalConnections();

    // 1. Verify all node linkages
    for (const auto& node : ast.nodes) {
        for (const auto& pin : node->pins) {
            for (const auto& [targetNodeName, targetPinId] : pin.linkedTo) {
                const ASTNode* targetNode = ast.FindNode(targetNodeName);
                if (!targetNode) {
                    report.danglingReferences++;
                    std::ostringstream oss;
                    oss << "Node '" << node->name << "' pin '" << pin.name
                        << "' links to non-existent target node '" << targetNodeName << "'.";
                    report.AddError(oss.str());
                    continue;
                }

                // Verify target pin exists
                bool targetPinFound = false;
                for (const auto& tPin : targetNode->pins) {
                    if (tPin.pinId == targetPinId) {
                        targetPinFound = true;
                        // Direction check: Output should link to Input, Input should link to Output
                        if (pin.direction == tPin.direction) {
                            report.typeMismatches++;
                            std::ostringstream oss;
                            oss << "Direction mismatch between node '" << node->name << "' pin '" << pin.name
                                << "' and target node '" << targetNodeName << "' pin '" << tPin.name << "'.";
                            report.AddWarning(oss.str());
                        }

                        // Category check: Exec must link to Exec
                        if (pin.category == EPinCategory::Exec && tPin.category != EPinCategory::Exec) {
                            report.typeMismatches++;
                            std::ostringstream oss;
                            oss << "Category mismatch: exec pin '" << pin.name << "' on node '" << node->name
                                << "' connected to non-exec pin '" << tPin.name << "'.";
                            report.AddError(oss.str());
                        }
                        break;
                    }
                }

                if (!targetPinFound) {
                    report.danglingReferences++;
                    std::ostringstream oss;
                    oss << "Target pin id '" << targetPinId << "' not found on node '" << targetNodeName << "'.";
                    report.AddError(oss.str());
                }
            }
        }
    }

    // 2. Cycle detection on execution flow
    // State: 0 = unvisited, 1 = visiting, 2 = visited
    std::unordered_map<std::string, int> visitState;
    for (const auto& node : ast.nodes) {
        visitState[node->name] = 0;
    }

    for (const auto& node : ast.nodes) {
        if (visitState[node->name] == 0) {
            if (CheckCycleRecursive(node.get(), ast, visitState)) {
                report.executionCyclesDetected++;
                report.AddWarning("Potential cyclic execution loop detected in graph: " + node->name);
            }
        }
    }

    return report;
}

bool BlueprintValidator::CheckCycleRecursive(const ASTNode* currentNode,
                                            const BlueprintGraphAST& ast,
                                            std::unordered_map<std::string, int>& visitState) {
    if (!currentNode) return false;
    visitState[currentNode->name] = 1; // Visiting

    for (const auto* execPin : currentNode->GetExecOutputs()) {
        for (const auto& [targetNodeName, _] : execPin->linkedTo) {
            auto it = visitState.find(targetNodeName);
            if (it != visitState.end()) {
                if (it->second == 1) {
                    return true; // Cycle found
                } else if (it->second == 0) {
                    const ASTNode* nextNode = ast.FindNode(targetNodeName);
                    if (CheckCycleRecursive(nextNode, ast, visitState)) {
                        return true;
                    }
                }
            }
        }
    }

    visitState[currentNode->name] = 2; // Visited
    return false;
}

} // namespace Kestrel::VM
