#pragma once

#include "BlueprintAST.h"
#include <vector>
#include <string>

namespace Kestrel::VM {

struct ValidationReport {
    bool bIsValid{true};
    size_t totalNodesChecked{0};
    size_t totalPinsChecked{0};
    size_t totalWiresChecked{0};
    size_t danglingReferences{0};
    size_t typeMismatches{0};
    size_t executionCyclesDetected{0};
    std::vector<std::string> errorMessages;
    std::vector<std::string> warningMessages;

    void AddError(const std::string& err) {
        bIsValid = false;
        errorMessages.push_back(err);
    }

    void AddWarning(const std::string& warn) {
        warningMessages.push_back(warn);
    }
};

/**
 * Static Semantic Analyzer and Type Checker for Blueprint Graphs.
 * Enforces topological consistency, pin type compatibility, and acyclic control flows.
 */
class BlueprintValidator {
public:
    static ValidationReport ValidateGraph(const BlueprintGraphAST& ast);

private:
    static bool CheckCycleRecursive(const ASTNode* currentNode,
                                   const BlueprintGraphAST& ast,
                                   std::unordered_map<std::string, int>& visitState);
};

} // namespace Kestrel::VM
