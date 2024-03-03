#pragma once

#include "BlueprintAST.h"
#include "Math/KestrelMathTypes.h"
#include "Math/KestrelQuaternionMath.h"
#include <string>
#include <unordered_map>
#include <variant>

namespace Kestrel::VM {

using VMValue = std::variant<double, int64_t, bool, std::string, Vector3, Quat>;

struct ExecutionContext {
    std::unordered_map<std::string, VMValue> variables;
    std::unordered_map<std::string, VMValue> pinValues;
    int instructionsExecuted{0};
    double simulationDeltaTime{0.002}; // 500 Hz

    void SetPinValue(const std::string& pinId, const VMValue& val) {
        pinValues[pinId] = val;
    }

    VMValue GetPinValue(const std::string& pinId, const VMValue& fallback = 0.0) const {
        auto it = pinValues.find(pinId);
        if (it != pinValues.end()) return it->second;
        return fallback;
    }
};

/**
 * Headless Execution Virtual Machine for Blueprint Graphs.
 * Dispatches instruction steps, evaluates custom K2Nodes,
 * tracks variable bindings, and executes 500 Hz simulation cycles.
 */
class BlueprintEvaluator {
public:
    explicit BlueprintEvaluator(const BlueprintGraphAST& inAST);

    /**
     * Executes the graph starting from the specified entry node name.
     */
    bool Execute(const std::string& entryNodeName, ExecutionContext& ctx);

    /**
     * Steps one instruction node forward along the active execution wire.
     */
    const ASTNode* StepNode(const ASTNode* currentNode, ExecutionContext& ctx);

private:
    const BlueprintGraphAST& ast;

    void EvaluateCustomK2Node(const ASTNode* node, ExecutionContext& ctx);
    void EvaluateCascadedPID(const ASTNode* node, ExecutionContext& ctx);
    void EvaluateQuaternionIntegrator(const ASTNode* node, ExecutionContext& ctx);
    void EvaluateCallFunction(const ASTNode* node, ExecutionContext& ctx);
};

} // namespace Kestrel::VM
