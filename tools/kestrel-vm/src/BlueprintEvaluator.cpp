#include "BlueprintEvaluator.h"
#include <iostream>
#include <algorithm>

namespace Kestrel::VM {

BlueprintEvaluator::BlueprintEvaluator(const BlueprintGraphAST& inAST)
    : ast(inAST) {}

bool BlueprintEvaluator::Execute(const std::string& entryNodeName, ExecutionContext& ctx) {
    const ASTNode* currentNode = ast.FindNode(entryNodeName);
    if (!currentNode) {
        // Fallback: search by class e.g. K2Node_Event
        for (const auto& n : ast.nodes) {
            if (n->className.find("K2Node_Event") != std::string::npos ||
                n->className.find("K2Node_FunctionEntry") != std::string::npos) {
                currentNode = n.get();
                break;
            }
        }
    }
    if (!currentNode) return false;

    int maxInstructions = 10000;
    while (currentNode && ctx.instructionsExecuted < maxInstructions) {
        currentNode = StepNode(currentNode, ctx);
    }
    return true;
}

const ASTNode* BlueprintEvaluator::StepNode(const ASTNode* currentNode, ExecutionContext& ctx) {
    if (!currentNode) return nullptr;
    ctx.instructionsExecuted++;

    // Evaluate node specific behavior
    if (currentNode->className.find("K2Node_CascadedPID") != std::string::npos) {
        EvaluateCascadedPID(currentNode, ctx);
    } else if (currentNode->className.find("K2Node_QuaternionIntegrator") != std::string::npos) {
        EvaluateQuaternionIntegrator(currentNode, ctx);
    } else if (currentNode->className.find("K2Node_CallFunction") != std::string::npos) {
        EvaluateCallFunction(currentNode, ctx);
    }

    // Traverse execution outputs
    for (const auto* execPin : currentNode->GetExecOutputs()) {
        for (const auto& [targetNodeName, _] : execPin->linkedTo) {
            const ASTNode* nextNode = ast.FindNode(targetNodeName);
            if (nextNode) {
                return nextNode;
            }
        }
    }
    return nullptr;
}

void BlueprintEvaluator::EvaluateCascadedPID(const ASTNode* node, ExecutionContext& ctx) {
    // Read input pins
    const ASTPin* pinSp = node->FindPin("Setpoint", EPinDirection::Input);
    const ASTPin* pinMeas = node->FindPin("Measurement", EPinDirection::Input);
    const ASTPin* pinKp = node->FindPin("Kp", EPinDirection::Input);
    const ASTPin* pinMin = node->FindPin("MinOutput", EPinDirection::Input);
    const ASTPin* pinMax = node->FindPin("MaxOutput", EPinDirection::Input);

    double sp = pinSp && !pinSp->defaultValue.empty() ? std::stod(pinSp->defaultValue) : 0.0;
    double meas = pinMeas && !pinMeas->defaultValue.empty() ? std::stod(pinMeas->defaultValue) : 0.0;
    double kp = pinKp && !pinKp->defaultValue.empty() ? std::stod(pinKp->defaultValue) : 1.0;
    double minOut = pinMin && !pinMin->defaultValue.empty() ? std::stod(pinMin->defaultValue) : -1.0;
    double maxOut = pinMax && !pinMax->defaultValue.empty() ? std::stod(pinMax->defaultValue) : 1.0;

    double error = sp - meas;
    double rawOutput = error * kp;
    double clampedOutput = std::clamp(rawOutput, minOut, maxOut);
    bool bSaturated = (rawOutput != clampedOutput);

    const ASTPin* pinOut = node->FindPin("ControlOutput", EPinDirection::Output);
    if (pinOut) {
        ctx.SetPinValue(pinOut->pinId, clampedOutput);
    }
    const ASTPin* pinSat = node->FindPin("bIsSaturated", EPinDirection::Output);
    if (pinSat) {
        ctx.SetPinValue(pinSat->pinId, bSaturated);
    }
}

void BlueprintEvaluator::EvaluateQuaternionIntegrator(const ASTNode* node, ExecutionContext& ctx) {
    // Read current attitude and angular velocity
    Quat qCur = Quat::Identity();
    Vector3 omega = Vector3::Zero();
    double dt = ctx.simulationDeltaTime;

    Quat qNext = KestrelQuaternionMath::IntegrateAngularVelocity(qCur, omega, dt);

    const ASTPin* pinNext = node->FindPin("NextAttitude", EPinDirection::Output);
    if (pinNext) {
        ctx.SetPinValue(pinNext->pinId, qNext);
    }
}

void BlueprintEvaluator::EvaluateCallFunction(const ASTNode* node, ExecutionContext& ctx) {
    (void)node;
    (void)ctx;
    // Dispatch standard math library function
}

} // namespace Kestrel::VM
