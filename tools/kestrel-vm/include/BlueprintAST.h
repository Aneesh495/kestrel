#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace Kestrel::VM {

enum class EPinDirection {
    Input,
    Output,
    Unknown
};

enum class EPinCategory {
    Exec,
    Real,
    Int,
    Boolean,
    Struct,
    Object,
    String,
    Byte,
    Delegate,
    Unknown
};

struct ASTPin {
    std::string pinId;
    std::string name;
    EPinDirection direction{EPinDirection::Unknown};
    EPinCategory category{EPinCategory::Unknown};
    std::string subCategoryObject;
    std::string defaultValue;
    bool bIsArray{false};

    // Linked connections: pairs of (TargetNodeName, TargetPinId)
    std::vector<std::pair<std::string, std::string>> linkedTo;

    bool IsConnected() const { return !linkedTo.empty(); }
};

struct ASTNode {
    std::string className;
    std::string name;
    std::string comment;
    int posX{0};
    int posY{0};
    std::string guid;
    std::unordered_map<std::string, std::string> properties;
    std::vector<ASTPin> pins;

    const ASTPin* FindPin(const std::string& pinName, EPinDirection dir = EPinDirection::Unknown) const;
    ASTPin* FindPinMutable(const std::string& pinName, EPinDirection dir = EPinDirection::Unknown);
    std::vector<const ASTPin*> GetExecOutputs() const;
};

class BlueprintGraphAST {
public:
    std::string graphName;
    std::string blueprintClass;
    std::vector<std::unique_ptr<ASTNode>> nodes;
    std::unordered_map<std::string, ASTNode*> nodeLookup;

    void AddNode(std::unique_ptr<ASTNode> node);
    const ASTNode* FindNode(const std::string& nodeName) const;
    ASTNode* FindNodeMutable(const std::string& nodeName);

    size_t GetNodeCount() const { return nodes.size(); }
    size_t CountTotalPins() const;
    size_t CountTotalConnections() const;
    std::vector<const ASTNode*> FindNodesByClass(const std::string& classSubstr) const;
};

} // namespace Kestrel::VM
