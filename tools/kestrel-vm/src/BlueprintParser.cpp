#include "BlueprintParser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace Kestrel::VM {

std::string BlueprintParser::ExtractAttribute(const std::string& line, const std::string& key) {
    std::string pattern = key + "=";
    size_t pos = line.find(pattern);
    if (pos == std::string::npos) return "";

    pos += pattern.length();
    if (pos >= line.length()) return "";

    char delim = ',';
    if (line[pos] == '"') {
        pos++;
        size_t endPos = line.find('"', pos);
        if (endPos != std::string::npos) {
            return line.substr(pos, endPos - pos);
        }
        return line.substr(pos);
    } else if (line[pos] == '(') {
        size_t depth = 1;
        size_t start = pos + 1;
        size_t cur = start;
        while (cur < line.length() && depth > 0) {
            if (line[cur] == '(') depth++;
            else if (line[cur] == ')') depth--;
            cur++;
        }
        return line.substr(start, (cur - 1) - start);
    }

    size_t endPos = line.find_first_of(",)\n\r", pos);
    if (endPos != std::string::npos) {
        return line.substr(pos, endPos - pos);
    }
    return line.substr(pos);
}

void BlueprintParser::ParseLinkedTo(const std::string& linkedStr, ASTPin& outPin) {
    if (linkedStr.empty()) return;
    std::stringstream ss(linkedStr);
    std::string token;

    while (std::getline(ss, token, ',')) {
        if (token.empty()) continue;
        size_t start = token.find_first_not_of(" \t");
        if (start != std::string::npos) token = token.substr(start);
        size_t end = token.find_last_not_of(" \t");
        if (end != std::string::npos) token = token.substr(0, end + 1);

        size_t spacePos = token.find(' ');
        if (spacePos != std::string::npos) {
            std::string targetNode = token.substr(0, spacePos);
            std::string targetPinId = token.substr(spacePos + 1);
            outPin.linkedTo.push_back({targetNode, targetPinId});
        }
    }
}

ASTPin BlueprintParser::ParsePinString(const std::string& pinLine) {
    ASTPin pin;
    pin.pinId = ExtractAttribute(pinLine, "PinId");
    pin.name = ExtractAttribute(pinLine, "PinName");
    std::string dir = ExtractAttribute(pinLine, "Direction");
    if (dir == "EGPD_Input") pin.direction = EPinDirection::Input;
    else if (dir == "EGPD_Output") pin.direction = EPinDirection::Output;
    else pin.direction = EPinDirection::Unknown;

    std::string cat = ExtractAttribute(pinLine, "PinType.PinCategory");
    if (cat == "exec") pin.category = EPinCategory::Exec;
    else if (cat == "real") pin.category = EPinCategory::Real;
    else if (cat == "int") pin.category = EPinCategory::Int;
    else if (cat == "bool") pin.category = EPinCategory::Boolean;
    else if (cat == "struct") pin.category = EPinCategory::Struct;
    else if (cat == "object") pin.category = EPinCategory::Object;
    else if (cat == "string") pin.category = EPinCategory::String;
    else if (cat == "byte") pin.category = EPinCategory::Byte;
    else if (cat == "delegate") pin.category = EPinCategory::Delegate;
    else pin.category = EPinCategory::Unknown;

    pin.subCategoryObject = ExtractAttribute(pinLine, "PinType.PinSubCategoryObject");
    pin.defaultValue = ExtractAttribute(pinLine, "DefaultValue");

    if (pinLine.find("PinType.ContainerType=Array") != std::string::npos) {
        pin.bIsArray = true;
    }

    std::string linkedStr = ExtractAttribute(pinLine, "LinkedTo");
    ParseLinkedTo(linkedStr, pin);

    return pin;
}

std::unique_ptr<BlueprintGraphAST> BlueprintParser::ParseStream(std::istream& inStream) {
    auto ast = std::make_unique<BlueprintGraphAST>();
    std::string line;
    std::unique_ptr<ASTNode> currentNode = nullptr;

    while (std::getline(inStream, line)) {
        // Strip carriage returns and leading spaces
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            line.pop_back();
        }
        size_t startIdx = line.find_first_not_of(" \t");
        if (startIdx == std::string::npos) continue;
        std::string trimmed = line.substr(startIdx);

        if (trimmed.rfind("; Blueprint Export:", 0) == 0) {
            ast->blueprintClass = trimmed.substr(20);
        } else if (trimmed.rfind("; Graph:", 0) == 0) {
            ast->graphName = trimmed.substr(9);
        } else if (trimmed.rfind("Begin Object", 0) == 0) {
            currentNode = std::make_unique<ASTNode>();
            currentNode->className = ExtractAttribute(trimmed, "Class");
            currentNode->name = ExtractAttribute(trimmed, "Name");
        } else if (trimmed.rfind("End Object", 0) == 0) {
            if (currentNode) {
                ast->AddNode(std::move(currentNode));
                currentNode = nullptr;
            }
        } else if (currentNode) {
            if (trimmed.rfind("NodeComment=", 0) == 0) {
                currentNode->comment = ExtractAttribute(trimmed, "NodeComment");
            } else if (trimmed.rfind("NodePosX=", 0) == 0) {
                try { currentNode->posX = std::stoi(trimmed.substr(9)); } catch (...) {}
            } else if (trimmed.rfind("NodePosY=", 0) == 0) {
                try { currentNode->posY = std::stoi(trimmed.substr(9)); } catch (...) {}
            } else if (trimmed.rfind("NodeGuid=", 0) == 0) {
                currentNode->guid = trimmed.substr(9);
            } else if (trimmed.rfind("CustomProperties Pin", 0) == 0) {
                currentNode->pins.push_back(ParsePinString(trimmed));
            } else {
                size_t eqPos = trimmed.find('=');
                if (eqPos != std::string::npos) {
                    std::string key = trimmed.substr(0, eqPos);
                    std::string val = trimmed.substr(eqPos + 1);
                    currentNode->properties[key] = val;
                }
            }
        }
    }

    return ast;
}

std::unique_ptr<BlueprintGraphAST> BlueprintParser::ParseFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        return nullptr;
    }
    return ParseStream(file);
}

std::unique_ptr<BlueprintGraphAST> BlueprintParser::ParseString(const std::string& content) {
    std::istringstream stream(content);
    return ParseStream(stream);
}

} // namespace Kestrel::VM
