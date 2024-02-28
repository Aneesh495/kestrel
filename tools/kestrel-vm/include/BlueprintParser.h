#pragma once

#include "BlueprintAST.h"
#include <string>
#include <memory>
#include <istream>

namespace Kestrel::VM {

/**
 * High-Speed Parser for Unreal Engine Blueprint Text Serialization.
 * Lexes and parses Begin Object blocks, CustomProperties Pin declarations,
 * and pin interconnection topologies into an in-memory BlueprintGraphAST.
 */
class BlueprintParser {
public:
    static std::unique_ptr<BlueprintGraphAST> ParseFile(const std::string& filePath);
    static std::unique_ptr<BlueprintGraphAST> ParseStream(std::istream& inStream);
    static std::unique_ptr<BlueprintGraphAST> ParseString(const std::string& content);

private:
    static ASTPin ParsePinString(const std::string& pinLine);
    static std::string ExtractAttribute(const std::string& line, const std::string& key);
    static void ParseLinkedTo(const std::string& linkedStr, ASTPin& outPin);
};

} // namespace Kestrel::VM
