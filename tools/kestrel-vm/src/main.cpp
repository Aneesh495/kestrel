#include "BlueprintParser.h"
#include "BlueprintValidator.h"
#include "BlueprintEvaluator.h"
#include <iostream>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;
using namespace Kestrel::VM;

void PrintBanner() {
    std::cout << "===================================================================\n";
    std::cout << " KESTREL HEADLESS BLUEPRINT VIRTUAL MACHINE & STATIC VALIDATOR\n";
    std::cout << " Unreal Engine 5 K2Node Graph Execution Engine\n";
    std::cout << "===================================================================\n";
}

int main(int argc, char** argv) {
    PrintBanner();

    std::string targetPath = "Content/Blueprints";
    if (argc > 1) {
        targetPath = argv[1];
    }

    std::vector<std::string> bpFiles;
    if (fs::is_directory(targetPath)) {
        for (const auto& entry : fs::recursive_directory_iterator(targetPath)) {
            if (entry.path().extension() == ".bp") {
                bpFiles.push_back(entry.path().string());
            }
        }
    } else if (fs::exists(targetPath)) {
        bpFiles.push_back(targetPath);
    } else {
        std::cerr << "Error: Specified path does not exist: " << targetPath << "\n";
        return 1;
    }

    std::cout << "Found " << bpFiles.size() << " Blueprint graph file(s) for verification.\n\n";

    size_t totalNodes = 0;
    size_t totalPins = 0;
    size_t totalWires = 0;
    bool allValid = true;

    for (const auto& file : bpFiles) {
        std::cout << "[PARSING] " << file << " ... ";
        auto ast = BlueprintParser::ParseFile(file);
        if (!ast) {
            std::cout << "FAILED to open.\n";
            allValid = false;
            continue;
        }

        size_t nodes = ast->GetNodeCount();
        size_t pins = ast->CountTotalPins();
        size_t wires = ast->CountTotalConnections();
        totalNodes += nodes;
        totalPins += pins;
        totalWires += wires;

        ValidationReport report = BlueprintValidator::ValidateGraph(*ast);
        if (!report.bIsValid) {
            std::cout << "INVALID (" << report.errorMessages.size() << " errors)\n";
            for (const auto& err : report.errorMessages) {
                std::cout << "  ERROR: " << err << "\n";
            }
            allValid = false;
        } else {
            std::cout << "PASSED (" << nodes << " nodes, " << pins << " pins, " << wires << " wires)\n";

            // Run sample evaluation step
            BlueprintEvaluator evaluator(*ast);
            ExecutionContext ctx;
            evaluator.Execute("ReceiveBeginPlay", ctx);
        }
    }

    std::cout << "\n-------------------------------------------------------------------\n";
    std::cout << " VERIFICATION SUMMARY\n";
    std::cout << " Total Blueprint Files Verified: " << bpFiles.size() << "\n";
    std::cout << " Total Graph Nodes Parsed:       " << totalNodes << "\n";
    std::cout << " Total Graph Pins Validated:     " << totalPins << "\n";
    std::cout << " Total Interconnection Wires:    " << totalWires << "\n";
    std::cout << " Overall Graph Integrity:        " << (allValid ? "100% PASSED" : "FAILED") << "\n";
    std::cout << "-------------------------------------------------------------------\n";

    return allValid ? 0 : 1;
}
