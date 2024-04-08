#include "BlueprintParser.h"
#include "BlueprintValidator.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;
using namespace Kestrel::VM;

int main() {
    std::cout << "=== KESTREL TEST SUITE: BLUEPRINT AST STATIC VERIFICATION ===\n";

    std::vector<std::string> bpFiles = {
        "Content/Blueprints/BP_KestrelPawn.bp",
        "Content/Blueprints/BP_FlightDynamicsComponent.bp",
        "Content/Blueprints/BP_SensorSuiteComponent.bp",
        "Content/Blueprints/BP_StateEstimatorComponent.bp",
        "Content/Blueprints/BP_FlightControllerComponent.bp",
        "Content/Blueprints/BP_NavigatorComponent.bp",
        "Content/Blueprints/BP_MissionExecutive.bp",
        "Content/Blueprints/BP_SafetySupervisorComponent.bp",
        "Content/Blueprints/BP_TelemetryBridge.bp",
        "Content/Blueprints/UI/BP_GroundControlStation_Widget.bp"
    };

    std::string rootPrefix = "";
    if (fs::exists("Content/Blueprints/BP_KestrelPawn.bp")) {
        rootPrefix = "";
    } else if (fs::exists("../Content/Blueprints/BP_KestrelPawn.bp")) {
        rootPrefix = "../";
    } else if (fs::exists("../../Content/Blueprints/BP_KestrelPawn.bp")) {
        rootPrefix = "../../";
    }

    size_t totalNodes = 0;
    size_t totalPins = 0;
    size_t totalWires = 0;
    size_t totalLines = 0;

    for (const auto& relPath : bpFiles) {
        std::string filePath = rootPrefix + relPath;
        assert(fs::exists(filePath));

        // Count lines
        std::ifstream fileCount(filePath);
        std::string line;
        size_t fileLines = 0;
        while (std::getline(fileCount, line)) {
            fileLines++;
        }
        totalLines += fileLines;

        auto ast = BlueprintParser::ParseFile(filePath);
        assert(ast != nullptr);

        size_t nNodes = ast->GetNodeCount();
        size_t nPins = ast->CountTotalPins();
        size_t nWires = ast->CountTotalConnections();

        assert(nNodes >= 250);
        assert(nPins >= 750);

        totalNodes += nNodes;
        totalPins += nPins;
        totalWires += nWires;

        ValidationReport report = BlueprintValidator::ValidateGraph(*ast);
        if (!report.bIsValid) {
            std::cerr << "VALIDATION FAILED FOR " << filePath << ":\n";
            for (const auto& err : report.errorMessages) {
                std::cerr << "  " << err << "\n";
            }
        }
        assert(report.bIsValid);
        assert(report.danglingReferences == 0);

        std::cout << "  [PASS] " << filePath << " (" << nNodes << " nodes, " << fileLines << " lines)\n";
    }

    std::cout << "\nAGGREGATE BLUEPRINT METRICS:\n";
    std::cout << "  Total Blueprint Files:   " << bpFiles.size() << "\n";
    std::cout << "  Total Parsed AST Nodes:  " << totalNodes << " (> 3000 required)\n";
    std::cout << "  Total Validated Pins:    " << totalPins << " (> 8000 required)\n";
    std::cout << "  Total Interconnections:  " << totalWires << "\n";
    std::cout << "  Total Lines of Blueprint Code: " << totalLines << " (> 30000 lines required)\n";

    assert(totalNodes >= 3000);
    assert(totalPins >= 8000);
    assert(totalLines >= 30000);

    std::cout << "ALL BLUEPRINT AST STATIC VERIFICATION CHECKS PASSED.\n\n";
    return 0;
}
