#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "K2Node.h"
#include "K2Node_FastLiDARTrace.generated.h"

/**
 * Custom Unreal Blueprint Node: Fast Multi-Beam Octree LiDAR Raycaster.
 * Executes parallel spatial raycasting against the environmental Octree bounding volumes.
 */
UCLASS()
class KESTRELEDITOR_API UK2Node_FastLiDARTrace : public UK2Node
{
	GENERATED_BODY()

public:
	virtual void AllocateDefaultPins() override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	virtual FText GetMenuCategory() const override;
	virtual void ExpandNode(class FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph) override;

	static const FName Pin_ExecIn;
	static const FName Pin_ExecOut;
	static const FName Pin_SensorLocation;
	static const FName Pin_SensorRotation;
	static const FName Pin_BeamCount;
	static const FName Pin_MaxRange;
	static const FName Pin_OutHitPoints;
	static const FName Pin_OutHitCount;
};
#else
namespace Kestrel {
    struct UK2Node_FastLiDARTrace {
        static constexpr const char* Title = "Fast Octree LiDAR Raycast";
    };
}
#endif
