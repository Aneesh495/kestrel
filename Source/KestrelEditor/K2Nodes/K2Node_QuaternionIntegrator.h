#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "K2Node.h"
#include "K2Node_QuaternionIntegrator.generated.h"

/**
 * Custom Unreal Blueprint Node: Singularity-Free Lie Algebra Quaternion Integrator.
 * Propagates unit attitude quaternion given 3D angular velocity vector and time delta.
 */
UCLASS()
class KESTRELEDITOR_API UK2Node_QuaternionIntegrator : public UK2Node
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
	static const FName Pin_CurrentQuat;
	static const FName Pin_AngularVelocity;
	static const FName Pin_DeltaTime;
	static const FName Pin_NextQuat;
};
#else
namespace Kestrel {
    struct UK2Node_QuaternionIntegrator {
        static constexpr const char* Title = "Quaternion Attitude Integrator";
    };
}
#endif
