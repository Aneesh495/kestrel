#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "K2Node.h"
#include "K2Node_CascadedPID.generated.h"

/**
 * Custom Unreal Blueprint Node: Cascaded PID Controller with Anti-Windup Clamping.
 * Provides custom execution and data pins for error computation, integrator anti-windup,
 * derivative low-pass filtering, and feedforward acceleration injection.
 */
UCLASS()
class KESTRELEDITOR_API UK2Node_CascadedPID : public UK2Node
{
	GENERATED_BODY()

public:
	virtual void AllocateDefaultPins() override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	virtual FText GetMenuCategory() const override;
	virtual void ExpandNode(class FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph) override;

	// Pin names
	static const FName Pin_ExecIn;
	static const FName Pin_ExecOut;
	static const FName Pin_Setpoint;
	static const FName Pin_Measurement;
	static const FName Pin_Kp;
	static const FName Pin_Ki;
	static const FName Pin_Kd;
	static const FName Pin_Kf;
	static const FName Pin_MinOutput;
	static const FName Pin_MaxOutput;
	static const FName Pin_DeltaTime;
	static const FName Pin_ControlOutput;
	static const FName Pin_Saturated;
};
#else
namespace Kestrel {
    struct UK2Node_CascadedPID {
        static constexpr const char* Title = "Cascaded PID Controller";
    };
}
#endif
