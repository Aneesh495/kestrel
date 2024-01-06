#include "K2Node_CascadedPID.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "EdGraph/EdGraphPin.h"
#include "KismetCompiler.h"
#include "BlueprintNodeSpawner.h"
#include "EditorCategoryUtils.h"

const FName UK2Node_CascadedPID::Pin_ExecIn(TEXT("execute"));
const FName UK2Node_CascadedPID::Pin_ExecOut(TEXT("then"));
const FName UK2Node_CascadedPID::Pin_Setpoint(TEXT("Setpoint"));
const FName UK2Node_CascadedPID::Pin_Measurement(TEXT("Measurement"));
const FName UK2Node_CascadedPID::Pin_Kp(TEXT("Kp"));
const FName UK2Node_CascadedPID::Pin_Ki(TEXT("Ki"));
const FName UK2Node_CascadedPID::Pin_Kd(TEXT("Kd"));
const FName UK2Node_CascadedPID::Pin_Kf(TEXT("Kf"));
const FName UK2Node_CascadedPID::Pin_MinOutput(TEXT("MinOutput"));
const FName UK2Node_CascadedPID::Pin_MaxOutput(TEXT("MaxOutput"));
const FName UK2Node_CascadedPID::Pin_DeltaTime(TEXT("DeltaTime"));
const FName UK2Node_CascadedPID::Pin_ControlOutput(TEXT("ControlOutput"));
const FName UK2Node_CascadedPID::Pin_Saturated(TEXT("bIsSaturated"));

void UK2Node_CascadedPID::AllocateDefaultPins()
{
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Exec, Pin_ExecIn);
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Exec, Pin_ExecOut);

	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Setpoint);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Measurement);

	UEdGraphPin* PinKp = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Kp);
	PinKp->DefaultValue = TEXT("1.0");

	UEdGraphPin* PinKi = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Ki);
	PinKi->DefaultValue = TEXT("0.0");

	UEdGraphPin* PinKd = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Kd);
	PinKd->DefaultValue = TEXT("0.0");

	UEdGraphPin* PinKf = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Kf);
	PinKf->DefaultValue = TEXT("0.0");

	UEdGraphPin* PinMin = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_MinOutput);
	PinMin->DefaultValue = TEXT("-1.0");

	UEdGraphPin* PinMax = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_MaxOutput);
	PinMax->DefaultValue = TEXT("1.0");

	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_DeltaTime);

	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_ControlOutput);
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Boolean, Pin_Saturated);

	Super::AllocateDefaultPins();
}

FText UK2Node_CascadedPID::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return FText::FromString(TEXT("Cascaded PID (Anti-Windup)"));
}

FText UK2Node_CascadedPID::GetTooltipText() const
{
	return FText::FromString(TEXT("Evaluates proportional, integral, derivative, and feedforward control laws with anti-windup integrator clamping."));
}

FText UK2Node_CascadedPID::GetMenuCategory() const
{
	return FText::FromString(TEXT("Kestrel|FlightControl"));
}

void UK2Node_CascadedPID::ExpandNode(FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph)
{
	Super::ExpandNode(CompilerContext, SourceGraph);
}

#endif
