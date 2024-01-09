#include "K2Node_QuaternionIntegrator.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "EdGraph/EdGraphPin.h"
#include "KismetCompiler.h"

const FName UK2Node_QuaternionIntegrator::Pin_ExecIn(TEXT("execute"));
const FName UK2Node_QuaternionIntegrator::Pin_ExecOut(TEXT("then"));
const FName UK2Node_QuaternionIntegrator::Pin_CurrentQuat(TEXT("CurrentAttitude"));
const FName UK2Node_QuaternionIntegrator::Pin_AngularVelocity(TEXT("AngularVelocityBody"));
const FName UK2Node_QuaternionIntegrator::Pin_DeltaTime(TEXT("DeltaTime"));
const FName UK2Node_QuaternionIntegrator::Pin_NextQuat(TEXT("NextAttitude"));

void UK2Node_QuaternionIntegrator::AllocateDefaultPins()
{
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Exec, Pin_ExecIn);
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Exec, Pin_ExecOut);

	UScriptStruct* QuatStruct = TBaseStructure<FQuat>::Get();
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Struct, QuatStruct, Pin_CurrentQuat);

	UScriptStruct* VectorStruct = TBaseStructure<FVector>::Get();
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Struct, VectorStruct, Pin_AngularVelocity);

	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_DeltaTime);

	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Struct, QuatStruct, Pin_NextQuat);

	Super::AllocateDefaultPins();
}

FText UK2Node_QuaternionIntegrator::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return FText::FromString(TEXT("Integrate Quaternion Kinematics"));
}

FText UK2Node_QuaternionIntegrator::GetTooltipText() const
{
	return FText::FromString(TEXT("Propagates attitude orientation quaternion using exact SO(3) Lie group exponential map."));
}

FText UK2Node_QuaternionIntegrator::GetMenuCategory() const
{
	return FText::FromString(TEXT("Kestrel|Kinematics"));
}

void UK2Node_QuaternionIntegrator::ExpandNode(FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph)
{
	Super::ExpandNode(CompilerContext, SourceGraph);
}

#endif
