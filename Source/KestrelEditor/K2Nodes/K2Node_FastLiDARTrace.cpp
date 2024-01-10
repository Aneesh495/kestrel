#include "K2Node_FastLiDARTrace.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "EdGraph/EdGraphPin.h"
#include "KismetCompiler.h"

const FName UK2Node_FastLiDARTrace::Pin_ExecIn(TEXT("execute"));
const FName UK2Node_FastLiDARTrace::Pin_ExecOut(TEXT("then"));
const FName UK2Node_FastLiDARTrace::Pin_SensorLocation(TEXT("SensorLocation"));
const FName UK2Node_FastLiDARTrace::Pin_SensorRotation(TEXT("SensorRotation"));
const FName UK2Node_FastLiDARTrace::Pin_BeamCount(TEXT("BeamCount"));
const FName UK2Node_FastLiDARTrace::Pin_MaxRange(TEXT("MaxRangeMeters"));
const FName UK2Node_FastLiDARTrace::Pin_OutHitPoints(TEXT("HitPoints"));
const FName UK2Node_FastLiDARTrace::Pin_OutHitCount(TEXT("HitCount"));

void UK2Node_FastLiDARTrace::AllocateDefaultPins()
{
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Exec, Pin_ExecIn);
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Exec, Pin_ExecOut);

	UScriptStruct* VectorStruct = TBaseStructure<FVector>::Get();
	UScriptStruct* RotatorStruct = TBaseStructure<FRotator>::Get();

	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Struct, VectorStruct, Pin_SensorLocation);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Struct, RotatorStruct, Pin_SensorRotation);

	UEdGraphPin* PinBeams = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Int, Pin_BeamCount);
	PinBeams->DefaultValue = TEXT("16");

	UEdGraphPin* PinRange = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_MaxRange);
	PinRange->DefaultValue = TEXT("50.0");

	FCreatePinParams ArrayParams;
	ArrayParams.ContainerType = EPinContainerType::Array;
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Struct, VectorStruct, Pin_OutHitPoints, ArrayParams);

	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Int, Pin_OutHitCount);

	Super::AllocateDefaultPins();
}

FText UK2Node_FastLiDARTrace::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return FText::FromString(TEXT("Fast Octree LiDAR Raycast"));
}

FText UK2Node_FastLiDARTrace::GetTooltipText() const
{
	return FText::FromString(TEXT("Performs parallel multi-beam spatial LiDAR sweeps against the accelerated Octree."));
}

FText UK2Node_FastLiDARTrace::GetMenuCategory() const
{
	return FText::FromString(TEXT("Kestrel|Sensors"));
}

void UK2Node_FastLiDARTrace::ExpandNode(FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph)
{
	Super::ExpandNode(CompilerContext, SourceGraph);
}

#endif
