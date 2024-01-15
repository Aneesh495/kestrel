#include "K2Node_MAVLinkDispatch.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "EdGraph/EdGraphPin.h"
#include "KismetCompiler.h"

const FName UK2Node_MAVLinkDispatch::Pin_ExecIn(TEXT("execute"));
const FName UK2Node_MAVLinkDispatch::Pin_ExecOut(TEXT("then"));
const FName UK2Node_MAVLinkDispatch::Pin_SystemId(TEXT("SystemId"));
const FName UK2Node_MAVLinkDispatch::Pin_ComponentId(TEXT("ComponentId"));
const FName UK2Node_MAVLinkDispatch::Pin_MsgType(TEXT("MessageType"));
const FName UK2Node_MAVLinkDispatch::Pin_Roll(TEXT("RollRad"));
const FName UK2Node_MAVLinkDispatch::Pin_Pitch(TEXT("PitchRad"));
const FName UK2Node_MAVLinkDispatch::Pin_Yaw(TEXT("YawRad"));
const FName UK2Node_MAVLinkDispatch::Pin_BatteryPercent(TEXT("BatteryPercent"));
const FName UK2Node_MAVLinkDispatch::Pin_OutPacketBytes(TEXT("PacketBytes"));

void UK2Node_MAVLinkDispatch::AllocateDefaultPins()
{
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Exec, Pin_ExecIn);
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Exec, Pin_ExecOut);

	UEdGraphPin* PinSys = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Byte, Pin_SystemId);
	PinSys->DefaultValue = TEXT("1");

	UEdGraphPin* PinComp = CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Byte, Pin_ComponentId);
	PinComp->DefaultValue = TEXT("1");

	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_String, Pin_MsgType);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Roll);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Pitch);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_Yaw);
	CreatePin(EGPD_Input, UEdGraphSchema_K2::PC_Real, UEdGraphSchema_K2::PC_Double, Pin_BatteryPercent);

	FCreatePinParams ArrayParams;
	ArrayParams.ContainerType = EPinContainerType::Array;
	CreatePin(EGPD_Output, UEdGraphSchema_K2::PC_Byte, Pin_OutPacketBytes, ArrayParams);

	Super::AllocateDefaultPins();
}

FText UK2Node_MAVLinkDispatch::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return FText::FromString(TEXT("MAVLink 2.0 Dispatcher"));
}

FText UK2Node_MAVLinkDispatch::GetTooltipText() const
{
	return FText::FromString(TEXT("Encodes and formats vehicle telemetry into MAVLink 2.0 wire protocol frames."));
}

FText UK2Node_MAVLinkDispatch::GetMenuCategory() const
{
	return FText::FromString(TEXT("Kestrel|Telemetry"));
}

void UK2Node_MAVLinkDispatch::ExpandNode(FKismetCompilerContext& CompilerContext, UEdGraph* SourceGraph)
{
	Super::ExpandNode(CompilerContext, SourceGraph);
}

#endif
