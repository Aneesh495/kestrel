#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "K2Node.h"
#include "K2Node_MAVLinkDispatch.generated.h"

/**
 * Custom Unreal Blueprint Node: MAVLink 2.0 Telemetry Packet Dispatcher.
 * Encodes flight states, attitude quaternions, GNSS coordinates, and battery telemetry
 * into binary frames with hardware-accelerated CRC-16 checksums.
 */
UCLASS()
class KESTRELEDITOR_API UK2Node_MAVLinkDispatch : public UK2Node
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
	static const FName Pin_SystemId;
	static const FName Pin_ComponentId;
	static const FName Pin_MsgType;
	static const FName Pin_Roll;
	static const FName Pin_Pitch;
	static const FName Pin_Yaw;
	static const FName Pin_BatteryPercent;
	static const FName Pin_OutPacketBytes;
};
#else
namespace Kestrel {
    struct UK2Node_MAVLinkDispatch {
        static constexpr const char* Title = "MAVLink 2.0 Dispatcher";
    };
}
#endif
