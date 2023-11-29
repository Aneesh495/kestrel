#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "KestrelGameModeBase.generated.h"

/**
 * AKestrelGameModeBase
 *
 * Primary game mode orchestrating the autonomous robotics simulation environment,
 * coordinate systems, and physics sub-stepping synchronization.
 */
UCLASS()
class KESTREL_API AKestrelGameModeBase : public AGameModeBase
{
	GENERATED_BODY()

public:
	AKestrelGameModeBase();

	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	virtual void StartPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Total simulation elapsed time in seconds */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Kestrel|Simulation")
	double SimulationTimeSeconds;

	/** Number of physics simulation substeps evaluated */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Kestrel|Simulation")
	int64 PhysicsSubstepCount;

	/** Whether software-in-the-loop (SITL) network bridge is active */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kestrel|Simulation")
	bool bEnableSITLBridge;
};
