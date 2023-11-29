#include "KestrelGameModeBase.h"

AKestrelGameModeBase::AKestrelGameModeBase()
	: SimulationTimeSeconds(0.0)
	, PhysicsSubstepCount(0)
	, bEnableSITLBridge(true)
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
}

void AKestrelGameModeBase::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
	SimulationTimeSeconds = 0.0;
	PhysicsSubstepCount = 0;
}

void AKestrelGameModeBase::StartPlay()
{
	Super::StartPlay();
}

void AKestrelGameModeBase::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	SimulationTimeSeconds += static_cast<double>(DeltaSeconds);
	PhysicsSubstepCount++;
}
