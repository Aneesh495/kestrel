#include "Kestrel.h"

DEFINE_LOG_CATEGORY_STATIC(LogKestrel, Log, All);

void FKestrelModule::StartupModule()
{
	UE_LOG(LogKestrel, Log, TEXT("Kestrel Autonomous Flight Simulation Module initialized."));
}

void FKestrelModule::ShutdownModule()
{
	UE_LOG(LogKestrel, Log, TEXT("Kestrel Autonomous Flight Simulation Module shut down."));
}

IMPLEMENT_PRIMARY_GAME_MODULE(FKestrelModule, Kestrel, "Kestrel");
