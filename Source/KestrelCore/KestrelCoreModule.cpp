#include "KestrelCoreModule.h"

DEFINE_LOG_CATEGORY_STATIC(LogKestrelCore, Log, All);

void FKestrelCoreModule::StartupModule()
{
	UE_LOG(LogKestrelCore, Log, TEXT("KestrelCore High-Performance Primitives Module initialized."));
}

void FKestrelCoreModule::ShutdownModule()
{
	UE_LOG(LogKestrelCore, Log, TEXT("KestrelCore High-Performance Primitives Module shut down."));
}

IMPLEMENT_MODULE(FKestrelCoreModule, KestrelCore);
