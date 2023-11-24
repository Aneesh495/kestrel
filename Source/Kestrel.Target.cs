using UnrealBuildTool;
using System.Collections.Generic;

public class KestrelTarget : TargetRules
{
	public KestrelTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V4;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_4;
		ExtraModuleNames.AddRange(new string[] { "Kestrel", "KestrelCore" });
	}
}
