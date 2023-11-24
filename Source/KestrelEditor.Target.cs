using UnrealBuildTool;
using System.Collections.Generic;

public class KestrelEditorTarget : TargetRules
{
	public KestrelEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V4;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_4;
		ExtraModuleNames.AddRange(new string[] { "Kestrel", "KestrelCore", "KestrelEditor" });
	}
}
