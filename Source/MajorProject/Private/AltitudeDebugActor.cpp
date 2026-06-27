// Altitude debug actor
// Converts world Z values into meters above sea level
// Uses height cache sea level as reference
// Logs start altitude, goal altitude and altitude difference
// Manual editor debug helper for route setup checks

#include "AltitudeDebugActor.h"
#include "Navigation/Grid/VoxelHeightCache.h"

// Disable ticking because altitude debug output is triggered manually
AAltitudeDebugActor::AAltitudeDebugActor()
{
	// No runtime tick needed
	PrimaryActorTick.bCanEverTick = false;
}

// Check required references before altitude calculation
bool AAltitudeDebugActor::ValidateReferences() const
{
	if (!HeightCache)
	{
		// Missing sea level reference
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: HeightCache is missing."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		// Height cache not baked or invalid
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: HeightCache is invalid."));
		return false;
	}

	if (!StartActor)
	{
		// Missing start altitude source
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: StartActor is missing."));
		return false;
	}

	if (!GoalActor)
	{
		// Missing goal altitude source
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: GoalActor is missing."));
		return false;
	}

	return true;
}

// Convert Unreal world Z in centimeters to meters ASL
float AAltitudeDebugActor::GetAltitudeMetersASLFromWorldZ(float WorldZCm) const
{
	if (!HeightCache)
	{
		// Safe fallback, no sea level data
		return 0.0f;
	}

	// ASL = world height minus sea level, converted cm to m
	return (WorldZCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

// Print start / goal altitude values and difference
void AAltitudeDebugActor::PrintStartAndGoalASL()
{
	if (!ValidateReferences())
	{
		return;
	}

	// StartASL: start actor height above sea level
	const float StartASL = GetAltitudeMetersASLFromWorldZ(StartActor->GetActorLocation().Z);

	// GoalASL: goal actor height above sea level
	const float GoalASL = GetAltitudeMetersASLFromWorldZ(GoalActor->GetActorLocation().Z);

	// Debug log: values for ASL conversion check
	UE_LOG(LogTemp, Display, TEXT("Start ASL: %.2f m"), StartASL);
	UE_LOG(LogTemp, Display, TEXT("Goal ASL: %.2f m"), GoalASL);

	// Delta ASL: climb/descent difference from start to goal
	UE_LOG(LogTemp, Display, TEXT("Delta ASL: %.2f m"), GoalASL - StartASL);
}