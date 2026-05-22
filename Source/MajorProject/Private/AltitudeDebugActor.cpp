// Fill out your copyright notice in the Description page of Project Settings.


#include "AltitudeDebugActor.h"
#include "Navigation/Grid/VoxelHeightCache.h"

// Sets default values
AAltitudeDebugActor::AAltitudeDebugActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

}

bool AAltitudeDebugActor::ValidateReferences() const
{
	if (!HeightCache)
	{
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: HeightCache fehlt."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: HeightCache ist ungueltig."));
		return false;
	}

	if (!StartActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: StartActor fehlt."));
		return false;
	}

	if (!GoalActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("AltitudeDebug: GoalActor fehlt."));
		return false;
	}

	return true;
}

float AAltitudeDebugActor::GetAltitudeMetersASLFromWorldZ(float WorldZCm) const
{
	if (!HeightCache)
	{
		return 0.0f;
	}

	return (WorldZCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

void AAltitudeDebugActor::PrintStartAndGoalASL()
{
	if (!ValidateReferences())
	{
		return;
	}

	const float StartASL = GetAltitudeMetersASLFromWorldZ(StartActor->GetActorLocation().Z);
	const float GoalASL = GetAltitudeMetersASLFromWorldZ(GoalActor->GetActorLocation().Z);

	UE_LOG(LogTemp, Display, TEXT("Start ASL: %.2f m"), StartASL);
	UE_LOG(LogTemp, Display, TEXT("Goal ASL: %.2f m"), GoalASL);
	UE_LOG(LogTemp, Display, TEXT("Delta ASL: %.2f m"), GoalASL - StartASL);
}