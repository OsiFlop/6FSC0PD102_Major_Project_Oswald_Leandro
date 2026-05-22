// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AltitudeDebugActor.generated.h"

class UVoxelHeightCache;

UCLASS()
class MAJORPROJECT_API AAltitudeDebugActor : public AActor
{
	GENERATED_BODY()
	
public:
	AAltitudeDebugActor();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> StartActor;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> GoalActor;

	UFUNCTION(CallInEditor, Category="Altitude Debug")
	void PrintStartAndGoalASL();

protected:
	bool ValidateReferences() const;
	float GetAltitudeMetersASLFromWorldZ(float WorldZCm) const;
};