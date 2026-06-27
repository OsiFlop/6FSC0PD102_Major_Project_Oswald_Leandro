// Altitude debug actor
// Prints start and goal altitude values
// Uses height cache sea level for ASL conversion
// Helps verify route setup altitude values
// Editor-only debug helper for start / goal checks

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

	// HeightCache: provides sea level reference for ASL conversion
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	// StartActor: actor used as start altitude reference
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> StartActor;

	// GoalActor: actor used as goal altitude reference
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> GoalActor;

	// Print start and goal altitude values to log
	UFUNCTION(CallInEditor, Category="Altitude Debug")
	void PrintStartAndGoalASL();

protected:
	// Check height cache, start actor and goal actor references
	bool ValidateReferences() const;

	// Convert world Z in centimeters to altitude meters ASL
	float GetAltitudeMetersASLFromWorldZ(float WorldZCm) const;
};