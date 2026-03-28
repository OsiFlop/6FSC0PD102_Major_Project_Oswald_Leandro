// Helper actor providing access to landscape reference and world bounds
// Used as a shared terrain source for grid and height queries

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TerrainReferenceActor.generated.h"

class ALandscapeProxy;

UCLASS()
class MAJORPROJECT_API ATerrainReferenceActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ATerrainReferenceActor();

	// Reference to landscape proxy actor
	UPROPERTY(EditAnywhere, Category="Terrain")
	TObjectPtr<ALandscapeProxy> Landscape;

	// Automatically find landscape in current world
	UFUNCTION(CallInEditor, Category="Terrain")
	void AutoFindLandscape();

	// Get landscape world-space bounds (Axis-Aligned Bounding Box)
	UFUNCTION(BlueprintCallable, Category="Terrain")
	bool GetLandscapeWorldBounds(FVector& OutMin, FVector& OutMax) const;

	// Check if world position is inside landscape bounds
	UFUNCTION(BlueprintCallable, Category="Terrain")
	bool IsWorldPosInsideLandscapeBounds(const FVector& WorldPos) const;

	// Debug print and draw landscape bounds
	UFUNCTION(CallInEditor, Category="Terrain|Debug")
	void DebugPrintLandscapeBounds();
};