// Runtime probe actor for querying baked voxel heights
// Uses grid + height cache to query max height at world position

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectPtr.h"
#include "GameFramework/Actor.h"
#include "HeightQueryProbeActor.generated.h"

class AVoxelGridBaker;
class UVoxelHeightCache;

UCLASS()
class MAJORPROJECT_API AHeightQueryProbeActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AHeightQueryProbeActor();

	// Reference to built voxel grid
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AVoxelGridBaker> GridBaker;

	// Reference to baked height cache
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	// Query max height at actor world location
	UFUNCTION(CallInEditor, Category="HeightQuery")
	void QueryHeightAtMyLocation();

	// Enable debug marker visualization
	UPROPERTY(EditAnywhere, Category="Debug")
	bool bDrawMarker = true;

	// Debug marker lifetime (seconds)
	UPROPERTY(EditAnywhere, Category="Debug")
	float MarkerLifeTime = 5.0f;

private:
	// Convert world XY position into grid cell indices
	bool WorldXYToCell(const FVector& WorldPos, int32& OutX, int32& OutY) const;
};