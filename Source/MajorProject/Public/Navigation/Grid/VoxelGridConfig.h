// Data asset implementation for voxel grid configuration
// Stores grid, sampling, tracing, and debug parameters

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "VoxelGridConfig.generated.h"

/**
 * 
 */
UCLASS()
class MAJORPROJECT_API UVoxelGridConfig : public UDataAsset
{
	GENERATED_BODY()
	
public:
	// Grid cell size in meters (XY)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Grid")
	float CellSizeMeters = 100.0f;

	// Padding around landscape bounds (meters)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Grid")
	float PaddingMeters = 0.0f;

	// Debug line lifetime (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	float DebugDrawLifetime = 10.0f;

	// Debug line thickness
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	float DebugLineThickness = 2.0f;

	// Samples per cell per axis
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Sampling")
	int32 SamplesPerAxis = 3;

	// Trace start height above cell (meters)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Sampling")
	float TraceStartAboveMeters = 3000.0f;

	// Trace end depth below cell (meters)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Sampling")
	float TraceEndBelowMeters = 3000.0f;

	// Collision channel used for terrain tracing
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Sampling")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;
};