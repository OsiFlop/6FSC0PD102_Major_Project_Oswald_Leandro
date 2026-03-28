// Runtime data container for baked voxel height results
// Stores per-cell max height and grid metadata

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "VoxelHeightCache.generated.h"

/**
 * 
 */
UCLASS()
class MAJORPROJECT_API UVoxelHeightCache : public UDataAsset
{
	GENERATED_BODY()

public:
	// Grid minimum world-space corner (XY)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Grid")
	FVector GridMinWorld = FVector::ZeroVector;

	// Grid resolution in cells (X,Y)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Grid")
	FIntPoint GridSize = FIntPoint(0, 0);

	// Grid cell size in centimeters
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Grid")
	float CellSizeCm = 0.0f;

	// Per-cell maximum world Z value (cm)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Data")
	TArray<float> MaxHeightCm;

	// Optional sea level reference in world Z (cm)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Data")
	float SeaLevelWorldZCm = 0.0f;

	// Allocate storage for grid and initialize values
	UFUNCTION(BlueprintCallable, Category="Data")
	void Allocate(int32 SizeX, int32 SizeY)
	{
		GridSize = FIntPoint(SizeX, SizeY);
		MaxHeightCm.SetNum(SizeX * SizeY);
		
		// Initialize as invalid height
		for (float& V : MaxHeightCm) V = -FLT_MAX;
	}

	// Check if cache data is valid and consistent
	UFUNCTION(BlueprintCallable, Category="Data")
	bool IsValid() const
	{
		return CellSizeCm > 0.0f && GridSize.X > 0 && GridSize.Y > 0 && MaxHeightCm.Num() == GridSize.X * GridSize.Y;
	}

	// Convert 2D cell coordinates to flat array index
	UFUNCTION(BlueprintCallable, Category="Data")
	int32 ToIndex(int32 X, int32 Y) const
	{
		return X + Y * GridSize.X;
	}

	// Get height above sea level in meters
	UFUNCTION(BlueprintCallable, Category="Data")
	float GetHeightMetersASL(int32 X, int32 Y) const
	{
		const int32 Idx = ToIndex(X, Y);
		if (!MaxHeightCm.IsValidIndex(Idx)) return 0.0f;
		return (MaxHeightCm[Idx] - SeaLevelWorldZCm) / 100.0f;
	}
};