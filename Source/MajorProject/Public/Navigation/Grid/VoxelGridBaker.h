// Core grid and baking actor for voxel-based terrain analysis
// Builds grid over landscape, bakes max heights, and provides debug previews

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "VoxelGridBaker.generated.h"

class ATerrainReferenceActor;
class UVoxelGridConfig;
class UVoxelHeightCache;
class AHeightQueryProbeActor;

UCLASS()
class MAJORPROJECT_API AVoxelGridBaker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AVoxelGridBaker();

	// Landscape reference used to build grid bounds
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<ATerrainReferenceActor> TerrainRef;

	// Grid and baking configuration data
	UPROPERTY(EditAnywhere, Category="Config")
	TObjectPtr<UVoxelGridConfig> GridConfig;

	// Output cache storing baked max heights
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Output")
	UVoxelHeightCache* HeightCache = nullptr;

	// Grid minimum world-space corner (XY)
	UPROPERTY(VisibleAnywhere, Category="Grid")
	FVector GridMinWorld = FVector::ZeroVector;

	// Grid maximum world-space corner (XY)
	UPROPERTY(VisibleAnywhere, Category="Grid")
	FVector GridMaxWorld = FVector::ZeroVector;

	// Grid resolution in cells (X,Y)
	UPROPERTY(VisibleAnywhere, Category="Grid")
	FIntPoint GridSize = FIntPoint(0, 0);

	// Grid cell size in centimeters
	UPROPERTY(VisibleAnywhere, Category="Grid")
	float CellSizeCm = 0.0f;

	// Build grid layout from landscape bounds
	UFUNCTION(CallInEditor, Category="Voxel | Grid")
	void BuildGrid();

	// Draw grid outline for debugging
	UFUNCTION(CallInEditor, Category="Voxel | Grid")
	void DebugDrawGridOutline();

	// Draw a limited number of grid cells
	UFUNCTION(CallInEditor, Category="Voxel | Grid")
	void DebugDrawSomeCells(int32 MaxCellsToDraw = 1000);

	// Convenience: draw preset number of cells
	UFUNCTION(CallInEditor, Category="Voxel | Grid")
	void DebugDrawSomeCells50();

	// Bake per-cell maximum heights using line traces
	UFUNCTION(CallInEditor, Category="Voxel|Bake")
	void BakeMaxHeights();

	
	// Preview Voxels (nur Debug/Visual):

	// Optional center actor for preview area
	UPROPERTY(EditAnywhere, Category="Preview")
	AHeightQueryProbeActor* PreviewCenterActor = nullptr;

	// Preview radius in grid cells
	UPROPERTY(EditAnywhere, Category="Preview", meta=(ClampMin="1", ClampMax="200"))
	int32 PreviewRadiusCells = 20; // 20 = 41x41 cells

	// Use cached sea level as preview base height
	UPROPERTY(EditAnywhere, Category="Preview")
	bool bPreviewUseSeaLevelAsBase = true;

	// Manual base height override (cm)
	UPROPERTY(EditAnywhere, Category="Preview")
	float PreviewBaseZOverrideCm = 0.0f; 

	// Build voxel column preview
	UFUNCTION(CallInEditor, Category="Voxel|Preview")
	void BuildPreviewVoxels();

	// Clear existing preview voxels
	UFUNCTION(CallInEditor, Category="Voxel|Preview")
	void ClearPreviewVoxels();

	// Color of cubes
	UPROPERTY(EditAnywhere, Category="Preview")
	UMaterialInterface* PreviewMaterial = nullptr;

private:
	// Validate grid parameters
	bool IsGridValid() const;

	// Compute world-space XY bounds for a grid cell
	void GetCellMinMaxXY(const int32 X, const int32 Y, FVector2D& OutMin, FVector2D& OutMax) const;

	// Instanced mesh component for voxel preview
	UPROPERTY(Transient)
	UHierarchicalInstancedStaticMeshComponent* PreviewHISM = nullptr;

	// Resolve preview base Z height (cm)
	float GetPreviewBaseZCm() const;
};