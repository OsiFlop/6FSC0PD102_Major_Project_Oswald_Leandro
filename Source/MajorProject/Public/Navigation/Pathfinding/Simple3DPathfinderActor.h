// Simple 3D pathfinder actor
// Header for voxel-based A* pathfinding
// Stores grid references, voxel settings, debug values and result path
// Supports terrain checks, corner cutting prevention and path smoothing

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simple3DPathfinderActor.generated.h"

class AVoxelGridBaker;
class UVoxelHeightCache;
class UFlightProfile;

// Voxel grid coordinate
// X / Y = horizontal grid cell
// Z = vertical voxel layer
USTRUCT(BlueprintType)
struct FGridVoxelCoord
{
	GENERATED_BODY()

	// X: grid column
	UPROPERTY(VisibleAnywhere)
	int32 X = 0;

	// Y: grid row
	UPROPERTY(VisibleAnywhere)
	int32 Y = 0;

	// Z: vertical layer
	UPROPERTY(VisibleAnywhere)
	int32 Z = 0;

	FGridVoxelCoord() = default;

	// Create voxel coordinate from X, Y, Z values
	FGridVoxelCoord(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ)
	{
	}

	// Compare two voxel coordinates
	bool operator==(const FGridVoxelCoord& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}
};

// Hash voxel coordinate for TMap / TSet usage
FORCEINLINE uint32 GetTypeHash(const FGridVoxelCoord& Coord)
{
	uint32 Hash = GetTypeHash(Coord.X);
	Hash = HashCombine(Hash, GetTypeHash(Coord.Y));
	Hash = HashCombine(Hash, GetTypeHash(Coord.Z));
	return Hash;
}

// A* node data for one voxel
// Stores cost values, parent link and closed state
USTRUCT()
struct FGridVoxelNodeRecord
{
	GENERATED_BODY()

	// G: cost from start to this voxel
	float G = TNumericLimits<float>::Max();

	// H: estimated cost from this voxel to goal
	float H = 0.0f;

	// F: total priority cost, G + H
	float F = TNumericLimits<float>::Max();

	// Parent: previous voxel in best known route
	FGridVoxelCoord Parent;

	// bHasParent: false for start node
	bool bHasParent = false;

	// bClosed: already fully checked by A*
	bool bClosed = false;
};

UCLASS()
class MAJORPROJECT_API ASimple3DPathfinderActor : public AActor
{
	GENERATED_BODY()
	
public:
	ASimple3DPathfinderActor();

	// GridBaker: source actor for baked terrain grid
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AVoxelGridBaker> GridBaker;

	// HeightCache: baked terrain height data used for voxel blocking
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	// StartActor: route start position
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AActor> StartActor;

	// GoalActor: route target position
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AActor> GoalActor;

	// VoxelSizeZMeters: vertical height of one voxel layer
	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="1.0"))
	float VoxelSizeZMeters = 100.0f;

	// ExtraBottomLayers: added layers below relevant terrain / actor height
	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="0"))
	int32 ExtraBottomLayers = 1;

	// ExtraTopLayers: added layers above relevant terrain / actor height
	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="0"))
	int32 ExtraTopLayers = 5;

	// bDrawVisitedNodes: draw checked A* nodes for debugging
	UPROPERTY(EditAnywhere, Category="Debug")
	bool bDrawVisitedNodes = false;

	// DebugDrawLifetime: seconds until debug shapes disappear
	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugDrawLifetime = 30.0f;

	// DebugLineThickness: path line thickness
	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugLineThickness = 8.0f;

	// DebugVisitedNodeSize: debug point size for visited nodes
	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugVisitedNodeSize = 20.0f;

	// VoxelMinWorld: minimum world-space bounds of built voxel space
	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	FVector VoxelMinWorld = FVector::ZeroVector;

	// VoxelMaxWorld: maximum world-space bounds of built voxel space
	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	FVector VoxelMaxWorld = FVector::ZeroVector;

	// ZLayerCount: amount of vertical voxel layers
	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	int32 ZLayerCount = 0;

	// CurrentPathWorldPoints: final route as world-space positions
	UPROPERTY(VisibleAnywhere, Category="Result")
	TArray<FVector> CurrentPathWorldPoints;

	// Build voxel search space from height cache
	UFUNCTION(CallInEditor, Category="Pathfinding")
	void BuildVoxelSpace();

	// Run A* pathfinding from start to goal
	UFUNCTION(CallInEditor, Category="Pathfinding")
	void FindPath();

	// Draw current path in viewport
	UFUNCTION(CallInEditor, Category="Pathfinding")
	void DebugDrawCurrentPath();

	// bPreventCornerCutting: block diagonal moves through blocked corners
	UPROPERTY(EditAnywhere, Category="Pathfinding")
	bool bPreventCornerCutting = true;

	// bEnablePathSmoothing: simplify path after A* result
	UPROPERTY(EditAnywhere, Category="Pathfinding")
	bool bEnablePathSmoothing = true;

	// MaxSmoothSkipPoints: max path points skipped during smoothing check
	UPROPERTY(EditAnywhere, Category="Pathfinding|Smoothing", meta=(ClampMin="1", ClampMax="20"))
	int32 MaxSmoothSkipPoints = 3;

	// Clear stored path and debug result data
	UFUNCTION(CallInEditor, Category="Pathfinding")
	void ClearCurrentPath();

	// FlightProfile: aircraft limits used for route validation
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UFlightProfile> FlightProfile;

private:
	// Check required actors and cached data
	bool ValidateReferences() const;

	// Convert world position to exact voxel coordinate
	bool WorldToVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const;
	
	// Find nearest unblocked voxel at world position
	bool WorldToNearestFreeVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const;

	// Convert voxel coordinate to world-space center
	FVector VoxelToWorldCenter(const FGridVoxelCoord& Voxel) const;

	// Check voxel inside built search bounds
	bool IsVoxelInsideBounds(const FGridVoxelCoord& Voxel) const;

	// Check voxel blocked by terrain or invalid cache data
	bool IsVoxelBlocked(const FGridVoxelCoord& Voxel) const;

	// Get cached terrain height at grid cell
	float GetTerrainHeightCmAtCell(const int32 X, const int32 Y) const;

	// Estimate remaining A* cost to goal
	float HeuristicCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const;

	// Calculate movement cost between two voxels
	float MovementCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const;

	// Get valid neighbor voxels around center
	void GetNeighbors(const FGridVoxelCoord& Center, TArray<FGridVoxelCoord>& OutNeighbors) const;

	// Rebuild path from A* parent records
	void ReconstructPath(
		const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
		const FGridVoxelCoord& GoalVoxel
	);

	// Check if movement from one voxel to another is allowed
	bool IsMoveAllowed(const FGridVoxelCoord& From, const FGridVoxelCoord& To) const;

	// Get terrain height from world X / Y position
	bool GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const;

	// Find open A* node with lowest F cost
	bool FindBestOpenNode(
		const TSet<FGridVoxelCoord>& OpenSet,
		const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
		FGridVoxelCoord& OutBestNode
	) const;

	// Check if straight path segment is free and safe
	bool CanTravelDirect(const FVector& From, const FVector& To) const;

	// Simplify path by removing unnecessary intermediate points
	void SmoothPath();
};