// Flight pathfinder actor
// Header for aircraft-aware A* route search
// Uses voxel position plus heading direction as search state
// Checks terrain clearance, aircraft limits and influence zones
// Stores route result and debug draw settings

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RouteFailureReason.h"
#include "RouteCalculationResult.h"
#include "FlightPathfinderActor.generated.h"

class AVoxelGridBaker;
class UVoxelHeightCache;
class UFlightProfile;
class AFlightInfluenceZoneActor;

// Flight search state
// X / Y / Z = voxel position
// HeadingIndex = discretized flight direction
USTRUCT(BlueprintType)
struct FFlightPathState
{
	GENERATED_BODY()

	// X: grid column
	UPROPERTY(VisibleAnywhere)
	int32 X = 0;

	// Y: grid row
	UPROPERTY(VisibleAnywhere)
	int32 Y = 0;

	// Z: vertical voxel layer
	UPROPERTY(VisibleAnywhere)
	int32 Z = 0;

	// HeadingIndex: current direction bucket
	UPROPERTY(VisibleAnywhere)
	int32 HeadingIndex = 0;

	FFlightPathState() = default;

	// Create state from voxel position and heading
	FFlightPathState(int32 InX, int32 InY, int32 InZ, int32 InHeadingIndex)
		: X(InX), Y(InY), Z(InZ), HeadingIndex(InHeadingIndex)
	{
	}

	// Compare position and heading
	bool operator==(const FFlightPathState& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z && HeadingIndex == Other.HeadingIndex;
	}
};

USTRUCT()
struct FRouteFailureStats
{
	GENERATED_BODY()

	int32 OutOfBoundsCount = 0;
	int32 InvalidTargetStateCount = 0;
	int32 TerrainClearanceCount = 0;
	int32 HardBlockZoneCount = 0;
	int32 MaxClimbExceededCount = 0;
	int32 MaxDescentExceededCount = 0;
	int32 TurnRadiusTooSmallCount = 0;
	int32 NoValidNeighborsCount = 0;

	void Reset()
	{
		OutOfBoundsCount = 0;
		InvalidTargetStateCount = 0;
		TerrainClearanceCount = 0;
		HardBlockZoneCount = 0;
		MaxClimbExceededCount = 0;
		MaxDescentExceededCount = 0;
		TurnRadiusTooSmallCount = 0;
		NoValidNeighborsCount = 0;
	}
};

// Hash flight state for TMap / TSet usage
FORCEINLINE uint32 GetTypeHash(const FFlightPathState& State)
{
	uint32 Hash = GetTypeHash(State.X);
	Hash = HashCombine(Hash, GetTypeHash(State.Y));
	Hash = HashCombine(Hash, GetTypeHash(State.Z));
	Hash = HashCombine(Hash, GetTypeHash(State.HeadingIndex));
	return Hash;
}

// A* node data for one flight state
// Stores cost values, parent link and closed state
USTRUCT()
struct FFlightPathNodeRecord
{
	GENERATED_BODY()

	// G: cost from start to this state
	float G = TNumericLimits<float>::Max();

	// H: estimated cost from this state to goal
	float H = 0.0f;

	// F: total priority cost, G + HeuristicWeight * H
	float F = TNumericLimits<float>::Max();

	// Parent: previous state in best known route
	FFlightPathState Parent;

	// bHasParent: false for start state
	bool bHasParent = false;

	// bClosed: already fully checked by A*
	bool bClosed = false;
};

USTRUCT()
struct FFlightOpenEntry
{
	GENERATED_BODY()

	FFlightPathState State;
	float FScore = 0.0f;

	FFlightOpenEntry() = default;

	FFlightOpenEntry(const FFlightPathState& InState, float InFScore)
		: State(InState), FScore(InFScore)
	{
	}
};

struct FFlightOpenEntryMinHeapPredicate
{
	bool operator()(const FFlightOpenEntry& A, const FFlightOpenEntry& B) const
	{
		return A.FScore < B.FScore;
	}
};

USTRUCT()
struct FFlightTransitionKey
{
	GENERATED_BODY()

	FFlightPathState From;
	FFlightPathState To;

	FFlightTransitionKey() = default;

	FFlightTransitionKey(const FFlightPathState& InFrom, const FFlightPathState& InTo)
		: From(InFrom), To(InTo)
	{
	}

	bool operator==(const FFlightTransitionKey& Other) const
	{
		return From == Other.From && To == Other.To;
	}
};

FORCEINLINE uint32 GetTypeHash(const FFlightTransitionKey& Key)
{
	uint32 Hash = GetTypeHash(Key.From);
	Hash = HashCombine(Hash, GetTypeHash(Key.To));
	return Hash;
}

UCLASS()
class MAJORPROJECT_API AFlightPathfinderActor : public AActor
{
	GENERATED_BODY()

public:
	AFlightPathfinderActor();

	// GridBaker: source actor for baked terrain grid
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AVoxelGridBaker> GridBaker;

	// HeightCache: baked terrain height data for route validation
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	// FlightProfile: aircraft limits and safety values
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UFlightProfile> FlightProfile;

	// StartActor: route start position
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> StartActor;

	// GoalActor: route target position
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> GoalActor;

	// InfluenceZones: hard blocks or soft cost areas for route search
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TArray<TObjectPtr<AFlightInfluenceZoneActor>> InfluenceZones;

	// VoxelSizeZMeters: vertical height of one search layer
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float VoxelSizeZMeters = 25.0f;

	// ExtraBottomLayers: added layers below relevant terrain / route height
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0"))
	int32 ExtraBottomLayers = 0;

	// ExtraTopLayers: added layers above relevant terrain / route height
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0"))
	int32 ExtraTopLayers = 8;

	// HeadingBucketCount: number of possible flight directions
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="4", ClampMax="32"))
	int32 HeadingBucketCount = 16;

	// MaxHeadingChangePerStep: max direction change per search step
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="0", ClampMax="4"))
	int32 MaxHeadingChangePerStep = 1;

	// bAllowClimbStep: allow neighbor states one layer higher
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model")
	bool bAllowClimbStep = true;

	// bAllowDescentStep: allow neighbor states one layer lower
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model")
	bool bAllowDescentStep = true;

	// bDrawVisitedStates: draw checked A* states for debugging
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	bool bDrawVisitedStates = false;

	// bAutoDrawPathAfterSearch: draw route after successful search
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	bool bAutoDrawPathAfterSearch = true;

	// DebugDrawLifetime: seconds until temporary debug shapes disappear
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugDrawLifetime = 20.0f;

	// DebugLineThickness: route line thickness
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugLineThickness = 6.0f;

	// DebugVisitedPointSize: point size for visited states
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugVisitedPointSize = 16.0f;
	
	UPROPERTY(VisibleAnywhere, Category="Debug")
	mutable FRouteFailureStats FailureStats;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Debug")
	mutable ERouteFailureReason LastFailureReason = ERouteFailureReason::None;

	// SearchMinWorld: minimum world-space bounds of search space
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	FVector SearchMinWorld = FVector::ZeroVector;

	// SearchMaxWorld: maximum world-space bounds of search space
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	FVector SearchMaxWorld = FVector::ZeroVector;

	// ZLayerCount: amount of vertical search layers
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 ZLayerCount = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float HeuristicWeight = 1.2f;

	// CurrentRouteWorldPoints: final route as world-space positions
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Result")
	TArray<FVector> CurrentRouteWorldPoints;

	UPROPERTY(BlueprintReadOnly, Category = "Flight Pathfinding|Debug")
	int32 LastExpandedStates = 0;

	// Build 3D search space from terrain cache
	UFUNCTION(CallInEditor, Category="Flight Pathfinding")
	void BuildSearchSpace();

	// Run aircraft-aware A* route search
	UFUNCTION(CallInEditor, Category="Flight Pathfinding")
	void FindFlightRoute();

	// Draw current route in viewport
	UFUNCTION(CallInEditor, Category="Flight Pathfinding")
	void DebugDrawCurrentRoute();

	// Clear route result and debug drawings
	UFUNCTION(CallInEditor, Category="Flight Pathfinding")
	void ClearCurrentRoute();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float PrimitiveSegmentLengthMeters = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="2", ClampMax="32"))
	int32 PrimitiveSamplesPerSegment = 8;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="0.1", ClampMax="1.0"))
	float PrimitiveClimbRateFactor = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="1", ClampMax="4"))
	int32 PrimitiveTurnDeltaBuckets = 1;

	UFUNCTION(BlueprintCallable, Category = "Flight Pathfinding")
	FRouteCalculationResult CalculateFlightRouteForUI(
		FVector StartWorldLocation,
		FVector TargetWorldLocation,
		float StartAltitudeMetersASL,
		float TargetAltitudeMetersASL,
		UFlightProfile* InFlightProfile
	);

	UFUNCTION(BlueprintCallable, Category = "Flight Pathfinding")
	FText GetFailureReasonText(ERouteFailureReason Reason) const;

protected:
	// Check required references and profile values
	bool ValidateReferences() const;

	// Check state inside built search bounds
	bool IsStateInsideBounds(const FFlightPathState& State) const;

	// Convert state to world-space center position
	FVector StateToWorldCenter(const FFlightPathState& State) const;

	// Find nearest valid state around world position
	bool WorldToNearestValidState(const FVector& WorldPos, int32 PreferredHeadingIndex,
	                              FFlightPathState& OutState) const;
	
	bool WorldToStateExact(const FVector& WorldPos, int32 HeadingIndex, FFlightPathState& OutState) const;

	bool BuildPrimitiveSamplePoints(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		TArray<FVector>& OutSamplePoints,
		int32& OutEndHeading
	) const;

	bool ApplyMotionPrimitive(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		FFlightPathState& OutState
	) const;

	bool IsMotionPrimitiveValid(
		const FFlightPathState& FromState,
		const FFlightPathState& ToState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode
	) const;

	int32 GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const;

	bool RebuildPrimitiveSamplesBetweenStates(
		const FFlightPathState& FromState,
		const FFlightPathState& ToState,
		TArray<FVector>& OutSamplePoints
	) const;

	// Check if current state reaches goal position
	bool IsGoalState(const FFlightPathState& Current, const FFlightPathState& Goal) const;

	// Normalize heading index into valid bucket range
	int32 NormalizeHeadingIndex(int32 HeadingIndex) const;

	// Convert heading bucket to angle in radians
	float HeadingIndexToAngleRad(int32 HeadingIndex) const;

	// Convert heading bucket to XY grid step
	FIntPoint HeadingIndexToGridStep(int32 HeadingIndex) const;

	// Get closest heading bucket from 2D direction
	int32 ComputeNearestHeadingIndexFromDirection(const FVector2D& Direction) const;

	// Get shortest signed / wrapped heading difference
	int32 GetSmallestHeadingDelta(int32 FromHeading, int32 ToHeading) const;

	// Get cached terrain height at grid cell
	float GetTerrainHeightCmAtCell(int32 X, int32 Y) const;

	// Get terrain height from world X / Y position
	bool GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const;

	// Convert Unreal world Z to altitude in meters ASL
	float GetAltitudeMetersASLFromWorldZ(float WorldZCm) const;

	// Get terrain height at cell in meters ASL
	float GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const;

	// Check if state is flyable and safe
	bool IsStateValid(const FFlightPathState& State) const;

	// Check if movement from one state to another is allowed
	bool IsTransitionValid(const FFlightPathState& From, const FFlightPathState& To) const;

	// Check terrain clearance along segment
	bool DoesSegmentRespectTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check if point is inside hard block zone
	bool IsStateInsideHardBlockZone(const FVector& WorldPoint, float AltitudeMetersASL) const;

	// Check if segment intersects hard block zone
	bool DoesSegmentIntersectHardBlockZone(
		const FVector& FromWorld,
		const FVector& ToWorld,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL
	) const;

	// Calculate extra cost for soft influence zones
	float GetSoftZoneTraversalCost(
		const FVector& FromWorld,
		const FVector& ToWorld,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL
	) const;

	// Generate valid neighbor states for A*
	void GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors) const;

	// Estimate remaining A* cost to goal
	float HeuristicCost(const FFlightPathState& A, const FFlightPathState& B) const;

	// Calculate movement cost between two states
	float TransitionCost(const FFlightPathState& From, const FFlightPathState& To) const;

	bool PopBestOpenStateFromHeap(
		TArray<FFlightOpenEntry>& OpenHeap,
		const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
		FFlightPathState& OutBestState
	) const;

	bool IsTransitionValidCached(const FFlightPathState& From, const FFlightPathState& To) const;
	float TransitionCostCached(const FFlightPathState& From, const FFlightPathState& To) const;

	// Rebuild route from A* parent records
	void ReconstructRoute(
		const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
		const FFlightPathState& GoalState
	);

	// Calculate total horizontal / 3D route length
	float CalculateRouteLengthMeters() const;

	// Calculate altitude difference from start to goal
	float CalculateNetAltitudeChangeMeters() const;

	// Calculate summed positive climb over route
	float CalculateTotalClimbMeters() const;

	mutable TMap<FFlightTransitionKey, bool> TransitionValidityCache;
	mutable TMap<FFlightTransitionKey, float> TransitionCostCache;

	float CalculateTurnReversalPenalty(
	const FFlightPathState& GrandParent,
	const FFlightPathState& Parent,
	const FFlightPathState& Current
) const;

	bool RunFlightRouteSearch(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	UFlightProfile* InFlightProfile,
	TArray<FVector>& OutRoutePoints
);

	bool ValidateCoreReferences() const;

	void BuildSearchSpaceForRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation
);
};
