// Flight pathfinder actor
// Header for aircraft-aware A* route search
// Uses voxel position plus heading direction as search state
// Checks terrain clearance, aircraft limits and influence zones
// Supports motion primitives, direct routes, caching and UI result output
// Stores route result, failure stats and debug draw settings

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
class AFlightWeatherZoneActor;

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

// Route failure statistics
// Counts why neighbor states or route checks failed
// For debugging 
USTRUCT()
struct FRouteFailureStats
{
	GENERATED_BODY()

	// OutOfBoundsCount: states outside search space
	int32 OutOfBoundsCount = 0;
	// InvalidTargetStateCount: target state could not be used
	int32 InvalidTargetStateCount = 0;
	// TerrainClearanceCount: terrain clearance rule failed
	int32 TerrainClearanceCount = 0;
	// HardBlockZoneCount: route touched forbidden zone
	int32 HardBlockZoneCount = 0;
	// MaxClimbExceededCount: climb rate too high
	int32 MaxClimbExceededCount = 0;
	// MaxDescentExceededCount: descent rate too high
	int32 MaxDescentExceededCount = 0;
	// TurnRadiusTooSmallCount: turn tighter than aircraft allows
	int32 TurnRadiusTooSmallCount = 0;
	// NoValidNeighborsCount: state had no usable next steps
	int32 NoValidNeighborsCount = 0;

	// Reset all debug counters before new search
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

// Open heap entry
// Stores state plus current F score for priority queue
USTRUCT()
struct FFlightOpenEntry
{
	GENERATED_BODY()

	// State: candidate waiting for A* expansion
	FFlightPathState State;

	// FScore: priority value used by heap
	float FScore = 0.0f;

	FFlightOpenEntry() = default;

	// Create heap entry from state and F score
	FFlightOpenEntry(const FFlightPathState& InState, float InFScore)
		: State(InState), FScore(InFScore)
	{
	}
};

// Heap compare rule -> Lower FScore should be popped first
struct FFlightOpenEntryMinHeapPredicate
{
	bool operator()(const FFlightOpenEntry& A, const FFlightOpenEntry& B) const
	{
		return A.FScore < B.FScore;
	}
};

// Transition cache key -> Identifies movement from one state to another
USTRUCT()
struct FFlightTransitionKey
{
	GENERATED_BODY()

	// From: start state of cached transition
	FFlightPathState From;
	
	// To: end state of cached transition
	FFlightPathState To;

	FFlightTransitionKey() = default;

	// Create key from two states
	FFlightTransitionKey(const FFlightPathState& InFrom, const FFlightPathState& InTo)
		: From(InFrom), To(InTo)
	{
	}

	// Compare full transition direction
	bool operator==(const FFlightTransitionKey& Other) const
	{
		return From == Other.From && To == Other.To;
	}
};

// Hash transition key for transition caches
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

	// bUseWeatherZones: true when the UI/weather option should affect route search
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Weather")
	bool bUseWeatherZones = false;

	// WeatherZones: static weather volumes collected from the current level
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Weather")
	TArray<TObjectPtr<AFlightWeatherZoneActor>> WeatherZones;

	// ScatteredCloudFlightLevelThresholdMetersASL: FL100 threshold in meters ASL
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudFlightLevelThresholdMetersASL = 3048.0f;

	// ScatteredCloudHorizontalClearanceBelowFL100Meters: VFR horizontal cloud distance below FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudHorizontalClearanceBelowFL100Meters = 1500.0f;

	// ScatteredCloudVerticalClearanceBelowFL100Meters: VFR vertical cloud distance below FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVerticalClearanceBelowFL100Meters = 300.0f;

	// ScatteredCloudHorizontalClearanceAboveFL100Meters: VFR horizontal cloud distance at or above FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudHorizontalClearanceAboveFL100Meters = 1500.0f;

	// ScatteredCloudVerticalClearanceAboveFL100Meters: VFR vertical cloud distance at or above FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVerticalClearanceAboveFL100Meters = 300.0f;

	// ScatteredCloudVisibilityBelowFL100Meters: reference VFR visibility below FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVisibilityBelowFL100Meters = 5000.0f;

	// ScatteredCloudVisibilityAboveFL100Meters: reference VFR visibility at or above FL100
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVisibilityAboveFL100Meters = 8000.0f;

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

	// FailureStats: counters for failed validation reasons
	UPROPERTY(VisibleAnywhere, Category="Debug")
	mutable FRouteFailureStats FailureStats;

	// LastFailureReason: latest high-level route failure reason
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

	// HeuristicWeight: A* goal bias, higher = faster but less optimal
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float HeuristicWeight = 1.0f;

	// SearchMaxExpandedStates: hard limit for A* search work
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="10000"))
	int32 SearchMaxExpandedStates = 1500000;

	// GoalConnectionToleranceMeters: distance where direct goal connection may be accepted
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float GoalConnectionToleranceMeters = 900.0f;

	// DirectRoutePreferredClearanceMaxLengthMeters: max length for direct preferred-clearance route check
	// 0 = disabled or no extra max length rule
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float DirectRoutePreferredClearanceMaxLengthMeters = 0.0f;

	// MinimumAbsoluteTerrainClearanceMeters: absolute lower safety clearance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float MinimumAbsoluteTerrainClearanceMeters = 50.0f;

	// TerrainClearanceSafetyMultiplier: multiplier for profile clearance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="1.0"))
	float TerrainClearanceSafetyMultiplier = 1.5f;

	// LateralTerrainSafetyRadiusMultiplier: horizontal radius for conservative terrain sampling
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float LateralTerrainSafetyRadiusMultiplier = 1.0f;

	// PreferredClearanceSpeedLookaheadSeconds: speed-based extra clearance lookahead
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float PreferredClearanceSpeedLookaheadSeconds = 6.0f;

	// MinOutputWaypointSpacingMeters: minimum spacing after route compaction
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Result", meta=(ClampMin="0.0"))
	float MinOutputWaypointSpacingMeters = 750.0f;

	// MaxOutputWaypointCompactionLookahead: max points tested during waypoint simplification
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Result", meta=(ClampMin="2", ClampMax="64"))
	int32 MaxOutputWaypointCompactionLookahead = 16;

	// CurrentRouteWorldPoints: final route as world-space positions
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Result")
	TArray<FVector> CurrentRouteWorldPoints;

	// LastExpandedStates: amount of states checked in last search
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

	// Collect static weather zones from the current level
	UFUNCTION(CallInEditor, BlueprintCallable, Category="Flight Pathfinding|Weather")
	void CollectWeatherZones();

	// PrimitiveSegmentLengthMeters: preferred motion primitive length
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float PrimitiveSegmentLengthMeters = 400.0f;

	// PrimitiveSamplesPerSegment: sample count for primitive validation
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="2", ClampMax="32"))
	int32 PrimitiveSamplesPerSegment = 8;

	// PrimitiveClimbRateFactor: safety factor for climb / descent inside primitive
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="0.1", ClampMax="1.0"))
	float PrimitiveClimbRateFactor = 0.85f;

	// MaxAutoPrimitiveSegmentLengthMeters: upper limit for automatic primitive length
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float MaxAutoPrimitiveSegmentLengthMeters = 2500.0f;

	// PrimitiveTurnDeltaBuckets: heading change used by primitive turns
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="1", ClampMax="4"))
	int32 PrimitiveTurnDeltaBuckets = 1;

	// Calculate route from UI input and return structured result
	UFUNCTION(BlueprintCallable, Category = "Flight Pathfinding")
	FRouteCalculationResult CalculateFlightRouteForUI(
		FVector StartWorldLocation,
		FVector TargetWorldLocation,
		float StartAltitudeMetersASL,
		float TargetAltitudeMetersASL,
		UFlightProfile* InFlightProfile
	);

	// Convert failure reason enum to UI text
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

	// Convert world position directly to state -> fails if position is outside grid or Z range
	bool WorldToStateExact(const FVector& WorldPos, int32 HeadingIndex, FFlightPathState& OutState) const;

	// Build sampled points for one motion primitive -> used to check terrain, altitude and zone safety along curved/straight movement
	bool BuildPrimitiveSamplePoints(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		TArray<FVector>& OutSamplePoints,
		int32& OutEndHeading
	) const;

	// Apply motion primitive and output resulting state -> uses heading delta and vertical mode to move aircraft-like
	bool ApplyMotionPrimitive(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		FFlightPathState& OutState
	) const;

	// Validate full motion primitive between states -> checks sampled path, climb/descent and aircraft limits
	bool IsMotionPrimitiveValid(
		const FFlightPathState& FromState,
		const FFlightPathState& ToState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode
	) const;

	// Get signed heading difference between buckets -> keeps left/right turn direction information
	int32 GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const;

	// Rebuild primitive samples for known state transition -> used for route validation or route reconstruction
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

	// Get conservative terrain height around world XY -> samples nearby terrain using horizontal safety radius
	bool GetConservativeTerrainHeightCmAtWorldXY(
		float WorldX,
		float WorldY,
		float HorizontalSafetyRadiusMeters,
		float& OutTerrainHeightCm
	) const;

	// Convert Unreal world Z to altitude in meters ASL
	float GetAltitudeMetersASLFromWorldZ(float WorldZCm) const;

	// Get terrain height at cell in meters ASL
	float GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const;

	// Get final required terrain clearance -> uses profile clearance and absolute minimum
	float GetRequiredTerrainClearanceMeters() const;

	// Get preferred terrain clearance for safer direct routes
	float GetPreferredTerrainClearanceMeters() const;

	// Get lateral safety radius for required terrain checks
	float GetTerrainSafetyRadiusMeters() const;

	// Get larger lateral safety radius for preferred clearance checks
	float GetPreferredTerrainSafetyRadiusMeters() const;

	// Get effective primitive length from aircraft and settings
	float GetEffectivePrimitiveSegmentLengthMeters() const;

	// Get maximum allowed world Z from profile max altitude
	float GetMaxAllowedWorldZCm() const;

	// Check if point is below aircraft max altitude
	bool DoesPointRespectAltitudeLimit(const FVector& WorldPoint) const;
	
	// Check if every route point respects max altitude
	bool DoesRouteRespectAltitudeLimit(const TArray<FVector>& RoutePoints) const;

	// Check altitude limit along a segment
	bool DoesSegmentRespectAltitudeLimit(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check if point has required terrain clearance
	bool DoesPointRespectTerrainClearance(const FVector& WorldPoint) const;

	// Check if point has preferred terrain clearance
	bool DoesPointRespectPreferredTerrainClearance(const FVector& WorldPoint) const;

	// Check preferred terrain clearance along a segment
	bool DoesSegmentRespectPreferredTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check direct segment against core flight rules
	bool DoesDirectSegmentRespectFlightRules(
		const FVector& FromWorld,
		const FVector& ToWorld,
		int32 FromHeadingIndex,
		int32 ToHeadingIndex
	) const;

	// Try simple direct VFR route before full A* search -> can skip expensive search if direct path is safe
	bool TryBuildDirectVfrRoute(
		const FVector& StartWorldLocation,
		const FVector& TargetWorldLocation,
		int32 StartHeadingIndex,
		TArray<FVector>& OutRoutePoints
	);

	// Check if current state can connect directly to goal -> used near the target to finish route early
	bool CanConnectToGoal(
		const FFlightPathState& Current,
		const FFlightPathState& Goal,
		const FVector& GoalWorldLocation,
		int32 GoalHeadingIndex
	) const;

	// Validate full route against turn radius limits
	bool DoesRouteRespectTurnRadius(const TArray<FVector>& RoutePoints) const;

	// Final safety validation for CurrentRouteWorldPoints
	bool ValidateCurrentRouteSafety() const;

	// Reduce waypoint count while keeping route safe
	void CompactCurrentRouteWaypoints();

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

	// Check if point is inside blocking thunderstorm or fog weather
	bool IsPointInsideBlockingWeatherZone(const FVector& WorldPoint) const;

	// Check if segment intersects blocking thunderstorm or fog weather
	bool DoesSegmentIntersectBlockingWeatherZone(const FVector& FromWorld, const FVector& ToWorld) const;

	// Get scattered-cloud clearance for the current altitude band
	void GetScatteredCloudClearanceForAltitude(
		float AltitudeMetersASL,
		float& OutHorizontalClearanceMeters,
		float& OutVerticalClearanceMeters
	) const;

	// Check if point violates scattered-cloud VFR clearance
	bool DoesPointViolateScatteredCloudClearance(const FVector& WorldPoint) const;

	// Check if segment violates scattered-cloud VFR clearance
	bool DoesSegmentViolateScatteredCloudClearance(const FVector& FromWorld, const FVector& ToWorld) const;


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

	// Pop best open state from heap -> skips outdated heap entries using current records
	bool PopBestOpenStateFromHeap(
		TArray<FFlightOpenEntry>& OpenHeap,
		const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
		FFlightPathState& OutBestState
	) const;

	// Cached transition validity lookup -> avoids repeated expensive validation checks
	bool IsTransitionValidCached(const FFlightPathState& From, const FFlightPathState& To) const;
	
	// Cached transition cost lookup -> avoids repeated soft-zone and movement cost calculation
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

	// TransitionValidityCache: stores valid/invalid result per transition
	mutable TMap<FFlightTransitionKey, bool> TransitionValidityCache;

	// TransitionCostCache: stores movement cost per transition
	mutable TMap<FFlightTransitionKey, float> TransitionCostCache;

	// ConservativeTerrainHeightCache: stores sampled max terrain height per area
	mutable TMap<FIntVector, float> ConservativeTerrainHeightCache;

	// Calculate penalty for alternating turn direction -> helps avoid nervous left-right-left route shapes
	float CalculateTurnReversalPenalty(
	const FFlightPathState& GrandParent,
	const FFlightPathState& Parent,
	const FFlightPathState& Current
) const;

	// Run internal flight route search
	bool RunFlightRouteSearch(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	UFlightProfile* InFlightProfile,
	TArray<FVector>& OutRoutePoints
);
	// Validate references that are always required
	bool ValidateCoreReferences() const;

	// Build search space around a specific route request -> uses start and target location instead of only actor references
	void BuildSearchSpaceForRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation
);
};