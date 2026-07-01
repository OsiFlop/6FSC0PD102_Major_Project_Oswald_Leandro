// Flight pathfinder actor
// Header for aircraft-aware A* route search
// Uses voxel position plus heading direction as search state
// Checks terrain clearance, aircraft limits, weather and influence zones
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

// Search state used by A*
USTRUCT(BlueprintType)
struct FFlightPathState
{
	GENERATED_BODY()

	// Grid column
	UPROPERTY(VisibleAnywhere)
	int32 X = 0;

	// Grid row
	UPROPERTY(VisibleAnywhere)
	int32 Y = 0;

	// Vertical search layer
	UPROPERTY(VisibleAnywhere)
	int32 Z = 0;

	// Discrete flight direction
	UPROPERTY(VisibleAnywhere)
	int32 HeadingIndex = 0;

	FFlightPathState() = default;

	// Create state from voxel position and heading
	FFlightPathState(int32 InX, int32 InY, int32 InZ, int32 InHeadingIndex)
		: X(InX), Y(InY), Z(InZ), HeadingIndex(InHeadingIndex)
	{
	}

	// Compare full search state
	bool operator==(const FFlightPathState& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z && HeadingIndex == Other.HeadingIndex;
	}
};

// Debug counters for failed route checks
USTRUCT()
struct FRouteFailureStats
{
	GENERATED_BODY()

	// State was outside search space
	int32 OutOfBoundsCount = 0;

	// Target state could not be used
	int32 InvalidTargetStateCount = 0;

	// Terrain clearance rule failed
	int32 TerrainClearanceCount = 0;

	// Route touched forbidden airspace
	int32 HardBlockZoneCount = 0;

	// Climb rate was too high
	int32 MaxClimbExceededCount = 0;

	// Descent rate was too high
	int32 MaxDescentExceededCount = 0;

	// Turn was tighter than aircraft allows
	int32 TurnRadiusTooSmallCount = 0;

	// State had no usable next steps
	int32 NoValidNeighborsCount = 0;

	// Reset counters before a new search
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

// Hash flight state for maps and sets
FORCEINLINE uint32 GetTypeHash(const FFlightPathState& State)
{
	uint32 Hash = GetTypeHash(State.X);
	Hash = HashCombine(Hash, GetTypeHash(State.Y));
	Hash = HashCombine(Hash, GetTypeHash(State.Z));
	Hash = HashCombine(Hash, GetTypeHash(State.HeadingIndex));
	return Hash;
}

// A* node data for one search state
USTRUCT()
struct FFlightPathNodeRecord
{
	GENERATED_BODY()

	// Cost from start to this state
	float G = TNumericLimits<float>::Max();

	// Estimated cost from this state to goal
	float H = 0.0f;

	// Total priority cost
	float F = TNumericLimits<float>::Max();

	// Previous state in best known route
	FFlightPathState Parent;

	// Start state has no parent
	bool bHasParent = false;

	// State was already fully processed
	bool bClosed = false;
};

// Priority queue entry for A*
USTRUCT()
struct FFlightOpenEntry
{
	GENERATED_BODY()

	// Candidate waiting for expansion
	FFlightPathState State;

	// Priority value used by heap
	float FScore = 0.0f;

	FFlightOpenEntry() = default;

	// Create heap entry from state and score
	FFlightOpenEntry(const FFlightPathState& InState, float InFScore)
		: State(InState), FScore(InFScore)
	{
	}
};

struct FFlightOpenEntryMinHeapPredicate
{
	bool operator()(const FFlightOpenEntry& A, const FFlightOpenEntry& B) const
	{
		return A.FScore > B.FScore;
	}
};

// Cache key for one state transition
USTRUCT()
struct FFlightTransitionKey
{
	GENERATED_BODY()

	// Transition start state
	FFlightPathState From;

	// Transition end state
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

	// Source actor for baked terrain grid
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AVoxelGridBaker> GridBaker;

	// Baked terrain height data for route validation
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	// Aircraft limits and safety values
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<UFlightProfile> FlightProfile;

	// Actor used as editor route start
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> StartActor;

	// Actor used as editor route target
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TObjectPtr<AActor> GoalActor;

	// Hard blocks or soft cost areas for route search
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="References")
	TArray<TObjectPtr<AFlightInfluenceZoneActor>> InfluenceZones;

	// Enable restricted and influence zones for route search
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Influence")
	bool bUseInfluenceZones = true;

	// Horizontal no-go buffer around hard influence zones
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Influence|Hard Blocks", meta=(ClampMin="0.0"))
	float InfluenceZoneHorizontalAvoidanceMeters = 500.0f;

	// Vertical no-go buffer around hard influence zones
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Influence|Hard Blocks", meta=(ClampMin="0.0"))
	float InfluenceZoneVerticalAvoidanceMeters = 300.0f;

	// Enable weather zones for route search
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Weather")
	bool bUseWeatherZones = false;

	// Static weather volumes collected from the level
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Weather")
	TArray<TObjectPtr<AFlightWeatherZoneActor>> WeatherZones;

	// Altitude threshold for VFR cloud clearance rules
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudFlightLevelThresholdMetersASL = 3048.0f;

	// Horizontal cloud clearance below threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudHorizontalClearanceBelowFL100Meters = 1500.0f;

	// Vertical cloud clearance below threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVerticalClearanceBelowFL100Meters = 300.0f;

	// Horizontal cloud clearance at or above threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudHorizontalClearanceAboveFL100Meters = 1500.0f;

	// Vertical cloud clearance at or above threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVerticalClearanceAboveFL100Meters = 300.0f;

	// Reference VFR visibility below threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVisibilityBelowFL100Meters = 5000.0f;

	// Reference VFR visibility at or above threshold altitude
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Scattered Clouds", meta=(ClampMin="0.0"))
	float ScatteredCloudVisibilityAboveFL100Meters = 8000.0f;

	// Horizontal no-go buffer around thunderstorms
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Hard Blocks", meta=(ClampMin="0.0"))
	float ThunderstormHorizontalAvoidanceMeters = 5000.0f;

	// Vertical no-go buffer around thunderstorms
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Hard Blocks", meta=(ClampMin="0.0"))
	float ThunderstormVerticalAvoidanceMeters = 20000.0f;

	// Horizontal no-go buffer around fog
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Hard Blocks", meta=(ClampMin="0.0"))
	float FogHorizontalAvoidanceMeters = 1000.0f;

	// Vertical no-go buffer around fog
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Weather|Hard Blocks", meta=(ClampMin="0.0"))
	float FogVerticalAvoidanceMeters = 300.0f;

	// Vertical height of one search layer
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float VoxelSizeZMeters = 25.0f;

	// Extra layers below relevant terrain and route height
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0"))
	int32 ExtraBottomLayers = 0;

	// Extra layers above relevant terrain and route height
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0"))
	int32 ExtraTopLayers = 32;

	// Number of possible flight directions
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="4", ClampMax="32"))
	int32 HeadingBucketCount = 16;

	// Enable several movement lengths per A* state
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model|Adaptive Search")
	bool bUseAdaptivePrimitiveLengths = true;

	// Short movement multiplier for tight local corrections
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model|Adaptive Search", meta=(ClampMin="0.2", ClampMax="1.0"))
	float ShortPrimitiveLengthMultiplier = 0.55f;

	// Long movement multiplier for early detours and early climb planning
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model|Adaptive Search", meta=(ClampMin="1.0", ClampMax="6.0"))
	float LongPrimitiveLengthMultiplier = 3.0f;

	// Optional extra-long movement used only when obstacle-aware routing is active.
	// Kept small enough to fit the ~16km x 12km map without overshooting the grid edge.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model|Adaptive Search", meta=(ClampMin="1.0", ClampMax="10.0"))
	float ObstacleAwareLongPrimitiveLengthMultiplier = 2.5f;

	// Maximum direction change per search step
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="0", ClampMax="4"))
	int32 MaxHeadingChangePerStep = 1;

	// Allow neighbor states one layer higher
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model")
	bool bAllowClimbStep = true;

	// Allow neighbor states one layer lower
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model")
	bool bAllowDescentStep = true;

	// Draw checked A* states for debugging
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	bool bDrawVisitedStates = false;

	// Draw route after successful search
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug")
	bool bAutoDrawPathAfterSearch = true;

	// Lifetime of debug shapes in seconds
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugDrawLifetime = 20.0f;

	// Thickness of drawn route lines
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugLineThickness = 6.0f;

	// Size of visited-state debug points
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Debug", meta=(ClampMin="0.0"))
	float DebugVisitedPointSize = 16.0f;

	// Counters for failed validation reasons
	UPROPERTY(VisibleAnywhere, Category="Debug")
	mutable FRouteFailureStats FailureStats;

	// Latest high-level route failure reason
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Debug")
	mutable ERouteFailureReason LastFailureReason = ERouteFailureReason::None;

	// Minimum world-space bounds of search space
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	FVector SearchMinWorld = FVector::ZeroVector;

	// Maximum world-space bounds of search space
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	FVector SearchMaxWorld = FVector::ZeroVector;

	// Number of vertical search layers
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 ZLayerCount = 0;

	// Highest baked terrain height (world Z, cm) near the direct start-target line, for HeuristicCost
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	float RouteTerrainPeakWorldZ = 0.0f;

	// Current XY search corridor minimum column in terrain grid cell indices
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 SearchMinGridX = 0;

	// Current XY search corridor minimum row in terrain grid cell indices
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 SearchMinGridY = 0;

	// Current XY search corridor maximum column in terrain grid cell indices (exclusive)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 SearchMaxGridX = 0;

	// Current XY search corridor maximum row in terrain grid cell indices (exclusive)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Search Space")
	int32 SearchMaxGridY = 0;

	// A* goal bias, higher is faster but less optimal
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float HeuristicWeight = 1.5f;

	// Minimum A* goal bias when obstacles are active
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float ObstacleAwareHeuristicWeight = 2.0f;

	// Hard limit for A* search work
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="10000"))
	int32 SearchMaxExpandedStates = 1500000;

	// Reject a found route longer than this multiple of the straight-line start-goal distance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float MaxRouteLengthToStraightLineRatio = 16.0f;

	// Minimum allowed route length regardless of the ratio above
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float MinAcceptableRouteLengthMeters = 3000.0f;

	// Minimum horizontal padding around start/goal for the first search attempt
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="500.0"))
	float SearchCorridorMinPaddingMeters = 3000.0f;

	// Extra corridor padding relative to the straight-line start-goal distance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float SearchCorridorPaddingRatio = 0.5f;

	// How many times the search corridor doubles before falling back to the full map.
	// Keeps short routes fast while still guaranteeing the full map is tried for hard routes.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0", ClampMax="6"))
	int32 MaxSearchCorridorWideningAttempts = 3;

	// Distance where direct goal connection may be accepted
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float GoalConnectionToleranceMeters = 900.0f;

	// Larger finish tolerance when detour obstacles are active
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="1.0"))
	float ObstacleAwareGoalConnectionMultiplier = 3.0f;

	// Maximum length for direct preferred-clearance shortcut checks
	// 0 disables this extra length rule
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Search Space", meta=(ClampMin="0.0"))
	float DirectRoutePreferredClearanceMaxLengthMeters = 0.0f;

	// Absolute lower terrain clearance limit
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float MinimumAbsoluteTerrainClearanceMeters = 50.0f;

	// Multiplier for preferred terrain clearance
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="1.0"))
	float TerrainClearanceSafetyMultiplier = 1.5f;

	// Horizontal radius for conservative terrain sampling
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float LateralTerrainSafetyRadiusMultiplier = 1.0f;

	// Speed-based extra clearance lookahead
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float PreferredClearanceSpeedLookaheadSeconds = 6.0f;

	// Minimum spacing after route compaction
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Result", meta=(ClampMin="0.0"))
	float MinOutputWaypointSpacingMeters = 300.0f;

	// Maximum points tested during waypoint simplification
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Result", meta=(ClampMin="2", ClampMax="2048"))
	int32 MaxOutputWaypointCompactionLookahead = 512;

	// Enable waypoint simplification after A*.
	// Each candidate shortcut is re-validated against the same safety checks as any other segment.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Result")
	bool bUseWaypointCompaction = true;

	// Final route as world-space positions
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Result")
	TArray<FVector> CurrentRouteWorldPoints;

	// A* states behind CurrentRouteWorldPoints, one per waypoint, before compaction
	TArray<FFlightPathState> CurrentRouteStates;

	// Number of states checked in last search
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

	// Collect static influence zones from current level
	UFUNCTION(CallInEditor, BlueprintCallable, Category="Flight Pathfinding|Influence")
	void CollectInfluenceZones();
	
	// Collect static weather zones from current level
	UFUNCTION(CallInEditor, BlueprintCallable, Category="Flight Pathfinding|Weather")
	void CollectWeatherZones();

	// Preferred motion primitive length
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float PrimitiveSegmentLengthMeters = 400.0f;

	// Minimum step size when detour obstacles are active
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float ObstacleAwarePrimitiveSegmentLengthMeters = 1200.0f;

	// Sample count for primitive validation
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="2", ClampMax="32"))
	int32 PrimitiveSamplesPerSegment = 8;

	// Safety factor for climb and descent inside primitive
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="0.1", ClampMax="1.0"))
	float PrimitiveClimbRateFactor = 0.85f;

	// Upper limit for automatic primitive length
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="100.0"))
	float MaxAutoPrimitiveSegmentLengthMeters = 2500.0f;

	// Heading change used by primitive turns
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="1", ClampMax="4"))
	int32 PrimitiveTurnDeltaBuckets = 1;

	// Wider turn options when routing around blocked zones
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Flight Model", meta=(ClampMin="1", ClampMax="10"))
	int32 ObstacleAwareTurnDeltaBuckets = 4;

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

	// Check if state lies inside built search bounds
	bool IsStateInsideBounds(const FFlightPathState& State) const;

	// Convert state to world-space center position
	FVector StateToWorldCenter(const FFlightPathState& State) const;

	// Find nearest valid state around world position
	bool WorldToNearestValidState(const FVector& WorldPos, int32 PreferredHeadingIndex,
	                              FFlightPathState& OutState) const;

	// Convert world position directly to a search state
	bool WorldToStateExact(const FVector& WorldPos, int32 HeadingIndex, FFlightPathState& OutState) const;

	// Build sampled points for one aircraft movement step
	bool BuildPrimitiveSamplePoints(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		float LengthMultiplier,
		TArray<FVector>& OutSamplePoints,
		int32& OutEndHeading
	) const;

	// Apply motion primitive and output resulting state
	bool ApplyMotionPrimitive(
		const FFlightPathState& FromState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		float LengthMultiplier,
		FFlightPathState& OutState
	) const;

	// Validate one full motion primitive between states
	bool IsMotionPrimitiveValid(
		const FFlightPathState& FromState,
		const FFlightPathState& ToState,
		int32 HeadingDeltaBuckets,
		int32 VerticalMode,
		float LengthMultiplier
	) const;

	// Get the primitive length multipliers to try for one neighbor expansion
	void GetPrimitiveLengthMultipliers(TArray<float, TInlineAllocator<3>>& OutMultipliers) const;

	// Get signed turn direction between heading buckets
	int32 GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const;

	// Rebuild sampled primitive between stored states
	bool RebuildPrimitiveSamplesBetweenStates(
		const FFlightPathState& FromState,
		const FFlightPathState& ToState,
		TArray<FVector>& OutSamplePoints
	) const;

	// Check if current state reaches goal voxel
	bool IsGoalState(const FFlightPathState& Current, const FFlightPathState& Goal) const;

	// Keep heading index inside valid bucket range
	int32 NormalizeHeadingIndex(int32 HeadingIndex) const;

	// Convert heading bucket to angle in radians
	float HeadingIndexToAngleRad(int32 HeadingIndex) const;

	// Convert heading bucket to XY grid step
	FIntPoint HeadingIndexToGridStep(int32 HeadingIndex) const;

	// Get closest heading bucket from 2D direction
	int32 ComputeNearestHeadingIndexFromDirection(const FVector2D& Direction) const;

	// Get smallest turn amount between two headings
	int32 GetSmallestHeadingDelta(int32 FromHeading, int32 ToHeading) const;

	// Read cached terrain height at grid cell
	float GetTerrainHeightCmAtCell(int32 X, int32 Y) const;

	// Read terrain height from world XY position
	bool GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const;

	// Read highest nearby terrain around world XY
	bool GetConservativeTerrainHeightCmAtWorldXY(
		float WorldX,
		float WorldY,
		float HorizontalSafetyRadiusMeters,
		float& OutTerrainHeightCm
	) const;

	// Convert Unreal world Z to meters ASL
	float GetAltitudeMetersASLFromWorldZ(float WorldZCm) const;

	// Read terrain height at cell in meters ASL
	float GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const;

	// Get mandatory terrain clearance for this route
	float GetRequiredTerrainClearanceMeters() const;

	// Get preferred clearance used by route cost decisions
	float GetPreferredTerrainClearanceMeters() const;

	// Get radius used for mandatory terrain clearance
	float GetTerrainSafetyRadiusMeters() const;

	// Get wider radius used for preferred terrain clearance
	float GetPreferredTerrainSafetyRadiusMeters() const;

	// Get movement length used by motion primitives
	float GetEffectivePrimitiveSegmentLengthMeters() const;

	// Get A* heuristic weight for current obstacle setup
	float GetEffectiveHeuristicWeight() const;

	// Check if active obstacles need detour-oriented search settings
	bool HasActiveRoutingDetourObstacles() const;

	// Convert aircraft ceiling into world height
	float GetMaxAllowedWorldZCm() const;

	// Check if point stays below aircraft ceiling
	bool DoesPointRespectAltitudeLimit(const FVector& WorldPoint) const;

	// Check if every route point stays below aircraft ceiling
	bool DoesRouteRespectAltitudeLimit(const TArray<FVector>& RoutePoints) const;

	// Check if segment stays below aircraft ceiling
	bool DoesSegmentRespectAltitudeLimit(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check mandatory terrain clearance at one point
	bool DoesPointRespectTerrainClearance(const FVector& WorldPoint) const;

	// Check preferred terrain clearance at one point
	bool DoesPointRespectPreferredTerrainClearance(const FVector& WorldPoint) const;

	// Check preferred terrain clearance along a segment
	bool DoesSegmentRespectPreferredTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check direct segment against all main flight rules
	bool DoesDirectSegmentRespectFlightRules(
		const FVector& FromWorld,
		const FVector& ToWorld,
		int32 FromHeadingIndex,
		int32 ToHeadingIndex
	) const;

	// Try simple direct VFR route before full A* search
	bool TryBuildDirectVfrRoute(
		const FVector& StartWorldLocation,
		const FVector& TargetWorldLocation,
		int32 StartHeadingIndex,
		TArray<FVector>& OutRoutePoints
	);

	// Last-resort climb/cruise/descend fallback when the primitive search only finds degenerate routes
	bool TryBuildHighAltitudeCrossingRoute(
		const FVector& StartWorldLocation,
		const FVector& TargetWorldLocation,
		TArray<FVector>& OutRoutePoints
	) const;

	// Check if search can finish with a direct goal connection
	bool CanConnectToGoal(
		const FFlightPathState& Current,
		const FFlightPathState& Goal,
		const FVector& GoalWorldLocation,
		int32 GoalHeadingIndex
	) const;

	// Check if route corners respect aircraft turn radius
	bool DoesRouteRespectTurnRadius(const TArray<FVector>& RoutePoints) const;

	// Run final safety checks on current route
	bool ValidateCurrentRouteSafety() const;

	// Reduce waypoint count while keeping route safe
	void CompactCurrentRouteWaypoints();

	// Check if one search state is usable
	bool IsStateValid(const FFlightPathState& State) const;

	// Check if movement between two states is allowed
	bool IsTransitionValid(const FFlightPathState& From, const FFlightPathState& To) const;

	// Check terrain clearance along a segment
	bool DoesSegmentRespectTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check if point lies inside forbidden airspace
	bool IsStateInsideHardBlockZone(const FVector& WorldPoint, float AltitudeMetersASL) const;

	// Check if segment crosses forbidden airspace
	bool DoesSegmentIntersectHardBlockZone(
		const FVector& FromWorld,
		const FVector& ToWorld,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL
	) const;

	// Check if influence zone should affect route planning
	bool IsInfluenceZoneActiveForRouting(const AFlightInfluenceZoneActor* Zone) const;

	// Check if point enters a blocking influence zone
	bool IsPointInsideBlockingInfluenceZone(const FVector& WorldPoint, float AltitudeMetersASL) const;

	// Check if segment crosses a blocking influence zone
	bool DoesSegmentIntersectBlockingInfluenceZone(
		const FVector& FromWorld,
		const FVector& ToWorld,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL
	) const;
	
	// Check if point enters blocking weather
	bool IsPointInsideBlockingWeatherZone(const FVector& WorldPoint) const;

	// Check if segment crosses blocking weather
	bool DoesSegmentIntersectBlockingWeatherZone(const FVector& FromWorld, const FVector& ToWorld) const;

	// Check if weather zone should affect route planning
	bool IsWeatherZoneActiveForRouting(const AFlightWeatherZoneActor* Zone) const;

	// Get no-go clearance buffer for hard weather zones
	void GetHardWeatherAvoidanceForZone(
		const AFlightWeatherZoneActor* Zone,
		float& OutHorizontalAvoidanceMeters,
		float& OutVerticalAvoidanceMeters
	) const;

	// Get scattered-cloud clearance for current altitude band
	void GetScatteredCloudClearanceForAltitude(
		float AltitudeMetersASL,
		float& OutHorizontalClearanceMeters,
		float& OutVerticalClearanceMeters
	) const;

	// Check if point violates scattered-cloud clearance
	bool DoesPointViolateScatteredCloudClearance(const FVector& WorldPoint) const;

	// Check if segment violates scattered-cloud clearance
	bool DoesSegmentViolateScatteredCloudClearance(const FVector& FromWorld, const FVector& ToWorld) const;

	// Add route cost for soft influence zones
	float GetSoftZoneTraversalCost(
		const FVector& FromWorld,
		const FVector& ToWorld,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL
	) const;

	// Generate valid next states for A*.
	// bUnrestrictedHeading allows the full heading range, for the start state's first expansion only.
	void GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors, bool bUnrestrictedHeading = false) const;

	// Estimate remaining cost for A*
	float HeuristicCost(const FFlightPathState& A, const FFlightPathState& B) const;

	// Calculate movement cost between two states
	float TransitionCost(const FFlightPathState& From, const FFlightPathState& To) const;

	// Pop best usable A* state from open heap
	bool PopBestOpenStateFromHeap(
		TArray<FFlightOpenEntry>& OpenHeap,
		const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
		FFlightPathState& OutBestState
	) const;

	// Get transition validity with cache support
	bool IsTransitionValidCached(const FFlightPathState& From, const FFlightPathState& To) const;

	// Get transition cost with cache support
	float TransitionCostCached(const FFlightPathState& From, const FFlightPathState& To) const;

	// Rebuild output route from A* parent links
	void ReconstructRoute(
		const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
		const FFlightPathState& GoalState
	);

	// Calculate total route length
	float CalculateRouteLengthMeters() const;

	// Calculate altitude difference from start to goal
	float CalculateNetAltitudeChangeMeters() const;

	// Calculate total climbed height along route
	float CalculateTotalClimbMeters() const;

	// Cached valid/invalid result per transition
	mutable TMap<FFlightTransitionKey, bool> TransitionValidityCache;

	// Cached movement cost per transition
	mutable TMap<FFlightTransitionKey, float> TransitionCostCache;

	// Cached highest nearby terrain height per sampled area
	mutable TMap<FIntVector, float> ConservativeTerrainHeightCache;

	// Marks whether detour obstacle mode was already evaluated
	mutable bool bRoutingDetourObstacleCacheValid = false;

	// Cached detour mode for inner A* loops
	mutable bool bCachedHasActiveRoutingDetourObstacles = false;

	// Forces detour-aware search settings once a direct route was rejected
	mutable bool bForceDetourAwareSearch = false;

	// Add penalty for immediate left-right turn changes
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

	// Build search space around a specific route request, padded by CorridorPaddingMeters
	void BuildSearchSpaceForRoute(
		const FVector& StartWorldLocation,
		const FVector& TargetWorldLocation,
		float CorridorPaddingMeters
	);
};