// Flight pathfinder actor
// Aircraft-aware A* route search
// Uses terrain height cache, flight profile, weather and influence zones
// Search state combines voxel position and heading direction
// Validates terrain clearance, aircraft limits, turn radius and blocked zones
// Supports direct VFR routes, motion primitives, caching and UI result output
// Outputs route points and debug drawing

#include "FlightPathfinderActor.h"

#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "FlightProfile.h"
#include "FlightInfluenceZoneActor.h"
#include "FlightWeatherZoneActor.h"

#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Algo/Reverse.h"

// Setup pathfinder actor
AFlightPathfinderActor::AFlightPathfinderActor()
{
	// No runtime tick needed
	PrimaryActorTick.bCanEverTick = false;
}

// Collect all influence zones from the current level
void AFlightPathfinderActor::CollectInfluenceZones()
{
	// Zone collection can change detour behavior
	bRoutingDetourObstacleCacheValid = false;

	// Remove old collected references
	InfluenceZones.Reset();

	if (!GetWorld())
	{
		// Cannot collect actors without a world
		return;
	}

	for (TActorIterator<AFlightInfluenceZoneActor> It(GetWorld()); It; ++It)
	{
		AFlightInfluenceZoneActor* Zone = *It;
		if (Zone)
		{
			// Store zone for route validation
			InfluenceZones.Add(Zone);
		}
	}

	UE_LOG(LogTemp, Display, TEXT("FlightPathfinder: Collected %d influence zones."), InfluenceZones.Num());
}

// Collect all weather zones from the current level
void AFlightPathfinderActor::CollectWeatherZones()
{
	// Zone collection can change detour behavior
	bRoutingDetourObstacleCacheValid = false;

	// Remove old collected references
	WeatherZones.Reset();

	if (!GetWorld())
	{
		// Cannot collect actors without a world
		return;
	}

	for (TActorIterator<AFlightWeatherZoneActor> It(GetWorld()); It; ++It)
	{
		AFlightWeatherZoneActor* Zone = *It;
		if (Zone)
		{
			// Store zone for route validation
			WeatherZones.Add(Zone);
		}
	}

	UE_LOG(LogTemp, Display, TEXT("FlightPathfinder: Collected %d weather zones."), WeatherZones.Num());
}

// Validate setup for editor-based route search
bool AFlightPathfinderActor::ValidateReferences() const
{
	if (!GridBaker)
	{
		// Terrain grid is required for map bounds
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: GridBaker is missing."));
		return false;
	}

	if (!HeightCache)
	{
		// Height data is required for terrain checks
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is missing."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		// Terrain height cache is required for clearance checks
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is invalid."));
		return false;
	}

	if (!FlightProfile)
	{
		// Aircraft limits are required for safe routing
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: FlightProfile is missing."));
		return false;
	}

	if (!StartActor || !GoalActor)
	{
		// Editor route needs both placed markers
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: StartActor or GoalActor is missing."));
		return false;
	}

	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Speed is needed to check climb and descent over time
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: CruiseSpeedMetersPerSecond must be > 0."));
		return false;
	}

	if (HeadingBucketCount < 4)
	{
		// Too few directions make heading-based search unusable
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeadingBucketCount must be >= 4."));
		return false;
	}

	return true;
}

// Validate setup shared by UI and editor route search
bool AFlightPathfinderActor::ValidateCoreReferences() const
{
	if (!GridBaker)
	{
		// Terrain grid is required for map bounds
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: GridBaker is missing."));
		return false;
	}

	if (!HeightCache)
	{
		// Terrain height cache is required for clearance checks
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is missing."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		// Terrain height cache is required for clearance checks
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is invalid."));
		return false;
	}

	if (!FlightProfile)
	{
		// Aircraft limits are required for safe routing
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: FlightProfile is missing."));
		return false;
	}

	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Speed is needed to check climb and descent over time
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: CruiseSpeedMetersPerSecond must be > 0."));
		return false;
	}

	if (HeadingBucketCount < 4)
	{
		// Too few directions make heading-based search unusable
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeadingBucketCount must be >= 4."));
		return false;
	}

	return true;
}

// Build 3D search space from terrain height range
void AFlightPathfinderActor::BuildSearchSpace()
{
	// Reset layers before rebuild
	ZLayerCount = 0;

	// Use baked cache from grid baker if not assigned directly
	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!HeightCache || !HeightCache->IsValid())
	{
		// Cannot build route space without terrain data
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace failed: HeightCache is missing or invalid."));
		return;
	}

	if (VoxelSizeZMeters <= 0.0f)
	{
		// Vertical layer size must be positive
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace failed: VoxelSizeZMeters must be > 0."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	// Find usable terrain height range
	for (const float HeightCm : HeightCache->MaxHeightCm)
	{
		if (HeightCm <= -1e20f)
		{
			// Ignore invalid cache cells
			continue;
		}

		MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
		MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		// No valid terrain means no safe search space
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace failed: No valid terrain heights in cache."));
		return;
	}

	// Convert vertical voxel size to Unreal units
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// Start with terrain range as base search height
	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	if (StartActor)
	{
		// Ensure start marker is inside vertical range
		MinRelevantZ = FMath::Min(MinRelevantZ, StartActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, StartActor->GetActorLocation().Z);
	}

	if (GoalActor)
	{
		// Ensure goal marker is inside vertical range
		MinRelevantZ = FMath::Min(MinRelevantZ, GoalActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, GoalActor->GetActorLocation().Z);
	}

	// Build full XY bounds from the baked grid
	SearchMinGridX = 0;
	SearchMinGridY = 0;
	SearchMaxGridX = HeightCache->GridSize.X;
	SearchMaxGridY = HeightCache->GridSize.Y;

	SearchMinWorld.X = HeightCache->GridMinWorld.X;
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y;
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		// Keep search space below aircraft ceiling
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	if (SearchMaxWorld.Z <= SearchMinWorld.Z)
	{
		// No vertical room remains after altitude ceiling clamp
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace: No Z-layer within the maximum allowed flight altitude."));
		ZLayerCount = 0;
		return;
	}

	// Convert vertical world range into search layers
	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	// Log final search grid dimensions for debugging
	UE_LOG(LogTemp, Display, TEXT("Flight search space built: XY=%d x %d, ZLayers=%d, HeadingBuckets=%d"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
	       ZLayerCount,
	       HeadingBucketCount);
}

// Build the search space for a UI route request, restricted to a padded corridor around start/goal
void AFlightPathfinderActor::BuildSearchSpaceForRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	float CorridorPaddingMeters
)
{
	// Reset layers before rebuild
	ZLayerCount = 0;
	SearchMinGridX = 0;
	SearchMinGridY = 0;
	SearchMaxGridX = 0;
	SearchMaxGridY = 0;

	// Use baked cache from grid baker if needed
	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!HeightCache || !HeightCache->IsValid())
	{
		// UI route also needs valid terrain data
		UE_LOG(LogTemp, Warning,
		       TEXT("BuildSearchSpaceForRoute failed: HeightCache is missing or invalid."));
		return;
	}

	if (VoxelSizeZMeters <= 0.0f)
	{
		// Vertical layer size must be positive
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute failed: VoxelSizeZMeters must be > 0."));
		return;
	}

	// Build a padded XY corridor around start/goal instead of the whole map, clamped to the terrain grid
	const float PaddingCm = FMath::Max(0.0f, CorridorPaddingMeters) * 100.0f;

	const float MinWorldX = FMath::Min(StartWorldLocation.X, TargetWorldLocation.X) - PaddingCm;
	const float MaxWorldX = FMath::Max(StartWorldLocation.X, TargetWorldLocation.X) + PaddingCm;
	const float MinWorldY = FMath::Min(StartWorldLocation.Y, TargetWorldLocation.Y) - PaddingCm;
	const float MaxWorldY = FMath::Max(StartWorldLocation.Y, TargetWorldLocation.Y) + PaddingCm;

	SearchMinGridX = FMath::Clamp(
		FMath::FloorToInt((MinWorldX - HeightCache->GridMinWorld.X) / HeightCache->CellSizeCm),
		0, HeightCache->GridSize.X);
	SearchMinGridY = FMath::Clamp(
		FMath::FloorToInt((MinWorldY - HeightCache->GridMinWorld.Y) / HeightCache->CellSizeCm),
		0, HeightCache->GridSize.Y);
	SearchMaxGridX = FMath::Clamp(
		FMath::CeilToInt((MaxWorldX - HeightCache->GridMinWorld.X) / HeightCache->CellSizeCm),
		0, HeightCache->GridSize.X);
	SearchMaxGridY = FMath::Clamp(
		FMath::CeilToInt((MaxWorldY - HeightCache->GridMinWorld.Y) / HeightCache->CellSizeCm),
		0, HeightCache->GridSize.Y);

	if (SearchMaxGridX <= SearchMinGridX || SearchMaxGridY <= SearchMinGridY)
	{
		// Corridor collapsed to nothing
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute: Corridor has no usable cells."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	// Find usable terrain height range near the direct start-target line, not the whole corridor
	const float HeightScanPaddingCm = FMath::Max(0.0f, SearchCorridorMinPaddingMeters) * 100.0f;
	const FVector2D SegA(StartWorldLocation.X, StartWorldLocation.Y);
	const FVector2D SegB(TargetWorldLocation.X, TargetWorldLocation.Y);
	const FVector2D SegAB = SegB - SegA;
	const float SegLengthSquared = FMath::Max(KINDA_SMALL_NUMBER, SegAB.SizeSquared());

	for (int32 Y = SearchMinGridY; Y < SearchMaxGridY; ++Y)
	{
		for (int32 X = SearchMinGridX; X < SearchMaxGridX; ++X)
		{
			// Skip cells whose center is farther than the padding distance
			// from the direct start-target line
			const FVector2D CellCenter(
				HeightCache->GridMinWorld.X + (static_cast<float>(X) + 0.5f) * HeightCache->CellSizeCm,
				HeightCache->GridMinWorld.Y + (static_cast<float>(Y) + 0.5f) * HeightCache->CellSizeCm
			);
			const float Alpha = FMath::Clamp(
				FVector2D::DotProduct(CellCenter - SegA, SegAB) / SegLengthSquared, 0.0f, 1.0f);
			const FVector2D ClosestPointOnSeg = SegA + SegAB * Alpha;
			if (FVector2D::DistSquared(CellCenter, ClosestPointOnSeg) > FMath::Square(HeightScanPaddingCm))
			{
				continue;
			}

			const float HeightCm = GetTerrainHeightCmAtCell(X, Y);
			if (HeightCm <= -1e20f)
			{
				// Skip invalid terrain cells
				continue;
			}

			MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
			MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
		}
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		// No terrain data means no route search
		UE_LOG(LogTemp, Warning,
		       TEXT("BuildSearchSpaceForRoute failed: No valid terrain heights in corridor."));
		return;
	}

	// Record the raw peak terrain height near the direct line for HeuristicCost
	RouteTerrainPeakWorldZ = MaxTerrainZ;

	// Convert vertical layer size to centimeters
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// Start with terrain range
	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	// Include requested endpoint altitudes
	MinRelevantZ = FMath::Min(MinRelevantZ, StartWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, StartWorldLocation.Z);

	MinRelevantZ = FMath::Min(MinRelevantZ, TargetWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, TargetWorldLocation.Z);

	SearchMinWorld.X = HeightCache->GridMinWorld.X + (SearchMinGridX * HeightCache->CellSizeCm);
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y + (SearchMinGridY * HeightCache->CellSizeCm);
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (SearchMaxGridX * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (SearchMaxGridY * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		// Do not search above aircraft maximum altitude
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	if (SearchMaxWorld.Z <= SearchMinWorld.Z || SearchMaxGridX <= SearchMinGridX || SearchMaxGridY <= SearchMinGridY)
	{
		// Requested route is outside usable altitude range or corridor collapsed
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute: No usable search volume for this corridor."));
		ZLayerCount = 0;
		return;
	}

	// Convert final height range to voxel layers
	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	// Log route-specific search space
	UE_LOG(LogTemp, Display, TEXT("Flight search space built for UI: XY=%d x %d (corridor padding %.0fm), ZLayers=%d, HeadingBuckets=%d"),
	       SearchMaxGridX - SearchMinGridX,
	       SearchMaxGridY - SearchMinGridY,
	       CorridorPaddingMeters,
	       ZLayerCount,
	       HeadingBucketCount
	);
}

// Convert route failure reason into readable UI text
FText AFlightPathfinderActor::GetFailureReasonText(ERouteFailureReason Reason) const
{
	switch (Reason)
	{
	case ERouteFailureReason::None:
		// No error message needed
		return FText::GetEmpty();

	case ERouteFailureReason::InvalidStart:
		// Start position cannot be used
		return FText::FromString(TEXT("Starting point is invalid."));

	case ERouteFailureReason::InvalidTarget:
		// Target position cannot be used
		return FText::FromString(TEXT("Destination point is invalid."));

	case ERouteFailureReason::InvalidTargetState:
		// Target could not be mapped to a valid search state
		return FText::FromString(
			TEXT("The target state is invalid or cannot be included in the search grid."));

	case ERouteFailureReason::InvalidFlightProfile:
		// Missing or invalid aircraft profile
		return FText::FromString(TEXT("No valid aircraft type selected."));

	case ERouteFailureReason::StartAltitudeTooLow:
		// Start is too close to terrain
		return FText::FromString(TEXT("The starting altitude is too low."));

	case ERouteFailureReason::TargetAltitudeTooLow:
		// Target is too close to terrain
		return FText::FromString(TEXT("The target height is too low."));

	case ERouteFailureReason::TargetAltitudeTooHigh:
		// Target exceeds aircraft ceiling
		return FText::FromString(TEXT("The target altitude is above the maximum permitted flight altitude of the aircraft."));

	case ERouteFailureReason::TargetAltitudeNotReachable:
		// Aircraft cannot reach requested target altitude
		return FText::FromString(TEXT("The target altitude cannot be reached with this aircraft."));

	case ERouteFailureReason::TerrainCollision:
	case ERouteFailureReason::TerrainClearance:
		// Route violates terrain safety distance
		return FText::FromString(TEXT("The route falls below the necessary safety distance to the terrain."));

	case ERouteFailureReason::RestrictedAirspace:
	case ERouteFailureReason::HardBlockZone:
		// Route touches blocked airspace
		return FText::FromString(TEXT("The route passes through a restricted or blocked flight zone."));

	case ERouteFailureReason::ClimbLimitExceeded:
	case ERouteFailureReason::MaxClimbExceeded:
		// Aircraft cannot climb fast enough
		return FText::FromString(TEXT("The required climb rate exceeds the aircraft's limit."));

	case ERouteFailureReason::DescentLimitExceeded:
	case ERouteFailureReason::MaxDescentExceeded:
		// Aircraft cannot descend fast enough
		return FText::FromString(TEXT("The required descent rate exceeds the aircraft's limit."));

	case ERouteFailureReason::TurnRadiusExceeded:
	case ERouteFailureReason::TurnRadiusTooSmall:
		// Turn would be too tight for aircraft
		return FText::FromString(TEXT("The required turning radius is too small for this aircraft."));

	case ERouteFailureReason::NoValidNeighbors:
		// Search got stuck without valid next states
		return FText::FromString(
			TEXT("No valid neighboring waypoints found. The route is blocked by terrain or flight restrictions."));

	case ERouteFailureReason::SearchLimitReached:
	case ERouteFailureReason::MaxExpandedStatesReached:
		// Search stopped before finding a route
		return FText::FromString(TEXT("No route found. The maximum search limit has been reached."));

	case ERouteFailureReason::RouteTooLong:
		// Search found a path but it wandered far more than necessary
		return FText::FromString(
			TEXT("No usable route found. The search only found an unreasonably long, indirect path."));

	case ERouteFailureReason::Unknown:
	default:
		// Fallback when no specific reason is available
		return FText::FromString(TEXT("The route could not be calculated."));
	}
}

// Check if a state lies inside the built search grid
bool AFlightPathfinderActor::IsStateInsideBounds(const FFlightPathState& State) const
{
	if (!HeightCache)
	{
		// Bounds depend on grid size from height cache
		return false;
	}

	return
		State.X >= SearchMinGridX && State.X < SearchMaxGridX &&
		State.Y >= SearchMinGridY && State.Y < SearchMaxGridY &&
		State.Z >= 0 && State.Z < ZLayerCount;
}

// Convert search state into world-space position
FVector AFlightPathfinderActor::StateToWorldCenter(const FFlightPathState& State) const
{
	// Use cell center so route points sit inside voxels
	const float WorldX = HeightCache->GridMinWorld.X + (static_cast<float>(State.X) + 0.5f) * HeightCache->CellSizeCm;
	const float WorldY = HeightCache->GridMinWorld.Y + (static_cast<float>(State.Y) + 0.5f) * HeightCache->CellSizeCm;
	const float WorldZ = SearchMinWorld.Z + (static_cast<float>(State.Z) + 0.5f) * (VoxelSizeZMeters * 100.0f);

	return FVector(WorldX, WorldY, WorldZ);
}

// Find a safe search state near a world position
bool AFlightPathfinderActor::WorldToNearestValidState(
	const FVector& WorldPos,
	int32 PreferredHeadingIndex,
	FFlightPathState& OutState
) const
{
	if (!HeightCache || !HeightCache->IsValid() || ZLayerCount <= 0)
	{
		// Cannot map position without valid grid and vertical layers
		return false;
	}

	// Move position into local grid space
	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	// Find matching terrain grid cell
	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		// Position is outside map grid
		return false;
	}

	// Convert vertical voxel size to Unreal units
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// Start search near requested height
	int32 StartZ = FMath::FloorToInt((WorldPos.Z - SearchMinWorld.Z) / VoxelSizeZCm);
	StartZ = FMath::Clamp(StartZ, 0, ZLayerCount - 1);

	// Use valid heading bucket for generated state
	const int32 NormalizedHeading = NormalizeHeadingIndex(PreferredHeadingIndex);

	// Prefer safe state at or above requested height
	for (int32 Z = StartZ; Z < ZLayerCount; ++Z)
	{
		const FFlightPathState Candidate(GridX, GridY, Z, NormalizedHeading);
		if (IsStateValid(Candidate))
		{
			OutState = Candidate;
			return true;
		}
	}

	// Fallback to lower layers if nothing above works
	for (int32 Z = StartZ - 1; Z >= 0; --Z)
	{
		const FFlightPathState Candidate(GridX, GridY, Z, NormalizedHeading);
		if (IsStateValid(Candidate))
		{
			OutState = Candidate;
			return true;
		}
	}

	// No safe state found at this XY position
	return false;
}

// Convert a world position directly into a search state
bool AFlightPathfinderActor::WorldToStateExact(
	const FVector& WorldPos,
	int32 HeadingIndex,
	FFlightPathState& OutState
) const
{
	if (!HeightCache || !HeightCache->IsValid() || ZLayerCount <= 0)
	{
		// Exact conversion needs valid grid data
		return false;
	}

	// Move position into local grid space
	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	// Find exact grid cell for world XY
	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		// Position is outside grid
		return false;
	}

	// Convert world height to vertical layer
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const int32 GridZ = FMath::FloorToInt((WorldPos.Z - SearchMinWorld.Z) / VoxelSizeZCm);

	if (GridZ < 0 || GridZ >= ZLayerCount)
	{
		// Height is outside built search space
		return false;
	}

	// Output exact state with safe heading index
	OutState = FFlightPathState(GridX, GridY, GridZ, NormalizeHeadingIndex(HeadingIndex));
	return true;
}

// Build sampled points for one aircraft movement step
bool AFlightPathfinderActor::BuildPrimitiveSamplePoints(
	const FFlightPathState& FromState,
	int32 HeadingDeltaBuckets,
	int32 VerticalMode,
	float LengthMultiplier,
	TArray<FVector>& OutSamplePoints,
	int32& OutEndHeading
) const
{
	// Remove old samples from previous call
	OutSamplePoints.Reset();

	if (!FlightProfile || PrimitiveSamplesPerSegment < 2)
	{
		// Primitive needs aircraft data and at least start/end samples
		return false;
	}

	// Start primitive at current state center
	const FVector StartWorld = StateToWorldCenter(FromState);

	// Define start and end direction of this movement
	const int32 StartHeading = FromState.HeadingIndex;
	const int32 EndHeading = NormalizeHeadingIndex(StartHeading + HeadingDeltaBuckets);
	OutEndHeading = EndHeading;

	// Convert heading buckets into angles
	const float StartAngle = HeadingIndexToAngleRad(StartHeading);
	const float EndAngle = HeadingIndexToAngleRad(EndHeading);

	// Use shortest rotation direction between headings
	float SignedDeltaAngle = EndAngle - StartAngle;

	while (SignedDeltaAngle > PI)
	{
		// Wrap large positive turn
		SignedDeltaAngle -= 2.0f * PI;
	}

	while (SignedDeltaAngle < -PI)
	{
		// Wrap large negative turn
		SignedDeltaAngle += 2.0f * PI;
	}

	// Choose movement length for this aircraft and grid setup
	const float SegmentLengthMeters = GetEffectivePrimitiveSegmentLengthMeters() * FMath::Max(0.1f, LengthMultiplier);
	const float SegmentLengthCm = SegmentLengthMeters * 100.0f;

	// Increase samples for longer primitives
	const int32 EffectiveSampleCount = FMath::Clamp(
		FMath::Max(PrimitiveSamplesPerSegment, FMath::CeilToInt(SegmentLengthMeters / 150.0f)),
		2,
		32
	);

	// Split movement into equal distance steps
	const float StepLengthCm = SegmentLengthCm / static_cast<float>(EffectiveSampleCount);

	// Estimate available time for climb or descent
	const float TravelTimeSeconds = SegmentLengthMeters / FlightProfile->CruiseSpeedMetersPerSecond;

	// Keep climb/descent slightly below aircraft limit
	const float ClimbRateFactor = FMath::Clamp(FMath::Max(PrimitiveClimbRateFactor, 0.75f), 0.1f, 1.0f);

	// Default movement stays level
	float TotalDeltaZCm = 0.0f;

	if (VerticalMode > 0)
	{
		// Create climbing primitive
		TotalDeltaZCm = FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds * 100.0f;
	}
	else if (VerticalMode < 0)
	{
		// Create descending primitive
		TotalDeltaZCm = -FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds * 100.0f;
	}

	// Add first sample at current state
	FVector CurrentWorld = StartWorld;
	OutSamplePoints.Add(CurrentWorld);

	for (int32 Step = 1; Step <= EffectiveSampleCount; ++Step)
	{
		// Move forward through this primitive
		const float Alpha = static_cast<float>(Step) / static_cast<float>(EffectiveSampleCount);

		// Gradually turn toward end heading
		const float CurrentAngle = StartAngle + SignedDeltaAngle * Alpha;

		// Advance in current heading direction
		CurrentWorld.X += FMath::Cos(CurrentAngle) * StepLengthCm;
		CurrentWorld.Y += FMath::Sin(CurrentAngle) * StepLengthCm;

		// Apply climb/descent smoothly over primitive
		CurrentWorld.Z = StartWorld.Z + TotalDeltaZCm * Alpha;

		OutSamplePoints.Add(CurrentWorld);
	}

	// Primitive is usable only if it has a path
	return OutSamplePoints.Num() >= 2;
}

// Recreate movement samples between two existing states
bool AFlightPathfinderActor::RebuildPrimitiveSamplesBetweenStates(
	const FFlightPathState& FromState,
	const FFlightPathState& ToState,
	TArray<FVector>& OutSamplePoints
) const
{
	// Remove old sample data
	OutSamplePoints.Reset();

	// Recover turn direction from stored headings
	const int32 HeadingDeltaBuckets = GetSignedHeadingDeltaBuckets(
		FromState.HeadingIndex,
		ToState.HeadingIndex
	);

	// Recover climb/descent direction from layer change
	int32 VerticalMode = 0;
	if (ToState.Z > FromState.Z)
	{
		VerticalMode = 1;
	}
	else if (ToState.Z < FromState.Z)
	{
		VerticalMode = -1;
	}

	// Try each adaptive length multiplier and keep whichever reproduces the closest endpoint
	TArray<float, TInlineAllocator<3>> CandidateMultipliers;
	GetPrimitiveLengthMultipliers(CandidateMultipliers);

	const FVector TargetWorldCenter = StateToWorldCenter(ToState);
	float BestDistanceSquared = TNumericLimits<float>::Max();
	TArray<FVector> BestSamplePoints;

	for (const float LengthMultiplier : CandidateMultipliers)
	{
		TArray<FVector> CandidateSamplePoints;
		int32 EndHeading = 0;

		if (!BuildPrimitiveSamplePoints(
			FromState,
			HeadingDeltaBuckets,
			VerticalMode,
			LengthMultiplier,
			CandidateSamplePoints,
			EndHeading))
		{
			continue;
		}

		if (CandidateSamplePoints.Num() < 2)
		{
			continue;
		}

		const float DistanceSquared = FVector::DistSquared(CandidateSamplePoints.Last(), TargetWorldCenter);
		if (DistanceSquared < BestDistanceSquared)
		{
			BestDistanceSquared = DistanceSquared;
			BestSamplePoints = MoveTemp(CandidateSamplePoints);
		}
	}

	if (BestSamplePoints.Num() < 2)
	{
		return false;
	}

	// Force final sample onto exact target state center
	BestSamplePoints.Last() = TargetWorldCenter;
	OutSamplePoints = MoveTemp(BestSamplePoints);

	// Rebuilt primitive must contain at least start and end
	return OutSamplePoints.Num() >= 2;
}

// Check if search reached the goal voxel
bool AFlightPathfinderActor::IsGoalState(const FFlightPathState& Current, const FFlightPathState& Goal) const
{
	// Heading is ignored, only position must match
	return Current.X == Goal.X && Current.Y == Goal.Y && Current.Z == Goal.Z;
}

// Keep heading index inside valid circular range
int32 AFlightPathfinderActor::NormalizeHeadingIndex(int32 HeadingIndex) const
{
	if (HeadingBucketCount <= 0)
	{
		// Invalid heading setup fallback
		return 0;
	}

	// Wrap heading around bucket circle
	int32 Result = HeadingIndex % HeadingBucketCount;
	if (Result < 0)
	{
		Result += HeadingBucketCount;
	}

	return Result;
}

// Convert heading bucket to world angle
float AFlightPathfinderActor::HeadingIndexToAngleRad(int32 HeadingIndex) const
{
	// Work with a valid heading bucket
	const int32 Normalized = NormalizeHeadingIndex(HeadingIndex);

	// Convert bucket position into full circle fraction
	const float Fraction = static_cast<float>(Normalized) / static_cast<float>(HeadingBucketCount);

	return Fraction * 2.0f * PI;
}

// Convert heading bucket to grid movement direction
FIntPoint AFlightPathfinderActor::HeadingIndexToGridStep(int32 HeadingIndex) const
{
	// Use heading angle to derive XY direction
	const float Angle = HeadingIndexToAngleRad(HeadingIndex);

	// Round to nearest grid step
	const int32 StepX = FMath::RoundToInt(FMath::Cos(Angle));
	const int32 StepY = FMath::RoundToInt(FMath::Sin(Angle));

	return FIntPoint(StepX, StepY);
}

// Find closest heading bucket for a 2D direction
int32 AFlightPathfinderActor::ComputeNearestHeadingIndexFromDirection(const FVector2D& Direction) const
{
	if (Direction.IsNearlyZero())
	{
		// No direction available, use default heading
		return 0;
	}

	// Convert direction vector to angle
	const float Angle = FMath::Atan2(Direction.Y, Direction.X);

	// Convert angle to positive 0..2PI range
	float NormalizedAngle = Angle;
	if (NormalizedAngle < 0.0f)
	{
		NormalizedAngle += 2.0f * PI;
	}

	// Map angle fraction to nearest heading bucket
	const float Fraction = NormalizedAngle / (2.0f * PI);
	return NormalizeHeadingIndex(FMath::RoundToInt(Fraction * static_cast<float>(HeadingBucketCount)));
}

// Get smallest turn amount between two headings
int32 AFlightPathfinderActor::GetSmallestHeadingDelta(int32 FromHeading, int32 ToHeading) const
{
	// Normalize both headings before comparing
	const int32 A = NormalizeHeadingIndex(FromHeading);
	const int32 B = NormalizeHeadingIndex(ToHeading);

	// Check both directions around heading circle
	const int32 Forward = (B - A + HeadingBucketCount) % HeadingBucketCount;
	const int32 Backward = (A - B + HeadingBucketCount) % HeadingBucketCount;

	return FMath::Min(Forward, Backward);
}

// Get signed turn direction between two headings
int32 AFlightPathfinderActor::GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const
{
	// Normalize both headings before signed comparison
	const int32 A = NormalizeHeadingIndex(FromHeading);
	const int32 B = NormalizeHeadingIndex(ToHeading);

	// Start with direct difference
	int32 Delta = B - A;

	if (Delta > HeadingBucketCount / 2)
	{
		// Use shorter wrapped left turn
		Delta -= HeadingBucketCount;
	}
	else if (Delta < -HeadingBucketCount / 2)
	{
		// Use shorter wrapped right turn
		Delta += HeadingBucketCount;
	}

	return Delta;
}

// Read terrain height from a grid cell
float AFlightPathfinderActor::GetTerrainHeightCmAtCell(int32 X, int32 Y) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		// No usable terrain data
		return -FLT_MAX;
	}

	// Convert grid coordinate to cache index
	const int32 Index = HeightCache->ToIndex(X, Y);
	if (!HeightCache->MaxHeightCm.IsValidIndex(Index))
	{
		// Cell outside cached data
		return -FLT_MAX;
	}

	// Return baked maximum terrain height
	return HeightCache->MaxHeightCm[Index];
}

// Read terrain height at a world XY position
bool AFlightPathfinderActor::GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		// Terrain lookup needs valid height cache
		return false;
	}

	// Move world position into local grid space
	const float LocalX = WorldX - HeightCache->GridMinWorld.X;
	const float LocalY = WorldY - HeightCache->GridMinWorld.Y;

	// Find terrain cell under this world position
	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		// Position is outside terrain grid
		return false;
	}

	// Read baked height from cache
	const float TerrainHeightCm = GetTerrainHeightCmAtCell(GridX, GridY);
	if (TerrainHeightCm <= -1e20f)
	{
		// Ignore invalid terrain cells
		return false;
	}

	// Return valid terrain height
	OutTerrainHeightCm = TerrainHeightCm;
	return true;
}

// Convert Unreal world height to altitude above sea level
float AFlightPathfinderActor::GetAltitudeMetersASLFromWorldZ(float WorldZCm) const
{
	if (!HeightCache)
	{
		// Sea level reference missing
		return 0.0f;
	}

	// Convert from world centimeters to meters ASL
	return (WorldZCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

// Read terrain altitude at a grid cell
float AFlightPathfinderActor::GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const
{
	// Get baked terrain height first
	const float TerrainHeightCm = GetTerrainHeightCmAtCell(X, Y);
	if (TerrainHeightCm <= -1e20f || !HeightCache)
	{
		// Invalid terrain or missing sea level reference
		return -FLT_MAX;
	}

	// Convert terrain world height to meters ASL
	return (TerrainHeightCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

// Read the highest terrain around a world position
bool AFlightPathfinderActor::GetConservativeTerrainHeightCmAtWorldXY(
	float WorldX,
	float WorldY,
	float HorizontalSafetyRadiusMeters,
	float& OutTerrainHeightCm
) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		// Terrain safety lookup needs valid cache data
		return false;
	}

	// Move world position into local grid space
	const float LocalX = WorldX - HeightCache->GridMinWorld.X;
	const float LocalY = WorldY - HeightCache->GridMinWorld.Y;

	// Find center cell for the safety check
	const int32 CenterX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 CenterY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (CenterX < 0 || CenterY < 0 || CenterX >= HeightCache->GridSize.X || CenterY >= HeightCache->GridSize.Y)
	{
		// Safety area starts outside terrain grid
		return false;
	}

	// Convert safety radius to grid cells
	const float RadiusCm = FMath::Max(0.0f, HorizontalSafetyRadiusMeters) * 100.0f;
	const int32 RadiusCells = FMath::Max(0, FMath::CeilToInt(RadiusCm / HeightCache->CellSizeCm));

	// Reuse previous terrain safety lookup
	const FIntVector CacheKey(CenterX, CenterY, RadiusCells);
	if (const float* CachedHeight = ConservativeTerrainHeightCache.Find(CacheKey))
	{
		OutTerrainHeightCm = *CachedHeight;
		return true;
	}

	float MaxTerrainHeightCm = -FLT_MAX;
	bool bFoundTerrain = false;

	// Clamp sampled safety area to grid bounds
	const int32 MinX = FMath::Max(0, CenterX - RadiusCells);
	const int32 MaxX = FMath::Min(HeightCache->GridSize.X - 1, CenterX + RadiusCells);
	const int32 MinY = FMath::Max(0, CenterY - RadiusCells);
	const int32 MaxY = FMath::Min(HeightCache->GridSize.Y - 1, CenterY + RadiusCells);

	for (int32 Y = MinY; Y <= MaxY; ++Y)
	{
		for (int32 X = MinX; X <= MaxX; ++X)
		{
			// Check every terrain cell in safety area
			const float TerrainHeightCm = GetTerrainHeightCmAtCell(X, Y);
			if (TerrainHeightCm <= -1e20f)
			{
				// Skip invalid terrain cells
				continue;
			}

			// Use highest terrain for conservative clearance
			MaxTerrainHeightCm = FMath::Max(MaxTerrainHeightCm, TerrainHeightCm);
			bFoundTerrain = true;
		}
	}

	if (!bFoundTerrain)
	{
		// No baked height in the safety radius (likely unbaked water) -- treat as sea level
		// instead of unsafe, so lakes don't become an absolute no-fly zone at any altitude
		OutTerrainHeightCm = HeightCache->SeaLevelWorldZCm;
		return true;
	}

	// Return conservative terrain height
	OutTerrainHeightCm = MaxTerrainHeightCm;

	// Cache result for repeated segment checks
	ConservativeTerrainHeightCache.Add(CacheKey, MaxTerrainHeightCm);
	return true;
}

// Get mandatory terrain clearance for this route
float AFlightPathfinderActor::GetRequiredTerrainClearanceMeters() const
{
	if (!FlightProfile)
	{
		// Fallback to project safety minimum
		return FMath::Max(0.0f, MinimumAbsoluteTerrainClearanceMeters);
	}

	// Use the stricter value between aircraft profile and project minimum
	const float ProfileClearanceMeters = FMath::Max(0.0f, FlightProfile->MinimumTerrainClearanceMeters);
	const float BaseClearanceMeters = FMath::Max(ProfileClearanceMeters, MinimumAbsoluteTerrainClearanceMeters);
	return BaseClearanceMeters;
}

// Get preferred safety clearance for route cost decisions
float AFlightPathfinderActor::GetPreferredTerrainClearanceMeters() const
{
	// Preferred clearance is built on top of mandatory clearance
	const float RequiredClearanceMeters = GetRequiredTerrainClearanceMeters();
	if (!FlightProfile)
	{
		// Conservative fallback without aircraft details
		return RequiredClearanceMeters * 2.0f;
	}

	// Add buffer for faster aircraft
	const float SpeedBufferMeters =
		FMath::Max(0.0f, FlightProfile->CruiseSpeedMetersPerSecond) *
		FMath::Max(0.0f, PreferredClearanceSpeedLookaheadSeconds) *
		0.2f;

	// Add buffer for wider aircraft turns
	const float TurnBufferMeters = FMath::Max(0.0f, FlightProfile->MinimumTurnRadiusMeters) * 0.1f;

	// Increase base clearance with safety multiplier
	const float SafetyClearanceMeters =
		RequiredClearanceMeters * FMath::Max(1.0f, TerrainClearanceSafetyMultiplier);

	// Prefer the largest useful safety clearance
	return FMath::Max(RequiredClearanceMeters, SafetyClearanceMeters + SpeedBufferMeters + TurnBufferMeters);
}

// Get radius used for mandatory terrain clearance
float AFlightPathfinderActor::GetTerrainSafetyRadiusMeters() const
{
	// Mandatory clearance only checks the current terrain cell
	return 0.0f;
}

// Get wider radius used for preferred terrain clearance
float AFlightPathfinderActor::GetPreferredTerrainSafetyRadiusMeters() const
{
	if (!FlightProfile)
	{
		// No aircraft data, no extra lateral buffer
		return 0.0f;
	}

	// Faster aircraft should look farther ahead
	const float SpeedLookaheadRadiusMeters =
		FMath::Max(0.0f, FlightProfile->CruiseSpeedMetersPerSecond) *
		FMath::Max(0.0f, PreferredClearanceSpeedLookaheadSeconds) *
		0.35f;

	// Larger turn radius needs more lateral terrain space
	const float TurnRadiusMeters =
		FMath::Max(0.0f, FlightProfile->MinimumTurnRadiusMeters) *
		FMath::Max(0.0f, LateralTerrainSafetyRadiusMultiplier);

	// Grid resolution gives a minimum useful sample radius
	const float HalfCellMeters = HeightCache ? (HeightCache->CellSizeCm / 200.0f) : 0.0f;

	// Use the largest relevant lateral safety radius
	const float PreferredRadiusMeters = FMath::Max(
		FMath::Max(SpeedLookaheadRadiusMeters, TurnRadiusMeters),
		HalfCellMeters
	);

	// Keep preferred terrain sampling within a practical limit
	return FMath::Clamp(PreferredRadiusMeters, 0.0f, 5000.0f);
}

// Get movement length used by motion primitives
float AFlightPathfinderActor::GetEffectivePrimitiveSegmentLengthMeters() const
{
	// Check if obstacles need larger route steps
	const bool bObstacleAwareSearch = HasActiveRoutingDetourObstacles();

	// Start with user-configured minimum length
	float SegmentLengthMeters = FMath::Max(100.0f, PrimitiveSegmentLengthMeters);

	if (bObstacleAwareSearch)
	{
		// Large no-go boxes need longer primitives for useful detours
		SegmentLengthMeters = FMath::Max(SegmentLengthMeters, ObstacleAwarePrimitiveSegmentLengthMeters);
	}

	if (FlightProfile && FlightProfile->CruiseSpeedMetersPerSecond > 0.0f)
	{
		// Determine heading resolution for turn checks
		const float AnglePerBucketRad = (HeadingBucketCount > 0)
			? ((2.0f * PI) / static_cast<float>(HeadingBucketCount))
			: 0.0f;

		if (FlightProfile->MinimumTurnRadiusMeters > 0.0f && AnglePerBucketRad > KINDA_SMALL_NUMBER)
		{
			// Make movement long enough for aircraft turn radius
			SegmentLengthMeters = FMath::Max(
				SegmentLengthMeters,
				FlightProfile->MinimumTurnRadiusMeters * AnglePerBucketRad * 1.15f
			);
		}

		// Keep primitive climb/descent inside aircraft limits
		const float ClimbRateFactor = FMath::Clamp(FMath::Max(PrimitiveClimbRateFactor, 0.75f), 0.1f, 1.0f);

		// One vertical layer is the climb/descent target per step
		const float VerticalLayerMeters = FMath::Max(1.0f, VoxelSizeZMeters);

		if (bAllowClimbStep && FlightProfile->MaxClimbRateMetersPerSecond > 0.0f)
		{
			// Ensure enough horizontal travel for one safe climb layer
			const float DistanceForOneClimbLayer =
				(VerticalLayerMeters * FlightProfile->CruiseSpeedMetersPerSecond) /
				(FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor);

			SegmentLengthMeters = FMath::Max(SegmentLengthMeters, DistanceForOneClimbLayer * 1.1f);
		}

		if (bAllowDescentStep && FlightProfile->MaxDescentRateMetersPerSecond > 0.0f)
		{
			// Ensure enough horizontal travel for one safe descent layer
			const float DistanceForOneDescentLayer =
				(VerticalLayerMeters * FlightProfile->CruiseSpeedMetersPerSecond) /
				(FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor);

			SegmentLengthMeters = FMath::Max(SegmentLengthMeters, DistanceForOneDescentLayer * 1.05f);
		}
	}

	// Limit automatic extension of primitive length
	float MaxSegmentLengthMeters = FMath::Max(PrimitiveSegmentLengthMeters, MaxAutoPrimitiveSegmentLengthMeters);
	if (bObstacleAwareSearch)
	{
		// Obstacle-aware mode must allow its configured minimum step size
		MaxSegmentLengthMeters = FMath::Max(MaxSegmentLengthMeters, ObstacleAwarePrimitiveSegmentLengthMeters);
	}

	return FMath::Min(SegmentLengthMeters, MaxSegmentLengthMeters);
}

// Get A* heuristic weight for current obstacle setup
float AFlightPathfinderActor::GetEffectiveHeuristicWeight() const
{
	// Start with normal A* goal bias
	float Weight = FMath::Max(1.0f, HeuristicWeight);

	if (HasActiveRoutingDetourObstacles())
	{
		// Use stronger goal bias when detour obstacles are active
		Weight = FMath::Max(Weight, ObstacleAwareHeuristicWeight);
	}

	return Weight;
}

// Check if active obstacles need detour-oriented search settings
bool AFlightPathfinderActor::HasActiveRoutingDetourObstacles() const
{
	if (bRoutingDetourObstacleCacheValid)
	{
		// Reuse cached obstacle mode during inner search loops
		return bCachedHasActiveRoutingDetourObstacles;
	}

	if (bForceDetourAwareSearch)
	{
		// A direct route was already rejected, so terrain (or another rule)
		// forces a detour even without a hard-block zone in the way
		bCachedHasActiveRoutingDetourObstacles = true;
		bRoutingDetourObstacleCacheValid = true;
		return true;
	}

	bCachedHasActiveRoutingDetourObstacles = false;

	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (IsInfluenceZoneActiveForRouting(Zone.Get()) && Zone->bHardBlock)
		{
			// Hard influence zones require detour search behavior
			bCachedHasActiveRoutingDetourObstacles = true;
			break;
		}
	}

	if (!bCachedHasActiveRoutingDetourObstacles)
	{
		for (const TObjectPtr<AFlightWeatherZoneActor>& Zone : WeatherZones)
		{
			if (IsWeatherZoneActiveForRouting(Zone.Get()) &&
				(Zone->IsHardBlock() || Zone->IsScatteredClouds()))
			{
				// Weather hazards also require detour search behavior
				bCachedHasActiveRoutingDetourObstacles = true;
				break;
			}
		}
	}

	// Store result for repeated calls during this route search
	bRoutingDetourObstacleCacheValid = true;
	return bCachedHasActiveRoutingDetourObstacles;
}

// Convert aircraft ceiling into Unreal world height
float AFlightPathfinderActor::GetMaxAllowedWorldZCm() const
{
	if (!FlightProfile || !HeightCache || FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		// No ceiling configured
		return FLT_MAX;
	}

	// Aircraft max altitude is stored as meters ASL
	return HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
}

// Check if one point stays below aircraft ceiling
bool AFlightPathfinderActor::DoesPointRespectAltitudeLimit(const FVector& WorldPoint) const
{
	// Get world-space ceiling for current aircraft
	const float MaxAllowedWorldZCm = GetMaxAllowedWorldZCm();
	if (MaxAllowedWorldZCm >= FLT_MAX * 0.5f)
	{
		// No ceiling active
		return true;
	}

	// Small tolerance avoids false rejects from float precision
	return WorldPoint.Z <= MaxAllowedWorldZCm + 1.0f;
}

// Check if every route point stays below aircraft ceiling
bool AFlightPathfinderActor::DoesRouteRespectAltitudeLimit(const TArray<FVector>& RoutePoints) const
{
	if (!FlightProfile)
	{
		// No aircraft limits available
		return false;
	}

	if (FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		// No ceiling configured
		return true;
	}

	for (const FVector& RoutePoint : RoutePoints)
	{
		if (!DoesPointRespectAltitudeLimit(RoutePoint))
		{
			// Reject route if any point exceeds ceiling
			UE_LOG(LogTemp, Warning,
				TEXT("Flight Pathfinder: Route discarded, point %.2f m ASL is above Max Altitude %.2f m."),
				GetAltitudeMetersASLFromWorldZ(RoutePoint.Z),
				FlightProfile->MaxAltitudeMetersASL
			);
			return false;
		}
	}

	return true;
}

// Check if a full segment stays below aircraft ceiling
bool AFlightPathfinderActor::DoesSegmentRespectAltitudeLimit(const FVector& FromWorld, const FVector& ToWorld) const
{
	if (!FlightProfile)
	{
		// Cannot check altitude limit without aircraft profile
		return false;
	}

	if (FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		// No ceiling configured
		return true;
	}

	// Sample segment to catch ceiling violations between endpoints
	const int32 NumSteps = 8;
	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Move along segment from start to end
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (!DoesPointRespectAltitudeLimit(Sample))
		{
			// Segment crosses above aircraft ceiling
			return false;
		}
	}

	return true;
}

// Check mandatory terrain clearance at one point
bool AFlightPathfinderActor::DoesPointRespectTerrainClearance(const FVector& WorldPoint) const
{
	if (!HeightCache || !FlightProfile)
	{
		// Terrain and aircraft clearance data are required
		return false;
	}

	if (!HeightCache->IsInsideLandscapeBounds(WorldPoint.X, WorldPoint.Y))
	{
		// Past the real landscape edge, inside the search grid's padding margin only
		return false;
	}

	float TerrainHeightCm = 0.0f;
	if (!GetConservativeTerrainHeightCmAtWorldXY(
		WorldPoint.X,
		WorldPoint.Y,
		GetTerrainSafetyRadiusMeters(),
		TerrainHeightCm))
	{
		// Cannot verify terrain safety at this point
		return false;
	}

	// Compare aircraft altitude against terrain altitude
	const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);
	const float TerrainMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

	// Accept only if mandatory clearance is reached
	return (AltitudeMetersASL - TerrainMetersASL) >= GetRequiredTerrainClearanceMeters();
}

// Check preferred terrain clearance at one point
bool AFlightPathfinderActor::DoesPointRespectPreferredTerrainClearance(const FVector& WorldPoint) const
{
	if (!HeightCache || !FlightProfile)
	{
		// Preferred clearance depends on terrain and profile
		return false;
	}

	float TerrainHeightCm = 0.0f;
	if (!GetConservativeTerrainHeightCmAtWorldXY(
		WorldPoint.X,
		WorldPoint.Y,
		GetPreferredTerrainSafetyRadiusMeters(),
		TerrainHeightCm))
	{
		// Cannot verify preferred safety at this point
		return false;
	}

	// Compare point altitude against conservative terrain altitude
	const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);
	const float TerrainMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

	// Preferred clearance is stricter than mandatory clearance
	return (AltitudeMetersASL - TerrainMetersASL) >= GetPreferredTerrainClearanceMeters();
}

// Check preferred clearance along a full segment
bool AFlightPathfinderActor::DoesSegmentRespectPreferredTerrainClearance(
	const FVector& FromWorld,
	const FVector& ToWorld
) const
{
	if (!HeightCache || !FlightProfile)
	{
		// Segment check needs terrain and aircraft data
		return false;
	}

	// Handle zero-length segment as single point
	const float DistanceCm = FVector::Distance(FromWorld, ToWorld);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return DoesPointRespectPreferredTerrainClearance(FromWorld);
	}

	// Use dense enough sampling for terrain clearance
	const float SampleStepCm = FMath::Max(100.0f, HeightCache->CellSizeCm * 0.5f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Check each point along the segment
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (!DoesPointRespectAltitudeLimit(Sample) || !DoesPointRespectPreferredTerrainClearance(Sample))
		{
			// Segment violates ceiling or preferred clearance
			return false;
		}
	}

	return true;
}

// Check a direct segment against all main flight rules
bool AFlightPathfinderActor::DoesDirectSegmentRespectFlightRules(
	const FVector& FromWorld,
	const FVector& ToWorld,
	int32 FromHeadingIndex,
	int32 ToHeadingIndex
) const
{
	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Cannot validate movement without usable aircraft data
		LastFailureReason = ERouteFailureReason::InvalidFlightProfile;
		return false;
	}

	if (!DoesSegmentRespectAltitudeLimit(FromWorld, ToWorld))
	{
		// Segment would exceed aircraft ceiling
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		return false;
	}

	if (!DoesSegmentRespectTerrainClearance(FromWorld, ToWorld))
	{
		// Segment is too close to terrain
		FailureStats.TerrainClearanceCount++;
		LastFailureReason = ERouteFailureReason::TerrainClearance;
		return false;
	}

	// Convert segment endpoints to ASL for zone checks
	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	if (DoesSegmentIntersectHardBlockZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
		// Segment crosses forbidden airspace
		FailureStats.HardBlockZoneCount++;
		LastFailureReason = ERouteFailureReason::HardBlockZone;
		return false;
	}

	// Use horizontal distance for flight-time based limits
	const float HorizontalDistanceMeters = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	) / 100.0f;

	// Estimate travel time from cruise speed
	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		HorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	// Check climb and descent against available travel time
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;
	if (DeltaZMeters > 0.0f)
	{
		// Reject if climb is too steep for this aircraft
		const float MaxAllowedClimbMeters = FlightProfile->MaxClimbRateMetersPerSecond * TravelTimeSeconds;
		if (DeltaZMeters > MaxAllowedClimbMeters)
		{
			FailureStats.MaxClimbExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxClimbExceeded;
			return false;
		}
	}
	else if (DeltaZMeters < 0.0f)
	{
		// Reject if descent is too steep for this aircraft
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	// Check if direction change is flyable
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(FromHeadingIndex, ToHeadingIndex);
	if (HeadingDeltaBuckets > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		// Convert bucket difference into turn angle
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float HeadingDeltaRad = static_cast<float>(HeadingDeltaBuckets) * AnglePerBucketRad;

		if (HeadingDeltaRad > KINDA_SMALL_NUMBER && HorizontalDistanceMeters > KINDA_SMALL_NUMBER)
		{
			// Reject if turn would be tighter than aircraft allows
			const float ImpliedTurnRadius = HorizontalDistanceMeters / HeadingDeltaRad;
			if (ImpliedTurnRadius < FlightProfile->MinimumTurnRadiusMeters)
			{
				FailureStats.TurnRadiusTooSmallCount++;
				LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
				return false;
			}
		}
	}

	return true;
}

// Check if search can finish with a direct connection to the goal
bool AFlightPathfinderActor::CanConnectToGoal(
	const FFlightPathState& Current,
	const FFlightPathState& Goal,
	const FVector& GoalWorldLocation,
	int32 GoalHeadingIndex
) const
{
	// Convert current and snapped goal state to world positions
	const FVector CurrentWorld = StateToWorldCenter(Current);
	const FVector GoalStateWorld = StateToWorldCenter(Goal);

	// Check distance to exact UI goal
	const float GoalXYDistanceMeters = FVector2D::Distance(
		FVector2D(CurrentWorld.X, CurrentWorld.Y),
		FVector2D(GoalWorldLocation.X, GoalWorldLocation.Y)
	) / 100.0f;

	// Check distance to snapped grid goal
	const float GoalStateDistanceMeters = FVector::Distance(CurrentWorld, GoalStateWorld) / 100.0f;

	// Allow larger finish connection around detour obstacles
	const float GoalConnectionMultiplier = HasActiveRoutingDetourObstacles()
		? FMath::Max(1.5f, ObstacleAwareGoalConnectionMultiplier)
		: 1.5f;

	// Allow direct finish only near the goal
	const float AllowedConnectionMeters = FMath::Max(
		GoalConnectionToleranceMeters,
		GetEffectivePrimitiveSegmentLengthMeters() * GoalConnectionMultiplier
	);

	if (GoalXYDistanceMeters > AllowedConnectionMeters && GoalStateDistanceMeters > AllowedConnectionMeters * 1.75f)
	{
		// Current state is still too far away
		return false;
	}

	// Aim direct connection toward exact goal location
	const FVector2D SegmentDirection(
		GoalWorldLocation.X - CurrentWorld.X,
		GoalWorldLocation.Y - CurrentWorld.Y
	);
	const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);

	// Goal heading is currently not enforced
	(void)GoalHeadingIndex;

	// Final direct connection must still follow flight rules
	return DoesDirectSegmentRespectFlightRules(CurrentWorld, GoalWorldLocation, SegmentHeading, SegmentHeading);
}

// Try a simple straight route before running A*
bool AFlightPathfinderActor::TryBuildDirectVfrRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	int32 StartHeadingIndex,
	TArray<FVector>& OutRoutePoints
)
{
	// Clear output before trying shortcut
	OutRoutePoints.Reset();

	// Straight route does not need start heading yet
	(void)StartHeadingIndex;

	if (!FlightProfile || !HeightCache)
	{
		// Direct route still needs terrain and aircraft data
		return false;
	}

	// Measure direct route length in XY
	const FVector2D StartXY(StartWorldLocation.X, StartWorldLocation.Y);
	const FVector2D TargetXY(TargetWorldLocation.X, TargetWorldLocation.Y);
	const float HorizontalDistanceMeters = FVector2D::Distance(StartXY, TargetXY) / 100.0f;

	if (DirectRoutePreferredClearanceMaxLengthMeters > 0.0f &&
		HorizontalDistanceMeters > DirectRoutePreferredClearanceMaxLengthMeters)
	{
		// Skip shortcut for long routes if configured
		return false;
	}

	// Build two-point direct route
	TArray<FVector> StraightRoute;
	StraightRoute.Add(StartWorldLocation);
	StraightRoute.Add(TargetWorldLocation);

	// Use normal final route validation for shortcut.
	// No A* states back this route, so force the straight-chord validation path.
	CurrentRouteWorldPoints = StraightRoute;
	CurrentRouteStates.Reset();
	if (!ValidateCurrentRouteSafety())
	{
		// Shortcut is unsafe, fallback to A*
		CurrentRouteWorldPoints.Reset();
		return false;
	}

	// Direct route is safe and can be returned
	OutRoutePoints = StraightRoute;
	CurrentRouteWorldPoints = StraightRoute;
	return true;
}

// Build a simple climb/cruise/descend route straight toward the target,
// used as a last resort when the primitive-based A* search only finds degenerate paths
bool AFlightPathfinderActor::TryBuildHighAltitudeCrossingRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	TArray<FVector>& OutRoutePoints
) const
{
	OutRoutePoints.Reset();

	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		return false;
	}

	const FVector2D StartXY(StartWorldLocation.X, StartWorldLocation.Y);
	const FVector2D TargetXY(TargetWorldLocation.X, TargetWorldLocation.Y);
	const float TotalHorizontalMeters = FVector2D::Distance(StartXY, TargetXY) / 100.0f;

	if (TotalHorizontalMeters <= KINDA_SMALL_NUMBER)
	{
		// Start and target are effectively the same point
		return false;
	}

	// Find the highest terrain actually under the direct line, not the whole padded search corridor
	float HighestTerrainCm = -TNumericLimits<float>::Max();
	const int32 NumTerrainSamples = FMath::Clamp(FMath::CeilToInt(TotalHorizontalMeters / 50.0f), 4, 400);
	for (int32 Step = 0; Step <= NumTerrainSamples; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumTerrainSamples);
		const FVector2D SampleXY = FMath::Lerp(StartXY, TargetXY, Alpha);

		float TerrainHeightCm = 0.0f;
		if (GetConservativeTerrainHeightCmAtWorldXY(
			SampleXY.X, SampleXY.Y, GetPreferredTerrainSafetyRadiusMeters(), TerrainHeightCm))
		{
			HighestTerrainCm = FMath::Max(HighestTerrainCm, TerrainHeightCm);
		}
	}

	// Extra fixed buffer on top of preferred clearance for this straight climb/descend ramp
	const float ClearanceCm = (GetPreferredTerrainClearanceMeters() + 100.0f) * 100.0f;
	float DesiredCruiseWorldZ = (HighestTerrainCm > -TNumericLimits<float>::Max() * 0.5f)
		? (HighestTerrainCm + ClearanceCm)
		: FMath::Max(StartWorldLocation.Z, TargetWorldLocation.Z);

	// Never plan above the aircraft's own ceiling or the search corridor's Z range
	DesiredCruiseWorldZ = FMath::Min(DesiredCruiseWorldZ, GetMaxAllowedWorldZCm());
	DesiredCruiseWorldZ = FMath::Min(DesiredCruiseWorldZ, SearchMaxWorld.Z);

	// Cruise altitude must still be at or above both endpoints
	DesiredCruiseWorldZ = FMath::Max(DesiredCruiseWorldZ, FMath::Max(StartWorldLocation.Z, TargetWorldLocation.Z));

	const float ClimbRateFactor = FMath::Clamp(FMath::Max(PrimitiveClimbRateFactor, 0.75f), 0.1f, 1.0f);
	const float ClimbRateEff = FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor);
	const float DescentRateEff = FMath::Max(0.1f, FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor);
	const float CruiseSpeed = FlightProfile->CruiseSpeedMetersPerSecond;

	// Horizontal distance needed to climb to cruise altitude and descend back down again,
	// within the aircraft's climb/descent rate (never shrunk to fit the direct distance, see dogleg case below)
	const float DesiredClimbMeters = FMath::Max(0.0f, (DesiredCruiseWorldZ - StartWorldLocation.Z) / 100.0f);
	const float DesiredDescentMeters = FMath::Max(0.0f, (DesiredCruiseWorldZ - TargetWorldLocation.Z) / 100.0f);
	const float ClimbHorizontalMeters = (DesiredClimbMeters / ClimbRateEff) * CruiseSpeed;
	const float DescentHorizontalMeters = (DesiredDescentMeters / DescentRateEff) * CruiseSpeed;

	const FVector2D DirectBearing = (TargetXY - StartXY) / (TotalHorizontalMeters * 100.0f);

	FVector2D ClimbEndXY;
	FVector2D DescentStartXY;

	if (ClimbHorizontalMeters + DescentHorizontalMeters <= TotalHorizontalMeters)
	{
		// Enough room: climb and descend directly along the bearing to the
		// target, same as a normal straight VFR leg.
		ClimbEndXY = StartXY + DirectBearing * (ClimbHorizontalMeters * 100.0f);
		DescentStartXY = StartXY + DirectBearing * ((TotalHorizontalMeters - DescentHorizontalMeters) * 100.0f);
	}
	else
	{
		// Not enough room directly between start and target: climb and descend at an
		// angle off the direct bearing instead, like a pilot flying an angled climb-out
		const float DoglegAngleRad = FMath::DegreesToRadians(45.0f);
		const FVector2D ClimbOutBearing(
			DirectBearing.X * FMath::Cos(DoglegAngleRad) - DirectBearing.Y * FMath::Sin(DoglegAngleRad),
			DirectBearing.X * FMath::Sin(DoglegAngleRad) + DirectBearing.Y * FMath::Cos(DoglegAngleRad)
		);
		ClimbEndXY = StartXY + ClimbOutBearing * (ClimbHorizontalMeters * 100.0f);

		const FVector2D ReverseBearing = DirectBearing * -1.0f;
		const FVector2D DescentInBearing(
			ReverseBearing.X * FMath::Cos(DoglegAngleRad) - ReverseBearing.Y * FMath::Sin(DoglegAngleRad),
			ReverseBearing.X * FMath::Sin(DoglegAngleRad) + ReverseBearing.Y * FMath::Cos(DoglegAngleRad)
		);
		DescentStartXY = TargetXY + DescentInBearing * (DescentHorizontalMeters * 100.0f);
	}

	OutRoutePoints.Add(StartWorldLocation);
	OutRoutePoints.Add(FVector(ClimbEndXY.X, ClimbEndXY.Y, DesiredCruiseWorldZ));
	OutRoutePoints.Add(FVector(DescentStartXY.X, DescentStartXY.Y, DesiredCruiseWorldZ));
	OutRoutePoints.Add(TargetWorldLocation);
	return true;
}

// Check if route corners respect aircraft turn radius
bool AFlightPathfinderActor::DoesRouteRespectTurnRadius(const TArray<FVector>& RoutePoints) const
{
	if (!FlightProfile || FlightProfile->MinimumTurnRadiusMeters <= 0.0f || RoutePoints.Num() < 3)
	{
		// No turn-radius check needed
		return true;
	}

	// Aircraft minimum turn radius
	const float RequiredRadiusMeters = FlightProfile->MinimumTurnRadiusMeters;

	// Ignore tiny direction changes
	const float MinTurnAngleRad = FMath::DegreesToRadians(5.0f);

	for (int32 i = 1; i < RoutePoints.Num() - 1; ++i)
	{
		// Check each local corner A -> B -> C
		const FVector& A = RoutePoints[i - 1];
		const FVector& B = RoutePoints[i];
		const FVector& C = RoutePoints[i + 1];

		// Build incoming and outgoing 2D movement vectors
		const FVector2D InVector(B.X - A.X, B.Y - A.Y);
		const FVector2D OutVector(C.X - B.X, C.Y - B.Y);

		// Measure available segment lengths
		const float InLengthMeters = InVector.Size() / 100.0f;
		const float OutLengthMeters = OutVector.Size() / 100.0f;

		if (InLengthMeters <= KINDA_SMALL_NUMBER || OutLengthMeters <= KINDA_SMALL_NUMBER)
		{
			// Skip broken or duplicate route points
			continue;
		}

		// Normalize directions for angle check
		const FVector2D InDir = InVector / (InLengthMeters * 100.0f);
		const FVector2D OutDir = OutVector / (OutLengthMeters * 100.0f);

		// Find actual turn angle at this waypoint
		const float Dot = FMath::Clamp(FVector2D::DotProduct(InDir, OutDir), -1.0f, 1.0f);
		const float TurnAngleRad = FMath::Acos(Dot);

		if (TurnAngleRad < MinTurnAngleRad)
		{
			// Almost straight, no turn radius problem
			continue;
		}

		// Detect very sharp reversal
		const float Cross = InDir.X * OutDir.Y - InDir.Y * OutDir.X;
		if (FMath::Abs(Cross) < 0.001f)
		{
			// Route turns back on itself
			FailureStats.TurnRadiusTooSmallCount++;
			LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
			return false;
		}

		// Estimate needed space to fly this corner
		const float RequiredTangentMeters = RequiredRadiusMeters * FMath::Tan(TurnAngleRad * 0.5f);
		const float AvailableTangentMeters = FMath::Min(InLengthMeters, OutLengthMeters) * 0.75f;

		if (RequiredTangentMeters > AvailableTangentMeters)
		{
			// Not enough distance to perform the turn
			FailureStats.TurnRadiusTooSmallCount++;
			LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
			return false;
		}
	}

	return true;
}

// Run final safety checks on the current route
bool AFlightPathfinderActor::ValidateCurrentRouteSafety() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		// A route needs at least start and target
		return false;
	}

	for (const FVector& Point : CurrentRouteWorldPoints)
	{
		if (!DoesPointRespectAltitudeLimit(Point))
		{
			// A waypoint exceeds aircraft ceiling
			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}

		if (!DoesPointRespectTerrainClearance(Point))
		{
			// A waypoint is too close to terrain
			FailureStats.TerrainClearanceCount++;
			LastFailureReason = ERouteFailureReason::TerrainClearance;
			return false;
		}
	}

	// State-backed hops re-validate along their real curved/climbing primitive arc.
	// Waypoints without a state (e.g. a compacted route) fall back to a straight-chord check.
	const int32 StateBackedSegmentCount = FMath::Max(0, CurrentRouteStates.Num() - 1);

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		if (i < StateBackedSegmentCount)
		{
			TArray<FVector> SamplePoints;
			if (RebuildPrimitiveSamplesBetweenStates(CurrentRouteStates[i], CurrentRouteStates[i + 1], SamplePoints))
			{
				for (int32 SampleIndex = 0; SampleIndex < SamplePoints.Num() - 1; ++SampleIndex)
				{
					const FVector& From = SamplePoints[SampleIndex];
					const FVector& To = SamplePoints[SampleIndex + 1];

					if (!DoesSegmentRespectAltitudeLimit(From, To))
					{
						LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
						return false;
					}

					if (!DoesSegmentRespectTerrainClearance(From, To))
					{
						FailureStats.TerrainClearanceCount++;
						LastFailureReason = ERouteFailureReason::TerrainClearance;
						return false;
					}

					const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(From.Z);
					const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(To.Z);
					if (DoesSegmentIntersectHardBlockZone(From, To, FromAltitudeMetersASL, ToAltitudeMetersASL))
					{
						FailureStats.HardBlockZoneCount++;
						LastFailureReason = ERouteFailureReason::HardBlockZone;
						return false;
					}
				}

				continue;
			}

			// Could not rebuild the primitive, fall through to chord validation
		}

		// Check every segment between output waypoints
		const FVector& From = CurrentRouteWorldPoints[i];
		const FVector& To = CurrentRouteWorldPoints[i + 1];

		// Use segment direction as heading for direct rule validation
		const FVector2D SegmentDirection(To.X - From.X, To.Y - From.Y);
		const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);

		if (!DoesDirectSegmentRespectFlightRules(From, To, SegmentHeading, SegmentHeading))
		{
			// Segment failed one of the flight rules
			return false;
		}
	}

	// State-backed corners already had their turn radius validated per primitive.
	// Only re-check corners introduced after the search (compaction, or the final goal hop).
	if (CurrentRouteStates.Num() != CurrentRouteWorldPoints.Num())
	{
		if (!DoesRouteRespectTurnRadius(CurrentRouteWorldPoints))
		{
			// Route corners are not flyable
			return false;
		}
	}

	return true;
}

// Check if point lies inside forbidden airspace
bool AFlightPathfinderActor::IsStateInsideHardBlockZone(const FVector& WorldPoint, float AltitudeMetersASL) const
{
	if (IsPointInsideBlockingInfluenceZone(WorldPoint, AltitudeMetersASL))
	{
		// Hard influence zones cannot be entered, including safety buffer
		return true;
	}

	if (IsPointInsideBlockingWeatherZone(WorldPoint))
	{
		// Thunderstorm and fog weather zones cannot be entered
		return true;
	}

	if (DoesPointViolateScatteredCloudClearance(WorldPoint))
	{
		// Scattered clouds need VFR cloud clearance
		return true;
	}

	return false;
}

// Check if segment crosses forbidden airspace
bool AFlightPathfinderActor::DoesSegmentIntersectHardBlockZone(
	const FVector& FromWorld,
	const FVector& ToWorld,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL
) const
{
	if (DoesSegmentIntersectBlockingInfluenceZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
		// Segment intersects blocked influence zone, including safety buffer
		return true;
	}

	if (DoesSegmentIntersectBlockingWeatherZone(FromWorld, ToWorld))
	{
		// Segment intersects blocking thunderstorm or fog weather
		return true;
	}

	if (DoesSegmentViolateScatteredCloudClearance(FromWorld, ToWorld))
	{
		// Segment is too close to scattered clouds
		return true;
	}

	return false;
}

// Check if influence zone should affect route planning
bool AFlightPathfinderActor::IsInfluenceZoneActiveForRouting(const AFlightInfluenceZoneActor* Zone) const
{
	if (!Zone)
	{
		// Missing zone cannot affect routing
		return false;
	}

	// Influence zones only count when globally enabled and the zone itself is enabled
	return bUseInfluenceZones && Zone->bInfluenceEnabled;
}


// Check if point enters a blocking influence zone
bool AFlightPathfinderActor::IsPointInsideBlockingInfluenceZone(
	const FVector& WorldPoint,
	float AltitudeMetersASL
) const
{
	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!IsInfluenceZoneActiveForRouting(Zone.Get()) || !Zone->bHardBlock)
		{
			// Ignore disabled, missing or soft influence zones
			continue;
		}

		if (Zone->ContainsPointWithClearance(
			WorldPoint,
			AltitudeMetersASL,
			InfluenceZoneHorizontalAvoidanceMeters,
			InfluenceZoneVerticalAvoidanceMeters
		))
		{
			// Point enters forbidden zone including safety buffer
			return true;
		}
	}

	return false;
}

// Check if segment crosses a blocking influence zone
bool AFlightPathfinderActor::DoesSegmentIntersectBlockingInfluenceZone(
	const FVector& FromWorld,
	const FVector& ToWorld,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL
) const
{
	// Use terrain resolution as minimum useful sample spacing
	const float SampleStepCm = FMath::Max(
		500.0f,
		HeightCache ? HeightCache->CellSizeCm * 0.5f : 10000.0f
	);

	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!IsInfluenceZoneActiveForRouting(Zone.Get()) || !Zone->bHardBlock)
		{
			// Ignore disabled, missing or soft influence zones
			continue;
		}

		if (Zone->IntersectsSegmentWithClearanceBySampling(
			FromWorld,
			ToWorld,
			FromAltitudeMetersASL,
			ToAltitudeMetersASL,
			InfluenceZoneHorizontalAvoidanceMeters,
			InfluenceZoneVerticalAvoidanceMeters,
			SampleStepCm
		))
		{
			// Segment crosses forbidden zone including safety buffer
			return true;
		}
	}

	return false;
}

// Check if weather zone should affect route planning
bool AFlightPathfinderActor::IsWeatherZoneActiveForRouting(const AFlightWeatherZoneActor* Zone) const
{
	if (!Zone)
	{
		// Missing zone cannot affect routing
		return false;
	}

	// Global weather toggle or per-zone enabled state can activate it
	return bUseWeatherZones || Zone->IsWeatherEnabled();
}

// Get no-go clearance buffer for hard weather zones
void AFlightPathfinderActor::GetHardWeatherAvoidanceForZone(
	const AFlightWeatherZoneActor* Zone,
	float& OutHorizontalAvoidanceMeters,
	float& OutVerticalAvoidanceMeters
) const
{
	// Default to no extra avoidance
	OutHorizontalAvoidanceMeters = 0.0f;
	OutVerticalAvoidanceMeters = 0.0f;

	if (!Zone)
	{
		// No zone type available
		return;
	}

	if (Zone->WeatherType == EFlightWeatherType::Thunderstorm)
	{
		// Thunderstorms get large avoidance buffers
		OutHorizontalAvoidanceMeters = ThunderstormHorizontalAvoidanceMeters;
		OutVerticalAvoidanceMeters = ThunderstormVerticalAvoidanceMeters;
		return;
	}

	if (Zone->WeatherType == EFlightWeatherType::Fog)
	{
		// Fog blocks lower airspace with smaller buffer
		OutHorizontalAvoidanceMeters = FogHorizontalAvoidanceMeters;
		OutVerticalAvoidanceMeters = FogVerticalAvoidanceMeters;
	}
}

// Check if point enters blocking weather
bool AFlightPathfinderActor::IsPointInsideBlockingWeatherZone(const FVector& WorldPoint) const
{
	for (const TObjectPtr<AFlightWeatherZoneActor>& Zone : WeatherZones)
	{
		if (!IsWeatherZoneActiveForRouting(Zone.Get()) || !Zone->IsHardBlock())
		{
			// Ignore inactive or non-blocking weather
			continue;
		}

		float HorizontalAvoidanceMeters = 0.0f;
		float VerticalAvoidanceMeters = 0.0f;

		// Use weather-specific no-go buffer
		GetHardWeatherAvoidanceForZone(Zone.Get(), HorizontalAvoidanceMeters, VerticalAvoidanceMeters);

		if (Zone->ContainsPointWithClearance(WorldPoint, HorizontalAvoidanceMeters, VerticalAvoidanceMeters))
		{
			// Point enters blocking weather including buffer
			return true;
		}
	}

	return false;
}

// Check if segment crosses blocking weather
bool AFlightPathfinderActor::DoesSegmentIntersectBlockingWeatherZone(
	const FVector& FromWorld,
	const FVector& ToWorld
) const
{
	for (const TObjectPtr<AFlightWeatherZoneActor>& Zone : WeatherZones)
	{
		if (!IsWeatherZoneActiveForRouting(Zone.Get()) || !Zone->IsHardBlock())
		{
			// Ignore inactive or non-blocking weather
			continue;
		}

		float HorizontalAvoidanceMeters = 0.0f;
		float VerticalAvoidanceMeters = 0.0f;

		// Use weather-specific no-go buffer
		GetHardWeatherAvoidanceForZone(Zone.Get(), HorizontalAvoidanceMeters, VerticalAvoidanceMeters);

		if (Zone->IntersectsSegmentWithClearanceBySampling(
			FromWorld,
			ToWorld,
			HorizontalAvoidanceMeters,
			VerticalAvoidanceMeters
		))
		{
			// Segment crosses blocking weather including buffer
			return true;
		}
	}

	return false;
}

// Get scattered-cloud clearance for current altitude band
void AFlightPathfinderActor::GetScatteredCloudClearanceForAltitude(
	float AltitudeMetersASL,
	float& OutHorizontalClearanceMeters,
	float& OutVerticalClearanceMeters
) const
{
	if (AltitudeMetersASL >= ScatteredCloudFlightLevelThresholdMetersASL)
	{
		// Use higher-altitude VFR cloud clearance
		OutHorizontalClearanceMeters = FMath::Max(0.0f, ScatteredCloudHorizontalClearanceAboveFL100Meters);
		OutVerticalClearanceMeters = FMath::Max(0.0f, ScatteredCloudVerticalClearanceAboveFL100Meters);
		return;
	}

	// Use lower-altitude VFR cloud clearance
	OutHorizontalClearanceMeters = FMath::Max(0.0f, ScatteredCloudHorizontalClearanceBelowFL100Meters);
	OutVerticalClearanceMeters = FMath::Max(0.0f, ScatteredCloudVerticalClearanceBelowFL100Meters);
}

// Check if point violates scattered-cloud clearance
bool AFlightPathfinderActor::DoesPointViolateScatteredCloudClearance(const FVector& WorldPoint) const
{
	// Clearance depends on aircraft altitude
	const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);

	float HorizontalClearanceMeters = 0.0f;
	float VerticalClearanceMeters = 0.0f;

	// Select VFR cloud clearance for this altitude
	GetScatteredCloudClearanceForAltitude(AltitudeMetersASL, HorizontalClearanceMeters, VerticalClearanceMeters);

	for (const TObjectPtr<AFlightWeatherZoneActor>& Zone : WeatherZones)
	{
		if (!IsWeatherZoneActiveForRouting(Zone.Get()) || !Zone->IsScatteredClouds())
		{
			// Ignore inactive or non-cloud weather
			continue;
		}

		if (Zone->ContainsPointWithClearance(WorldPoint, HorizontalClearanceMeters, VerticalClearanceMeters))
		{
			// Point is too close to scattered clouds
			return true;
		}
	}

	return false;
}

// Check if segment violates scattered-cloud clearance
bool AFlightPathfinderActor::DoesSegmentViolateScatteredCloudClearance(
	const FVector& FromWorld,
	const FVector& ToWorld
) const
{
	// Handle zero-length segment as a single point
	const float DistanceCm = FVector::Distance(FromWorld, ToWorld);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return DoesPointViolateScatteredCloudClearance(FromWorld);
	}

	// Sample long enough steps for cloud clearance checks
	const float SampleStepCm = FMath::Max(10000.0f, HeightCache ? HeightCache->CellSizeCm : 10000.0f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Check cloud clearance along the segment
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector SamplePoint = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (DoesPointViolateScatteredCloudClearance(SamplePoint))
		{
			// Segment comes too close to scattered clouds
			return true;
		}
	}

	return false;
}

// Add route cost for soft influence zones
float AFlightPathfinderActor::GetSoftZoneTraversalCost(
	const FVector& FromWorld,
	const FVector& ToWorld,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL
) const
{
	if (!bUseInfluenceZones)
	{
		// Soft zones are ignored when influence routing is disabled
		return 0.0f;
	}

	// Collect all soft-zone penalties along segment
	float TotalExtraCost = 0.0f;

	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!IsInfluenceZoneActiveForRouting(Zone.Get()))
		{
			// Ignore inactive or missing zones
			continue;
		}

		if (!Zone->bHardBlock && Zone->AdditionalTraversalCost > 0.0f)
		{
			// Soft zone is allowed but should be avoided if possible
			if (Zone->IntersectsSegmentBySampling(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
			{
				TotalExtraCost += Zone->AdditionalTraversalCost;
			}
		}
	}

	return TotalExtraCost;
}

// Check if one search state is usable
bool AFlightPathfinderActor::IsStateValid(const FFlightPathState& State) const
{
	if (!HeightCache || !HeightCache->IsValid() || !FlightProfile)
	{
		// State validation needs terrain and aircraft data
		return false;
	}

	if (!IsStateInsideBounds(State))
	{
		// State is outside built search space
		return false;
	}

	// Convert state into world position for safety checks
	const FVector WorldPoint = StateToWorldCenter(State);

	if (!HeightCache->IsInsideLandscapeBounds(WorldPoint.X, WorldPoint.Y))
	{
		// The search corridor is padded beyond the real landscape, which has no ground
		return false;
	}

	if (!DoesPointRespectAltitudeLimit(WorldPoint))
	{
		// State is above aircraft ceiling
		return false;
	}

	if (!DoesPointRespectTerrainClearance(WorldPoint))
	{
		// State is too close to terrain
		return false;
	}

	// Zone checks need ASL altitude
	const float StateAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);

	if (IsStateInsideHardBlockZone(WorldPoint, StateAltitudeMetersASL))
	{
		// State lies inside forbidden airspace
		return false;
	}

	return true;
}

// Check terrain clearance along a segment
bool AFlightPathfinderActor::DoesSegmentRespectTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const
{
	if (!HeightCache || !FlightProfile)
	{
		// Segment terrain check needs terrain and aircraft data
		return false;
	}

	// Handle zero-length segment as single point
	const float DistanceCm = FVector::Distance(FromWorld, ToWorld);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return DoesPointRespectTerrainClearance(FromWorld);
	}

	// Use dense samples to catch terrain between endpoints
	const float SampleStepCm = FMath::Max(100.0f, HeightCache->CellSizeCm * 0.25f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Check one sample along the segment
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (!DoesPointRespectTerrainClearance(Sample))
		{
			// Segment drops below required terrain clearance
			return false;
		}
	}

	return true;
}

// Check if movement between two states is allowed
bool AFlightPathfinderActor::IsTransitionValid(const FFlightPathState& From, const FFlightPathState& To) const
{
	if (!FlightProfile)
	{
		// Transition checks need aircraft limits
		return false;
	}

	if (!IsStateValid(To))
	{
		// Target state cannot be used
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	// Convert both states to world positions for safety checks
	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	if (!DoesSegmentRespectTerrainClearance(FromWorld, ToWorld))
	{
		// Movement path is too close to terrain
		FailureStats.TerrainClearanceCount++;
		LastFailureReason = ERouteFailureReason::TerrainClearance;
		return false;
	}

	// Convert endpoint heights for airspace checks
	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	if (DoesSegmentIntersectHardBlockZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
		// Movement path crosses blocked airspace
		FailureStats.HardBlockZoneCount++;
		LastFailureReason = ERouteFailureReason::HardBlockZone;
		return false;
	}

	// Use horizontal distance to estimate travel time
	const float HorizontalDistanceCm = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	);

	// Convert distance to meters for aircraft profile values
	const float HorizontalDistanceMeters = HorizontalDistanceCm / 100.0f;

	// Estimate how long the aircraft has for this movement
	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		HorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	// Check vertical movement against climb/descent limits
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;

	if (DeltaZMeters > 0.0f)
	{
		// Reject climb that is too steep for available time
		const float MaxAllowedClimbMeters = FlightProfile->MaxClimbRateMetersPerSecond * TravelTimeSeconds;
		if (DeltaZMeters > MaxAllowedClimbMeters)
		{
			FailureStats.MaxClimbExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxClimbExceeded;
			return false;
		}
	}

	if (DeltaZMeters < 0.0f)
	{
		// Reject descent that is too steep for available time
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	// Check if heading change is possible over this distance
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	if (HeadingDeltaBuckets > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		// Convert heading bucket change into turn angle
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float HeadingDeltaRad = static_cast<float>(HeadingDeltaBuckets) * AnglePerBucketRad;

		if (HeadingDeltaRad > KINDA_SMALL_NUMBER && HorizontalDistanceMeters > KINDA_SMALL_NUMBER)
		{
			// Reject if the implied turn is too tight
			const float ImpliedTurnRadius = HorizontalDistanceMeters / HeadingDeltaRad;
			if (ImpliedTurnRadius < FlightProfile->MinimumTurnRadiusMeters)
			{
				FailureStats.TurnRadiusTooSmallCount++;
				LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
				return false;
			}
		}
	}

	return true;
}

// Apply one motion primitive and convert its end into a state
bool AFlightPathfinderActor::ApplyMotionPrimitive(
	const FFlightPathState& FromState,
	int32 HeadingDeltaBuckets,
	int32 VerticalMode,
	float LengthMultiplier,
	FFlightPathState& OutState
) const
{
	TArray<FVector> SamplePoints;
	int32 EndHeading = 0;

	if (!BuildPrimitiveSamplePoints(FromState, HeadingDeltaBuckets, VerticalMode, LengthMultiplier, SamplePoints, EndHeading))
	{
		// Primitive could not be built
		return false;
	}

	// Use the primitive end point as next state target
	const FVector EndWorld = SamplePoints.Last();

	if (!WorldToStateExact(EndWorld, EndHeading, OutState))
	{
		// Primitive ended outside usable search grid
		return false;
	}

	return true;
}

// Validate one full motion primitive
bool AFlightPathfinderActor::IsMotionPrimitiveValid(
	const FFlightPathState& FromState,
	const FFlightPathState& ToState,
	int32 HeadingDeltaBuckets,
	int32 VerticalMode,
	float LengthMultiplier
) const
{
	if (!FlightProfile)
	{
		// Cannot validate primitive without aircraft data
		LastFailureReason = ERouteFailureReason::InvalidFlightProfile;
		return false;
	}

	if (!IsStateValid(ToState))
	{
		// Primitive end state is not safe
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	TArray<FVector> SamplePoints;
	int32 EndHeading = 0;

	if (!BuildPrimitiveSamplePoints(FromState, HeadingDeltaBuckets, VerticalMode, LengthMultiplier, SamplePoints, EndHeading))
	{
		// Primitive path could not be created
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	if (SamplePoints.Num() < 2)
	{
		// Primitive needs at least start and end samples
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	// Force last sample to exact target state center
	SamplePoints.Last() = StateToWorldCenter(ToState);

	float TotalHorizontalDistanceMeters = 0.0f;
	float TotalPathLengthMeters = 0.0f;

	for (int32 i = 0; i < SamplePoints.Num() - 1; ++i)
	{
		// Check each primitive sub-segment
		const FVector& A = SamplePoints[i];
		const FVector& B = SamplePoints[i + 1];

		// Convert sub-segment altitudes for zone checks
		const float AltA = GetAltitudeMetersASLFromWorldZ(A.Z);
		const float AltB = GetAltitudeMetersASLFromWorldZ(B.Z);

		if (!DoesSegmentRespectAltitudeLimit(A, B))
		{
			// Primitive exceeds aircraft ceiling
			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}

		if (!DoesSegmentRespectTerrainClearance(A, B))
		{
			// Primitive is too close to terrain
			FailureStats.TerrainClearanceCount++;
			LastFailureReason = ERouteFailureReason::TerrainClearance;
			return false;
		}

		if (DoesSegmentIntersectHardBlockZone(A, B, AltA, AltB))
		{
			// Primitive crosses forbidden airspace
			FailureStats.HardBlockZoneCount++;
			LastFailureReason = ERouteFailureReason::HardBlockZone;
			return false;
		}

		// Accumulate horizontal distance for climb/descent timing
		TotalHorizontalDistanceMeters += FVector2D::Distance(
			FVector2D(A.X, A.Y),
			FVector2D(B.X, B.Y)
		) / 100.0f;

		// Accumulate full path length for turn radius estimate
		TotalPathLengthMeters += FVector::Distance(A, B) / 100.0f;
	}

	// Estimate primitive travel time from horizontal distance
	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		TotalHorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	// Measure total vertical change over primitive
	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(SamplePoints[0].Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(SamplePoints.Last().Z);
	const float DeltaZMeters = ToAltitudeMetersASL - FromAltitudeMetersASL;

	if (DeltaZMeters > 0.0f)
	{
		// Reject primitive if climb is too steep
		const float MaxAllowedClimbMeters = FlightProfile->MaxClimbRateMetersPerSecond * TravelTimeSeconds;
		if (DeltaZMeters > MaxAllowedClimbMeters)
		{
			FailureStats.MaxClimbExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxClimbExceeded;
			return false;
		}
	}

	if (DeltaZMeters < 0.0f)
	{
		// Reject primitive if descent is too steep
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	// Check if primitive turn shape is flyable
	const int32 HeadingDeltaAbs = FMath::Abs(HeadingDeltaBuckets);
	if (HeadingDeltaAbs > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		// Convert heading change to angular turn
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float DeltaHeadingRad = static_cast<float>(HeadingDeltaAbs) * AnglePerBucketRad;

		if (DeltaHeadingRad > KINDA_SMALL_NUMBER)
		{
			// Reject if primitive turn radius is too small
			const float ImpliedTurnRadius = TotalPathLengthMeters / DeltaHeadingRad;
			if (ImpliedTurnRadius < FlightProfile->MinimumTurnRadiusMeters)
			{
				FailureStats.TurnRadiusTooSmallCount++;
				LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
				return false;
			}
		}
	}

	return true;
}

// Generate valid next states for A*
void AFlightPathfinderActor::GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors, bool bUnrestrictedHeading) const
{
	// Clear previous neighbor list
	OutNeighbors.Reset();

	// Reserve common maximum to avoid repeated allocations
	OutNeighbors.Reserve(192);

	// Start with straight movement
	TArray<int32, TInlineAllocator<21>> HeadingOptions;
	HeadingOptions.Add(0);

	// Use wider turn options around detour obstacles
	int32 EffectiveTurnBuckets = PrimitiveTurnDeltaBuckets;
	if (HasActiveRoutingDetourObstacles())
	{
		EffectiveTurnBuckets = FMath::Max(EffectiveTurnBuckets, ObstacleAwareTurnDeltaBuckets);
	}

	// Limit how much the aircraft may turn per step
	const int32 MaxTurnBuckets = FMath::Clamp(
		FMath::Max(1, FMath::Max(MaxHeadingChangePerStep, EffectiveTurnBuckets)),
		1,
		10
	);

	if (bUnrestrictedHeading)
	{
		// The start state has no real prior heading to turn from, so this one
		// expansion may use the full heading range; every hop after it turns normally
		HeadingOptions.Reset();
		for (int32 Bucket = 0; Bucket < HeadingBucketCount; ++Bucket)
		{
			HeadingOptions.Add(Bucket);
		}
	}
	else
	{
		for (int32 Delta = 1; Delta <= MaxTurnBuckets; ++Delta)
		{
			// Add left and right turn options
			HeadingOptions.Add(-Delta);
			HeadingOptions.Add(Delta);
		}
	}

	// Start with level flight
	TArray<int32, TInlineAllocator<3>> VerticalModes;
	VerticalModes.Add(0);

	if (bAllowClimbStep)
	{
		// Allow climbing primitives
		VerticalModes.Add(1);
	}

	if (bAllowDescentStep)
	{
		// Allow descending primitives
		VerticalModes.Add(-1);
	}

	// Try several movement lengths so tight corrections and long detours
	// are both reachable, instead of a single fixed step size
	TArray<float, TInlineAllocator<3>> LengthMultipliers;
	GetPrimitiveLengthMultipliers(LengthMultipliers);

	for (const int32 HeadingOption : HeadingOptions)
	{
		// Unrestricted: HeadingOptions holds absolute buckets, so treat each as a zero-turn departure heading
		const FFlightPathState PrimitiveFrom = bUnrestrictedHeading
			? FFlightPathState(Current.X, Current.Y, Current.Z, HeadingOption)
			: Current;
		const int32 HeadingDelta = bUnrestrictedHeading ? 0 : HeadingOption;

		for (const int32 VerticalMode : VerticalModes)
		{
			for (const float LengthMultiplier : LengthMultipliers)
			{
				FFlightPathState Candidate;

				if (!ApplyMotionPrimitive(PrimitiveFrom, HeadingDelta, VerticalMode, LengthMultiplier, Candidate))
				{
					// Primitive cannot produce a usable candidate
					continue;
				}

				if (!IsStateInsideBounds(Candidate))
				{
					// Candidate ended outside search space
					continue;
				}

				if (!IsMotionPrimitiveValid(PrimitiveFrom, Candidate, HeadingDelta, VerticalMode, LengthMultiplier))
				{
					// Candidate movement violates flight rules
					continue;
				}

				// Store only valid and unique neighbors
				OutNeighbors.AddUnique(Candidate);
			}
		}
	}

	if (OutNeighbors.Num() == 0)
	{
		// Current state is a dead end
		FailureStats.NoValidNeighborsCount++;
		LastFailureReason = ERouteFailureReason::NoValidNeighbors;
	}
}

// Get the primitive length multipliers to try for one neighbor expansion
void AFlightPathfinderActor::GetPrimitiveLengthMultipliers(TArray<float, TInlineAllocator<3>>& OutMultipliers) const
{
	OutMultipliers.Reset();

	// Normal step size is always available
	OutMultipliers.Add(1.0f);

	if (!bUseAdaptivePrimitiveLengths)
	{
		return;
	}

	// Short steps allow tight local corrections close to terrain or zones
	OutMultipliers.Add(FMath::Clamp(ShortPrimitiveLengthMultiplier, 0.2f, 1.0f));

	// Long steps make early detours and climbs cheaper to reach
	const float LongMultiplier = HasActiveRoutingDetourObstacles()
		? FMath::Max(LongPrimitiveLengthMultiplier, ObstacleAwareLongPrimitiveLengthMultiplier)
		: LongPrimitiveLengthMultiplier;

	OutMultipliers.Add(FMath::Max(1.0f, LongMultiplier));
}

// Estimate remaining cost for A*
float AFlightPathfinderActor::HeuristicCost(const FFlightPathState& A, const FFlightPathState& B) const
{
	// Compare states in world space
	const FVector AWorld = StateToWorldCenter(A);
	const FVector BWorld = StateToWorldCenter(B);

	// Use straight-line distance as optimistic estimate
	const float StraightLineDistanceMeters = FVector::Distance(AWorld, BWorld) / 100.0f;

	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Fallback if aircraft speed is missing
		return StraightLineDistanceMeters;
	}

	// Estimate remaining flight time
	float Cost = StraightLineDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond;

	// A goal at the same altitude as A doesn't mean zero climb cost: terrain between them
	// can force a climb-then-descend, so use the route's known peak terrain height as a floor
	const float RequiredPeakWorldZ = RouteTerrainPeakWorldZ + (GetRequiredTerrainClearanceMeters() * 100.0f);
	const float PeakWorldZ = FMath::Max3(
		static_cast<float>(AWorld.Z), static_cast<float>(BWorld.Z), RequiredPeakWorldZ);

	const float ClimbMeters = FMath::Max(0.0f, (PeakWorldZ - AWorld.Z) / 100.0f);
	if (ClimbMeters > 0.0f)
	{
		// Climb takes time and should influence search direction
		Cost += (ClimbMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 4.0f;
	}

	const float DescentMeters = FMath::Max(0.0f, (PeakWorldZ - BWorld.Z) / 100.0f);
	if (DescentMeters > 0.0f)
	{
		// Matches TransitionCost's descent multiplier
		Cost += (DescentMeters / FMath::Max(0.1f, FlightProfile->MaxDescentRateMetersPerSecond)) * 2.0f;
	}

	return Cost;
}

// Calculate movement cost between two states
float AFlightPathfinderActor::TransitionCost(const FFlightPathState& From, const FFlightPathState& To) const
{
	// Convert states to world positions for distance and safety cost
	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	// Use 3D path length for base travel cost
	const float PathLengthMeters = FVector::Distance(FromWorld, ToWorld) / 100.0f;

	// Use horizontal distance for jitter and flight behavior checks
	const float HorizontalDistanceMeters = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	) / 100.0f;

	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Fallback cost if aircraft speed is unavailable
		return PathLengthMeters;
	}

	// Base cost represents estimated flight time
	float Cost = PathLengthMeters / FlightProfile->CruiseSpeedMetersPerSecond;

	// Convert segment altitudes for influence zone checks
	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	// Add penalty when crossing soft influence zones
	Cost += GetSoftZoneTraversalCost(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL);

	// Penalize climbs so the search plans altitude changes earlier.
	// Multiplier matches HeuristicCost's climb term, both being time-based.
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;
	if (DeltaZMeters > 0.0f)
	{
		const float ClimbPenalty =
			(DeltaZMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 4.0f;

		Cost += ClimbPenalty;
	}

	if (DeltaZMeters < 0.0f)
	{
		// Penalize descents less than climbs, but avoid nervous altitude changes
		const float DescentPenalty =
			(FMath::Abs(DeltaZMeters) / FMath::Max(0.1f, FlightProfile->MaxDescentRateMetersPerSecond)) * 2.0f;

		Cost += DescentPenalty;
	}

	// Penalize heading changes so smoother routes are preferred
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	if (HeadingDeltaBuckets > 0)
	{
		// Kept small since HeuristicCost has no matching turn-cost term to offset it
		const float TurnPenalty =
			static_cast<float>(HeadingDeltaBuckets * HeadingDeltaBuckets) *
			(FMath::Max(1.0f, FlightProfile->MinimumTurnRadiusMeters) /
				FMath::Max(50.0f, GetEffectivePrimitiveSegmentLengthMeters())) *
			1.5f;

		Cost += TurnPenalty;
	}

	// Track the lowest terrain clearance along this transition
	float LowestClearanceMeters = TNumericLimits<float>::Max();
	const int32 NumSamples = 12;

	for (int32 Step = 0; Step <= NumSamples; ++Step)
	{
		// Sample transition for terrain proximity cost
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSamples);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		float TerrainHeightCm = 0.0f;
		if (GetConservativeTerrainHeightCmAtWorldXY(
			Sample.X,
			Sample.Y,
			GetPreferredTerrainSafetyRadiusMeters(),
			TerrainHeightCm))
		{
			// Compare aircraft height with conservative terrain height
			const float SampleAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(Sample.Z);
			const float TerrainAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);
			const float ClearanceMeters = SampleAltitudeMetersASL - TerrainAltitudeMetersASL;

			LowestClearanceMeters = FMath::Min(LowestClearanceMeters, ClearanceMeters);
		}
	}

	if (LowestClearanceMeters < TNumericLimits<float>::Max())
	{
		// Prefer transitions with comfortable terrain clearance
		const float PreferredClearanceMeters = GetPreferredTerrainClearanceMeters();

		if (LowestClearanceMeters < PreferredClearanceMeters)
		{
			// Still legal, but close to terrain, so make it less attractive.
			// Kept small: a soft preference on top of the hard clearance check above.
			const float ClearanceDeficit = PreferredClearanceMeters - LowestClearanceMeters;
			Cost += ClearanceDeficit * 0.15f;
		}
	}

	// Penalize flying too close to the aircraft ceiling
	const float MaxAllowedWorldZCm = GetMaxAllowedWorldZCm();
	if (MaxAllowedWorldZCm < FLT_MAX * 0.5f)
	{
		const float CeilingMarginMeters = (MaxAllowedWorldZCm - ToWorld.Z) / 100.0f;
		const float PreferredCeilingMarginMeters = FMath::Max(GetRequiredTerrainClearanceMeters(), VoxelSizeZMeters);

		if (CeilingMarginMeters < PreferredCeilingMarginMeters)
		{
			// Keep route away from the altitude ceiling if possible. Kept small, same reason as clearance deficit above.
			Cost += (PreferredCeilingMarginMeters - CeilingMarginMeters) * 0.15f;
		}
	}

	if (HorizontalDistanceMeters < GetEffectivePrimitiveSegmentLengthMeters() * 0.5f)
	{
		// Discourage tiny movement steps that create jitter
		Cost += 10.0f;
	}

	return Cost;
}

// Get transition validity with cache support
bool AFlightPathfinderActor::IsTransitionValidCached(const FFlightPathState& From, const FFlightPathState& To) const
{
	// Identify this exact movement for cache lookup
	const FFlightTransitionKey Key(From, To);

	if (const bool* CachedValue = TransitionValidityCache.Find(Key))
	{
		// Reuse previous validation result
		return *CachedValue;
	}

	// Validate transition only once
	const bool bValid = IsTransitionValid(From, To);

	// Store result for future checks
	TransitionValidityCache.Add(Key, bValid);
	return bValid;
}

// Get transition cost with cache support
float AFlightPathfinderActor::TransitionCostCached(const FFlightPathState& From, const FFlightPathState& To) const
{
	// Identify this exact movement for cache lookup
	const FFlightTransitionKey Key(From, To);

	if (const float* CachedValue = TransitionCostCache.Find(Key))
	{
		// Reuse previous cost result
		return *CachedValue;
	}

	// Calculate cost only once
	const float Cost = TransitionCost(From, To);

	// Store result for future checks
	TransitionCostCache.Add(Key, Cost);
	return Cost;
}

// Add penalty for immediate left-right turn changes
float AFlightPathfinderActor::CalculateTurnReversalPenalty(
	const FFlightPathState& GrandParent,
	const FFlightPathState& Parent,
	const FFlightPathState& Current
) const
{
	// Compare turn direction before and after parent state
	const int32 PrevHeading = NormalizeHeadingIndex(Parent.HeadingIndex - GrandParent.HeadingIndex);
	const int32 NextHeading = NormalizeHeadingIndex(Current.HeadingIndex - Parent.HeadingIndex);

	// Convert wrapped heading deltas into signed turn values
	int32 SignedPrevTurn = PrevHeading;
	int32 SignedNextTurn = NextHeading;

	if (SignedPrevTurn > HeadingBucketCount / 2)
	{
		// Interpret previous turn as negative wrapped direction
		SignedPrevTurn -= HeadingBucketCount;
	}

	if (SignedNextTurn > HeadingBucketCount / 2)
	{
		// Interpret next turn as negative wrapped direction
		SignedNextTurn -= HeadingBucketCount;
	}

	if (SignedPrevTurn == 0 || SignedNextTurn == 0)
	{
		// Straight movement is not a reversal
		return 0.0f;
	}

	// Detect immediate left-right or right-left behavior
	const bool bTurnDirectionChanged =
		(SignedPrevTurn > 0 && SignedNextTurn < 0) ||
		(SignedPrevTurn < 0 && SignedNextTurn > 0);

	if (!bTurnDirectionChanged)
	{
		// Same turn direction is fine
		return 0.0f;
	}

	// Make nervous zig-zag routes less attractive
	return 2000.0f;
}

// Pop the best usable A* state from the open heap
bool AFlightPathfinderActor::PopBestOpenStateFromHeap(
	TArray<FFlightOpenEntry>& OpenHeap,
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	FFlightPathState& OutBestState
) const
{
	while (OpenHeap.Num() > 0)
	{
		// Take current best heap entry
		FFlightOpenEntry BestEntry;
		OpenHeap.HeapPop(BestEntry, FFlightOpenEntryMinHeapPredicate());

		// Compare heap entry with latest record data
		const FFlightPathNodeRecord* Record = Records.Find(BestEntry.State);
		if (!Record)
		{
			// Entry no longer has valid node data
			continue;
		}

		if (Record->bClosed)
		{
			// State was already fully processed
			continue;
		}

		if (!FMath::IsNearlyEqual(Record->F, BestEntry.FScore, KINDA_SMALL_NUMBER))
		{
			// Heap entry is outdated after a better path was found
			continue;
		}

		// Return current best open state
		OutBestState = BestEntry.State;
		return true;
	}

	// No usable open state remains
	return false;
}

// Rebuild output route from A* parent links
void AFlightPathfinderActor::ReconstructRoute(
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	const FFlightPathState& GoalState
)
{
	// Remove old output route
	CurrentRouteWorldPoints.Reset();
	CurrentRouteStates.Reset();

	// Walk parent links goal-to-start, appending rather than inserting at front, then reverse once
	TArray<FFlightPathState> StatePath;
	FFlightPathState Current = GoalState;

	while (true)
	{
		StatePath.Add(Current);

		// Follow parent link backward through A* records
		const FFlightPathNodeRecord* Record = Records.Find(Current);
		if (!Record || !Record->bHasParent)
		{
			// Start state reached or record missing
			break;
		}

		Current = Record->Parent;
	}

	if (StatePath.Num() == 0)
	{
		// No path could be reconstructed
		return;
	}

	Algo::Reverse(StatePath);

	// Convert state path into world route points
	CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[0]));
	CurrentRouteStates.Add(StatePath[0]);
	for (int32 i = 1; i < StatePath.Num(); ++i)
	{
		CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[i]));
		CurrentRouteStates.Add(StatePath[i]);
	}
}

// Reduce route waypoint count while keeping safety
void AFlightPathfinderActor::CompactCurrentRouteWaypoints()
{
	if (CurrentRouteWorldPoints.Num() < 3)
	{
		// Nothing to simplify
		return;
	}

	TArray<FVector> CompactRoute;

	// Start compaction at first waypoint
	int32 CurrentIndex = 0;
	CompactRoute.Add(CurrentRouteWorldPoints[0]);

	while (CurrentIndex < CurrentRouteWorldPoints.Num() - 1)
	{
		// Limit how far ahead the simplifier may test
		const int32 LastIndex = CurrentRouteWorldPoints.Num() - 1;
		const int32 MaxTestIndex = FMath::Min(
			LastIndex,
			CurrentIndex + FMath::Max(2, MaxOutputWaypointCompactionLookahead)
		);

		// Fallback keeps the next original waypoint
		int32 BestNextIndex = CurrentIndex + 1;

		for (int32 TestIndex = MaxTestIndex; TestIndex > CurrentIndex + 1; --TestIndex)
		{
			// Test if a later waypoint can be reached directly
			const float DistanceMeters = FVector::Distance(
				CurrentRouteWorldPoints[CurrentIndex],
				CurrentRouteWorldPoints[TestIndex]
			) / 100.0f;

			if (TestIndex != LastIndex && DistanceMeters < MinOutputWaypointSpacingMeters)
			{
				// Skip replacement if spacing would still be too small
				continue;
			}

			// Use direct segment heading for safety validation
			const FVector2D SegmentDirection(
				CurrentRouteWorldPoints[TestIndex].X - CurrentRouteWorldPoints[CurrentIndex].X,
				CurrentRouteWorldPoints[TestIndex].Y - CurrentRouteWorldPoints[CurrentIndex].Y
			);
			const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);

			// No distance cap on the shortcut: DoesDirectSegmentRespectFlightRules below
			// densely samples the whole chord, so a long shortcut that passes it is safe

			// Keep previous turn flyable after simplification
			bool bTurnAtCurrentWaypointIsFlyable = true;
			if (CompactRoute.Num() >= 2)
			{
				// Test local corner created by this shortcut
				TArray<FVector> TurnProbe;
				TurnProbe.Add(CompactRoute[CompactRoute.Num() - 2]);
				TurnProbe.Add(CurrentRouteWorldPoints[CurrentIndex]);
				TurnProbe.Add(CurrentRouteWorldPoints[TestIndex]);

				bTurnAtCurrentWaypointIsFlyable = DoesRouteRespectTurnRadius(TurnProbe);
			}

			if (bTurnAtCurrentWaypointIsFlyable &&
				DoesDirectSegmentRespectFlightRules(
					CurrentRouteWorldPoints[CurrentIndex],
					CurrentRouteWorldPoints[TestIndex],
					SegmentHeading,
					SegmentHeading))
			{
				// Use farthest safe shortcut
				BestNextIndex = TestIndex;
				break;
			}
		}

		// Add selected next waypoint
		CompactRoute.Add(CurrentRouteWorldPoints[BestNextIndex]);

		// Continue compaction from selected point
		CurrentIndex = BestNextIndex;
	}

	// Replace original route with simplified route; states no longer map 1:1 to waypoints
	CurrentRouteWorldPoints = CompactRoute;
	CurrentRouteStates.Reset();
}

// Calculate total route length
float AFlightPathfinderActor::CalculateRouteLengthMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		// No measurable route available
		return 0.0f;
	}

	float TotalLengthMeters = 0.0f;

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		// Add distance between consecutive route points
		const float SegmentLengthCm = FVector::Distance(
			CurrentRouteWorldPoints[i],
			CurrentRouteWorldPoints[i + 1]
		);

		// Convert Unreal centimeters to meters
		TotalLengthMeters += SegmentLengthCm / 100.0f;
	}

	return TotalLengthMeters;
}

// Calculate altitude difference from start to goal
float AFlightPathfinderActor::CalculateNetAltitudeChangeMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		// No start and goal available
		return 0.0f;
	}

	// Read route endpoint altitudes
	const float StartAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[0].Z);
	const float EndAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints.Last().Z);

	// Positive value means net climb
	return EndAltitudeMetersASL - StartAltitudeMetersASL;
}

// Calculate total climbed height along the route
float AFlightPathfinderActor::CalculateTotalClimbMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		// No route segments available
		return 0.0f;
	}

	float TotalClimbMeters = 0.0f;

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		// Compare altitude between neighboring points
		const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[i].Z);
		const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[i + 1].Z);

		const float DeltaMeters = ToAltitudeMetersASL - FromAltitudeMetersASL;

		if (DeltaMeters > 0.0f)
		{
			// Count only positive climb, not descent
			TotalClimbMeters += DeltaMeters;
		}
	}

	return TotalClimbMeters;
}

// Run full route search for UI and editor calls
bool AFlightPathfinderActor::RunFlightRouteSearch(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	UFlightProfile* InFlightProfile,
	TArray<FVector>& OutRoutePoints
)
{
	// Clear output before starting new route search
	OutRoutePoints.Reset();

	// Remove old route and cached search data
	bRoutingDetourObstacleCacheValid = false;
	bForceDetourAwareSearch = false;
	CurrentRouteWorldPoints.Reset();
	CurrentRouteStates.Reset();
	TransitionValidityCache.Reset();
	TransitionCostCache.Reset();
	ConservativeTerrainHeightCache.Reset();
	FailureStats.Reset();

	if (GetWorld())
	{
		// Clear old debug route drawings
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	// Reset last search status
	LastFailureReason = ERouteFailureReason::None;
	LastExpandedStates = 0;

	// Use profile selected by UI or editor caller
	FlightProfile = InFlightProfile;

	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		// Use baked cache from grid baker if not assigned
		HeightCache = GridBaker->HeightCache;
	}

	if (!ValidateCoreReferences())
	{
		// Stop if required route data is missing
		LastFailureReason = ERouteFailureReason::Unknown;
		return false;
	}

	// Always collect static zones before route validation
	CollectInfluenceZones();
	CollectWeatherZones();

	int32 HardInfluenceZoneCount = 0;
	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (IsInfluenceZoneActiveForRouting(Zone.Get()) && Zone->bHardBlock)
		{
			// Count active hard influence zones for debug output
			++HardInfluenceZoneCount;
		}
	}


	// Log influence-zone routing setup
	UE_LOG(LogTemp, Display,
		TEXT("FlightPathfinder influence routing: Active=%s, HardZones=%d, CollectedZones=%d, HorizontalBuffer=%.1fm, VerticalBuffer=%.1fm"),
		bUseInfluenceZones ? TEXT("true") : TEXT("false"),
		HardInfluenceZoneCount,
		InfluenceZones.Num(),
		InfluenceZoneHorizontalAvoidanceMeters,
		InfluenceZoneVerticalAvoidanceMeters
	);

	int32 EnabledWeatherZoneCount = 0;
	for (const TObjectPtr<AFlightWeatherZoneActor>& Zone : WeatherZones)
	{
		if (Zone && Zone->IsWeatherEnabled())
		{
			// Count manually enabled weather zones for debug output
			++EnabledWeatherZoneCount;
		}
	}

	// Log weather-zone routing setup
	UE_LOG(LogTemp, Display,
		TEXT("FlightPathfinder weather routing: Global=%s, EnabledZones=%d, CollectedZones=%d"),
		bUseWeatherZones ? TEXT("true") : TEXT("false"),
		EnabledWeatherZoneCount,
		WeatherZones.Num()
	);

	// Log aircraft values used for this route
	UE_LOG(LogTemp, Display,
		TEXT("FlightPathfinder profile: %s | MaxAlt=%.1fm | Clearance=%.1fm | Speed=%.1fm/s | Climb=%.1fm/s | Descent=%.1fm/s | TurnRadius=%.1fm"),
		*FlightProfile->AircraftName,
		FlightProfile->MaxAltitudeMetersASL,
		FlightProfile->MinimumTerrainClearanceMeters,
		FlightProfile->CruiseSpeedMetersPerSecond,
		FlightProfile->MaxClimbRateMetersPerSecond,
		FlightProfile->MaxDescentRateMetersPerSecond,
		FlightProfile->MinimumTurnRadiusMeters
	);

	// Keep local route endpoints fixed for this search
	const FVector StartPos = StartWorldLocation;
	const FVector GoalPos = TargetWorldLocation;

	// Use start-to-goal direction as initial heading
	const FVector2D StartToGoalXY(GoalPos.X - StartPos.X, GoalPos.Y - StartPos.Y);

	const int32 StartHeading = ComputeNearestHeadingIndexFromDirection(StartToGoalXY);
	const int32 GoalHeading = StartHeading;

	// Check requested endpoint altitudes early
	const float StartAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(StartPos.Z);
	const float GoalAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(GoalPos.Z);

	if (FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		if (StartAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
		{
			// Reject start above aircraft ceiling
			UE_LOG(LogTemp, Warning,
				TEXT("FindFlightRoute aborted: Takeoff altitude %.2f m ASL is above the maximum flight altitude %.2f m ASL."),
				StartAltitudeMetersASL,
				FlightProfile->MaxAltitudeMetersASL
			);

			LastFailureReason = ERouteFailureReason::InvalidStart;
			return false;
		}

		if (GoalAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
		{
			// Reject target above aircraft ceiling
			UE_LOG(LogTemp, Warning,
				TEXT("FindFlightRoute aborted: Target altitude %.2f m ASL is above the maximum flight altitude %.2f m ASL."),
				GoalAltitudeMetersASL,
				FlightProfile->MaxAltitudeMetersASL
			);

			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}
	}

	if (!DoesPointRespectTerrainClearance(StartPos))
	{
		// Reject start too close to terrain
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Starting point falls below the safe distance to the terrain."));
		LastFailureReason = ERouteFailureReason::StartAltitudeTooLow;
		return false;
	}

	if (!DoesPointRespectTerrainClearance(GoalPos))
	{
		// Reject goal too close to terrain
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Target point falls below the safe distance to the terrain."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		return false;
	}

	if (IsStateInsideHardBlockZone(StartPos, StartAltitudeMetersASL))
	{
		// Reject start inside active weather or influence no-fly zone
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Starting point is inside an active no-fly zone."));
		LastFailureReason = ERouteFailureReason::InvalidStart;
		return false;
	}

	if (IsStateInsideHardBlockZone(GoalPos, GoalAltitudeMetersASL))
	{
		// Reject goal inside active weather or influence no-fly zone
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Target point is inside an active no-fly zone."));
		LastFailureReason = ERouteFailureReason::InvalidTarget;
		return false;
	}

	TArray<FVector> DirectRoute;
	if (TryBuildDirectVfrRoute(StartPos, GoalPos, StartHeading, DirectRoute))
	{
		// Use direct route if it already passes all safety checks
		CurrentRouteWorldPoints = DirectRoute;
		OutRoutePoints = DirectRoute;
		LastExpandedStates = 0;

		// Log shortcut route result
		UE_LOG(LogTemp, Display, TEXT("FindFlightRoute: Direct VFR route used. Waypoints=%d"), CurrentRouteWorldPoints.Num());

		if (bAutoDrawPathAfterSearch)
		{
			// Draw direct route in level
			DebugDrawCurrentRoute();
		}

		return true;
	}

	// Direct route failed, start full A* search
	LastFailureReason = ERouteFailureReason::None;
	FailureStats.Reset();

	// A rejected direct route already proves a detour is required, even without a hard-block zone
	bForceDetourAwareSearch = true;
	bRoutingDetourObstacleCacheValid = false;

	// Try progressively wider search corridors around start/goal instead of always searching the whole map
	const float StraightLineDistanceMeters = FVector::Distance(StartPos, GoalPos) / 100.0f;
	const float FullMapCorridorPaddingMeters =
		(HeightCache->GridSize.X + HeightCache->GridSize.Y) * (HeightCache->CellSizeCm / 100.0f);
	const int32 TotalCorridorAttempts = FMath::Max(0, MaxSearchCorridorWideningAttempts) + 1;

	TMap<FFlightPathState, FFlightPathNodeRecord> Records;
	FFlightPathState ReachedGoalState;
	bool bFoundRoute = false;
	int32 TotalExpandedCount = 0;

	for (int32 AttemptIndex = 0; AttemptIndex < TotalCorridorAttempts; ++AttemptIndex)
	{
	const bool bFinalAttempt = (AttemptIndex == TotalCorridorAttempts - 1);

	// Widen the corridor with every retry; the last attempt always covers
	// the full map so a route is never missed just because of padding.
	const float CorridorPaddingMeters = bFinalAttempt
		? FullMapCorridorPaddingMeters
		: FMath::Max(SearchCorridorMinPaddingMeters, StraightLineDistanceMeters * SearchCorridorPaddingRatio)
			* FMath::Pow(2.0f, static_cast<float>(AttemptIndex));

	// Build search grid around requested route
	BuildSearchSpaceForRoute(StartWorldLocation, TargetWorldLocation, CorridorPaddingMeters);

	if (ZLayerCount <= 0)
	{
		// Search space could not be created
		UE_LOG(LogTemp, Warning, TEXT("RunFlightRouteSearch aborted: SearchSpace could not be built."));
		LastFailureReason = ERouteFailureReason::Unknown;
		return false;
	}

	FFlightPathState StartState;
	FFlightPathState GoalState;

	if (!WorldToNearestValidState(StartPos, StartHeading, StartState))
	{
		// Start could not be placed into safe search state
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: No valid starting state found."));
		LastFailureReason = ERouteFailureReason::InvalidStart;
		return false;
	}

	if (!DoesPointRespectAltitudeLimit(GoalPos))
	{
		// Goal exceeds aircraft ceiling after conversion
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Target altitude is above maximum flight altitude."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		return false;
	}

	if (!DoesPointRespectTerrainClearance(GoalPos))
	{
		// Goal is unsafe before search starts
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: Target point falls below the safe distance to the terrain."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		return false;
	}

	if (!WorldToNearestValidState(GoalPos, GoalHeading, GoalState))
	{
		// Goal could not be mapped to safe search state
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute aborted: No valid destination state found."));
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	// Open states waiting for A* expansion
	TArray<FFlightOpenEntry> OpenHeap;

	// Reset records for this attempt's corridor
	Records.Reset();

	// Initialize A* at start state
	FFlightPathNodeRecord StartRecord;
	StartRecord.G = 0.0f;

	// Back the heuristic weight off toward 1.0 on later attempts, so a greedy
	// search doesn't just reproduce the same wandering route on every retry
	const float DegenerateBackoffFraction = TotalCorridorAttempts > 1
		? static_cast<float>(AttemptIndex) / static_cast<float>(TotalCorridorAttempts - 1)
		: 0.0f;
	const float EffectiveHeuristicWeight = FMath::Lerp(GetEffectiveHeuristicWeight(), 1.0f, DegenerateBackoffFraction);

	// Log the tuning values actually used for this attempt
	UE_LOG(LogTemp, Display,
		TEXT("RunFlightRouteSearch attempt %d/%d: HeuristicWeight=%.2f, PrimitiveLength=%.0fm, StartZ_ASL=%.1fm, GoalZ_ASL=%.1fm, MinSearchZ_ASL=%.1fm, MaxSearchZ_ASL=%.1fm"),
		AttemptIndex + 1, TotalCorridorAttempts,
		EffectiveHeuristicWeight,
		GetEffectivePrimitiveSegmentLengthMeters(),
		GetAltitudeMetersASLFromWorldZ(StateToWorldCenter(StartState).Z),
		GetAltitudeMetersASLFromWorldZ(StateToWorldCenter(GoalState).Z),
		GetAltitudeMetersASLFromWorldZ(SearchMinWorld.Z),
		GetAltitudeMetersASLFromWorldZ(SearchMaxWorld.Z)
	);

	StartRecord.H = HeuristicCost(StartState, GoalState);
	StartRecord.F = StartRecord.G + EffectiveHeuristicWeight * StartRecord.H;
	StartRecord.bHasParent = false;
	StartRecord.bClosed = false;

	// Add start state to search
	Records.Add(StartState, StartRecord);
	OpenHeap.HeapPush(
		FFlightOpenEntry(StartState, StartRecord.F),
		FFlightOpenEntryMinHeapPredicate()
	);

	// Track A* progress and limits for this attempt
	int32 ExpandedCount = 0;
	const int32 MaxExpandedStates = FMath::Max(10000, SearchMaxExpandedStates);

	// Reuse neighbor storage to avoid allocations in the A* loop
	TArray<FFlightPathState> Neighbors;
	Neighbors.Reserve(64);

	while (OpenHeap.Num() > 0 && ExpandedCount < MaxExpandedStates)
	{
		// Count processed search states
		++ExpandedCount;

		FFlightPathState Current;
		if (!PopBestOpenStateFromHeap(OpenHeap, Records, Current))
		{
			// No valid open state left
			break;
		}

		// Get editable record for current state
		FFlightPathNodeRecord* CurrentRecord = Records.Find(Current);
		if (!CurrentRecord)
		{
			// Safety fallback for missing record
			continue;
		}

		if (CurrentRecord->bClosed)
		{
			// Already processed by an earlier heap entry
			continue;
		}

		// Mark state as fully processed
		CurrentRecord->bClosed = true;

		// Keep current path cost for neighbor updates
		const float CurrentG = CurrentRecord->G;

		if (bDrawVisitedStates && GetWorld())
		{
			// Optional debug draw for explored search states
			DrawDebugPoint(
				GetWorld(),
				StateToWorldCenter(Current),
				DebugVisitedPointSize,
				FColor::Blue,
				false,
				DebugDrawLifetime
			);
		}

		if (IsGoalState(Current, GoalState) || CanConnectToGoal(Current, GoalState, GoalPos, GoalHeading))
		{
			// Goal reached or direct finish is possible
			bFoundRoute = true;
			ReachedGoalState = Current;
			break;
		}

		// Generate aircraft-valid next states, unrestricted heading for the start state's first expansion
		GetNeighbors(Current, Neighbors, Current == StartState);

		for (const FFlightPathState& Neighbor : Neighbors)
		{
			// Allow reopening closed states: TransitionCost has penalty terms HeuristicCost
			// doesn't, so h is admissible but not consistent, and a closed state can still improve
			FFlightPathNodeRecord* ExistingRecord = Records.Find(Neighbor);

			// Calculate new path cost through current state
			float TentativeG = CurrentG + TransitionCostCached(Current, Neighbor);

			bool bShouldUpdate = false;

			if (!ExistingRecord)
			{
				// First time this neighbor is reached
				FFlightPathNodeRecord NewRecord;
				Records.Add(Neighbor, NewRecord);
				ExistingRecord = Records.Find(Neighbor);
				bShouldUpdate = true;
			}
			else if (TentativeG < ExistingRecord->G)
			{
				// Better path found to existing state
				bShouldUpdate = true;
			}

			if (!bShouldUpdate || !ExistingRecord)
			{
				// Existing path is already better
				continue;
			}

			// Store best known path to neighbor
			ExistingRecord->G = TentativeG;
			ExistingRecord->H = HeuristicCost(Neighbor, GoalState);
			ExistingRecord->F = ExistingRecord->G + EffectiveHeuristicWeight * ExistingRecord->H;
			ExistingRecord->Parent = Current;
			ExistingRecord->bHasParent = true;
			ExistingRecord->bClosed = false;

			// Add updated neighbor to open heap
			OpenHeap.HeapPush(
				FFlightOpenEntry(Neighbor, ExistingRecord->F),
				FFlightOpenEntryMinHeapPredicate()
			);
		}
	}

	// Track total work across all corridor attempts
	TotalExpandedCount += ExpandedCount;

	if (bFoundRoute)
	{
		// Reject a route that wandered unreasonably long compared to the straight-line distance
		ReconstructRoute(Records, ReachedGoalState);
		const float CandidateRouteLengthMeters = CalculateRouteLengthMeters();
		const float MaxAcceptableRouteLengthMeters = FMath::Max(
			StraightLineDistanceMeters * MaxRouteLengthToStraightLineRatio,
			MinAcceptableRouteLengthMeters
		);

		// Diagnostic: log the route's grid bounding box and distinct XY cell count,
		// to tell a genuinely long detour apart from spiraling/thrashing over a small area
		if (CurrentRouteStates.Num() > 0)
		{
			int32 MinGridX = TNumericLimits<int32>::Max();
			int32 MaxGridX = TNumericLimits<int32>::Min();
			int32 MinGridY = TNumericLimits<int32>::Max();
			int32 MaxGridY = TNumericLimits<int32>::Min();
			int32 MinGridZ = TNumericLimits<int32>::Max();
			int32 MaxGridZ = TNumericLimits<int32>::Min();
			TSet<TPair<int32, int32>> UniqueXYCells;

			for (const FFlightPathState& RouteState : CurrentRouteStates)
			{
				MinGridX = FMath::Min(MinGridX, RouteState.X);
				MaxGridX = FMath::Max(MaxGridX, RouteState.X);
				MinGridY = FMath::Min(MinGridY, RouteState.Y);
				MaxGridY = FMath::Max(MaxGridY, RouteState.Y);
				MinGridZ = FMath::Min(MinGridZ, RouteState.Z);
				MaxGridZ = FMath::Max(MaxGridZ, RouteState.Z);
				UniqueXYCells.Add(TPair<int32, int32>(RouteState.X, RouteState.Y));
			}

			UE_LOG(LogTemp, Display,
				TEXT("RunFlightRouteSearch attempt %d/%d: Degenerate route footprint: GridX=[%d,%d], GridY=[%d,%d], GridZ=[%d,%d], UniqueXYCells=%d, TotalWaypoints=%d"),
				AttemptIndex + 1, TotalCorridorAttempts,
				MinGridX, MaxGridX, MinGridY, MaxGridY, MinGridZ, MaxGridZ,
				UniqueXYCells.Num(), CurrentRouteStates.Num());

			// Diagnostic: print an evenly-spaced path sample, since the visited-states debug
			// draw only shows in the 3D viewport, not the minimap. First attempt only.
			if (AttemptIndex == 0)
			{
				const int32 NumSamples = FMath::Min(60, CurrentRouteStates.Num());
				FString PathSampleText;
				for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
				{
					const int32 StateIndex = (CurrentRouteStates.Num() - 1) * SampleIndex / FMath::Max(1, NumSamples - 1);
					const FFlightPathState& S = CurrentRouteStates[StateIndex];
					PathSampleText += FString::Printf(TEXT("(%d,%d,%d,h%d) "), S.X, S.Y, S.Z, S.HeadingIndex);
				}
				UE_LOG(LogTemp, Display,
					TEXT("RunFlightRouteSearch attempt %d/%d: Path sample (%d of %d states, X,Y,Z,heading): %s"),
					AttemptIndex + 1, TotalCorridorAttempts, NumSamples, CurrentRouteStates.Num(), *PathSampleText);
			}
		}

		// Diagnostic: always log what the degenerate-route check evaluated
		UE_LOG(LogTemp, Display,
			TEXT("RunFlightRouteSearch attempt %d/%d: Degenerate-length check: Waypoints=%d, CandidateRouteLength=%.0fm, MaxAllowed=%.0fm, StraightLine=%.0fm, WillReject=%s"),
			AttemptIndex + 1, TotalCorridorAttempts,
			CurrentRouteWorldPoints.Num(),
			CandidateRouteLengthMeters, MaxAcceptableRouteLengthMeters, StraightLineDistanceMeters,
			(CandidateRouteLengthMeters > MaxAcceptableRouteLengthMeters) ? TEXT("true") : TEXT("false"));

		if (CandidateRouteLengthMeters > MaxAcceptableRouteLengthMeters)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("RunFlightRouteSearch attempt %d/%d: Found route rejected as degenerate (RouteLength=%.0fm, StraightLine=%.0fm, MaxAllowed=%.0fm)."),
				AttemptIndex + 1, TotalCorridorAttempts,
				CandidateRouteLengthMeters, StraightLineDistanceMeters, MaxAcceptableRouteLengthMeters);

			bFoundRoute = false;
			LastFailureReason = ERouteFailureReason::RouteTooLong;
		}
		else
		{
			// Stop widening the corridor once a usable route was found
			break;
		}
	}

	if (ExpandedCount >= MaxExpandedStates)
	{
		// Search stopped because the work limit was reached
		LastFailureReason = ERouteFailureReason::MaxExpandedStatesReached;
	}

	// Log failed search size for this corridor attempt
	UE_LOG(LogTemp, Warning,
		TEXT("FindFlightRoute: No route found with corridor padding %.0fm. ExpandedStates=%d"),
		CorridorPaddingMeters, ExpandedCount);

	if (!bFinalAttempt)
	{
		// Widen the corridor and try again
		continue;
	}

	// Log detailed failure counters for debugging
	UE_LOG(LogTemp, Warning,
	       TEXT(
		       "Failure Summary: Terrain=%d, HardZone=%d, Climb=%d, Descent=%d, TurnRadius=%d, InvalidTarget=%d, NoNeighbors=%d"
	       ),
	       FailureStats.TerrainClearanceCount,
	       FailureStats.HardBlockZoneCount,
	       FailureStats.MaxClimbExceededCount,
	       FailureStats.MaxDescentExceededCount,
	       FailureStats.TurnRadiusTooSmallCount,
	       FailureStats.InvalidTargetStateCount,
	       FailureStats.NoValidNeighborsCount
	);

	LastExpandedStates = TotalExpandedCount;

	// Primitive search failed on every corridor width; try a direct climb/cruise/descend crossing
	TArray<FVector> FallbackRoutePoints;
	if (TryBuildHighAltitudeCrossingRoute(StartWorldLocation, TargetWorldLocation, FallbackRoutePoints))
	{
		FString FallbackPointsText;
		for (const FVector& Point : FallbackRoutePoints)
		{
			FallbackPointsText += FString::Printf(TEXT("(%.0f,%.0f,%.0f) "), Point.X, Point.Y, Point.Z);
		}
		UE_LOG(LogTemp, Display,
			TEXT("FindFlightRoute: Trying high-altitude crossing fallback with %d waypoints: %s"),
			FallbackRoutePoints.Num(), *FallbackPointsText);

		CurrentRouteWorldPoints = FallbackRoutePoints;
		CurrentRouteStates.Reset();

		if (DoesRouteRespectAltitudeLimit(CurrentRouteWorldPoints) && ValidateCurrentRouteSafety())
		{
			const float RouteLengthMeters = CalculateRouteLengthMeters();
			const float NetAltitudeChangeMeters = CalculateNetAltitudeChangeMeters();
			const float TotalClimbMeters = CalculateTotalClimbMeters();

			UE_LOG(LogTemp, Display,
				TEXT("FindFlightRoute: Primitive search found no usable route; high-altitude crossing fallback succeeded. Waypoints=%d, RouteLength=%.2f m, NetAltitudeChange=%.2f m, TotalClimb=%.2f m"),
				CurrentRouteWorldPoints.Num(), RouteLengthMeters, NetAltitudeChangeMeters, TotalClimbMeters);

			OutRoutePoints = CurrentRouteWorldPoints;
			LastFailureReason = ERouteFailureReason::None;
			return true;
		}

		UE_LOG(LogTemp, Warning,
			TEXT("FindFlightRoute: High-altitude crossing fallback was rejected too. Reason=%s"),
			*GetFailureReasonText(LastFailureReason).ToString());

		// Fallback route was not safe either; report the original failure
		CurrentRouteWorldPoints.Reset();
		CurrentRouteStates.Reset();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: High-altitude crossing fallback could not be built."));
	}

	return false;
	}

	// Store how much work the search needed across all corridor attempts
	LastExpandedStates = TotalExpandedCount;

	// Build route points from found A* path
	ReconstructRoute(Records, ReachedGoalState);

	if (CurrentRouteWorldPoints.Num() == 0 ||
		FVector::DistSquared(CurrentRouteWorldPoints.Last(), GoalPos) > 1.0f)
	{
		// Add exact goal position if search ended near the target
		CurrentRouteWorldPoints.Add(GoalPos);
	}

	// Remove unnecessary route points where safe
	if (bUseWaypointCompaction)
	{
		CompactCurrentRouteWaypoints();
	}

	if (!DoesRouteRespectAltitudeLimit(CurrentRouteWorldPoints))
	{
		// Final route still must obey aircraft ceiling
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		CurrentRouteWorldPoints.Reset();
		CurrentRouteStates.Reset();
		OutRoutePoints.Reset();
		return false;
	}

	if (!ValidateCurrentRouteSafety())
	{
		// Final route must pass all safety checks after compaction
		UE_LOG(LogTemp, Warning,
			TEXT("FindFlightRoute: The route found was discarded during the final safety check. Reason=%s"),
			*GetFailureReasonText(LastFailureReason).ToString());
		CurrentRouteWorldPoints.Reset();
		CurrentRouteStates.Reset();
		OutRoutePoints.Reset();
		return false;
	}

	// Calculate route metrics for debug output
	const float RouteLengthMeters = CalculateRouteLengthMeters();
	const float NetAltitudeChangeMeters = CalculateNetAltitudeChangeMeters();
	const float TotalClimbMeters = CalculateTotalClimbMeters();

	// Log successful route summary
	UE_LOG(
		LogTemp,
		Display,
		TEXT(
			"FindFlightRoute successful. Waypoints=%d, ExpandedStates=%d, RouteLength=%.2f m, NetAltitudeChange=%.2f m, TotalClimb=%.2f m"
		),
		CurrentRouteWorldPoints.Num(),
		TotalExpandedCount,
		RouteLengthMeters,
		NetAltitudeChangeMeters,
		TotalClimbMeters
	);

	if (bAutoDrawPathAfterSearch)
	{
		// Draw final route in the level
		DebugDrawCurrentRoute();
	}

	// Return route to caller
	OutRoutePoints = CurrentRouteWorldPoints;
	return true;
}

// Calculate route from UI input values
FRouteCalculationResult AFlightPathfinderActor::CalculateFlightRouteForUI(
	FVector StartWorldLocation,
	FVector TargetWorldLocation,
	float StartAltitudeMetersASL,
	float TargetAltitudeMetersASL,
	UFlightProfile* InFlightProfile
)
{
	// Result returned to UI
	FRouteCalculationResult Result;

	// Clear old route and cached search data
	CurrentRouteWorldPoints.Reset();
	CurrentRouteStates.Reset();
	TransitionValidityCache.Reset();
	TransitionCostCache.Reset();
	ConservativeTerrainHeightCache.Reset();
	FailureStats.Reset();

	// Reset status from previous calculation
	LastFailureReason = ERouteFailureReason::None;
	LastExpandedStates = 0;

	// Use aircraft profile selected in UI
	FlightProfile = InFlightProfile;

	if (GetWorld())
	{
		// Clear old debug drawing before new calculation
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	if (!InFlightProfile)
	{
		// UI did not provide a valid aircraft profile
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidFlightProfile;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (StartAltitudeMetersASL <= 0.0f)
	{
		// UI start altitude is not usable
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::StartAltitudeTooLow;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (TargetAltitudeMetersASL <= 0.0f)
	{
		// UI target altitude is not usable
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (InFlightProfile->MaxAltitudeMetersASL > 0.0f &&
		StartAltitudeMetersASL > InFlightProfile->MaxAltitudeMetersASL)
	{
		// Start altitude is above selected aircraft ceiling
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidStart;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = FText::Format(
			FText::FromString(TEXT("The take-off altitude ({0}m ASL) is above the maximum permitted flight altitude of this aircraft ({1}m ASL).")),
			FText::AsNumber(StartAltitudeMetersASL),
			FText::AsNumber(InFlightProfile->MaxAltitudeMetersASL)
		);
		return Result;
	}

	if (InFlightProfile->MaxAltitudeMetersASL > 0.0f &&
		TargetAltitudeMetersASL > InFlightProfile->MaxAltitudeMetersASL)
	{
		// Target altitude is above selected aircraft ceiling
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (!HeightCache)
	{
		if (GridBaker && GridBaker->HeightCache)
		{
			// Use baked cache from grid baker for ASL conversion
			HeightCache = GridBaker->HeightCache;
		}
	}

	if (!HeightCache)
	{
		// Cannot convert UI altitudes without sea level reference
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::Unknown;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = FText::FromString(TEXT("HeightCache missing."));
		return Result;
	}

	// Convert UI start altitude from meters ASL to world Z centimeters
	StartWorldLocation.Z = HeightCache->SeaLevelWorldZCm + (StartAltitudeMetersASL * 100.0f);

	// Convert UI target altitude from meters ASL to world Z centimeters
	TargetWorldLocation.Z = HeightCache->SeaLevelWorldZCm + (TargetAltitudeMetersASL * 100.0f);

	TArray<FVector> FoundRoute;

	// Run actual route search with converted world positions
	const bool bSuccess = RunFlightRouteSearch(
		StartWorldLocation,
		TargetWorldLocation,
		InFlightProfile,
		FoundRoute
	);

	// Copy search result into UI result
	Result.bSuccess = bSuccess;
	Result.RoutePoints = FoundRoute;
	Result.ExpandedStates = LastExpandedStates;

	if (!bSuccess)
	{
		// Keep failed result clean for UI drawing
		Result.RoutePoints.Empty();

		// Use detailed failure reason from route search
		Result.FailureReason = LastFailureReason;

		if (Result.FailureReason == ERouteFailureReason::None)
		{
			// Fallback if search failed without specific reason
			Result.FailureReason = ERouteFailureReason::Unknown;
			LastFailureReason = Result.FailureReason;
		}

		// Convert failure reason to UI text
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	// Mark UI result as successful
	Result.FailureReason = ERouteFailureReason::None;
	Result.FailureText = FText::GetEmpty();

	return Result;
}

// Calculate route from placed start and goal actors
void AFlightPathfinderActor::FindFlightRoute()
{
	if (!StartActor || !GoalActor)
	{
		// Editor route needs both actor markers
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: StartActor or GoalActor is missing."));
		LastFailureReason = ERouteFailureReason::InvalidStart;
		return;
	}

	TArray<FVector> FoundRoute;

	// Run route search with actor world positions
	const bool bSuccess = RunFlightRouteSearch(
		StartActor->GetActorLocation(),
		GoalActor->GetActorLocation(),
		FlightProfile,
		FoundRoute
	);

	if (!bSuccess)
	{
		// Log readable reason for failed editor route search
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: Route could not be calculated. Reason: %s"),
		       *GetFailureReasonText(LastFailureReason).ToString()
		);
	}
}

// Draw current route with debug lines and spheres
void AFlightPathfinderActor::DebugDrawCurrentRoute()
{
	if (!GetWorld())
	{
		// Debug drawing needs an active world
		return;
	}

	if (CurrentRouteWorldPoints.Num() < 2)
	{
		// Nothing useful to draw
		UE_LOG(LogTemp, Warning, TEXT("DebugDrawCurrentRoute: No valid route found."));
		return;
	}

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		// Draw route segment between two waypoints
		DrawDebugLine(
			GetWorld(),
			CurrentRouteWorldPoints[i],
			CurrentRouteWorldPoints[i + 1],
			FColor::Red,
			true,
			DebugDrawLifetime,
			0,
			DebugLineThickness
		);

		// Draw waypoint marker at segment start
		DrawDebugSphere(
			GetWorld(),
			CurrentRouteWorldPoints[i],
			40.0f,
			8,
			FColor::Yellow,
			true,
			DebugDrawLifetime,
			0,
			2.0f
		);
	}

	// Draw final waypoint marker
	DrawDebugSphere(
		GetWorld(),
		CurrentRouteWorldPoints.Last(),
		40.0f,
		8,
		FColor::Yellow,
		true,
		DebugDrawLifetime,
		0,
		2.0f
	);
}

// Clear stored route and debug drawing
void AFlightPathfinderActor::ClearCurrentRoute()
{
	// Remove current route result
	CurrentRouteWorldPoints.Reset();
	CurrentRouteStates.Reset();

	if (GetWorld())
	{
		// Remove persistent debug visuals
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	// Log manual route clear action
	UE_LOG(LogTemp, Display, TEXT("The current flight route has been cleared."));
}