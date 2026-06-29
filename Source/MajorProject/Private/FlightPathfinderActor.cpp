// Flight pathfinder actor
// Aircraft-aware A* route search
// Uses terrain height cache, flight profile and influence zones
// Search state = voxel position plus heading direction
// Validates terrain clearance, climb / descent limits, turn radius and blocked zones
// Supports direct VFR route, motion primitives, caching and UI result output
// Outputs route points and debug drawing

#include "FlightPathfinderActor.h"

#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "FlightProfile.h"
#include "FlightInfluenceZoneActor.h"

#include "DrawDebugHelpers.h"

AFlightPathfinderActor::AFlightPathfinderActor()
{
	// No runtime tick needed
	PrimaryActorTick.bCanEverTick = false;
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
		// Cannot build a route space without terrain data
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

	// Find the usable terrain height range
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

	// Ensure the start marker is inside the vertical range
	if (StartActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, StartActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, StartActor->GetActorLocation().Z);
	}

	// Ensure the goal marker is inside the vertical range
	if (GoalActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, GoalActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, GoalActor->GetActorLocation().Z);
	}

	// Build full XY bounds from the baked grid
	SearchMinWorld.X = HeightCache->GridMinWorld.X;
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y;
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	// Keep search space below the aircraft ceiling
	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	// No vertical room remains after altitude ceiling clamp
	if (SearchMaxWorld.Z <= SearchMinWorld.Z)
	{
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

// Build the vertical search space for a UI route request
void AFlightPathfinderActor::BuildSearchSpaceForRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation
)
{
	// Reset layers before rebuild
	ZLayerCount = 0;

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
		// Invalid vertical resolution
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute failed: VoxelSizeZMeters must be > 0."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	// Find usable terrain height range
	for (const float HeightCm : HeightCache->MaxHeightCm)
	{
		if (HeightCm <= -1e20f)
		{
			// Skip invalid terrain cells
			continue;
		}

		MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
		MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		// No terrain data means no route search
		UE_LOG(LogTemp, Warning,
		       TEXT("BuildSearchSpaceForRoute failed: No valid terrain heights in cache."));
		return;
	}

	// Convert vertical layer size to centimeters
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// Start with terrain range
	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	// Include requested start altitude
	MinRelevantZ = FMath::Min(MinRelevantZ, StartWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, StartWorldLocation.Z);

	// Include requested target altitude
	MinRelevantZ = FMath::Min(MinRelevantZ, TargetWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, TargetWorldLocation.Z);

	// Use full map XY bounds and route-specific Z bounds
	SearchMinWorld.X = HeightCache->GridMinWorld.X;
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y;
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);
	
	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		// Do not search above aircraft maximum altitude
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	if (SearchMaxWorld.Z <= SearchMinWorld.Z)
	{
		// Requested route is outside the usable altitude range
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute: No Z-layer within the maximum allowed flight altitude."));
		ZLayerCount = 0;
		return;
	}

	// Convert final height range to voxel layers
	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	// Log route-specific search space
	UE_LOG(LogTemp, Display, TEXT("Flight search space built for UI: XY=%d x %d, ZLayers=%d, HeadingBuckets=%d"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
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
		return FText::FromString(TEXT("Der Startpunkt ist ungültig."));

	case ERouteFailureReason::InvalidTarget:
		// Target position cannot be used
		return FText::FromString(TEXT("Der Zielpunkt ist ungültig."));

	case ERouteFailureReason::InvalidTargetState:
		// Target could not be mapped to a valid search state
		return FText::FromString(
			TEXT("Der Zielzustand ist ungültig oder kann nicht in das Suchraster übernommen werden."));

	case ERouteFailureReason::InvalidFlightProfile:
		// Missing or invalid aircraft profile
		return FText::FromString(TEXT("Kein gültiger Flugzeugtyp ausgewählt."));

	case ERouteFailureReason::StartAltitudeTooLow:
		// Start is too close to terrain
		return FText::FromString(TEXT("Die Starthöhe ist zu niedrig."));

	case ERouteFailureReason::TargetAltitudeTooLow:
		// Target is too close to terrain
		return FText::FromString(TEXT("Die Zielhöhe ist zu niedrig."));

	case ERouteFailureReason::TargetAltitudeTooHigh:
		// Target exceeds aircraft ceiling
		return FText::FromString(TEXT("Die Zielhöhe liegt über der maximal erlaubten Flughöhe des Flugzeugs."));

	case ERouteFailureReason::TargetAltitudeNotReachable:
		// Aircraft cannot reach requested target altitude
		return FText::FromString(TEXT("Die Zielflughöhe ist mit diesem Flugzeug nicht erreichbar."));

	case ERouteFailureReason::TerrainCollision:
	case ERouteFailureReason::TerrainClearance:
		// Route violates terrain safety distance
		return FText::FromString(TEXT("Die Route unterschreitet die nötige Sicherheitsdistanz zum Terrain."));

	case ERouteFailureReason::RestrictedAirspace:
	case ERouteFailureReason::HardBlockZone:
		// Route touches blocked airspace
		return FText::FromString(TEXT("Die Route führt durch eine gesperrte oder blockierte Flugzone."));

	case ERouteFailureReason::ClimbLimitExceeded:
	case ERouteFailureReason::MaxClimbExceeded:
		// Aircraft cannot climb fast enough
		return FText::FromString(TEXT("Die notwendige Steigrate überschreitet das Flugzeuglimit."));

	case ERouteFailureReason::DescentLimitExceeded:
	case ERouteFailureReason::MaxDescentExceeded:
		// Aircraft cannot descend fast enough
		return FText::FromString(TEXT("Die notwendige Sinkrate überschreitet das Flugzeuglimit."));

	case ERouteFailureReason::TurnRadiusExceeded:
	case ERouteFailureReason::TurnRadiusTooSmall:
		// Turn would be too tight for aircraft
		return FText::FromString(TEXT("Der benötigte Kurvenradius ist für dieses Flugzeug zu klein."));

	case ERouteFailureReason::NoValidNeighbors:
		// Search got stuck without valid next states
		return FText::FromString(
			TEXT("Keine gültigen Nachbarpunkte gefunden. Die Route ist durch Terrain oder Fluglimits blockiert."));

	case ERouteFailureReason::SearchLimitReached:
	case ERouteFailureReason::MaxExpandedStatesReached:
		// Search stopped before finding a route
		return FText::FromString(TEXT("Keine Route gefunden. Das maximale Suchlimit wurde erreicht."));

	case ERouteFailureReason::Unknown:
	default:
		// Fallback when no specific reason is available
		return FText::FromString(TEXT("Die Route konnte nicht berechnet werden."));
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
		State.X >= 0 && State.X < HeightCache->GridSize.X &&
		State.Y >= 0 && State.Y < HeightCache->GridSize.Y &&
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
bool AFlightPathfinderActor::WorldToNearestValidState(const FVector& WorldPos, int32 PreferredHeadingIndex,
                                                      FFlightPathState& OutState) const
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
		// Position is outside the map grid
		return false;
	}

	// Convert vertical voxel size to Unreal units
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// Start search near the requested height
	int32 StartZ = FMath::FloorToInt((WorldPos.Z - SearchMinWorld.Z) / VoxelSizeZCm);
	StartZ = FMath::Clamp(StartZ, 0, ZLayerCount - 1);

	// Use a valid heading bucket for the generated state
	const int32 NormalizedHeading = NormalizeHeadingIndex(PreferredHeadingIndex);

	// Prefer a safe state at or above the requested height
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
bool AFlightPathfinderActor::WorldToStateExact(const FVector& WorldPos, int32 HeadingIndex,
                                               FFlightPathState& OutState) const
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
		// Position is outside the grid
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
	const float SegmentLengthMeters = GetEffectivePrimitiveSegmentLengthMeters();
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
		TotalDeltaZCm = FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds *
			100.0f;
	}
	else if (VerticalMode < 0)
	{
		// Create descending primitive
		TotalDeltaZCm = -FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds *
			100.0f;
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

		// Apply climb/descent smoothly over the primitive
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

	int32 EndHeading = 0;

	// Build the same primitive shape again
	if (!BuildPrimitiveSamplePoints(
		FromState,
		HeadingDeltaBuckets,
		VerticalMode,
		OutSamplePoints,
		EndHeading))
	{
		return false;
	}
	
	if (OutSamplePoints.Num() > 0)
	{
		// Force final sample onto exact target state center
		OutSamplePoints.Last() = StateToWorldCenter(ToState);
	}

	// Rebuilt primitive must contain at least start and end
	return OutSamplePoints.Num() >= 2;
}

// Check if the search reached the goal voxel
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

// Find the closest heading bucket for a 2D direction
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

	// Check both directions around the heading circle
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
		// No valid terrain in safety area
		return false;
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
	// Start with user-configured minimum length
	float SegmentLengthMeters = FMath::Max(100.0f, PrimitiveSegmentLengthMeters);

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
	const float MaxSegmentLengthMeters = FMath::Max(PrimitiveSegmentLengthMeters, MaxAutoPrimitiveSegmentLengthMeters);
	return FMath::Min(SegmentLengthMeters, MaxSegmentLengthMeters);
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

	// Allow direct finish only near the goal
	const float AllowedConnectionMeters = FMath::Max(
		GoalConnectionToleranceMeters,
		GetEffectivePrimitiveSegmentLengthMeters() * 1.5f
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

	// Use normal final route validation for shortcut
	CurrentRouteWorldPoints = StraightRoute;
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

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
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

	if (!DoesRouteRespectTurnRadius(CurrentRouteWorldPoints))
	{
		// Route corners are not flyable
		return false;
	}
	return true;
}

// Check if point lies inside forbidden airspace
bool AFlightPathfinderActor::IsStateInsideHardBlockZone(const FVector& WorldPoint, float AltitudeMetersASL) const
{
	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!Zone)
		{
			// Ignore missing zone references
			continue;
		}
		
		if (Zone->bHardBlock && Zone->ContainsPoint(WorldPoint, AltitudeMetersASL))
		{
			// Hard block zone cannot be entered
			return true;
		}
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
	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!Zone)
		{
			// Ignore missing zone references
			continue;
		}

		if (Zone->bHardBlock && Zone->IntersectsSegmentBySampling(FromWorld, ToWorld, FromAltitudeMetersASL,
		                                                          ToAltitudeMetersASL))
		{
			// Segment intersects blocked zone
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
	// Collect all soft-zone penalties along segment
	float TotalExtraCost = 0.0f;

	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!Zone)
		{
			// Ignore missing zone references
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
	FFlightPathState& OutState
) const
{
	TArray<FVector> SamplePoints;
	int32 EndHeading = 0;

	if (!BuildPrimitiveSamplePoints(FromState, HeadingDeltaBuckets, VerticalMode, SamplePoints, EndHeading))
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
	int32 VerticalMode
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

	if (!BuildPrimitiveSamplePoints(FromState, HeadingDeltaBuckets, VerticalMode, SamplePoints, EndHeading))
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
void AFlightPathfinderActor::GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors) const
{
	// Clear previous neighbor list
	OutNeighbors.Reset();

	// Avoid duplicate states from different primitive combinations
	TSet<FFlightPathState> UniqueNeighbors;

	// Start with straight movement
	TArray<int32> HeadingOptions;
	HeadingOptions.Add(0);
	
	// Limit how much the aircraft may turn per step
	const int32 MaxTurnBuckets = FMath::Clamp(
		FMath::Max(1, FMath::Max(MaxHeadingChangePerStep, PrimitiveTurnDeltaBuckets)),
		1,
		3
	);

	for (int32 Delta = 1; Delta <= MaxTurnBuckets; ++Delta)
	{
		// Add left and right turn options
		HeadingOptions.Add(-Delta);
		HeadingOptions.Add(Delta);
	}

	// Start with level flight
	TArray<int32> VerticalModes;
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

	for (const int32 HeadingDelta : HeadingOptions)
	{
		for (const int32 VerticalMode : VerticalModes)
		{
			FFlightPathState Candidate;
			if (!ApplyMotionPrimitive(Current, HeadingDelta, VerticalMode, Candidate))
			{
				// Primitive cannot produce a usable candidate
				continue;
			}

			if (!IsStateInsideBounds(Candidate))
			{
				// Candidate ended outside search space
				continue;
			}

			if (!IsMotionPrimitiveValid(Current, Candidate, HeadingDelta, VerticalMode))
			{
				// Candidate movement violates flight rules
				continue;
			}

			// Store only valid and unique neighbors
			UniqueNeighbors.Add(Candidate);
		}
	}

	// Return generated neighbors to A*
	OutNeighbors = UniqueNeighbors.Array();

	if (OutNeighbors.Num() == 0)
	{
		// Current state is a dead end
		FailureStats.NoValidNeighborsCount++;
		LastFailureReason = ERouteFailureReason::NoValidNeighbors;
	}
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

	// Add extra pressure for routes that still need to climb
	const float DeltaZMeters = FMath::Max(0.0f, (BWorld.Z - AWorld.Z) / 100.0f);
	if (DeltaZMeters > 0.0f)
	{
		// Climb takes time and should influence search direction
		Cost += (DeltaZMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 4.0f;
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

	// Penalize climbs so the search plans altitude changes earlier
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;
	if (DeltaZMeters > 0.0f)
	{
		const float ClimbPenalty =
			(DeltaZMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 25.0f;
		Cost += ClimbPenalty;
	}
	
	if (DeltaZMeters < 0.0f)
	{
		// Penalize descents less than climbs, but avoid nervous altitude changes
		const float DescentPenalty =
			(FMath::Abs(DeltaZMeters) / FMath::Max(0.1f, FlightProfile->MaxDescentRateMetersPerSecond)) * 8.0f;
		Cost += DescentPenalty;
	}

	// Penalize heading changes so smoother routes are preferred
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	if (HeadingDeltaBuckets > 0)
	{
		const float TurnPenalty =
			static_cast<float>(HeadingDeltaBuckets * HeadingDeltaBuckets) *
			(FMath::Max(1.0f, FlightProfile->MinimumTurnRadiusMeters) / FMath::Max(50.0f, GetEffectivePrimitiveSegmentLengthMeters())) * 25.0f;

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
		if (GetConservativeTerrainHeightCmAtWorldXY(Sample.X, Sample.Y, GetPreferredTerrainSafetyRadiusMeters(), TerrainHeightCm))
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
			// Still legal, but close to terrain, so make it less attractive
			const float ClearanceDeficit = PreferredClearanceMeters - LowestClearanceMeters;
			Cost += ClearanceDeficit * 6.0f;
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
			// Keep route away from the altitude ceiling if possible
			Cost += (PreferredCeilingMarginMeters - CeilingMarginMeters) * 40.0f;
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

	TArray<FFlightPathState> StatePath;
	FFlightPathState Current = GoalState;

	while (true)
	{
		// Insert current state at front to build start-to-goal order
		StatePath.Insert(Current, 0);

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

	// Convert state path into world route points
	CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[0]));
	for (int32 i = 1; i < StatePath.Num(); ++i)
	{
		CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[i]));
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

			// Avoid replacing too much route structure at once
			const float MaxCompactSegmentMeters = FMath::Max(
				MinOutputWaypointSpacingMeters * 4.0f,
				FMath::Max(GetEffectivePrimitiveSegmentLengthMeters() * 4.0f, FlightProfile ? FlightProfile->MinimumTurnRadiusMeters * 4.0f : 0.0f)
			);

			if (TestIndex != LastIndex && DistanceMeters > MaxCompactSegmentMeters)
			{
				// Direct replacement would be too long
				continue;
			}

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

	// Replace original route with simplified route
	CurrentRouteWorldPoints = CompactRoute;
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
	CurrentRouteWorldPoints.Reset();
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
			// Reject start above aircraft ceiling
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

	// Build search grid around requested route
	BuildSearchSpaceForRoute(StartWorldLocation, TargetWorldLocation);

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

	// Store best known cost and parent for each state
	TMap<FFlightPathState, FFlightPathNodeRecord> Records;

	// Initialize A* at start state
	FFlightPathNodeRecord StartRecord;
	StartRecord.G = 0.0f;
	StartRecord.H = HeuristicCost(StartState, GoalState);
	StartRecord.F = StartRecord.G + HeuristicWeight * StartRecord.H;
	StartRecord.bHasParent = false;
	StartRecord.bClosed = false;

	// Add start state to search
	Records.Add(StartState, StartRecord);
	OpenHeap.HeapPush(
		FFlightOpenEntry(StartState, StartRecord.F),
		FFlightOpenEntryMinHeapPredicate()
	);

	// Track A* progress and limits
	int32 ExpandedCount = 0;
	const int32 MaxExpandedStates = FMath::Max(10000, SearchMaxExpandedStates);
	bool bFoundRoute = false;

	// Store the state where route search finished
	FFlightPathState ReachedGoalState;

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
		
		// Generate aircraft-valid next states
		TArray<FFlightPathState> Neighbors;
		GetNeighbors(Current, Neighbors);

		for (const FFlightPathState& Neighbor : Neighbors)
		{
			// Skip neighbor if it was already finalized
			FFlightPathNodeRecord* ExistingRecord = Records.Find(Neighbor);
			if (ExistingRecord && ExistingRecord->bClosed)
			{
				continue;
			}

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
			ExistingRecord->F = ExistingRecord->G + HeuristicWeight * ExistingRecord->H;
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

	// Store how much work the search needed
	LastExpandedStates = ExpandedCount;

	if (!bFoundRoute)
	{
		if (ExpandedCount >= MaxExpandedStates)
		{
			// Search stopped because the work limit was reached
			LastFailureReason = ERouteFailureReason::MaxExpandedStatesReached;
		}

		// Log failed search size
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: No route found. ExpandedStates=%d"), ExpandedCount);

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
		return false;
	}

	// Build route points from found A* path
	ReconstructRoute(Records, ReachedGoalState);

	if (CurrentRouteWorldPoints.Num() == 0 ||
		FVector::DistSquared(CurrentRouteWorldPoints.Last(), GoalPos) > 1.0f)
	{
		// Add exact goal position if search ended near the target
		CurrentRouteWorldPoints.Add(GoalPos);
	}

	// Remove unnecessary route points where safe
	CompactCurrentRouteWaypoints();

	if (!DoesRouteRespectAltitudeLimit(CurrentRouteWorldPoints))
	{
		// Final route still must obey aircraft ceiling
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		CurrentRouteWorldPoints.Reset();
		OutRoutePoints.Reset();
		return false;
	}

	if (!ValidateCurrentRouteSafety())
	{
		// Final route must pass all safety checks after compaction
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: The route found was discarded during the final safety check."));
		CurrentRouteWorldPoints.Reset();
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
		ExpandedCount,
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
		Result.FailureText = FText::Format(FText::FromString(TEXT("Die Starthoehe ({0}m ASL) liegt ueber der maximal erlaubten Flughoehe dieses Flugzeugs ({1}m ASL).")), FText::AsNumber(StartAltitudeMetersASL), FText::AsNumber(InFlightProfile->MaxAltitudeMetersASL));
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
		Result.FailureText = FText::FromString(TEXT("HeightCache fehlt."));
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

	if (GetWorld())
	{
		// Remove persistent debug visuals
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	UE_LOG(LogTemp, Display, TEXT("The current flight route has been cleared."));
}