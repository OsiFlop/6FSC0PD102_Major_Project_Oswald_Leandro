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

// Check required references and basic profile values
bool AFlightPathfinderActor::ValidateReferences() const
{
	if (!GridBaker)
	{
		// Missing terrain grid source
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: GridBaker is missing."));
		return false;
	}

	if (!HeightCache)
	{
		// Missing baked terrain height data
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is missing."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		// Height cache not baked or invalid
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is invalid."));
		return false;
	}

	if (!FlightProfile)
	{
		// Missing aircraft profile
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: FlightProfile is missing."));
		return false;
	}

	if (!StartActor || !GoalActor)
	{
		// Missing actor route endpoints
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: StartActor or GoalActor is missing."));
		return false;
	}
	
	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Speed needed for travel time
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: CruiseSpeedMetersPerSecond must be > 0."));
		return false;
	}
	
	if (HeadingBucketCount < 4)
	{
		// Too few heading directions
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeadingBucketCount must be >= 4."));
		return false;
	}

	return true;
}

// Check core references for UI and internal route search
bool AFlightPathfinderActor::ValidateCoreReferences() const
{
	if (!GridBaker)
	{
		// Missing terrain grid source
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: GridBaker is missing."));
		return false;
	}

	if (!HeightCache)
	{
		// Missing baked terrain height data
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is missing."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		// Height cache not baked or invalid
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache is invalid."));
		return false;
	}

	if (!FlightProfile)
	{
		// Missing aircraft profile
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: FlightProfile is missing."));
		return false;
	}

	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		// Speed needed for travel time
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: CruiseSpeedMetersPerSecond must be > 0."));
		return false;
	}

	if (HeadingBucketCount < 4)
	{
		// Too few heading directions
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

	// Auto-link height cache from baker
	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!HeightCache || !HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace fehlgeschlagen: HeightCache fehlt oder ist ungueltig."));
		return;
	}

	if (VoxelSizeZMeters <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace fehlgeschlagen: VoxelSizeZMeters muss > 0 sein."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	// MinTerrainZ / MaxTerrainZ: valid terrain height range
	for (const float HeightCm : HeightCache->MaxHeightCm)
	{
		if (HeightCm <= -1e20f)
		{
			continue;
		}

		MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
		MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace fehlgeschlagen: Keine gueltigen Terrainhoehen im Cache."));
		return;
	}

	// VoxelSizeZCm: vertical layer size in Unreal centimeters
	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	// Include start height in search space
	if (StartActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, StartActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, StartActor->GetActorLocation().Z);
	}

	// Include goal height in search space
	if (GoalActor)
	{
		MinRelevantZ = FMath::Min(MinRelevantZ, GoalActor->GetActorLocation().Z);
		MaxRelevantZ = FMath::Max(MaxRelevantZ, GoalActor->GetActorLocation().Z);
	}

	// SearchMinWorld / SearchMaxWorld: full 3D search bounds
	SearchMinWorld.X = HeightCache->GridMinWorld.X;
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y;
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	// Clamp search space to the aircraft altitude ceiling
	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	if (SearchMaxWorld.Z <= SearchMinWorld.Z)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpace: Keine Z-Schicht innerhalb der maximal erlaubten Flughoehe."));
		ZLayerCount = 0;
		return;
	}
	
	// ZLayerCount: amount of vertical search layers
	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	UE_LOG(LogTemp, Display, TEXT("Flight search space gebaut: XY=%d x %d, ZLayers=%d, HeadingBuckets=%d"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
	       ZLayerCount,
	       HeadingBucketCount);
}

void AFlightPathfinderActor::BuildSearchSpaceForRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation
)
{
	ZLayerCount = 0;

	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!HeightCache || !HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("BuildSearchSpaceForRoute fehlgeschlagen: HeightCache fehlt oder ist ungueltig."));
		return;
	}

	if (VoxelSizeZMeters <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute fehlgeschlagen: VoxelSizeZMeters muss > 0 sein."));
		return;
	}

	float MinTerrainZ = TNumericLimits<float>::Max();
	float MaxTerrainZ = -TNumericLimits<float>::Max();

	for (const float HeightCm : HeightCache->MaxHeightCm)
	{
		if (HeightCm <= -1e20f)
		{
			continue;
		}

		MinTerrainZ = FMath::Min(MinTerrainZ, HeightCm);
		MaxTerrainZ = FMath::Max(MaxTerrainZ, HeightCm);
	}

	if (MinTerrainZ == TNumericLimits<float>::Max() || MaxTerrainZ == -TNumericLimits<float>::Max())
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("BuildSearchSpaceForRoute fehlgeschlagen: Keine gueltigen Terrainhoehen im Cache."));
		return;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	float MinRelevantZ = MinTerrainZ;
	float MaxRelevantZ = MaxTerrainZ;

	// Include UI-provided start and target altitudes in the search space
	MinRelevantZ = FMath::Min(MinRelevantZ, StartWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, StartWorldLocation.Z);

	MinRelevantZ = FMath::Min(MinRelevantZ, TargetWorldLocation.Z);
	MaxRelevantZ = FMath::Max(MaxRelevantZ, TargetWorldLocation.Z);

	SearchMinWorld.X = HeightCache->GridMinWorld.X;
	SearchMinWorld.Y = HeightCache->GridMinWorld.Y;
	SearchMinWorld.Z = MinRelevantZ - (ExtraBottomLayers * VoxelSizeZCm);

	SearchMaxWorld.X = HeightCache->GridMinWorld.X + (HeightCache->GridSize.X * HeightCache->CellSizeCm);
	SearchMaxWorld.Y = HeightCache->GridMinWorld.Y + (HeightCache->GridSize.Y * HeightCache->CellSizeCm);
	SearchMaxWorld.Z = MaxRelevantZ + (ExtraTopLayers * VoxelSizeZCm);

	// Clamp search space to the aircraft altitude ceiling
	if (FlightProfile && FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		const float MaxAllowedWorldZ = HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
		SearchMaxWorld.Z = FMath::Min(SearchMaxWorld.Z, MaxAllowedWorldZ);
	}

	if (SearchMaxWorld.Z <= SearchMinWorld.Z)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildSearchSpaceForRoute: Keine Z-Schicht innerhalb der maximal erlaubten Flughoehe."));
		ZLayerCount = 0;
		return;
	}

	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	UE_LOG(LogTemp, Display, TEXT("Flight search space fuer UI gebaut: XY=%d x %d, ZLayers=%d, HeadingBuckets=%d"),
	       HeightCache->GridSize.X,
	       HeightCache->GridSize.Y,
	       ZLayerCount,
	       HeadingBucketCount
	);
}

FText AFlightPathfinderActor::GetFailureReasonText(ERouteFailureReason Reason) const
{
	switch (Reason)
	{
	case ERouteFailureReason::None:
		return FText::GetEmpty();

	case ERouteFailureReason::InvalidStart:
		return FText::FromString(TEXT("Der Startpunkt ist ungültig."));

	case ERouteFailureReason::InvalidTarget:
		return FText::FromString(TEXT("Der Zielpunkt ist ungültig."));

	case ERouteFailureReason::InvalidTargetState:
		return FText::FromString(
			TEXT("Der Zielzustand ist ungültig oder kann nicht in das Suchraster übernommen werden."));

	case ERouteFailureReason::InvalidFlightProfile:
		return FText::FromString(TEXT("Kein gültiger Flugzeugtyp ausgewählt."));

	case ERouteFailureReason::StartAltitudeTooLow:
		return FText::FromString(TEXT("Die Starthöhe ist zu niedrig."));

	case ERouteFailureReason::TargetAltitudeTooLow:
		return FText::FromString(TEXT("Die Zielhöhe ist zu niedrig."));

	case ERouteFailureReason::TargetAltitudeTooHigh:
		return FText::FromString(TEXT("Die Zielhöhe liegt über der maximal erlaubten Flughöhe des Flugzeugs."));

	case ERouteFailureReason::TargetAltitudeNotReachable:
		return FText::FromString(TEXT("Die Zielflughöhe ist mit diesem Flugzeug nicht erreichbar."));

	case ERouteFailureReason::TerrainCollision:
	case ERouteFailureReason::TerrainClearance:
		return FText::FromString(TEXT("Die Route unterschreitet die nötige Sicherheitsdistanz zum Terrain."));

	case ERouteFailureReason::RestrictedAirspace:
	case ERouteFailureReason::HardBlockZone:
		return FText::FromString(TEXT("Die Route führt durch eine gesperrte oder blockierte Flugzone."));

	case ERouteFailureReason::ClimbLimitExceeded:
	case ERouteFailureReason::MaxClimbExceeded:
		return FText::FromString(TEXT("Die notwendige Steigrate überschreitet das Flugzeuglimit."));

	case ERouteFailureReason::DescentLimitExceeded:
	case ERouteFailureReason::MaxDescentExceeded:
		return FText::FromString(TEXT("Die notwendige Sinkrate überschreitet das Flugzeuglimit."));

	case ERouteFailureReason::TurnRadiusExceeded:
	case ERouteFailureReason::TurnRadiusTooSmall:
		return FText::FromString(TEXT("Der benötigte Kurvenradius ist für dieses Flugzeug zu klein."));

	case ERouteFailureReason::NoValidNeighbors:
		return FText::FromString(
			TEXT("Keine gültigen Nachbarpunkte gefunden. Die Route ist durch Terrain oder Fluglimits blockiert."));

	case ERouteFailureReason::SearchLimitReached:
	case ERouteFailureReason::MaxExpandedStatesReached:
		return FText::FromString(TEXT("Keine Route gefunden. Das maximale Suchlimit wurde erreicht."));

	case ERouteFailureReason::Unknown:
	default:
		return FText::FromString(TEXT("Die Route konnte nicht berechnet werden."));
	}
}

// Check if state is inside search bounds
bool AFlightPathfinderActor::IsStateInsideBounds(const FFlightPathState& State) const
{
	if (!HeightCache)
	{
		return false;
	}

	return
		State.X >= 0 && State.X < HeightCache->GridSize.X &&
		State.Y >= 0 && State.Y < HeightCache->GridSize.Y &&
		State.Z >= 0 && State.Z < ZLayerCount;
}

// Convert flight state to world-space center point
FVector AFlightPathfinderActor::StateToWorldCenter(const FFlightPathState& State) const
{
	const float WorldX = HeightCache->GridMinWorld.X + (static_cast<float>(State.X) + 0.5f) * HeightCache->CellSizeCm;
	const float WorldY = HeightCache->GridMinWorld.Y + (static_cast<float>(State.Y) + 0.5f) * HeightCache->CellSizeCm;
	const float WorldZ = SearchMinWorld.Z + (static_cast<float>(State.Z) + 0.5f) * (VoxelSizeZMeters * 100.0f);

	return FVector(WorldX, WorldY, WorldZ);
}

// Find nearest valid state for a world position
bool AFlightPathfinderActor::WorldToNearestValidState(const FVector& WorldPos, int32 PreferredHeadingIndex,
                                                      FFlightPathState& OutState) const
{
	if (!HeightCache || !HeightCache->IsValid() || ZLayerCount <= 0)
	{
		return false;
	}

	// LocalX / LocalY: world position relative to grid origin
	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;

	// StartZ: closest vertical layer to world height
	int32 StartZ = FMath::FloorToInt((WorldPos.Z - SearchMinWorld.Z) / VoxelSizeZCm);
	StartZ = FMath::Clamp(StartZ, 0, ZLayerCount - 1);

	// NormalizedHeading: safe bucket index
	const int32 NormalizedHeading = NormalizeHeadingIndex(PreferredHeadingIndex);

	// Prefer valid state above position
	for (int32 Z = StartZ; Z < ZLayerCount; ++Z)
	{
		const FFlightPathState Candidate(GridX, GridY, Z, NormalizedHeading);
		if (IsStateValid(Candidate))
		{
			OutState = Candidate;
			return true;
		}
	}

	// Fallback: search below position
	for (int32 Z = StartZ - 1; Z >= 0; --Z)
	{
		const FFlightPathState Candidate(GridX, GridY, Z, NormalizedHeading);
		if (IsStateValid(Candidate))
		{
			OutState = Candidate;
			return true;
		}
	}

	return false;
}

bool AFlightPathfinderActor::WorldToStateExact(const FVector& WorldPos, int32 HeadingIndex,
                                               FFlightPathState& OutState) const
{
	if (!HeightCache || !HeightCache->IsValid() || ZLayerCount <= 0)
	{
		return false;
	}

	const float LocalX = WorldPos.X - HeightCache->GridMinWorld.X;
	const float LocalY = WorldPos.Y - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float VoxelSizeZCm = VoxelSizeZMeters * 100.0f;
	const int32 GridZ = FMath::FloorToInt((WorldPos.Z - SearchMinWorld.Z) / VoxelSizeZCm);

	if (GridZ < 0 || GridZ >= ZLayerCount)
	{
		return false;
	}

	OutState = FFlightPathState(GridX, GridY, GridZ, NormalizeHeadingIndex(HeadingIndex));
	return true;
}

bool AFlightPathfinderActor::BuildPrimitiveSamplePoints(
	const FFlightPathState& FromState,
	int32 HeadingDeltaBuckets,
	int32 VerticalMode,
	TArray<FVector>& OutSamplePoints,
	int32& OutEndHeading
) const
{
	OutSamplePoints.Reset();

	if (!FlightProfile || PrimitiveSamplesPerSegment < 2)
	{
		return false;
	}

	const FVector StartWorld = StateToWorldCenter(FromState);

	const int32 StartHeading = FromState.HeadingIndex;
	const int32 EndHeading = NormalizeHeadingIndex(StartHeading + HeadingDeltaBuckets);
	OutEndHeading = EndHeading;

	const float StartAngle = HeadingIndexToAngleRad(StartHeading);
	const float EndAngle = HeadingIndexToAngleRad(EndHeading);

	float SignedDeltaAngle = EndAngle - StartAngle;

	while (SignedDeltaAngle > PI)
	{
		SignedDeltaAngle -= 2.0f * PI;
	}
	while (SignedDeltaAngle < -PI)
	{
		SignedDeltaAngle += 2.0f * PI;
	}

	const float SegmentLengthMeters = GetEffectivePrimitiveSegmentLengthMeters();
	const float SegmentLengthCm = SegmentLengthMeters * 100.0f;
	const int32 EffectiveSampleCount = FMath::Clamp(
		FMath::Max(PrimitiveSamplesPerSegment, FMath::CeilToInt(SegmentLengthMeters / 150.0f)),
		2,
		32
	);
	const float StepLengthCm = SegmentLengthCm / static_cast<float>(EffectiveSampleCount);

	const float TravelTimeSeconds = SegmentLengthMeters / FlightProfile->CruiseSpeedMetersPerSecond;
	const float ClimbRateFactor = FMath::Clamp(FMath::Max(PrimitiveClimbRateFactor, 0.75f), 0.1f, 1.0f);

	float TotalDeltaZCm = 0.0f;
	if (VerticalMode > 0)
	{
		TotalDeltaZCm = FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds *
			100.0f;
	}
	else if (VerticalMode < 0)
	{
		TotalDeltaZCm = -FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor * TravelTimeSeconds *
			100.0f;
	}

	FVector CurrentWorld = StartWorld;
	OutSamplePoints.Add(CurrentWorld);

	for (int32 Step = 1; Step <= EffectiveSampleCount; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(EffectiveSampleCount);
		const float CurrentAngle = StartAngle + SignedDeltaAngle * Alpha;

		CurrentWorld.X += FMath::Cos(CurrentAngle) * StepLengthCm;
		CurrentWorld.Y += FMath::Sin(CurrentAngle) * StepLengthCm;
		CurrentWorld.Z = StartWorld.Z + TotalDeltaZCm * Alpha;

		OutSamplePoints.Add(CurrentWorld);
	}

	return OutSamplePoints.Num() >= 2;
}

bool AFlightPathfinderActor::RebuildPrimitiveSamplesBetweenStates(
	const FFlightPathState& FromState,
	const FFlightPathState& ToState,
	TArray<FVector>& OutSamplePoints
) const
{
	OutSamplePoints.Reset();

	const int32 HeadingDeltaBuckets = GetSignedHeadingDeltaBuckets(
		FromState.HeadingIndex,
		ToState.HeadingIndex
	);

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
		OutSamplePoints.Last() = StateToWorldCenter(ToState);
	}

	return OutSamplePoints.Num() >= 2;
}

// Check if current state reached goal voxel
bool AFlightPathfinderActor::IsGoalState(const FFlightPathState& Current, const FFlightPathState& Goal) const
{
	return Current.X == Goal.X && Current.Y == Goal.Y && Current.Z == Goal.Z;
}

// Wrap heading index into valid bucket range
int32 AFlightPathfinderActor::NormalizeHeadingIndex(int32 HeadingIndex) const
{
	if (HeadingBucketCount <= 0)
	{
		return 0;
	}

	int32 Result = HeadingIndex % HeadingBucketCount;
	if (Result < 0)
	{
		Result += HeadingBucketCount;
	}

	return Result;
}

// Convert heading bucket to angle in radians
float AFlightPathfinderActor::HeadingIndexToAngleRad(int32 HeadingIndex) const
{
	const int32 Normalized = NormalizeHeadingIndex(HeadingIndex);
	const float Fraction = static_cast<float>(Normalized) / static_cast<float>(HeadingBucketCount);

	return Fraction * 2.0f * PI;
}

// Convert heading bucket to one-cell XY grid step
FIntPoint AFlightPathfinderActor::HeadingIndexToGridStep(int32 HeadingIndex) const
{
	const float Angle = HeadingIndexToAngleRad(HeadingIndex);

	const int32 StepX = FMath::RoundToInt(FMath::Cos(Angle));
	const int32 StepY = FMath::RoundToInt(FMath::Sin(Angle));

	return FIntPoint(StepX, StepY);
}

// Find closest heading bucket from 2D direction
int32 AFlightPathfinderActor::ComputeNearestHeadingIndexFromDirection(const FVector2D& Direction) const
{
	if (Direction.IsNearlyZero())
	{
		return 0;
	}

	const float Angle = FMath::Atan2(Direction.Y, Direction.X);

	// NormalizedAngle: angle range 0 to 2PI
	float NormalizedAngle = Angle;
	if (NormalizedAngle < 0.0f)
	{
		NormalizedAngle += 2.0f * PI;
	}

	const float Fraction = NormalizedAngle / (2.0f * PI);
	return NormalizeHeadingIndex(FMath::RoundToInt(Fraction * static_cast<float>(HeadingBucketCount)));
}

// Get shortest heading change between two buckets
int32 AFlightPathfinderActor::GetSmallestHeadingDelta(int32 FromHeading, int32 ToHeading) const
{
	const int32 A = NormalizeHeadingIndex(FromHeading);
	const int32 B = NormalizeHeadingIndex(ToHeading);

	const int32 Forward = (B - A + HeadingBucketCount) % HeadingBucketCount;
	const int32 Backward = (A - B + HeadingBucketCount) % HeadingBucketCount;

	return FMath::Min(Forward, Backward);
}

int32 AFlightPathfinderActor::GetSignedHeadingDeltaBuckets(int32 FromHeading, int32 ToHeading) const
{
	const int32 A = NormalizeHeadingIndex(FromHeading);
	const int32 B = NormalizeHeadingIndex(ToHeading);

	int32 Delta = B - A;

	if (Delta > HeadingBucketCount / 2)
	{
		Delta -= HeadingBucketCount;
	}
	else if (Delta < -HeadingBucketCount / 2)
	{
		Delta += HeadingBucketCount;
	}

	return Delta;
}

// Get cached terrain height at grid cell
float AFlightPathfinderActor::GetTerrainHeightCmAtCell(int32 X, int32 Y) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return -FLT_MAX;
	}

	const int32 Index = HeightCache->ToIndex(X, Y);
	if (!HeightCache->MaxHeightCm.IsValidIndex(Index))
	{
		return -FLT_MAX;
	}

	return HeightCache->MaxHeightCm[Index];
}

// Get terrain height from world X / Y position
bool AFlightPathfinderActor::GetTerrainHeightCmAtWorldXY(float WorldX, float WorldY, float& OutTerrainHeightCm) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

	// Convert world XY to grid cell
	const float LocalX = WorldX - HeightCache->GridMinWorld.X;
	const float LocalY = WorldY - HeightCache->GridMinWorld.Y;

	const int32 GridX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 GridY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (GridX < 0 || GridY < 0 || GridX >= HeightCache->GridSize.X || GridY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float TerrainHeightCm = GetTerrainHeightCmAtCell(GridX, GridY);
	if (TerrainHeightCm <= -1e20f)
	{
		return false;
	}

	OutTerrainHeightCm = TerrainHeightCm;
	return true;
}

// Convert world Z in centimeters to altitude meters ASL
float AFlightPathfinderActor::GetAltitudeMetersASLFromWorldZ(float WorldZCm) const
{
	if (!HeightCache)
	{
		return 0.0f;
	}

	return (WorldZCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

// Get terrain height at grid cell in meters ASL
float AFlightPathfinderActor::GetTerrainHeightMetersASLAtCell(int32 X, int32 Y) const
{
	const float TerrainHeightCm = GetTerrainHeightCmAtCell(X, Y);
	if (TerrainHeightCm <= -1e20f || !HeightCache)
	{
		return -FLT_MAX;
	}

	return (TerrainHeightCm - HeightCache->SeaLevelWorldZCm) / 100.0f;
}

bool AFlightPathfinderActor::GetConservativeTerrainHeightCmAtWorldXY(
	float WorldX,
	float WorldY,
	float HorizontalSafetyRadiusMeters,
	float& OutTerrainHeightCm
) const
{
	if (!HeightCache || !HeightCache->IsValid())
	{
		return false;
	}

	const float LocalX = WorldX - HeightCache->GridMinWorld.X;
	const float LocalY = WorldY - HeightCache->GridMinWorld.Y;

	const int32 CenterX = FMath::FloorToInt(LocalX / HeightCache->CellSizeCm);
	const int32 CenterY = FMath::FloorToInt(LocalY / HeightCache->CellSizeCm);

	if (CenterX < 0 || CenterY < 0 || CenterX >= HeightCache->GridSize.X || CenterY >= HeightCache->GridSize.Y)
	{
		return false;
	}

	const float RadiusCm = FMath::Max(0.0f, HorizontalSafetyRadiusMeters) * 100.0f;
	const int32 RadiusCells = FMath::Max(0, FMath::CeilToInt(RadiusCm / HeightCache->CellSizeCm));

	const FIntVector CacheKey(CenterX, CenterY, RadiusCells);
	if (const float* CachedHeight = ConservativeTerrainHeightCache.Find(CacheKey))
	{
		OutTerrainHeightCm = *CachedHeight;
		return true;
	}

	float MaxTerrainHeightCm = -FLT_MAX;
	bool bFoundTerrain = false;

	const int32 MinX = FMath::Max(0, CenterX - RadiusCells);
	const int32 MaxX = FMath::Min(HeightCache->GridSize.X - 1, CenterX + RadiusCells);
	const int32 MinY = FMath::Max(0, CenterY - RadiusCells);
	const int32 MaxY = FMath::Min(HeightCache->GridSize.Y - 1, CenterY + RadiusCells);

	for (int32 Y = MinY; Y <= MaxY; ++Y)
	{
		for (int32 X = MinX; X <= MaxX; ++X)
		{
			const float TerrainHeightCm = GetTerrainHeightCmAtCell(X, Y);
			if (TerrainHeightCm <= -1e20f)
			{
				continue;
			}

			MaxTerrainHeightCm = FMath::Max(MaxTerrainHeightCm, TerrainHeightCm);
			bFoundTerrain = true;
		}
	}

	if (!bFoundTerrain)
	{
		return false;
	}

	OutTerrainHeightCm = MaxTerrainHeightCm;
	ConservativeTerrainHeightCache.Add(CacheKey, MaxTerrainHeightCm);
	return true;
}

float AFlightPathfinderActor::GetRequiredTerrainClearanceMeters() const
{
	if (!FlightProfile)
	{
		return FMath::Max(0.0f, MinimumAbsoluteTerrainClearanceMeters);
	}

	const float ProfileClearanceMeters = FMath::Max(0.0f, FlightProfile->MinimumTerrainClearanceMeters);
	const float BaseClearanceMeters = FMath::Max(ProfileClearanceMeters, MinimumAbsoluteTerrainClearanceMeters);
	return BaseClearanceMeters;
}

float AFlightPathfinderActor::GetPreferredTerrainClearanceMeters() const
{
	const float RequiredClearanceMeters = GetRequiredTerrainClearanceMeters();
	if (!FlightProfile)
	{
		return RequiredClearanceMeters * 2.0f;
	}

	const float SpeedBufferMeters =
		FMath::Max(0.0f, FlightProfile->CruiseSpeedMetersPerSecond) *
		FMath::Max(0.0f, PreferredClearanceSpeedLookaheadSeconds) *
		0.2f;
	const float TurnBufferMeters = FMath::Max(0.0f, FlightProfile->MinimumTurnRadiusMeters) * 0.1f;

	const float SafetyClearanceMeters =
		RequiredClearanceMeters * FMath::Max(1.0f, TerrainClearanceSafetyMultiplier);

	return FMath::Max(RequiredClearanceMeters, SafetyClearanceMeters + SpeedBufferMeters + TurnBufferMeters);
}

float AFlightPathfinderActor::GetTerrainSafetyRadiusMeters() const
{
	// Hard clearance uses the baked max height of the current voxel cell.
	// Wider profile-dependent buffers are handled as costs, not as start/goal blockers.
	return 0.0f;
}

float AFlightPathfinderActor::GetPreferredTerrainSafetyRadiusMeters() const
{
	if (!FlightProfile)
	{
		return 0.0f;
	}

	const float SpeedLookaheadRadiusMeters =
		FMath::Max(0.0f, FlightProfile->CruiseSpeedMetersPerSecond) *
		FMath::Max(0.0f, PreferredClearanceSpeedLookaheadSeconds) *
		0.35f;
	const float TurnRadiusMeters =
		FMath::Max(0.0f, FlightProfile->MinimumTurnRadiusMeters) *
		FMath::Max(0.0f, LateralTerrainSafetyRadiusMultiplier);
	const float HalfCellMeters = HeightCache ? (HeightCache->CellSizeCm / 200.0f) : 0.0f;

	const float PreferredRadiusMeters = FMath::Max(
		FMath::Max(SpeedLookaheadRadiusMeters, TurnRadiusMeters),
		HalfCellMeters
	);

	return FMath::Clamp(PreferredRadiusMeters, 0.0f, 5000.0f);
}

float AFlightPathfinderActor::GetEffectivePrimitiveSegmentLengthMeters() const
{
	float SegmentLengthMeters = FMath::Max(100.0f, PrimitiveSegmentLengthMeters);

	if (FlightProfile && FlightProfile->CruiseSpeedMetersPerSecond > 0.0f)
	{
		const float AnglePerBucketRad = (HeadingBucketCount > 0)
			? ((2.0f * PI) / static_cast<float>(HeadingBucketCount))
			: 0.0f;

		if (FlightProfile->MinimumTurnRadiusMeters > 0.0f && AnglePerBucketRad > KINDA_SMALL_NUMBER)
		{
			SegmentLengthMeters = FMath::Max(
				SegmentLengthMeters,
				FlightProfile->MinimumTurnRadiusMeters * AnglePerBucketRad * 1.15f
			);
		}

		const float ClimbRateFactor = FMath::Clamp(FMath::Max(PrimitiveClimbRateFactor, 0.75f), 0.1f, 1.0f);
		const float VerticalLayerMeters = FMath::Max(1.0f, VoxelSizeZMeters);

		if (bAllowClimbStep && FlightProfile->MaxClimbRateMetersPerSecond > 0.0f)
		{
			const float DistanceForOneClimbLayer =
				(VerticalLayerMeters * FlightProfile->CruiseSpeedMetersPerSecond) /
				(FlightProfile->MaxClimbRateMetersPerSecond * ClimbRateFactor);
			SegmentLengthMeters = FMath::Max(SegmentLengthMeters, DistanceForOneClimbLayer * 1.1f);
		}

		if (bAllowDescentStep && FlightProfile->MaxDescentRateMetersPerSecond > 0.0f)
		{
			const float DistanceForOneDescentLayer =
				(VerticalLayerMeters * FlightProfile->CruiseSpeedMetersPerSecond) /
				(FlightProfile->MaxDescentRateMetersPerSecond * ClimbRateFactor);
			SegmentLengthMeters = FMath::Max(SegmentLengthMeters, DistanceForOneDescentLayer * 1.05f);
		}
	}

	const float MaxSegmentLengthMeters = FMath::Max(PrimitiveSegmentLengthMeters, MaxAutoPrimitiveSegmentLengthMeters);
	return FMath::Min(SegmentLengthMeters, MaxSegmentLengthMeters);
}

float AFlightPathfinderActor::GetMaxAllowedWorldZCm() const
{
	if (!FlightProfile || !HeightCache || FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		return FLT_MAX;
	}

	return HeightCache->SeaLevelWorldZCm + (FlightProfile->MaxAltitudeMetersASL * 100.0f);
}

bool AFlightPathfinderActor::DoesPointRespectAltitudeLimit(const FVector& WorldPoint) const
{
	const float MaxAllowedWorldZCm = GetMaxAllowedWorldZCm();
	if (MaxAllowedWorldZCm >= FLT_MAX * 0.5f)
	{
		return true;
	}

	return WorldPoint.Z <= MaxAllowedWorldZCm + 1.0f;
}

bool AFlightPathfinderActor::DoesRouteRespectAltitudeLimit(const TArray<FVector>& RoutePoints) const
{
	if (!FlightProfile)
	{
		return false;
	}

	if (FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		return true;
	}

	for (const FVector& RoutePoint : RoutePoints)
	{
		if (!DoesPointRespectAltitudeLimit(RoutePoint))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FlightPathfinder: Route verworfen, Punkt %.2fm ASL liegt ueber MaxAltitude %.2fm."),
				GetAltitudeMetersASLFromWorldZ(RoutePoint.Z),
				FlightProfile->MaxAltitudeMetersASL
			);
			return false;
		}
	}

	return true;
}

bool AFlightPathfinderActor::DoesSegmentRespectAltitudeLimit(const FVector& FromWorld, const FVector& ToWorld) const
{
	if (!FlightProfile)
	{
		return false;
	}

	if (FlightProfile->MaxAltitudeMetersASL <= 0.0f)
	{
		return true;
	}

	const int32 NumSteps = 8;
	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);
		if (!DoesPointRespectAltitudeLimit(Sample))
		{
			return false;
		}
	}

	return true;
}

bool AFlightPathfinderActor::DoesPointRespectTerrainClearance(const FVector& WorldPoint) const
{
	if (!HeightCache || !FlightProfile)
	{
		return false;
	}

	float TerrainHeightCm = 0.0f;
	if (!GetConservativeTerrainHeightCmAtWorldXY(
		WorldPoint.X,
		WorldPoint.Y,
		GetTerrainSafetyRadiusMeters(),
		TerrainHeightCm))
	{
		return false;
	}

	const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);
	const float TerrainMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

	return (AltitudeMetersASL - TerrainMetersASL) >= GetRequiredTerrainClearanceMeters();
}

bool AFlightPathfinderActor::DoesPointRespectPreferredTerrainClearance(const FVector& WorldPoint) const
{
	if (!HeightCache || !FlightProfile)
	{
		return false;
	}

	float TerrainHeightCm = 0.0f;
	if (!GetConservativeTerrainHeightCmAtWorldXY(
		WorldPoint.X,
		WorldPoint.Y,
		GetPreferredTerrainSafetyRadiusMeters(),
		TerrainHeightCm))
	{
		return false;
	}

	const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);
	const float TerrainMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

	return (AltitudeMetersASL - TerrainMetersASL) >= GetPreferredTerrainClearanceMeters();
}

bool AFlightPathfinderActor::DoesSegmentRespectPreferredTerrainClearance(
	const FVector& FromWorld,
	const FVector& ToWorld
) const
{
	if (!HeightCache || !FlightProfile)
	{
		return false;
	}

	const float DistanceCm = FVector::Distance(FromWorld, ToWorld);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return DoesPointRespectPreferredTerrainClearance(FromWorld);
	}

	const float SampleStepCm = FMath::Max(100.0f, HeightCache->CellSizeCm * 0.5f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (!DoesPointRespectAltitudeLimit(Sample) || !DoesPointRespectPreferredTerrainClearance(Sample))
		{
			return false;
		}
	}

	return true;
}

bool AFlightPathfinderActor::DoesDirectSegmentRespectFlightRules(
	const FVector& FromWorld,
	const FVector& ToWorld,
	int32 FromHeadingIndex,
	int32 ToHeadingIndex
) const
{
	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		LastFailureReason = ERouteFailureReason::InvalidFlightProfile;
		return false;
	}

	if (!DoesSegmentRespectAltitudeLimit(FromWorld, ToWorld))
	{
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		return false;
	}

	if (!DoesSegmentRespectTerrainClearance(FromWorld, ToWorld))
	{
		FailureStats.TerrainClearanceCount++;
		LastFailureReason = ERouteFailureReason::TerrainClearance;
		return false;
	}

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	if (DoesSegmentIntersectHardBlockZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
		FailureStats.HardBlockZoneCount++;
		LastFailureReason = ERouteFailureReason::HardBlockZone;
		return false;
	}

	const float HorizontalDistanceMeters = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	) / 100.0f;

	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		HorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;
	if (DeltaZMeters > 0.0f)
	{
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
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(FromHeadingIndex, ToHeadingIndex);
	if (HeadingDeltaBuckets > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float HeadingDeltaRad = static_cast<float>(HeadingDeltaBuckets) * AnglePerBucketRad;
		if (HeadingDeltaRad > KINDA_SMALL_NUMBER && HorizontalDistanceMeters > KINDA_SMALL_NUMBER)
		{
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

bool AFlightPathfinderActor::CanConnectToGoal(
	const FFlightPathState& Current,
	const FFlightPathState& Goal,
	const FVector& GoalWorldLocation,
	int32 GoalHeadingIndex
) const
{
	const FVector CurrentWorld = StateToWorldCenter(Current);
	const FVector GoalStateWorld = StateToWorldCenter(Goal);

	const float GoalXYDistanceMeters = FVector2D::Distance(
		FVector2D(CurrentWorld.X, CurrentWorld.Y),
		FVector2D(GoalWorldLocation.X, GoalWorldLocation.Y)
	) / 100.0f;

	const float GoalStateDistanceMeters = FVector::Distance(CurrentWorld, GoalStateWorld) / 100.0f;
	const float AllowedConnectionMeters = FMath::Max(
		GoalConnectionToleranceMeters,
		GetEffectivePrimitiveSegmentLengthMeters() * 1.5f
	);

	if (GoalXYDistanceMeters > AllowedConnectionMeters && GoalStateDistanceMeters > AllowedConnectionMeters * 1.75f)
	{
		return false;
	}

	const FVector2D SegmentDirection(
		GoalWorldLocation.X - CurrentWorld.X,
		GoalWorldLocation.Y - CurrentWorld.Y
	);
	const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);
	(void)GoalHeadingIndex;

	return DoesDirectSegmentRespectFlightRules(CurrentWorld, GoalWorldLocation, SegmentHeading, SegmentHeading);
}

bool AFlightPathfinderActor::TryBuildDirectVfrRoute(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	int32 StartHeadingIndex,
	TArray<FVector>& OutRoutePoints
)
{
	OutRoutePoints.Reset();
	(void)StartHeadingIndex;

	if (!FlightProfile || !HeightCache)
	{
		return false;
	}

	const FVector2D StartXY(StartWorldLocation.X, StartWorldLocation.Y);
	const FVector2D TargetXY(TargetWorldLocation.X, TargetWorldLocation.Y);
	const float HorizontalDistanceMeters = FVector2D::Distance(StartXY, TargetXY) / 100.0f;

	if (DirectRoutePreferredClearanceMaxLengthMeters > 0.0f &&
		HorizontalDistanceMeters > DirectRoutePreferredClearanceMaxLengthMeters)
	{
		return false;
	}

	TArray<FVector> StraightRoute;
	StraightRoute.Add(StartWorldLocation);
	StraightRoute.Add(TargetWorldLocation);

	CurrentRouteWorldPoints = StraightRoute;
	if (!ValidateCurrentRouteSafety())
	{
		CurrentRouteWorldPoints.Reset();
		return false;
	}

	OutRoutePoints = StraightRoute;
	CurrentRouteWorldPoints = StraightRoute;
	return true;
}

bool AFlightPathfinderActor::DoesRouteRespectTurnRadius(const TArray<FVector>& RoutePoints) const
{
	if (!FlightProfile || FlightProfile->MinimumTurnRadiusMeters <= 0.0f || RoutePoints.Num() < 3)
	{
		return true;
	}

	const float RequiredRadiusMeters = FlightProfile->MinimumTurnRadiusMeters;
	const float MinTurnAngleRad = FMath::DegreesToRadians(5.0f);

	for (int32 i = 1; i < RoutePoints.Num() - 1; ++i)
	{
		const FVector& A = RoutePoints[i - 1];
		const FVector& B = RoutePoints[i];
		const FVector& C = RoutePoints[i + 1];

		const FVector2D InVector(B.X - A.X, B.Y - A.Y);
		const FVector2D OutVector(C.X - B.X, C.Y - B.Y);
		const float InLengthMeters = InVector.Size() / 100.0f;
		const float OutLengthMeters = OutVector.Size() / 100.0f;

		if (InLengthMeters <= KINDA_SMALL_NUMBER || OutLengthMeters <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const FVector2D InDir = InVector / (InLengthMeters * 100.0f);
		const FVector2D OutDir = OutVector / (OutLengthMeters * 100.0f);
		const float Dot = FMath::Clamp(FVector2D::DotProduct(InDir, OutDir), -1.0f, 1.0f);
		const float TurnAngleRad = FMath::Acos(Dot);

		if (TurnAngleRad < MinTurnAngleRad)
		{
			continue;
		}

		const float Cross = InDir.X * OutDir.Y - InDir.Y * OutDir.X;
		if (FMath::Abs(Cross) < 0.001f)
		{
			FailureStats.TurnRadiusTooSmallCount++;
			LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
			return false;
		}

		const float RequiredTangentMeters = RequiredRadiusMeters * FMath::Tan(TurnAngleRad * 0.5f);
		const float AvailableTangentMeters = FMath::Min(InLengthMeters, OutLengthMeters) * 0.75f;

		if (RequiredTangentMeters > AvailableTangentMeters)
		{
			FailureStats.TurnRadiusTooSmallCount++;
			LastFailureReason = ERouteFailureReason::TurnRadiusTooSmall;
			return false;
		}
	}

	return true;
}


bool AFlightPathfinderActor::ValidateCurrentRouteSafety() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		return false;
	}

	for (const FVector& Point : CurrentRouteWorldPoints)
	{
		if (!DoesPointRespectAltitudeLimit(Point))
		{
			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}

		if (!DoesPointRespectTerrainClearance(Point))
		{
			FailureStats.TerrainClearanceCount++;
			LastFailureReason = ERouteFailureReason::TerrainClearance;
			return false;
		}
	}

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		const FVector& From = CurrentRouteWorldPoints[i];
		const FVector& To = CurrentRouteWorldPoints[i + 1];
		const FVector2D SegmentDirection(To.X - From.X, To.Y - From.Y);
		const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);

		if (!DoesDirectSegmentRespectFlightRules(From, To, SegmentHeading, SegmentHeading))
		{
			return false;
		}
	}

	if (!DoesRouteRespectTurnRadius(CurrentRouteWorldPoints))
	{
		return false;
	}

	return true;
}
// Check if point is inside a hard block influence zone
bool AFlightPathfinderActor::IsStateInsideHardBlockZone(const FVector& WorldPoint, float AltitudeMetersASL) const
{
	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!Zone)
		{
			continue;
		}

		// Hard block zone = forbidden airspace
		if (Zone->bHardBlock && Zone->ContainsPoint(WorldPoint, AltitudeMetersASL))
		{
			return true;
		}
	}

	return false;
}

// Check if segment crosses any hard block influence zone
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
			continue;
		}

		if (Zone->bHardBlock && Zone->IntersectsSegmentBySampling(FromWorld, ToWorld, FromAltitudeMetersASL,
		                                                          ToAltitudeMetersASL))
		{
			return true;
		}
	}

	return false;
}

// Calculate additional cost from soft influence zones
float AFlightPathfinderActor::GetSoftZoneTraversalCost(
	const FVector& FromWorld,
	const FVector& ToWorld,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL
) const
{
	float TotalExtraCost = 0.0f;

	for (const TObjectPtr<AFlightInfluenceZoneActor>& Zone : InfluenceZones)
	{
		if (!Zone)
		{
			continue;
		}

		// Soft zone = allowed, but more expensive
		if (!Zone->bHardBlock && Zone->AdditionalTraversalCost > 0.0f)
		{
			if (Zone->IntersectsSegmentBySampling(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
			{
				TotalExtraCost += Zone->AdditionalTraversalCost;
			}
		}
	}

	return TotalExtraCost;
}

// Check if state is flyable and safe
bool AFlightPathfinderActor::IsStateValid(const FFlightPathState& State) const
{
	if (!HeightCache || !HeightCache->IsValid() || !FlightProfile)
	{
		return false;
	}

	if (!IsStateInsideBounds(State))
	{
		return false;
	}

	const FVector WorldPoint = StateToWorldCenter(State);

	if (!DoesPointRespectAltitudeLimit(WorldPoint))
	{
		return false;
	}

	if (!DoesPointRespectTerrainClearance(WorldPoint))
	{
		return false;
	}

	const float StateAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);

	if (IsStateInsideHardBlockZone(WorldPoint, StateAltitudeMetersASL))
	{
		return false;
	}

	return true;
}

// Check terrain clearance along a segment
bool AFlightPathfinderActor::DoesSegmentRespectTerrainClearance(const FVector& FromWorld, const FVector& ToWorld) const
{
	if (!HeightCache || !FlightProfile)
	{
		return false;
	}

	const float DistanceCm = FVector::Distance(FromWorld, ToWorld);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return DoesPointRespectTerrainClearance(FromWorld);
	}

	const float SampleStepCm = FMath::Max(100.0f, HeightCache->CellSizeCm * 0.25f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		if (!DoesPointRespectTerrainClearance(Sample))
		{
			return false;
		}
	}

	return true;
}

// Check if transition between two states respects flight rules
bool AFlightPathfinderActor::IsTransitionValid(const FFlightPathState& From, const FFlightPathState& To) const
{
	if (!FlightProfile)
	{
		return false;
	}

	if (!IsStateValid(To))
	{
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	// Terrain clearance along segment
	if (!DoesSegmentRespectTerrainClearance(FromWorld, ToWorld))
	{
		FailureStats.TerrainClearanceCount++;
		LastFailureReason = ERouteFailureReason::TerrainClearance;
		return false;
	}

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	// Hard influence zones along segment
	if (DoesSegmentIntersectHardBlockZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
		FailureStats.HardBlockZoneCount++;
		LastFailureReason = ERouteFailureReason::HardBlockZone;
		return false;
	}

	// HorizontalDistanceMeters: basis for flight time estimate
	const float HorizontalDistanceCm = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	);

	const float HorizontalDistanceMeters = HorizontalDistanceCm / 100.0f;

	// TravelTimeSeconds: approximated from cruise speed
	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		HorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	// DeltaZMeters: vertical movement for climb / descent limit
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;

	if (DeltaZMeters > 0.0f)
	{
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
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	// Approximate turn radius from heading change and segment length
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	if (HeadingDeltaBuckets > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float HeadingDeltaRad = static_cast<float>(HeadingDeltaBuckets) * AnglePerBucketRad;

		if (HeadingDeltaRad > KINDA_SMALL_NUMBER && HorizontalDistanceMeters > KINDA_SMALL_NUMBER)
		{
			// ImpliedTurnRadius: R = s / theta
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
		return false;
	}

	const FVector EndWorld = SamplePoints.Last();
	
	if (!WorldToStateExact(EndWorld, EndHeading, OutState))
	{
		return false;
	}

	return true;
}

bool AFlightPathfinderActor::IsMotionPrimitiveValid(
	const FFlightPathState& FromState,
	const FFlightPathState& ToState,
	int32 HeadingDeltaBuckets,
	int32 VerticalMode
) const
{
	if (!FlightProfile)
	{
		LastFailureReason = ERouteFailureReason::InvalidFlightProfile;
		return false;
	}

	if (!IsStateValid(ToState))
	{
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	TArray<FVector> SamplePoints;
	int32 EndHeading = 0;

	if (!BuildPrimitiveSamplePoints(FromState, HeadingDeltaBuckets, VerticalMode, SamplePoints, EndHeading))
	{
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	if (SamplePoints.Num() < 2)
	{
		FailureStats.InvalidTargetStateCount++;
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}

	SamplePoints.Last() = StateToWorldCenter(ToState);

	float TotalHorizontalDistanceMeters = 0.0f;
	float TotalPathLengthMeters = 0.0f;

	for (int32 i = 0; i < SamplePoints.Num() - 1; ++i)
	{
		const FVector& A = SamplePoints[i];
		const FVector& B = SamplePoints[i + 1];

		const float AltA = GetAltitudeMetersASLFromWorldZ(A.Z);
		const float AltB = GetAltitudeMetersASLFromWorldZ(B.Z);

		if (!DoesSegmentRespectAltitudeLimit(A, B))
		{
			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}

		if (!DoesSegmentRespectTerrainClearance(A, B))
		{
			FailureStats.TerrainClearanceCount++;
			LastFailureReason = ERouteFailureReason::TerrainClearance;
			return false;
		}

		if (DoesSegmentIntersectHardBlockZone(A, B, AltA, AltB))
		{
			FailureStats.HardBlockZoneCount++;
			LastFailureReason = ERouteFailureReason::HardBlockZone;
			return false;
		}

		TotalHorizontalDistanceMeters += FVector2D::Distance(
			FVector2D(A.X, A.Y),
			FVector2D(B.X, B.Y)
		) / 100.0f;

		TotalPathLengthMeters += FVector::Distance(A, B) / 100.0f;
	}

	const float TravelTimeSeconds = FMath::Max(
		0.001f,
		TotalHorizontalDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond
	);

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(SamplePoints[0].Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(SamplePoints.Last().Z);
	const float DeltaZMeters = ToAltitudeMetersASL - FromAltitudeMetersASL;

	if (DeltaZMeters > 0.0f)
	{
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
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
			FailureStats.MaxDescentExceededCount++;
			LastFailureReason = ERouteFailureReason::MaxDescentExceeded;
			return false;
		}
	}

	const int32 HeadingDeltaAbs = FMath::Abs(HeadingDeltaBuckets);
	if (HeadingDeltaAbs > 0 && FlightProfile->MinimumTurnRadiusMeters > 0.0f)
	{
		const float AnglePerBucketRad = (2.0f * PI) / static_cast<float>(HeadingBucketCount);
		const float DeltaHeadingRad = static_cast<float>(HeadingDeltaAbs) * AnglePerBucketRad;

		if (DeltaHeadingRad > KINDA_SMALL_NUMBER)
		{
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

void AFlightPathfinderActor::GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors) const
{
	OutNeighbors.Reset();

	TSet<FFlightPathState> UniqueNeighbors;

	TArray<int32> HeadingOptions;
	HeadingOptions.Add(0);

	const int32 MaxTurnBuckets = FMath::Clamp(
		FMath::Max(1, FMath::Max(MaxHeadingChangePerStep, PrimitiveTurnDeltaBuckets)),
		1,
		3
	);

	for (int32 Delta = 1; Delta <= MaxTurnBuckets; ++Delta)
	{
		HeadingOptions.Add(-Delta);
		HeadingOptions.Add(Delta);
	}

	TArray<int32> VerticalModes;
	VerticalModes.Add(0);

	if (bAllowClimbStep)
	{
		VerticalModes.Add(1);
	}

	if (bAllowDescentStep)
	{
		VerticalModes.Add(-1);
	}

	for (const int32 HeadingDelta : HeadingOptions)
	{
		for (const int32 VerticalMode : VerticalModes)
		{
			FFlightPathState Candidate;
			if (!ApplyMotionPrimitive(Current, HeadingDelta, VerticalMode, Candidate))
			{
				continue;
			}

			if (!IsStateInsideBounds(Candidate))
			{
				continue;
			}

			if (!IsMotionPrimitiveValid(Current, Candidate, HeadingDelta, VerticalMode))
			{
				continue;
			}

			UniqueNeighbors.Add(Candidate);
		}
	}

	OutNeighbors = UniqueNeighbors.Array();

	if (OutNeighbors.Num() == 0)
	{
		FailureStats.NoValidNeighborsCount++;
		LastFailureReason = ERouteFailureReason::NoValidNeighbors;
	}
}

// Estimate remaining A* cost to goal
float AFlightPathfinderActor::HeuristicCost(const FFlightPathState& A, const FFlightPathState& B) const
{
	const FVector AWorld = StateToWorldCenter(A);
	const FVector BWorld = StateToWorldCenter(B);

	const float StraightLineDistanceMeters = FVector::Distance(AWorld, BWorld) / 100.0f;

	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		return StraightLineDistanceMeters;
	}

	// Base heuristic: direct flight time
	float Cost = StraightLineDistanceMeters / FlightProfile->CruiseSpeedMetersPerSecond;

	// Extra heuristic cost for required climb
	const float DeltaZMeters = FMath::Max(0.0f, (BWorld.Z - AWorld.Z) / 100.0f);
	if (DeltaZMeters > 0.0f)
	{
		Cost += (DeltaZMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 4.0f;
	}

	return Cost;
}

// Calculate transition cost between two states
float AFlightPathfinderActor::TransitionCost(const FFlightPathState& From, const FFlightPathState& To) const
{
	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	const float PathLengthMeters = FVector::Distance(FromWorld, ToWorld) / 100.0f;
	const float HorizontalDistanceMeters = FVector2D::Distance(
		FVector2D(FromWorld.X, FromWorld.Y),
		FVector2D(ToWorld.X, ToWorld.Y)
	) / 100.0f;

	if (!FlightProfile || FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		return PathLengthMeters;
	}

	// Base cost: flight time instead of centimeter distance
	float Cost = PathLengthMeters / FlightProfile->CruiseSpeedMetersPerSecond;

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	// Soft influence zones
	Cost += GetSoftZoneTraversalCost(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL);

	// Climb penalty: weaker aircraft need earlier or wider climb planning
	const float DeltaZMeters = (ToWorld.Z - FromWorld.Z) / 100.0f;
	if (DeltaZMeters > 0.0f)
	{
		const float ClimbPenalty =
			(DeltaZMeters / FMath::Max(0.1f, FlightProfile->MaxClimbRateMetersPerSecond)) * 25.0f;
		Cost += ClimbPenalty;
	}

	// Descent penalty: keep descent changes moderate
	if (DeltaZMeters < 0.0f)
	{
		const float DescentPenalty =
			(FMath::Abs(DeltaZMeters) / FMath::Max(0.1f, FlightProfile->MaxDescentRateMetersPerSecond)) * 8.0f;
		Cost += DescentPenalty;
	}

	// Turn penalty: larger turn radii should influence route choice
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	if (HeadingDeltaBuckets > 0)
	{
		const float TurnPenalty =
			static_cast<float>(HeadingDeltaBuckets * HeadingDeltaBuckets) *
			(FMath::Max(1.0f, FlightProfile->MinimumTurnRadiusMeters) / FMath::Max(50.0f, GetEffectivePrimitiveSegmentLengthMeters())) * 25.0f;

		Cost += TurnPenalty;
	}

	// Terrain proximity penalty: prefer safer clearance where possible
	// Avoid routes that are only barely legal when safer options exist
	float LowestClearanceMeters = TNumericLimits<float>::Max();
	const int32 NumSamples = 12;

	for (int32 Step = 0; Step <= NumSamples; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSamples);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		float TerrainHeightCm = 0.0f;
		if (GetConservativeTerrainHeightCmAtWorldXY(Sample.X, Sample.Y, GetPreferredTerrainSafetyRadiusMeters(), TerrainHeightCm))
		{
			const float SampleAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(Sample.Z);
			const float TerrainAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);
			const float ClearanceMeters = SampleAltitudeMetersASL - TerrainAltitudeMetersASL;

			LowestClearanceMeters = FMath::Min(LowestClearanceMeters, ClearanceMeters);
		}
	}

	if (LowestClearanceMeters < TNumericLimits<float>::Max())
	{
		const float PreferredClearanceMeters = GetPreferredTerrainClearanceMeters();

		if (LowestClearanceMeters < PreferredClearanceMeters)
		{
			const float ClearanceDeficit = PreferredClearanceMeters - LowestClearanceMeters;
			Cost += ClearanceDeficit * 6.0f;
		}
	}

	const float MaxAllowedWorldZCm = GetMaxAllowedWorldZCm();
	if (MaxAllowedWorldZCm < FLT_MAX * 0.5f)
	{
		const float CeilingMarginMeters = (MaxAllowedWorldZCm - ToWorld.Z) / 100.0f;
		const float PreferredCeilingMarginMeters = FMath::Max(GetRequiredTerrainClearanceMeters(), VoxelSizeZMeters);
		if (CeilingMarginMeters < PreferredCeilingMarginMeters)
		{
			Cost += (PreferredCeilingMarginMeters - CeilingMarginMeters) * 40.0f;
		}
	}

	// Small extra penalty for very short horizontal segments to reduce jitter
	if (HorizontalDistanceMeters < GetEffectivePrimitiveSegmentLengthMeters() * 0.5f)
	{
		Cost += 10.0f;
	}

	return Cost;
}

bool AFlightPathfinderActor::IsTransitionValidCached(const FFlightPathState& From, const FFlightPathState& To) const
{
	const FFlightTransitionKey Key(From, To);

	if (const bool* CachedValue = TransitionValidityCache.Find(Key))
	{
		return *CachedValue;
	}

	const bool bValid = IsTransitionValid(From, To);
	TransitionValidityCache.Add(Key, bValid);
	return bValid;
}

float AFlightPathfinderActor::TransitionCostCached(const FFlightPathState& From, const FFlightPathState& To) const
{
	const FFlightTransitionKey Key(From, To);

	if (const float* CachedValue = TransitionCostCache.Find(Key))
	{
		return *CachedValue;
	}

	const float Cost = TransitionCost(From, To);
	TransitionCostCache.Add(Key, Cost);
	return Cost;
}

float AFlightPathfinderActor::CalculateTurnReversalPenalty(
	const FFlightPathState& GrandParent,
	const FFlightPathState& Parent,
	const FFlightPathState& Current
) const
{
	const int32 PrevHeading = NormalizeHeadingIndex(Parent.HeadingIndex - GrandParent.HeadingIndex);
	const int32 NextHeading = NormalizeHeadingIndex(Current.HeadingIndex - Parent.HeadingIndex);

	int32 SignedPrevTurn = PrevHeading;
	int32 SignedNextTurn = NextHeading;

	if (SignedPrevTurn > HeadingBucketCount / 2)
	{
		SignedPrevTurn -= HeadingBucketCount;
	}

	if (SignedNextTurn > HeadingBucketCount / 2)
	{
		SignedNextTurn -= HeadingBucketCount;
	}

	// No real direction change or one segment is straight
	if (SignedPrevTurn == 0 || SignedNextTurn == 0)
	{
		return 0.0f;
	}

	// Penalize immediate left-right or right-left turn reversals
	const bool bTurnDirectionChanged =
		(SignedPrevTurn > 0 && SignedNextTurn < 0) ||
		(SignedPrevTurn < 0 && SignedNextTurn > 0);

	if (!bTurnDirectionChanged)
	{
		return 0.0f;
	}

	// Reversal penalty strength
	return 2000.0f;
}

bool AFlightPathfinderActor::PopBestOpenStateFromHeap(
	TArray<FFlightOpenEntry>& OpenHeap,
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	FFlightPathState& OutBestState
) const
{
	while (OpenHeap.Num() > 0)
	{
		FFlightOpenEntry BestEntry;
		OpenHeap.HeapPop(BestEntry, FFlightOpenEntryMinHeapPredicate());

		const FFlightPathNodeRecord* Record = Records.Find(BestEntry.State);
		if (!Record)
		{
			continue;
		}

		// Ignore stale heap entries
		if (Record->bClosed)
		{
			continue;
		}

		// Only the latest F score is valid
		if (!FMath::IsNearlyEqual(Record->F, BestEntry.FScore, KINDA_SMALL_NUMBER))
		{
			continue;
		}

		OutBestState = BestEntry.State;
		return true;
	}

	return false;
}

// Rebuild route from A* parent records
void AFlightPathfinderActor::ReconstructRoute(
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	const FFlightPathState& GoalState
)
{
	CurrentRouteWorldPoints.Reset();

	TArray<FFlightPathState> StatePath;
	FFlightPathState Current = GoalState;

	while (true)
	{
		StatePath.Insert(Current, 0);

		const FFlightPathNodeRecord* Record = Records.Find(Current);
		if (!Record || !Record->bHasParent)
		{
			break;
		}

		Current = Record->Parent;
	}

	if (StatePath.Num() == 0)
	{
		return;
	}

	// Output only A* support states. Primitive samples stay internal for safety checks.
	CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[0]));
	for (int32 i = 1; i < StatePath.Num(); ++i)
	{
		CurrentRouteWorldPoints.Add(StateToWorldCenter(StatePath[i]));
	}
}

void AFlightPathfinderActor::CompactCurrentRouteWaypoints()
{
	if (CurrentRouteWorldPoints.Num() < 3)
	{
		return;
	}

	TArray<FVector> CompactRoute;
	int32 CurrentIndex = 0;
	CompactRoute.Add(CurrentRouteWorldPoints[0]);

	while (CurrentIndex < CurrentRouteWorldPoints.Num() - 1)
	{
		const int32 LastIndex = CurrentRouteWorldPoints.Num() - 1;
		const int32 MaxTestIndex = FMath::Min(
			LastIndex,
			CurrentIndex + FMath::Max(2, MaxOutputWaypointCompactionLookahead)
		);

		int32 BestNextIndex = CurrentIndex + 1;
		for (int32 TestIndex = MaxTestIndex; TestIndex > CurrentIndex + 1; --TestIndex)
		{
			const float DistanceMeters = FVector::Distance(
				CurrentRouteWorldPoints[CurrentIndex],
				CurrentRouteWorldPoints[TestIndex]
			) / 100.0f;

			if (TestIndex != LastIndex && DistanceMeters < MinOutputWaypointSpacingMeters)
			{
				continue;
			}

			const FVector2D SegmentDirection(
				CurrentRouteWorldPoints[TestIndex].X - CurrentRouteWorldPoints[CurrentIndex].X,
				CurrentRouteWorldPoints[TestIndex].Y - CurrentRouteWorldPoints[CurrentIndex].Y
			);
			const int32 SegmentHeading = ComputeNearestHeadingIndexFromDirection(SegmentDirection);

			const float MaxCompactSegmentMeters = FMath::Max(
				MinOutputWaypointSpacingMeters * 4.0f,
				FMath::Max(GetEffectivePrimitiveSegmentLengthMeters() * 4.0f, FlightProfile ? FlightProfile->MinimumTurnRadiusMeters * 4.0f : 0.0f)
			);

			if (TestIndex != LastIndex && DistanceMeters > MaxCompactSegmentMeters)
			{
				continue;
			}

			bool bTurnAtCurrentWaypointIsFlyable = true;
			if (CompactRoute.Num() >= 2)
			{
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
				BestNextIndex = TestIndex;
				break;
			}
		}

		CompactRoute.Add(CurrentRouteWorldPoints[BestNextIndex]);
		CurrentIndex = BestNextIndex;
	}

	CurrentRouteWorldPoints = CompactRoute;
}

// Calculate full route length in meters
float AFlightPathfinderActor::CalculateRouteLengthMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		return 0.0f;
	}

	float TotalLengthMeters = 0.0f;

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		const float SegmentLengthCm = FVector::Distance(
			CurrentRouteWorldPoints[i],
			CurrentRouteWorldPoints[i + 1]
		);

		TotalLengthMeters += SegmentLengthCm / 100.0f;
	}

	return TotalLengthMeters;
}

// Calculate altitude difference from route start to end
float AFlightPathfinderActor::CalculateNetAltitudeChangeMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		return 0.0f;
	}

	const float StartAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[0].Z);
	const float EndAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints.Last().Z);

	return EndAltitudeMetersASL - StartAltitudeMetersASL;
}

// Calculate total positive climb over route
float AFlightPathfinderActor::CalculateTotalClimbMeters() const
{
	if (CurrentRouteWorldPoints.Num() < 2)
	{
		return 0.0f;
	}

	float TotalClimbMeters = 0.0f;

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
		const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[i].Z);
		const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(CurrentRouteWorldPoints[i + 1].Z);

		const float DeltaMeters = ToAltitudeMetersASL - FromAltitudeMetersASL;

		if (DeltaMeters > 0.0f)
		{
			TotalClimbMeters += DeltaMeters;
		}
	}

	return TotalClimbMeters;
}

// Run aircraft-aware A* route search
bool AFlightPathfinderActor::RunFlightRouteSearch(
	const FVector& StartWorldLocation,
	const FVector& TargetWorldLocation,
	UFlightProfile* InFlightProfile,
	TArray<FVector>& OutRoutePoints
)
{
	OutRoutePoints.Reset();

	CurrentRouteWorldPoints.Reset();
	TransitionValidityCache.Reset();
	TransitionCostCache.Reset();
	ConservativeTerrainHeightCache.Reset();
	FailureStats.Reset();

	if (GetWorld())
	{
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	LastFailureReason = ERouteFailureReason::None;
	LastExpandedStates = 0;

	// Use profile selected by UI or caller.
	FlightProfile = InFlightProfile;

	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!ValidateCoreReferences())
	{
		LastFailureReason = ERouteFailureReason::Unknown;
		return false;
	}

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

	const FVector StartPos = StartWorldLocation;
	const FVector GoalPos = TargetWorldLocation;

	// Initial headings based on the planned travel direction.
	const FVector2D StartToGoalXY(GoalPos.X - StartPos.X, GoalPos.Y - StartPos.Y);

	const int32 StartHeading = ComputeNearestHeadingIndexFromDirection(StartToGoalXY);
	const int32 GoalHeading = StartHeading;

	const float StartAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(StartPos.Z);
	const float GoalAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(GoalPos.Z);

	if (FlightProfile->MaxAltitudeMetersASL > 0.0f)
	{
		if (StartAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FindFlightRoute abgebrochen: Starthoehe %.2f m ASL liegt ueber der maximalen Flughoehe %.2f m ASL."),
				StartAltitudeMetersASL,
				FlightProfile->MaxAltitudeMetersASL
			);
			LastFailureReason = ERouteFailureReason::InvalidStart;
			return false;
		}

		if (GoalAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FindFlightRoute abgebrochen: Zielhoehe %.2f m ASL liegt ueber der maximalen Flughoehe %.2f m ASL."),
				GoalAltitudeMetersASL,
				FlightProfile->MaxAltitudeMetersASL
			);
			LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
			return false;
		}
	}

	if (!DoesPointRespectTerrainClearance(StartPos))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Startpunkt unterschreitet den Sicherheitsabstand zum Terrain."));
		LastFailureReason = ERouteFailureReason::StartAltitudeTooLow;
		return false;
	}

	if (!DoesPointRespectTerrainClearance(GoalPos))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Zielpunkt unterschreitet den Sicherheitsabstand zum Terrain."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		return false;
	}

	TArray<FVector> DirectRoute;
	if (TryBuildDirectVfrRoute(StartPos, GoalPos, StartHeading, DirectRoute))
	{
		CurrentRouteWorldPoints = DirectRoute;
		OutRoutePoints = DirectRoute;
		LastExpandedStates = 0;

		UE_LOG(LogTemp, Display, TEXT("FindFlightRoute: Direkte VFR-Route verwendet. Wegpunkte=%d"), CurrentRouteWorldPoints.Num());

		if (bAutoDrawPathAfterSearch)
		{
			DebugDrawCurrentRoute();
		}

		return true;
	}

	LastFailureReason = ERouteFailureReason::None;
	FailureStats.Reset();

	BuildSearchSpaceForRoute(StartWorldLocation, TargetWorldLocation);

	if (ZLayerCount <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("RunFlightRouteSearch abgebrochen: SearchSpace konnte nicht aufgebaut werden."));
		LastFailureReason = ERouteFailureReason::Unknown;
		return false;
	}
	FFlightPathState StartState;
	FFlightPathState GoalState;

	if (!WorldToNearestValidState(StartPos, StartHeading, StartState))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Kein gueltiger Startzustand gefunden."));
		LastFailureReason = ERouteFailureReason::InvalidStart;
		return false;
	}

	if (!DoesPointRespectAltitudeLimit(GoalPos))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Zielhoehe liegt ueber der maximalen Flughoehe."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		return false;
	}

	if (!DoesPointRespectTerrainClearance(GoalPos))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Zielpunkt unterschreitet den Sicherheitsabstand zum Terrain."));
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		return false;
	}

	if (!WorldToNearestValidState(GoalPos, GoalHeading, GoalState))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Kein gueltiger Zielzustand gefunden."));
		LastFailureReason = ERouteFailureReason::InvalidTargetState;
		return false;
	}
	// OpenHeap: priority queue for open states
	TArray<FFlightOpenEntry> OpenHeap;

	// Records: cost, parent and closed state for all touched states
	TMap<FFlightPathState, FFlightPathNodeRecord> Records;

	// StartRecord: first A* state
	FFlightPathNodeRecord StartRecord;
	StartRecord.G = 0.0f;
	StartRecord.H = HeuristicCost(StartState, GoalState);
	StartRecord.F = StartRecord.G + HeuristicWeight * StartRecord.H;
	StartRecord.bHasParent = false;
	StartRecord.bClosed = false;

	Records.Add(StartState, StartRecord);
	OpenHeap.HeapPush(
		FFlightOpenEntry(StartState, StartRecord.F),
		FFlightOpenEntryMinHeapPredicate()
	);

	int32 ExpandedCount = 0;
	const int32 MaxExpandedStates = FMath::Max(10000, SearchMaxExpandedStates);
	bool bFoundRoute = false;

	// ReachedGoalState: actual found goal state
	FFlightPathState ReachedGoalState;

	while (OpenHeap.Num() > 0 && ExpandedCount < MaxExpandedStates)
	{
		++ExpandedCount;

		FFlightPathState Current;
		if (!PopBestOpenStateFromHeap(OpenHeap, Records, Current))
		{
			break;
		}

		FFlightPathNodeRecord* CurrentRecord = Records.Find(Current);
		if (!CurrentRecord)
		{
			continue;
		}

		if (CurrentRecord->bClosed)
		{
			continue;
		}

		// Mark state as fully checked
		CurrentRecord->bClosed = true;

		const float CurrentG = CurrentRecord->G;

		// Optional debug for visited states
		if (bDrawVisitedStates && GetWorld())
		{
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
			bFoundRoute = true;
			ReachedGoalState = Current;
			break;
		}
		TArray<FFlightPathState> Neighbors;
		GetNeighbors(Current, Neighbors);

		for (const FFlightPathState& Neighbor : Neighbors)
		{
			FFlightPathNodeRecord* ExistingRecord = Records.Find(Neighbor);
			if (ExistingRecord && ExistingRecord->bClosed)
			{
				continue;
			}

			float TentativeG = CurrentG + TransitionCostCached(Current, Neighbor);

			bool bShouldUpdate = false;

			// First visit of this neighbor
			if (!ExistingRecord)
			{
				FFlightPathNodeRecord NewRecord;
				Records.Add(Neighbor, NewRecord);
				ExistingRecord = Records.Find(Neighbor);
				bShouldUpdate = true;
			}
			else if (TentativeG < ExistingRecord->G)
			{
				bShouldUpdate = true;
			}

			if (!bShouldUpdate || !ExistingRecord)
			{
				continue;
			}

			// Store better route to neighbor
			ExistingRecord->G = TentativeG;
			ExistingRecord->H = HeuristicCost(Neighbor, GoalState);
			ExistingRecord->F = ExistingRecord->G + HeuristicWeight * ExistingRecord->H;
			ExistingRecord->Parent = Current;
			ExistingRecord->bHasParent = true;
			ExistingRecord->bClosed = false;

			OpenHeap.HeapPush(
				FFlightOpenEntry(Neighbor, ExistingRecord->F),
				FFlightOpenEntryMinHeapPredicate()
			);
		}
	}

	LastExpandedStates = ExpandedCount;

	if (!bFoundRoute)
	{
		if (ExpandedCount >= MaxExpandedStates)
		{
			LastFailureReason = ERouteFailureReason::MaxExpandedStatesReached;
		}

		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: Keine Route gefunden. ExpandedStates=%d"), ExpandedCount);

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

	ReconstructRoute(Records, ReachedGoalState);

	if (CurrentRouteWorldPoints.Num() == 0 ||
		FVector::DistSquared(CurrentRouteWorldPoints.Last(), GoalPos) > 1.0f)
	{
		CurrentRouteWorldPoints.Add(GoalPos);
	}

	CompactCurrentRouteWaypoints();

	if (!DoesRouteRespectAltitudeLimit(CurrentRouteWorldPoints))
	{
		LastFailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		CurrentRouteWorldPoints.Reset();
		OutRoutePoints.Reset();
		return false;
	}

	if (!ValidateCurrentRouteSafety())
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: Gefundene Route wurde bei der finalen Sicherheitspruefung verworfen."));
		CurrentRouteWorldPoints.Reset();
		OutRoutePoints.Reset();
		return false;
	}

	const float RouteLengthMeters = CalculateRouteLengthMeters();
	const float NetAltitudeChangeMeters = CalculateNetAltitudeChangeMeters();
	const float TotalClimbMeters = CalculateTotalClimbMeters();

	UE_LOG(
		LogTemp,
		Display,
		TEXT(
			"FindFlightRoute erfolgreich. Wegpunkte=%d, ExpandedStates=%d, RouteLength=%.2f m, NetAltitudeChange=%.2f m, TotalClimb=%.2f m"
		),
		CurrentRouteWorldPoints.Num(),
		ExpandedCount,
		RouteLengthMeters,
		NetAltitudeChangeMeters,
		TotalClimbMeters
	);

	if (bAutoDrawPathAfterSearch)
	{
		DebugDrawCurrentRoute();
	}

	OutRoutePoints = CurrentRouteWorldPoints;
	return true;
}

FRouteCalculationResult AFlightPathfinderActor::CalculateFlightRouteForUI(
	FVector StartWorldLocation,
	FVector TargetWorldLocation,
	float StartAltitudeMetersASL,
	float TargetAltitudeMetersASL,
	UFlightProfile* InFlightProfile
)
{
	FRouteCalculationResult Result;

	CurrentRouteWorldPoints.Reset();
	TransitionValidityCache.Reset();
	TransitionCostCache.Reset();
	ConservativeTerrainHeightCache.Reset();
	FailureStats.Reset();
	LastFailureReason = ERouteFailureReason::None;
	LastExpandedStates = 0;
	FlightProfile = InFlightProfile;

	if (GetWorld())
	{
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	if (!InFlightProfile)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidFlightProfile;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (StartAltitudeMetersASL <= 0.0f)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::StartAltitudeTooLow;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (TargetAltitudeMetersASL <= 0.0f)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::TargetAltitudeTooLow;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	if (InFlightProfile->MaxAltitudeMetersASL > 0.0f &&
		StartAltitudeMetersASL > InFlightProfile->MaxAltitudeMetersASL)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidStart;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = FText::Format(FText::FromString(TEXT("Die Starthoehe ({0}m ASL) liegt ueber der maximal erlaubten Flughoehe dieses Flugzeugs ({1}m ASL).")), FText::AsNumber(StartAltitudeMetersASL), FText::AsNumber(InFlightProfile->MaxAltitudeMetersASL));
		return Result;
	}

	if (InFlightProfile->MaxAltitudeMetersASL > 0.0f &&
		TargetAltitudeMetersASL > InFlightProfile->MaxAltitudeMetersASL)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::TargetAltitudeTooHigh;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	// IMPORTANT:
	// UI values are meters above sea level.
	// Unreal world Z is centimeters.
	if (!HeightCache)
	{
		if (GridBaker && GridBaker->HeightCache)
		{
			HeightCache = GridBaker->HeightCache;
		}
	}

	if (!HeightCache)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::Unknown;
		LastFailureReason = Result.FailureReason;
		Result.FailureText = FText::FromString(TEXT("HeightCache fehlt."));
		return Result;
	}

	StartWorldLocation.Z = HeightCache->SeaLevelWorldZCm + (StartAltitudeMetersASL * 100.0f);
	TargetWorldLocation.Z = HeightCache->SeaLevelWorldZCm + (TargetAltitudeMetersASL * 100.0f);

	TArray<FVector> FoundRoute;

	const bool bSuccess = RunFlightRouteSearch(
		StartWorldLocation,
		TargetWorldLocation,
		InFlightProfile,
		FoundRoute
	);

	Result.bSuccess = bSuccess;
	Result.RoutePoints = FoundRoute;
	Result.ExpandedStates = LastExpandedStates;

	if (!bSuccess)
	{
		Result.RoutePoints.Empty();

		Result.FailureReason = LastFailureReason;

		if (Result.FailureReason == ERouteFailureReason::None)
		{
			Result.FailureReason = ERouteFailureReason::Unknown;
		LastFailureReason = Result.FailureReason;
		}

		Result.FailureText = GetFailureReasonText(Result.FailureReason);
		return Result;
	}

	Result.FailureReason = ERouteFailureReason::None;
	Result.FailureText = FText::GetEmpty();

	return Result;
}

void AFlightPathfinderActor::FindFlightRoute()
{
	if (!StartActor || !GoalActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: StartActor oder GoalActor fehlt."));
		LastFailureReason = ERouteFailureReason::InvalidStart;
		return;
	}

	TArray<FVector> FoundRoute;

	const bool bSuccess = RunFlightRouteSearch(
		StartActor->GetActorLocation(),
		GoalActor->GetActorLocation(),
		FlightProfile,
		FoundRoute
	);

	if (!bSuccess)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: Route konnte nicht berechnet werden. Grund: %s"),
		       *GetFailureReasonText(LastFailureReason).ToString()
		);
	}
}

// Draw current route with debug lines and spheres
void AFlightPathfinderActor::DebugDrawCurrentRoute()
{
	if (!GetWorld())
	{
		return;
	}

	if (CurrentRouteWorldPoints.Num() < 2)
	{
		UE_LOG(LogTemp, Warning, TEXT("DebugDrawCurrentRoute: Keine gueltige Route vorhanden."));
		return;
	}

	for (int32 i = 0; i < CurrentRouteWorldPoints.Num() - 1; ++i)
	{
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

	// Draw final route point
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

// Clear route result and debug strings
void AFlightPathfinderActor::ClearCurrentRoute()
{
	CurrentRouteWorldPoints.Reset();

	if (GetWorld())
	{
		FlushPersistentDebugLines(GetWorld());
		FlushDebugStrings(GetWorld());
	}

	UE_LOG(LogTemp, Display, TEXT("Aktuelle Flugroute wurde geleert."));
}
