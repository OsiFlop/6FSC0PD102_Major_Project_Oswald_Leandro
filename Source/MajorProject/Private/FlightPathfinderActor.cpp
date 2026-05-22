// Flight pathfinder actor
// Aircraft-aware A* route search
// Uses terrain height cache, flight profile and influence zones
// Search state = voxel position plus heading direction
// Validates terrain clearance, climb / descent limits, turn radius and blocked zones
// Outputs route points and debug drawing

#include "FlightPathfinderActor.h"

#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "FlightProfile.h"
#include "FlightInfluenceZoneActor.h"

#include "DrawDebugHelpers.h"

AFlightPathfinderActor::AFlightPathfinderActor()
{
	PrimaryActorTick.bCanEverTick = false;
}

// Check required references and basic profile values
bool AFlightPathfinderActor::ValidateReferences() const
{
	if (!GridBaker)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: GridBaker fehlt."));
		return false;
	}

	if (!HeightCache)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache fehlt."));
		return false;
	}

	if (!HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeightCache ist ungueltig."));
		return false;
	}

	if (!FlightProfile)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: FlightProfile fehlt."));
		return false;
	}

	if (!StartActor || !GoalActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: StartActor oder GoalActor fehlt."));
		return false;
	}

	// CruiseSpeedMetersPerSecond: required for climb / descent timing
	if (FlightProfile->CruiseSpeedMetersPerSecond <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: CruiseSpeedMetersPerSecond muss > 0 sein."));
		return false;
	}

	// HeadingBucketCount: minimum needed for usable direction steps
	if (HeadingBucketCount < 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlightPathfinder: HeadingBucketCount muss >= 4 sein."));
		return false;
	}

	return true;
}

// Build 3D search space from terrain height range
void AFlightPathfinderActor::BuildSearchSpace()
{
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

	// ZLayerCount: amount of vertical search layers
	const float HeightRangeZ = SearchMaxWorld.Z - SearchMinWorld.Z;
	ZLayerCount = FMath::Max(1, FMath::CeilToInt(HeightRangeZ / VoxelSizeZCm));

	UE_LOG(LogTemp, Display, TEXT("Flight search space gebaut: XY=%d x %d, ZLayers=%d, HeadingBuckets=%d"),
		HeightCache->GridSize.X,
		HeightCache->GridSize.Y,
		ZLayerCount,
		HeadingBucketCount);
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
bool AFlightPathfinderActor::WorldToNearestValidState(const FVector& WorldPos, int32 PreferredHeadingIndex, FFlightPathState& OutState) const
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

		if (Zone->bHardBlock && Zone->IntersectsSegmentBySampling(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
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

	const float TerrainHeightCm = GetTerrainHeightCmAtCell(State.X, State.Y);
	if (TerrainHeightCm <= -1e20f)
	{
		return false;
	}

	// Not inside terrain
	if (WorldPoint.Z <= TerrainHeightCm)
	{
		return false;
	}

	const float StateAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(WorldPoint.Z);
	const float TerrainAltitudeMetersASL = GetTerrainHeightMetersASLAtCell(State.X, State.Y);

	if (TerrainAltitudeMetersASL <= -FLT_MAX / 2.0f)
	{
		return false;
	}

	// ClearanceMeters: vertical safety distance above terrain
	const float ClearanceMeters = StateAltitudeMetersASL - TerrainAltitudeMetersASL;
	if (ClearanceMeters < FlightProfile->MinimumTerrainClearanceMeters)
	{
		return false;
	}

	// Max altitude from aircraft profile
	if (StateAltitudeMetersASL > FlightProfile->MaxAltitudeMetersASL)
	{
		return false;
	}

	// Hard influence zones are forbidden
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
		float TerrainHeightCm = 0.0f;
		if (!GetTerrainHeightCmAtWorldXY(FromWorld.X, FromWorld.Y, TerrainHeightCm))
		{
			return false;
		}

		const float AltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
		const float TerrainMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

		return (AltitudeMetersASL - TerrainMetersASL) >= FlightProfile->MinimumTerrainClearanceMeters;
	}

	// SampleStepCm: small terrain check interval along route segment
	const float SampleStepCm = FMath::Max(100.0f, HeightCache->CellSizeCm * 0.25f);
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / SampleStepCm));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Alpha: normalized position between FromWorld and ToWorld
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector Sample = FMath::Lerp(FromWorld, ToWorld, Alpha);

		float TerrainHeightCm = 0.0f;
		if (!GetTerrainHeightCmAtWorldXY(Sample.X, Sample.Y, TerrainHeightCm))
		{
			return false;
		}

		const float SampleAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(Sample.Z);
		const float TerrainAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(TerrainHeightCm);

		if ((SampleAltitudeMetersASL - TerrainAltitudeMetersASL) < FlightProfile->MinimumTerrainClearanceMeters)
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
		return false;
	}

	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	// Terrain clearance along segment
	if (!DoesSegmentRespectTerrainClearance(FromWorld, ToWorld))
	{
		return false;
	}

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	// Hard influence zones along segment
	if (DoesSegmentIntersectHardBlockZone(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL))
	{
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
			return false;
		}
	}

	if (DeltaZMeters < 0.0f)
	{
		const float MaxAllowedDescentMeters = FlightProfile->MaxDescentRateMetersPerSecond * TravelTimeSeconds;
		if (FMath::Abs(DeltaZMeters) > MaxAllowedDescentMeters)
		{
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
				return false;
			}
		}
	}

	return true;
}

// Generate valid neighbor states for A*
void AFlightPathfinderActor::GetNeighbors(const FFlightPathState& Current, TArray<FFlightPathState>& OutNeighbors) const
{
	OutNeighbors.Reset();

	// UniqueNeighbors: avoids duplicate states from rounded headings
	TSet<FFlightPathState> UniqueNeighbors;

	const int32 MinHeadingDelta = -MaxHeadingChangePerStep;
	const int32 MaxHeadingDelta = MaxHeadingChangePerStep;

	// VerticalOptions: same layer, optional climb, optional descent
	TArray<int32> VerticalOptions;
	VerticalOptions.Add(0);

	if (bAllowClimbStep)
	{
		VerticalOptions.Add(1);
	}

	if (bAllowDescentStep)
	{
		VerticalOptions.Add(-1);
	}

	for (int32 HeadingDelta = MinHeadingDelta; HeadingDelta <= MaxHeadingDelta; ++HeadingDelta)
	{
		const int32 NewHeading = NormalizeHeadingIndex(Current.HeadingIndex + HeadingDelta);
		const FIntPoint StepXY = HeadingIndexToGridStep(NewHeading);

		if (StepXY.X == 0 && StepXY.Y == 0)
		{
			continue;
		}

		for (const int32 DZ : VerticalOptions)
		{
			const FFlightPathState Candidate(
				Current.X + StepXY.X,
				Current.Y + StepXY.Y,
				Current.Z + DZ,
				NewHeading
			);

			if (!IsStateInsideBounds(Candidate))
			{
				continue;
			}

			if (!IsTransitionValid(Current, Candidate))
			{
				continue;
			}

			UniqueNeighbors.Add(Candidate);
		}
	}

	OutNeighbors = UniqueNeighbors.Array();
}

// Estimate remaining A* cost to goal
float AFlightPathfinderActor::HeuristicCost(const FFlightPathState& A, const FFlightPathState& B) const
{
	// Basic admissible heuristic: direct 3D distance
	return FVector::Distance(StateToWorldCenter(A), StateToWorldCenter(B));
}

// Calculate transition cost between two states
float AFlightPathfinderActor::TransitionCost(const FFlightPathState& From, const FFlightPathState& To) const
{
	const FVector FromWorld = StateToWorldCenter(From);
	const FVector ToWorld = StateToWorldCenter(To);

	// BaseDistanceCost: normal movement cost
	const float BaseDistanceCost = FVector::Distance(FromWorld, ToWorld);

	const float FromAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(FromWorld.Z);
	const float ToAltitudeMetersASL = GetAltitudeMetersASLFromWorldZ(ToWorld.Z);

	float Cost = BaseDistanceCost;

	// Soft influence zone penalty
	Cost += GetSoftZoneTraversalCost(FromWorld, ToWorld, FromAltitudeMetersASL, ToAltitudeMetersASL);

	// Small penalty for stronger heading change
	const int32 HeadingDeltaBuckets = GetSmallestHeadingDelta(From.HeadingIndex, To.HeadingIndex);
	Cost += static_cast<float>(HeadingDeltaBuckets) * 25.0f;

	return Cost;
}

// Find open A* state with lowest F cost
bool AFlightPathfinderActor::FindBestOpenState(
	const TSet<FFlightPathState>& OpenSet,
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	FFlightPathState& OutBestState
) const
{
	float BestF = TNumericLimits<float>::Max();
	bool bFound = false;

	// OpenSet: states waiting for A* check
	for (const FFlightPathState& Candidate : OpenSet)
	{
		const FFlightPathNodeRecord* Record = Records.Find(Candidate);
		if (!Record || Record->bClosed)
		{
			continue;
		}

		if (Record->F < BestF)
		{
			BestF = Record->F;
			OutBestState = Candidate;
			bFound = true;
		}
	}

	return bFound;
}

// Rebuild route from A* parent records
void AFlightPathfinderActor::ReconstructRoute(
	const TMap<FFlightPathState, FFlightPathNodeRecord>& Records,
	const FFlightPathState& GoalState
)
{
	CurrentRouteWorldPoints.Reset();

	FFlightPathState Current = GoalState;

	// Walk backwards from reached goal to start
	while (true)
	{
		CurrentRouteWorldPoints.Insert(StateToWorldCenter(Current), 0);

		const FFlightPathNodeRecord* Record = Records.Find(Current);
		if (!Record || !Record->bHasParent)
		{
			break;
		}

		Current = Record->Parent;
	}
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
void AFlightPathfinderActor::FindFlightRoute()
{
	CurrentRouteWorldPoints.Reset();

	// Auto-link height cache from baker
	if (!HeightCache && GridBaker && GridBaker->HeightCache)
	{
		HeightCache = GridBaker->HeightCache;
	}

	if (!ValidateReferences())
	{
		return;
	}

	if (ZLayerCount <= 0)
	{
		BuildSearchSpace();
	}

	if (ZLayerCount <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: SearchSpace konnte nicht aufgebaut werden."));
		return;
	}

	const FVector StartPos = StartActor->GetActorLocation();
	const FVector GoalPos = GoalActor->GetActorLocation();

	// Initial headings based on start-goal direction
	const FVector2D StartToGoalXY(GoalPos.X - StartPos.X, GoalPos.Y - StartPos.Y);
	const FVector2D GoalToStartXY(StartPos.X - GoalPos.X, StartPos.Y - GoalPos.Y);

	const int32 StartHeading = ComputeNearestHeadingIndexFromDirection(StartToGoalXY);
	const int32 GoalHeading = ComputeNearestHeadingIndexFromDirection(GoalToStartXY);

	FFlightPathState StartState;
	FFlightPathState GoalState;

	if (!WorldToNearestValidState(StartPos, StartHeading, StartState))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Kein gueltiger Startzustand gefunden."));
		return;
	}

	if (!WorldToNearestValidState(GoalPos, GoalHeading, GoalState))
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute abgebrochen: Kein gueltiger Zielzustand gefunden."));
		return;
	}

	// OpenSet: states waiting for A* evaluation
	TSet<FFlightPathState> OpenSet;

	// Records: cost, parent and closed state for all touched states
	TMap<FFlightPathState, FFlightPathNodeRecord> Records;

	// StartRecord: first A* state
	FFlightPathNodeRecord StartRecord;
	StartRecord.G = 0.0f;
	StartRecord.H = HeuristicCost(StartState, GoalState);
	StartRecord.F = StartRecord.G + StartRecord.H;
	StartRecord.bHasParent = false;
	StartRecord.bClosed = false;

	Records.Add(StartState, StartRecord);
	OpenSet.Add(StartState);

	int32 ExpandedCount = 0;
	const int32 MaxExpandedStates = 500000;
	bool bFoundRoute = false;

	// ReachedGoalState: actual found goal state
	FFlightPathState ReachedGoalState;

	while (OpenSet.Num() > 0 && ExpandedCount < MaxExpandedStates)
	{
		++ExpandedCount;

		FFlightPathState Current;
		if (!FindBestOpenState(OpenSet, Records, Current))
		{
			break;
		}

		OpenSet.Remove(Current);

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

		if (IsGoalState(Current, GoalState))
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

			// TentativeG: new possible cost from start to neighbor
			const float TentativeG = CurrentG + TransitionCost(Current, Neighbor);

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
			ExistingRecord->F = ExistingRecord->G + ExistingRecord->H;
			ExistingRecord->Parent = Current;
			ExistingRecord->bHasParent = true;
			ExistingRecord->bClosed = false;

			OpenSet.Add(Neighbor);
		}
	}

	if (!bFoundRoute)
	{
		UE_LOG(LogTemp, Warning, TEXT("FindFlightRoute: Keine Route gefunden. ExpandedStates=%d"), ExpandedCount);
		return;
	}

	ReconstructRoute(Records, ReachedGoalState);

	const float RouteLengthMeters = CalculateRouteLengthMeters();
	const float NetAltitudeChangeMeters = CalculateNetAltitudeChangeMeters();
	const float TotalClimbMeters = CalculateTotalClimbMeters();

	UE_LOG(
		LogTemp,
		Display,
		TEXT("FindFlightRoute erfolgreich. Wegpunkte=%d, ExpandedStates=%d, RouteLength=%.2f m, NetAltitudeChange=%.2f m, TotalClimb=%.2f m"),
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
			false,
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
			false,
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
		false,
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
		FlushDebugStrings(GetWorld());
	}

	UE_LOG(LogTemp, Display, TEXT("Aktuelle Flugroute wurde geleert."));
}