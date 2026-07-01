// Flight influence zone actor
// Defines restricted or costly airspace volumes
// Can be used as hard block or soft route penalty
// Supports random placement for shuffled influence zones
// Provides point and segment checks for pathfinding

#include "FlightInfluenceZoneActor.h"

#include "Components/BoxComponent.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"

namespace
{
	// Clip one segment axis against box limits
	bool IntersectInfluenceSegmentAxis(float Start, float Delta, float Min, float Max, float& InOutTMin, float& InOutTMax)
	{
		if (FMath::Abs(Delta) <= KINDA_SMALL_NUMBER)
		{
			// Parallel segment must already be inside this axis range
			return Start >= Min && Start <= Max;
		}

		// Find where the segment enters and exits this axis range
		float T1 = (Min - Start) / Delta;
		float T2 = (Max - Start) / Delta;

		if (T1 > T2)
		{
			// Keep enter value before exit value
			const float Temp = T1;
			T1 = T2;
			T2 = Temp;
		}

		// Narrow the valid intersection interval
		InOutTMin = FMath::Max(InOutTMin, T1);
		InOutTMax = FMath::Min(InOutTMax, T2);

		// Segment still intersects while interval remains valid
		return InOutTMin <= InOutTMax;
	}

	// Fast segment-box intersection test
	bool DoesInfluenceSegmentIntersectBox(const FBox& Box, const FVector& From, const FVector& To)
	{
		// Segment direction used for axis clipping
		const FVector Delta = To - From;

		// Valid segment range from start to end
		float TMin = 0.0f;
		float TMax = 1.0f;

		// Segment must overlap the box on all three axes
		return
			IntersectInfluenceSegmentAxis(From.X, Delta.X, Box.Min.X, Box.Max.X, TMin, TMax) &&
			IntersectInfluenceSegmentAxis(From.Y, Delta.Y, Box.Min.Y, Box.Max.Y, TMin, TMax) &&
			IntersectInfluenceSegmentAxis(From.Z, Delta.Z, Box.Min.Z, Box.Max.Z, TMin, TMax);
	}
}

// Setup influence zone actor
AFlightInfluenceZoneActor::AFlightInfluenceZoneActor()
{
	// Zone is only queried manually, no tick needed
	PrimaryActorTick.bCanEverTick = false;

	// Box defines the influence volume in the level
	ZoneBox = CreateDefaultSubobject<UBoxComponent>(TEXT("ZoneBox"));
	SetRootComponent(ZoneBox);

	// Zone is a visual/query helper, not a physics object
	ZoneBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ZoneBox->SetGenerateOverlapEvents(false);
	ZoneBox->SetCanEverAffectNavigation(false);
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);
}

// Randomize zone position and size
void AFlightInfluenceZoneActor::ShuffleInfluenceZone()
{
	if (!ZoneBox)
	{
		// Cannot shuffle without a volume component
		return;
	}

#if WITH_EDITOR
	// Allow shuffle result to be undoable in editor
	Modify();
	ZoneBox->Modify();
#endif

	// Use landscape bounds or fallback map bounds
	FBox MapBounds;
	GetShuffleWorldBounds(MapBounds);

	// Clamp configured shuffle sizes to safe minimums
	const float MinHorizontalSizeMeters = FMath::Max(100.0f, ShuffleMinHorizontalSizeMeters);
	const float MaxHorizontalSizeMeters = FMath::Max(MinHorizontalSizeMeters, ShuffleMaxHorizontalSizeMeters);
	const float MinVerticalSizeMeters = FMath::Max(100.0f, ShuffleMinVerticalSizeMeters);
	const float MaxVerticalSizeMeters = FMath::Max(MinVerticalSizeMeters, ShuffleMaxVerticalSizeMeters);

	// Pick random horizontal zone size
	const float SizeXMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);
	const float SizeYMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);

	// Box extent uses half size in Unreal centimeters
	FVector NewExtentCm(SizeXMeters * 50.0f, SizeYMeters * 50.0f, 0.0f);

	// Keep shuffled zone away from map border
	const float PaddingCm = FMath::Max(0.0f, ShuffleMapPaddingMeters) * 100.0f;
	const float MinX = MapBounds.Min.X + PaddingCm + NewExtentCm.X;
	const float MaxX = MapBounds.Max.X - PaddingCm - NewExtentCm.X;
	const float MinY = MapBounds.Min.Y + PaddingCm + NewExtentCm.Y;
	const float MaxY = MapBounds.Max.Y - PaddingCm - NewExtentCm.Y;

	// Pick random position inside usable map area
	const float WorldX = (MaxX > MinX) ? FMath::FRandRange(MinX, MaxX) : MapBounds.GetCenter().X;
	const float WorldY = (MaxY > MinY) ? FMath::FRandRange(MinY, MaxY) : MapBounds.GetCenter().Y;

	// Place zone relative to terrain height
	const float TerrainZ = GetShuffleTerrainZAtWorldXY(WorldX, WorldY, MapBounds);

	float WorldZ = TerrainZ;

	if (ShuffleAltitudeMode == EFlightInfluenceShuffleAltitudeMode::CompleteAirspaceBlock)
	{
		// Build a tall zone from terrain up into the airspace
		const float BottomOffsetMeters = FMath::Max(0.0f, ShuffleFullBlockBottomOffsetMeters);
		const float TopHeightMeters = FMath::Max(1000.0f, ShuffleFullBlockTopHeightMeters);
		const float BottomZ = TerrainZ - (BottomOffsetMeters * 100.0f);
		const float TotalHeightCm = (BottomOffsetMeters + TopHeightMeters) * 100.0f;

		// Center the box between bottom and top height
		NewExtentCm.Z = TotalHeightCm * 0.5f;
		WorldZ = BottomZ + NewExtentCm.Z;
	}
	else
	{
		// Build a smaller floating/local influence volume
		const float SizeZMeters = FMath::FRandRange(MinVerticalSizeMeters, MaxVerticalSizeMeters);

		// Place box directly above terrain
		NewExtentCm.Z = SizeZMeters * 50.0f;
		WorldZ = TerrainZ + NewExtentCm.Z;
	}

	// Box height now defines the active airspace
	MinAltitudeMetersASL = 0.0f;
	MaxAltitudeMetersASL = -1.0f;

	// Apply shuffled transform and size
	SetActorScale3D(FVector::OneVector);
	SetActorRotation(FRotator::ZeroRotator);
	SetActorLocation(FVector(WorldX, WorldY, WorldZ));
	ZoneBox->SetBoxExtent(NewExtentCm, true);

	// Keep zone visible after shuffle
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);
	ZoneBox->MarkRenderStateDirty();
}

// Get map bounds used for zone shuffling
bool AFlightInfluenceZoneActor::GetShuffleWorldBounds(FBox& OutBounds) const
{
	// Start with empty bounds
	OutBounds.Init();

	if (const UWorld* World = GetWorld())
	{
		// Use all landscape actors to define map area
		for (TActorIterator<ALandscapeProxy> It(World); It; ++It)
		{
			const ALandscapeProxy* Landscape = *It;
			if (Landscape)
			{
				OutBounds += Landscape->GetComponentsBoundingBox(true);
			}
		}
	}

	if (OutBounds.IsValid)
	{
		// Landscape bounds found
		return true;
	}

	// Fallback bounds are used when no landscape is found
	const FVector CurrentLocation = GetActorLocation();
	OutBounds = FBox(
		FVector(ShuffleFallbackMapMin.X, ShuffleFallbackMapMin.Y, CurrentLocation.Z),
		FVector(ShuffleFallbackMapMax.X, ShuffleFallbackMapMax.Y, CurrentLocation.Z)
	);

	return false;
}

// Find terrain height for shuffled zone placement
float AFlightInfluenceZoneActor::GetShuffleTerrainZAtWorldXY(float WorldX, float WorldY, const FBox& MapBounds) const
{
	if (const UWorld* World = GetWorld())
	{
		// Trace vertically through the map at the chosen XY position
		const FVector TraceStart(WorldX, WorldY, MapBounds.Max.Z + 100000.0f);
		const FVector TraceEnd(WorldX, WorldY, MapBounds.Min.Z - 100000.0f);

		FHitResult Hit;
		FCollisionQueryParams QueryParams(TEXT("ShuffleInfluenceZone"), true, this);

		if (World->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_Visibility, QueryParams))
		{
			// Use hit position as terrain height
			return Hit.ImpactPoint.Z;
		}
	}

	// Fallback keeps zone within known map height
	return MapBounds.IsValid ? MapBounds.Max.Z : GetActorLocation().Z;
}

// Enable or disable this influence zone
void AFlightInfluenceZoneActor::SetInfluenceEnabled(bool bEnabled)
{
	// Store active state for pathfinding/UI
	bInfluenceEnabled = bEnabled;

	// Hide whole actor when disabled
	SetActorHiddenInGame(!bEnabled);

	if (ZoneBox)
	{
		// Keep box visibility in sync with enabled state
		ZoneBox->SetVisibility(bEnabled, true);
		ZoneBox->SetHiddenInGame(!bEnabled);
		ZoneBox->MarkRenderStateDirty();
	}
}

// Get 2D world bounds for map display
void AFlightInfluenceZoneActor::GetInfluenceWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const
{
	// Default output if zone is missing
	OutMinXY = FVector2D::ZeroVector;
	OutMaxXY = FVector2D::ZeroVector;

	if (!ZoneBox)
	{
		// No volume to read from
		return;
	}

	// Read current world-space box bounds
	const FBox WorldBox = ZoneBox->Bounds.GetBox();

	// Return only horizontal map area
	OutMinXY = FVector2D(WorldBox.Min.X, WorldBox.Min.Y);
	OutMaxXY = FVector2D(WorldBox.Max.X, WorldBox.Max.Y);
}

// Get vertical zone range for UI display
void AFlightInfluenceZoneActor::GetInfluenceHeightRangeMetersASL(float& OutBottomMetersASL,
	float& OutTopMetersASL) const
{
	// Default output if zone is missing
	OutBottomMetersASL = 0.0f;
	OutTopMetersASL = 0.0f;

	if (!ZoneBox)
	{
		// No volume to read from
		return;
	}

	// Read current world-space box bounds
	const FBox WorldBox = ZoneBox->Bounds.GetBox();

	// Convert world height to meters above sea level
	OutBottomMetersASL = (WorldBox.Min.Z - SeaLevelWorldZCm) / 100.0f;
	OutTopMetersASL = (WorldBox.Max.Z - SeaLevelWorldZCm) / 100.0f;
}

// Check if a point is inside this zone
bool AFlightInfluenceZoneActor::ContainsPoint(const FVector& WorldPoint, float PointAltitudeMetersASL) const
{
	if (!ZoneBox)
	{
		// Zone has no volume
		return false;
	}

	// Point must be inside the 3D box first
	const bool bInsideBox = ZoneBox->Bounds.GetBox().IsInsideOrOn(WorldPoint);
	if (!bInsideBox)
	{
		return false;
	}

	if (MaxAltitudeMetersASL >= MinAltitudeMetersASL)
	{
		// Optional ASL filter limits active zone height
		if (PointAltitudeMetersASL < MinAltitudeMetersASL || PointAltitudeMetersASL > MaxAltitudeMetersASL)
		{
			return false;
		}
	}

	return true;
}

// Check if a point enters the zone including safety clearance
bool AFlightInfluenceZoneActor::ContainsPointWithClearance(
	const FVector& WorldPoint,
	float PointAltitudeMetersASL,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters
) const
{
	if (!ZoneBox)
	{
		// Zone has no volume
		return false;
	}

	// Expand zone by required safety clearance
	const FVector ClearanceCm(
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, VerticalClearanceMeters) * 100.0f
	);

	if (!ZoneBox->Bounds.GetBox().ExpandBy(ClearanceCm).IsInsideOrOn(WorldPoint))
	{
		// Point is outside expanded safety volume
		return false;
	}

	if (MaxAltitudeMetersASL >= MinAltitudeMetersASL)
	{
		// Expand optional ASL filter by vertical clearance
		const float VerticalClearance = FMath::Max(0.0f, VerticalClearanceMeters);
		if (PointAltitudeMetersASL < MinAltitudeMetersASL - VerticalClearance ||
			PointAltitudeMetersASL > MaxAltitudeMetersASL + VerticalClearance)
		{
			return false;
		}
	}

	return true;
}

// Check if a route segment touches this zone
bool AFlightInfluenceZoneActor::IntersectsSegmentBySampling(
	const FVector& From,
	const FVector& To,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL,
	float SampleStepCm
) const
{
	// Treat very short segment as one point
	const float DistanceCm = FVector::Distance(From, To);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return ContainsPoint(From, FromAltitudeMetersASL);
	}

	// Choose enough samples for this segment length
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / FMath::Max(1.0f, SampleStepCm)));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Walk from segment start to end
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);

		// Check position and altitude at this sample
		const FVector SamplePoint = FMath::Lerp(From, To, Alpha);
		const float SampleAltitude = FMath::Lerp(FromAltitudeMetersASL, ToAltitudeMetersASL, Alpha);

		if (ContainsPoint(SamplePoint, SampleAltitude))
		{
			// Segment enters this zone
			return true;
		}
	}

	return false;
}

// Check if a route segment touches this zone including safety clearance
bool AFlightInfluenceZoneActor::IntersectsSegmentWithClearanceBySampling(
	const FVector& From,
	const FVector& To,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters,
	float SampleStepCm
) const
{
	if (!ZoneBox)
	{
		// Zone has no volume
		return false;
	}

	// Expand box by requested route clearance
	const FVector ClearanceCm(
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, VerticalClearanceMeters) * 100.0f
	);

	// Fast reject if segment does not even touch expanded box
	const FBox ExpandedBox = ZoneBox->Bounds.GetBox().ExpandBy(ClearanceCm);
	if (!DoesInfluenceSegmentIntersectBox(ExpandedBox, From, To))
	{
		return false;
	}

	if (MaxAltitudeMetersASL < MinAltitudeMetersASL)
	{
		// Box height alone defines the active zone
		return true;
	}

	// Treat very short segment as one point
	const float DistanceCm = FVector::Distance(From, To);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return ContainsPointWithClearance(
			From,
			FromAltitudeMetersASL,
			HorizontalClearanceMeters,
			VerticalClearanceMeters
		);
	}

	// Sample segment only after fast box intersection passed
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / FMath::Max(1.0f, SampleStepCm)));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Walk from segment start to end
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);

		// Check position and altitude at this sample
		const FVector SamplePoint = FMath::Lerp(From, To, Alpha);
		const float SampleAltitude = FMath::Lerp(FromAltitudeMetersASL, ToAltitudeMetersASL, Alpha);

		if (ContainsPointWithClearance(
			SamplePoint,
			SampleAltitude,
			HorizontalClearanceMeters,
			VerticalClearanceMeters
		))
		{
			// Segment enters clearance-expanded zone
			return true;
		}
	}

	return false;
}

// Check if shuffle mode creates a full-height airspace block
bool AFlightInfluenceZoneActor::IsCompleteAirspaceBlock() const
{
	return ShuffleAltitudeMode == EFlightInfluenceShuffleAltitudeMode::CompleteAirspaceBlock;
}