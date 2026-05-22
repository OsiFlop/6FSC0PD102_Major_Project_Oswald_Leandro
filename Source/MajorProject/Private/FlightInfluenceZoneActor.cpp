// Flight influence zone actor
// Defines 3D zones for route restrictions or extra traversal cost
// Used by pathfinding to avoid or penalize specific airspace areas

#include "FlightInfluenceZoneActor.h"
#include "Components/BoxComponent.h"

// Sets default values
AFlightInfluenceZoneActor::AFlightInfluenceZoneActor()
{
	PrimaryActorTick.bCanEverTick = false;

	// ZoneBox: editable influence volume in level
	ZoneBox = CreateDefaultSubobject<UBoxComponent>(TEXT("ZoneBox"));
	SetRootComponent(ZoneBox);

	// Debug / query volume only, no physics collision
	ZoneBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ZoneBox->SetHiddenInGame(false);
}

// Check if world point is inside zone box and altitude limits
bool AFlightInfluenceZoneActor::ContainsPoint(const FVector& WorldPoint, float PointAltitudeMetersASL) const
{
	if (!ZoneBox)
	{
		return false;
	}

	// Box bounds check in world space
	const bool bInsideBox = ZoneBox->Bounds.GetBox().IsInsideOrOn(WorldPoint);
	if (!bInsideBox)
	{
		return false;
	}

	// Optional altitude filter, only active if max >= min
	if (MaxAltitudeMetersASL >= MinAltitudeMetersASL)
	{
		if (PointAltitudeMetersASL < MinAltitudeMetersASL || PointAltitudeMetersASL > MaxAltitudeMetersASL)
		{
			return false;
		}
	}

	return true;
}

// Check if route segment touches zone by sampling along line
bool AFlightInfluenceZoneActor::IntersectsSegmentBySampling(
	const FVector& From,
	const FVector& To,
	float FromAltitudeMetersASL,
	float ToAltitudeMetersASL,
	float SampleStepCm
) const
{
	// DistanceCm: segment length for sample count
	const float DistanceCm = FVector::Distance(From, To);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return ContainsPoint(From, FromAltitudeMetersASL);
	}

	// NumSteps: more distance = more samples
	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / FMath::Max(1.0f, SampleStepCm)));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		// Alpha: normalized position between From and To
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);

		// SamplePoint / SampleAltitude: interpolated route state
		const FVector SamplePoint = FMath::Lerp(From, To, Alpha);
		const float SampleAltitude = FMath::Lerp(FromAltitudeMetersASL, ToAltitudeMetersASL, Alpha);

		if (ContainsPoint(SamplePoint, SampleAltitude))
		{
			return true;
		}
	}

	return false;
}