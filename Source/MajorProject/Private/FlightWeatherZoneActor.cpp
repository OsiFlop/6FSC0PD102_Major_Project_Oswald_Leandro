// Flight weather zone actor
// Static 3D weather boxes used by flight route planning

#include "FlightWeatherZoneActor.h"

#include "Components/BoxComponent.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"


AFlightWeatherZoneActor::AFlightWeatherZoneActor()
{
	PrimaryActorTick.bCanEverTick = false;

#if WITH_EDITOR
	SetLockLocation(true);
#endif

	ZoneBox = CreateDefaultSubobject<UBoxComponent>(TEXT("ZoneBox"));
	SetRootComponent(ZoneBox);

	// Default to a broad, easy-to-grab volume in the level.
	ZoneBox->InitBoxExtent(FVector(1500.0f, 1500.0f, 500.0f));
	ZoneBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ZoneBox->SetGenerateOverlapEvents(false);
	ZoneBox->SetCanEverAffectNavigation(false);
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);

	UpdateDebugAppearance();
}

void AFlightWeatherZoneActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	UpdateDebugAppearance();
}

#if WITH_EDITOR
void AFlightWeatherZoneActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	UpdateDebugAppearance();
}
#endif

void AFlightWeatherZoneActor::ShuffleWeatherZone()
{
	if (!ZoneBox)
	{
		return;
	}

#if WITH_EDITOR
	Modify();
	ZoneBox->Modify();
#endif

	FBox MapBounds;
	GetShuffleWorldBounds(MapBounds);

	const float MinHorizontalSizeMeters = FMath::Max(100.0f, ShuffleMinHorizontalSizeMeters);
	const float MaxHorizontalSizeMeters = FMath::Max(MinHorizontalSizeMeters, ShuffleMaxHorizontalSizeMeters);
	const float MinVerticalSizeMeters = FMath::Max(100.0f, ShuffleMinVerticalSizeMeters);
	const float MaxVerticalSizeMeters = FMath::Max(MinVerticalSizeMeters, ShuffleMaxVerticalSizeMeters);

	const float SizeXMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);
	const float SizeYMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);
	const float SizeZMeters = FMath::FRandRange(MinVerticalSizeMeters, MaxVerticalSizeMeters);
	const FVector NewExtentCm(SizeXMeters * 50.0f, SizeYMeters * 50.0f, SizeZMeters * 50.0f);

	const float PaddingCm = FMath::Max(0.0f, ShuffleMapPaddingMeters) * 100.0f;
	const float MinX = MapBounds.Min.X + PaddingCm + NewExtentCm.X;
	const float MaxX = MapBounds.Max.X - PaddingCm - NewExtentCm.X;
	const float MinY = MapBounds.Min.Y + PaddingCm + NewExtentCm.Y;
	const float MaxY = MapBounds.Max.Y - PaddingCm - NewExtentCm.Y;

	const float WorldX = (MaxX > MinX) ? FMath::FRandRange(MinX, MaxX) : MapBounds.GetCenter().X;
	const float WorldY = (MaxY > MinY) ? FMath::FRandRange(MinY, MaxY) : MapBounds.GetCenter().Y;

	const float MinCenterHeightCm = FMath::Max(ShuffleMinCenterHeightAboveMapMeters * 100.0f, NewExtentCm.Z + 10000.0f);
	const float MaxCenterHeightCm = FMath::Max(MinCenterHeightCm, ShuffleMaxCenterHeightAboveMapMeters * 100.0f);
	const float WorldZ = MapBounds.Max.Z + FMath::FRandRange(MinCenterHeightCm, MaxCenterHeightCm);

	SetActorScale3D(FVector::OneVector);
	SetActorRotation(FRotator::ZeroRotator);
	SetActorLocation(FVector(WorldX, WorldY, WorldZ));
	ZoneBox->SetBoxExtent(NewExtentCm, true);

	UpdateDebugAppearance();
}

bool AFlightWeatherZoneActor::IsHardBlock() const
{
	return WeatherType == EFlightWeatherType::Thunderstorm || WeatherType == EFlightWeatherType::Fog;
}

bool AFlightWeatherZoneActor::IsScatteredClouds() const
{
	return WeatherType == EFlightWeatherType::ScatteredClouds;
}

bool AFlightWeatherZoneActor::ContainsPoint(const FVector& WorldPoint) const
{
	if (!ZoneBox)
	{
		return false;
	}

	const FTransform& BoxTransform = ZoneBox->GetComponentTransform();
	const FVector LocalPoint = BoxTransform.InverseTransformPosition(WorldPoint);
	const FVector BoxExtent = ZoneBox->GetUnscaledBoxExtent();

	return FMath::Abs(LocalPoint.X) <= BoxExtent.X + KINDA_SMALL_NUMBER &&
		FMath::Abs(LocalPoint.Y) <= BoxExtent.Y + KINDA_SMALL_NUMBER &&
		FMath::Abs(LocalPoint.Z) <= BoxExtent.Z + KINDA_SMALL_NUMBER;
}

bool AFlightWeatherZoneActor::ContainsPointWithClearance(
	const FVector& WorldPoint,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters
) const
{
	if (!ZoneBox)
	{
		return false;
	}

	const FVector ClearanceCm(
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, VerticalClearanceMeters) * 100.0f
	);

	return ZoneBox->Bounds.GetBox().ExpandBy(ClearanceCm).IsInsideOrOn(WorldPoint);
}

bool AFlightWeatherZoneActor::IntersectsSegmentBySampling(
	const FVector& From,
	const FVector& To,
	float SampleStepCm
) const
{
	const float DistanceCm = FVector::Distance(From, To);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return ContainsPoint(From);
	}

	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / FMath::Max(1.0f, SampleStepCm)));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector SamplePoint = FMath::Lerp(From, To, Alpha);

		if (ContainsPoint(SamplePoint))
		{
			return true;
		}
	}

	return false;
}

bool AFlightWeatherZoneActor::IntersectsSegmentWithClearanceBySampling(
	const FVector& From,
	const FVector& To,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters,
	float SampleStepCm
) const
{
	const float DistanceCm = FVector::Distance(From, To);
	if (DistanceCm <= KINDA_SMALL_NUMBER)
	{
		return ContainsPointWithClearance(From, HorizontalClearanceMeters, VerticalClearanceMeters);
	}

	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(DistanceCm / FMath::Max(1.0f, SampleStepCm)));

	for (int32 Step = 0; Step <= NumSteps; ++Step)
	{
		const float Alpha = static_cast<float>(Step) / static_cast<float>(NumSteps);
		const FVector SamplePoint = FMath::Lerp(From, To, Alpha);

		if (ContainsPointWithClearance(SamplePoint, HorizontalClearanceMeters, VerticalClearanceMeters))
		{
			return true;
		}
	}

	return false;
}

FColor AFlightWeatherZoneActor::GetWeatherDebugColor() const
{
	switch (WeatherType)
	{
	case EFlightWeatherType::Thunderstorm:
		return FColor(160, 64, 255, 0.35);
	case EFlightWeatherType::Fog:
		return FColor(190, 190, 190, 0.35);
	case EFlightWeatherType::ScatteredClouds:
		return FColor(80, 220, 255, 0.35);
	default:
		return FColor::White;
	}
}

bool AFlightWeatherZoneActor::GetShuffleWorldBounds(FBox& OutBounds) const
{
	OutBounds.Init();

	if (const UWorld* World = GetWorld())
	{
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
		return true;
	}

	const FVector CurrentLocation = GetActorLocation();
	OutBounds = FBox(
		FVector(ShuffleFallbackMapMin.X, ShuffleFallbackMapMin.Y, CurrentLocation.Z),
		FVector(ShuffleFallbackMapMax.X, ShuffleFallbackMapMax.Y, CurrentLocation.Z)
	);
	return false;
}


void AFlightWeatherZoneActor::UpdateDebugAppearance()
{
	if (!ZoneBox)
	{
		return;
	}

	ZoneBox->ShapeColor = GetWeatherDebugColor();
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);
	ZoneBox->MarkRenderStateDirty();
}

void AFlightWeatherZoneActor::SetWeatherEnabled(bool bEnabled)
{
	bWeatherEnabled = bEnabled;

	SetActorHiddenInGame(!bEnabled);

	if (ZoneBox)
	{
		ZoneBox->SetVisibility(bEnabled, true);
		ZoneBox->SetHiddenInGame(!bEnabled);
		ZoneBox->MarkRenderStateDirty();
	}
}

bool AFlightWeatherZoneActor::IsWeatherEnabled() const
{
	return bWeatherEnabled;
}

void AFlightWeatherZoneActor::GetWeatherWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const
{
	OutMinXY = FVector2D::ZeroVector;
	OutMaxXY = FVector2D::ZeroVector;

	if (!ZoneBox)
	{
		return;
	}

	const FBox WorldBox = ZoneBox->Bounds.GetBox();

	OutMinXY = FVector2D(WorldBox.Min.X, WorldBox.Min.Y);
	OutMaxXY = FVector2D(WorldBox.Max.X, WorldBox.Max.Y);
}

void AFlightWeatherZoneActor::GetWeatherHeightRangeMeters(float& OutBottomMeters, float& OutTopMeters) const
{
	OutBottomMeters = 0.0f;
	OutTopMeters = 0.0f;

	if (!ZoneBox)
	{
		return;
	}

	const FBox WorldBox = ZoneBox->Bounds.GetBox();

	OutBottomMeters = WorldBox.Min.Z / 100.0f;
	OutTopMeters = WorldBox.Max.Z / 100.0f;
}

FLinearColor AFlightWeatherZoneActor::GetWeatherUIColor() const
{
	const FColor DebugColor = GetWeatherDebugColor();

	FLinearColor Color = FLinearColor(DebugColor);
	Color.A = 0.35f;

	return Color;
}

FText AFlightWeatherZoneActor::GetWeatherTypeText() const
{
	switch (WeatherType)
	{
	case EFlightWeatherType::Thunderstorm:
		return FText::FromString(TEXT("Thunderstorm"));

	case EFlightWeatherType::Fog:
		return FText::FromString(TEXT("Fog"));

	case EFlightWeatherType::ScatteredClouds:
		return FText::FromString(TEXT("Scattered Clouds"));

	default:
		return FText::FromString(TEXT("Unknown Weather"));
	}
}