// Flight weather zone actor
// Static 3D weather boxes used by flight route planning
// Blocks routes for dangerous weather
// Checks cloud clearance for scattered clouds
// Provides debug and UI data for weather zones

#include "FlightWeatherZoneActor.h"

#include "Components/BoxComponent.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"

namespace
{
	// Clip one segment axis against box limits
	bool IntersectSegmentAxis(float Start, float Delta, float Min, float Max, float& InOutTMin, float& InOutTMax)
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
	bool DoesSegmentIntersectBox(const FBox& Box, const FVector& From, const FVector& To)
	{
		// Segment direction used for axis clipping
		const FVector Delta = To - From;

		// Valid segment range from start to end
		float TMin = 0.0f;
		float TMax = 1.0f;

		// Segment must overlap the box on all three axes
		return
			IntersectSegmentAxis(From.X, Delta.X, Box.Min.X, Box.Max.X, TMin, TMax) &&
			IntersectSegmentAxis(From.Y, Delta.Y, Box.Min.Y, Box.Max.Y, TMin, TMax) &&
			IntersectSegmentAxis(From.Z, Delta.Z, Box.Min.Z, Box.Max.Z, TMin, TMax);
	}
}

// Setup weather zone actor
AFlightWeatherZoneActor::AFlightWeatherZoneActor()
{
	// Weather zone is only queried manually, no tick needed
	PrimaryActorTick.bCanEverTick = false;

#if WITH_EDITOR
	// Prevent accidental movement by transform gizmo
	SetLockLocation(true);
#endif

	// Box defines the weather volume in the level
	ZoneBox = CreateDefaultSubobject<UBoxComponent>(TEXT("ZoneBox"));
	SetRootComponent(ZoneBox);

	// Start with a broad, easy-to-select editor volume
	ZoneBox->InitBoxExtent(FVector(1500.0f, 1500.0f, 500.0f));

	// Weather volume is a visual/query helper, not a physics object
	ZoneBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ZoneBox->SetGenerateOverlapEvents(false);
	ZoneBox->SetCanEverAffectNavigation(false);
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);

	// Apply initial color and visibility
	UpdateDebugAppearance();
}

// Refresh debug appearance when actor is constructed
void AFlightWeatherZoneActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// Keep editor preview synced with current weather type
	UpdateDebugAppearance();
}

#if WITH_EDITOR
// Refresh debug appearance after editor property changes
void AFlightWeatherZoneActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Update color when weather settings change in Details
	UpdateDebugAppearance();
}
#endif

// Randomize weather zone position and size
void AFlightWeatherZoneActor::ShuffleWeatherZone()
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

	// Start with default shuffle ranges
	float MinHorizontalSizeMeters = FMath::Max(100.0f, ShuffleMinHorizontalSizeMeters);
	float MaxHorizontalSizeMeters = FMath::Max(MinHorizontalSizeMeters, ShuffleMaxHorizontalSizeMeters);
	float MinVerticalSizeMeters = FMath::Max(100.0f, ShuffleMinVerticalSizeMeters);
	float MaxVerticalSizeMeters = FMath::Max(MinVerticalSizeMeters, ShuffleMaxVerticalSizeMeters);
	float MinCenterHeightAboveMapMeters = ShuffleMinCenterHeightAboveMapMeters;
	float MaxCenterHeightAboveMapMeters = ShuffleMaxCenterHeightAboveMapMeters;
	float MinBottomClearanceMeters = 100.0f;

	if (WeatherType == EFlightWeatherType::Fog)
	{
		// Fog stays lower and wider near the terrain
		MinHorizontalSizeMeters = 3000.0f;
		MaxHorizontalSizeMeters = 9000.0f;
		MinVerticalSizeMeters = 200.0f;
		MaxVerticalSizeMeters = 900.0f;
		MinCenterHeightAboveMapMeters = 150.0f;
		MaxCenterHeightAboveMapMeters = 700.0f;
		MinBottomClearanceMeters = 20.0f;
	}
	else if (WeatherType == EFlightWeatherType::Thunderstorm)
	{
		// Thunderstorms are large and extend high into the airspace
		MinHorizontalSizeMeters = 7000.0f;
		MaxHorizontalSizeMeters = 15000.0f;
		MinVerticalSizeMeters = 2500.0f;
		MaxVerticalSizeMeters = 6500.0f;
		MinCenterHeightAboveMapMeters = 1200.0f;
		MaxCenterHeightAboveMapMeters = 3800.0f;
		MinBottomClearanceMeters = 50.0f;
	}

	// Pick random weather volume size
	const float SizeXMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);
	const float SizeYMeters = FMath::FRandRange(MinHorizontalSizeMeters, MaxHorizontalSizeMeters);
	const float SizeZMeters = FMath::FRandRange(MinVerticalSizeMeters, MaxVerticalSizeMeters);

	// Box extent uses half size in Unreal centimeters
	const FVector NewExtentCm(SizeXMeters * 50.0f, SizeYMeters * 50.0f, SizeZMeters * 50.0f);

	// Keep shuffled zone away from map border
	const float PaddingCm = FMath::Max(0.0f, ShuffleMapPaddingMeters) * 100.0f;
	const float MinX = MapBounds.Min.X + PaddingCm + NewExtentCm.X;
	const float MaxX = MapBounds.Max.X - PaddingCm - NewExtentCm.X;
	const float MinY = MapBounds.Min.Y + PaddingCm + NewExtentCm.Y;
	const float MaxY = MapBounds.Max.Y - PaddingCm - NewExtentCm.Y;

	// Pick random position inside usable map area
	const float WorldX = (MaxX > MinX) ? FMath::FRandRange(MinX, MaxX) : MapBounds.GetCenter().X;
	const float WorldY = (MaxY > MinY) ? FMath::FRandRange(MinY, MaxY) : MapBounds.GetCenter().Y;

	// Keep the weather volume above the map surface
	const float MinCenterHeightCm = FMath::Max(
		MinCenterHeightAboveMapMeters * 100.0f,
		NewExtentCm.Z + (MinBottomClearanceMeters * 100.0f)
	);

	// Ensure maximum height is never below the minimum
	const float MaxCenterHeightCm = FMath::Max(MinCenterHeightCm, MaxCenterHeightAboveMapMeters * 100.0f);

	// Place weather volume relative to map top
	const float WorldZ = MapBounds.Max.Z + FMath::FRandRange(MinCenterHeightCm, MaxCenterHeightCm);

	// Apply shuffled transform and size
	SetActorScale3D(FVector::OneVector);
	SetActorRotation(FRotator::ZeroRotator);
	SetActorLocation(FVector(WorldX, WorldY, WorldZ));
	ZoneBox->SetBoxExtent(NewExtentCm, true);

	// Refresh color and visibility after shuffle
	UpdateDebugAppearance();
}

// Check if this weather type blocks route traversal
bool AFlightWeatherZoneActor::IsHardBlock() const
{
	return WeatherType == EFlightWeatherType::Thunderstorm || WeatherType == EFlightWeatherType::Fog;
}

// Check if this weather type needs cloud clearance rules
bool AFlightWeatherZoneActor::IsScatteredClouds() const
{
	return WeatherType == EFlightWeatherType::ScatteredClouds;
}

// Check if a point is inside the weather volume
bool AFlightWeatherZoneActor::ContainsPoint(const FVector& WorldPoint) const
{
	if (!ZoneBox)
	{
		// Weather zone has no volume
		return false;
	}

	// Transform world point into box-local space
	const FTransform& BoxTransform = ZoneBox->GetComponentTransform();
	const FVector LocalPoint = BoxTransform.InverseTransformPosition(WorldPoint);

	// Use unscaled extent for local box check
	const FVector BoxExtent = ZoneBox->GetUnscaledBoxExtent();

	// Point must be inside all three local box axes
	return FMath::Abs(LocalPoint.X) <= BoxExtent.X + KINDA_SMALL_NUMBER &&
		FMath::Abs(LocalPoint.Y) <= BoxExtent.Y + KINDA_SMALL_NUMBER &&
		FMath::Abs(LocalPoint.Z) <= BoxExtent.Z + KINDA_SMALL_NUMBER;
}

// Check if a point is inside the weather volume with safety clearance
bool AFlightWeatherZoneActor::ContainsPointWithClearance(
	const FVector& WorldPoint,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters
) const
{
	if (!ZoneBox)
	{
		// Weather zone has no volume
		return false;
	}

	// Expand zone by required weather clearance
	const FVector ClearanceCm(
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, VerticalClearanceMeters) * 100.0f
	);

	// Check point against expanded world-space box
	return ZoneBox->Bounds.GetBox().ExpandBy(ClearanceCm).IsInsideOrOn(WorldPoint);
}

// Check if a route segment crosses this weather zone
bool AFlightWeatherZoneActor::IntersectsSegmentBySampling(
	const FVector& From,
	const FVector& To,
	float SampleStepCm
) const
{
	// Segment-box test replaces step sampling here
	(void)SampleStepCm;

	if (!ZoneBox)
	{
		// Weather zone has no volume
		return false;
	}

	// Transform segment into local box space
	const FTransform& BoxTransform = ZoneBox->GetComponentTransform();
	const FVector LocalFrom = BoxTransform.InverseTransformPosition(From);
	const FVector LocalTo = BoxTransform.InverseTransformPosition(To);

	// Build local box around origin
	const FVector BoxExtent = ZoneBox->GetUnscaledBoxExtent();
	const FBox LocalBox(-BoxExtent, BoxExtent);

	// Test segment against local weather box
	return DoesSegmentIntersectBox(LocalBox, LocalFrom, LocalTo);
}

// Check if a route segment crosses the weather zone with safety clearance
bool AFlightWeatherZoneActor::IntersectsSegmentWithClearanceBySampling(
	const FVector& From,
	const FVector& To,
	float HorizontalClearanceMeters,
	float VerticalClearanceMeters,
	float SampleStepCm
) const
{
	// Segment-box test replaces step sampling here
	(void)SampleStepCm;

	if (!ZoneBox)
	{
		// Weather zone has no volume
		return false;
	}

	// Expand zone by required weather clearance
	const FVector ClearanceCm(
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, HorizontalClearanceMeters) * 100.0f,
		FMath::Max(0.0f, VerticalClearanceMeters) * 100.0f
	);

	// Test segment against expanded world-space box
	const FBox ExpandedBox = ZoneBox->Bounds.GetBox().ExpandBy(ClearanceCm);
	return DoesSegmentIntersectBox(ExpandedBox, From, To);
}

// Get debug color for current weather type
FColor AFlightWeatherZoneActor::GetWeatherDebugColor() const
{
	switch (WeatherType)
	{
	case EFlightWeatherType::Thunderstorm:
		// Purple marks dangerous thunderstorm cells
		return FColor(160, 64, 255);

	case EFlightWeatherType::Fog:
		// Grey marks fog areas
		return FColor(190, 190, 190);

	case EFlightWeatherType::ScatteredClouds:
		// Cyan marks cloud clearance areas
		return FColor(80, 220, 255);

	default:
		// Fallback color for unknown weather type
		return FColor::White;
	}
}

// Get map bounds used for weather shuffling
bool AFlightWeatherZoneActor::GetShuffleWorldBounds(FBox& OutBounds) const
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

// Apply debug color and visibility settings
void AFlightWeatherZoneActor::UpdateDebugAppearance()
{
	if (!ZoneBox)
	{
		// No box component to update
		return;
	}

	// Color volume based on weather type
	ZoneBox->ShapeColor = GetWeatherDebugColor();

	// Keep weather zone visible for editor and debug use
	ZoneBox->SetHiddenInGame(false);
	ZoneBox->SetVisibility(true);
	ZoneBox->MarkRenderStateDirty();
}

// Enable or disable this weather zone
void AFlightWeatherZoneActor::SetWeatherEnabled(bool bEnabled)
{
	// Store active state for route planning/UI
	bWeatherEnabled = bEnabled;

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

// Check if this weather zone is active
bool AFlightWeatherZoneActor::IsWeatherEnabled() const
{
	return bWeatherEnabled;
}

// Get horizontal world bounds for map/UI display
void AFlightWeatherZoneActor::GetWeatherWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const
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

// Get vertical weather range in meters above sea level
void AFlightWeatherZoneActor::GetWeatherHeightRangeMetersASL(
	float& OutBottomMetersASL,
	float& OutTopMetersASL
) const
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

// Get transparent UI color for this weather type
FLinearColor AFlightWeatherZoneActor::GetWeatherUIColor() const
{
	// Start from debug color for consistent UI mapping
	const FColor DebugColor = GetWeatherDebugColor();

	// Use transparent fill color for map overlays
	FLinearColor Color = FLinearColor(DebugColor);
	Color.A = 0.35f;

	return Color;
}

// Get readable weather type text for UI
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