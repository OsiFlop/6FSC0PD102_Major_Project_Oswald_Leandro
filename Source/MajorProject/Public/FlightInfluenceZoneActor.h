// Flight influence zone actor
// Defines restricted or costly airspace volumes
// Can work as hard block or soft route penalty
// Uses a box volume plus optional altitude limits
// Used by pathfinding to avoid or penalize airspace areas

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlightInfluenceZoneActor.generated.h"

class UBoxComponent;

UENUM(BlueprintType)
enum class EFlightInfluenceShuffleAltitudeMode : uint8
{
	CompleteAirspaceBlock UMETA(DisplayName="Complete Airspace Block"),
	RandomAltitudeBand UMETA(DisplayName="Random Altitude Band")
};

UCLASS()
class MAJORPROJECT_API AFlightInfluenceZoneActor : public AActor
{
	GENERATED_BODY()
	
public:
	AFlightInfluenceZoneActor();

	// Box volume used for zone checks and editor visibility
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Zone")
	TObjectPtr<UBoxComponent> ZoneBox;

	// Hard block zones are forbidden, soft zones only add route cost
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	bool bHardBlock = true;

	// Extra A* cost applied when route crosses a soft zone
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone", meta=(ClampMin="0.0"))
	float AdditionalTraversalCost = 0.0f;

	// Optional lower ASL limit for this zone
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	float MinAltitudeMetersASL = 0.0f;

	// Optional upper ASL limit, disabled when smaller than minimum
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	float MaxAltitudeMetersASL = -1.0f;

	// World Z reference used to convert box height to meters ASL
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Zone")
	float SeaLevelWorldZCm = 0.0f;

	// Randomize this influence zone's position and size on the terrain
	UFUNCTION(CallInEditor, BlueprintCallable, Category="Zone")
	void ShuffleInfluenceZone();

	// Chooses how shuffled zones are placed vertically
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Zone")
	EFlightInfluenceShuffleAltitudeMode ShuffleAltitudeMode = EFlightInfluenceShuffleAltitudeMode::RandomAltitudeBand;

	// Enable or disable this zone for route planning and visibility
	UFUNCTION(BlueprintCallable, Category="Zone")
	void SetInfluenceEnabled(bool bEnabled);

	// Get horizontal world bounds for map/UI display
	UFUNCTION(BlueprintPure, Category="Zone")
	void GetInfluenceWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const;

	// Get vertical zone range in meters above sea level
	UFUNCTION(BlueprintPure, Category="Zone")
	void GetInfluenceHeightRangeMetersASL(float& OutBottomMetersASL, float& OutTopMetersASL) const;

	// Fallback map minimum used when no landscape bounds are available
	UPROPERTY()
	FVector2D ShuffleFallbackMapMin = FVector2D(-50000.0f, -50000.0f);

	// Fallback map maximum used when no landscape bounds are available
	UPROPERTY()
	FVector2D ShuffleFallbackMapMax = FVector2D(50000.0f, 50000.0f);

	// Keeps shuffled zones away from the map edge
	UPROPERTY()
	float ShuffleMapPaddingMeters = 500.0f;

	// Minimum shuffled zone size in XY
	UPROPERTY()
	float ShuffleMinHorizontalSizeMeters = 1500.0f;

	// Maximum shuffled zone size in XY
	UPROPERTY()
	float ShuffleMaxHorizontalSizeMeters = 6500.0f;

	// Minimum shuffled height for altitude-band zones
	UPROPERTY()
	float ShuffleMinVerticalSizeMeters = 300.0f;

	// Maximum shuffled height for altitude-band zones
	UPROPERTY()
	float ShuffleMaxVerticalSizeMeters = 1800.0f;

	// Full airspace block starts this far below terrain
	UPROPERTY()
	float ShuffleFullBlockBottomOffsetMeters = 500.0f;

	// Full airspace block reaches this far above terrain
	UPROPERTY()
	float ShuffleFullBlockTopHeightMeters = 20000.0f;

	// Disabled zones are ignored by route planning and hidden
	UPROPERTY()
	bool bInfluenceEnabled = false;

	// Check if a point is inside the active zone
	bool ContainsPoint(const FVector& WorldPoint, float PointAltitudeMetersASL) const;

	// Check if a point is inside the zone with safety clearance
	bool ContainsPointWithClearance(
		const FVector& WorldPoint,
		float PointAltitudeMetersASL,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters
	) const;

	// Check if a route segment crosses this zone
	bool IntersectsSegmentBySampling(
		const FVector& From,
		const FVector& To,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL,
		float SampleStepCm = 500.0f
	) const;

	// Check if a route segment crosses the zone with safety clearance
	bool IntersectsSegmentWithClearanceBySampling(
		const FVector& From,
		const FVector& To,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters,
		float SampleStepCm = 500.0f
	) const;

	// Check if this zone represents a full-height airspace block
	UFUNCTION(BlueprintPure, Category="Zone|Shuffle")
	bool IsCompleteAirspaceBlock() const;

protected:
	// Get map bounds from landscape or fallback settings
	bool GetShuffleWorldBounds(FBox& OutBounds) const;

	// Get terrain height at shuffled XY position
	float GetShuffleTerrainZAtWorldXY(float WorldX, float WorldY, const FBox& MapBounds) const;
};