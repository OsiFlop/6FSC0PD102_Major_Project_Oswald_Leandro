// Flight weather zone actor
// Defines static 3D weather volumes for route planning
// Hard weather blocks routes
// Scattered clouds require VFR cloud clearance
// Provides UI bounds, colors and route safety checks

#pragma once

#include "CoreMinimal.h"
#include "FlightWeatherType.h"
#include "GameFramework/Actor.h"
#include "FlightWeatherZoneActor.generated.h"

class UBoxComponent;

#if WITH_EDITOR
struct FPropertyChangedEvent;
#endif

UCLASS(HideCategories=(Transformation))
class MAJORPROJECT_API AFlightWeatherZoneActor : public AActor
{
	GENERATED_BODY()

public:
	AFlightWeatherZoneActor();

	virtual void OnConstruction(const FTransform& Transform) override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	// Box volume used for weather checks and editor visibility
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Weather")
	TObjectPtr<UBoxComponent> ZoneBox;

	// Weather type controls color and route behavior
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Weather")
	EFlightWeatherType WeatherType = EFlightWeatherType::Thunderstorm;

	// Randomize this weather zone's position and size on the map
	UFUNCTION(CallInEditor, BlueprintCallable, Category="Weather")
	void ShuffleWeatherZone();

	// Check if this weather type blocks traversal completely
	UFUNCTION(BlueprintPure, Category="Weather")
	bool IsHardBlock() const;

	// Check if this weather zone represents scattered clouds
	UFUNCTION(BlueprintPure, Category="Weather")
	bool IsScatteredClouds() const;

	// Check if a world point is inside the weather volume
	UFUNCTION(BlueprintPure, Category="Weather")
	bool ContainsPoint(const FVector& WorldPoint) const;

	// Get debug color for the selected weather type
	UFUNCTION(BlueprintPure, Category="Weather")
	FColor GetWeatherDebugColor() const;

	// Enable or disable this zone for route planning and visibility
	UFUNCTION(BlueprintCallable, Category="Weather")
	void SetWeatherEnabled(bool bEnabled);

	// Check if this weather zone is active
	UFUNCTION(BlueprintPure, Category="Weather")
	bool IsWeatherEnabled() const;

	// Get horizontal world bounds for map/UI display
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	void GetWeatherWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const;

	// Get vertical weather range in meters above sea level
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	void GetWeatherHeightRangeMetersASL(float& OutBottomMetersASL, float& OutTopMetersASL) const;

	// Get UI color for the selected weather type
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	FLinearColor GetWeatherUIColor() const;

	// Get readable weather type text for UI
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	FText GetWeatherTypeText() const;

	// World Z reference used to convert box height to meters ASL
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Weather")
	float SeaLevelWorldZCm = 0.0f;

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
	float ShuffleMinHorizontalSizeMeters = 2500.0f;

	// Maximum shuffled zone size in XY
	UPROPERTY()
	float ShuffleMaxHorizontalSizeMeters = 8000.0f;

	// Minimum shuffled weather volume height
	UPROPERTY()
	float ShuffleMinVerticalSizeMeters = 600.0f;

	// Maximum shuffled weather volume height
	UPROPERTY()
	float ShuffleMaxVerticalSizeMeters = 1800.0f;

	// Minimum shuffled center height above map
	UPROPERTY()
	float ShuffleMinCenterHeightAboveMapMeters = 1200.0f;

	// Maximum shuffled center height above map
	UPROPERTY()
	float ShuffleMaxCenterHeightAboveMapMeters = 4000.0f;

	// Disabled zones are ignored by route planning and hidden
	UPROPERTY()
	bool bWeatherEnabled = false;

	// Check if a point is inside the weather volume with safety clearance
	bool ContainsPointWithClearance(
		const FVector& WorldPoint,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters
	) const;

	// Check if a route segment crosses this weather zone
	bool IntersectsSegmentBySampling(
		const FVector& From,
		const FVector& To,
		float SampleStepCm = 500.0f
	) const;

	// Check if a route segment crosses the weather zone with safety clearance
	bool IntersectsSegmentWithClearanceBySampling(
		const FVector& From,
		const FVector& To,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters,
		float SampleStepCm = 500.0f
	) const;

protected:
	// Get map bounds from landscape or fallback settings
	bool GetShuffleWorldBounds(FBox& OutBounds) const;

	// Apply weather color and visibility to the box component
	void UpdateDebugAppearance();

};