// Flight weather zone actor
// Defines static 3D weather volumes for route planning
// Thunderstorms and fog block routes, scattered clouds require VFR cloud clearance

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

	// ZoneBox: visible 3D debug volume for this static weather area
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Weather")
	TObjectPtr<UBoxComponent> ZoneBox;

	// WeatherType: selected in Details; updates color and route behavior
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

	// Check if a world point lies inside this 3D weather box
	UFUNCTION(BlueprintPure, Category="Weather")
	bool ContainsPoint(const FVector& WorldPoint) const;

	// Get debug color for the selected weather type
	UFUNCTION(BlueprintPure, Category="Weather")
	FColor GetWeatherDebugColor() const;
	
	// Enable/disable this weather zone for gameplay and visibility.
	UFUNCTION(BlueprintCallable, Category="Weather")
	void SetWeatherEnabled(bool bEnabled);

	// Check if this weather zone is currently enabled.
	UFUNCTION(BlueprintPure, Category="Weather")
	bool IsWeatherEnabled() const;

	// Get the XY world bounds of the box.
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	void GetWeatherWorldXYBounds(FVector2D& OutMinXY, FVector2D& OutMaxXY) const;

	// Get the vertical altitude range of the weather box in meters relative to world Z.
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	void GetWeatherHeightRangeMeters(float& OutBottomMeters, float& OutTopMeters) const;

	// Get the UI color as LinearColor.
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	FLinearColor GetWeatherUIColor() const;

	// Get readable weather type text.
	UFUNCTION(BlueprintPure, Category="Weather|UI")
	FText GetWeatherTypeText() const;
	
	// ShuffleFallbackMapMin: XY bounds used when no landscape is found
	UPROPERTY()
	FVector2D ShuffleFallbackMapMin = FVector2D(-50000.0f, -50000.0f);

	// ShuffleFallbackMapMax: XY bounds used when no landscape is found
	UPROPERTY()
	FVector2D ShuffleFallbackMapMax = FVector2D(50000.0f, 50000.0f);

	// ShuffleMapPaddingMeters: keeps shuffled zones away from the map edge
	UPROPERTY()
	float ShuffleMapPaddingMeters = 500.0f;

	// ShuffleMinHorizontalSizeMeters: minimum random box width/depth
	UPROPERTY()
	float ShuffleMinHorizontalSizeMeters = 2500.0f;

	// ShuffleMaxHorizontalSizeMeters: maximum random box width/depth
	UPROPERTY()
	float ShuffleMaxHorizontalSizeMeters = 8000.0f;

	// ShuffleMinVerticalSizeMeters: minimum random box height
	UPROPERTY()
	float ShuffleMinVerticalSizeMeters = 600.0f;

	// ShuffleMaxVerticalSizeMeters: maximum random box height
	UPROPERTY()
	float ShuffleMaxVerticalSizeMeters = 1800.0f;

	// ShuffleMinCenterHeightAboveMapMeters: minimum random center height above landscape top
	UPROPERTY()
	float ShuffleMinCenterHeightAboveMapMeters = 1200.0f;

	// ShuffleMaxCenterHeightAboveMapMeters: maximum random center height above landscape top
	UPROPERTY()
	float ShuffleMaxCenterHeightAboveMapMeters = 4000.0f;

	// If false, this zone is ignored by route planning and hidden in UI/game.
	UPROPERTY()
	bool bWeatherEnabled = false;

	// Check if a point is inside this box plus a horizontal/vertical clearance buffer
	bool ContainsPointWithClearance(
		const FVector& WorldPoint,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters
	) const;

	// Check if a segment touches this weather zone by sampling along the line
	bool IntersectsSegmentBySampling(
		const FVector& From,
		const FVector& To,
		float SampleStepCm = 500.0f
	) const;

	// Check if a segment touches this weather zone plus a clearance buffer
	bool IntersectsSegmentWithClearanceBySampling(
		const FVector& From,
		const FVector& To,
		float HorizontalClearanceMeters,
		float VerticalClearanceMeters,
		float SampleStepCm = 500.0f
	) const;

protected:
	// Find map bounds from the landscape or fallback settings
	bool GetShuffleWorldBounds(FBox& OutBounds) const;

	// Apply debug color and visibility settings to the box component
	void UpdateDebugAppearance();
};