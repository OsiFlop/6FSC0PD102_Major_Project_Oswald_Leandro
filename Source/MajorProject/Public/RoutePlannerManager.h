// Route planner manager actor
// Public interface for UI-driven route calculation
// Stores aircraft profiles, selected route markers and result data
// Connects UI input with flight pathfinder actor
// Handles map bounds and optional display settings

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RouteCalculationResult.h"
#include "RoutePlannerManager.generated.h"

class UFlightProfile;
class AFlightPathfinderActor;

UCLASS()
class MAJORPROJECT_API ARoutePlannerManager : public AActor
{
	GENERATED_BODY()

public:
	ARoutePlannerManager();

	// AvailableFlightProfiles: selectable aircraft profiles for UI
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|Aircraft")
	TArray<TObjectPtr<UFlightProfile>> AvailableFlightProfiles;

	// SelectedFlightProfile: currently active aircraft profile
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Aircraft")
	TObjectPtr<UFlightProfile> SelectedFlightProfile;

	// PathfinderActor: actor that performs the actual route search
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AFlightPathfinderActor> PathfinderActor;

	// StartMarker: world actor used as route start point
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> StartMarker;

	// TargetMarker: world actor used as route target point
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> TargetMarker;

	// MapWorldMin: minimum world-space XY corner shown by route map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Map")
	FVector2D MapWorldMin = FVector2D(-50000.0f, -50000.0f);

	// MapWorldMax: maximum world-space XY corner shown by route map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Map")
	FVector2D MapWorldMax = FVector2D(50000.0f, 50000.0f);

	// CurrentRoutePoints: latest route result as world-space points
	UPROPERTY(BlueprintReadOnly, Category = "Route Planner|Result")
	TArray<FVector> CurrentRoutePoints;

	// LastResult: latest calculation status, route data and messages
	UPROPERTY(BlueprintReadOnly, Category = "Route Planner|Result")
	FRouteCalculationResult LastResult;

	// bShowWeather: UI option for weather overlay / weather-aware routing
	UPROPERTY(BlueprintReadWrite, Category = "Route Planner|Options")
	bool bShowWeather = false;

	// bShowRestrictedZones: UI option for restricted zone overlay / restriction-aware routing
	UPROPERTY(BlueprintReadWrite, Category = "Route Planner|Options")
	bool bShowRestrictedZones = true;

	// Set active aircraft profile from UI
	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	void SetSelectedFlightProfile(UFlightProfile* NewProfile);
	
	// Calculate route using marker positions and requested altitudes
	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	FRouteCalculationResult CalculateRouteFromMarkers(
		float StartAltitudeMetersASL,
		float TargetAltitudeMetersASL
	);

	// Clear current route result and pathfinder route
	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	void ClearCurrentRoute();

	// Move start marker to map-selected world XY position
	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	void SetStartMarkerWorldXY(FVector2D WorldXY);

	// Move target marker to map-selected world XY position
	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	void SetTargetMarkerWorldXY(FVector2D WorldXY);

	// Get current start marker world location
	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	FVector GetStartMarkerLocation() const;

	// Get current target marker world location
	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	FVector GetTargetMarkerLocation() const;
};