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

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|Aircraft")
	TArray<TObjectPtr<UFlightProfile>> AvailableFlightProfiles;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Aircraft")
	TObjectPtr<UFlightProfile> SelectedFlightProfile;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AFlightPathfinderActor> PathfinderActor;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> StartMarker;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> TargetMarker;

	// World-space minimum corner shown by the route map.
	// X/Y only. Usually matches the terrain or height cache bounds.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Map")
	FVector2D MapWorldMin = FVector2D(-50000.0f, -50000.0f);

	// World-space maximum corner shown by the route map.
	// X/Y only. Usually matches the terrain or height cache bounds.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route Planner|Map")
	FVector2D MapWorldMax = FVector2D(50000.0f, 50000.0f);

	UPROPERTY(BlueprintReadOnly, Category = "Route Planner|Result")
	TArray<FVector> CurrentRoutePoints;

	UPROPERTY(BlueprintReadOnly, Category = "Route Planner|Result")
	FRouteCalculationResult LastResult;

	UPROPERTY(BlueprintReadWrite, Category = "Route Planner|Options")
	bool bShowWeather = false;

	UPROPERTY(BlueprintReadWrite, Category = "Route Planner|Options")
	bool bShowRestrictedZones = true;

	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	void SetSelectedFlightProfile(UFlightProfile* NewProfile);

	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	UFlightProfile* GetSelectedFlightProfile() const;

	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	FRouteCalculationResult CalculateRouteFromMarkers(
		float StartAltitudeMetersASL,
		float TargetAltitudeMetersASL
	);

	UFUNCTION(BlueprintCallable, Category = "Route Planner")
	void ClearCurrentRoute();

	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	void SetStartMarkerWorldXY(FVector2D WorldXY);

	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	void SetTargetMarkerWorldXY(FVector2D WorldXY);

	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	FVector GetStartMarkerLocation() const;

	UFUNCTION(BlueprintCallable, Category = "Route Planner|Map")
	FVector GetTargetMarkerLocation() const;
};
