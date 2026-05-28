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

	UPROPERTY(BlueprintReadOnly, Category = "Route Planner|Aircraft")
	TObjectPtr<UFlightProfile> SelectedFlightProfile;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AFlightPathfinderActor> PathfinderActor;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> StartMarker;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Route Planner|References")
	TObjectPtr<AActor> TargetMarker;

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
};