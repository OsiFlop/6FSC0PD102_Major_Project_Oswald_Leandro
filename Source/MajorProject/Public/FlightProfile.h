// Flight profile data asset
// Stores aircraft limits and safety values
// Used by pathfinding and route checks

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FlightProfile.generated.h"

/**
 * Aircraft profile settings
 * Editable asset for aircraft performance data
 */
UCLASS(BlueprintType)
class MAJORPROJECT_API UFlightProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	// AircraftName: display name / profile identifier
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft")
	FString AircraftName;

	// MaxAltitudeMetersASL: highest allowed altitude above sea level
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxAltitudeMetersASL = 0.0f;

	// CruiseSpeedMetersPerSecond: normal flight speed used for time estimates
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float CruiseSpeedMetersPerSecond = 0.0f;

	// MaxClimbRateMetersPerSecond: maximum vertical climb speed
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxClimbRateMetersPerSecond = 0.0f;

	// MaxDescentRateMetersPerSecond: maximum vertical descent speed
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxDescentRateMetersPerSecond = 0.0f;

	// MinimumTerrainClearanceMeters: required safety distance above terrain
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float MinimumTerrainClearanceMeters = 0.0f;

	// RangeMeters: maximum flight distance for this aircraft
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float RangeMeters = 0.0f;

	// StallSpeedMetersPerSecond: minimum safe airspeed
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float StallSpeedMetersPerSecond = 0.0f;

	// MinimumTurnRadiusMeters: smallest possible (safe) turn radius 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MinimumTurnRadiusMeters = 0.0f;
};