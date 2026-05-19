// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FlightProfile.generated.h"

/**
 * 
 */
UCLASS(BlueprintType)
class MAJORPROJECT_API UFlightProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	// Anzeigename des Flugzeugs
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft")
	FString AircraftName;

	// Maximale Flughöhe in Metern über Meer
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxAltitudeMetersASL = 0.0f;

	// Reisegeschwindigkeit in Metern pro Sekunde
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float CruiseSpeedMetersPerSecond = 0.0f;

	// Maximale Steigrate in Metern pro Sekunde
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxClimbRateMetersPerSecond = 0.0f;

	// Maximale Sinkrate in Metern pro Sekunde
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float MaxDescentRateMetersPerSecond = 0.0f;

	// Mindestabstand zum Terrain in Metern
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Safety", meta=(ClampMin="0.0"))
	float MinimumTerrainClearanceMeters = 0.0f;

	// Reichweite in Kilometern
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float RangeMeters = 0.0f;

	// Stall Speed in Knoten
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Aircraft", meta=(ClampMin="0.0"))
	float StallSpeedMetersPerSecond = 0.0f;
	
};
