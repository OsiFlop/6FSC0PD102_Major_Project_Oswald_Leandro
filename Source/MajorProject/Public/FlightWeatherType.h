// Flight weather type enum
// Defines static weather zone behavior for route planning

#pragma once

#include "CoreMinimal.h"
#include "FlightWeatherType.generated.h"

UENUM(BlueprintType)
enum class EFlightWeatherType : uint8
{
	Thunderstorm UMETA(DisplayName = "Thunderstorm"),
	Fog UMETA(DisplayName = "Fog"),
	ScatteredClouds UMETA(DisplayName = "Scattered Clouds")
};
