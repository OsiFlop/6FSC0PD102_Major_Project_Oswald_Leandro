// Route calculation result
// Stores success state, route points, failure reason, and debug stats
// Returned from route planning calls to the UI

#pragma once

#include "CoreMinimal.h"
#include "RouteFailureReason.h"
#include "RouteCalculationResult.generated.h"

/**
 * Full result of a route calculation.
 * This is returned to the RoutePlanner UI.
 */
USTRUCT(BlueprintType)
struct FRouteCalculationResult
{
	GENERATED_BODY()

public:

	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	TArray<FVector> RoutePoints;

	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	ERouteFailureReason FailureReason = ERouteFailureReason::None;

	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	FText FailureText;

	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	int32 ExpandedStates = 0;
};