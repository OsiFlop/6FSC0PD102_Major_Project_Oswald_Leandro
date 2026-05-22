#pragma once

#include "CoreMinimal.h"
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

	// True if a valid route was found.
	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	bool bSuccess = false;

	// Route points in world space. Only valid if bSuccess is true.
	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	TArray<FVector> RoutePoints;

	// Reason why the route calculation failed.
	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	ERouteFailureReason FailureReason = ERouteFailureReason::None;

	// Human-readable failure message for the UI.
	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	FText FailureText;

	// Optional debug information: how many states the pathfinder expanded.
	UPROPERTY(BlueprintReadOnly, Category = "Route Result")
	int32 ExpandedStates = 0;
};