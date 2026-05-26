#pragma once

#include "CoreMinimal.h"
#include "RouteFailureReason.generated.h"

/**
 * Reason why a route calculation failed.
 * Used by the pathfinder and by the route planner UI.
 */
UENUM(BlueprintType)
enum class ERouteFailureReason : uint8
{
	None UMETA(DisplayName = "None"),

	InvalidStart UMETA(DisplayName = "Invalid Start"),
	InvalidTarget UMETA(DisplayName = "Invalid Target"),
	InvalidTargetState UMETA(DisplayName = "Invalid Target State"),
	InvalidFlightProfile UMETA(DisplayName = "Invalid Flight Profile"),

	StartAltitudeTooLow UMETA(DisplayName = "Start Altitude Too Low"),
	TargetAltitudeTooLow UMETA(DisplayName = "Target Altitude Too Low"),
	TargetAltitudeTooHigh UMETA(DisplayName = "Target Altitude Too High"),
	TargetAltitudeNotReachable UMETA(DisplayName = "Target Altitude Not Reachable"),

	TerrainCollision UMETA(DisplayName = "Terrain Collision"),
	TerrainClearance UMETA(DisplayName = "Terrain Clearance"),

	RestrictedAirspace UMETA(DisplayName = "Restricted Airspace"),
	HardBlockZone UMETA(DisplayName = "Hard Block Zone"),

	ClimbLimitExceeded UMETA(DisplayName = "Climb Limit Exceeded"),
	MaxClimbExceeded UMETA(DisplayName = "Max Climb Exceeded"),

	DescentLimitExceeded UMETA(DisplayName = "Descent Limit Exceeded"),
	MaxDescentExceeded UMETA(DisplayName = "Max Descent Exceeded"),

	TurnRadiusExceeded UMETA(DisplayName = "Turn Radius Exceeded"),
	TurnRadiusTooSmall UMETA(DisplayName = "Turn Radius Too Small"),

	NoValidNeighbors UMETA(DisplayName = "No Valid Neighbors"),

	SearchLimitReached UMETA(DisplayName = "Search Limit Reached"),
	MaxExpandedStatesReached UMETA(DisplayName = "Max Expanded States Reached"),

	Unknown UMETA(DisplayName = "Unknown")
};