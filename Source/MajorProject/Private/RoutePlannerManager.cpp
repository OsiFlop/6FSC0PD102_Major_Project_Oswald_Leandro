// Route planner manager actor
// Connects UI route requests with aircraft-aware pathfinder
// Stores selected flight profile, route markers and latest result
// Handles marker movement from map input
// Keeps UI route data in sync with pathfinder output

#include "RoutePlannerManager.h"

#include "FlightPathfinderActor.h"
#include "FlightProfile.h"

ARoutePlannerManager::ARoutePlannerManager()
{
	// No runtime tick needed
	PrimaryActorTick.bCanEverTick = false;
}

// Set active flight profile from UI
void ARoutePlannerManager::SetSelectedFlightProfile(UFlightProfile* NewProfile)
{
	// SelectedFlightProfile: profile used for next route calculation
	SelectedFlightProfile = NewProfile;

	// Clear old result after profile change
	CurrentRoutePoints.Empty();
	LastResult = FRouteCalculationResult();

	if (PathfinderActor)
	{
		// Pass selected profile to pathfinder
		PathfinderActor->FlightProfile = NewProfile;
		
		// Remove old route from pathfinder/debug
		PathfinderActor->ClearCurrentRoute();
	}

	if (NewProfile)
	{
		// Debug log: show selected aircraft values
		UE_LOG(LogTemp, Display,
			TEXT("RoutePlanner: Aircraft profile changed: %s | Asset=%s | MaxAlt=%.1fm | Clearance=%.1fm | Speed=%.1fm/s | Climb=%.1fm/s | Descent=%.1fm/s | TurnRadius=%.1fm"),
			*NewProfile->AircraftName,
			*NewProfile->GetPathName(),
			NewProfile->MaxAltitudeMetersASL,
			NewProfile->MinimumTerrainClearanceMeters,
			NewProfile->CruiseSpeedMetersPerSecond,
			NewProfile->MaxClimbRateMetersPerSecond,
			NewProfile->MaxDescentRateMetersPerSecond,
			NewProfile->MinimumTurnRadiusMeters
		);
	}
}

// Calculate route from current markers and UI altitude values
FRouteCalculationResult ARoutePlannerManager::CalculateRouteFromMarkers(
	float StartAltitudeMetersASL,
	float TargetAltitudeMetersASL
)
{
	// Result: local calculation result, returned to UI
	FRouteCalculationResult Result;

	// Reset previous route before starting new calculation
	CurrentRoutePoints.Empty();
	LastResult = Result;

	if (PathfinderActor)
	{
		// Clear old pathfinder route/debug before recalculation
		PathfinderActor->ClearCurrentRoute();
	}

	if (!PathfinderActor)
	{
		// Missing pathfinder -> route calculation impossible
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::Unknown;
		Result.FailureText = FText::FromString(TEXT("No Pathfinder Actor assigned."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	if (!StartMarker)
	{
		// Missing start marker from UI/map
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidStart;
		Result.FailureText = FText::FromString(TEXT("No start marker assigned."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	if (!TargetMarker)
	{
		// Missing target marker from UI/map
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidTarget;
		Result.FailureText = FText::FromString(TEXT("No target marker assigned."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	// ProfileToUse: selected profile, pathfinder fallback, then first available profile
	UFlightProfile* ProfileToUse = SelectedFlightProfile;
	if (!ProfileToUse && PathfinderActor->FlightProfile)
	{
		// Fallback: use profile already assigned on pathfinder
		ProfileToUse = PathfinderActor->FlightProfile;
	}

	if (!ProfileToUse && AvailableFlightProfiles.Num() > 0)
	{
		// Fallback: use first available profile from manager list
		ProfileToUse = AvailableFlightProfiles[0];
	}

	if (!ProfileToUse)
	{
		// No usable aircraft profile found
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidFlightProfile;
		Result.FailureText = FText::FromString(TEXT("No aircraft type selected."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	// Sync final profile choice and route options with manager and pathfinder
	SelectedFlightProfile = ProfileToUse;
	PathfinderActor->FlightProfile = ProfileToUse;
	PathfinderActor->bUseWeatherZones = bShowWeather;

	// Debug log: profile values used for this calculation
	UE_LOG(LogTemp, Display,
		TEXT("RoutePlanner: Calculate route with profile: %s | Asset=%s | MaxAlt=%.1fm | Clearance=%.1fm | Speed=%.1fm/s | Climb=%.1fm/s | Descent=%.1fm/s | TurnRadius=%.1fm"),
		*ProfileToUse->AircraftName,
		*ProfileToUse->GetPathName(),
		ProfileToUse->MaxAltitudeMetersASL,
		ProfileToUse->MinimumTerrainClearanceMeters,
		ProfileToUse->CruiseSpeedMetersPerSecond,
		ProfileToUse->MaxClimbRateMetersPerSecond,
		ProfileToUse->MaxDescentRateMetersPerSecond,
		ProfileToUse->MinimumTurnRadiusMeters
	);

	// Start pathfinder route calculation with marker positions and UI altitudes
	Result = PathfinderActor->CalculateFlightRouteForUI(
		StartMarker->GetActorLocation(),
		TargetMarker->GetActorLocation(),
		StartAltitudeMetersASL,
		TargetAltitudeMetersASL,
		ProfileToUse
	);

	// Store latest result for UI feedback
	LastResult = Result;

	if (Result.bSuccess)
	{
		// Route successful -> cache points for UI drawing
		CurrentRoutePoints = Result.RoutePoints;
	}
	else
	{
		// Route failed -> keep route point list empty
		CurrentRoutePoints.Empty();
	}

	return Result;
}

// Clear current route result in manager
void ARoutePlannerManager::ClearCurrentRoute()
{
	// Remove cached UI route
	CurrentRoutePoints.Empty();
	
	// Reset status / failure / route result data
	LastResult = FRouteCalculationResult();
}

// Move start marker to selected world XY position
void ARoutePlannerManager::SetStartMarkerWorldXY(FVector2D WorldXY)
{
	if (!StartMarker)
	{
		return;
	}

	// NewLocation: keep old altitude, replace map XY
	FVector NewLocation = StartMarker->GetActorLocation();
	NewLocation.X = WorldXY.X;
	NewLocation.Y = WorldXY.Y;

	// Apply new start marker position in level
	StartMarker->SetActorLocation(NewLocation);
}

// Move target marker to selected world XY position
void ARoutePlannerManager::SetTargetMarkerWorldXY(FVector2D WorldXY)
{
	if (!TargetMarker)
	{
		return;
	}

	// NewLocation: keep old altitude, replace map XY
	FVector NewLocation = TargetMarker->GetActorLocation();
	NewLocation.X = WorldXY.X;
	NewLocation.Y = WorldXY.Y;

	// Apply new target marker position in level
	TargetMarker->SetActorLocation(NewLocation);
}

// Get current start marker world location
FVector ARoutePlannerManager::GetStartMarkerLocation() const
{
	if (!StartMarker)
	{
		// Safe fallback for UI
		return FVector::ZeroVector;
	}

	return StartMarker->GetActorLocation();
}

// Get current target marker world location
FVector ARoutePlannerManager::GetTargetMarkerLocation() const
{
	if (!TargetMarker)
	{
		// Safe fallback for UI
		return FVector::ZeroVector;
	}

	return TargetMarker->GetActorLocation();
}