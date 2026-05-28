#include "RoutePlannerManager.h"

#include "FlightPathfinderActor.h"
#include "FlightProfile.h"

ARoutePlannerManager::ARoutePlannerManager()
{
	PrimaryActorTick.bCanEverTick = false;
}

void ARoutePlannerManager::SetSelectedFlightProfile(UFlightProfile* NewProfile)
{
	SelectedFlightProfile = NewProfile;
}

UFlightProfile* ARoutePlannerManager::GetSelectedFlightProfile() const
{
	return SelectedFlightProfile;
}

FRouteCalculationResult ARoutePlannerManager::CalculateRouteFromMarkers(
	float StartAltitudeMetersASL,
	float TargetAltitudeMetersASL
)
{
	FRouteCalculationResult Result;

	if (!PathfinderActor)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::Unknown;
		Result.FailureText = FText::FromString(TEXT("Kein Pathfinder Actor zugewiesen."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	if (!StartMarker)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidStart;
		Result.FailureText = FText::FromString(TEXT("Kein Startmarker zugewiesen."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	if (!TargetMarker)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidTarget;
		Result.FailureText = FText::FromString(TEXT("Kein Zielmarker zugewiesen."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	if (!SelectedFlightProfile)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidFlightProfile;
		Result.FailureText = FText::FromString(TEXT("Kein Flugzeugtyp ausgewählt."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	Result = PathfinderActor->CalculateFlightRouteForUI(
		StartMarker->GetActorLocation(),
		TargetMarker->GetActorLocation(),
		StartAltitudeMetersASL,
		TargetAltitudeMetersASL,
		SelectedFlightProfile
	);

	LastResult = Result;

	if (Result.bSuccess)
	{
		CurrentRoutePoints = Result.RoutePoints;
	}
	else
	{
		CurrentRoutePoints.Empty();
	}

	return Result;
}

void ARoutePlannerManager::ClearCurrentRoute()
{
	CurrentRoutePoints.Empty();
	LastResult = FRouteCalculationResult();
}