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
	CurrentRoutePoints.Empty();
	LastResult = FRouteCalculationResult();

	if (PathfinderActor)
	{
		PathfinderActor->FlightProfile = NewProfile;
		PathfinderActor->ClearCurrentRoute();
	}

	if (NewProfile)
	{
		UE_LOG(LogTemp, Display,
			TEXT("RoutePlanner: Flugzeugprofil gewechselt: %s | Asset=%s | MaxAlt=%.1fm | Clearance=%.1fm | Speed=%.1fm/s | Climb=%.1fm/s | Descent=%.1fm/s | TurnRadius=%.1fm"),
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
	CurrentRoutePoints.Empty();
	LastResult = Result;

	if (PathfinderActor)
	{
		PathfinderActor->ClearCurrentRoute();
	}

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

	UFlightProfile* ProfileToUse = SelectedFlightProfile;
	if (!ProfileToUse && PathfinderActor->FlightProfile)
	{
		ProfileToUse = PathfinderActor->FlightProfile;
	}

	if (!ProfileToUse && AvailableFlightProfiles.Num() > 0)
	{
		ProfileToUse = AvailableFlightProfiles[0];
	}

	if (!ProfileToUse)
	{
		Result.bSuccess = false;
		Result.FailureReason = ERouteFailureReason::InvalidFlightProfile;
		Result.FailureText = FText::FromString(TEXT("Kein Flugzeugtyp ausgewaehlt."));
		LastResult = Result;
		CurrentRoutePoints.Empty();
		return Result;
	}

	SelectedFlightProfile = ProfileToUse;
	PathfinderActor->FlightProfile = ProfileToUse;

	UE_LOG(LogTemp, Display,
		TEXT("RoutePlanner: Berechne Route mit Profil: %s | Asset=%s | MaxAlt=%.1fm | Clearance=%.1fm | Speed=%.1fm/s | Climb=%.1fm/s | Descent=%.1fm/s | TurnRadius=%.1fm"),
		*ProfileToUse->AircraftName,
		*ProfileToUse->GetPathName(),
		ProfileToUse->MaxAltitudeMetersASL,
		ProfileToUse->MinimumTerrainClearanceMeters,
		ProfileToUse->CruiseSpeedMetersPerSecond,
		ProfileToUse->MaxClimbRateMetersPerSecond,
		ProfileToUse->MaxDescentRateMetersPerSecond,
		ProfileToUse->MinimumTurnRadiusMeters
	);

	Result = PathfinderActor->CalculateFlightRouteForUI(
		StartMarker->GetActorLocation(),
		TargetMarker->GetActorLocation(),
		StartAltitudeMetersASL,
		TargetAltitudeMetersASL,
		ProfileToUse
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

void ARoutePlannerManager::SetStartMarkerWorldXY(FVector2D WorldXY)
{
	if (!StartMarker)
	{
		return;
	}

	FVector NewLocation = StartMarker->GetActorLocation();
	NewLocation.X = WorldXY.X;
	NewLocation.Y = WorldXY.Y;

	StartMarker->SetActorLocation(NewLocation);
}

void ARoutePlannerManager::SetTargetMarkerWorldXY(FVector2D WorldXY)
{
	if (!TargetMarker)
	{
		return;
	}

	FVector NewLocation = TargetMarker->GetActorLocation();
	NewLocation.X = WorldXY.X;
	NewLocation.Y = WorldXY.Y;

	TargetMarker->SetActorLocation(NewLocation);
}

FVector ARoutePlannerManager::GetStartMarkerLocation() const
{
	if (!StartMarker)
	{
		return FVector::ZeroVector;
	}

	return StartMarker->GetActorLocation();
}

FVector ARoutePlannerManager::GetTargetMarkerLocation() const
{
	if (!TargetMarker)
	{
		return FVector::ZeroVector;
	}

	return TargetMarker->GetActorLocation();
}