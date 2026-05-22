// Flight influence zone actor
// Defines 3D zones for route restrictions or extra traversal cost
// Used by pathfinding to avoid or penalize specific airspace areas

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlightInfluenceZoneActor.generated.h"

class UBoxComponent;

UCLASS()
class MAJORPROJECT_API AFlightInfluenceZoneActor : public AActor
{
	GENERATED_BODY()
	
public:
	AFlightInfluenceZoneActor();

	// ZoneBox: visible 3D volume of this influence zone
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Zone")
	TObjectPtr<UBoxComponent> ZoneBox;

	// bHardBlock: true = route cannot pass through this zone
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	bool bHardBlock = true;

	// AdditionalTraversalCost: extra A* cost when passing through soft zone
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone", meta=(ClampMin="0.0"))
	float AdditionalTraversalCost = 0.0f;

	// MinAltitudeMetersASL: lower altitude limit for active zone
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	float MinAltitudeMetersASL = 0.0f;

	// MaxAltitudeMetersASL: upper altitude limit, -1 = no upper limit
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Zone")
	float MaxAltitudeMetersASL = -1.0f;

	// Check if point is inside zone bounds and altitude range
	bool ContainsPoint(const FVector& WorldPoint, float PointAltitudeMetersASL) const;

	// Check if segment crosses zone, sampled step by step
	bool IntersectsSegmentBySampling(
		const FVector& From,
		const FVector& To,
		float FromAltitudeMetersASL,
		float ToAltitudeMetersASL,
		float SampleStepCm = 500.0f
	) const;
};