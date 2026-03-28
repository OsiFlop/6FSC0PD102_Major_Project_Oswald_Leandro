// Helper actor providing access to landscape reference and world bounds
// Used as a shared terrain source for grid and height queries

#include "Navigation/Terrain/TerrainReferenceActor.h"
#include "Landscape.h"
#include "LandscapeProxy.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "DrawDebugHelpers.h"

// Sets default values
ATerrainReferenceActor::ATerrainReferenceActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

}

// Automatically find first landscape in the world
void ATerrainReferenceActor::AutoFindLandscape()
{
	// Guard: valid world
	if (!GetWorld()) return;

	// Find all landscape proxy actors
	TArray<AActor*> Found;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ALandscapeProxy::StaticClass(), Found);

	// Use first found landscape
	if (Found.Num() > 0)
	{
		Landscape = Cast<ALandscapeProxy>(Found[0]);
	}
}

// Get landscape world-space bounding box (Axis-Aligned Bounding Box)
bool ATerrainReferenceActor::GetLandscapeWorldBounds(FVector& OutMin, FVector& OutMax) const
{
	// Guard: landscape reference
	if (!Landscape) return false;

	// Read bounds from landscape components
	const FBox Bounds = Landscape->GetComponentsBoundingBox(true);
	OutMin = Bounds.Min;
	OutMax = Bounds.Max;
	return true;
}

// Check if world position lies inside landscape bounds
bool ATerrainReferenceActor::IsWorldPosInsideLandscapeBounds(const FVector& WorldPos) const
{
	FVector Min, Max;

	// Guard: valid bounds
	if (!GetLandscapeWorldBounds(Min, Max)) return false;

	// XY bounds test (Axis-Aligned Bounding Box)
	return (WorldPos.X >= Min.X && WorldPos.X <= Max.X &&
			WorldPos.Y >= Min.Y && WorldPos.Y <= Max.Y);
}

// Debug print and visualize landscape bounds
void ATerrainReferenceActor::DebugPrintLandscapeBounds()
{
	// Guard: valid world
	if (!GetWorld())
	{
		UE_LOG(LogTemp, Warning, TEXT("World is null"));
		return;
	}

	FVector Min, Max;

	// Guard: valid landscape
	if (!GetLandscapeWorldBounds(Min, Max))
	{
		UE_LOG(LogTemp, Warning, TEXT("Landscape not valid"));
		return;
	}

	// Log bounds values
	UE_LOG(LogTemp, Display, TEXT("Landscape Bounds Min: %s"), *Min.ToString());
	UE_LOG(LogTemp, Display, TEXT("Landscape Bounds Max: %s"), *Max.ToString());

	// Debug draw: bounding box in viewport
	DrawDebugBox(
		GetWorld(),
		(Min + Max) * 0.5f,     // Center
		(Max - Min) * 0.5f,     // Extent
		FColor::Green,
		false,                  // Not persistent
		10.0f,                  // Lifetime 
		0,
		50.0f                   // Thickness
	);
}