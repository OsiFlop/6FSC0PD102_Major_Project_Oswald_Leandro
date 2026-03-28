// Runtime probe actor for querying baked voxel heights
// Uses grid + height cache to query max height at world position

#include "Navigation/Terrain/HeightQueryProbeActor.h"
#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "DrawDebugHelpers.h"

// Sets default values
AHeightQueryProbeActor::AHeightQueryProbeActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Convert world position (XY) into grid cell coordinates
bool AHeightQueryProbeActor::WorldXYToCell(const FVector& WorldPos, int32& OutX, int32& OutY) const
{
	// Guard: required references and baked cache available
	if (!GridBaker || !HeightCache || !HeightCache->IsValid())
		return false;

	const FVector Min = GridBaker->GridMinWorld;
	const float CellSizeCm = GridBaker->CellSizeCm;

	// Guard: valid grid cell size
	if (CellSizeCm <= 0.0f)
		return false;

	// Transform world XY into grid-local space
	const float LocalX = WorldPos.X - Min.X;
	const float LocalY = WorldPos.Y - Min.Y;

	// Convert local position into integer cell indices
	OutX = FMath::FloorToInt(LocalX / CellSizeCm);
	OutY = FMath::FloorToInt(LocalY / CellSizeCm);

	// Bounds check against baked grid size
	if (OutX < 0 || OutY < 0 || OutX >= HeightCache->GridSize.X || OutY >= HeightCache->GridSize.Y)
		return false;

	return true;
}

// Query maximum terrain height at current actor location
void AHeightQueryProbeActor::QueryHeightAtMyLocation()
{
	// Guard: missing grid or cache reference
	if (!GridBaker || !HeightCache)
	{
		UE_LOG(LogTemp, Warning, TEXT("QueryHeightAtMyLocation failed: missing GridBaker or HeightCache ref"));
		return;
	}

	// Guard: height cache not baked or invalid
	if (!HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("QueryHeightAtMyLocation failed: HeightCache invalid. Bake first."));
		return;
	}

	const FVector P = GetActorLocation();

	// Convert actor world position into grid cell
	int32 X, Y;
	if (!WorldXYToCell(P, X, Y))
	{
		UE_LOG(LogTemp, Warning, TEXT("Query: position not inside grid. World=%s"), *P.ToString());
		return;
	}

	// Resolve cell index and read cached max height (cm)
	const int32 Idx = HeightCache->ToIndex(X, Y);
	const float MaxZcm = HeightCache->MaxHeightCm[Idx];

	// Convert to ASL using calibrated sea level (cm -> m)
	const float SeaLevelCm = HeightCache->SeaLevelWorldZCm;
	const float HeightASLm = (MaxZcm - SeaLevelCm) / 100.0f;

	// Log query result (world pos, cell, height)
	UE_LOG(LogTemp, Display, TEXT("Query @ WorldXY(%.1f, %.1f) -> Cell(%d,%d) -> WorldZ=%.1f cm (%.2f m), SeaLevel=%.1f cm -> ASL=%.2f m"),
		P.X, P.Y, X, Y, MaxZcm, MaxZcm / 100.0f, SeaLevelCm, HeightASLm
	);

	// Optional debug visualization at queried height
	if (bDrawMarker && GetWorld())
	{
		const FVector MarkerLoc(P.X, P.Y, MaxZcm);
		DrawDebugSphere(GetWorld(), MarkerLoc, 200.0f, 12, FColor::Yellow, false, MarkerLifeTime, 0, 4.0f);
		DrawDebugString(GetWorld(), MarkerLoc + FVector(0,0,250.0f),
			FString::Printf(TEXT("Cell(%d,%d)\nZ=%.2fm"), X, Y, MaxZcm/100.0f),
			nullptr, FColor::White, MarkerLifeTime, false);
	}
}