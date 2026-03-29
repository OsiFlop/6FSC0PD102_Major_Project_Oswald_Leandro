// Core grid and baking actor for voxel-based terrain analysis
// Builds grid over landscape, bakes max heights, and provides debug previews

#include "Navigation/Grid/VoxelGridBaker.h"
#include "Navigation/Terrain/TerrainReferenceActor.h"
#include "Navigation/Grid/VoxelGridConfig.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "LandscapeProxy.h"
#include "Navigation/Grid/VoxelHeightCache.h"
#include "Engine/StaticMesh.h"
#include "Navigation/Terrain/HeightQueryProbeActor.h"

// Sets default values
AVoxelGridBaker::AVoxelGridBaker()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Validate grid setup values
bool AVoxelGridBaker::IsGridValid() const
{
	// Basic sanity checks
	return (GridSize.X > 0 && GridSize.Y > 0 && CellSizeCm > 0.0f);
}

// Get world-space XY bounds for a single cell
void AVoxelGridBaker::GetCellMinMaxXY(const int32 X, const int32 Y, FVector2D& OutMin, FVector2D& OutMax) const
{
	// Cell corner from grid origin + cell index * size
	const float MinX = GridMinWorld.X + (float)X * CellSizeCm;
	const float MinY = GridMinWorld.Y + (float)Y * CellSizeCm;
	const float MaxX = MinX + CellSizeCm;
	const float MaxY = MinY + CellSizeCm;

	OutMin = FVector2D(MinX, MinY);
	OutMax = FVector2D(MaxX, MaxY);
}

// Build grid extents and resolution from landscape bounds
void AVoxelGridBaker::BuildGrid()
{
	// Guard: required refs
	if (!TerrainRef || !GridConfig)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildGrid failed: TerrainRef or GridConfig missing"));
		return;
	}

	// Read landscape bounds
	FVector LMin, LMax;
	if (!TerrainRef->GetLandscapeWorldBounds(LMin, LMax))
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildGrid failed: Landscape bounds not available"));
		return;
	}

	// Config meters -> cm
	CellSizeCm = GridConfig->CellSizeMeters * 100.0f;
	if (CellSizeCm <= 0.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildGrid failed: CellSizeMeters must be > 0"));
		return;
	}

	// Optional padding around landscape bounds
	const float PaddingCm = GridConfig->PaddingMeters * 100.0f;

	// Grid XY covers padded landscape area
	GridMinWorld = FVector(LMin.X - PaddingCm, LMin.Y - PaddingCm, 0.0f);
	GridMaxWorld = FVector(LMax.X + PaddingCm, LMax.Y + PaddingCm, 0.0f);

	const float WidthCm  = GridMaxWorld.X - GridMinWorld.X;
	const float HeightCm = GridMaxWorld.Y - GridMinWorld.Y;

	// Resolution: ceil to fully cover bounds
	GridSize.X = FMath::Max(1, FMath::CeilToInt(WidthCm  / CellSizeCm));
	GridSize.Y = FMath::Max(1, FMath::CeilToInt(HeightCm / CellSizeCm));

	const int32 TotalCells = GridSize.X * GridSize.Y;

	UE_LOG(LogTemp, Display, TEXT("Grid Built: Size=%d x %d (Cells=%d), CellSize=%.2fm"),
		GridSize.X, GridSize.Y, TotalCells, GridConfig->CellSizeMeters);
}

// Draw the grid outer border as debug lines
void AVoxelGridBaker::DebugDrawGridOutline()
{
	// Guard: world + config + grid
	if (!GetWorld() || !GridConfig || !IsGridValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("DebugDrawGridOutline failed: grid not built or config missing"));
		return;
	}

	// Outline corners in world space
	const float Z = 0.0f;
	const FVector A(GridMinWorld.X, GridMinWorld.Y, Z);
	const FVector B(GridMaxWorld.X, GridMinWorld.Y, Z);
	const FVector C(GridMaxWorld.X, GridMaxWorld.Y, Z);
	const FVector D(GridMinWorld.X, GridMaxWorld.Y, Z);

	DrawDebugLine(GetWorld(), A, B, FColor::Cyan, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
	DrawDebugLine(GetWorld(), B, C, FColor::Cyan, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
	DrawDebugLine(GetWorld(), C, D, FColor::Cyan, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
	DrawDebugLine(GetWorld(), D, A, FColor::Cyan, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
}

// Draw a limited number of cells for quick debug checks
void AVoxelGridBaker::DebugDrawSomeCells(int32 MaxCellsToDraw)
{
	// Guard: world + config + grid
	if (!GetWorld() || !GridConfig || !IsGridValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("DebugDrawSomeCells failed: grid not built or config missing"));
		return;
	}

	// Clamp draw count
	MaxCellsToDraw = FMath::Max(0, MaxCellsToDraw);

	const int32 TotalCells = GridSize.X * GridSize.Y;
	const int32 Count = FMath::Min(MaxCellsToDraw, TotalCells);

	const float Z = 0.0f;

	// Draw first N cells in row-major order
	for (int32 i = 0; i < Count; ++i)
	{
		const int32 X = i % GridSize.X;
		const int32 Y = i / GridSize.X;

		FVector2D MinXY, MaxXY;
		GetCellMinMaxXY(X, Y, MinXY, MaxXY);

		const FVector A(MinXY.X, MinXY.Y, Z);
		const FVector B(MaxXY.X, MinXY.Y, Z);
		const FVector C(MaxXY.X, MaxXY.Y, Z);
		const FVector D(MinXY.X, MaxXY.Y, Z);

		DrawDebugLine(GetWorld(), A, B, FColor::Green, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
		DrawDebugLine(GetWorld(), B, C, FColor::Green, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
		DrawDebugLine(GetWorld(), C, D, FColor::Green, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
		DrawDebugLine(GetWorld(), D, A, FColor::Green, false, GridConfig->DebugDrawLifetime, 0, GridConfig->DebugLineThickness);
	}
}


// Convenience: draw a preset number of cells
void AVoxelGridBaker::DebugDrawSomeCells50()
{
	DebugDrawSomeCells(1000);
}

// Bake per-cell maximum Z by sampling line traces inside each cell
void AVoxelGridBaker::BakeMaxHeights()
{
	// Guard: required refs
	if (!GetWorld() || !TerrainRef || !GridConfig || !HeightCache)
	{
		UE_LOG(LogTemp, Warning, TEXT("BakeMaxHeights failed: missing references (World/TerrainRef/GridConfig/HeightCache)"));
		return;
	}

	// Guard: grid built first
	if (!IsGridValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("BakeMaxHeights failed: Grid not built. Click BuildGrid first."));
		return;
	}

	// Guard: landscape ref
	ALandscapeProxy* Landscape = TerrainRef->Landscape;
	if (!Landscape)
	{
		UE_LOG(LogTemp, Warning, TEXT("BakeMaxHeights failed: TerrainRef has no Landscape set."));
		return;
	}

	// Init cache metadata + storage
	HeightCache->GridMinWorld = GridMinWorld;
	HeightCache->CellSizeCm = CellSizeCm;
	HeightCache->Allocate(GridSize.X, GridSize.Y);

	// Sampling density per cell
	const int32 SamplesPerAxis = FMath::Max(1, GridConfig->SamplesPerAxis);
	const int32 SamplesTotal = SamplesPerAxis * SamplesPerAxis;

	// Trace range (meters -> cm)
	const float TraceStartCm = GridConfig->TraceStartAboveMeters * 100.0f;
	const float TraceEndCm   = GridConfig->TraceEndBelowMeters * 100.0f;

	// Trace params: ignore baker actor
	FCollisionQueryParams Params(SCENE_QUERY_STAT(VoxelBakeTrace), true);
	Params.bReturnPhysicalMaterial = false;
	Params.AddIgnoredActor(this);

	int32 HitCount = 0;
	const int32 TotalCells = GridSize.X * GridSize.Y;

	// Iterate over all grid cells
	for (int32 Y = 0; Y < GridSize.Y; ++Y)
	{
		for (int32 X = 0; X < GridSize.X; ++X)
		{
			float MaxZ = -FLT_MAX;

			// Cell origin in world space
			const float CellMinX = GridMinWorld.X + (float)X * CellSizeCm;
			const float CellMinY = GridMinWorld.Y + (float)Y * CellSizeCm;

			// Sub-sample points inside cell (center of sub-cells)
			for (int32 sy = 0; sy < SamplesPerAxis; ++sy)
			{
				for (int32 sx = 0; sx < SamplesPerAxis; ++sx)
				{
					const float U = ((float)sx + 0.5f) / (float)SamplesPerAxis; // 0..1
					const float V = ((float)sy + 0.5f) / (float)SamplesPerAxis;

					const float SampleX = CellMinX + U * CellSizeCm;
					const float SampleY = CellMinY + V * CellSizeCm;

					// Trace vertical line: above -> below landscape
					const FVector Start(SampleX, SampleY, Landscape->GetActorLocation().Z + TraceStartCm);
					const FVector End  (SampleX, SampleY, Landscape->GetActorLocation().Z - TraceEndCm);

					FHitResult Hit;
					const bool bHit = GetWorld()->LineTraceSingleByChannel(
						Hit,
						Start,
						End,
						GridConfig->TraceChannel,
						Params
					);

					// Track highest hit inside this cell
					if (bHit)
					{
						HitCount++;
						MaxZ = FMath::Max(MaxZ, Hit.Location.Z);
					}
				}
			}

			// Store max height (world Z in cm) for this cell
			const int32 Idx = HeightCache->ToIndex(X, Y);
			HeightCache->MaxHeightCm[Idx] = MaxZ;
		}
	}

	// Mark asset dirty in editor (save changes)
#if WITH_EDITOR
	HeightCache->Modify();
#endif

	UE_LOG(LogTemp, Display, TEXT("Bake complete. Cells=%d, SamplesPerCell=%d, TotalTraces=%d, Hits=%d"),
		TotalCells, SamplesTotal, TotalCells * SamplesTotal, HitCount);
}

// Choose preview base Z (sea level or manual override)
float AVoxelGridBaker::GetPreviewBaseZCm() const
{
	// Optional: use cached sea level as base reference
	if (bPreviewUseSeaLevelAsBase && HeightCache)
	{
		return HeightCache->SeaLevelWorldZCm;
	}
	return PreviewBaseZOverrideCm;
}

// Destroy existing preview voxel instances
void AVoxelGridBaker::ClearPreviewVoxels()
{
	// Remove and destroy preview component
	if (PreviewHISM)
	{
		PreviewHISM->ClearInstances();
		PreviewHISM->DestroyComponent();
		PreviewHISM = nullptr;
	}
	UE_LOG(LogTemp, Display, TEXT("Preview voxels cleared."));
}

// Build voxel column preview around a center point
void AVoxelGridBaker::BuildPreviewVoxels()
{
	// Guard: baked cache required
	if (!GetWorld() || !HeightCache || !HeightCache->IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildPreviewVoxels: HeightCache invalid. Bake first."));
		return;
	}

	// Guard: grid required
	if (CellSizeCm <= 0.0f || GridSize.X <= 0 || GridSize.Y <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildPreviewVoxels: Grid invalid. BuildGrid first."));
		return;
	}

	// Reset preview instances
	ClearPreviewVoxels();

	// Create HISM for efficient instancing
	PreviewHISM = NewObject<UHierarchicalInstancedStaticMeshComponent>(this, TEXT("PreviewHISM"));
	PreviewHISM->SetupAttachment(GetRootComponent());
	PreviewHISM->RegisterComponent();

	// Load engine cube mesh (100cm base size)
	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!CubeMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("BuildPreviewVoxels: Could not load Cube mesh."));
		return;
	}
	PreviewHISM->SetStaticMesh(CubeMesh);
	PreviewHISM->SetMobility(EComponentMobility::Movable);

	if (PreviewMaterial)
	{
		PreviewHISM->SetMaterial(0, PreviewMaterial);
	}

	// Choose preview center (actor location or grid center)
	FVector CenterWorld = (GridMinWorld + GridMaxWorld) * 0.5f;
	if (PreviewCenterActor)
	{
		CenterWorld = PreviewCenterActor->GetActorLocation();
	}

	// Convert center world position to cell indices
	const int32 CenterX = FMath::FloorToInt((CenterWorld.X - GridMinWorld.X) / CellSizeCm);
	const int32 CenterY = FMath::FloorToInt((CenterWorld.Y - GridMinWorld.Y) / CellSizeCm);

	// Clamp preview area radius
	const int32 R = FMath::Max(1, PreviewRadiusCells);

	const int32 MinX = FMath::Clamp(CenterX - R, 0, GridSize.X - 1);
	const int32 MaxX = FMath::Clamp(CenterX + R, 0, GridSize.X - 1);
	const int32 MinY = FMath::Clamp(CenterY - R, 0, GridSize.Y - 1);
	const int32 MaxY = FMath::Clamp(CenterY + R, 0, GridSize.Y - 1);

	// Base reference height for columns
	const float BaseZcm = GetPreviewBaseZCm();

	// Cube is 100cm: scale cell width relative to that
	const float CellScaleXY = CellSizeCm / 100.0f;

	int32 Added = 0;

	// Spawn one cube column per cell in preview area
	for (int32 Y = MinY; Y <= MaxY; ++Y)
	{
		for (int32 X = MinX; X <= MaxX; ++X)
		{
			const int32 Idx = HeightCache->ToIndex(X, Y);
			if (!HeightCache->MaxHeightCm.IsValidIndex(Idx))
				continue;

			const float MaxZcm = HeightCache->MaxHeightCm[Idx];

			// Skip uninitialized cells
			if (MaxZcm <= -1e20f)
				continue;

			// Column height from base to max cell height
			float ColumnHeightCm = MaxZcm - BaseZcm;
			if (ColumnHeightCm < 10.0f) ColumnHeightCm = 10.0f;

			// Cube Z scale based on 100cm mesh height
			const float ScaleZ = ColumnHeightCm / 100.0f;

			// Cell center position in world space
			const float CellCenterX = GridMinWorld.X + (X + 0.5f) * CellSizeCm;
			const float CellCenterY = GridMinWorld.Y + (Y + 0.5f) * CellSizeCm;

			// Cube center Z at half column height
			const float CubeCenterZ = BaseZcm + ColumnHeightCm * 0.5f;

			const FVector Location(CellCenterX, CellCenterY, CubeCenterZ);
			const FVector Scale(CellScaleXY, CellScaleXY, ScaleZ);

			FTransform T;
			T.SetLocation(Location);
			T.SetRotation(FQuat::Identity);
			T.SetScale3D(Scale);

			PreviewHISM->AddInstance(T);
			Added++;
		}
	}

	UE_LOG(LogTemp, Display, TEXT("Preview voxels built. Instances=%d, Area=%dx%d cells"),
		Added, (MaxX - MinX + 1), (MaxY - MinY + 1));
}