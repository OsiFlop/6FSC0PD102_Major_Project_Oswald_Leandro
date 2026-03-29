// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simple3DPathfinderActor.generated.h"

class AVoxelGridBaker;
class UVoxelHeightCache;

USTRUCT(BlueprintType)
struct FGridVoxelCoord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	int32 X = 0;

	UPROPERTY(VisibleAnywhere)
	int32 Y = 0;

	UPROPERTY(VisibleAnywhere)
	int32 Z = 0;

	FGridVoxelCoord() = default;

	FGridVoxelCoord(const int32 InX, const int32 InY, const int32 InZ)
		: X(InX), Y(InY), Z(InZ)
	{
	}

	bool operator==(const FGridVoxelCoord& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}
};

FORCEINLINE uint32 GetTypeHash(const FGridVoxelCoord& Coord)
{
	uint32 Hash = GetTypeHash(Coord.X);
	Hash = HashCombine(Hash, GetTypeHash(Coord.Y));
	Hash = HashCombine(Hash, GetTypeHash(Coord.Z));
	return Hash;
}

struct FGridVoxelNodeRecord
{
	float G = TNumericLimits<float>::Max();
	float H = 0.0f;
	float F = TNumericLimits<float>::Max();

	FGridVoxelCoord Parent;
	bool bHasParent = false;
	bool bClosed = false;
};

UCLASS()
class MAJORPROJECT_API ASimple3DPathfinderActor : public AActor
{
	GENERATED_BODY()
	
public:
	ASimple3DPathfinderActor();

	// Referenzen
	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AVoxelGridBaker> GridBaker;

	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<UVoxelHeightCache> HeightCache;

	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AActor> StartActor;

	UPROPERTY(EditAnywhere, Category="References")
	TObjectPtr<AActor> GoalActor;

	// 3D Suchraum
	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="1.0"))
	float VoxelSizeZMeters = 100.0f;

	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="0"))
	int32 ExtraBottomLayers = 1;

	UPROPERTY(EditAnywhere, Category="Voxel Space", meta=(ClampMin="0"))
	int32 ExtraTopLayers = 5;

	// Debug
	UPROPERTY(EditAnywhere, Category="Debug")
	bool bDrawVisitedNodes = false;

	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugDrawLifetime = 30.0f;

	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugLineThickness = 8.0f;

	UPROPERTY(EditAnywhere, Category="Debug", meta=(ClampMin="0.0"))
	float DebugVisitedNodeSize = 20.0f;

	// Berechnete Voxelraum Daten
	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	FVector VoxelMinWorld = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	FVector VoxelMaxWorld = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, Category="Voxel Space")
	int32 ZLayerCount = 0;

	UPROPERTY(VisibleAnywhere, Category="Result")
	TArray<FVector> CurrentPathWorldPoints;

	UFUNCTION(CallInEditor, Category="Pathfinding")
	void BuildVoxelSpace();

	UFUNCTION(CallInEditor, Category="Pathfinding")
	void FindPath();

	UFUNCTION(CallInEditor, Category="Pathfinding")
	void DebugDrawCurrentPath();

private:
	bool ValidateReferences() const;

	bool WorldToVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const;
	bool WorldToNearestFreeVoxel(const FVector& WorldPos, FGridVoxelCoord& OutVoxel) const;

	FVector VoxelToWorldCenter(const FGridVoxelCoord& Voxel) const;

	bool IsVoxelInsideBounds(const FGridVoxelCoord& Voxel) const;
	bool IsVoxelBlocked(const FGridVoxelCoord& Voxel) const;

	float GetTerrainHeightCmAtCell(const int32 X, const int32 Y) const;
	float HeuristicCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const;
	float MovementCost(const FGridVoxelCoord& A, const FGridVoxelCoord& B) const;

	void GetNeighbors(const FGridVoxelCoord& Center, TArray<FGridVoxelCoord>& OutNeighbors) const;
	void ReconstructPath(
		const TMap<FGridVoxelCoord, FGridVoxelNodeRecord>& Records,
		const FGridVoxelCoord& GoalVoxel
	);
};
