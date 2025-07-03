#include <Eigen/Dense>

#include "solver/BlockThomasSolver.hpp"
#include "solver/BlockBandedSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 1;
constexpr static int BlockSize = 6;
constexpr static int NumBlocks = 30;
using ThomasSolverType = Solver::SymmetricBlockThomasSolver<BlockSize>;
using BandedSolverType = Solver::SymmetricBlockBandedSolver<BlockSize>;

ThomasSolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename ThomasSolverType::BlockMatType mat = ThomasSolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += ThomasSolverType::BlockSize * ThomasSolverType::BlockMatType::Identity();

    return mat;
}

int main() 
{
    std::cout << "=== THOMAS SOLVER ===" << std::endl;
    // "Dynamic" solver

    std::vector<ThomasSolverType::BlockMatType> A_diag_vec(NumBlocks);
    std::vector<ThomasSolverType::BlockMatType> A_off_diag_vec(NumBlocks-1);
    VecXr b = VecXr::Random(NumBlocks*BlockSize);
    VecXr x2(NumBlocks*BlockSize);
    for (int i = 0; i < NumBlocks; i++)
        A_diag_vec[i] = generateRandomSPDBlockMat();
    for (int i = 0; i < NumBlocks-1; i++)
        A_off_diag_vec[i] = ThomasSolverType::BlockMatType::Random();

    ThomasSolverType solver(NumBlocks);
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        solver.solve(A_diag_vec, A_off_diag_vec, b, x2);
    auto t4 = std::chrono::high_resolution_clock::now();
    const double ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() / 1.0e6;
    std::cout << "[Non-fixed size] Elapsed solve time (" <<  NumIters << " iterations): " << ms2 << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms2 / NumIters << " ms" << std::endl;

    // evaluate solver accuracy
    VecXr test(NumBlocks*BlockSize);
    test.block<BlockSize,1>(0,0) = 
        A_diag_vec[0] * x2.block<BlockSize,1>(0,0) + 
        A_off_diag_vec[0].transpose() * x2.block<BlockSize,1>(BlockSize,0);

    test.block<BlockSize,1>(BlockSize*(NumBlocks-1),0) = 
        A_diag_vec[NumBlocks-1] * x2.block<BlockSize,1>(BlockSize*(NumBlocks-1),0) + 
        A_off_diag_vec[NumBlocks-2] * x2.block<BlockSize,1>(BlockSize*(NumBlocks-2),0);

    for (int i = 1; i < NumBlocks-1; i++)
    {
        test.block<BlockSize,1>(BlockSize*i,0) =
            A_off_diag_vec[i-1] * x2.block<BlockSize,1>(BlockSize*(i-1),0) +
            A_diag_vec[i] * x2.block<BlockSize,1>(BlockSize*i,0) +
            A_off_diag_vec[i].transpose() * x2.block<BlockSize,1>(BlockSize*(i+1),0);
    }
    const double norm = (test - b).norm();

    std::cout << "Residual: " << norm << std::endl;

    std::cout << "=== BANDED SOLVER ===" << std::endl;
    std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals(2);
    diagonals[0].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
    diagonals[1].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
    for (int i = 0; i < NumBlocks; i++)
    {
        diagonals[0][i] = A_diag_vec[i];
    }
    for (int i = 0; i < NumBlocks-1; i++)
    {
        diagonals[1][i] = A_off_diag_vec[i];
    }

    VecXr x3(NumBlocks*BlockSize);
    BandedSolverType banded_solver(1, NumBlocks);
    banded_solver.solve(diagonals, b, x3);

    std::cout << "Residual: " << (x2 - x3).norm() << std::endl;
}