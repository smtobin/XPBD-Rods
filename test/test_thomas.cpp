#include <Eigen/Dense>

#include "solver/BlockThomasSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 100;
constexpr static int BlockSize = 6;
constexpr static int NumBlocks = 50;
using SolverType = Solver::FixedSizeSymmetricBlockThomasSolver<NumBlocks*BlockSize, BlockSize>;

SolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename SolverType::BlockMatType mat = SolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += SolverType::BlockSize * SolverType::BlockMatType::Identity();

    return mat;
}

int main() 
{
    

    typename SolverType::DiagBlockArrayType A_diag;
    typename SolverType::OffDiagBlockArrayType A_off_diag;
    typename SolverType::TotalVecType b, x;

    // populate matrix
    for (auto& mat : A_diag)
    {
        mat = generateRandomSPDBlockMat();
    }

    for (auto& mat : A_off_diag)
    {
        mat = SolverType::BlockMatType::Random();
    }

    b = SolverType::TotalVecType::Random();
    
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        SolverType::solve(A_diag, A_off_diag, b, x);
    auto t2 = std::chrono::high_resolution_clock::now();
    const double ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms / NumIters << " ms" << std::endl;

    // evaluate solver accuracy
    typename SolverType::TotalVecType test;
    test.template block<BlockSize,1>(0,0) = 
        A_diag[0] * x.template block<BlockSize,1>(0,0) + 
        A_off_diag[0].transpose() * x.template block<BlockSize,1>(BlockSize,0);

    test.template block<BlockSize,1>(BlockSize*(NumBlocks-1),0) = 
        A_diag[NumBlocks-1] * x.template block<BlockSize,1>(BlockSize*(NumBlocks-1),0) + 
        A_off_diag[NumBlocks-2] * x.template block<BlockSize,1>(BlockSize*(NumBlocks-2),0);

    for (int i = 1; i < SolverType::NumBlocks-1; i++)
    {
        test.template block<BlockSize,1>(BlockSize*i,0) =
            A_off_diag[i-1] * x.template block<BlockSize,1>(BlockSize*(i-1),0) +
            A_diag[i] * x.template block<BlockSize,1>(BlockSize*i,0) +
            A_off_diag[i].transpose() * x.template block<BlockSize,1>(BlockSize*(i+1),0);
    }
    const double norm = (test - b).norm();

    std::cout << "Residual: " << norm << std::endl;
}