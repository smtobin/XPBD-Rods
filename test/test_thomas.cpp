#include <Eigen/Dense>

#include "solver/BlockThomasSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 100;
constexpr static int BlockSize = 6;
constexpr static int NumBlocks = 50;
using FixedSizeSolverType = Solver::FixedSizeSymmetricBlockThomasSolver<NumBlocks*BlockSize, BlockSize>;
using SolverType = Solver::SymmetricBlockThomasSolver<BlockSize>;

FixedSizeSolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename FixedSizeSolverType::BlockMatType mat = FixedSizeSolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += FixedSizeSolverType::BlockSize * FixedSizeSolverType::BlockMatType::Identity();

    return mat;
}

int main() 
{
    

    typename FixedSizeSolverType::DiagBlockArrayType A_diag;
    typename FixedSizeSolverType::OffDiagBlockArrayType A_off_diag;
    typename FixedSizeSolverType::TotalVecType b, x1;


    // populate matrix
    for (auto& mat : A_diag)
    {
        mat = generateRandomSPDBlockMat();
    }

    for (auto& mat : A_off_diag)
    {
        mat = FixedSizeSolverType::BlockMatType::Random();
    }

    b = FixedSizeSolverType::TotalVecType::Random();
    
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        FixedSizeSolverType::solve(A_diag, A_off_diag, b, x1);
    auto t2 = std::chrono::high_resolution_clock::now();
    const double ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    std::cout << "[Fixed size] Elapsed solve time (" <<  NumIters << " iterations): " << ms << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms / NumIters << " ms" << std::endl;

    // evaluate solver accuracy
    typename FixedSizeSolverType::TotalVecType test;
    test.template block<BlockSize,1>(0,0) = 
        A_diag[0] * x1.template block<BlockSize,1>(0,0) + 
        A_off_diag[0].transpose() * x1.template block<BlockSize,1>(BlockSize,0);

    test.template block<BlockSize,1>(BlockSize*(NumBlocks-1),0) = 
        A_diag[NumBlocks-1] * x1.template block<BlockSize,1>(BlockSize*(NumBlocks-1),0) + 
        A_off_diag[NumBlocks-2] * x1.template block<BlockSize,1>(BlockSize*(NumBlocks-2),0);

    for (int i = 1; i < FixedSizeSolverType::NumBlocks-1; i++)
    {
        test.template block<BlockSize,1>(BlockSize*i,0) =
            A_off_diag[i-1] * x1.template block<BlockSize,1>(BlockSize*(i-1),0) +
            A_diag[i] * x1.template block<BlockSize,1>(BlockSize*i,0) +
            A_off_diag[i].transpose() * x1.template block<BlockSize,1>(BlockSize*(i+1),0);
    }
    const double norm = (test - b).norm();

    std::cout << "Residual: " << norm << std::endl;

    
    // "Dynamic" solver

    std::vector<FixedSizeSolverType::BlockMatType> A_diag_vec(NumBlocks);
    std::vector<FixedSizeSolverType::BlockMatType> A_off_diag_vec(NumBlocks-1);
    VecXr x2(NumBlocks*6);
    for (int i = 0; i < NumBlocks; i++)
        A_diag_vec[i] = A_diag[i];
    for (int i = 0; i < NumBlocks-1; i++)
        A_off_diag_vec[i] = A_off_diag[i];

    SolverType solver(NumBlocks);
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        solver.solve(A_diag_vec, A_off_diag_vec, b, x2);
    auto t4 = std::chrono::high_resolution_clock::now();
    const double ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() / 1.0e6;
    std::cout << "[Non-fixed size] Elapsed solve time (" <<  NumIters << " iterations): " << ms2 << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms2 / NumIters << " ms" << std::endl;

    std::cout << "x1 and x2 residual: " << (x2 - x1).norm() << std::endl;
}