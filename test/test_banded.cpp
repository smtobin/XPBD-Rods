#include <Eigen/Dense>

#include "solver/BlockBandedSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 1;
constexpr static int BlockSize = 2;
constexpr static int NumBlocks = 6;
constexpr static int Bandwidth = 2;
using BandedSolverType = Solver::SymmetricBlockBandedSolver<BlockSize>;

BandedSolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename BandedSolverType::BlockMatType mat = BandedSolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += BandedSolverType::BlockSize * BandedSolverType::BlockMatType::Identity();

    return mat;
}

MatXr fullMatFromDiagonals(std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals)
{
    MatXr mat(NumBlocks*BlockSize, NumBlocks*BlockSize);
    for (unsigned i = 0; i < diagonals.size(); i++)
    {
        for (int row = 0; row < NumBlocks-static_cast<int>(i); row++)
        {
            if (i == 0) // main diagonal
            {
                mat.block<BlockSize,BlockSize>(row*BlockSize, row*BlockSize) = diagonals[i][row];
            }
            else // off diagonal
            {
                mat.block<BlockSize,BlockSize>( (row+i)*BlockSize, row*BlockSize) = diagonals[i][row];
                mat.block<BlockSize,BlockSize>( row*BlockSize, (row+i)*BlockSize) = diagonals[i][row].transpose();
            }
        }
    }

    return mat;
}

int main() 
{
    std::vector<BandedSolverType::BlockMatType> A_diag_vec(NumBlocks);
    std::vector<BandedSolverType::BlockMatType> A_off_diag_vec(NumBlocks-1);
    VecXr b = VecXr::Random(NumBlocks*BlockSize);

    std::cout << "=== BANDED SOLVER ===" << std::endl;
    std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals(Bandwidth+1);
    for (int b = 0; b < Bandwidth+1; b++)
    {
        diagonals[b].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
        for (int row = 0; row < NumBlocks-b; row++)
        {
            if (b == 0) // main diagonal
            {
                diagonals[b][row] = generateRandomSPDBlockMat();
            }
            else // if (b == 1) // off diagonal
            {
                diagonals[b][row] = BandedSolverType::BlockMatType::Random();
            }
        }
    }

    VecXr x1(NumBlocks*BlockSize);
    BandedSolverType banded_solver(Bandwidth, NumBlocks);
    banded_solver.solve(diagonals, b, x1);

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        banded_solver.solve(diagonals, b, x1);
    auto t2 = std::chrono::high_resolution_clock::now();
    const double ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms1 << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms1 / NumIters << " ms" << std::endl;

    std::cout << "=== FULL MATRIX SOLVER ===" << std::endl;
    MatXr full_mat = fullMatFromDiagonals(diagonals);


    VecXr x2(NumBlocks*BlockSize);
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        x2 = full_mat.ldlt().solve(b);
    auto t4 = std::chrono::high_resolution_clock::now();
    const double ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms2 << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms2 / NumIters << " ms" << std::endl;

    std::cout << "Full mat:\n" << full_mat << std::endl;
    std::cout << "x:\n" << x2 << std::endl;

    std::cout << "Residual: " << (x1 - x2).norm() << std::endl;
}